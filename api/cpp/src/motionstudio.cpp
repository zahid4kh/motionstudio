#include "motionstudio/motionstudio.hpp"
#include "json.hpp"

#include <atomic>
#include <condition_variable>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <mutex>
#include <ostream>
#include <sstream>
#include <thread>

#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  pragma comment(lib, "ws2_32.lib")
using socket_t = SOCKET;
constexpr socket_t kNoSocket = INVALID_SOCKET;
inline void closeSocket(socket_t s) { closesocket(s); }
#else
#  include <netdb.h>
#  include <netinet/in.h>
#  include <netinet/tcp.h>
#  include <sys/socket.h>
#  include <unistd.h>
using socket_t = int;
constexpr socket_t kNoSocket = -1;
inline void closeSocket(socket_t s) { ::close(s); }
#endif

namespace motionstudio {

using detail::Json;

namespace {

[[noreturn]] void throwError(const Json& err) {
    const std::string code = err["code"].isString() ? err["code"].str() : "error";
    const std::string msg  = err["message"].isString() ? err["message"].str() : code;
    if (code == "not_found" || code == "ambiguous") throw NotFound(code, msg);
    if (code == "out_of_reach")                      throw OutOfReach(code, msg);
    if (code == "out_of_limits")                     throw OutOfLimits(code, msg);
    if (code == "cancelled" || code == "reset")      throw MotionCancelled(code, msg);
    if (code == "timeout")                           throw Timeout(code, msg);
    if (code == "simulation" || code == "not_running" || code == "simulation_active") throw SimulationError(code, msg);
    if (code == "unsaved")                           throw UnsavedChanges(code, msg);
    throw Error(code, msg);
}

Json targetJson(const Target& t) {
    Json o = Json::Object{};
    if (t.x_)  o.set("x", *t.x_);
    if (t.y_)  o.set("y", *t.y_);
    if (t.z_)  o.set("z", *t.z_);
    if (t.rx_) o.set("rx", *t.rx_);
    if (t.ry_) o.set("ry", *t.ry_);
    if (t.rz_) o.set("rz", *t.rz_);
    return o;
}

Pose poseFrom(const Json& j) {
    return {j["x"].number(), j["y"].number(), j["z"].number(), j["rx"].number(), j["ry"].number(), j["rz"].number()};
}

std::vector<std::string> strings(const Json& a) {
    std::vector<std::string> out;
    for (const Json& j : a.array()) out.push_back(j.isString() ? j.str() : j["name"].str());
    return out;
}

} // namespace

// ── Connection ──────────────────────────────────────────────────────────────

namespace detail {

// One TCP connection: requests are written under a lock, a reader thread
// hands each reply to the call waiting for its id
class Connection {
public:
    Connection(const std::string& host, int port) {
#ifdef _WIN32
        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) throw ConnectionError("connection", "can't start Winsock");
#endif
        addrinfo hints{};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        addrinfo* res = nullptr;
        const std::string portText = std::to_string(port);
        if (getaddrinfo(host.c_str(), portText.c_str(), &hints, &res) != 0)
            throw ConnectionError("connection", "can't find the host " + host);
        for (addrinfo* a = res; a && sock_ == kNoSocket; a = a->ai_next) {
            socket_t s = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
            if (s == kNoSocket) continue;
            if (::connect(s, a->ai_addr, int(a->ai_addrlen)) == 0) sock_ = s;
            else closeSocket(s);
        }
        freeaddrinfo(res);
        if (sock_ == kNoSocket)
            throw ConnectionError("connection", "can't reach MotionStudio at " + host + ":" + portText +
                                  ". Is the app running with Tools > Robot API switched on, or motionstudio-server started?");
        int one = 1;
        setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof one);
        reader_ = std::thread([this] { read(); });
    }

    ~Connection() {
        close();
        if (reader_.joinable()) reader_.join();
        if (sock_ != kNoSocket) closeSocket(sock_);
#ifdef _WIN32
        WSACleanup();
#endif
    }

    Json call(const std::string& method, Json params = Json::Object{}) {
        auto slot = std::make_shared<Slot>();
        int id;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!closed_.empty()) throw ConnectionError("connection", closed_);
            id = nextId_++;
            waiting_[id] = slot;
        }
        Json req = Json::Object{};
        req.set("id", id).set("method", method).set("params", std::move(params));
        const std::string line = req.dump() + "\n";
        {
            std::lock_guard<std::mutex> lock(sendMutex_);
            size_t sent = 0;
            while (sent < line.size()) {
                const auto n = ::send(sock_, line.data() + sent, int(line.size() - sent), 0);
                if (n <= 0) {
                    std::lock_guard<std::mutex> l(mutex_);
                    waiting_.erase(id);
                    throw ConnectionError("connection", "can't send to MotionStudio");
                }
                sent += size_t(n);
            }
        }
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&] { return slot->ready; });
        if (slot->reply.isNull()) throw ConnectionError("connection", closed_.empty() ? "the connection was closed" : closed_);
        if (slot->reply.has("error")) {
            const Json err = slot->reply["error"];
            lock.unlock();
            throwError(err);
        }
        return slot->reply["result"];
    }

    void close() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (closed_.empty()) closed_ = "the connection was closed";
        }
        if (sock_ != kNoSocket) {
#ifdef _WIN32
            shutdown(sock_, SD_BOTH);
#else
            shutdown(sock_, SHUT_RDWR);
#endif
        }
    }

private:
    struct Slot { bool ready = false; Json reply; };

    void read() {
        std::string buf;
        char chunk[65536];
        while (true) {
            const auto n = ::recv(sock_, chunk, sizeof chunk, 0);
            if (n <= 0) break;
            buf.append(chunk, size_t(n));
            size_t nl;
            while ((nl = buf.find('\n')) != std::string::npos) {
                const std::string line = buf.substr(0, nl);
                buf.erase(0, nl + 1);
                if (line.find_first_not_of(" \r\t") == std::string::npos) continue;
                Json msg;
                try { msg = Json::parse(line); } catch (const std::exception&) { continue; }
                std::lock_guard<std::mutex> lock(mutex_);
                auto it = waiting_.find(int(msg["id"].number(-1)));
                if (it == waiting_.end()) continue;
                it->second->reply = std::move(msg);
                it->second->ready = true;
                waiting_.erase(it);
                cv_.notify_all();
            }
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_.empty()) closed_ = "the connection to MotionStudio was closed";
        for (auto& [id, slot] : waiting_) slot->ready = true;
        waiting_.clear();
        cv_.notify_all();
    }

    socket_t sock_ = kNoSocket;
    std::thread reader_;
    std::mutex mutex_, sendMutex_;
    std::condition_variable cv_;
    std::map<int, std::shared_ptr<Slot>> waiting_;
    int nextId_ = 1;
    std::string closed_;
};

} // namespace detail

// ── Values ──────────────────────────────────────────────────────────────────

std::ostream& operator<<(std::ostream& os, const Place& p) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(1) << "Place(x=" << p.x << ", z=" << p.z << ", heading=" << p.heading << ")";
    return os << s.str();
}

std::ostream& operator<<(std::ostream& os, const Pose& p) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(2) << "Pose(x=" << p.x << ", y=" << p.y << ", z=" << p.z
      << ", rx=" << p.rx << ", ry=" << p.ry << ", rz=" << p.rz << ")";
    return os << s.str();
}

// ── Motion ──────────────────────────────────────────────────────────────────

void Motion::wait(double timeout) const {
    Json p = Json::Object{};
    p.set("motion", id_);
    if (timeout >= 0) p.set("timeout", timeout);
    conn_->call("motion.wait", p);
}

void Motion::cancel() const { conn_->call("motion.cancel", Json(Json::Object{{"motion", id_}})); }

std::string Motion::state() const { return conn_->call("motion.status", Json(Json::Object{{"motion", id_}}))["state"].str(); }

bool Motion::done() const {
    const std::string s = state();
    return s == "done" || s == "failed" || s == "cancelled";
}

double Motion::progress() const {
    return conn_->call("motion.status", Json(Json::Object{{"motion", id_}}))["progress"].number();
}

// ── Tool ────────────────────────────────────────────────────────────────────

Pose Tool::pose(const std::string& frame) const {
    Json p = Json::Object{};
    p.set("robot", robot_).set("tool", name_).set("frame", frame);
    if (!base_.empty()) p.set("base", base_);
    return poseFrom(conn_->call("tool.pose", p));
}

Motion Tool::move(const char* method, const Target& t, const MoveOptions& o, bool relative) const {
    Json p = Json::Object{};
    p.set("robot", robot_).set("tool", name_).set("target", targetJson(t)).set("frame", o.frame)
     .set("relative", relative).set("wait", o.wait);
    if (o.speed > 0) p.set("speed", o.speed);
    if (!base_.empty()) p.set("base", base_);
    return Motion(conn_, int(conn_->call(method, p)["motion"].number()));
}

Motion Tool::moveL(const Target& t, const MoveOptions& o) const { return move("tool.moveL", t, o, false); }
Motion Tool::moveJ(const Target& t, const MoveOptions& o) const { return move("tool.moveJ", t, o, false); }
Motion Tool::moveBy(const Target& t, const MoveOptions& o) const { return move("tool.moveL", t, o, true); }
Motion Tool::moveJBy(const Target& t, const MoveOptions& o) const { return move("tool.moveJ", t, o, true); }

// ── Robot ───────────────────────────────────────────────────────────────────

std::map<std::string, double> Robot::joints() const {
    std::map<std::string, double> out;
    const Json r = conn_->call("robot.joints", Json(Json::Object{{"robot", path_}}));
    for (const auto& [k, v] : r.object()) out[k] = v.number();
    return out;
}

std::vector<JointInfo> Robot::jointInfo() const {
    std::vector<JointInfo> out;
    const Json info = conn_->call("robot.info", Json(Json::Object{{"robot", path_}}));
    for (const Json& j : info["joints"].array()) {
        JointInfo i;
        i.name = j["name"].str();
        i.type = j["type"].str();
        i.unit = j["unit"].str();
        i.value = j["value"].number();
        if (j.has("min")) i.min = j["min"].number();
        if (j.has("max")) i.max = j["max"].number();
        i.speed = j["speed"].number();
        i.acceleration = j["acceleration"].number();
        out.push_back(i);
    }
    return out;
}

std::vector<std::string> Robot::poses() const {
    return strings(conn_->call("robot.info", Json(Json::Object{{"robot", path_}}))["poses"]);
}

Motion Robot::jointMove(const std::map<std::string, double>& values, const MoveOptions& o, bool relative) const {
    Json joints = Json::Object{};
    for (const auto& [k, v] : values) joints.set(k, v);
    Json p = Json::Object{};
    p.set("robot", path_).set("joints", joints).set("relative", relative).set("wait", o.wait);
    if (o.speed > 0) p.set("speed", o.speed);
    return Motion(conn_, int(conn_->call("robot.moveJoints", p)["motion"].number()));
}

Motion Robot::moveJoints(const std::map<std::string, double>& v, const MoveOptions& o) const { return jointMove(v, o, false); }
Motion Robot::moveJointsBy(const std::map<std::string, double>& v, const MoveOptions& o) const { return jointMove(v, o, true); }

Motion Robot::moveToPose(const std::string& pose, const MoveOptions& o) const {
    Json p = Json::Object{};
    p.set("robot", path_).set("pose", pose).set("wait", o.wait);
    if (o.speed > 0) p.set("speed", o.speed);
    return Motion(conn_, int(conn_->call("robot.moveToPose", p)["motion"].number()));
}

Motion Robot::home(const MoveOptions& o) const {
    Json p = Json::Object{};
    p.set("robot", path_).set("wait", o.wait);
    if (o.speed > 0) p.set("speed", o.speed);
    return Motion(conn_, int(conn_->call("robot.home", p)["motion"].number()));
}

int Robot::stop() const { return int(conn_->call("robot.stop", Json(Json::Object{{"robot", path_}}))["stopped"].number()); }

Tool Robot::tool(const std::string& name, const std::string& base) const {
    std::string n = name;
    if (n.empty()) {
        if (tools_.size() == 1) n = tools_.front();
        else if (tools_.empty()) throw NotFound("not_found", name_ + " has no tool; give the name of an object to use");
        else {
            std::string list;
            for (const auto& t : tools_) list += (list.empty() ? "" : ", ") + t;
            throw NotFound("ambiguous", name_ + " has " + std::to_string(tools_.size()) + " tools: pick one of " + list);
        }
    }
    return Tool(conn_, path_, n, base);
}

// ── Humanoid ────────────────────────────────────────────────────────────────

namespace {
const std::pair<const char*, std::optional<double> Gait::*> kGaitKeys[] = {
    {"stepLength", &Gait::stepLength}, {"sideStep", &Gait::sideStep}, {"turnPerStep", &Gait::turnPerStep},
    {"stepHeight", &Gait::stepHeight}, {"stepTime", &Gait::stepTime}, {"speed", &Gait::speed},
    {"crouch", &Gait::crouch},         {"sway", &Gait::sway},         {"armSwing", &Gait::armSwing},
};

Json walkParams(const std::string& robot, const WalkOptions& o) {
    Json p = Json::Object{};
    p.set("robot", robot).set("wait", o.wait);
    for (const auto& [key, field] : kGaitKeys)
        if (o.gait.*field) p.set(key, *(o.gait.*field));
    return p;
}
} // namespace

Motion Humanoid::walk(double forward, double sideways, double turn, const WalkOptions& o) const {
    Json p = walkParams(path_, o);
    p.set("forward", forward).set("sideways", sideways).set("turn", turn);
    return Motion(conn_, int(conn_->call("humanoid.walk", p)["motion"].number()));
}

Motion Humanoid::turn(double degrees, const WalkOptions& o) const { return walk(0, 0, degrees, o); }
Motion Humanoid::sideStep(double mm, const WalkOptions& o) const { return walk(0, mm, 0, o); }

Motion Humanoid::walkTo(double x, double z, std::optional<double> heading, const WalkOptions& o) const {
    return walkPath({{x, z}}, heading, o);
}

Motion Humanoid::walkPath(const std::vector<Point>& points, std::optional<double> heading, const WalkOptions& o) const {
    Json p = walkParams(path_, o);
    Json::Array pts;
    for (const Point& q : points) pts.push_back(Json(Json::Array{Json(q.x), Json(q.z)}));
    p.set("points", Json(std::move(pts))).set("frame", o.frame);
    if (heading) p.set("heading", *heading);
    return Motion(conn_, int(conn_->call("humanoid.walkPath", p)["motion"].number()));
}

int Humanoid::stopWalking() const {
    return int(conn_->call("humanoid.stop", Json(Json::Object{{"robot", path_}}))["stopping"].number());
}

WalkState Humanoid::state() const {
    const Json s = conn_->call("humanoid.state", Json(Json::Object{{"robot", path_}}));
    WalkState w;
    w.place = {s["x"].number(), s["z"].number(), s["heading"].number()};
    w.walking = s["walking"].boolean();
    w.motion = int(s["motion"].number(0));
    w.step = int(s["step"].number(0));
    w.steps = int(s["steps"].number(0));
    const Json g = s["gait"];
    for (const auto& [key, field] : kGaitKeys)
        if (g.has(key)) w.gait.*field = g[key].number();
    return w;
}

// ── Actuator ────────────────────────────────────────────────────────────────

double Actuator::value() const {
    return conn_->call("actuator.get", Json(Json::Object{{"name", name_}}))["value"].number();
}

Motion Actuator::moveTo(double value, const MoveOptions& o) const {
    Json p = Json::Object{};
    p.set("joints", Json(Json::Object{{name_, value}})).set("wait", o.wait);
    if (o.speed > 0) p.set("speed", o.speed);
    return Motion(conn_, int(conn_->call("robot.moveJoints", p)["motion"].number()));
}

Motion Actuator::moveBy(double amount, const MoveOptions& o) const {
    Json p = Json::Object{};
    p.set("joints", Json(Json::Object{{name_, amount}})).set("relative", true).set("wait", o.wait);
    if (o.speed > 0) p.set("speed", o.speed);
    return Motion(conn_, int(conn_->call("robot.moveJoints", p)["motion"].number()));
}

void Actuator::run(double speed) const { conn_->call("actuator.run", Json(Json::Object{{"name", name_}, {"speed", speed}})); }
void Actuator::stop() const { run(0); }

// ── Studio ──────────────────────────────────────────────────────────────────

Studio Studio::connect(const std::string& host, int port) {
    Studio s(std::make_shared<detail::Connection>(host, port));
    const Json hello = s.conn_->call("hello", Json(Json::Object{{"client", "c++"}}));
    s.version_ = hello["version"].str();
    if (int(hello["protocol"].number()) != kProtocol)
        throw Error("protocol", "MotionStudio speaks protocol " + std::to_string(int(hello["protocol"].number())) +
                                ", this library " + std::to_string(kProtocol) + ": update one of them");
    return s;
}

std::vector<Robot> Studio::robots() const {
    std::vector<Robot> out;
    const Json all = conn_->call("scene.robots");
    for (const Json& j : all.array()) {
        Robot r(conn_);
        r.name_ = j["name"].str();
        r.path_ = j["path"].str();
        r.joints_ = strings(j["joints"]);
        r.tools_ = strings(j["tools"]);
        r.walks_ = j["walks"].boolean();
        out.push_back(std::move(r));
    }
    return out;
}

Robot Studio::robot(const std::string& name) const {
    if (name.empty()) {
        auto all = robots();
        if (all.size() == 1) return all.front();
        if (all.empty()) throw NotFound("not_found", "the scene has no robot");
        std::string list;
        for (const auto& r : all) list += (list.empty() ? "" : ", ") + r.path();
        throw NotFound("ambiguous", "the scene has several robots: pick one of " + list);
    }
    const Json info = conn_->call("robot.info", Json(Json::Object{{"robot", name}}));
    Robot r(conn_);
    r.name_ = info["name"].str();
    r.path_ = info["path"].str();
    r.joints_ = strings(info["joints"]);
    r.tools_ = strings(info["tools"]);
    r.walks_ = info["walks"].boolean();
    return r;
}

Humanoid Studio::humanoid(const std::string& name) const {
    std::string n = name;
    if (n.empty()) {
        std::vector<Robot> walkers;
        for (const Robot& r : robots()) if (r.walks()) walkers.push_back(r);
        if (walkers.empty()) throw NotFound("not_found", "the scene has no robot with legs");
        if (walkers.size() > 1) {
            std::string list;
            for (const auto& r : walkers) list += (list.empty() ? "" : ", ") + r.path();
            throw NotFound("ambiguous", "the scene has several robots with legs: pick one of " + list);
        }
        n = walkers.front().path();
    }
    const Robot r = robot(n);
    if (!r.walks()) throw Error("bad_request", r.name() + " has no legs to walk on");
    return Humanoid(r);
}

Actuator Studio::actuator(const std::string& name) const { return Actuator(conn_, name); }

double Studio::get(const std::string& signal) const {
    return conn_->call("signal.get", Json(Json::Object{{"name", signal}}))["value"].number();
}

void Studio::set(const std::string& signal, double value) const {
    conn_->call("signal.set", Json(Json::Object{{"name", signal}, {"value", value}}));
}

std::vector<SignalInfo> Studio::signals() const {
    std::vector<SignalInfo> out;
    const Json all = conn_->call("signal.list");
    for (const Json& j : all.array())
        out.push_back({j["name"].str(), j["kind"].str(), j["type"].str(), j["writable"].boolean(), j["value"].number()});
    return out;
}

double Studio::waitUntil(const std::string& condition, double timeout) const {
    Json p = Json::Object{};
    p.set("condition", condition);
    if (timeout >= 0) p.set("timeout", timeout);
    return conn_->call("signal.waitUntil", p)["time"].number();
}

void   Studio::play() const  { conn_->call("sim.play"); }
void   Studio::pause() const { conn_->call("sim.pause"); }
void   Studio::reset() const { conn_->call("sim.reset"); }
double Studio::sleep(double seconds) const {
    return conn_->call("sim.wait", Json(Json::Object{{"seconds", seconds}}))["time"].number();
}
double Studio::time() const    { return conn_->call("sim.state")["time"].number(); }
bool   Studio::running() const { return conn_->call("sim.state")["running"].boolean(); }
void   Studio::setSpeed(double factor) const { conn_->call("sim.speed", Json(Json::Object{{"factor", factor}})); }
void   Studio::close() { if (conn_) conn_->close(); }


// ── Building scenes ─────────────────────────────────────────────────────────

namespace {

Json vecJson(const Vec3& v) { return Json::Array{v.x, v.y, v.z}; }
Vec3 vecFrom(const Json& j) { return {j["x"].number(), j["y"].number(), j["z"].number()}; }
Json valueJson(const Value& v) { return v.isNumber() ? Json(v.number()) : Json(v.expression()); }
Json strings(const std::vector<std::string>& v) {
    Json::Array a;
    for (const auto& s : v) a.push_back(s);
    return a;
}
std::vector<std::string> names(const Json& a) {
    std::vector<std::string> out;
    for (const Json& j : a.array()) out.push_back(j.isString() ? j.str() : j["name"].str());
    return out;
}
std::string absolute(const std::string& path) {
    std::error_code ec;
    const auto p = std::filesystem::absolute(path, ec);
    return ec ? path : p.string();
}
Json objectOptions(const ObjectOptions& o) {
    Json p = Json::Object{};
    if (!o.parent.empty()) p.set("parent", o.parent);
    if (o.position) p.set("position", targetJson(*o.position));
    if (!o.frame.empty()) p.set("frame", o.frame);
    if (o.size) p.set("size", vecJson(*o.size));
    if (o.diameter) p.set("diameter", *o.diameter);
    if (o.height) p.set("height", *o.height);
    if (!o.color.empty()) p.set("color", o.color);
    if (o.opacity) p.set("opacity", *o.opacity);
    if (o.hidden) p.set("hidden", *o.hidden);
    return p;
}
SceneInfo sceneInfo(const Json& j) {
    SceneInfo i;
    i.path = j["path"].str();
    i.objects = int(j["objects"].number());
    i.top = names(j["top"]);
    i.robots = names(j["robots"]);
    i.joints = names(j["joints"]);
    i.signals = names(j["signals"]);
    i.sensors = names(j["sensors"]);
    i.sequences = names(j["sequences"]);
    i.poses = names(j["poses"]);
    i.sources = names(j["sources"]);
    i.sinks = names(j["sinks"]);
    i.motions = names(j["motions"]);
    return i;
}
Json common(Json op, const std::string& name, const Ops::Common& c) {
    op.set("name", name);
    if (c.first) op.set("after", Json::Array{});
    else if (!c.after.empty()) op.set("after", strings(c.after));
    if (!c.onStart.empty()) op.set("onStart", c.onStart);
    if (!c.onFinish.empty()) op.set("onFinish", c.onFinish);
    if (!c.startCondition.empty()) op.set("startCondition", c.startCondition);
    return op;
}
Json op(const char* type) {
    Json o = Json::Object{};
    o.set("type", type);
    return o;
}

} // namespace

// Objects

#define OBJ(extra) Json(Json::Object{{"object", ref()}}) extra

std::string SceneObject::name() const { return conn_->call("object.get", OBJ())["name"].str(); }
std::string SceneObject::type() const { return conn_->call("object.get", OBJ())["type"].str(); }
std::string SceneObject::parentPath() const { return conn_->call("object.get", OBJ())["parent"].str(); }
Pose SceneObject::position() const { return poseFrom(conn_->call("object.get", OBJ())["position"]); }
Pose SceneObject::world() const { return poseFrom(conn_->call("object.get", OBJ())["world"]); }
Pose SceneObject::poseIn(const std::string& frame) const {
    return poseFrom(conn_->call("object.get", OBJ(.set("frame", frame)))["inFrame"]);
}
std::optional<Vec3> SceneObject::size() const {
    const Json j = conn_->call("object.get", OBJ());
    if (!j.has("size")) return std::nullopt;
    return vecFrom(j["size"]);
}
std::vector<SceneObject> SceneObject::children() const {
    std::vector<SceneObject> out;
    for (const Json& j : conn_->call("scene.objects", Json(Json::Object{{"under", ref()}, {"depth", 1}})).array())
        out.push_back(SceneObject(conn_, int(j["id"].number()), j["path"].str()));
    return out;
}
SceneObject& SceneObject::set(const ObjectOptions& o) {
    Json p = objectOptions(o);
    p.set("object", ref());
    path_ = conn_->call("object.set", p)["path"].str();
    return *this;
}
SceneObject& SceneObject::moveTo(const Target& position, const std::string& frame) {
    ObjectOptions o;
    o.position = position;
    o.frame = frame;
    return set(o);
}
SceneObject& SceneObject::rename(const std::string& name) {
    path_ = conn_->call("object.set", OBJ(.set("name", name)))["path"].str();
    return *this;
}
SceneObject& SceneObject::setParent(const std::string& parent) {
    Json p = OBJ();
    p.set("parent", parent.empty() ? Json() : Json(parent));
    path_ = conn_->call("object.set", p)["path"].str();
    return *this;
}
SceneObject SceneObject::add(const std::string& type, const std::string& name, ObjectOptions o) const {
    o.parent = ref();
    return Scene(conn_).add(type, name, std::move(o));
}
int SceneObject::remove() const { return int(conn_->call("object.remove", OBJ())["removed"].number()); }
SceneObject SceneObject::duplicate(const std::string& name) const {
    Json p = OBJ();
    if (!name.empty()) p.set("name", name);
    const Json j = conn_->call("object.duplicate", p);
    return SceneObject(conn_, int(j["id"].number()), j["path"].str());
}
Box SceneObject::bounds(const std::string& frame, bool children) const {
    const Json b = conn_->call("object.bounds", OBJ(.set("frame", frame).set("children", children)));
    return {vecFrom(b["min"]), vecFrom(b["max"]), vecFrom(b["center"]), vecFrom(b["size"])};
}
std::vector<Feature> SceneObject::features(const std::vector<std::string>& kinds, const std::string& frame,
                                           std::optional<double> minRadius, std::optional<double> maxRadius) const {
    Json p = OBJ(.set("kinds", strings(kinds)).set("frame", frame));
    if (minRadius) p.set("minRadius", *minRadius);
    if (maxRadius) p.set("maxRadius", *maxRadius);
    std::vector<Feature> out;
    for (const Json& f : conn_->call("object.features", p)["features"].array()) {
        Feature e{f["object"].str(), f["kind"].str(), vecFrom(f["origin"]), vecFrom(f["axis"]), std::nullopt, std::nullopt};
        if (f.has("radius")) e.radius = f["radius"].number();
        if (f.has("length")) e.length = f["length"].number();
        out.push_back(std::move(e));
    }
    return out;
}
SceneObject& SceneObject::setBody(const BodyOptions& o) {
    Json p = OBJ();
    if (o.mass) p.set("mass", *o.mass);
    if (o.gravity) p.set("gravity", *o.gravity);
    if (o.collision) p.set("collision", *o.collision);
    conn_->call("body.set", p);
    return *this;
}
SceneObject& SceneObject::removeBody() { conn_->call("body.set", OBJ(.set("remove", true))); return *this; }
SceneObject& SceneObject::setBelt(const Value& speed, std::optional<Vec3> direction, const std::string& frame) {
    Json p = OBJ(.set("speed", valueJson(speed)));
    if (direction) p.set("direction", vecJson(*direction));
    if (!frame.empty()) p.set("frame", frame);
    conn_->call("belt.set", p);
    return *this;
}
SceneObject& SceneObject::removeBelt() { conn_->call("belt.set", OBJ(.set("remove", true))); return *this; }

#undef OBJ

// Joints

JointDetails Joint::details() const {
    const Json j = conn_->call("joint.get", Json(Json::Object{{"joint", name_}}));
    JointDetails d;
    d.name = j["name"].str();
    d.type = j["type"].str();
    d.parent = j["parent"].str();
    d.child = j["child"].str();
    d.axis = vecFrom(j["axis"]);
    if (j.has("pivot")) d.pivot = vecFrom(j["pivot"]);
    if (j.has("min")) d.min = j["min"].number();
    if (j.has("max")) d.max = j["max"].number();
    d.value = j["value"].number();
    d.actuator = j["actuator"]["name"].str();
    d.follows = j["follows"]["joint"].str();
    return d;
}
void Joint::setValue(double v) const { conn_->call("joint.set", Json(Json::Object{{"joint", name_}, {"value", v}})); }
void Joint::setRange(double min, double max) const {
    conn_->call("joint.set", Json(Json::Object{{"joint", name_}, {"min", min}, {"max", max}}));
}
void Joint::remove() const { conn_->call("joint.remove", Json(Json::Object{{"joint", name_}})); }

// Sequences

Ops& Ops::push(std::string json) { ops_.push_back(std::move(json)); return *this; }
Ops& Ops::raw(const std::string& json) { return push(json); }
Ops& Ops::move(const std::string& name, const std::string& actuator, const Value& target, std::optional<Value> speed,
               const Common& c) {
    Json o = op("move");
    o.set("actuator", actuator).set("target", valueJson(target));
    if (speed) o.set("speed", valueJson(*speed));
    return push(common(o, name, c).dump());
}
Ops& Ops::moveToPose(const std::string& name, const std::string& pose, std::optional<Value> speed, const Common& c) {
    Json o = op("moveToPose");
    o.set("pose", pose);
    if (speed) o.set("speed", valueJson(*speed));
    return push(common(o, name, c).dump());
}
Ops& Ops::run(const std::string& name, const std::string& actuator, const Value& speed, const Common& c) {
    Json o = op("run");
    o.set("actuator", actuator).set("speed", valueJson(speed));
    return push(common(o, name, c).dump());
}
Ops& Ops::stop(const std::string& name, const std::string& actuator, const Common& c) {
    Json o = op("stop");
    o.set("actuator", actuator);
    return push(common(o, name, c).dump());
}
Ops& Ops::set(const std::string& name, const std::string& actions, const Common& c) {
    Common k = c;
    k.onStart = actions;
    return push(common(op("set"), name, k).dump());
}
Ops& Ops::wait(const std::string& name, const Value& seconds, const Common& c) {
    Json o = op("wait");
    o.set("seconds", valueJson(seconds));
    return push(common(o, name, c).dump());
}
Ops& Ops::waitFor(const std::string& name, const std::string& condition, const Common& c) {
    Json o = op("waitFor");
    o.set("condition", condition);
    return push(common(o, name, c).dump());
}
Ops& Ops::grip(const std::string& name, const std::string& gripper, const std::string& what, bool bySensor,
               const Common& c) {
    Json o = op("grip");
    o.set("gripper", gripper).set(bySensor ? "sensor" : "part", what);
    return push(common(o, name, c).dump());
}
Ops& Ops::release(const std::string& name, const std::string& gripper, const Common& c) {
    Json o = op("release");
    o.set("gripper", gripper);
    return push(common(o, name, c).dump());
}
Ops& Ops::moveL(const std::string& name, const std::string& tool, const std::string& pose, std::optional<Value> speed,
                const Common& c) {
    Json o = op("moveL");
    o.set("tool", tool).set("pose", pose);
    if (speed) o.set("speed", valueJson(*speed));
    return push(common(o, name, c).dump());
}

// Scene

void Scene::newScene(bool discard) const { conn_->call("scene.new", Json(Json::Object{{"discard", discard}})); }
SceneInfo Scene::open(const std::string& path, bool discard) const {
    return sceneInfo(conn_->call("scene.open", Json(Json::Object{{"path", absolute(path)}, {"discard", discard}})));
}
std::string Scene::save(const std::string& path) const {
    Json p = Json::Object{};
    if (!path.empty()) p.set("path", absolute(path));
    return conn_->call("scene.save", p)["path"].str();
}
SceneInfo Scene::info() const { return sceneInfo(conn_->call("scene.info")); }
std::vector<std::string> Scene::check() const { return names(conn_->call("scene.check")["problems"]); }
SceneObject Scene::importFile(const std::string& path, const ImportOptions& o) const {
    Json p = Json::Object{};
    p.set("path", absolute(path));
    if (!o.parent.empty()) p.set("parent", o.parent);
    if (!o.name.empty()) p.set("name", o.name);
    if (!o.upAxis.empty()) p.set("upAxis", o.upAxis);
    const Json root = conn_->call("scene.import", p)["root"];
    return SceneObject(conn_, int(root["id"].number()), root["path"].str());
}
std::vector<SceneObject> Scene::objects(const std::string& pattern, const std::string& under, const std::string& type,
                                        int depth) const {
    Json p = Json::Object{};
    if (!pattern.empty()) p.set("name", pattern);
    if (!under.empty()) p.set("under", under);
    if (!type.empty()) p.set("type", type);
    if (depth >= 0) p.set("depth", depth);
    std::vector<SceneObject> out;
    for (const Json& j : conn_->call("scene.objects", p).array())
        out.push_back(SceneObject(conn_, int(j["id"].number()), j["path"].str()));
    return out;
}
SceneObject Scene::object(const std::string& name) const {
    const Json j = conn_->call("object.get", Json(Json::Object{{"object", name}}));
    return SceneObject(conn_, int(j["id"].number()), j["path"].str());
}
bool Scene::exists(const std::string& name) const {
    try { object(name); return true; } catch (const NotFound&) { return false; }
}
SceneObject Scene::add(const std::string& type, const std::string& name, ObjectOptions o) const {
    Json p = objectOptions(o);
    p.set("type", type);
    if (!name.empty()) p.set("name", name);
    const Json j = conn_->call("object.add", p);
    return SceneObject(conn_, int(j["id"].number()), j["path"].str());
}
SceneObject Scene::addBox(const std::string& name, Vec3 size, ObjectOptions o) const {
    o.size = size;
    return add("box", name, std::move(o));
}
SceneObject Scene::addCylinder(const std::string& name, double diameter, double height, ObjectOptions o) const {
    o.diameter = diameter;
    o.height = height;
    return add("cylinder", name, std::move(o));
}
SceneObject Scene::group(const std::vector<std::string>& objects, const std::string& name) const {
    const Json j = conn_->call("object.group", Json(Json::Object{{"objects", strings(objects)}, {"name", name}}));
    return SceneObject(conn_, int(j["id"].number()), j["path"].str());
}
Joint Scene::addJoint(const std::string& name, const std::string& parent, const std::string& child, Vec3 axis,
                      const JointOptions& o) const {
    Json p = Json::Object{};
    if (!name.empty()) p.set("name", name);
    p.set("parent", parent).set("child", child).set("axis", vecJson(axis)).set("type", o.type);
    if (o.pivot) p.set("pivot", vecJson(*o.pivot));
    if (!o.frame.empty()) p.set("frame", o.frame);
    if (o.min) p.set("min", *o.min);
    if (o.max) p.set("max", *o.max);
    if (o.continuous) p.set("continuous", *o.continuous);
    if (o.value) p.set("value", *o.value);
    if (!o.follows.empty())
        p.set("follows", Json(Json::Object{{"joint", o.follows}, {"ratio", o.ratio}, {"offset", o.offset}}));
    if (o.noActuator) p.set("actuator", false);
    else if (o.actuator) {
        Json a = Json::Object{};
        if (!o.actuator->name.empty()) a.set("name", o.actuator->name);
        if (o.actuator->speed) a.set("speed", *o.actuator->speed);
        if (o.actuator->acceleration) a.set("acceleration", *o.actuator->acceleration);
        if (!o.actuator->mode.empty()) a.set("mode", o.actuator->mode);
        if (!o.actuator->target.empty()) a.set("target", o.actuator->target);
        p.set("actuator", a);
    }
    return Joint(conn_, conn_->call("joint.add", p)["name"].str());
}
Joint Scene::joint(const std::string& name) const {
    return Joint(conn_, conn_->call("joint.get", Json(Json::Object{{"joint", name}}))["name"].str());
}
void Scene::addSignal(const std::string& name, const SignalOptions& o) const {
    Json p = Json::Object{};
    p.set("name", name).set("kind", o.kind).set("type", o.type).set("value", o.value);
    if (!o.expression.empty()) p.set("expression", o.expression);
    if (!o.description.empty()) p.set("description", o.description);
    conn_->call("signal.add", p);
}
void Scene::addSensor(const std::string& name, const std::string& type, const SensorOptions& o) const {
    Json p = Json::Object{};
    p.set("name", name).set("type", type).set("detects", o.detects);
    if (!o.mount.empty()) p.set("mount", o.mount);
    if (o.at) p.set("at", vecJson(*o.at));
    if (o.direction) p.set("direction", vecJson(*o.direction));
    if (o.size) p.set("size", vecJson(*o.size));
    if (o.range) p.set("range", *o.range);
    if (o.bounds) p.set("bounds", true);
    if (!o.frame.empty()) p.set("frame", o.frame);
    if (!o.description.empty()) p.set("description", o.description);
    conn_->call("sensor.add", p);
}
void Scene::addPose(const std::string& name, const std::map<std::string, double>& joints, bool replace) const {
    Json j = Json::Object{};
    for (const auto& [k, v] : joints) j.set(k, v);
    conn_->call("pose.add", Json(Json::Object{{"name", name}, {"joints", j}, {"replace", replace}}));
}
void Scene::addSequence(const std::string& name, const Ops& ops, const SequenceOptions& o) const {
    Json::Array list;
    for (const std::string& s : ops.ops_) list.push_back(Json::parse(s));
    Json p = Json::Object{};
    p.set("name", name).set("operations", list).set("loop", o.loop).set("replace", o.replace);
    if (!o.enable.empty()) p.set("enable", o.enable);
    if (!o.start.empty()) p.set("start", o.start);
    conn_->call("sequence.add", p);
}
std::string Scene::sequenceJson(const std::string& name) const {
    return conn_->call("sequence.get", Json(Json::Object{{"name", name}})).dump();
}
void Scene::addSource(const std::string& name, const std::string& templateObject, const SourceOptions& o) const {
    Json p = Json::Object{};
    p.set("name", name).set("template", templateObject).set("max", o.max).set("whenClear", o.whenClear);
    if (o.interval) p.set("interval", valueJson(*o.interval));
    if (!o.trigger.empty()) p.set("trigger", o.trigger).set("mode", "trigger");
    if (!o.enable.empty()) p.set("enable", o.enable);
    conn_->call("source.add", p);
}
void Scene::addSink(const std::string& name, const SinkOptions& o) const {
    Json p = Json::Object{};
    p.set("name", name);
    if (!o.mount.empty()) p.set("mount", o.mount);
    if (o.at) p.set("at", vecJson(*o.at));
    if (o.size) p.set("size", vecJson(*o.size));
    if (o.bounds) p.set("bounds", true);
    if (!o.frame.empty()) p.set("frame", o.frame);
    if (!o.source.empty()) p.set("source", o.source);
    if (!o.enable.empty()) p.set("enable", o.enable);
    conn_->call("sink.add", p);
}
std::string Scene::remove(const std::string& name) const {
    return conn_->call("automation.remove", Json(Json::Object{{"name", name}}))["removed"].str();
}

} // namespace motionstudio
