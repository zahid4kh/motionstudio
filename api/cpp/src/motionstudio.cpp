#include "motionstudio/motionstudio.hpp"
#include "json.hpp"

#include <atomic>
#include <condition_variable>
#include <cstring>
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
    if (code == "simulation" || code == "not_running") throw SimulationError(code, msg);
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

} // namespace motionstudio
