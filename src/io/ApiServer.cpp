#include "ApiServer.h"
#include "../automation/Kinematics.h"
#include "../physics/SimulationEngine.h"
#include "../scene/SceneGraph.h"
#include <QCoreApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTcpServer>
#include <QTcpSocket>
#include <algorithm>
#include <cmath>
#include <functional>
#include <set>

namespace {

constexpr double kMm = 1000.0;                 // API lengths are millimetres
constexpr int    kMaxLine = 1 << 20;           // a request longer than this closes the connection

struct ApiError {
    QString code, message;
};
[[noreturn]] void fail(const QString& code, const QString& message) { throw ApiError{code, message}; }

using MS = AutomationEngine::MotionState;

// Joint values: degrees, or millimetres for prismatic joints
double round6(double v) { return std::round(v * 1e6) / 1e6; }
double toApi(const Joint& j, double v)   { return round6(j.type == JointType::Prismatic ? v * kMm : v); }
double fromApi(const Joint& j, double v) { return j.type == JointType::Prismatic ? v / kMm : v; }

bool isTcp(const QString& name) { return name.contains("TCP", Qt::CaseInsensitive); }

bool inSubtree(const SceneGraph& g, int root, int id) {
    for (const SceneObject* o = g.getObject(id); o; o = g.getObject(o->parentId))
        if (o->id == root) return true;
    return false;
}

void collect(const SceneGraph& g, int id, std::vector<int>& out) {
    const SceneObject* o = g.getObject(id);
    if (!o) return;
    out.push_back(id);
    for (int c : o->childIds) collect(g, c, out);
}

// A machine's tools: frames named …TCP… that its own joints move, else (with
// two joints or more) the deepest object at the end of its longest chain, or
// where the longest chains split (a wrist carrying two fingers)
std::vector<int> toolsOf(const SceneGraph& g, int root, const std::vector<int>& joints) {
    const std::set<int> mine(joints.begin(), joints.end());
    auto ownChain = [&](int id) {
        std::vector<int> c;
        for (int j : kin::chainTo(g, id)) if (mine.count(j)) c.push_back(j);
        return c;
    };
    std::vector<int> sub, tools;
    collect(g, root, sub);
    for (int id : sub)
        if (isTcp(g.getObject(id)->name) && !ownChain(id).empty()) tools.push_back(id);
    if (!tools.empty() || joints.size() < 2) return tools;
    size_t longest = 0;
    for (int id : sub) longest = std::max(longest, ownChain(id).size());
    if (longest == 0) return tools;
    int best = -1, bestDepth = -1;
    std::vector<int> common;
    bool first = true;
    for (int id : sub) {
        const std::vector<int> c = ownChain(id);
        if (c.size() != longest) continue;
        if (first) { common = c; first = false; }
        else {
            size_t k = 0;
            while (k < common.size() && k < c.size() && common[k] == c[k]) ++k;
            common.resize(k);
        }
        int depth = 0;
        for (const SceneObject* o = g.getObject(id); o && o->id != root; o = g.getObject(o->parentId)) ++depth;
        if (depth > bestDepth) { best = id; bestDepth = depth; }
    }
    if (common.size() < longest && !common.empty()) best = g.getJoint(common.back())->childBodyId;
    if (best != -1) tools.push_back(best);
    return tools;
}

double number(const QJsonObject& p, const QString& key, double fallback) {
    const QJsonValue v = p.value(key);
    if (v.isUndefined() || v.isNull()) return fallback;
    if (v.isBool()) return v.toBool() ? 1 : 0;
    if (!v.isDouble()) fail("bad_request", QString("\"%1\" must be a number").arg(key));
    return v.toDouble();
}
QString text(const QJsonObject& p, const QString& key, bool required = true) {
    const QJsonValue v = p.value(key);
    if ((v.isUndefined() || v.isNull()) && !required) return {};
    if (!v.isString() || v.toString().trimmed().isEmpty())
        fail("bad_request", QString("\"%1\" must be a name").arg(key));
    return v.toString().trimmed();
}
bool flag(const QJsonObject& p, const QString& key, bool fallback) {
    const QJsonValue v = p.value(key);
    if (v.isUndefined() || v.isNull()) return fallback;
    if (!v.isBool()) fail("bad_request", QString("\"%1\" must be true or false").arg(key));
    return v.toBool();
}

QString listOf(QStringList names, int max = 12) {
    names.sort(Qt::CaseInsensitive);
    if (names.size() > max) { names = names.mid(0, max); names << "…"; }
    return names.join(", ");
}

} // namespace

struct ApiServer::Call {
    QTcpSocket* socket = nullptr;
    QJsonValue  id;
    bool        deferred = false;
    Pending     pending;
};

ApiServer::ApiServer(SceneGraph* scene, SimulationEngine* sim, QObject* parent)
    : QObject(parent), m_scene(scene), m_sim(sim)
{
    connect(m_sim->automation(), &AutomationEngine::scanned, this, &ApiServer::checkPending);
    connect(m_sim, &SimulationEngine::activeChanged, this, [this](bool active) {
        if (!active) checkPending();
    });
}

ApiServer::~ApiServer() { stop(); }

// ── Robots ───────────────────────────────────────────────────────────────────

QString ApiServer::pathOf(const SceneGraph& g, int id) {
    QStringList parts;
    for (const SceneObject* o = g.getObject(id); o; o = g.getObject(o->parentId)) parts.prepend(o->name);
    return parts.join('/');
}

// Joints are one machine when one hangs below another, or when their top
// joints sit on one rigid part below the scene's top (the pelvis of a
// humanoid carries its waist and both hips). A machine's root is the highest
// object above them that holds no other machine's joints.
std::vector<ApiServer::Robot> ApiServer::findRobots(const SceneGraph& g) {
    std::vector<int> joints;
    for (int mid : g.motorIdsSorted())
        if (const Motor* m = g.getMotor(mid); m && g.getJoint(m->jointId)) joints.push_back(m->jointId);
    if (joints.empty()) return {};

    std::map<int, int> parent;   // union-find over joint ids
    std::function<int(int)> find = [&](int j) { return parent[j] == j ? j : parent[j] = find(parent[j]); };
    auto unite = [&](int a, int b) { parent[find(a)] = find(b); };
    std::map<int, int> actuatedByChild;
    for (int j : joints) { parent[j] = j; actuatedByChild[g.getJoint(j)->childBodyId] = j; }

    // Each joint joins the nearest actuated joint above it (through followers)
    std::set<int> tops;
    for (int j : joints) {
        bool found = false;
        for (const SceneObject* o = g.getObject(g.getJoint(j)->parentBodyId); o; o = g.getObject(o->parentId)) {
            auto it = actuatedByChild.find(o->id);
            if (it != actuatedByChild.end()) { unite(j, it->second); found = true; break; }
        }
        if (!found) tops.insert(j);
    }
    auto ancestors = [&](int id) {
        std::vector<int> out;
        for (const SceneObject* o = g.getObject(id); o; o = g.getObject(o->parentId)) out.push_back(o->id);
        return out;   // self first
    };
    auto lca = [&](int a, int b) {
        if (a == -1) return b;
        const std::vector<int> aa = ancestors(a);
        for (int x : ancestors(b))
            if (std::find(aa.begin(), aa.end(), x) != aa.end()) return x;
        return -1;
    };
    // The rigid part an object belongs to: up to a joint's child or the top
    std::set<int> jointChildren;
    for (const auto& [jid, j] : g.allJoints()) jointChildren.insert(j.childBodyId);
    auto rigidRoot = [&](int id) {
        int r = id;
        for (const SceneObject* o = g.getObject(id); o; o = g.getObject(o->parentId)) {
            r = o->id;
            if (jointChildren.count(o->id)) break;
        }
        return r;
    };
    // Where each group's top joints sit
    std::map<int, int> base;   // group → lowest common ancestor of its top joints' parents
    for (int t : tops) base[find(t)] = lca(base.count(find(t)) ? base[find(t)] : -1, g.getJoint(t)->parentBodyId);
    for (bool merged = true; merged;) {
        merged = false;
        for (auto a = base.begin(); a != base.end() && !merged; ++a)
            for (auto b = std::next(a); b != base.end() && !merged; ++b) {
                const int l = lca(a->second, b->second);
                if (l == -1 || rigidRoot(a->second) != rigidRoot(b->second) || l == rigidRoot(a->second)) continue;
                unite(a->first, b->first);
                const int keep = find(a->first);
                const int at = l;
                base.erase(a->first == keep ? b->first : a->first);
                base[keep] = at;
                merged = true;
            }
    }

    std::map<int, std::vector<int>> groups;
    for (int j : joints) groups[find(j)].push_back(j);
    std::vector<Robot> out;
    for (auto& [key, members] : groups) {
        int start = base.count(key) ? base[key] : g.getJoint(members.front())->parentBodyId;
        if (!g.getObject(start)) continue;
        const std::set<int> mine(members.begin(), members.end());
        auto onlyMine = [&](int id) {
            std::vector<int> sub;
            collect(g, id, sub);
            for (int o : sub) {
                auto it = actuatedByChild.find(o);
                if (it != actuatedByChild.end() && !mine.count(it->second)) return false;
            }
            return true;
        };
        int root = start;
        // A machine on a part that carries other machines too (a turntable
        // in a cell) is named after what moves
        std::vector<int> myTops;
        for (int t : tops) if (mine.count(t)) myTops.push_back(t);
        if (!onlyMine(start) && myTops.size() == 1) root = g.getJoint(myTops.front())->childBodyId;
        while (true) {
            const SceneObject* o = g.getObject(root);
            if (!o || o->parentId == -1 || !onlyMine(o->parentId)) break;
            root = o->parentId;
        }
        Robot r;
        r.root = root;
        r.name = g.getObject(root)->name;
        r.path = pathOf(g, root);
        r.joints = members;
        r.tools = toolsOf(g, root, members);
        out.push_back(std::move(r));
    }
    std::sort(out.begin(), out.end(), [](const Robot& a, const Robot& b) { return a.path < b.path; });
    return out;
}

// ── Server ───────────────────────────────────────────────────────────────────

bool ApiServer::start(quint16 port, const QHostAddress& address) {
    stop();
    m_server = new QTcpServer(this);
    if (!m_server->listen(address, port)) {
        m_error = m_server->errorString();
        delete m_server;
        m_server = nullptr;
        return false;
    }
    m_error.clear();
    m_requests = 0;
    connect(m_server, &QTcpServer::newConnection, this, &ApiServer::onNewConnection);
    return true;
}

void ApiServer::stop() {
    m_pending.clear();
    for (QTcpSocket* s : m_clients) {
        s->disconnect(this);
        s->abort();
        s->deleteLater();
    }
    const bool had = !m_clients.empty();
    m_clients.clear();
    m_buffers.clear();
    if (m_server) {
        m_server->close();
        delete m_server;
        m_server = nullptr;
    }
    if (had) emit clientsChanged(0);
}

bool ApiServer::isListening() const { return m_server && m_server->isListening(); }
quint16 ApiServer::port() const { return m_server ? m_server->serverPort() : 0; }

void ApiServer::onNewConnection() {
    while (QTcpSocket* s = m_server->nextPendingConnection()) {
        s->setSocketOption(QAbstractSocket::LowDelayOption, 1);
        m_clients.push_back(s);
        connect(s, &QTcpSocket::readyRead, this, [this, s] { onReadyRead(s); });
        connect(s, &QTcpSocket::disconnected, this, [this, s] { onDisconnected(s); });
        emit clientsChanged(clientCount());
    }
}

void ApiServer::onDisconnected(QTcpSocket* s) {
    // Its motions carry on; only the answers it waits for are dropped
    m_pending.erase(std::remove_if(m_pending.begin(), m_pending.end(),
                                   [s](const Pending& p) { return p.socket == s; }),
                    m_pending.end());
    m_clients.erase(std::remove(m_clients.begin(), m_clients.end(), s), m_clients.end());
    m_buffers.remove(s);
    s->deleteLater();
    emit clientsChanged(clientCount());
}

void ApiServer::onReadyRead(QTcpSocket* s) {
    QByteArray& buf = m_buffers[s];
    buf += s->readAll();
    int nl;
    while ((nl = buf.indexOf('\n')) >= 0) {
        const QByteArray line = buf.left(nl).trimmed();
        buf.remove(0, nl + 1);
        if (!line.isEmpty()) handleLine(s, line);
        if (!m_buffers.contains(s)) return;   // disconnected while handling
    }
    if (buf.size() > kMaxLine) {
        send(s, QJsonObject{{"id", QJsonValue()}, {"error", QJsonObject{{"code", "bad_request"},
                                                                         {"message", "request too long"}}}});
        s->disconnectFromHost();
    }
}

void ApiServer::send(QTcpSocket* s, const QJsonObject& msg) {
    if (!s || s->state() != QAbstractSocket::ConnectedState) return;
    s->write(QJsonDocument(msg).toJson(QJsonDocument::Compact) + '\n');
}

const std::map<QString, ApiServer::Handler>& ApiServer::handlers() {
    static const std::map<QString, Handler> h = {
        {"hello",            &ApiServer::hello},
        {"sim.state",        &ApiServer::simState},
        {"sim.play",         &ApiServer::simPlay},
        {"sim.pause",        &ApiServer::simPause},
        {"sim.reset",        &ApiServer::simReset},
        {"sim.speed",        &ApiServer::simSpeed},
        {"sim.wait",         &ApiServer::simWait},
        {"scene.robots",     &ApiServer::sceneRobots},
        {"robot.info",       &ApiServer::robotInfo},
        {"robot.joints",     &ApiServer::robotJoints},
        {"robot.moveJoints", &ApiServer::robotMoveJoints},
        {"robot.moveToPose", &ApiServer::robotMoveToPose},
        {"robot.home",       &ApiServer::robotHome},
        {"robot.stop",       &ApiServer::robotStop},
        {"tool.pose",        &ApiServer::toolPose},
        {"tool.moveL",       [](ApiServer* s, const QJsonObject& p, Call& c) { return s->toolMove(p, c, true); }},
        {"tool.moveJ",       [](ApiServer* s, const QJsonObject& p, Call& c) { return s->toolMove(p, c, false); }},
        {"motion.status",    &ApiServer::motionStatus},
        {"motion.wait",      &ApiServer::motionWait},
        {"motion.cancel",    &ApiServer::motionCancel},
        {"actuator.get",     &ApiServer::actuatorGet},
        {"actuator.run",     &ApiServer::actuatorRun},
        {"signal.list",      &ApiServer::signalList},
        {"signal.get",       &ApiServer::signalGet},
        {"signal.set",       &ApiServer::signalSet},
        {"signal.waitUntil", &ApiServer::signalWaitUntil},
    };
    return h;
}

void ApiServer::handleLine(QTcpSocket* s, const QByteArray& line) {
    ++m_requests;
    emit activity();
    QJsonParseError perr;
    const QJsonDocument doc = QJsonDocument::fromJson(line, &perr);
    Call call;
    call.socket = s;
    auto error = [&](const QString& code, const QString& message) {
        send(s, QJsonObject{{"id", call.id}, {"error", QJsonObject{{"code", code}, {"message", message}}}});
    };
    if (perr.error != QJsonParseError::NoError || !doc.isObject())
        return error("bad_request", "each line must be one JSON object: " +
                     (perr.error != QJsonParseError::NoError ? perr.errorString() : QString("not an object")));
    const QJsonObject req = doc.object();
    call.id = req.value("id");
    const QString method = req.value("method").toString();
    auto it = handlers().find(method);
    if (it == handlers().end())
        return error("unknown_method", method.isEmpty() ? QString("no \"method\"")
                                                        : QString("there is no method \"%1\"").arg(method));
    const QJsonValue pv = req.value("params");
    if (!pv.isUndefined() && !pv.isNull() && !pv.isObject()) return error("bad_request", "\"params\" must be an object");
    try {
        const QJsonValue result = it->second(this, pv.toObject(), call);
        if (call.deferred) {
            call.pending.socket = s;
            call.pending.id = call.id;
            m_pending.push_back(std::move(call.pending));
            return;
        }
        send(s, QJsonObject{{"id", call.id}, {"result", result}});
    } catch (const ApiError& e) {
        error(e.code, e.message);
    }
}

void ApiServer::checkPending() {
    AutomationEngine* a = m_sim->automation();
    const bool active = a->isActive();
    const double now = a->time();
    for (size_t i = 0; i < m_pending.size();) {
        Pending& p = m_pending[i];
        QJsonObject reply{{"id", p.id}};
        bool answered = true;
        auto error = [&](const QString& code, const QString& message) {
            reply["error"] = QJsonObject{{"code", code}, {"message", message}};
        };
        if (p.kind == Pending::Kind::Motion) {
            const auto st = a->motionStatus(p.motion);
            if (st.state == MS::Done) reply["result"] = motionReply(p.motion);
            else if (st.finished()) {
                QJsonObject e{{"code", st.code}, {"message", st.error}, {"motion", p.motion}};
                reply["error"] = e;
            } else if (p.timeoutAt >= 0 && now >= p.timeoutAt - 1e-9)
                error("timeout", QString("motion %1 is still %2").arg(p.motion).arg(AutomationEngine::motionStateName(st.state)));
            else answered = false;
        } else if (!active) {
            error("reset", "the simulation was reset");
        } else if (p.kind == Pending::Kind::Time) {
            if (now >= p.until - 1e-9) reply["result"] = QJsonObject{{"time", now}};
            else answered = false;
        } else {
            if (a->evaluate(p.condition) != 0.0) reply["result"] = QJsonObject{{"time", now}};
            else if (p.timeoutAt >= 0 && now >= p.timeoutAt - 1e-9)
                error("timeout", QString("\"%1\" did not become true in time").arg(p.conditionText));
            else answered = false;
        }
        if (!answered) { ++i; continue; }
        QTcpSocket* s = p.socket;
        m_pending.erase(m_pending.begin() + long(i));
        send(s, reply);
    }
}

// ── Helpers ──────────────────────────────────────────────────────────────────

void ApiServer::ensureRunning() {
    if (m_sim->isRunning()) return;
    if (!m_sim->isActive()) {
        const QStringList errs = AutomationEngine::validateAll(*m_scene);
        if (!errs.isEmpty()) fail("simulation", "the simulation can't start: " + errs.join("; "));
    }
    m_sim->play();
    if (!m_sim->isRunning()) fail("simulation", "the simulation can't start");
}

int ApiServer::objectParam(const QString& what, const QString& name, int within) const {
    // Exact name or path (or the end of a path), preferring objects under `within`
    std::vector<int> hits;
    for (const auto& [id, o] : m_scene->allObjects()) {
        if (o.copyOf != -1) continue;
        if (name.contains('/')) {
            const QString path = pathOf(*m_scene, id);
            if (path == name || path.endsWith('/' + name)) hits.push_back(id);
        } else if (o.name == name) {
            hits.push_back(id);
        }
    }
    if (within != -1) {
        std::vector<int> inside;
        for (int id : hits)
            if (inSubtree(*m_scene, within, id)) inside.push_back(id);
        if (!inside.empty()) hits = inside;
    }
    if (hits.size() == 1) return hits.front();
    if (hits.empty()) fail("not_found", QString("there is no %1 \"%2\"").arg(what, name));
    QStringList paths;
    for (int id : hits) paths << pathOf(*m_scene, id);
    fail("ambiguous", QString("%1 objects are called \"%2\"; use a path: %3").arg(hits.size()).arg(name, listOf(paths)));
}

ApiServer::Robot ApiServer::robotParam(const QJsonObject& p, bool required) const {
    const QString name = text(p, "robot", required);
    if (name.isEmpty()) return {};
    const auto robots = findRobots(*m_scene);
    for (const Robot& r : robots)
        if (r.path == name || r.name == name || r.path.endsWith('/' + name)) {
            // Names can repeat: only take a match by name when it is the only one
            const int n = int(std::count_if(robots.begin(), robots.end(), [&](const Robot& o) {
                return o.path == name || o.name == name || o.path.endsWith('/' + name); }));
            if (n == 1 || r.path == name) return r;
        }
    // Any object with actuated joints below it works as a robot
    const int id = objectParam("robot", name);
    Robot r;
    r.root = id;
    r.name = m_scene->getObject(id)->name;
    r.path = pathOf(*m_scene, id);
    for (int mid : m_scene->motorIdsSorted())
        if (const Motor* m = m_scene->getMotor(mid))
            if (const Joint* j = m_scene->getJoint(m->jointId); j && inSubtree(*m_scene, id, j->childBodyId))
                r.joints.push_back(j->id);
    if (r.joints.empty()) {
        QStringList names;
        for (const Robot& o : robots) names << o.path;
        fail("not_found", QString("\"%1\" has no actuated joints; robots: %2").arg(name, names.isEmpty() ? QString("none") : listOf(names)));
    }
    r.tools = toolsOf(*m_scene, id, r.joints);
    return r;
}

int ApiServer::toolParam(const QJsonObject& p, const Robot& r) const {
    const QString name = text(p, "tool", false);
    if (!name.isEmpty()) return objectParam("tool", name, r.root);
    if (r.root == -1) fail("bad_request", "give a \"robot\" or a \"tool\"");
    if (r.tools.size() == 1) return r.tools.front();
    if (r.tools.empty()) fail("not_found", QString("%1 has no tool; name an object as the tool").arg(r.name));
    QStringList names;
    for (int t : r.tools) names << m_scene->getObject(t)->name;
    fail("ambiguous", QString("%1 has %2 tools: pick one of %3").arg(r.name).arg(names.size()).arg(listOf(names)));
}

int ApiServer::jointByName(const QString& name, const Robot* r) const {
    for (const auto& [mid, m] : m_scene->allMotors()) {
        if (m.name != name) continue;
        if (r && r->root != -1 && std::find(r->joints.begin(), r->joints.end(), m.jointId) == r->joints.end())
            fail("not_found", QString("%1 is not a joint of %2").arg(name, r->name));
        return m.jointId;
    }
    QStringList names;
    if (r && r->root != -1)
        for (int j : r->joints) { if (const Motor* m = m_scene->motorForJoint(j)) names << m->name; }
    else
        for (const auto& [mid, m] : m_scene->allMotors()) names << m.name;
    fail("not_found", QString("there is no joint \"%1\"%2").arg(name, names.isEmpty() ? QString()
                                                                    : "; joints: " + listOf(names, 30)));
}

QJsonObject ApiServer::poseOf(int objectId, int frameId) const {
    const QMatrix4x4 ref = frameId != -1 ? kin::toolFrame(*m_scene, frameId) : QMatrix4x4();
    const Transform t = SceneGraph::toTransform(ref.inverted() * kin::toolFrame(*m_scene, objectId));
    return QJsonObject{{"x", round6(t.position.x() * kMm)}, {"y", round6(t.position.y() * kMm)},
                       {"z", round6(t.position.z() * kMm)}, {"rx", round6(t.rotation.x())},
                       {"ry", round6(t.rotation.y())}, {"rz", round6(t.rotation.z())}};
}

QJsonObject ApiServer::motionReply(int id) const {
    const auto st = m_sim->automation()->motionStatus(id);
    QJsonObject o{{"motion", id}, {"state", AutomationEngine::motionStateName(st.state)}, {"progress", st.progress}};
    if (!st.code.isEmpty()) { o["code"] = st.code; o["error"] = st.error; }
    return o;
}

QJsonValue ApiServer::submit(const QJsonObject& p, Call& call, const AutomationEngine::MotionRequest& req) {
    ensureRunning();
    QString code, err;
    const int id = m_sim->automation()->submitMotion(req, &code, &err);
    if (!id) fail(code, err);
    if (flag(p, "wait", true)) {
        call.deferred = true;
        call.pending.kind = Pending::Kind::Motion;
        call.pending.motion = id;
        return {};
    }
    return motionReply(id);
}

// ── Methods ──────────────────────────────────────────────────────────────────

QJsonValue ApiServer::hello(const QJsonObject&, Call&) {
    return QJsonObject{{"server", "MotionStudio"},
                       {"version", QCoreApplication::applicationVersion()},
                       {"protocol", kProtocol},
                       {"units", QJsonObject{{"length", "mm"}, {"angle", "deg"}, {"time", "s"}}}};
}

QJsonValue ApiServer::simState(const QJsonObject&, Call&) {
    return QJsonObject{{"active", m_sim->isActive()}, {"running", m_sim->isRunning()},
                       {"time", m_sim->automation()->isActive() ? m_sim->automation()->time() : 0.0},
                       {"speed", m_sim->speedFactor()}};
}

QJsonValue ApiServer::simPlay(const QJsonObject& p, Call& c) {
    ensureRunning();
    return simState(p, c);
}

QJsonValue ApiServer::simPause(const QJsonObject& p, Call& c) {
    m_sim->pause();
    return simState(p, c);
}

QJsonValue ApiServer::simReset(const QJsonObject& p, Call& c) {
    m_sim->reset();
    return simState(p, c);
}

QJsonValue ApiServer::simSpeed(const QJsonObject& p, Call& c) {
    const double f = number(p, "factor", -1);
    if (f <= 0) fail("bad_request", "\"factor\" must be a positive number (1 = real time)");
    m_sim->setSpeedFactor(f);
    return simState(p, c);
}

QJsonValue ApiServer::simWait(const QJsonObject& p, Call& c) {
    const double s = number(p, "seconds", -1);
    if (s < 0) fail("bad_request", "\"seconds\" must be zero or more");
    ensureRunning();
    c.deferred = true;
    c.pending.kind = Pending::Kind::Time;
    c.pending.until = m_sim->automation()->time() + s;
    return {};
}

QJsonValue ApiServer::sceneRobots(const QJsonObject&, Call&) {
    QJsonArray out;
    for (const Robot& r : findRobots(*m_scene)) {
        QJsonArray joints, tools;
        for (int j : r.joints) if (const Motor* m = m_scene->motorForJoint(j)) joints.append(m->name);
        for (int t : r.tools) tools.append(m_scene->getObject(t)->name);
        out.append(QJsonObject{{"name", r.name}, {"path", r.path}, {"joints", joints}, {"tools", tools}});
    }
    return out;
}

QJsonValue ApiServer::robotInfo(const QJsonObject& p, Call&) {
    const Robot r = robotParam(p);
    QJsonArray joints, tools, poses;
    for (int jid : r.joints) {
        const Joint* j = m_scene->getJoint(jid);
        const Motor* m = m_scene->motorForJoint(jid);
        if (!j || !m) continue;
        const bool prismatic = j->type == JointType::Prismatic;
        QJsonObject o{{"name", m->name},
                      {"type", prismatic ? "prismatic" : "revolute"},
                      {"unit", prismatic ? "mm" : "deg"},
                      {"value", toApi(*j, j->currentValue)},
                      {"speed", toApi(*j, m->velocity)},
                      {"acceleration", toApi(*j, m->acceleration)}};
        if (!prismatic && j->continuous) o["continuous"] = true;
        else { o["min"] = toApi(*j, std::min(j->limitMin, j->limitMax)); o["max"] = toApi(*j, std::max(j->limitMin, j->limitMax)); }
        joints.append(o);
    }
    for (int t : r.tools)
        tools.append(QJsonObject{{"name", m_scene->getObject(t)->name}, {"path", pathOf(*m_scene, t)},
                                 {"joints", int(kin::chainTo(*m_scene, t).size())}});
    const std::set<int> mine(r.joints.begin(), r.joints.end());
    for (const auto& [id, pose] : m_scene->allPoses())
        if (std::any_of(pose.jointValues.begin(), pose.jointValues.end(), [&](auto& kv) { return mine.count(kv.first) > 0; }))
            poses.append(pose.name);
    return QJsonObject{{"name", r.name}, {"path", r.path}, {"joints", joints}, {"tools", tools}, {"poses", poses}};
}

QJsonValue ApiServer::robotJoints(const QJsonObject& p, Call&) {
    const Robot r = robotParam(p);
    QJsonObject out;
    for (int jid : r.joints)
        if (const Motor* m = m_scene->motorForJoint(jid))
            out[m->name] = toApi(*m_scene->getJoint(jid), m_scene->getJoint(jid)->currentValue);
    return out;
}

QJsonValue ApiServer::robotMoveJoints(const QJsonObject& p, Call& c) {
    const Robot r = robotParam(p, false);
    const QJsonValue jv = p.value("joints");
    if (!jv.isObject() || jv.toObject().isEmpty())
        fail("bad_request", "\"joints\" must be an object of joint names and values, e.g. {\"J1\": 30}");
    AutomationEngine::MotionRequest req;
    const QJsonObject jo = jv.toObject();
    for (auto it = jo.begin(); it != jo.end(); ++it) {
        if (!it.value().isDouble()) fail("bad_request", QString("the value of %1 must be a number").arg(it.key()));
        const int jid = jointByName(it.key(), r.root != -1 ? &r : nullptr);
        req.joints[jid] = fromApi(*m_scene->getJoint(jid), it.value().toDouble());
    }
    req.relativeJoints = flag(p, "relative", false);
    req.speed = number(p, "speed", 0);
    req.label = "moveJoints";
    return submit(p, c, req);
}

QJsonValue ApiServer::robotMoveToPose(const QJsonObject& p, Call& c) {
    const Robot r = robotParam(p, false);
    const QString name = text(p, "pose");
    const Pose* pose = nullptr;
    QStringList names;
    for (const auto& [id, ps] : m_scene->allPoses()) {
        if (ps.name == name) pose = &ps;
        names << ps.name;
    }
    if (!pose) fail("not_found", QString("there is no pose \"%1\"%2").arg(name, names.isEmpty() ? QString() : "; poses: " + listOf(names)));
    if (pose->isRelative())
        fail("bad_request", QString("%1 was taught on a part; move to it from a sequence (MoveL / MoveToPose)").arg(name));
    AutomationEngine::MotionRequest req;
    for (auto& [jid, v] : pose->jointValues) {
        if (!m_scene->motorForJoint(jid)) continue;
        if (r.root != -1 && std::find(r.joints.begin(), r.joints.end(), jid) == r.joints.end()) continue;
        req.joints[jid] = v;
    }
    if (req.joints.empty()) fail("bad_request", QString("%1 holds no joint of %2").arg(name, r.name));
    req.speed = number(p, "speed", 0);
    req.label = "moveToPose " + name;
    return submit(p, c, req);
}

QJsonValue ApiServer::robotHome(const QJsonObject& p, Call& c) {
    const Robot r = robotParam(p);
    QStringList candidates{r.name + " Home", "Home"};
    for (const QString& n : candidates)
        for (const auto& [id, ps] : m_scene->allPoses())
            if (ps.name == n && !ps.isRelative()) {
                QJsonObject q = p;
                q["pose"] = n;
                return robotMoveToPose(q, c);
            }
    fail("not_found", QString("%1 has no home pose (a pose called \"%1 Home\" or \"Home\")").arg(r.name));
}

QJsonValue ApiServer::robotStop(const QJsonObject& p, Call&) {
    const Robot r = robotParam(p, false);
    AutomationEngine* a = m_sim->automation();
    const std::set<int> joints(r.joints.begin(), r.joints.end());
    const int n = a->isActive() ? a->cancelMotions(r.root != -1 ? joints : std::set<int>{}, "stopped") : 0;
    return QJsonObject{{"stopped", n}};
}

QJsonValue ApiServer::toolPose(const QJsonObject& p, Call&) {
    const Robot r = robotParam(p, false);
    const int tool = toolParam(p, r);
    const QString frame = text(p, "frame", false);
    int ref = r.root;
    if (frame == "world") ref = -1;
    else if (!frame.isEmpty() && frame != "base") ref = objectParam("frame", frame);
    return poseOf(tool, ref);
}

QJsonValue ApiServer::toolMove(const QJsonObject& p, Call& c, bool linear) {
    Robot r = robotParam(p, false);
    const int tool = toolParam(p, r);
    if (r.root == -1)   // the robot the tool belongs to gives the base frame
        for (const Robot& o : findRobots(*m_scene))
            if (inSubtree(*m_scene, o.root, tool)) { r = o; break; }

    AutomationEngine::MotionRequest req;
    req.kind = linear ? AutomationEngine::MotionKind::Linear : AutomationEngine::MotionKind::JointsToPose;
    req.tool = tool;
    req.label = linear ? "moveL" : "moveJ";
    const QString base = text(p, "base", false);
    if (!base.isEmpty()) {
        // Only the joints below `base` move (a humanoid's arm without its waist)
        const int b = objectParam("base", base, r.root);
        for (int jid : kin::chainTo(*m_scene, tool))
            if (inSubtree(*m_scene, b, m_scene->getJoint(jid)->childBodyId) && m_scene->getJoint(jid)->childBodyId != b)
                req.chain.push_back(jid);
        if (req.chain.empty()) fail("bad_request", QString("no actuated joint lies between %1 and the tool").arg(base));
    }

    const QJsonValue tv = p.value("target");
    if (!tv.isObject()) fail("bad_request", "\"target\" must be an object with any of x, y, z, rx, ry, rz");
    const QJsonObject t = tv.toObject();
    for (auto it = t.begin(); it != t.end(); ++it)
        if (!QStringList{"x", "y", "z", "rx", "ry", "rz"}.contains(it.key()))
            fail("bad_request", QString("unknown target field \"%1\" (use x, y, z, rx, ry, rz)").arg(it.key()));
    auto get = [&](const char* k, double scale) -> std::optional<double> {
        const QJsonValue v = t.value(k);
        if (v.isUndefined() || v.isNull()) return std::nullopt;
        if (!v.isDouble()) fail("bad_request", QString("target %1 must be a number").arg(k));
        return v.toDouble() * scale;
    };
    auto& g = req.goal;
    g.x = get("x", 1 / kMm); g.y = get("y", 1 / kMm); g.z = get("z", 1 / kMm);
    g.rx = get("rx", 1); g.ry = get("ry", 1); g.rz = get("rz", 1);
    g.relative = flag(p, "relative", false);
    const QString frame = text(p, "frame", false);
    if (frame == "tool") { g.relative = true; g.toolAxes = true; }
    else if (frame == "world") g.frameId = -1;
    else if (frame.isEmpty() || frame == "base") g.frameId = r.root;
    else g.frameId = objectParam("frame", frame);

    const double speed = number(p, "speed", 0);
    if (speed < 0) fail("bad_request", "\"speed\" must be positive");
    req.speed = linear ? speed / kMm : speed;
    return submit(p, c, req);
}

QJsonValue ApiServer::motionStatus(const QJsonObject& p, Call&) {
    const int id = int(number(p, "motion", -1));
    if (!m_sim->automation()->hasMotion(id)) fail("not_found", QString("there is no motion %1").arg(id));
    return motionReply(id);
}

QJsonValue ApiServer::motionWait(const QJsonObject& p, Call& c) {
    const int id = int(number(p, "motion", -1));
    AutomationEngine* a = m_sim->automation();
    if (!a->hasMotion(id)) fail("not_found", QString("there is no motion %1").arg(id));
    c.deferred = true;
    c.pending.kind = Pending::Kind::Motion;
    c.pending.motion = id;
    const double timeout = number(p, "timeout", -1);
    if (timeout >= 0) c.pending.timeoutAt = a->time() + timeout;
    return {};
}

QJsonValue ApiServer::motionCancel(const QJsonObject& p, Call&) {
    const int id = int(number(p, "motion", -1));
    AutomationEngine* a = m_sim->automation();
    if (!a->hasMotion(id)) fail("not_found", QString("there is no motion %1").arg(id));
    a->cancelMotion(id);
    return motionReply(id);
}

QJsonValue ApiServer::actuatorGet(const QJsonObject& p, Call&) {
    const Joint& j = *m_scene->getJoint(jointByName(text(p, "name"), nullptr));
    return QJsonObject{{"name", p.value("name")}, {"unit", j.type == JointType::Prismatic ? "mm" : "deg"},
                       {"value", toApi(j, j.currentValue)}};
}

QJsonValue ApiServer::actuatorRun(const QJsonObject& p, Call&) {
    const int jid = jointByName(text(p, "name"), nullptr);
    const double speed = number(p, "speed", 0);
    ensureRunning();
    QString err;
    if (!m_sim->automation()->runActuator(jid, fromApi(*m_scene->getJoint(jid), speed), &err)) fail("simulation", err);
    return QJsonObject{{"name", p.value("name")}, {"speed", speed}};
}

QJsonValue ApiServer::signalList(const QJsonObject&, Call&) {
    QJsonArray out;
    AutomationEngine* a = m_sim->automation();
    for (const auto& [id, s] : m_scene->allSignals())
        out.append(QJsonObject{{"name", s.name},
                               {"kind", signalKindName(s.kind)},
                               {"type", signalTypeName(s.type)},
                               {"writable", s.kind == SignalKind::Input || s.kind == SignalKind::Internal},
                               {"value", a->isActive() ? a->value(s.name) : coerceSignal(s.type, s.initialValue)}});
    return out;
}

QJsonValue ApiServer::signalGet(const QJsonObject& p, Call&) {
    const QString name = text(p, "name");
    AutomationEngine* a = m_sim->automation();
    if (a->isActive()) {
        if (a->slotOf(name) < 0) fail("not_found", QString("there is no signal \"%1\"").arg(name));
        return QJsonObject{{"value", a->value(name)}};
    }
    for (const auto& [id, s] : m_scene->allSignals())
        if (s.name == name) return QJsonObject{{"value", coerceSignal(s.type, s.initialValue)}};
    if (AutomationEngine::symbolNames(*m_scene).contains(name))
        fail("not_running", QString("%1 has a value only while the simulation runs").arg(name));
    fail("not_found", QString("there is no signal \"%1\"").arg(name));
}

QJsonValue ApiServer::signalSet(const QJsonObject& p, Call&) {
    const QString name = text(p, "name");
    Signal* s = m_scene->findSignal(name);
    if (!s) fail("not_found", QString("there is no signal \"%1\"").arg(name));
    if (s->kind != SignalKind::Input && s->kind != SignalKind::Internal)
        fail("bad_request", QString("%1 is a %2 signal; only Input and Internal signals can be set")
                                .arg(name, signalKindName(s->kind)));
    const QJsonValue v = p.value("value");
    if (!v.isDouble() && !v.isBool()) fail("bad_request", "\"value\" must be a number or true/false");
    ensureRunning();
    const double value = coerceSignal(s->type, v.isBool() ? (v.toBool() ? 1 : 0) : v.toDouble());
    m_sim->automation()->setInput(name, value);
    return QJsonObject{{"value", value}};
}

QJsonValue ApiServer::signalWaitUntil(const QJsonObject& p, Call& c) {
    const QString cond = text(p, "condition");
    ensureRunning();
    AutomationEngine* a = m_sim->automation();
    QString err;
    if (!a->compileExpression(cond, c.pending.condition, &err)) fail("bad_request", QString("\"%1\": %2").arg(cond, err));
    c.pending.kind = Pending::Kind::Condition;
    c.pending.conditionText = cond;
    const double timeout = number(p, "timeout", -1);
    if (timeout >= 0) c.pending.timeoutAt = a->time() + timeout;
    c.deferred = true;
    return {};
}
