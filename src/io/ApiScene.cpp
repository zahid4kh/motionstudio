// The API's scene-building methods: projects, objects, joints and actuators,
// physics, signals and sensors, poses, sequences and part flow. Lengths are
// mm and angles degrees like the rest of the API; a number given for an
// automation value is in API units too, while a string is an expression in
// the automation's own units (m, m/s, degrees), as typed in the app.
#include "ApiCommon.h"
#include "RobotFormats.h"
#include "../automation/Expression.h"
#include "../automation/Kinematics.h"
#include "../import/StepImporter.h"
#include "../physics/SimulationEngine.h"
#include "../scene/SceneMerge.h"
#include "../scene/SceneQuery.h"
#include "../serialization/ProjectSerializer.h"
#include "../viewport/MeshFeatures.h"
#include <QFile>
#include <QFileInfo>
#include <QFutureWatcher>
#include <QJsonDocument>
#include <QPointer>
#include <QRegularExpression>
#include <QTcpSocket>
#include <QtConcurrent/QtConcurrentRun>
#include <QtMath>
#include <algorithm>
#include <set>

using namespace apicommon;

namespace {

QString typeName(const SceneObject& o) {
    switch (o.type) {
        case PrimitiveType::Cube:     return "box";
        case PrimitiveType::Cylinder: return "cylinder";
        case PrimitiveType::Sphere:   return "sphere";
        case PrimitiveType::Capsule:  return "capsule";
        case PrimitiveType::Basket:   return "basket";
        case PrimitiveType::ImportedMesh: return o.mesh ? "mesh" : "frame";
    }
    return "frame";
}

PrimitiveType typeFromName(const QString& name) {
    const QString n = name.toLower();
    if (n == "frame")                  return PrimitiveType::ImportedMesh;
    if (n == "box" || n == "cube")     return PrimitiveType::Cube;
    if (n == "cylinder")               return PrimitiveType::Cylinder;
    if (n == "sphere")                 return PrimitiveType::Sphere;
    if (n == "capsule")                return PrimitiveType::Capsule;
    if (n == "basket")                 return PrimitiveType::Basket;
    fail("bad_request", QString("there is no object type \"%1\"; types: frame, box, cylinder, sphere, capsule, basket").arg(name));
}

QJsonObject vecJson(const QVector3D& v, double scale = kMm) {
    auto r = [scale](float x) { return scale == kMm ? roundMm(x * scale) : round6(x * scale); };
    return QJsonObject{{"x", r(v.x())}, {"y", r(v.y())}, {"z", r(v.z())}};
}

// {x, y, z} (missing components are 0) or [x, y, z], as given (no units applied)
QVector3D readVec(const QJsonObject& p, const QString& key) {
    const QJsonValue v = p.value(key);
    if (v.isArray()) {
        const QJsonArray a = v.toArray();
        if (a.size() != 3 || !a[0].isDouble() || !a[1].isDouble() || !a[2].isDouble())
            fail("bad_request", QString("\"%1\" must be [x, y, z]").arg(key));
        return QVector3D(float(a[0].toDouble()), float(a[1].toDouble()), float(a[2].toDouble()));
    }
    if (!v.isObject()) fail("bad_request", QString("\"%1\" must be {x, y, z} or [x, y, z]").arg(key));
    const QJsonObject o = v.toObject();
    for (auto it = o.begin(); it != o.end(); ++it)
        if (it.key() != "x" && it.key() != "y" && it.key() != "z")
            fail("bad_request", QString("\"%1\" has no \"%2\": use x, y and z").arg(key, it.key()));
    return QVector3D(float(number(o, "x", 0)), float(number(o, "y", 0)), float(number(o, "z", 0)));
}

QMatrix4x4 compose(const QVector3D& position, const QVector3D& eulerDeg) {
    QMatrix4x4 m;
    m.translate(position);
    m.rotate(eulerDeg.x(), 1, 0, 0);
    m.rotate(eulerDeg.y(), 0, 1, 0);
    m.rotate(eulerDeg.z(), 0, 0, 1);
    return m;
}

QJsonObject poseJson(const QMatrix4x4& m) {
    const Transform t = SceneGraph::toTransform(m);
    return QJsonObject{{"x", roundMm(t.position.x() * kMm)}, {"y", roundMm(t.position.y() * kMm)},
                       {"z", roundMm(t.position.z() * kMm)}, {"rx", round6(t.rotation.x())},
                       {"ry", round6(t.rotation.y())}, {"rz", round6(t.rotation.z())}};
}

// A pose {x, y, z, rx, ry, rz} (mm, degrees); what is left out keeps `current`
QMatrix4x4 readPose(const QJsonObject& p, const QString& key, const QMatrix4x4& current) {
    const QJsonValue v = p.value(key);
    if (!v.isObject()) fail("bad_request", QString("\"%1\" must be {x, y, z, rx, ry, rz} (mm, degrees)").arg(key));
    const QJsonObject o = v.toObject();
    static const QStringList keys{"x", "y", "z", "rx", "ry", "rz"};
    for (auto it = o.begin(); it != o.end(); ++it)
        if (!keys.contains(it.key()))
            fail("bad_request", QString("\"%1\" has no \"%2\": use x, y, z, rx, ry, rz").arg(key, it.key()));
    const Transform t = SceneGraph::toTransform(current);
    const QVector3D pos(float(number(o, "x", t.position.x() * kMm) / kMm), float(number(o, "y", t.position.y() * kMm) / kMm),
                        float(number(o, "z", t.position.z() * kMm) / kMm));
    const QVector3D rot(float(number(o, "rx", t.rotation.x())), float(number(o, "ry", t.rotation.y())),
                        float(number(o, "rz", t.rotation.z())));
    return compose(pos, rot);
}

QVector3D columnScale(const QMatrix4x4& m) {
    return QVector3D(m.column(0).toVector3D().length(), m.column(1).toVector3D().length(), m.column(2).toVector3D().length());
}
bool uniform(const QVector3D& s) {
    const float lo = std::min({s.x(), s.y(), s.z()}), hi = std::max({s.x(), s.y(), s.z()});
    return hi - lo <= 1e-4f * std::max(1.0f, hi);
}

// A value that is an expression: a number is in API units (× scale to the
// automation's), a string is taken as written
QString exprParam(const QJsonObject& p, const QString& key, double scale = 1.0) {
    const QJsonValue v = p.value(key);
    if (v.isUndefined() || v.isNull()) return {};
    if (v.isDouble()) return QString::number(v.toDouble() * scale, 'g', 12);
    if (v.isBool()) return v.toBool() ? "1" : "0";
    if (v.isString()) return v.toString().trimmed();
    fail("bad_request", QString("\"%1\" must be a number or an expression").arg(key));
}

void checkSymbol(const SceneGraph& g, const QString& name, int exceptId = -1) {
    if (!expr::isValidName(name))
        fail("bad_request", QString("\"%1\" can't be used in expressions: use letters, digits and _, starting with a letter").arg(name));
    if (g.isSymbolNameTaken(name, exceptId))
        fail("bad_request", QString("the name \"%1\" is taken (signals, actuators, sequences, sources and sinks share names)").arg(name));
}

QString lowerFirst(QString s) { if (!s.isEmpty()) s[0] = s[0].toLower(); return s; }

SensorType sensorFromName(const QString& name) {
    const QString n = name.toLower();
    if (n == "beam")     return SensorType::Beam;
    if (n == "zone")     return SensorType::Zone;
    if (n == "distance") return SensorType::Distance;
    if (n == "position") return SensorType::Position;
    fail("bad_request", QString("there is no sensor type \"%1\"; types: beam, zone, distance, position").arg(name));
}

struct StepResult {
    bool ok = false;
    StepImporter::Part root;
    QString error;
};

} // namespace

// ── Helpers ──────────────────────────────────────────────────────────────────

void ApiServer::requireEditable() const {
    if (m_sim->isActive())
        fail("simulation_active", "the scene can't change while the simulation is active: call sim.reset first");
}

QMatrix4x4 ApiServer::frameParam(const QJsonObject& p, const QString& key, int parentId, const QString& fallback) const {
    QString f = fallback;
    if (given(p, key)) f = text(p, key);
    if (f == "world") return {};
    if (f == "parent" || f == "mount" || f == "self")
        return parentId == -1 ? QMatrix4x4() : kin::toolFrame(*m_scene, parentId);
    return kin::toolFrame(*m_scene, objectParam("frame", f));
}

int ApiServer::objectOrNull(const QJsonObject& p, const QString& key) const {
    if (!given(p, key)) return -1;
    return objectParam(key, text(p, key));
}

int ApiServer::jointParam(const QJsonObject& p, const QString& key) const {
    const QString name = text(p, key);
    std::vector<int> hits;
    for (const auto& [id, j] : m_scene->allJoints()) if (j.name == name) hits.push_back(id);
    if (hits.size() == 1) return hits.front();
    if (hits.size() > 1) fail("ambiguous", QString("%1 joints are called \"%2\"; name it by its actuator or moving object").arg(hits.size()).arg(name));
    for (const auto& [mid, m] : m_scene->allMotors())
        if (m.name == name && m_scene->getJoint(m.jointId)) return m.jointId;
    // The object it moves
    for (const auto& [id, o] : m_scene->allObjects())
        if (o.copyOf == -1 && (o.name == name || pathOf(*m_scene, id) == name))
            for (const auto& [jid, j] : m_scene->allJoints()) if (j.childBodyId == id) return jid;
    QStringList names;
    for (const auto& [id, j] : m_scene->allJoints()) names << j.name;
    fail("not_found", QString("there is no joint \"%1\"%2").arg(name, names.isEmpty() ? QString() : "; joints: " + listOf(names, 30)));
}

QJsonObject ApiServer::objectInfo(int id) const {
    const SceneObject* o = m_scene->getObject(id);
    QJsonArray children;
    for (int c : o->childIds)
        if (const SceneObject* co = m_scene->getObject(c); co && co->copyOf == -1) children.append(co->name);
    const QMatrix4x4 frame = kin::toolFrame(*m_scene, id);
    const QMatrix4x4 parent = o->parentId == -1 ? QMatrix4x4() : kin::toolFrame(*m_scene, o->parentId);
    QJsonObject info{{"name", o->name}, {"path", pathOf(*m_scene, id)}, {"type", typeName(*o)},
                     {"parent", o->parentId == -1 ? QJsonValue() : QJsonValue(pathOf(*m_scene, o->parentId))},
                     {"children", children},
                     {"position", poseJson(parent.inverted() * frame)},
                     {"world", poseJson(frame)},
                     {"color", o->color.isValid() ? QJsonValue(o->color.name()) : QJsonValue()},
                     {"opacity", round6(o->opacity)},
                     {"hidden", o->hidden}};
    if (o->type != PrimitiveType::ImportedMesh) info["size"] = vecJson(columnScale(m_scene->worldTransform(id)));
    for (const auto& [jid, j] : m_scene->allJoints())
        if (j.childBodyId == id) info["joint"] = j.name;
    if (const RigidBody* b = const_cast<SceneGraph*>(m_scene)->getRigidBody(id))
        info["body"] = QJsonObject{{"mass", round6(b->mass)}, {"gravity", b->gravityEnabled}, {"collision", b->hasCollision}};
    if (const ConveyorBelt* b = const_cast<SceneGraph*>(m_scene)->getConveyorBelt(id)) {
        QJsonObject belt{{"speed", roundMm(b->velocity * kMm)}, {"direction", vecJson(b->direction, 1)}, {"enabled", b->enabled}};
        if (!b->speedExpr.isEmpty()) belt["speed"] = b->speedExpr;
        info["belt"] = belt;
    }
    return info;
}

void ApiServer::applyObject(int id, const QJsonObject& p, bool creating) {
    SceneObject* o = m_scene->getObject(id);
    if (given(p, "name")) o->name = text(p, "name");
    if (!creating && p.contains("parent")) {
        const int parent = objectOrNull(p, "parent");
        if (parent != o->parentId) {
            if (m_scene->setParent(id, parent) != SceneGraph::ReparentResult::Ok)
                fail("bad_request", QString("%1 can't go under %2: it is %2 or holds it").arg(o->name, m_scene->getObject(parent)->name));
            o = m_scene->getObject(id);
        }
    }
    if (p.contains("color")) {
        const QJsonValue v = p.value("color");
        if (v.isNull()) o->color = QColor();
        else {
            const QColor c(v.toString());
            if (!v.isString() || !c.isValid()) fail("bad_request", "\"color\" must be \"#rrggbb\" (or null to take the parent's)");
            o->color = c;
        }
    }
    if (given(p, "opacity")) o->opacity = float(std::clamp(number(p, "opacity", 1), 0.0, 1.0));
    if (given(p, "hidden")) o->hidden = flag(p, "hidden", false);

    const bool sized = given(p, "size") || given(p, "diameter") || given(p, "height");
    if (sized && o->type == PrimitiveType::ImportedMesh)
        fail("bad_request", QString("%1 is a %2: only boxes, cylinders, spheres, capsules and baskets have a size").arg(o->name, typeName(*o)));
    if (given(p, "position") || sized) {
        const QMatrix4x4 frame = frameParam(p, "frame", o->parentId, "parent");
        const QMatrix4x4 now = kin::toolFrame(*m_scene, id);
        const QMatrix4x4 target = given(p, "position") ? frame * readPose(p, "position", frame.inverted() * now) : now;
        QVector3D scale = columnScale(m_scene->worldTransform(id));
        if (given(p, "size")) {
            const QJsonValue v = p.value("size");
            scale = v.isDouble() ? QVector3D(1, 1, 1) * float(v.toDouble()) : readVec(p, "size");
            scale /= float(kMm);
        }
        if (given(p, "diameter")) { const float d = float(number(p, "diameter", 0) / kMm); scale.setX(d); scale.setZ(d); }
        if (given(p, "height")) scale.setY(float(number(p, "height", 0) / kMm));
        if (scale.x() <= 0 || scale.y() <= 0 || scale.z() <= 0) fail("bad_request", "sizes must be more than 0 mm");
        QMatrix4x4 world = target;
        world.scale(scale);
        m_scene->setWorldTransform(id, world);
    }
    emit m_scene->objectChanged(id);
}

QJsonObject ApiServer::jointInfo(int jid, const QMatrix4x4& frame) const {
    const Joint* j = m_scene->getJoint(jid);
    const QMatrix4x4 parent = m_scene->worldTransform(j->parentBodyId);
    const QMatrix4x4 inv = frame.inverted();
    const bool prismatic = j->type == JointType::Prismatic;
    QJsonObject o{{"name", j->name}, {"type", prismatic ? "prismatic" : "revolute"},
                  {"unit", prismatic ? "mm" : "deg"},
                  {"parent", pathOf(*m_scene, j->parentBodyId)}, {"child", pathOf(*m_scene, j->childBodyId)},
                  {"axis", vecJson(inv.mapVector(parent.mapVector(j->axis)).normalized(), 1)},
                  {"value", toApi(*j, j->currentValue)}};
    if (!prismatic) o["pivot"] = vecJson(inv.map(parent.map(j->pivot)));
    if (!prismatic && j->continuous) o["continuous"] = true;
    else { o["min"] = toApi(*j, j->limitMin); o["max"] = toApi(*j, j->limitMax); }
    if (const Joint* m = m_scene->getJoint(j->coupledTo))
        o["follows"] = QJsonObject{{"joint", m->name}, {"ratio", round6(j->ratio)}, {"offset", toApi(*j, j->offset)}};
    if (const Motor* m = m_scene->motorForJoint(jid)) {
        QJsonObject a{{"name", m->name}, {"mode", m->mode == ActuatorMode::Speed ? "speed" : "position"},
                      {"speed", toApi(*j, m->velocity)}, {"acceleration", toApi(*j, m->acceleration)},
                      {"enabled", m->enabled}};
        if (!m->targetExpr.isEmpty())   a["target"] = m->targetExpr;
        if (!m->velocityExpr.isEmpty()) a["speed"] = m->velocityExpr;
        if (!m->activeExpr.isEmpty())   a["active"] = m->activeExpr;
        o["actuator"] = a;
    } else {
        o["actuator"] = QJsonValue();
    }
    return o;
}

void ApiServer::applyJoint(int jid, const QJsonObject& p, bool creating) {
    Joint* j = m_scene->getJoint(jid);
    const bool prismatic = j->type == JointType::Prismatic;
    const QMatrix4x4 parentWorld = m_scene->worldTransform(j->parentBodyId);
    const QMatrix4x4 toParent = parentWorld.inverted();
    if (given(p, "name")) j->name = text(p, "name");
    if (given(p, "axis")) {
        const QMatrix4x4 frame = frameParam(p, "frame", -1, "world");
        const QVector3D world = frame.mapVector(readVec(p, "axis")).normalized();
        if (world.isNull()) fail("bad_request", "\"axis\" must not be 0");
        if (!prismatic && !uniform(columnScale(parentWorld)))
            fail("bad_request", QString("%1 is stretched (a sized box?): a revolute joint needs a parent frame that isn't; "
                                        "add a frame (object.add type \"frame\") and hang both on it").arg(m_scene->getObject(j->parentBodyId)->name));
        QVector3D local = toParent.mapVector(world);
        if (!prismatic) local.normalize();   // prismatic: 1 m of joint value moves 1 m in the world
        j->axis = local;
    }
    if (given(p, "pivot")) {
        const QMatrix4x4 frame = frameParam(p, "frame", -1, "world");
        j->pivot = toParent.map(frame.map(readVec(p, "pivot") / float(kMm)));
    }
    if (given(p, "min")) j->limitMin = float(fromApi(*j, number(p, "min", 0)));
    if (given(p, "max")) j->limitMax = float(fromApi(*j, number(p, "max", 0)));
    if (j->limitMin > j->limitMax) std::swap(j->limitMin, j->limitMax);
    if (given(p, "continuous")) {
        if (prismatic && flag(p, "continuous", false)) fail("bad_request", "only revolute joints can be continuous");
        j->continuous = flag(p, "continuous", false);
    }
    if (p.contains("follows")) {
        const QJsonValue f = p.value("follows");
        if (f.isNull() || (f.isBool() && !f.toBool())) j->coupledTo = -1;
        else {
            QJsonObject fo = f.isString() ? QJsonObject{{"joint", f.toString()}} : f.toObject();
            if (!f.isString() && !f.isObject()) fail("bad_request", "\"follows\" must be a joint name or {joint, ratio, offset}");
            const int master = jointParam(fo, "joint");
            if (master == jid) fail("bad_request", "a joint can't follow itself");
            j = m_scene->getJoint(jid);
            j->coupledTo = master;
            j->ratio = float(number(fo, "ratio", 1));
            j->offset = float(fromApi(*j, number(fo, "offset", 0)));
            if (const Motor* m = m_scene->motorForJoint(jid)) m_scene->removeMotor(m->id);   // a follower has no actuator
        }
    }
    j = m_scene->getJoint(jid);
    if (given(p, "value")) {
        j->currentValue = float(fromApi(*j, number(p, "value", 0)));
        j->initialAngle = j->currentValue;
    }

    // The actuator: true / false / {name, mode, speed, acceleration, target, active, enabled}
    QJsonValue a = p.value("actuator");
    if (creating && (a.isUndefined() || a.isNull())) a = j->coupledTo == -1;
    if (a.isBool() && !a.toBool()) {
        if (const Motor* m = m_scene->motorForJoint(jid)) m_scene->removeMotor(m->id);
    } else if (a.isBool() || a.isObject()) {
        if (j->coupledTo != -1) fail("bad_request", QString("%1 follows another joint, so it can't have an actuator").arg(j->name));
        const QJsonObject ao = a.toObject();
        Motor* m = m_scene->motorForJoint(jid);
        if (!m) {
            const QString name = given(ao, "name") ? text(ao, "name") : m_scene->makeSymbolName(j->name);
            checkSymbol(*m_scene, name);
            m = m_scene->getMotor(m_scene->addActuatorForJoint(jid));
            m->name = name;
        } else if (given(ao, "name")) {
            const QString name = text(ao, "name");
            if (name != m->name) {
                checkSymbol(*m_scene, name, m->id);
                const QString old = m->name;
                m->name = name;
                m_scene->renameSymbolReferences(old, name);
                m = m_scene->motorForJoint(jid);
            }
        }
        j = m_scene->getJoint(jid);
        if (given(ao, "mode")) {
            const QString mode = text(ao, "mode").toLower();
            if (mode != "position" && mode != "speed") fail("bad_request", "\"mode\" is \"position\" or \"speed\"");
            m->mode = mode == "speed" ? ActuatorMode::Speed : ActuatorMode::Position;
        }
        if (given(ao, "speed")) {
            if (ao.value("speed").isString()) m->velocityExpr = exprParam(ao, "speed");
            else { m->velocity = float(fromApi(*j, number(ao, "speed", 0))); m->velocityExpr.clear(); }
        }
        if (given(ao, "acceleration")) m->acceleration = float(fromApi(*j, number(ao, "acceleration", 0)));
        if (ao.contains("target")) m->targetExpr = exprParam(ao, "target", prismatic ? 1 / kMm : 1);
        if (ao.contains("active")) {
            if (ao.value("active").isBool()) { m->enabled = ao.value("active").toBool(); m->activeExpr.clear(); }
            else m->activeExpr = exprParam(ao, "active");
        }
        if (given(ao, "enabled")) m->enabled = flag(ao, "enabled", true);
    } else if (!a.isUndefined() && !a.isNull()) {
        fail("bad_request", "\"actuator\" must be true, false or {name, mode, speed, acceleration, target, active}");
    }
    m_scene->notifyJointsMoved();
    emit m_scene->objectChanged(m_scene->getJoint(jid)->childBodyId);
    m_scene->notifyAutomationChanged();
}

void ApiServer::readSensor(SensorConfig& s, const QJsonObject& p, bool zoneOnly) {
    s.objectId = objectOrNull(p, "mount");
    if (zoneOnly) s.type = SensorType::Zone;
    else s.type = sensorFromName(text(p, "type"));
    const QMatrix4x4 mount = s.objectId == -1 ? QMatrix4x4() : kin::toolFrame(*m_scene, s.objectId);
    const QMatrix4x4 toMount = mount.inverted() * frameParam(p, "frame", s.objectId, "mount");
    if (given(p, "at")) s.offset = toMount.map(readVec(p, "at") / float(kMm));
    const bool ray = sensorIsRay(s.type);
    if (ray) {
        if (given(p, "direction")) {
            s.direction = toMount.mapVector(readVec(p, "direction")).normalized();
            if (s.direction.isNull()) fail("bad_request", "\"direction\" must not be 0");
        }
        if (given(p, "range")) s.range = float(number(p, "range", 500) / kMm);
        if (s.range <= 0) fail("bad_request", "\"range\" must be more than 0 mm");
    } else {
        s.objectBounds = flag(p, "bounds", false);
        if (s.objectBounds && s.objectId == -1) fail("bad_request", "\"bounds\" needs a \"mount\" whose box is the zone");
        if (given(p, "size")) {
            // Turned into the mount's frame: exact for boxes turned by right angles
            const QVector3D size = readVec(p, "size") / float(kMm);
            QVector3D out;
            for (int r = 0; r < 3; ++r)
                out[r] = std::fabs(toMount(r, 0)) * size.x() + std::fabs(toMount(r, 1)) * size.y() + std::fabs(toMount(r, 2)) * size.z();
            s.size = out;
        } else if (!s.objectBounds && !given(p, "size")) {
            fail("bad_request", "a zone needs a \"size\" {x, y, z} in mm, or \"bounds\": true for its mount's box");
        }
        if (s.size.x() <= 0 || s.size.y() <= 0 || s.size.z() <= 0) fail("bad_request", "zone sizes must be more than 0 mm");
    }
    const QString detects = given(p, "detects") ? text(p, "detects") : QString("any");
    if (detects == "any") s.target = SensorTarget::Any;
    else if (detects == "bodies") s.target = SensorTarget::RigidBodies;
    else { s.target = SensorTarget::Object; s.targetId = objectParam("object", detects); }
}

QJsonObject ApiServer::sensorJson(const SensorConfig& s) const {
    QJsonObject o{{"type", sensorTypeName(s.type).toLower()},
                  {"mount", s.objectId == -1 ? QJsonValue() : QJsonValue(pathOf(*m_scene, s.objectId))},
                  {"at", vecJson(s.offset)}};
    if (sensorIsRay(s.type)) { o["direction"] = vecJson(s.direction, 1); o["range"] = roundMm(s.range * kMm); }
    else if (s.objectBounds) o["bounds"] = true;
    else o["size"] = vecJson(s.size);
    o["detects"] = s.target == SensorTarget::Any ? QString("any")
                 : s.target == SensorTarget::RigidBodies ? QString("bodies")
                 : (m_scene->getObject(s.targetId) ? pathOf(*m_scene, s.targetId) : QString());
    return o;
}

void ApiServer::answerLater(QTcpSocket* socket, const QJsonValue& id, const QJsonObject& reply) {
    QJsonObject msg = reply;
    msg["id"] = id;
    send(socket, msg);
}

// ── Projects ─────────────────────────────────────────────────────────────────

namespace {
void checkDiscard(const QJsonObject& p, const std::function<bool()>& unsaved) {
    if (unsaved && unsaved() && !flag(p, "discard", false))
        fail("unsaved", "the open project has unsaved changes: save them (scene.save) or pass \"discard\": true");
}
} // namespace

QJsonValue ApiServer::sceneNew(const QJsonObject& p, Call&) {
    checkDiscard(p, m_unsaved);
    m_sim->reset();
    m_scene->clear();
    m_projectPath.clear();
    emit projectReplaced({});
    return QJsonObject{{"path", QJsonValue()}};
}

QJsonValue ApiServer::sceneOpen(const QJsonObject& p, Call& c) {
    const QString path = text(p, "path");
    if (!QFileInfo(path).isFile()) fail("not_found", QString("there is no file %1").arg(path));
    checkDiscard(p, m_unsaved);
    {
        SceneGraph probe;   // the scene stays as it is when the file can't be read
        if (!ProjectSerializer::load(probe, path)) fail("bad_request", QString("%1 is not a MotionStudio project").arg(path));
    }
    m_sim->reset();
    ProjectSerializer::load(*m_scene, path);
    m_projectPath = QFileInfo(path).absoluteFilePath();
    emit projectReplaced(m_projectPath);
    return sceneInfo(p, c);
}

QJsonValue ApiServer::sceneSave(const QJsonObject& p, Call&) {
    requireEditable();
    QString path = text(p, "path", false);
    if (path.isEmpty()) path = m_projectPath;
    if (path.isEmpty()) fail("bad_request", "the scene has no file yet: give a \"path\"");
    if (!ProjectSerializer::save(*m_scene, path)) fail("bad_request", QString("can't write %1").arg(path));
    m_projectPath = QFileInfo(path).absoluteFilePath();
    emit projectSaved(m_projectPath);
    return QJsonObject{{"path", m_projectPath}};
}

QJsonValue ApiServer::sceneInfo(const QJsonObject&, Call&) {
    int objects = 0;
    for (const auto& [id, o] : m_scene->allObjects()) if (o.copyOf == -1) ++objects;
    QJsonArray roots, joints, sigs, sensors, sequences, poses, sources, sinks, robots, motions;
    for (int r : m_scene->rootObjects())
        if (const SceneObject* o = m_scene->getObject(r); o && o->copyOf == -1) roots.append(o->name);
    std::vector<std::pair<QString, int>> js;
    for (const auto& [id, j] : m_scene->allJoints()) js.push_back({j.name, id});
    std::sort(js.begin(), js.end());
    for (auto& [name, id] : js) {
        const Motor* m = m_scene->motorForJoint(id);
        joints.append(QJsonObject{{"name", name}, {"actuator", m ? QJsonValue(m->name) : QJsonValue()}});
    }
    for (const auto& [id, s] : m_scene->allSignals()) (s.kind == SignalKind::Sensor ? sensors : sigs).append(s.name);
    for (const auto& [id, q] : m_scene->allSequences()) sequences.append(q.name);
    for (const auto& [id, q] : m_scene->allPoses()) poses.append(q.name);
    for (const auto& [id, q] : m_scene->allSources()) sources.append(q.name);
    for (const auto& [id, q] : m_scene->allSinks()) sinks.append(q.name);
    for (const auto& [id, q] : m_scene->allMotions()) motions.append(q.name);
    for (const auto& [id, q] : m_scene->allLearned()) motions.append(q.name);
    for (const Robot& r : findRobots(*m_scene)) robots.append(r.path);
    return QJsonObject{{"path", m_projectPath.isEmpty() ? QJsonValue() : QJsonValue(m_projectPath)},
                       {"objects", objects}, {"top", roots}, {"robots", robots}, {"joints", joints},
                       {"signals", sigs}, {"sensors", sensors}, {"sequences", sequences}, {"poses", poses},
                       {"sources", sources}, {"sinks", sinks}, {"motions", motions}};
}

QJsonValue ApiServer::sceneCheck(const QJsonObject&, Call&) {
    const QStringList problems = AutomationEngine::validateAll(*m_scene);
    return QJsonObject{{"ok", problems.isEmpty()}, {"problems", QJsonArray::fromStringList(problems)}};
}

QJsonValue ApiServer::sceneObjects(const QJsonObject& p, Call&) {
    const int under = objectOrNull(p, "under");
    const QString pattern = text(p, "name", false);
    const QRegularExpression re(pattern.isEmpty() ? QString(".*") : QRegularExpression::wildcardToRegularExpression(pattern));
    const QString type = text(p, "type", false).toLower();
    const int maxDepth = int(number(p, "depth", -1));   // levels below `under` (or the top); -1 = all
    QJsonArray out;
    std::function<void(int, int)> walk = [&](int id, int depth) {
        const SceneObject* o = m_scene->getObject(id);
        if (!o || o->copyOf != -1) return;
        if (depth >= 1 && re.match(o->name).hasMatch() && (type.isEmpty() || typeName(*o) == type))
            out.append(QJsonObject{{"name", o->name}, {"path", pathOf(*m_scene, id)}, {"type", typeName(*o)},
                                   {"parent", o->parentId == -1 ? QJsonValue() : QJsonValue(pathOf(*m_scene, o->parentId))}});
        if (maxDepth >= 0 && depth >= maxDepth) return;
        for (int c : o->childIds) walk(c, depth + 1);
    };
    if (under != -1) walk(under, 0);
    else for (int r : m_scene->rootObjects()) walk(r, 1);
    return out;
}

QJsonValue ApiServer::sceneImport(const QJsonObject& p, Call& c) {
    requireEditable();
    const QString path = text(p, "path");
    if (!QFileInfo(path).isFile()) fail("not_found", QString("there is no file %1").arg(path));
    const int parent = objectOrNull(p, "parent");
    const QString suffix = QFileInfo(path).suffix().toLower();
    const size_t before = m_scene->allObjects().size();
    auto finish = [this, p, parent, before](const std::vector<int>& roots, const QStringList& warnings) {
        if (roots.empty()) fail("bad_request", "nothing was imported");
        if (parent != -1)
            for (int r : roots) m_scene->setParent(r, parent);
        if (given(p, "name")) {
            m_scene->getObject(roots.front())->name = text(p, "name");
            emit m_scene->objectChanged(roots.front());
        }
        QJsonArray paths;
        for (int r : roots) paths.append(pathOf(*m_scene, r));
        emit imported(roots.front());
        return QJsonObject{{"root", objectInfo(roots.front())}, {"roots", paths},
                           {"objects", int(m_scene->allObjects().size() - before)},
                           {"warnings", QJsonArray::fromStringList(warnings)}};
    };

    if (suffix == "step" || suffix == "stp") {
        if (!StepImporter::available()) fail("bad_request", "this MotionStudio was built without STEP import (OpenCASCADE)");
        const QString up = given(p, "upAxis") ? text(p, "upAxis").toLower() : QString("z");
        if (up != "z" && up != "y") fail("bad_request", "\"upAxis\" is \"z\" (most CAD) or \"y\"");
        // Reading a big file takes a while: on a worker, answered when done
        c.async = true;
        QPointer<QTcpSocket> socket = c.socket;
        const QJsonValue id = c.id;
        auto* watcher = new QFutureWatcher<StepResult>(this);
        connect(watcher, &QFutureWatcher<StepResult>::finished, this, [=, this] {
            StepResult r = watcher->result();
            watcher->deleteLater();
            QJsonObject reply;
            try {
                if (!r.ok) fail("bad_request", QString("could not import %1: %2").arg(QFileInfo(path).fileName(), r.error));
                requireEditable();
                if (parent != -1 && !m_scene->getObject(parent)) fail("not_found", "the parent was removed while importing");
                StepImporter::applyUpAxis(r.root, up == "z" ? StepImporter::UpAxis::Z : StepImporter::UpAxis::Y);
                const int root = StepImporter::addToScene(*m_scene, r.root, -1);
                reply["result"] = finish({root}, {});
            } catch (const ApiError& e) {
                reply["error"] = QJsonObject{{"code", e.code}, {"message", e.message}};
            }
            if (socket) answerLater(socket, id, reply);
        });
        watcher->setFuture(QtConcurrent::run([path] {
            StepResult r;
            r.ok = StepImporter::importFile(path, r.root, r.error);
            return r;
        }));
        return {};
    }
    if (suffix == "urdf" || suffix == "xml" || suffix == "mjcf") {
        bool mjcf = suffix == "mjcf";
        if (suffix == "xml") {
            QFile f(path);
            if (f.open(QIODevice::ReadOnly)) mjcf = f.read(4096).contains("<mujoco");
        }
        const robotio::Result r = mjcf ? robotio::importMjcf(*m_scene, path) : robotio::importUrdf(*m_scene, path);
        if (!r.ok) fail("bad_request", QString("could not import %1: %2").arg(QFileInfo(path).fileName(), r.error));
        return finish({r.rootId}, r.warnings);
    }
    if (suffix == "mstudio" || suffix == "json") {
        SceneGraph component;
        if (!ProjectSerializer::load(component, path)) fail("bad_request", QString("%1 is not a MotionStudio project").arg(path));
        return finish(mergeScene(*m_scene, component, -1), {});
    }
    fail("bad_request", QString("can't import .%1 files: STEP (.step, .stp), URDF (.urdf), MJCF (.xml) and projects (.mstudio)").arg(suffix));
}

// ── Objects ──────────────────────────────────────────────────────────────────

QJsonValue ApiServer::objectAdd(const QJsonObject& p, Call&) {
    requireEditable();
    const PrimitiveType type = typeFromName(given(p, "type") ? text(p, "type") : QString("frame"));
    const int parent = objectOrNull(p, "parent");
    static const QMap<PrimitiveType, QString> names{{PrimitiveType::ImportedMesh, "Frame"}, {PrimitiveType::Cube, "Box"},
        {PrimitiveType::Cylinder, "Cylinder"}, {PrimitiveType::Sphere, "Sphere"}, {PrimitiveType::Capsule, "Capsule"},
        {PrimitiveType::Basket, "Basket"}};
    const int id = m_scene->addObject(given(p, "name") ? text(p, "name") : names.value(type), type, parent);
    try {
        QJsonObject q = p;
        if (type != PrimitiveType::ImportedMesh && !given(q, "size") && !given(q, "diameter") && !given(q, "height"))
            q["size"] = 100;   // a 100 mm part unless told
        if (!given(q, "position")) q["position"] = QJsonObject{};
        SceneObject* o = m_scene->getObject(id);
        o->transform = Transform{};
        if (type != PrimitiveType::ImportedMesh) o->transform.scale = QVector3D(0.1f, 0.1f, 0.1f);
        applyObject(id, q, true);
    } catch (...) {
        m_scene->removeObject(id);
        throw;
    }
    return objectInfo(id);
}

QJsonValue ApiServer::objectGet(const QJsonObject& p, Call&) {
    return objectInfo(objectParam("object", text(p, "object")));
}

QJsonValue ApiServer::objectSet(const QJsonObject& p, Call&) {
    requireEditable();
    const int id = objectParam("object", text(p, "object"));
    applyObject(id, p, false);
    return objectInfo(id);
}

QJsonValue ApiServer::objectRemove(const QJsonObject& p, Call&) {
    requireEditable();
    const int id = objectParam("object", text(p, "object"));
    std::vector<int> sub;
    collect(*m_scene, id, sub);
    m_scene->removeObject(id);
    m_scene->notifyAutomationChanged();
    return QJsonObject{{"removed", int(sub.size())}};
}

QJsonValue ApiServer::objectGroup(const QJsonObject& p, Call&) {
    requireEditable();
    const QJsonValue v = p.value("objects");
    if (!v.isArray() || v.toArray().isEmpty()) fail("bad_request", "\"objects\" must be a list of object names");
    std::vector<int> ids;
    for (const QJsonValue& e : v.toArray()) {
        if (!e.isString()) fail("bad_request", "\"objects\" must be a list of object names");
        ids.push_back(objectParam("object", e.toString()));
    }
    const int group = m_scene->groupObjects(ids, given(p, "name") ? text(p, "name") : QString("Group"));
    if (group == -1) fail("bad_request", "those objects can't be grouped");
    return objectInfo(group);
}

QJsonValue ApiServer::objectDuplicate(const QJsonObject& p, Call&) {
    requireEditable();
    const int id = objectParam("object", text(p, "object"));
    const int copy = m_scene->duplicateObject(id);
    if (copy == -1) fail("bad_request", "it can't be copied");
    QJsonObject rest = p;
    rest.remove("object");
    applyObject(copy, rest, false);
    return objectInfo(copy);
}

QJsonValue ApiServer::objectBounds(const QJsonObject& p, Call&) {
    const int id = objectParam("object", text(p, "object"));
    const QMatrix4x4 frame = frameParam(p, "frame", -1, "world").inverted();
    const bool children = flag(p, "children", true);
    std::vector<int> ids;
    if (children) collect(*m_scene, id, ids); else ids.push_back(id);
    QVector3D lo(1e30f, 1e30f, 1e30f), hi(-1e30f, -1e30f, -1e30f);
    bool any = false;
    auto add = [&](const QVector3D& v) { lo = QVector3D(std::min(lo.x(), v.x()), std::min(lo.y(), v.y()), std::min(lo.z(), v.z()));
                                         hi = QVector3D(std::max(hi.x(), v.x()), std::max(hi.y(), v.y()), std::max(hi.z(), v.z())); any = true; };
    for (int oid : ids) {
        const SceneObject* o = m_scene->getObject(oid);
        if (!o || o->copyOf != -1 || o->parked) continue;
        const QMatrix4x4 m = frame * m_scene->worldTransform(oid);
        if (o->type == PrimitiveType::ImportedMesh) {
            if (!o->mesh) continue;
            const auto& v = o->mesh->interleaved;
            for (size_t i = 0; i + 5 < v.size(); i += 6) add(m.map(QVector3D(v[i], v[i + 1], v[i + 2])));
        } else {
            for (int k = 0; k < 8; ++k)
                add(m.map(QVector3D(k & 1 ? 0.5f : -0.5f, k & 2 ? 0.5f : -0.5f, k & 4 ? 0.5f : -0.5f)));
        }
    }
    if (!any) fail("bad_request", QString("%1 has no geometry%2").arg(m_scene->getObject(id)->name, children ? QString() : QString(" of its own")));
    return QJsonObject{{"min", vecJson(lo)}, {"max", vecJson(hi)}, {"center", vecJson((lo + hi) / 2)}, {"size", vecJson(hi - lo)}};
}

QJsonValue ApiServer::objectFeatures(const QJsonObject& p, Call&) {
    const int id = objectParam("object", text(p, "object"));
    const QMatrix4x4 frame = frameParam(p, "frame", -1, "world").inverted();
    std::set<QString> kinds{"circle", "cylinder"};
    if (given(p, "kinds")) {
        kinds.clear();
        for (const QJsonValue& k : p.value("kinds").toArray()) {
            const QString s = k.toString().toLower();
            if (s != "circle" && s != "cylinder" && s != "line" && s != "plane")
                fail("bad_request", "\"kinds\" lists circle, cylinder, line or plane");
            kinds.insert(s);
        }
    }
    const double minR = number(p, "minRadius", 0), maxR = number(p, "maxRadius", 1e12);
    const int limit = int(number(p, "limit", 500));
    std::vector<int> ids;
    if (flag(p, "children", true)) collect(*m_scene, id, ids); else ids.push_back(id);
    QJsonArray out;
    bool truncated = false;
    for (int oid : ids) {
        const SceneObject* o = m_scene->getObject(oid);
        if (!o || !o->mesh || o->copyOf != -1) continue;
        const QMatrix4x4 m = frame * m_scene->worldTransform(oid);
        const MeshFeatures f(o->mesh->interleaved);
        auto emitFeature = [&](const AxisFeature& a) {
            QString kind;
            switch (a.kind) {
                case AxisFeature::Kind::CircularEdge:    kind = "circle"; break;
                case AxisFeature::Kind::CylindricalFace: kind = "cylinder"; break;
                case AxisFeature::Kind::StraightEdge:    kind = "line"; break;
                case AxisFeature::Kind::FlatFace:        kind = "plane"; break;
                default: return;
            }
            if (!kinds.count(kind)) return;
            const QVector3D axis = m.mapVector(a.axis).normalized();
            QJsonObject e{{"object", pathOf(*m_scene, oid)}, {"kind", kind},
                          {"origin", vecJson(m.map(a.origin))}, {"axis", vecJson(axis, 1)}};
            if (kind == "circle" || kind == "cylinder") {
                QVector3D perp = QVector3D::crossProduct(a.axis, QVector3D(1, 0, 0));
                if (perp.length() < 0.1f) perp = QVector3D::crossProduct(a.axis, QVector3D(0, 1, 0));
                const double r = a.radius * m.mapVector(perp.normalized()).length() * kMm;
                if (r < minR || r > maxR) return;
                e["radius"] = roundMm(r);
            }
            if (kind == "line" || kind == "cylinder") {
                // Length along the axis, from the outline
                float lo = 1e30f, hi = -1e30f;
                for (const QVector3D& q : a.outline) {
                    const float t = QVector3D::dotProduct(m.map(q) - m.map(a.origin), axis);
                    lo = std::min(lo, t); hi = std::max(hi, t);
                }
                if (hi >= lo) e["length"] = roundMm((hi - lo) * kMm);
            }
            if (out.size() >= limit) { truncated = true; return; }
            out.append(e);
        };
        for (const AxisFeature& a : f.faces()) emitFeature(a);
        for (const AxisFeature& a : f.edges()) emitFeature(a);
    }
    return QJsonObject{{"features", out}, {"truncated", truncated}};
}

// ── Joints ───────────────────────────────────────────────────────────────────

QJsonValue ApiServer::jointAdd(const QJsonObject& p, Call&) {
    requireEditable();
    const int parent = objectParam("parent", text(p, "parent"));
    const int child = objectParam("child", text(p, "child"));
    const QString type = given(p, "type") ? text(p, "type").toLower() : QString("revolute");
    if (type != "revolute" && type != "prismatic") fail("bad_request", "\"type\" is \"revolute\" or \"prismatic\"");
    if (!given(p, "axis")) fail("bad_request", "give the joint's \"axis\" {x, y, z}");
    if (type == "prismatic" && (!given(p, "min") || !given(p, "max")))
        fail("bad_request", "a prismatic joint needs \"min\" and \"max\" (mm)");
    for (const auto& [jid, j] : m_scene->allJoints())
        if (j.childBodyId == child) fail("bad_request", QString("%1 already moves on the joint %2").arg(m_scene->getObject(child)->name, j.name));
    if (inSubtree(*m_scene, child, parent)) fail("bad_request", "the parent can't be inside the child");
    // The joint's parent is the child's parent in the tree
    if (m_scene->getObject(child)->parentId != parent) m_scene->setParent(child, parent);

    Joint j;
    j.type = type == "prismatic" ? JointType::Prismatic : JointType::Revolute;
    j.name = given(p, "name") ? text(p, "name") : m_scene->getObject(child)->name;
    j.parentBodyId = parent;
    j.childBodyId = child;
    // Rotates about the child's origin unless told
    j.pivot = m_scene->worldTransform(parent).inverted().map(m_scene->worldTransform(child).column(3).toVector3D());
    if (j.type == JointType::Prismatic) { j.limitMin = 0; j.limitMax = 0; }
    const int jid = m_scene->addJoint(j);
    try {
        QJsonObject q = p;
        if (!given(q, "actuator") && !given(q, "follows")) {
            QJsonObject a;
            if (given(p, "name")) a["name"] = m_scene->makeSymbolName(text(p, "name"));
            q["actuator"] = a;
        }
        applyJoint(jid, q, true);
    } catch (...) {
        m_scene->removeJoint(jid);
        emit m_scene->objectChanged(child);
        throw;
    }
    return jointInfo(jid, frameParam(p, "frame", -1, "world"));
}

QJsonValue ApiServer::jointGet(const QJsonObject& p, Call&) {
    return jointInfo(jointParam(p, "joint"), frameParam(p, "frame", -1, "world"));
}

QJsonValue ApiServer::jointSet(const QJsonObject& p, Call&) {
    requireEditable();
    const int jid = jointParam(p, "joint");
    applyJoint(jid, p, false);
    return jointInfo(jid, frameParam(p, "frame", -1, "world"));
}

QJsonValue ApiServer::jointRemove(const QJsonObject& p, Call&) {
    requireEditable();
    const int jid = jointParam(p, "joint");
    const int child = m_scene->getJoint(jid)->childBodyId;
    // The child stays where it is now
    const QMatrix4x4 world = m_scene->worldTransform(child);
    m_scene->removeJoint(jid);
    m_scene->setWorldTransform(child, world);
    emit m_scene->objectChanged(child);
    m_scene->notifyAutomationChanged();
    return QJsonObject{{"removed", true}};
}

// ── Physics ──────────────────────────────────────────────────────────────────

QJsonValue ApiServer::bodySet(const QJsonObject& p, Call&) {
    requireEditable();
    const int id = objectParam("object", text(p, "object"));
    if (flag(p, "remove", false)) {
        m_scene->removeRigidBody(id);
        return objectInfo(id);
    }
    m_scene->addRigidBody(id);
    RigidBody* b = m_scene->getRigidBody(id);
    if (given(p, "mass")) {
        const double kg = number(p, "mass", 1);
        if (kg <= 0) fail("bad_request", "\"mass\" must be more than 0 kg");
        b->mass = float(kg);
    }
    if (given(p, "gravity")) b->gravityEnabled = flag(p, "gravity", true);
    if (given(p, "collision")) b->hasCollision = flag(p, "collision", false);
    emit m_scene->rigidBodyChanged(id);
    return objectInfo(id);
}

QJsonValue ApiServer::beltSet(const QJsonObject& p, Call&) {
    requireEditable();
    const int id = objectParam("object", text(p, "object"));
    if (flag(p, "remove", false)) {
        m_scene->removeConveyorBelt(id);
        return objectInfo(id);
    }
    m_scene->addConveyorBelt(id);
    ConveyorBelt* b = m_scene->getConveyorBelt(id);
    if (given(p, "speed")) {
        if (p.value("speed").isString()) b->speedExpr = exprParam(p, "speed");
        else { b->velocity = float(number(p, "speed", 0) / kMm); b->speedExpr.clear(); }
    }
    if (given(p, "direction")) {
        const QMatrix4x4 toBelt = kin::toolFrame(*m_scene, id).inverted() * frameParam(p, "frame", id, "self");
        const QVector3D d = toBelt.mapVector(readVec(p, "direction")).normalized();
        if (d.isNull()) fail("bad_request", "\"direction\" must not be 0");
        b->direction = d;
    }
    if (given(p, "enabled")) b->enabled = flag(p, "enabled", true);
    emit m_scene->conveyorBeltChanged(id);
    return objectInfo(id);
}

// ── Signals, sensors, poses ──────────────────────────────────────────────────

QJsonValue ApiServer::signalAdd(const QJsonObject& p, Call&) {
    requireEditable();
    Signal s;
    s.name = text(p, "name");
    checkSymbol(*m_scene, s.name);
    const QString kind = given(p, "kind") ? text(p, "kind").toLower() : QString("internal");
    if (kind == "input") s.kind = SignalKind::Input;
    else if (kind == "internal") s.kind = SignalKind::Internal;
    else if (kind == "derived") s.kind = SignalKind::Derived;
    else fail("bad_request", "\"kind\" is \"input\", \"internal\" or \"derived\" (sensors: sensor.add)");
    const QString type = given(p, "type") ? text(p, "type").toLower() : QString("bool");
    if (type == "bool") s.type = SignalType::Bool;
    else if (type == "int") s.type = SignalType::Int;
    else if (type == "real") s.type = SignalType::Real;
    else fail("bad_request", "\"type\" is \"bool\", \"int\" or \"real\"");
    s.initialValue = coerceSignal(s.type, number(p, "value", 0));
    s.expression = text(p, "expression", false);
    if (s.kind == SignalKind::Derived && s.expression.isEmpty()) fail("bad_request", "a derived signal needs an \"expression\"");
    s.description = p.value("description").toString();
    if (given(p, "address")) s.ioAddress = int(number(p, "address", -1));
    m_scene->addSignal(s);
    return QJsonObject{{"name", s.name}, {"kind", signalKindName(s.kind)}, {"type", signalTypeName(s.type)},
                       {"value", s.initialValue}};
}

QJsonValue ApiServer::sensorAdd(const QJsonObject& p, Call&) {
    requireEditable();
    Signal s;
    s.name = text(p, "name");
    checkSymbol(*m_scene, s.name);
    s.kind = SignalKind::Sensor;
    readSensor(s.sensor, p, false);
    s.type = sensorSignalType(s.sensor.type);
    s.description = p.value("description").toString();
    m_scene->addSignal(s);
    QJsonObject o = sensorJson(s.sensor);
    o["name"] = s.name;
    o["signal"] = signalTypeName(s.type);
    return o;
}

QJsonValue ApiServer::poseAdd(const QJsonObject& p, Call&) {
    requireEditable();
    Pose pose;
    pose.name = text(p, "name");
    const bool replace = flag(p, "replace", false);
    int existing = -1;
    for (const auto& [id, q] : m_scene->allPoses()) if (q.name == pose.name) existing = id;
    if (existing != -1 && !replace) fail("bad_request", QString("there is a pose \"%1\" already (\"replace\": true to change it)").arg(pose.name));
    const Robot r = robotParam(p, false);
    if (given(p, "joints")) {
        const QJsonValue jv = p.value("joints");
        if (!jv.isObject()) fail("bad_request", "\"joints\" must be {actuator: value}");
        const QJsonObject jo = jv.toObject();
        for (auto it = jo.begin(); it != jo.end(); ++it) {
            if (!it.value().isDouble()) fail("bad_request", QString("the value of %1 must be a number").arg(it.key()));
            const int jid = jointByName(it.key(), r.root != -1 ? &r : nullptr);
            pose.jointValues[jid] = float(fromApi(*m_scene->getJoint(jid), it.value().toDouble()));
        }
    } else if (r.root != -1) {
        for (int jid : r.joints) pose.jointValues[jid] = m_scene->getJoint(jid)->currentValue;   // where it stands now
    } else {
        fail("bad_request", "give \"joints\" {actuator: value}, or a \"robot\" to keep where it stands");
    }
    if (existing != -1) {
        Pose* old = m_scene->getPose(existing);
        old->jointValues = pose.jointValues;
        old->frameId = -1;
        old->toolId = -1;
        m_scene->notifyAutomationChanged();
    } else {
        m_scene->addPose(pose);
    }
    QJsonObject joints;
    for (auto& [jid, v] : pose.jointValues) {
        const Joint* j = m_scene->getJoint(jid);
        const Motor* m = m_scene->motorForJoint(jid);
        joints[m ? m->name : j->name] = toApi(*j, v);
    }
    return QJsonObject{{"name", pose.name}, {"joints", joints}};
}

// ── Sequences ────────────────────────────────────────────────────────────────

Operation ApiServer::readOperation(const QJsonObject& o, const std::vector<Operation>& before) {
    Operation op;
    op.id = m_scene->allocateId();
    const QString typeText = text(o, "type");
    bool known = false;
    for (int t = int(OperationType::Move); t <= int(OperationType::PlayMotion); ++t)
        if (operationTypeName(OperationType(t)).compare(typeText, Qt::CaseInsensitive) == 0) {
            op.type = OperationType(t);
            known = true;
        }
    if (!known) fail("bad_request", QString("there is no operation type \"%1\"; types: move, moveToPose, run, stop, set, wait, "
                                            "waitFor, grip, release, moveL, walk, playMotion").arg(typeText));
    op.name = given(o, "name") ? text(o, "name") : operationTypeName(op.type);
    const QString where = QString("operation %1 (%2)").arg(before.size() + 1).arg(op.name);
    auto need = [&](const QString& key) {
        if (!given(o, key)) fail("bad_request", QString("%1 needs \"%2\"").arg(where, key));
    };
    // Predecessors: names (the nearest one before with that name) or indices
    if (o.contains("after")) {
        const QJsonValue a = o.value("after");
        const QJsonArray list = a.isArray() ? a.toArray() : QJsonArray{a};
        for (const QJsonValue& v : list) {
            if (v.isNull()) continue;
            int index = -1;
            if (v.isDouble()) index = v.toInt();
            else if (v.isString())
                for (int i = int(before.size()) - 1; i >= 0 && index == -1; --i)
                    if (before[size_t(i)].name == v.toString()) index = i;
            if (index < 0 || index >= int(before.size()))
                fail("bad_request", QString("%1: \"after\" names %2, which is not an operation before it")
                                        .arg(where, v.isString() ? v.toString() : QString::number(v.toDouble())));
            op.predecessors.push_back(before[size_t(index)].id);
        }
    } else if (!before.empty()) {
        op.predecessors.push_back(before.back().id);   // one after another
    }
    op.startCondition = text(o, "startCondition", false);
    op.onStart = text(o, "onStart", false);
    op.onFinish = text(o, "onFinish", false);

    auto actuator = [&]() -> const Motor* {
        need("actuator");
        const QString name = text(o, "actuator");
        for (const auto& [mid, m] : m_scene->allMotors()) if (m.name == name) { op.actuatorId = mid; return &m; }
        QStringList names;
        for (const auto& [mid, m] : m_scene->allMotors()) names << m.name;
        fail("not_found", QString("%1: there is no actuator \"%2\"%3").arg(where, name, names.isEmpty() ? QString() : "; actuators: " + listOf(names, 30)));
    };
    auto pose = [&] {
        need("pose");
        const QString name = text(o, "pose");
        for (const auto& [id, q] : m_scene->allPoses()) if (q.name == name) { op.poseId = id; return; }
        fail("not_found", QString("%1: there is no pose \"%2\"").arg(where, name));
    };
    auto part = [&] {
        if (given(o, "part")) op.partId = objectParam("part", text(o, "part"));
        if (given(o, "sensor")) {
            Signal* s = m_scene->findSignal(text(o, "sensor"));
            if (!s || s->kind != SignalKind::Sensor) fail("not_found", QString("%1: there is no sensor \"%2\"").arg(where, text(o, "sensor")));
            op.sensorId = s->id;
        }
    };
    auto unitOf = [&](const Motor* m) { return m_scene->getJoint(m->jointId)->type == JointType::Prismatic ? 1 / kMm : 1.0; };

    switch (op.type) {
        case OperationType::Move: {
            const Motor* m = actuator();
            need("target");
            op.target = exprParam(o, "target", unitOf(m));
            op.velocity = exprParam(o, "speed", unitOf(m));
            break;
        }
        case OperationType::Run: {
            const Motor* m = actuator();
            need("speed");
            op.velocity = exprParam(o, "speed", unitOf(m));
            break;
        }
        case OperationType::Stop: actuator(); break;
        case OperationType::MoveToPose: pose(); op.velocity = exprParam(o, "speed"); part(); break;
        case OperationType::Set: break;
        case OperationType::Wait: need("seconds"); op.duration = exprParam(o, "seconds"); break;
        case OperationType::WaitFor: need("condition"); op.condition = text(o, "condition"); break;
        case OperationType::Grip:
            need("gripper");
            op.gripperId = objectParam("gripper", text(o, "gripper"));
            part();
            if (op.partId == -1 && op.sensorId == -1) fail("bad_request", QString("%1 needs a \"sensor\" that sees the part, or the \"part\"").arg(where));
            break;
        case OperationType::Release: need("gripper"); op.gripperId = objectParam("gripper", text(o, "gripper")); break;
        case OperationType::MoveL:
            need("tool");
            op.toolId = objectParam("tool", text(o, "tool"));
            pose();
            op.velocity = exprParam(o, "speed", 1 / kMm);
            part();
            break;
        case OperationType::Walk: {
            need("robot");
            op.walkerId = objectParam("robot", text(o, "robot"));
            op.velocity = exprParam(o, "speed", 1 / kMm);
            if (given(o, "to")) op.walkToId = objectParam("object", text(o, "to"));
            else if (given(o, "route")) {
                op.walkRoute = true;
                for (const QJsonValue& v : o.value("route").toArray()) {
                    const QJsonArray a = v.toArray();
                    const QJsonObject po = v.toObject();
                    if (a.size() == 2) op.route.push_back(QPointF(a[0].toDouble() / kMm, a[1].toDouble() / kMm));
                    else if (po.contains("x") && po.contains("z")) op.route.push_back(QPointF(po["x"].toDouble() / kMm, po["z"].toDouble() / kMm));
                    else fail("bad_request", QString("%1: \"route\" points are [x, z] or {x, z} (mm)").arg(where));
                }
                if (op.route.empty()) fail("bad_request", QString("%1: \"route\" has no points").arg(where));
                op.routeHeading = exprParam(o, "heading");
            } else {
                op.forward = exprParam(o, "forward", 1 / kMm);
                op.sideways = exprParam(o, "sideways", 1 / kMm);
                op.turn = exprParam(o, "turn");
            }
            break;
        }
        case OperationType::PlayMotion: {
            need("motion");
            const QString name = text(o, "motion");
            for (const auto& [id, m] : m_scene->allMotions()) if (m.name == name) op.motionId = id;
            for (const auto& [id, m] : m_scene->allLearned()) if (m.name == name) op.motionId = id;
            if (op.motionId == -1) fail("not_found", QString("%1: there is no motion \"%2\"").arg(where, name));
            if (given(o, "robot")) op.walkerId = objectParam("robot", text(o, "robot"));
            op.relative = flag(o, "relative", true);
            op.velocity = exprParam(o, "speed");
            op.clip = text(o, "clip", false);
            op.blendClip = text(o, "blendClip", false);
            op.blend = exprParam(o, "blend");
            op.duration = exprParam(o, "seconds");
            break;
        }
    }
    return op;
}

QJsonObject ApiServer::operationJson(const Operation& op, const Sequence& seq) const {
    QJsonObject o{{"name", op.name}, {"type", lowerFirst(operationTypeName(op.type))}};
    QJsonArray after;
    for (int pid : op.predecessors)
        for (size_t i = 0; i < seq.operations.size(); ++i)
            if (seq.operations[i].id == pid) after.append(int(i));
    o["after"] = after;
    auto put = [&](const char* key, const QString& v) { if (!v.isEmpty()) o[key] = v; };
    auto object = [&](const char* key, int id) { if (m_scene->getObject(id)) o[key] = pathOf(*m_scene, id); };
    if (const Motor* m = m_scene->getMotor(op.actuatorId)) o["actuator"] = m->name;
    if (const Pose* q = m_scene->getPose(op.poseId)) o["pose"] = q->name;
    object("gripper", op.gripperId);
    object("part", op.partId);
    object("tool", op.toolId);
    object("robot", op.walkerId);
    object("to", op.walkToId);
    if (op.sensorId != -1)
        for (const auto& [id, s] : m_scene->allSignals()) if (id == op.sensorId) o["sensor"] = s.name;
    if (const MotionClip* m = m_scene->getMotion(op.motionId)) o["motion"] = m->name;
    if (const LearnedMotion* m = m_scene->getLearned(op.motionId)) o["motion"] = m->name;
    put("target", op.target);
    put("speed", op.velocity);
    put("seconds", op.duration);
    put("condition", op.condition);
    put("startCondition", op.startCondition);
    put("onStart", op.onStart);
    put("onFinish", op.onFinish);
    if (op.type == OperationType::Walk) {
        put("forward", op.forward);
        put("sideways", op.sideways);
        put("turn", op.turn);
        if (op.walkRoute) {
            QJsonArray route;
            for (const QPointF& q : op.route) route.append(QJsonArray{roundMm(q.x() * kMm), roundMm(q.y() * kMm)});
            o["route"] = route;
            put("heading", op.routeHeading);
        }
    }
    if (op.type == OperationType::PlayMotion) {
        o["relative"] = op.relative;
        put("clip", op.clip);
        put("blendClip", op.blendClip);
        put("blend", op.blend);
    }
    return o;
}

QJsonValue ApiServer::sequenceAdd(const QJsonObject& p, Call& c) {
    requireEditable();
    Sequence q;
    q.name = text(p, "name");
    int existing = -1;
    for (const auto& [id, s] : m_scene->allSequences()) if (s.name == q.name) existing = id;
    if (existing != -1 && !flag(p, "replace", false))
        fail("bad_request", QString("there is a sequence \"%1\" already (\"replace\": true to change it)").arg(q.name));
    checkSymbol(*m_scene, q.name, existing);
    q.loop = flag(p, "loop", false);
    q.enableCondition = text(p, "enable", false);
    q.startCondition = text(p, "start", false);
    const QJsonValue ops = p.value("operations");
    if (!ops.isArray()) fail("bad_request", "\"operations\" must be a list");
    for (const QJsonValue& v : ops.toArray()) {
        if (!v.isObject()) fail("bad_request", "each operation must be an object with a \"type\"");
        q.operations.push_back(readOperation(v.toObject(), q.operations));
    }
    if (existing != -1) {
        Sequence* s = m_scene->getSequence(existing);
        q.id = s->id;
        *s = q;
        m_scene->notifyAutomationChanged();
    } else {
        m_scene->addSequence(q);
    }
    return sequenceGet(QJsonObject{{"name", q.name}}, c);
}

QJsonValue ApiServer::sequenceGet(const QJsonObject& p, Call&) {
    const QString name = text(p, "name");
    for (const auto& [id, q] : m_scene->allSequences()) {
        if (q.name != name) continue;
        QJsonArray ops;
        for (const Operation& op : q.operations) ops.append(operationJson(op, q));
        QJsonObject o{{"name", q.name}, {"loop", q.loop}, {"operations", ops}};
        if (!q.enableCondition.isEmpty()) o["enable"] = q.enableCondition;
        if (!q.startCondition.isEmpty()) o["start"] = q.startCondition;
        return o;
    }
    fail("not_found", QString("there is no sequence \"%1\"").arg(name));
}

// ── Part flow ────────────────────────────────────────────────────────────────

QJsonValue ApiServer::sourceAdd(const QJsonObject& p, Call&) {
    requireEditable();
    PartSource s;
    s.name = text(p, "name");
    checkSymbol(*m_scene, s.name);
    s.templateId = objectParam("template", text(p, "template"));
    const QString mode = given(p, "mode") ? text(p, "mode").toLower() : QString(given(p, "trigger") ? "trigger" : "interval");
    if (mode == "interval") {
        s.mode = SourceMode::Interval;
        if (given(p, "interval")) s.interval = exprParam(p, "interval");
    } else if (mode == "trigger") {
        s.mode = SourceMode::Trigger;
        s.trigger = text(p, "trigger");
    } else {
        fail("bad_request", "\"mode\" is \"interval\" or \"trigger\"");
    }
    s.enable = text(p, "enable", false);
    s.maxCount = int(number(p, "max", 0));
    s.whenClear = flag(p, "whenClear", true);
    m_scene->addSource(s);
    m_scene->notifyAutomationChanged();
    QJsonObject o{{"name", s.name}, {"template", pathOf(*m_scene, s.templateId)},
                  {"mode", s.mode == SourceMode::Trigger ? "trigger" : "interval"}, {"max", s.maxCount},
                  {"whenClear", s.whenClear}};
    if (s.mode == SourceMode::Trigger) o["trigger"] = s.trigger; else o["interval"] = s.interval;
    if (!s.enable.isEmpty()) o["enable"] = s.enable;
    return o;
}

QJsonValue ApiServer::sinkAdd(const QJsonObject& p, Call&) {
    requireEditable();
    PartSink s;
    s.name = text(p, "name");
    checkSymbol(*m_scene, s.name);
    readSensor(s.zone, p, true);
    if (given(p, "source")) {
        const QString name = text(p, "source");
        for (const auto& [id, src] : m_scene->allSources()) if (src.name == name) s.sourceId = id;
        if (s.sourceId == -1) fail("not_found", QString("there is no source \"%1\"").arg(name));
    }
    s.enable = text(p, "enable", false);
    m_scene->addSink(s);
    m_scene->notifyAutomationChanged();
    QJsonObject o = sensorJson(s.zone);
    o.remove("detects");
    o.remove("type");
    o["name"] = s.name;
    if (!s.enable.isEmpty()) o["enable"] = s.enable;
    return o;
}

QJsonValue ApiServer::automationRemove(const QJsonObject& p, Call&) {
    requireEditable();
    const QString name = text(p, "name");
    QString what;
    for (const auto& [id, s] : m_scene->allSignals())
        if (s.name == name) { what = s.kind == SignalKind::Sensor ? "sensor" : "signal"; m_scene->removeSignal(id); break; }
    if (what.isEmpty())
        for (const auto& [id, s] : m_scene->allSequences()) if (s.name == name) { what = "sequence"; m_scene->removeSequence(id); break; }
    if (what.isEmpty())
        for (const auto& [id, s] : m_scene->allSources()) if (s.name == name) { what = "source"; m_scene->removeSource(id); break; }
    if (what.isEmpty())
        for (const auto& [id, s] : m_scene->allSinks()) if (s.name == name) { what = "sink"; m_scene->removeSink(id); break; }
    if (what.isEmpty())
        for (const auto& [id, s] : m_scene->allPoses()) if (s.name == name) { what = "pose"; m_scene->removePose(id); break; }
    if (what.isEmpty())
        for (const auto& [id, s] : m_scene->allMotions()) if (s.name == name) { what = "motion"; m_scene->removeMotion(id); break; }
    if (what.isEmpty())
        fail("not_found", QString("there is no signal, sensor, sequence, source, sink, pose or motion \"%1\"").arg(name));
    m_scene->notifyAutomationChanged();
    return QJsonObject{{"removed", what}};
}
