#include "ProjectSerializer.h"
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QByteArray>
#include "../scene/MeshWeld.h"
#include <cstring>
#include <map>
#include <algorithm>
#include <vector>

// ── helpers ──────────────────────────────────────────────────────────────────

static QJsonObject vec3ToJson(const QVector3D& v) {
    return {{"x", v.x()}, {"y", v.y()}, {"z", v.z()}};
}
static QVector3D vec3FromJson(const QJsonObject& o) {
    return { float(o["x"].toDouble()), float(o["y"].toDouble()), float(o["z"].toDouble()) };
}

static QJsonObject transformToJson(const Transform& t) {
    return {
        {"position", vec3ToJson(t.position)},
        {"rotation", vec3ToJson(t.rotation)},
        {"scale",    vec3ToJson(t.scale)}
    };
}
static Transform transformFromJson(const QJsonObject& o) {
    Transform t;
    t.position = vec3FromJson(o["position"].toObject());
    t.rotation = vec3FromJson(o["rotation"].toObject());
    t.scale    = vec3FromJson(o["scale"].toObject());
    return t;
}

static QString primitiveToStr(PrimitiveType t) {
    switch (t) {
        case PrimitiveType::Cube:         return "Cube";
        case PrimitiveType::Cylinder:     return "Cylinder";
        case PrimitiveType::Sphere:       return "Sphere";
        case PrimitiveType::Capsule:      return "Capsule";
        case PrimitiveType::Basket:       return "Basket";
        case PrimitiveType::ImportedMesh: return "ImportedMesh";
    }
    return "Cube";
}
static PrimitiveType primitiveFromStr(const QString& s) {
    if (s == "Cylinder")     return PrimitiveType::Cylinder;
    if (s == "Sphere")       return PrimitiveType::Sphere;
    if (s == "Capsule")      return PrimitiveType::Capsule;
    if (s == "Basket")       return PrimitiveType::Basket;
    if (s == "ImportedMesh") return PrimitiveType::ImportedMesh;
    return PrimitiveType::Cube;
}

// Mesh geometry is stored once per unique MeshGeometry (shared_ptr identity)
// in a top-level "meshes" array; objects with the same imported part reference
// it by index so repeated assembly instances don't duplicate triangle data.
//
// On disk a mesh is welded into indexed form (lossless, see MeshWeld.h) and
// both arrays are zlib-compressed before base64. Older files stored the raw
// triangle soup as "vertexData"; those still load.
static QString packArray(const void* data, size_t bytes) {
    return QString::fromLatin1(qCompress(QByteArray::fromRawData(static_cast<const char*>(data),
                                                                 qsizetype(bytes))).toBase64());
}
template <typename T>
static std::vector<T> unpackArray(const QJsonValue& v) {
    const QByteArray raw = qUncompress(QByteArray::fromBase64(v.toString().toLatin1()));
    std::vector<T> out(size_t(raw.size()) / sizeof(T));
    if (!out.empty()) memcpy(out.data(), raw.constData(), out.size() * sizeof(T));
    return out;
}

static QJsonObject meshToJson(const MeshGeometry& m) {
    const WeldedMesh w = weldMesh(m.interleaved);
    QJsonObject o;
    o["encoding"] = "indexed-zlib";
    o["vertices"] = packArray(w.vertices.data(), w.vertices.size() * sizeof(float));
    o["indices"]  = packArray(w.indices.data(), w.indices.size() * sizeof(uint32_t));
    o["aabbMin"]  = vec3ToJson(m.aabbMin);
    o["aabbMax"]  = vec3ToJson(m.aabbMax);
    return o;
}
static std::shared_ptr<MeshGeometry> meshFromJson(const QJsonObject& o) {
    auto m = std::make_shared<MeshGeometry>();
    if (o["encoding"].toString() == "indexed-zlib") {
        m->interleaved = unweldMesh(unpackArray<float>(o["vertices"]), unpackArray<uint32_t>(o["indices"]));
    } else {
        QByteArray raw = QByteArray::fromBase64(o["vertexData"].toString().toLatin1());
        m->interleaved.resize(size_t(raw.size()) / sizeof(float));
        if (!m->interleaved.empty())
            memcpy(m->interleaved.data(), raw.constData(), m->interleaved.size() * sizeof(float));
    }
    m->aabbMin = vec3FromJson(o["aabbMin"].toObject());
    m->aabbMax = vec3FromJson(o["aabbMax"].toObject());
    return m;
}

// ── MeshPool ─────────────────────────────────────────────────────────────────

int MeshPool::add(const std::shared_ptr<MeshGeometry>& mesh) {
    auto it = m_keys.find(mesh.get());
    if (it != m_keys.end()) return it->second;
    const int key = m_nextKey++;
    m_keys[mesh.get()] = key;
    m_meshes[key] = mesh;
    return key;
}

std::shared_ptr<MeshGeometry> MeshPool::get(int key) const {
    auto it = m_meshes.find(key);
    return it != m_meshes.end() ? it->second : nullptr;
}

void MeshPool::retainOnly(const std::set<int>& keys) {
    for (auto it = m_meshes.begin(); it != m_meshes.end();) {
        if (keys.count(it->first)) { ++it; continue; }
        m_keys.erase(it->second.get());
        it = m_meshes.erase(it);
    }
}

void MeshPool::clear() {
    m_keys.clear();
    m_meshes.clear();
}

void MeshPool::collectKeys(const QJsonObject& snapshot, std::set<int>* keys) {
    for (auto v : snapshot["meshes"].toArray()) {
        const QJsonObject o = v.toObject();
        if (o.contains("ref")) keys->insert(o["ref"].toInt());
    }
}

static QString collisionTypeToStr(CollisionType t) {
    switch (t) {
        case CollisionType::Box:      return "Box";
        case CollisionType::Cylinder: return "Cylinder";
        case CollisionType::Sphere:   return "Sphere";
        case CollisionType::Capsule:  return "Capsule";
    }
    return "Box";
}
static CollisionType collisionTypeFromStr(const QString& s) {
    if (s == "Cylinder") return CollisionType::Cylinder;
    if (s == "Sphere")   return CollisionType::Sphere;
    if (s == "Capsule")  return CollisionType::Capsule;
    return CollisionType::Box;
}

// ── save ─────────────────────────────────────────────────────────────────────

QJsonObject ProjectSerializer::toJson(const SceneGraph& scene, MeshPool* pool) {
    QJsonArray objects, joints, motors, rigidBodies, conveyorBelts, meshes;

    // De-dupe by MeshGeometry identity so shared assembly parts serialize once
    std::map<const MeshGeometry*, int> meshIndex;

    // Iterate in id order so a save → load → save round trip is stable (ids
    // are reallocated in this order on load; undo relies on stable snapshots).
    auto sortedKeys = [](const auto& map) {
        std::vector<int> keys;
        for (auto& kv : map) keys.push_back(kv.first);
        std::sort(keys.begin(), keys.end());
        return keys;
    };

    for (int oid : sortedKeys(scene.allObjects())) {
        const SceneObject& obj = scene.allObjects().at(oid);
        QJsonObject o;
        o["id"]        = obj.id;
        o["name"]      = obj.name;
        o["type"]      = primitiveToStr(obj.type);
        o["parentId"]  = obj.parentId;
        o["transform"] = transformToJson(obj.transform);

        if (obj.mesh) {
            auto it = meshIndex.find(obj.mesh.get());
            int idx;
            if (it != meshIndex.end()) {
                idx = it->second;
            } else {
                idx = meshes.size();
                meshes.append(pool ? QJsonObject{{"ref", pool->add(obj.mesh)}} : meshToJson(*obj.mesh));
                meshIndex[obj.mesh.get()] = idx;
            }
            o["meshId"] = idx;
        }
        objects.append(o);
    }

    for (int jid : sortedKeys(scene.allJoints())) {
        const Joint& j = scene.allJoints().at(jid);
        QJsonObject o;
        o["id"]           = j.id;
        o["name"]         = j.name;
        o["type"]         = (j.type == JointType::Revolute ? "Revolute" : "Prismatic");
        o["axis"]         = vec3ToJson(j.axis);
        o["pivot"]        = vec3ToJson(j.pivot);
        o["limitMin"]     = j.limitMin;
        o["limitMax"]     = j.limitMax;
        o["initialAngle"] = j.initialAngle;
        o["continuous"]   = j.continuous;
        o["parentBodyId"] = j.parentBodyId;
        o["childBodyId"]  = j.childBodyId;
        joints.append(o);
    }

    for (int mid : scene.motorIdsSorted()) {
        const Motor& m = *scene.getMotor(mid);
        QJsonObject o;
        o["id"]           = m.id;
        o["name"]         = m.name;
        o["jointId"]      = m.jointId;
        o["mode"]         = m.mode == ActuatorMode::Speed ? "Speed" : "Position";
        o["velocity"]     = m.velocity;
        o["acceleration"] = m.acceleration;
        o["enabled"]      = m.enabled;
        o["targetExpr"]   = m.targetExpr;
        o["velocityExpr"] = m.velocityExpr;
        o["activeExpr"]   = m.activeExpr;
        motors.append(o);
    }

    QJsonArray signalsArr, sequences, poses;
    for (auto& [id, sg] : scene.allSignals()) {
        QJsonObject o;
        o["id"]           = sg.id;
        o["name"]         = sg.name;
        o["type"]         = signalTypeName(sg.type);
        o["kind"]         = signalKindName(sg.kind);
        o["initialValue"] = sg.initialValue;
        o["expression"]   = sg.expression;
        o["description"]  = sg.description;
        if (sg.kind == SignalKind::Sensor) {
            const SensorConfig& c = sg.sensor;
            QJsonObject so;
            so["type"]         = c.type == SensorType::Zone ? "Zone" : "Beam";
            so["objectId"]     = c.objectId;
            so["offset"]       = vec3ToJson(c.offset);
            so["direction"]    = vec3ToJson(c.direction);
            so["range"]        = double(c.range);
            so["size"]         = vec3ToJson(c.size);
            so["objectBounds"] = c.objectBounds;
            so["target"]       = c.target == SensorTarget::RigidBodies ? "RigidBodies"
                               : c.target == SensorTarget::Object ? "Object" : "Any";
            so["targetId"]     = c.targetId;
            o["sensor"] = so;
        }
        signalsArr.append(o);
    }
    for (auto& [id, q] : scene.allSequences()) {
        QJsonObject o;
        o["id"]              = q.id;
        o["name"]            = q.name;
        o["enableCondition"] = q.enableCondition;
        o["startCondition"]  = q.startCondition;
        o["loop"]            = q.loop;
        QJsonArray ops;
        for (auto& op : q.operations) {
            QJsonObject oo;
            oo["id"]             = op.id;
            oo["name"]           = op.name;
            oo["type"]           = operationTypeName(op.type);
            oo["actuatorId"]     = op.actuatorId;
            oo["poseId"]         = op.poseId;
            if (op.gripperId != -1) oo["gripperId"] = op.gripperId;
            if (op.partId != -1)    oo["partId"]    = op.partId;
            if (op.sensorId != -1)  oo["sensorId"]  = op.sensorId;
            if (op.toolId != -1)    oo["toolId"]    = op.toolId;
            oo["target"]         = op.target;
            oo["velocity"]       = op.velocity;
            oo["duration"]       = op.duration;
            oo["condition"]      = op.condition;
            oo["startCondition"] = op.startCondition;
            oo["onStart"]        = op.onStart;
            oo["onFinish"]       = op.onFinish;
            QJsonArray preds;
            for (int p : op.predecessors) preds.append(p);
            oo["predecessors"] = preds;
            ops.append(oo);
        }
        o["operations"] = ops;
        sequences.append(o);
    }
    for (auto& [id, pose] : scene.allPoses()) {
        QJsonObject o;
        o["id"]   = pose.id;
        o["name"] = pose.name;
        QJsonArray vals;
        for (auto& [jid, v] : pose.jointValues)
            vals.append(QJsonObject{{"jointId", jid}, {"value", double(v)}});
        o["values"] = vals;
        poses.append(o);
    }

    for (int objectId : sortedKeys(scene.allRigidBodies())) {
        const RigidBody& rb = scene.allRigidBodies().at(objectId);
        QJsonObject o;
        o["objectId"]       = objectId;
        o["mass"]           = rb.mass;
        o["gravityEnabled"] = rb.gravityEnabled;
        o["hasCollision"]   = rb.hasCollision;
        if (rb.hasCollision) {
            QJsonObject col;
            col["type"]   = collisionTypeToStr(rb.collision.type);
            col["halfW"]  = rb.collision.halfW;
            col["halfH"]  = rb.collision.halfH;
            col["halfD"]  = rb.collision.halfD;
            col["radius"] = rb.collision.radius;
            col["height"] = rb.collision.height;
            col["offset"] = vec3ToJson(rb.collision.offset);
            o["collision"] = col;
        }
        rigidBodies.append(o);
    }

    for (int objectId : sortedKeys(scene.allConveyorBelts())) {
        const ConveyorBelt& belt = scene.allConveyorBelts().at(objectId);
        QJsonObject o;
        o["objectId"]  = objectId;
        o["velocity"]  = double(belt.velocity);
        o["direction"] = vec3ToJson(belt.direction);
        o["enabled"]   = belt.enabled;
        if (!belt.speedExpr.isEmpty()) o["speedExpr"] = belt.speedExpr;
        conveyorBelts.append(o);
    }

    QJsonObject root;
    root["version"]       = 3;
    root["objects"]       = objects;
    root["joints"]        = joints;
    root["motors"]        = motors;
    root["rigidBodies"]   = rigidBodies;
    root["conveyorBelts"] = conveyorBelts;
    root["meshes"]        = meshes;
    root["signals"]       = signalsArr;
    root["sequences"]     = sequences;
    root["poses"]         = poses;
    return root;
}

bool ProjectSerializer::save(const SceneGraph& scene, const QString& filePath) {
    QFile f(filePath);
    if (!f.open(QIODevice::WriteOnly)) return false;
    f.write(QJsonDocument(toJson(scene)).toJson(QJsonDocument::Indented));
    return true;
}

// ── load ─────────────────────────────────────────────────────────────────────

bool ProjectSerializer::fromJson(SceneGraph& scene, const QJsonObject& root, const MeshPool* pool) {
    scene.clear();

    // Shared mesh geometry, indexed as written by save()
    std::vector<std::shared_ptr<MeshGeometry>> meshList;
    for (auto v : root["meshes"].toArray()) {
        const QJsonObject o = v.toObject();
        meshList.push_back(pool && o.contains("ref") ? pool->get(o["ref"].toInt()) : meshFromJson(o));
    }

    struct ObjData {
        int id; QString name; PrimitiveType type; int parentId; Transform transform;
        int meshId = -1;
    };
    QList<ObjData> objList;
    for (auto v : root["objects"].toArray()) {
        auto o = v.toObject();
        ObjData d;
        d.id        = o["id"].toInt();
        d.name      = o["name"].toString();
        d.type      = primitiveFromStr(o["type"].toString());
        d.parentId  = o["parentId"].toInt(-1);
        d.transform = transformFromJson(o["transform"].toObject());
        d.meshId    = o.contains("meshId") ? o["meshId"].toInt(-1) : -1;
        objList.append(d);
    }

    // Sort by id so parents come before children
    std::sort(objList.begin(), objList.end(), [](const ObjData& a, const ObjData& b){
        return a.id < b.id;
    });

    QHash<int, int> idMap;
    for (auto& d : objList) {
        int mappedParent = (d.parentId == -1) ? -1 : idMap.value(d.parentId, -1);
        int newId = scene.addObject(d.name, d.type, mappedParent);
        idMap[d.id] = newId;
        if (SceneObject* obj = scene.getObject(newId)) {
            obj->transform = d.transform;
            if (d.meshId >= 0 && d.meshId < int(meshList.size()) && meshList[size_t(d.meshId)])
                obj->mesh = meshList[size_t(d.meshId)];
        }
    }

    // Joints
    QHash<int, int> jointIdMap;
    for (auto v : root["joints"].toArray()) {
        auto o = v.toObject();
        Joint j;
        j.name         = o["name"].toString();
        j.type         = (o["type"].toString() == "Revolute" ? JointType::Revolute : JointType::Prismatic);
        j.axis         = vec3FromJson(o["axis"].toObject());
        j.pivot        = vec3FromJson(o["pivot"].toObject());
        j.limitMin     = float(o["limitMin"].toDouble(-180));
        j.limitMax     = float(o["limitMax"].toDouble(180));
        j.initialAngle = float(o["initialAngle"].toDouble());
        j.continuous   = o["continuous"].toBool(false);
        j.parentBodyId = idMap.value(o["parentBodyId"].toInt(-1), -1);
        j.childBodyId  = idMap.value(o["childBodyId"].toInt(-1), -1);
        jointIdMap[o["id"].toInt(-1)] = scene.addJoint(j);
    }

    // Actuators (motors)
    QHash<int, int> motorIdMap;
    for (auto v : root["motors"].toArray()) {
        auto o = v.toObject();
        Motor m;
        m.name         = o["name"].toString();
        m.jointId      = jointIdMap.value(o["jointId"].toInt(-1), -1);
        m.mode         = o["mode"].toString() == "Speed" ? ActuatorMode::Speed : ActuatorMode::Position;
        m.velocity     = float(o["velocity"].toDouble(45));
        m.acceleration = float(o["acceleration"].toDouble(0));
        m.enabled      = o["enabled"].toBool(true);
        m.targetExpr   = o["targetExpr"].toString();
        m.velocityExpr = o["velocityExpr"].toString();
        m.activeExpr   = o["activeExpr"].toString();
        if (m.jointId != -1 && !scene.motorForJoint(m.jointId))
            motorIdMap[o["id"].toInt(-1)] = scene.addMotor(m);
    }

    // Signals
    QHash<int, int> signalIdMap;
    for (auto v : root["signals"].toArray()) {
        auto o = v.toObject();
        Signal sg;
        sg.name         = o["name"].toString();
        QString t = o["type"].toString();
        sg.type         = t == "Int" ? SignalType::Int : (t == "Real" ? SignalType::Real : SignalType::Bool);
        sg.kind         = signalKindFromName(o["kind"].toString());
        sg.initialValue = o["initialValue"].toDouble();
        sg.expression   = o["expression"].toString();
        sg.description  = o["description"].toString();
        if (sg.kind == SignalKind::Sensor) {
            const QJsonObject so = o["sensor"].toObject();
            SensorConfig& c = sg.sensor;
            // An object reference that doesn't resolve stays broken (-2)
            // rather than silently becoming the world
            auto object = [&](const QJsonValue& v) { int id = v.toInt(-1); return id == -1 ? -1 : idMap.value(id, -2); };
            c.type         = so["type"].toString() == "Zone" ? SensorType::Zone : SensorType::Beam;
            c.objectId     = object(so["objectId"]);
            c.offset       = vec3FromJson(so["offset"].toObject());
            c.direction    = vec3FromJson(so["direction"].toObject());
            c.range        = float(so["range"].toDouble(0.5));
            c.size         = vec3FromJson(so["size"].toObject());
            c.objectBounds = so["objectBounds"].toBool(false);
            const QString t = so["target"].toString();
            c.target       = t == "RigidBodies" ? SensorTarget::RigidBodies
                           : t == "Object" ? SensorTarget::Object : SensorTarget::Any;
            c.targetId     = object(so["targetId"]);
            sg.type        = SignalType::Bool;
        }
        signalIdMap[o["id"].toInt(-1)] = scene.addSignal(sg);
    }

    // Poses
    QHash<int, int> poseIdMap;
    for (auto v : root["poses"].toArray()) {
        auto o = v.toObject();
        Pose p;
        p.name = o["name"].toString();
        for (auto pv : o["values"].toArray()) {
            auto po = pv.toObject();
            int jid = jointIdMap.value(po["jointId"].toInt(-1), -1);
            if (jid != -1) p.jointValues[jid] = float(po["value"].toDouble());
        }
        poseIdMap[o["id"].toInt(-1)] = scene.addPose(p);
    }

    // Sequences
    for (auto v : root["sequences"].toArray()) {
        auto o = v.toObject();
        Sequence q;
        q.name            = o["name"].toString();
        q.enableCondition = o["enableCondition"].toString();
        q.startCondition  = o["startCondition"].toString();
        q.loop            = o["loop"].toBool(false);
        QHash<int, int> opIdMap;
        QList<QJsonObject> opObjs;
        for (auto ov : o["operations"].toArray()) {
            auto oo = ov.toObject();
            opIdMap[oo["id"].toInt(-1)] = scene.allocateId();
            opObjs << oo;
        }
        for (auto& oo : opObjs) {
            Operation op;
            op.id             = opIdMap.value(oo["id"].toInt(-1));
            op.name           = oo["name"].toString();
            op.type           = operationTypeFromName(oo["type"].toString());
            op.actuatorId     = motorIdMap.value(oo["actuatorId"].toInt(-1), -1);
            op.poseId         = poseIdMap.value(oo["poseId"].toInt(-1), -1);
            // References that don't resolve stay broken (-2) so validation reports them
            auto ref = [](const QHash<int, int>& map, const QJsonValue& v) {
                const int id = v.toInt(-1);
                return id == -1 ? -1 : map.value(id, -2);
            };
            op.gripperId      = ref(idMap, oo["gripperId"]);
            op.partId         = ref(idMap, oo["partId"]);
            op.sensorId       = ref(signalIdMap, oo["sensorId"]);
            op.toolId         = ref(idMap, oo["toolId"]);
            op.target         = oo["target"].toString();
            op.velocity       = oo["velocity"].toString();
            op.duration       = oo["duration"].toString();
            op.condition      = oo["condition"].toString();
            op.startCondition = oo["startCondition"].toString();
            op.onStart        = oo["onStart"].toString();
            op.onFinish       = oo["onFinish"].toString();
            for (auto pv : oo["predecessors"].toArray())
                if (opIdMap.contains(pv.toInt())) op.predecessors.push_back(opIdMap.value(pv.toInt()));
            q.operations.push_back(op);
        }
        scene.addSequence(q);
    }

    // Rigid bodies
    for (auto v : root["rigidBodies"].toArray()) {
        auto o = v.toObject();
        int origId = o["objectId"].toInt();
        int newId  = idMap.value(origId, -1);
        if (newId == -1) continue;

        scene.addRigidBody(newId);   // creates with defaults
        if (RigidBody* rb = scene.getRigidBody(newId)) {
            rb->mass           = float(o["mass"].toDouble(1.0));
            rb->gravityEnabled = o["gravityEnabled"].toBool(true);
            rb->hasCollision   = o["hasCollision"].toBool(false);
            if (rb->hasCollision && o.contains("collision")) {
                auto c         = o["collision"].toObject();
                rb->collision.type   = collisionTypeFromStr(c["type"].toString());
                rb->collision.halfW  = float(c["halfW"].toDouble(0.5));
                rb->collision.halfH  = float(c["halfH"].toDouble(0.5));
                rb->collision.halfD  = float(c["halfD"].toDouble(0.5));
                rb->collision.radius = float(c["radius"].toDouble(0.5));
                rb->collision.height = float(c["height"].toDouble(1.0));
                rb->collision.offset = vec3FromJson(c["offset"].toObject());
            }
        }
    }

    // Conveyor belts
    for (auto v : root["conveyorBelts"].toArray()) {
        auto o = v.toObject();
        int origId = o["objectId"].toInt();
        int newId  = idMap.value(origId, -1);
        if (newId == -1) continue;
        scene.addConveyorBelt(newId);
        if (ConveyorBelt* belt = scene.getConveyorBelt(newId)) {
            belt->velocity  = float(o["velocity"].toDouble(1.0));
            belt->direction = vec3FromJson(o["direction"].toObject());
            belt->enabled   = o["enabled"].toBool(true);
            belt->speedExpr = o["speedExpr"].toString();
        }
    }

    return true;
}

bool ProjectSerializer::load(SceneGraph& scene, const QString& filePath) {
    QFile f(filePath);
    if (!f.open(QIODevice::ReadOnly)) return false;
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
    if (!doc.isObject()) return false;
    return fromJson(scene, doc.object());
}
