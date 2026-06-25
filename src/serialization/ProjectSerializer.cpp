#include "ProjectSerializer.h"
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

// ── helpers ──────────────────────────────────────────────────────────────────

static QJsonObject vec3ToJson(const QVector3D& v){
    return {{"x", v.x()}, {"y", v.y()}, {"z", v.z()}};
}
static QVector3D vec3FromJson(const QJsonObject& o){
    return { float(o["x"].toDouble()), float(o["y"].toDouble()), float(o["z"].toDouble()) };
}

static QJsonObject transformToJson(const Transform& t){
    return {
        {"position", vec3ToJson(t.position)},
        {"rotation", vec3ToJson(t.rotation)},
        {"scale",    vec3ToJson(t.scale)}
    };
}
static Transform transformFromJson(const QJsonObject& o){
    Transform t;
    t.position = vec3FromJson(o["position"].toObject());
    t.rotation = vec3FromJson(o["rotation"].toObject());
    t.scale    = vec3FromJson(o["scale"].toObject());
    return t;
}

static QString primitiveToStr(PrimitiveType t){
    switch(t){
        case PrimitiveType::Cube:     return "Cube";
        case PrimitiveType::Cylinder: return "Cylinder";
        case PrimitiveType::Sphere:   return "Sphere";
        case PrimitiveType::Capsule:  return "Capsule";
    }
    return "Cube";
}
static PrimitiveType primitiveFromStr(const QString& s){
    if(s=="Cylinder") return PrimitiveType::Cylinder;
    if(s=="Sphere")   return PrimitiveType::Sphere;
    if(s=="Capsule")  return PrimitiveType::Capsule;
    return PrimitiveType::Cube;
}

// ── save ─────────────────────────────────────────────────────────────────────

bool ProjectSerializer::save(const SceneGraph& scene, const QString& filePath) {
    QJsonArray objects, joints, motors;

    for(auto& [id, obj] : scene.allObjects()){
        QJsonObject o;
        o["id"]       = obj.id;
        o["name"]     = obj.name;
        o["type"]     = primitiveToStr(obj.type);
        o["parentId"] = obj.parentId;
        o["transform"] = transformToJson(obj.transform);
        objects.append(o);
    }

    for(auto& [id, j] : scene.allJoints()){
        QJsonObject o;
        o["id"]            = j.id;
        o["name"]          = j.name;
        o["type"]          = (j.type == JointType::Revolute ? "Revolute" : "Prismatic");
        o["axis"]          = vec3ToJson(j.axis);
        o["limitMin"]      = j.limitMin;
        o["limitMax"]      = j.limitMax;
        o["initialAngle"]  = j.initialAngle;
        o["parentBodyId"]  = j.parentBodyId;
        o["childBodyId"]   = j.childBodyId;
        joints.append(o);
    }

    for(auto& [id, m] : scene.allMotors()){
        QJsonObject o;
        o["id"]             = m.id;
        o["name"]           = m.name;
        o["velocity"]       = m.velocity;
        o["targetPosition"] = m.targetPosition;
        o["jointId"]        = m.jointId;
        o["enabled"]        = m.enabled;
        motors.append(o);
    }

    QJsonObject root;
    root["objects"] = objects;
    root["joints"]  = joints;
    root["motors"]  = motors;

    QFile f(filePath);
    if(!f.open(QIODevice::WriteOnly)) return false;
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return true;
}

// ── load ─────────────────────────────────────────────────────────────────────

bool ProjectSerializer::load(SceneGraph& scene, const QString& filePath) {
    QFile f(filePath);
    if(!f.open(QIODevice::ReadOnly)) return false;
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
    if(!doc.isObject()) return false;

    scene.clear();
    QJsonObject root = doc.object();

    struct ObjData {
        int id; QString name; PrimitiveType type; int parentId; Transform transform;
    };
    QList<ObjData> objList;
    for(auto v : root["objects"].toArray()){
        auto o = v.toObject();
        ObjData d;
        d.id       = o["id"].toInt();
        d.name     = o["name"].toString();
        d.type     = primitiveFromStr(o["type"].toString());
        d.parentId = o["parentId"].toInt(-1);
        d.transform = transformFromJson(o["transform"].toObject());
        objList.append(d);
    }

    // Sort by id so parents come before children (simple heuristic)
    std::sort(objList.begin(), objList.end(), [](const ObjData& a, const ObjData& b){
        return a.id < b.id;
    });

    // Add objects, tracking original→new ID mapping
    QHash<int,int> idMap;
    for(auto& d : objList){
        int mappedParent = (d.parentId == -1) ? -1 : idMap.value(d.parentId, -1);
        int newId = scene.addObject(d.name, d.type, mappedParent);
        idMap[d.id] = newId;
        if(SceneObject* obj = scene.getObject(newId))
            obj->transform = d.transform;
    }

    // Joints
    for(auto v : root["joints"].toArray()){
        auto o = v.toObject();
        Joint j;
        j.name        = o["name"].toString();
        j.type        = (o["type"].toString() == "Revolute" ? JointType::Revolute : JointType::Prismatic);
        j.axis        = vec3FromJson(o["axis"].toObject());
        j.limitMin    = float(o["limitMin"].toDouble(-180));
        j.limitMax    = float(o["limitMax"].toDouble(180));
        j.initialAngle = float(o["initialAngle"].toDouble());
        j.parentBodyId = idMap.value(o["parentBodyId"].toInt(-1), -1);
        j.childBodyId  = idMap.value(o["childBodyId"].toInt(-1),  -1);
        scene.addJoint(j);
    }

    // Motors — match by joint name
    for(auto v : root["motors"].toArray()){
        auto o = v.toObject();
        Motor m;
        m.name           = o["name"].toString();
        m.velocity       = float(o["velocity"].toDouble(45));
        m.targetPosition = float(o["targetPosition"].toDouble());
        m.enabled        = o["enabled"].toBool(true);
        // Find joint by matching child body
        m.jointId = -1;
        for(auto& [jid, j] : scene.allJoints()){
            if(j.name == m.name.mid(6)) { // "Motor_" prefix
                m.jointId = jid;
                break;
            }
        }
        if(m.jointId != -1) scene.addMotor(m);
    }

    return true;
}
