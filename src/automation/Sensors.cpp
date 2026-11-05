#include "Sensors.h"
#include "../scene/SceneGraph.h"
#include "../scene/PrimitiveMesh.h"
#include <cmath>
#include <limits>

namespace sensors {
namespace {

const SceneObject* mountOf(const SceneGraph& scene, const SensorConfig& cfg) {
    return cfg.objectId == -1 ? nullptr : scene.getObject(cfg.objectId);
}

// The object or nearest ancestor carrying a rigid body, else -1
int rigidOwner(const SceneGraph& scene, int id) {
    for (const SceneObject* o = scene.getObject(id); o; o = scene.getObject(o->parentId))
        if (scene.allRigidBodies().count(o->id)) return o->id;
    return -1;
}

// The object, or its nearest ancestor, that is `targetId` or a copy of it
// (made by an object source), else -1
int matchTarget(const SceneGraph& scene, int id, int targetId) {
    for (const SceneObject* o = scene.getObject(id); o; o = scene.getObject(o->parentId))
        if (o->id == targetId || o->copyOf == targetId) return o->id;
    return -1;
}

// Whether object `id` can trigger the sensor. A sensor never sees its own
// mount, the mount's children or the parts the mount hangs from.
bool accepts(const SceneGraph& scene, const SensorConfig& cfg, int id) {
    if (cfg.objectId != -1 && (scenequery::inSubtree(scene, id, cfg.objectId) ||
                               scenequery::inSubtree(scene, cfg.objectId, id)))
        return false;
    switch (cfg.target) {
        case SensorTarget::Any:         return true;
        case SensorTarget::RigidBodies: return rigidOwner(scene, id) != -1;
        case SensorTarget::Object:      return cfg.targetId != -1 && matchTarget(scene, id, cfg.targetId) != -1;
    }
    return true;
}

int detectedObject(const SceneGraph& scene, const SensorConfig& cfg, int hitId) {
    switch (cfg.target) {
        case SensorTarget::Any:         return hitId;
        case SensorTarget::RigidBodies: return rigidOwner(scene, hitId);
        case SensorTarget::Object:      return matchTarget(scene, hitId, cfg.targetId);
    }
    return hitId;
}

QString objectName(const SceneGraph& scene, int id) {
    const SceneObject* o = scene.getObject(id);
    return o ? o->name : QString("(deleted)");
}

} // namespace

const char* const kPositionFields[kPositionFieldCount] = {"x", "y", "z", "rx", "ry", "rz"};

void partPose(const SceneGraph& scene, const SensorConfig& cfg, int objectId, double out[kPositionFieldCount]) {
    for (int i = 0; i < kPositionFieldCount; ++i) out[i] = 0;
    if (!scene.getObject(objectId)) return;
    const QMatrix4x4 rel = frame(scene, cfg).inverted() * scenequery::stripScale(scene.worldTransform(objectId));
    const Transform t = SceneGraph::toTransform(rel);
    for (int i = 0; i < 3; ++i) { out[i] = t.position[i]; out[3 + i] = t.rotation[i]; }
}

QMatrix4x4 frame(const SceneGraph& scene, const SensorConfig& cfg) {
    QMatrix4x4 m;
    if (mountOf(scene, cfg)) m = scenequery::stripScale(scene.worldTransform(cfg.objectId));
    m.translate(cfg.offset);
    return m;
}

void beam(const SceneGraph& scene, const SensorConfig& cfg, QVector3D* from, QVector3D* dir) {
    const QMatrix4x4 f = frame(scene, cfg);
    *from = f.column(3).toVector3D();
    *dir  = f.mapVector(cfg.direction).normalized();
}

scenequery::Obb zone(const SceneGraph& scene, const SensorConfig& cfg) {
    if (cfg.objectBounds && mountOf(scene, cfg)) return scenequery::worldBox(scene, cfg.objectId);
    return scenequery::boxInFrame(frame(scene, cfg), cfg.size * 0.5f);
}

Reading read(const SceneGraph& scene, const SensorConfig& cfg) {
    Reading r;
    if (cfg.objectId != -1 && !mountOf(scene, cfg)) return r;
    auto accept = [&](int id) { return accepts(scene, cfg, id); };

    if (sensorIsRay(cfg.type)) {
        QVector3D from, dir;
        beam(scene, cfg, &from, &dir);
        r.distance = cfg.range;
        if (cfg.range <= 0 || dir.isNull()) return r;
        const scenequery::RayHit hit = scenequery::rayCast(scene, from, dir, cfg.range, accept);
        if (hit.hit()) {
            r.active   = true;
            r.distance = hit.t;
            r.objectId = detectedObject(scene, cfg, hit.objectId);
        }
        return r;
    }

    const scenequery::Obb box = zone(scene, cfg);
    float best = std::numeric_limits<float>::max();
    for (auto& [id, obj] : scene.allObjects()) {
        if (!surfaceOf(obj) || obj.parked || !accept(id)) continue;
        const scenequery::Obb other = scenequery::worldBox(scene, id);
        if (!scenequery::overlap(box, other)) continue;
        const float d = (other.center - box.center).lengthSquared();
        if (d < best) {
            best = d;
            r.active   = true;
            r.objectId = detectedObject(scene, cfg, id);
        }
    }
    return r;
}

QString problem(const SceneGraph& scene, const SensorConfig& cfg) {
    if (cfg.objectId != -1 && !mountOf(scene, cfg)) return "the object it is mounted on no longer exists";
    if (cfg.target == SensorTarget::Object && !scene.getObject(cfg.targetId))
        return cfg.targetId == -1 ? "no object to detect selected" : "the object it detects no longer exists";
    if (sensorIsRay(cfg.type)) {
        if (cfg.range <= 0) return "beam range must be greater than 0";
        if (cfg.direction.isNull()) return "beam has no direction";
    } else if (!(cfg.objectBounds && cfg.objectId != -1)) {
        if (cfg.size.x() <= 0 || cfg.size.y() <= 0 || cfg.size.z() <= 0) return "zone size must be greater than 0";
    }
    return {};
}

QString summary(const SceneGraph& scene, const SensorConfig& cfg) {
    QString where = cfg.objectId == -1 ? QString("world") : objectName(scene, cfg.objectId);
    QString what;
    switch (cfg.target) {
        case SensorTarget::Any:         what = "anything"; break;
        case SensorTarget::RigidBodies: what = "rigid bodies"; break;
        case SensorTarget::Object:      what = cfg.targetId == -1 ? QString("(no object)") : objectName(scene, cfg.targetId); break;
    }
    if (sensorIsRay(cfg.type))
        return QString("%1 on %2, %3 m → %4").arg(cfg.type == SensorType::Distance ? "Distance" : "Beam", where)
            .arg(double(cfg.range), 0, 'f', 2).arg(what);
    const QString kind = cfg.type == SensorType::Position ? "Position zone" : "Zone";
    if (cfg.objectBounds && cfg.objectId != -1)
        return QString("%1 %2 → %3").arg(cfg.type == SensorType::Position ? "Position, touching" : "Touching", where, what);
    return QString("%1 on %2, %3×%4×%5 m → %6").arg(kind, where)
        .arg(double(cfg.size.x()), 0, 'g', 3).arg(double(cfg.size.y()), 0, 'g', 3)
        .arg(double(cfg.size.z()), 0, 'g', 3).arg(what);
}

} // namespace sensors
