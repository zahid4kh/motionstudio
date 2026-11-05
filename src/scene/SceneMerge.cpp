#include "SceneMerge.h"
#include "SceneGraph.h"
#include "Assembly.h"
#include <algorithm>
#include <functional>
#include <unordered_map>

namespace {

// A name free in both scenes, starting from `base`
QString freeName(const SceneGraph& target, const SceneGraph& source, const QString& base) {
    QString n = target.makeSymbolName(base);
    for (int i = 2; source.isSymbolNameTaken(n) || target.isSymbolNameTaken(n); ++i)
        n = target.makeSymbolName(QString("%1_%2").arg(base).arg(i));
    return n;
}

int remap(const std::unordered_map<int, int>& map, int id) {
    if (id == -1) return -1;
    auto it = map.find(id);
    return it != map.end() ? it->second : -2;   // a reference that doesn't resolve stays broken
}

} // namespace

std::vector<int> mergeScene(SceneGraph& target, SceneGraph& source, int parentId) {
    // 1. Free every clashing symbol name inside the component first
    std::vector<std::pair<QString, QString>> renames;
    for (auto& [id, s] : source.allSignals())
        if (target.isSymbolNameTaken(s.name)) renames.push_back({s.name, freeName(target, source, s.name)});
    for (auto& [id, m] : source.allMotors())
        if (target.isSymbolNameTaken(m.name)) renames.push_back({m.name, freeName(target, source, m.name)});
    for (auto& [id, q] : source.allSequences())
        if (target.isSymbolNameTaken(q.name)) renames.push_back({q.name, freeName(target, source, q.name)});
    for (auto& [id, q] : source.allSources())
        if (target.isSymbolNameTaken(q.name)) renames.push_back({q.name, freeName(target, source, q.name)});
    for (auto& [id, q] : source.allSinks())
        if (target.isSymbolNameTaken(q.name)) renames.push_back({q.name, freeName(target, source, q.name)});
    for (const auto& [from, to] : renames) {
        if (Signal* s = source.findSignal(from)) s->name = to;
        for (auto& [id, m] : source.allMotors()) if (m.name == from) source.getMotor(id)->name = to;
        for (auto& [id, q] : source.allSequences()) if (q.name == from) source.getSequence(id)->name = to;
        for (auto& [id, q] : source.allSources()) if (q.name == from) source.getSource(id)->name = to;
        for (auto& [id, q] : source.allSinks()) if (q.name == from) source.getSink(id)->name = to;
        source.renameSymbolReferences(from, to);
    }

    // 2. Objects, parents first
    std::unordered_map<int, int> objects;
    std::vector<int> roots;
    std::function<void(int, int)> copyObject = [&](int id, int newParent) {
        const SceneObject* o = source.getObject(id);
        if (!o) return;
        const int copy = target.addObject(o->name, o->type, newParent);
        SceneObject* c = target.getObject(copy);
        c->transform = o->transform;
        c->mesh = o->mesh;
        c->color = o->color;
        c->opacity = o->opacity;
        objects[id] = copy;
        if (newParent == parentId) roots.push_back(copy);
        for (int child : o->childIds) copyObject(child, copy);
        emit target.objectChanged(copy);
    };
    for (int r : source.rootObjects()) copyObject(r, parentId);

    // 3. Joints (couplings once all exist), actuators, rigid bodies, belts
    std::unordered_map<int, int> joints, motors, signalIds, poses;
    std::vector<int> jointIds;
    for (auto& [id, j] : source.allJoints()) jointIds.push_back(id);
    std::sort(jointIds.begin(), jointIds.end());
    for (int id : jointIds) {
        Joint j = *source.getJoint(id);
        const float value = j.currentValue;
        j.parentBodyId = remap(objects, j.parentBodyId);
        j.childBodyId = remap(objects, j.childBodyId);
        const int copy = target.addJoint(j);
        target.getJoint(copy)->currentValue = value;
        joints[id] = copy;
    }
    for (auto& [id, copy] : joints) {
        Joint* j = target.getJoint(copy);
        j->coupledTo = j->coupledTo == -1 ? -1 : remap(joints, j->coupledTo);
        if (j->coupledTo == -2) j->coupledTo = -1;
    }
    for (int id : source.motorIdsSorted()) {
        Motor m = *source.getMotor(id);
        m.jointId = remap(joints, m.jointId);
        if (m.jointId < 0) continue;
        motors[id] = target.addMotor(m);
    }
    for (auto& [oid, rb] : source.allRigidBodies()) {
        const int to = remap(objects, oid);
        if (to < 0) continue;
        target.addRigidBody(to);
        RigidBody copy = rb;
        copy.objectId = to;
        *target.getRigidBody(to) = copy;
    }
    for (auto& [oid, belt] : source.allConveyorBelts()) {
        const int to = remap(objects, oid);
        if (to < 0) continue;
        target.addConveyorBelt(to);
        *target.getConveyorBelt(to) = belt;
    }

    // 4. Signals (sensor mounts remapped), poses, sequences
    for (auto& [id, s] : source.allSignals()) {
        Signal copy = s;
        copy.ioAddress = -1;   // I/O maps belong to the project, not the component
        copy.sensor.objectId = remap(objects, s.sensor.objectId);
        copy.sensor.targetId = remap(objects, s.sensor.targetId);
        signalIds[id] = target.addSignal(copy);
    }
    for (auto& [id, p] : source.allPoses()) {
        Pose copy = p;
        copy.jointValues.clear();
        copy.toolId = remap(objects, p.toolId);
        copy.frameId = remap(objects, p.frameId);
        for (auto& [jid, v] : p.jointValues) {
            const int to = remap(joints, jid);
            if (to >= 0) copy.jointValues[to] = v;
        }
        poses[id] = target.addPose(copy);
    }
    std::unordered_map<int, int> sources;
    for (auto& [id, src] : source.allSources()) {
        PartSource copy = src;
        copy.templateId = remap(objects, src.templateId);
        sources[id] = target.addSource(copy);
    }
    for (auto& [id, sink] : source.allSinks()) {
        PartSink copy = sink;
        copy.zone.objectId = remap(objects, sink.zone.objectId);
        copy.sourceId = remap(sources, sink.sourceId);
        target.addSink(copy);
    }
    for (auto& [id, q] : source.allSequences()) {
        Sequence copy = q;
        std::unordered_map<int, int> ops;
        for (auto& op : copy.operations) ops[op.id] = target.allocateId();
        for (auto& op : copy.operations) {
            op.id = ops.at(op.id);
            op.actuatorId = remap(motors, op.actuatorId);
            op.poseId = remap(poses, op.poseId);
            op.gripperId = remap(objects, op.gripperId);
            op.partId = remap(objects, op.partId);
            op.toolId = remap(objects, op.toolId);
            op.sensorId = remap(signalIds, op.sensorId);
            for (int& p : op.predecessors) p = remap(ops, p);
        }
        target.addSequence(copy);
    }
    for (auto& [id, m] : source.allMates()) {
        Mate copy = m;
        copy.moving.objectId = remap(objects, m.moving.objectId);
        copy.fixed.objectId = remap(objects, m.fixed.objectId);
        if (copy.moving.objectId < 0 || copy.fixed.objectId < 0) continue;
        for (auto& [tid, t] : target.allMates())
            if (t.name == copy.name) { copy.name = assembly::nextName(target); break; }
        target.addMate(copy);
    }
    target.applyCouplings();
    target.notifyAutomationChanged();
    return roots;
}
