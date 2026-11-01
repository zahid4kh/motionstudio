#include "SceneGraph.h"
#include "../automation/Expression.h"
#include <QRegularExpression>
#include <QtMath>
#include <algorithm>
#include <cmath>

// Decomposes an affine TRS matrix (built the same way as localTransform():
// T * Rx(a) * Ry(b) * Rz(c) * S) back into position / Euler-XYZ-degrees /
// per-axis scale. Mirrors the rotation-matrix-to-Euler formula used for
// MuJoCo quaternions in SimulationEngine::quatToEulerXYZ.
static void decomposeMatrix(const QMatrix4x4& m, QVector3D& pos, QVector3D& eulerDeg, QVector3D& scale) {
    pos = m.column(3).toVector3D();
    QVector3D c0 = m.column(0).toVector3D();
    QVector3D c1 = m.column(1).toVector3D();
    QVector3D c2 = m.column(2).toVector3D();
    scale = { c0.length(), c1.length(), c2.length() };
    QVector3D r0 = scale.x() > 1e-8f ? c0 / scale.x() : QVector3D(1, 0, 0);
    QVector3D r1 = scale.y() > 1e-8f ? c1 / scale.y() : QVector3D(0, 1, 0);
    QVector3D r2 = scale.z() > 1e-8f ? c2 / scale.z() : QVector3D(0, 0, 1);

    float r00 = r0.x();
    float r01 = r1.x(), r11 = r1.y(), r21 = r1.z();
    float r02 = r2.x(), r12 = r2.y(), r22 = r2.z();

    float b = asinf(std::clamp(r02, -1.0f, 1.0f));
    float a, c;
    if (fabsf(cosf(b)) > 1e-6f) {
        a = atan2f(-r12, r22);
        c = atan2f(-r01, r00);
    } else {
        a = atan2f(r21, r11);
        c = 0.0f;
    }
    const float kR2D = 180.0f / float(M_PI);
    eulerDeg = { a * kR2D, b * kR2D, c * kR2D };
}

SceneGraph::SceneGraph(QObject* parent) : QObject(parent) {}

int SceneGraph::addObject(const QString& name, PrimitiveType type, int parentId) {
    SceneObject obj;
    obj.id       = m_nextId++;
    obj.name     = name;
    obj.type     = type;
    obj.parentId = parentId;

    if (parentId != -1) {
        if (auto* p = getObject(parentId)) {
            p->childIds.push_back(obj.id);
        }
    } else {
        m_roots.push_back(obj.id);
    }

    m_objects[obj.id] = obj;
    emit objectAdded(obj.id);
    return obj.id;
}

int SceneGraph::duplicateObject(int id) {
    const SceneObject* source = getObject(id);
    if (!source) return -1;

    std::unordered_map<int, int> copyOf;   // original object id → copy id
    auto copySubtree = [&](auto& self, int originalId, int parentId) -> void {
        const SceneObject original = m_objects.at(originalId);   // by value: addObject may rehash
        const int copyId = addObject(originalId == id ? original.name + " copy" : original.name,
                                     original.type, parentId);
        SceneObject& copy = m_objects.at(copyId);
        copy.transform = original.transform;
        copy.mesh      = original.mesh;
        copy.color     = original.color;
        copyOf[originalId] = copyId;
        emit objectChanged(copyId);
        for (int childId : original.childIds) self(self, childId, copyId);
    };
    copySubtree(copySubtree, id, source->parentId);

    // Joints moving a copied object; a joint to a parent outside the subtree
    // (the copy's root hanging off the same parent) keeps that parent
    std::vector<int> jointIds;
    for (auto& [jid, j] : m_joints)
        if (copyOf.count(j.childBodyId)) jointIds.push_back(jid);
    std::sort(jointIds.begin(), jointIds.end());
    std::unordered_map<int, int> jointCopyOf;
    for (int jid : jointIds) {
        Joint joint = m_joints.at(jid);
        const float currentValue = joint.currentValue;
        joint.childBodyId = copyOf.at(joint.childBodyId);
        if (copyOf.count(joint.parentBodyId)) joint.parentBodyId = copyOf.at(joint.parentBodyId);
        const int copyJointId = addJoint(joint);
        m_joints.at(copyJointId).currentValue = currentValue;
        jointCopyOf[jid] = copyJointId;
        if (const Motor* motor = motorForJoint(jid)) {
            Motor copy = *motor;
            copy.jointId = copyJointId;
            addMotor(copy);   // the name is taken, so addMotor picks a free one (J1 → J1_2)
        }
    }
    // A copied follower follows the copy of its master when that was copied too
    for (auto& [original, copy] : jointCopyOf) {
        Joint& j = m_joints.at(copy);
        auto it = jointCopyOf.find(j.coupledTo);
        if (it != jointCopyOf.end()) j.coupledTo = it->second;
    }

    for (auto& [originalId, copyId] : copyOf) {
        if (const RigidBody* rb = getRigidBody(originalId)) {
            RigidBody copy = *rb;
            addRigidBody(copyId);
            copy.objectId = copyId;
            *getRigidBody(copyId) = copy;
            emit rigidBodyChanged(copyId);
        }
        if (const ConveyorBelt* belt = getConveyorBelt(originalId)) {
            const ConveyorBelt copy = *belt;
            addConveyorBelt(copyId);
            *getConveyorBelt(copyId) = copy;
            emit conveyorBeltChanged(copyId);
        }
    }
    return copyOf.at(id);
}

void SceneGraph::removeObject(int id) {
    auto it = m_objects.find(id);
    if (it == m_objects.end()) return;

    SceneObject& obj = it->second;

    if (m_rigidBodies.count(id) && !obj.childIds.empty()) {
        // Rigid body group: this object only exists to bundle its children
        // under one RigidBody, so deleting it should ungroup them back into
        // the surrounding hierarchy rather than deleting them too. Bake this
        // group's own transform into each child first, so if the merged unit
        // was moved as a whole, the children keep their current world
        // position/orientation instead of snapping back to pre-merge values.
        QMatrix4x4 groupLocal = localTransform(id);
        auto childrenCopy = obj.childIds;
        for (int cid : childrenCopy) {
            if (SceneObject* child = getObject(cid)) {
                QVector3D pos, eulerDeg, scale;
                decomposeMatrix(groupLocal * localTransform(cid), pos, eulerDeg, scale);
                child->transform.position = pos;
                child->transform.rotation = eulerDeg;
                child->transform.scale    = scale;
            }
            reparentObject(cid, obj.parentId);
        }
    } else {
        // Recursively remove children
        auto childsCopy = obj.childIds;
        for (int cid : childsCopy) removeObject(cid);
    }

    // Detach from parent
    if (obj.parentId != -1) {
        if (auto* p = getObject(obj.parentId)) {
            auto& c = p->childIds;
            c.erase(std::remove(c.begin(), c.end(), id), c.end());
        }
    } else {
        m_roots.erase(std::remove(m_roots.begin(), m_roots.end(), id), m_roots.end());
    }

    // Remove joints referencing this object
    std::vector<int> jointsToRemove;
    for (auto& [jid, j] : m_joints) {
        if (j.parentBodyId == id || j.childBodyId == id)
            jointsToRemove.push_back(jid);
    }
    for (int jid : jointsToRemove) removeJoint(jid);

    if (m_selectedId == id) m_selectedId = -1;

    m_rigidBodies.erase(id);
    m_conveyorBelts.erase(id);
    m_objects.erase(it);
    emit objectRemoved(id);
}

SceneObject* SceneGraph::getObject(int id) {
    auto it = m_objects.find(id);
    return it != m_objects.end() ? &it->second : nullptr;
}

const SceneObject* SceneGraph::getObject(int id) const {
    auto it = m_objects.find(id);
    return it != m_objects.end() ? &it->second : nullptr;
}

int SceneGraph::addJoint(Joint joint) {
    joint.id = m_nextId++;
    int id = joint.id;
    joint.currentValue = joint.initialAngle;
    m_joints[id] = joint;
    emit jointAdded(id);
    return id;
}

void SceneGraph::removeJoint(int id) {
    // Remove motors pointing at this joint
    std::vector<int> motorsToRemove;
    for (auto& [mid, m] : m_motors) {
        if (m.jointId == id) motorsToRemove.push_back(mid);
    }
    for (int mid : motorsToRemove) removeMotor(mid);
    m_joints.erase(id);
    for (auto& [pid, pose] : m_poses) pose.jointValues.erase(id);
    for (auto& [jid, j] : m_joints)
        if (j.coupledTo == id) j.coupledTo = -1;   // its master is gone
}

Joint* SceneGraph::getJoint(int id) {
    auto it = m_joints.find(id);
    return it != m_joints.end() ? &it->second : nullptr;
}

const Joint* SceneGraph::getJoint(int id) const {
    auto it = m_joints.find(id);
    return it != m_joints.end() ? &it->second : nullptr;
}

Joint* SceneGraph::jointForChild(int childBodyId) {
    for (auto& [id, j] : m_joints) {
        if (j.childBodyId == childBodyId) return &j;
    }
    return nullptr;
}

int SceneGraph::addMotor(Motor motor) {
    motor.id = m_nextId++;
    int id = motor.id;
    if (!expr::isValidName(motor.name) || isSymbolNameTaken(motor.name, id))
        motor.name = makeSymbolName(motor.name.isEmpty() ? QString("Actuator") : motor.name, id);
    m_motors[id] = motor;
    emit motorAdded(id);
    emit automationChanged();
    return id;
}

void SceneGraph::removeMotor(int id) {
    if (!m_motors.erase(id)) return;
    emit automationChanged();
}

Motor* SceneGraph::getMotor(int id) {
    auto it = m_motors.find(id);
    return it != m_motors.end() ? &it->second : nullptr;
}

const Motor* SceneGraph::getMotor(int id) const {
    auto it = m_motors.find(id);
    return it != m_motors.end() ? &it->second : nullptr;
}

int SceneGraph::addActuatorForJoint(int jointId) {
    const Joint* j = getJoint(jointId);
    if (!j) return -1;
    if (Motor* existing = motorForJoint(jointId)) return existing->id;
    int n = 1;
    while (isSymbolNameTaken(QString("J%1").arg(n))) ++n;
    Motor m;
    m.name     = QString("J%1").arg(n);
    m.jointId  = jointId;
    m.velocity = j->type == JointType::Prismatic ? 0.1f : 45.0f;
    return addMotor(m);
}

Motor* SceneGraph::motorForJoint(int jointId) {
    for (auto& [id, m] : m_motors)
        if (m.jointId == jointId) return &m;
    return nullptr;
}

const Motor* SceneGraph::motorForJoint(int jointId) const {
    for (auto& [id, m] : m_motors)
        if (m.jointId == jointId) return &m;
    return nullptr;
}

std::vector<int> SceneGraph::motorIdsSorted() const {
    std::vector<int> ids;
    for (auto& [id, _] : m_motors) ids.push_back(id);
    std::sort(ids.begin(), ids.end());
    return ids;
}

// ── Automation definitions ───────────────────────────────────────────────────

int SceneGraph::addSignal(Signal sig) {
    sig.id = m_nextId++;
    if (!expr::isValidName(sig.name) || isSymbolNameTaken(sig.name, sig.id))
        sig.name = makeSymbolName(sig.name.isEmpty() ? QString("Signal") : sig.name, sig.id);
    if (sig.kind == SignalKind::Sensor) sig.type = SignalType::Bool;
    sig.initialValue = coerceSignal(sig.type, sig.initialValue);
    m_signals[sig.id] = sig;
    emit automationChanged();
    return sig.id;
}

void SceneGraph::removeSignal(int id) {
    if (!m_signals.erase(id)) return;
    emit automationChanged();
}

Signal* SceneGraph::getSignal(int id) {
    auto it = m_signals.find(id);
    return it != m_signals.end() ? &it->second : nullptr;
}

Signal* SceneGraph::findSignal(const QString& name) {
    for (auto& [id, s] : m_signals)
        if (s.name == name) return &s;
    return nullptr;
}

int SceneGraph::addSequence(Sequence seq) {
    seq.id = m_nextId++;
    if (!expr::isValidName(seq.name) || isSymbolNameTaken(seq.name, seq.id))
        seq.name = makeSymbolName(seq.name.isEmpty() ? QString("Sequence") : seq.name, seq.id);
    for (auto& op : seq.operations)
        if (op.id == -1) op.id = m_nextId++;
    m_sequences[seq.id] = seq;
    emit automationChanged();
    return seq.id;
}

void SceneGraph::removeSequence(int id) {
    if (!m_sequences.erase(id)) return;
    emit automationChanged();
}

Sequence* SceneGraph::getSequence(int id) {
    auto it = m_sequences.find(id);
    return it != m_sequences.end() ? &it->second : nullptr;
}

int SceneGraph::addPose(Pose pose) {
    pose.id = m_nextId++;
    m_poses[pose.id] = pose;
    emit automationChanged();
    return pose.id;
}

void SceneGraph::removePose(int id) {
    if (!m_poses.erase(id)) return;
    emit automationChanged();
}

Pose* SceneGraph::getPose(int id) {
    auto it = m_poses.find(id);
    return it != m_poses.end() ? &it->second : nullptr;
}

const Pose* SceneGraph::getPose(int id) const {
    auto it = m_poses.find(id);
    return it != m_poses.end() ? &it->second : nullptr;
}

bool SceneGraph::isSymbolNameTaken(const QString& name, int exceptId) const {
    for (auto& [id, s] : m_signals)   if (id != exceptId && s.name == name) return true;
    for (auto& [id, m] : m_motors)    if (id != exceptId && m.name == name) return true;
    for (auto& [id, q] : m_sequences) if (id != exceptId && q.name == name) return true;
    return false;
}

QString SceneGraph::makeSymbolName(const QString& base, int exceptId) const {
    QString n = base;
    n.replace(QRegularExpression("[^A-Za-z0-9_]"), "_");
    if (n.isEmpty() || n[0].isDigit()) n.prepend('_');
    if (!expr::isValidName(n)) n += "_";
    if (!isSymbolNameTaken(n, exceptId)) return n;
    for (int i = 2;; ++i) {
        QString c = QString("%1_%2").arg(n).arg(i);
        if (!isSymbolNameTaken(c, exceptId)) return c;
    }
}

void SceneGraph::renameSymbolReferences(const QString& oldName, const QString& newName) {
    auto fix = [&](QString& s) { s = expr::renameIdentifier(s, oldName, newName); };
    for (auto& [id, s] : m_signals) fix(s.expression);
    for (auto& [id, m] : m_motors) { fix(m.targetExpr); fix(m.velocityExpr); fix(m.activeExpr); }
    for (auto& [id, b] : m_conveyorBelts) fix(b.speedExpr);
    for (auto& [id, q] : m_sequences) {
        fix(q.enableCondition);
        fix(q.startCondition);
        for (auto& op : q.operations) {
            fix(op.target); fix(op.velocity); fix(op.duration); fix(op.condition);
            fix(op.startCondition); fix(op.onStart); fix(op.onFinish);
        }
    }
    emit automationChanged();
}

void SceneGraph::selectObject(int id) {
    if (m_selectedId != -1) {
        if (auto* o = getObject(m_selectedId)) o->selected = false;
    }
    m_selectedId = id;
    if (id != -1) {
        if (auto* o = getObject(id)) o->selected = true;
    }
    emit selectionChanged(id);
}

void SceneGraph::deselectAll() {
    selectObject(-1);
}

QMatrix4x4 SceneGraph::localTransform(int id) const {
    const SceneObject* obj = getObject(id);
    if (!obj) return {};
    QMatrix4x4 m;
    m.translate(obj->transform.position);
    m.rotate(obj->transform.rotation.x(), 1, 0, 0);
    m.rotate(obj->transform.rotation.y(), 0, 1, 0);
    m.rotate(obj->transform.rotation.z(), 0, 0, 1);
    m.scale(obj->transform.scale);
    return m;
}

QMatrix4x4 SceneGraph::worldTransform(int id) const {
    const SceneObject* obj = getObject(id);
    if (!obj) return {};

    QMatrix4x4 local = localTransform(id);

    if (obj->parentId == -1) return local;

    // Apply joint offset between parent and this object
    QMatrix4x4 jointT;
    for (auto& [jid, joint] : m_joints) {
        if (joint.childBodyId == id) {
            jointT = jointMotion(joint);
            break;
        }
    }

    return worldTransform(obj->parentId) * jointT * local;
}

int SceneGraph::colorSource(int id) const {
    for (const SceneObject* o = getObject(id); o; o = getObject(o->parentId))
        if (o->color.isValid()) return o->id;
    return -1;
}

QColor SceneGraph::colorOf(int id) const {
    const SceneObject* o = getObject(colorSource(id));
    return o ? o->color : QColor();
}

void SceneGraph::setWorldTransform(int id, const QMatrix4x4& world) {
    SceneObject* obj = getObject(id);
    if (!obj) return;
    QMatrix4x4 parentWorld;
    if (obj->parentId != -1) {
        parentWorld = worldTransform(obj->parentId);
        for (auto& [jid, joint] : m_joints)
            if (joint.childBodyId == id) { parentWorld = parentWorld * jointMotion(joint); break; }
    }
    decomposeMatrix(parentWorld.inverted() * world, obj->transform.position,
                    obj->transform.rotation, obj->transform.scale);
}

void SceneGraph::applyCouplings() {
    // Masters before followers: a chain of n couplings settles in n passes
    // (a loop, which validation reports, stops after as many passes as joints)
    for (size_t pass = 0; pass <= m_joints.size(); ++pass) {
        bool changed = false;
        for (auto& [id, j] : m_joints) {
            if (j.coupledTo == -1 || j.coupledTo == id) continue;
            auto master = m_joints.find(j.coupledTo);
            if (master == m_joints.end()) continue;
            const float v = j.ratio * master->second.currentValue + j.offset;
            if (v != j.currentValue) { j.currentValue = v; changed = true; }
        }
        if (!changed) break;
    }
}

Transform SceneGraph::toTransform(const QMatrix4x4& m) {
    Transform t;
    decomposeMatrix(m, t.position, t.rotation, t.scale);
    return t;
}

QMatrix4x4 SceneGraph::jointMotion(const Joint& joint) {
    QMatrix4x4 m;
    if (joint.type == JointType::Revolute) {
        m.translate(joint.pivot);
        m.rotate(joint.currentValue, joint.axis);
        m.translate(-joint.pivot);
    } else {
        m.translate(joint.axis * joint.currentValue);
    }
    return m;
}

void SceneGraph::addRigidBody(int objectId) {
    if (m_rigidBodies.count(objectId)) return;
    RigidBody rb;
    rb.objectId = objectId;
    if (const SceneObject* obj = getObject(objectId)) {
        auto& c = rb.collision;
        switch (obj->type) {
            case PrimitiveType::Cube:
            case PrimitiveType::Basket:
                c.type  = CollisionType::Box;
                c.halfW = obj->transform.scale.x() / 2.0f;
                c.halfH = obj->transform.scale.y() / 2.0f;
                c.halfD = obj->transform.scale.z() / 2.0f;
                break;
            case PrimitiveType::Sphere:
                c.type   = CollisionType::Sphere;
                c.radius = obj->transform.scale.x() / 2.0f;
                break;
            case PrimitiveType::Cylinder:
                c.type   = CollisionType::Cylinder;
                c.radius = obj->transform.scale.x() / 2.0f;
                c.height = obj->transform.scale.y();
                break;
            case PrimitiveType::Capsule:
                c.type   = CollisionType::Capsule;
                c.radius = obj->transform.scale.x() / 2.0f;
                c.height = obj->transform.scale.y();
                break;
            case PrimitiveType::ImportedMesh: {
                QVector3D he = (obj->aabbMax() - obj->aabbMin()) * 0.5f * obj->transform.scale;
                c.type   = CollisionType::Box;
                c.halfW  = he.x();
                c.halfH  = he.y();
                c.halfD  = he.z();
                c.offset = (obj->aabbMax() + obj->aabbMin()) * 0.5f * obj->transform.scale;
                break;
            }
        }
    }
    m_rigidBodies[objectId] = rb;
    emit rigidBodyAdded(objectId);
}

void SceneGraph::addConveyorBelt(int objectId) {
    if (m_conveyorBelts.count(objectId)) return;
    m_conveyorBelts[objectId] = ConveyorBelt{};
    emit conveyorBeltAdded(objectId);
}

void SceneGraph::removeConveyorBelt(int objectId) {
    if (!m_conveyorBelts.count(objectId)) return;
    m_conveyorBelts.erase(objectId);
    emit conveyorBeltRemoved(objectId);
}

ConveyorBelt* SceneGraph::getConveyorBelt(int objectId) {
    auto it = m_conveyorBelts.find(objectId);
    return it != m_conveyorBelts.end() ? &it->second : nullptr;
}

void SceneGraph::setFloorEnabled(bool v) {
    if (m_floorEnabled == v) return;
    m_floorEnabled = v;
    emit floorChanged(v);
}

void SceneGraph::removeRigidBody(int objectId) {
    if (!m_rigidBodies.count(objectId)) return;
    m_rigidBodies.erase(objectId);
    emit rigidBodyRemoved(objectId);
}

RigidBody* SceneGraph::getRigidBody(int objectId) {
    auto it = m_rigidBodies.find(objectId);
    return it != m_rigidBodies.end() ? &it->second : nullptr;
}

void SceneGraph::reparentObject(int id, int newParentId) {
    SceneObject* obj = getObject(id);
    if (!obj) return;

    if (obj->parentId == -1) {
        m_roots.erase(std::remove(m_roots.begin(), m_roots.end(), id), m_roots.end());
    } else if (auto* oldParent = getObject(obj->parentId)) {
        auto& c = oldParent->childIds;
        c.erase(std::remove(c.begin(), c.end(), id), c.end());
    }

    obj->parentId = newParentId;
    if (newParentId == -1) {
        m_roots.push_back(id);
    } else if (auto* newParent = getObject(newParentId)) {
        newParent->childIds.push_back(id);
    }
}

SceneGraph::ReparentResult SceneGraph::setParent(int childId, int newParentId) {
    if (childId == newParentId) return ReparentResult::SameObject;
    SceneObject* child = getObject(childId);
    if (!child) return ReparentResult::SameObject;

    for (int p = newParentId; p != -1; ) {
        if (p == childId) return ReparentResult::WouldCreateCycle;
        const SceneObject* po = getObject(p);
        p = po ? po->parentId : -1;
    }

    if (child->parentId == newParentId) return ReparentResult::Ok;

    QMatrix4x4 childWorld = worldTransform(childId);

    if (Joint* j = jointForChild(childId)) removeJoint(j->id);

    reparentObject(childId, newParentId);

    QMatrix4x4 newParentWorld = newParentId != -1 ? worldTransform(newParentId) : QMatrix4x4();
    QMatrix4x4 newLocal = newParentWorld.inverted() * childWorld;

    QVector3D pos, eulerDeg, scale;
    decomposeMatrix(newLocal, pos, eulerDeg, scale);
    child->transform.position = pos;
    child->transform.rotation = eulerDeg;
    child->transform.scale    = scale;

    emit objectChanged(childId);
    // Structure changed (moved to a new place in the tree) — reuse the same
    // "please rebuild" signal combineIntoRigidBody() uses for the same reason.
    emit objectAdded(childId);

    return ReparentResult::Ok;
}

SceneGraph::CombineResult SceneGraph::combineIntoRigidBody(const std::vector<int>& objectIds,
                                                             const QString& groupName, int& outGroupId) {
    outGroupId = -1;

    // Only motionless parts (no incoming joint) are eligible for merging.
    std::vector<int> eligible;
    for (int id : objectIds) {
        if (!getObject(id)) continue;
        if (jointForChild(id)) continue;
        eligible.push_back(id);
    }
    if (eligible.size() < 2) return CombineResult::TooFewEligible;

    int commonParent = getObject(eligible.front())->parentId;
    for (int id : eligible) {
        if (getObject(id)->parentId != commonParent) return CombineResult::MixedParents;
    }

    int groupId = addObject(groupName, PrimitiveType::ImportedMesh, commonParent);

    float totalMass = 0.0f;
    bool  hadRigidBody = false;
    for (int id : eligible) {
        if (RigidBody* rb = getRigidBody(id)) {
            totalMass += rb->mass;
            hadRigidBody = true;
            removeRigidBody(id);
        }
    }

    for (int id : eligible) reparentObject(id, groupId);

    addRigidBody(groupId);
    if (hadRigidBody) {
        if (RigidBody* rb = getRigidBody(groupId)) rb->mass = totalMass;
        emit rigidBodyChanged(groupId);
    }

    selectObject(groupId);
    // Re-announce the group now that its children are attached, so listeners
    // that rebuild their view from the tree (e.g. HierarchyPanel) pick up the
    // final structure rather than the empty group emitted by addObject().
    emit objectAdded(groupId);

    outGroupId = groupId;
    return CombineResult::Ok;
}

void SceneGraph::clear() {
    m_objects.clear();
    m_joints.clear();
    m_motors.clear();
    m_rigidBodies.clear();
    m_conveyorBelts.clear();
    m_signals.clear();
    m_sequences.clear();
    m_poses.clear();
    m_roots.clear();
    m_selectedId = -1;
    m_nextId     = 1;
    emit sceneCleared();
}
