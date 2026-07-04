#include "SceneGraph.h"
#include <QtMath>
#include <algorithm>

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

void SceneGraph::removeObject(int id) {
    auto it = m_objects.find(id);
    if (it == m_objects.end()) return;

    SceneObject& obj = it->second;

    // Recursively remove children
    auto childsCopy = obj.childIds;
    for (int cid : childsCopy) removeObject(cid);

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
    m_motors[id] = motor;
    emit motorAdded(id);
    return id;
}

void SceneGraph::removeMotor(int id) {
    m_motors.erase(id);
}

Motor* SceneGraph::getMotor(int id) {
    auto it = m_motors.find(id);
    return it != m_motors.end() ? &it->second : nullptr;
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
            if (joint.type == JointType::Revolute) {
                jointT.rotate(joint.currentValue, joint.axis);
            } else {
                jointT.translate(joint.axis * joint.currentValue);
            }
            break;
        }
    }

    return worldTransform(obj->parentId) * jointT * local;
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
    m_roots.clear();
    m_selectedId = -1;
    m_nextId     = 1;
    emit sceneCleared();
}
