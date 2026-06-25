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

void SceneGraph::clear() {
    m_objects.clear();
    m_joints.clear();
    m_motors.clear();
    m_roots.clear();
    m_selectedId = -1;
    m_nextId     = 1;
    emit sceneCleared();
}
