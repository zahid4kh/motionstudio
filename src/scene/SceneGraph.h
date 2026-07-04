#pragma once
#include "SceneObject.h"
#include "Joint.h"
#include "Motor.h"
#include "RigidBody.h"
#include "ConveyorBelt.h"
#include <QObject>
#include <QMatrix4x4>
#include <unordered_map>
#include <vector>

class SceneGraph : public QObject {
    Q_OBJECT
public:
    explicit SceneGraph(QObject* parent = nullptr);

    // Objects
    int              addObject(const QString& name, PrimitiveType type, int parentId = -1);
    void             removeObject(int id);
    SceneObject*     getObject(int id);
    const SceneObject* getObject(int id) const;
    const std::vector<int>&                       rootObjects() const { return m_roots; }
    const std::unordered_map<int, SceneObject>&   allObjects()  const { return m_objects; }

    // Joints
    int     addJoint(Joint joint);
    void    removeJoint(int id);
    Joint*  getJoint(int id);
    const Joint* getJoint(int id) const;
    const std::unordered_map<int, Joint>& allJoints() const { return m_joints; }
    Joint*  jointForChild(int childBodyId);

    // Motors
    int     addMotor(Motor motor);
    void    removeMotor(int id);
    Motor*  getMotor(int id);
    const std::unordered_map<int, Motor>& allMotors() const { return m_motors; }

    // Rigid bodies
    void        addRigidBody(int objectId);
    void        removeRigidBody(int objectId);
    RigidBody*  getRigidBody(int objectId);
    const std::unordered_map<int, RigidBody>& allRigidBodies() const { return m_rigidBodies; }

    // Combine multiple motionless (no incoming joint) objects sharing the
    // same parent into a new group object carrying a single RigidBody.
    enum class CombineResult { Ok, TooFewEligible, MixedParents };
    CombineResult combineIntoRigidBody(const std::vector<int>& objectIds,
                                        const QString& groupName, int& outGroupId);

    // Manually re-parent one object under another (e.g. to fix a STEP import's
    // raw CAD assembly nesting not matching the intended kinematic structure).
    // Preserves the child's current world position/orientation by baking the
    // frame change into its local transform, and drops any existing joint on
    // the child since a stored pivot/axis is only meaningful relative to the
    // old parent's frame.
    enum class ReparentResult { Ok, SameObject, WouldCreateCycle };
    ReparentResult setParent(int childId, int newParentId);

    // Conveyor belts
    void           addConveyorBelt(int objectId);
    void           removeConveyorBelt(int objectId);
    ConveyorBelt*  getConveyorBelt(int objectId);
    const std::unordered_map<int, ConveyorBelt>& allConveyorBelts() const { return m_conveyorBelts; }

    // Floor plane
    bool floorEnabled() const { return m_floorEnabled; }
    void setFloorEnabled(bool v);

    // Selection
    void selectObject(int id);
    void deselectAll();
    int  selectedObjectId() const { return m_selectedId; }

    // Transform helpers
    QMatrix4x4 worldTransform(int id) const;
    QMatrix4x4 localTransform(int id) const;

    void clear();

private:
    void reparentObject(int id, int newParentId);

signals:
    void objectAdded(int id);
    void objectRemoved(int id);
    void objectChanged(int id);
    void selectionChanged(int id);
    void jointAdded(int id);
    void motorAdded(int id);
    void rigidBodyAdded(int objectId);
    void rigidBodyRemoved(int objectId);
    void rigidBodyChanged(int objectId);
    void conveyorBeltAdded(int objectId);
    void conveyorBeltRemoved(int objectId);
    void conveyorBeltChanged(int objectId);
    void floorChanged(bool enabled);
    void sceneCleared();

private:
    std::unordered_map<int, SceneObject>   m_objects;
    std::unordered_map<int, Joint>         m_joints;
    std::unordered_map<int, Motor>         m_motors;
    std::unordered_map<int, RigidBody>     m_rigidBodies;
    std::unordered_map<int, ConveyorBelt>  m_conveyorBelts;
    std::vector<int>                       m_roots;
    int  m_nextId      = 1;
    int  m_selectedId  = -1;
    bool m_floorEnabled = true;
};
