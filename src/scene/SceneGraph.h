#pragma once
#include "SceneObject.h"
#include "Joint.h"
#include "Motor.h"
#include "RigidBody.h"
#include "ConveyorBelt.h"
#include "Mate.h"
#include "../automation/Automation.h"
#include <QObject>
#include <QMatrix4x4>
#include <unordered_map>
#include <map>
#include <vector>

class SceneGraph : public QObject {
    Q_OBJECT
public:
    explicit SceneGraph(QObject* parent = nullptr);

    // Objects
    int              addObject(const QString& name, PrimitiveType type, int parentId = -1);
    void             removeObject(int id);
    // Copies the object and its subtree next to the original: same parent
    // and transform, shared mesh geometry, plus the joints into the copied
    // objects (with their actuators, renamed to free symbols), rigid bodies
    // and conveyor belts. Poses and sequences are not copied. Returns the
    // copy's id, or -1.
    int              duplicateObject(int id);
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
    const Motor* getMotor(int id) const;
    const std::unordered_map<int, Motor>& allMotors() const { return m_motors; }
    // Creates an actuator named J1, J2… for `jointId` with sensible defaults
    int     addActuatorForJoint(int jointId);
    Motor*  motorForJoint(int jointId);
    const Motor* motorForJoint(int jointId) const;
    std::vector<int> motorIdsSorted() const;   // stable display order

    // ── Automation definitions (see automation/Automation.h) ────────────────
    // Signals, actuators (motors) and sequences share one symbol namespace
    // used by expressions.
    int     addSignal(Signal s);
    void    removeSignal(int id);
    Signal* getSignal(int id);
    Signal* findSignal(const QString& name);
    const std::map<int, Signal>& allSignals() const { return m_signals; }

    int       addSequence(Sequence s);
    void      removeSequence(int id);
    Sequence* getSequence(int id);
    const std::map<int, Sequence>& allSequences() const { return m_sequences; }

    // Object sources and sinks (part flow); their names are symbols too
    int         addSource(PartSource s);
    void        removeSource(int id);
    PartSource* getSource(int id);
    const PartSource* getSource(int id) const;
    const std::map<int, PartSource>& allSources() const { return m_sources; }
    int         addSink(PartSink s);
    void        removeSink(int id);
    PartSink*   getSink(int id);
    const PartSink* getSink(int id) const;
    const std::map<int, PartSink>& allSinks() const { return m_sinks; }

    // ── Simulation-time objects (see SceneObject::copyOf / parked) ──────────
    // These change the scene without the usual signals: they are never
    // saved, never undone and never shown in the hierarchy.
    // Copies `templateId` and its subtree as a new top-level part placed
    // where the template is now (joint motion baked in), with its rigid
    // bodies; returns the copy's id
    int   spawnCopy(int templateId, int sourceId, const QString& name);
    // Takes the object and its subtree out of the run, or back in
    void  setParked(int id, bool parked);
    bool  isParked(int id) const { const SceneObject* o = getObject(id); return o && o->parked; }
    // Removes every copy and brings every parked object back
    void  clearRuntimeObjects();
    bool  hasRuntimeObjects() const;
    // The top of the copy `id` belongs to, or -1 when it is not a copy
    int   copyRoot(int id) const;

    int    addPose(Pose p);
    void   removePose(int id);
    Pose*  getPose(int id);
    const Pose* getPose(int id) const;
    const std::map<int, Pose>& allPoses() const { return m_poses; }

    // Assembly mates (see scene/Assembly.h for solving them). Removing an
    // object removes the mates that refer to it or its subtree.
    int   addMate(Mate m);
    void  removeMate(int id);
    Mate* getMate(int id);
    const Mate* getMate(int id) const;
    const std::map<int, Mate>& allMates() const { return m_mates; }
    // Emits matesChanged() — call after editing a mate in place
    void  notifyMatesChanged() { emit matesChanged(); }

    int  allocateId() { return m_nextId++; }

    // True when `name` is used by a signal, actuator or sequence other than
    // the entity with id `exceptId`.
    bool    isSymbolNameTaken(const QString& name, int exceptId = -1) const;
    // Turns `base` into a valid, unused symbol name (Motor_NS-16 → Motor_NS_16, J1 → J1_2 …)
    QString makeSymbolName(const QString& base, int exceptId = -1) const;
    // Rewrites every expression in the scene that references `oldName` (or
    // `oldName.something`) to use `newName`. Call after renaming the entity itself.
    void    renameSymbolReferences(const QString& oldName, const QString& newName);

    // Emits automationChanged() — call after editing signals, actuators,
    // sequences or poses in place.
    void notifyAutomationChanged() { emit automationChanged(); }
    // Emits jointsMoved() — call after changing Joint::currentValue outside
    // the simulation (jogging), so views redraw without dirtying the project.
    void notifyJointsMoved() { applyCouplings(); emit jointsMoved(); }
    // Sets every coupled joint from its master (chains in order)
    void applyCouplings();

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

    // The colour an object is drawn in: its own, else the nearest ancestor's;
    // invalid when none has one (the viewport then uses the type's default)
    QColor colorOf(int id) const;
    int    colorSource(int id) const;   // object whose colour colorOf() returns, or -1
    // Drawn opacity: the object's own times every ancestor's, so fading an
    // assembly fades its parts
    float  opacityOf(int id) const;
    // Hidden (Alt+V): the object or an ancestor has `hidden` set. Hidden
    // objects have opacityOf 0, so they aren't drawn or picked; simulation
    // and sensors ignore the flag. hiddenBy is the nearest hidden one, or -1.
    bool   isHidden(int id) const { return hiddenBy(id) != -1; }
    int    hiddenBy(int id) const;
    void   setHidden(int id, bool hidden);   // emits objectChanged
    // Alt+V on a selection: hides them all when any is shown, else clears
    // their own flags (one under a hidden parent stays hidden). Returns
    // whether they were hidden.
    bool   toggleHidden(const std::vector<int>& ids);

    // Transform helpers
    QMatrix4x4 worldTransform(int id) const;
    QMatrix4x4 localTransform(int id) const;
    // Sets the object's local transform so that its world transform becomes
    // `world` (position, rotation and scale; no shear)
    void       setWorldTransform(int id, const QMatrix4x4& world);
    // The joint's current motion, applied in the parent's local frame between
    // the parent and the child's own local transform
    static QMatrix4x4 jointMotion(const Joint& joint);
    // Position, Euler XYZ (degrees) and scale of an affine matrix built like
    // localTransform() (no shear)
    static Transform toTransform(const QMatrix4x4& m);

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
    void automationChanged();
    void jointsMoved();
    void matesChanged();

private:
    std::unordered_map<int, SceneObject>   m_objects;
    std::unordered_map<int, Joint>         m_joints;
    std::unordered_map<int, Motor>         m_motors;
    std::unordered_map<int, RigidBody>     m_rigidBodies;
    std::unordered_map<int, ConveyorBelt>  m_conveyorBelts;
    std::map<int, Signal>                  m_signals;
    std::map<int, Sequence>                m_sequences;
    std::map<int, Pose>                    m_poses;
    std::map<int, Mate>                    m_mates;
    std::map<int, PartSource>              m_sources;
    std::map<int, PartSink>                m_sinks;
    std::vector<int>                       m_roots;
    int  m_nextId      = 1;
    int  m_selectedId  = -1;
    bool m_floorEnabled = true;
};
