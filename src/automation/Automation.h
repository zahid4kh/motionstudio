#pragma once
#include <QQuaternion>
#include <QString>
#include <QVector3D>
#include <QMatrix4x4>
#include <map>
#include <vector>

// Definitions for the signal-driven automation system (see AUTOMATION_PLAN.md).
// These are pure data, owned by SceneGraph and persisted in .mstudio files;
// AutomationEngine holds all runtime state separately.

enum class SignalType { Bool, Int, Real };

enum class SignalKind {
    Input,     // set by the user from the Runtime panel, or by operation actions
    Internal,  // state/flag written by operation actions
    Derived,   // recomputed every scan from `expression`
    Sensor,    // read from the scene every scan (see SensorConfig)
};

// What a sensor signal looks at. A sensor is mounted on an object (or the
// world) and moves with it; offsets and sizes are in metres in the mount's
// frame, ignoring the mount's scale.
enum class SensorType {
    Beam,      // a ray from `offset` along `direction`, up to `range`: light barrier, part present
    Zone,      // a box of `size` at `offset` (or the mount's own bounds): contact, presence
    Distance,  // a beam whose value (Real) is the distance in m to the first surface, `range` when clear
    Position,  // a zone that also publishes where the part it sees is: <name>.x .y .z (m) and
               // .rx .ry .rz (degrees, Euler XYZ) of the part's frame in the sensor's frame
};
inline bool sensorIsRay(SensorType t) { return t == SensorType::Beam || t == SensorType::Distance; }
// A sensor signal's type: a distance sensor is Real, every other one Bool
inline SignalType sensorSignalType(SensorType t) { return t == SensorType::Distance ? SignalType::Real : SignalType::Bool; }
inline QString sensorTypeName(SensorType t) {
    switch (t) {
        case SensorType::Beam:     return "Beam";
        case SensorType::Zone:     return "Zone";
        case SensorType::Distance: return "Distance";
        case SensorType::Position: return "Position";
    }
    return "Beam";
}
inline SensorType sensorTypeFromName(const QString& s) {
    if (s == "Zone")     return SensorType::Zone;
    if (s == "Distance") return SensorType::Distance;
    if (s == "Position") return SensorType::Position;
    return SensorType::Beam;
}
enum class SensorTarget {
    Any,          // any object with geometry
    RigidBodies,  // only objects with a rigid body (or inside a rigid-body group)
    Object,       // only `targetId` and its children
};

struct SensorConfig {
    SensorType   type         = SensorType::Beam;
    int          objectId     = -1;              // mount; -1 = world
    QVector3D    offset;
    QVector3D    direction    = {1, 0, 0};       // Beam
    float        range        = 0.5f;            // Beam, m
    QVector3D    size         = {0.1f, 0.1f, 0.1f};   // Zone, full extents in m
    bool         objectBounds = false;           // Zone: use the mount's bounding box instead
    SensorTarget target       = SensorTarget::Any;
    int          targetId     = -1;              // SensorTarget::Object
};

struct Signal {
    int        id           = -1;
    QString    name;
    SignalType type         = SignalType::Bool;
    SignalKind kind         = SignalKind::Internal;
    double     initialValue = 0.0;
    QString    expression;          // Derived only
    QString    description;
    SensorConfig sensor;            // Sensor only (type follows it: see sensorSignalType)
    int        ioAddress    = -1;   // Modbus TCP address (see io/ModbusServer.h); -1 = not exposed
};

enum class OperationType {
    Move,        // actuator → target (position mode); done when in position
    MoveToPose,  // all actuated joints in a pose, synchronized; done when all in position
    Run,         // actuator → speed mode at `velocity`; done immediately
    Stop,        // actuator decelerates to rest; done when stopped
    Set,         // only runs its actions; done immediately
    Wait,        // done after `duration` seconds
    WaitFor,     // done when `condition` is true
    Grip,        // gripper picks up a part (from a sensor or a fixed object); done when held
    Release,     // gripper lets go of everything it holds; done immediately
    MoveL,       // tool moves in a straight line to where it is in a pose (inverse kinematics)
};

struct Operation {
    int           id   = -1;
    QString       name;
    OperationType type = OperationType::Move;

    int           actuatorId = -1;   // Move / Run / Stop  (Motor id)
    int           poseId     = -1;   // MoveToPose
    int           gripperId  = -1;   // Grip / Release: object that holds parts
    int           partId     = -1;   // Grip: the object to pick up, or
    int           sensorId   = -1;   // Grip: a sensor signal: pick up what it detects
                                     // MoveL / MoveToPose to a pose taught on a part: the part
                                     // (partId, or what sensorId detects; neither = the pose's own)
    int           toolId     = -1;   // MoveL: object whose frame is the tool centre point

    // All numeric/boolean fields are expressions (a constant is a valid expression)
    QString       target;            // Move: position
    QString       velocity;          // Move / Run: speed; MoveToPose: speed % (empty = actuator default / 100)
    QString       duration;          // Wait: seconds
    QString       condition;         // WaitFor
    QString       startCondition;    // extra gate before starting (empty = true)
    QString       onStart;           // "Sig = expr; Other = expr"
    QString       onFinish;

    std::vector<int> predecessors;   // operation ids (same sequence) that must be done first
};

struct Sequence {
    int     id   = -1;
    QString name;                    // identifier — publishes <name>.running / .done / .cycles
    QString enableCondition;         // empty = always enabled; false pauses before starting new operations
    QString startCondition;          // gate for (re)starting a cycle; empty = true
    bool    loop = false;            // restart after each completed cycle
    std::vector<Operation> operations;
};

struct Pose {
    int                  id = -1;
    QString              name;
    std::map<int, float> jointValues;   // joint id → value (deg or m)

    // A pose taught on a part: where the tool's frame is relative to the
    // part's frame (both unscaled). MoveL / MoveToPose then go there relative
    // to wherever the part is at run time (or a copy of it a sensor sees);
    // `jointValues` hold the robot's taught configuration (IK starts there).
    int         toolId  = -1;            // object whose frame is the tool centre point
    int         frameId = -1;            // the part; -1 = a plain joint pose
    QVector3D   relPosition;             // tool origin in the part's frame, m
    QQuaternion relRotation;             // tool orientation in the part's frame
    bool isRelative() const { return frameId != -1; }
    QMatrix4x4 relative() const {
        QMatrix4x4 m;
        m.translate(relPosition);
        m.rotate(relRotation);
        return m;
    }
};

// Object source: makes copies of a template object while simulating. The
// template itself is out of the run (hidden, not sensed, no physics); each
// copy appears where it is, as a free part at the top of the tree.
enum class SourceMode {
    Interval,   // a copy every `interval` seconds, the first at the start
    Trigger,    // a copy each time `trigger` becomes true
};
struct PartSource {
    int        id = -1;
    QString    name;                     // symbol: <name>.count, <name>.blocked
    int        templateId = -1;
    SourceMode mode       = SourceMode::Interval;
    QString    interval   = "2";         // expression, read as each copy is made
    QString    trigger;                  // Trigger mode
    QString    enable;                   // empty = always
    int        maxCount   = 0;           // 0 = no limit
    bool       whenClear  = true;        // wait until no part overlaps the spot
};

// Object sink: removes parts that touch its zone while simulating (a copy
// made by a source, or a free part: a top-level rigid body). Parts held by a
// gripper are left alone. Removed parts come back on Reset.
struct PartSink {
    int          id = -1;
    QString      name;                   // symbol: <name>.count
    SensorConfig zone;                   // type Zone: mount, offset and size, or the mount's bounds
    int          sourceId = -1;          // only copies from this source; -1 = any part
    QString      enable;                 // empty = always
};

inline QString signalTypeName(SignalType t) {
    switch (t) {
        case SignalType::Bool: return "Bool";
        case SignalType::Int:  return "Int";
        case SignalType::Real: return "Real";
    }
    return "Bool";
}

inline QString signalKindName(SignalKind k) {
    switch (k) {
        case SignalKind::Input:    return "Input";
        case SignalKind::Internal: return "Internal";
        case SignalKind::Derived:  return "Derived";
        case SignalKind::Sensor:   return "Sensor";
    }
    return "Internal";
}

inline SignalKind signalKindFromName(const QString& s) {
    if (s == "Input")   return SignalKind::Input;
    if (s == "Derived") return SignalKind::Derived;
    if (s == "Sensor")  return SignalKind::Sensor;
    return SignalKind::Internal;
}

inline QString operationTypeName(OperationType t) {
    switch (t) {
        case OperationType::Move:       return "Move";
        case OperationType::MoveToPose: return "MoveToPose";
        case OperationType::Run:        return "Run";
        case OperationType::Stop:       return "Stop";
        case OperationType::Set:        return "Set";
        case OperationType::Wait:       return "Wait";
        case OperationType::WaitFor:    return "WaitFor";
        case OperationType::Grip:       return "Grip";
        case OperationType::Release:    return "Release";
        case OperationType::MoveL:      return "MoveL";
    }
    return "Move";
}

inline OperationType operationTypeFromName(const QString& s) {
    if (s == "MoveToPose") return OperationType::MoveToPose;
    if (s == "Run")        return OperationType::Run;
    if (s == "Stop")       return OperationType::Stop;
    if (s == "Set")        return OperationType::Set;
    if (s == "Wait")       return OperationType::Wait;
    if (s == "WaitFor")    return OperationType::WaitFor;
    if (s == "Grip")       return OperationType::Grip;
    if (s == "Release")    return OperationType::Release;
    if (s == "MoveL")      return OperationType::MoveL;
    return OperationType::Move;
}

// Coerces a raw value to a signal's type (Bool → 0/1, Int → rounded)
inline double coerceSignal(SignalType t, double v) {
    switch (t) {
        case SignalType::Bool: return v != 0.0 ? 1.0 : 0.0;
        case SignalType::Int:  return double(qint64(v >= 0 ? v + 0.5 : v - 0.5));
        case SignalType::Real: return v;
    }
    return v;
}
