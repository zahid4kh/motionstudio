#pragma once
#include <QString>
#include <QVector3D>
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
    Sensor,    // Bool read from the scene every scan (see SensorConfig)
};

// What a sensor signal looks at. A sensor is mounted on an object (or the
// world) and moves with it; offsets and sizes are in metres in the mount's
// frame, ignoring the mount's scale.
enum class SensorType {
    Beam,   // a ray from `offset` along `direction`, up to `range`: light barrier, part present
    Zone,   // a box of `size` at `offset` (or the mount's own bounds): contact, presence
};
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
    SensorConfig sensor;            // Sensor only (type is always Bool)
};

enum class OperationType {
    Move,        // actuator → target (position mode); done when in position
    MoveToPose,  // all actuated joints in a pose, synchronized; done when all in position
    Run,         // actuator → speed mode at `velocity`; done immediately
    Stop,        // actuator decelerates to rest; done when stopped
    Set,         // only runs its actions; done immediately
    Wait,        // done after `duration` seconds
    WaitFor,     // done when `condition` is true
};

struct Operation {
    int           id   = -1;
    QString       name;
    OperationType type = OperationType::Move;

    int           actuatorId = -1;   // Move / Run / Stop  (Motor id)
    int           poseId     = -1;   // MoveToPose

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
