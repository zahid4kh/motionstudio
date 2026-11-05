#pragma once
#include <QObject>
#include <QHash>
#include <QMatrix4x4>
#include <QStringList>
#include <deque>
#include <map>
#include <vector>
#include "Automation.h"
#include "Expression.h"
#include "MotionProfile.h"
#include "../scene/Motor.h"
#include "../scene/SceneQuery.h"

class SceneGraph;

// Runs the automation logic defined in the SceneGraph (signals, actuators,
// sequences, poses) as a deterministic PLC-style scan cycle:
//
//   0. apply operation action writes from the previous scan + queued user inputs
//   1. read sensors, publish actuator / sequence status signals
//   2. evaluate derived signals (dependency order)
//   3. advance sequences (finish / start operations, issue actuator commands)
//   4. evaluate actuator expression bindings
//   5. integrate actuator motion profiles, write Joint::currentValue, then
//      move gripped parts with their grippers
//   6. part flow: sinks take the parts in their zones, sources make copies
//
// Joints without an actuator are never touched (in MuJoCo mode they stay
// physics-driven). Definitions are compiled once by start(); editing them
// while the engine is active has no effect until the next start().
class AutomationEngine : public QObject {
    Q_OBJECT
public:
    enum class SymbolKind { Signal, Actuator, Sequence, Flow };
    struct Symbol {
        QString    name;
        SymbolKind kind      = SymbolKind::Signal;
        int        ownerId   = -1;        // Signal / Motor / Sequence id
        bool       writable  = false;     // Input / Internal signals
        bool       isBool    = false;
        SignalType type      = SignalType::Real;
        QString    help;                  // short description for completion/tooltips
    };

    // ── Edit-time helpers (no runtime needed) ───────────────────────────────
    static std::vector<Symbol> buildSymbols(const SceneGraph& scene);
    static QStringList symbolNames(const SceneGraph& scene);
    // Returns an empty string when `source` compiles, else the error message.
    static QString validateExpression(const SceneGraph& scene, const QString& source);
    static QString validateAssignments(const SceneGraph& scene, const QString& source);
    // Compiles every definition; returns "Where: problem" lines.
    static QStringList validateAll(const SceneGraph& scene);

    explicit AutomationEngine(SceneGraph* scene, QObject* parent = nullptr);

    // ── Lifecycle ───────────────────────────────────────────────────────────
    bool start(QStringList* errors);   // compile + initialize runtime state
    void stop();                       // drop runtime state
    bool isActive() const { return m_active; }
    void scan(double dt);

    // ── Runtime queries ─────────────────────────────────────────────────────
    double time() const { return m_time; }
    long   scanCount() const { return m_scanCount; }
    const std::vector<Symbol>& symbols() const { return m_symbols; }
    int    slotOf(const QString& name) const { return m_slotIndex.value(name, -1); }
    double value(int slot) const { return (slot >= 0 && slot < int(m_cur.size())) ? m_cur[size_t(slot)] : 0.0; }
    double value(const QString& name) const { return value(slotOf(name)); }
    const std::vector<double>& values() const { return m_cur; }   // every slot, this scan

    // Queues a write to an Input/Internal signal; applied at the start of
    // the next scan. Writes are applied one per scan per signal, so a quick
    // press+release is still seen by exactly one scan.
    void setInput(const QString& name, double v);
    // Latest value written by setInput() that is still waiting for a scan,
    // else the current value — what an input control should display.
    double displayValue(const QString& name) const;

    enum class OpState { Waiting, Running, Done };
    struct OpStatus {
        OpState state = OpState::Waiting;
        QString detail;           // why it is waiting / what it is doing / error
        double  startTime = -1;   // this cycle
        double  endTime   = -1;
        double  prevStart = -1;   // previous completed cycle (for Gantt)
        double  prevEnd   = -1;
        double  progress  = 0;    // 0..1 while running, when estimable
    };
    OpStatus opStatus(int sequenceId, int operationId) const;

    enum class SeqState { Idle, Running, Finished };
    struct SeqStatus {
        SeqState state      = SeqState::Idle;
        bool     enabled    = true;
        int      cycles     = 0;
        double   cycleStart = 0;
        double   lastCycleTime = 0;
        QString  detail;
        // The last completed cycle's critical path: the chain of operations
        // (ids, in order) that set its length: each one waited on the one
        // before it. Shortening any of them shortens the cycle.
        std::vector<int> criticalPath;
        double   criticalBusy = 0;    // seconds those operations ran; the rest was waiting
    };
    SeqStatus seqStatus(int sequenceId) const;

    // Object the sensor signal `signalId` detected at the start of this scan, or -1
    int sensorDetection(int signalId) const;
    // Objects currently held by a gripper (Grip operations), moved with it
    std::vector<int> heldObjects() const;
    int    holderOf(int objectId) const;     // gripper holding `objectId`, or -1
    // A conveyor belt's speed this scan: its speed binding, else `fallback`
    double beltSpeed(int objectId, double fallback) const;
    // Motors whose actuator is moving this scan (for viewport highlighting)
    std::vector<int> movingMotors() const;
    // Velocity of an actuated joint this scan (deg/s or m/s)
    bool actuatedJointState(int jointId, double& pos, double& vel) const;
    // Names of currently running operations ("Seq: op, op")
    QStringList activeOperationLabels() const;
    // Runtime warnings collected during scans (e.g. move to an unreachable target)
    const QStringList& warnings() const { return m_warnings; }
    // True once after a scan in which a source made a copy or a sink took a
    // part (the physics model needs rebuilding); clears the flag
    bool takeStructureChanged() { const bool c = m_structureChanged; m_structureChanged = false; return c; }
    // Copies made / parts taken so far
    int  sourceCount(int sourceId) const;
    int  sinkCount(int sinkId) const;

signals:
    void scanned();

private:
    struct ActuatorRt {
        int    motorId = -1;
        int    jointId = -1;
        QString name;
        bool   limited = true;
        double limMin = -180, limMax = 180;
        double defVelocity = 45, defAccel = 0;
        // Current command
        ActuatorMode mode = ActuatorMode::Position;
        double target = 0, velocity = 45, accel = 0;
        bool   active = true;
        motion::State st;
        expr::Expression bindTarget, bindVelocity, bindActive;
        bool   direct = false;   // position set this scan by a MoveL (no profile)
        int    slotBase = -1;   // pos, vel, target, inPosition, moving, atMin, atMax, active
    };

    struct OpRt {
        Operation def;
        expr::Expression target, velocity, duration, condition, startCond;
        std::vector<expr::Assignment> onStart, onFinish;
        std::vector<int> predIdx;       // indices into SeqRt::ops
        int    actIdx = -1;
        OpStatus status;
        // Running-command bookkeeping
        double moveTarget = 0;
        double waitUntil  = 0;
        double moveFrom   = 0;
        std::vector<std::pair<int, double>> poseTargets;   // (actIdx, target)
        std::vector<double>                  poseFrom;
        // MoveL / MoveToPose to a pose taught on a part: the part this run
        // (-1 while a sensor has not seen one) and whether the move is planned
        int              part = -1;
        bool             planned = false;
        // MoveL: tool path from → to over `duration` seconds from pathStart;
        // to a part, `pathTo` follows the part every scan
        int              tool = -1;
        double           pathStart = 0;
        std::vector<int> chain;
        QMatrix4x4       pathFrom, pathTo;
        bool             pathOrient = false;
        double           pathDuration = 0;
        bool             pathWarned = false;
    };

    struct SeqRt {
        int     id = -1;
        QString name;
        bool    loop = false;
        expr::Expression enable, startCond;
        std::vector<OpRt> ops;
        SeqStatus status;
        int     slotBase = -1;      // running, done, cycles
    };

    struct SensorRt {
        int          signalId = -1;
        int          slot     = -1;      // the signal; a Position sensor's fields follow it
        SensorConfig cfg;
        int          detected = -1;
    };

    struct SourceRt {
        PartSource       def;
        expr::Expression interval, trigger, enable;
        int    count = 0;
        double nextAt = 0;
        bool   prevTrigger = false;
        int    pending = 0;               // trigger edges not yet served
        bool   blocked = false;           // a copy is due but the spot is taken
        int    slotBase = -1;             // count, blocked
    };

    struct SinkRt {
        PartSink         def;
        expr::Expression enable;
        int    count = 0;
        int    slot = -1;                 // count
    };

    // A part attached to a gripper: its pose relative to the gripper's
    // (unscaled) frame is kept fixed until released
    struct Hold {
        int        gripperId = -1;
        int        partId    = -1;
        QMatrix4x4 relative;
    };

    struct BeltRt {
        int              objectId = -1;
        expr::Expression speed;
        double           value = 0;
    };

    struct DerivedRt {
        int slot = -1;
        SignalType type = SignalType::Real;
        expr::Expression ex;
    };

    expr::EvalContext ctx() const;
    void queueWrite(const std::vector<expr::Assignment>& actions);
    void publishStatus();
    void advanceSequence(SeqRt& seq);
    void startOperation(SeqRt& seq, OpRt& op);
    bool operationFinished(OpRt& op);
    void finishOperation(OpRt& op);
    void applyBindings();
    void integrate(double dt);
    void attach(int gripperId, int partId);
    bool stepLinearMove(OpRt& op);
    void applyHolds();
    void updateFlow();
    bool partTouches(int rootId, const scenequery::Obb& box) const;
    bool spotTaken(const SourceRt& src) const;
    // The object a part-relative pose is applied to for `op` (see
    // Operation::sensorId), or -1 while its sensor sees nothing
    int  partFor(OpRt& op, const Pose& pose);
    // MoveToPose: commands the synchronized joint move; false while waiting for a part
    bool planPoseMove(OpRt& op);
    // MoveL: starts the tool path; false while waiting for a part
    bool planLinearMove(OpRt& op);
    void commandJoints(OpRt& op, const std::map<int, float>& values, double percent);
    double clampToLimits(const ActuatorRt& a, double v) const;
    bool inPosition(const ActuatorRt& a, double target) const;

    SceneGraph* m_scene;
    bool        m_active = false;
    double      m_time = 0, m_dt = 0;
    long        m_scanCount = 0;

    std::vector<Symbol>   m_symbols;
    QHash<QString, int>   m_slotIndex;
    std::vector<double>   m_cur, m_prev;
    std::vector<SignalType> m_slotType;

    std::vector<DerivedRt>  m_derived;
    std::vector<SensorRt>   m_sensors;
    std::vector<Hold>       m_holds;
    std::vector<BeltRt>     m_belts;
    std::vector<ActuatorRt> m_actuators;
    std::vector<SeqRt>      m_sequences;
    std::vector<SourceRt>   m_sources;
    std::vector<SinkRt>     m_sinks;
    bool                    m_structureChanged = false;
    std::map<int, int>      m_actIndexByMotor;

    std::vector<std::pair<int, double>>   m_pendingWrites;
    std::map<int, std::deque<double>>     m_inputQueue;
    QStringList                           m_warnings;
};
