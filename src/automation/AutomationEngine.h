#pragma once
#include <QObject>
#include <QHash>
#include <QMatrix4x4>
#include <QStringList>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>
#include "Automation.h"
#include "Expression.h"
#include "Kinematics.h"
#include "MotionProfile.h"
#include "../scene/Motor.h"
#include "../scene/SceneQuery.h"
#include "../robot/Gait.h"

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
    double dt() const { return m_dt; }         // the last scan's length (s)
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

    // ── Motions commanded from outside (the API server) ─────────────────────
    // A program asks for joints to move to values, or for a tool to go to a
    // frame by a joint move or along a straight line. Motions that share a
    // joint run one after another in the order they were submitted; the
    // others run at the same time. A motion that fails cancels the queued
    // ones that share a joint with it, as a robot program stops at a fault.
    struct CartesianGoal {
        int  frameId  = -1;        // the pose is in this object's unscaled frame; -1 = the world
        bool relative = false;     // an offset from where the tool is when the motion starts
        bool toolAxes = false;     // relative: along and about the tool's own axes, not frameId's
        std::optional<double> x, y, z;      // m; absolute: a missing one keeps the tool's
        std::optional<double> rx, ry, rz;   // degrees, Euler XYZ; absolute: all missing keeps the
                                            // tool's orientation, some missing keep theirs
        bool hasRotation() const { return rx || ry || rz; }
    };
    enum class MotionKind { Joints, JointsToPose, Linear, Walk };
    // A legged robot walks (see gait::): by distances and a turn from where
    // it stands, or through points on the floor
    struct WalkRequest {
        int  root = -1;                    // the robot: its root object is carried along
        int  leftFoot = -1, rightFoot = -1;   // -1 = found by name
        bool path = false;                 // false: forward / left / turn; true: through `points`
        double forward = 0, left = 0;      // m, in the robot's frame (left is its −Z)
        double turn = 0;                   // degrees, positive to its left
        std::vector<QPointF> points;       // x, z (m) in the frame below
        std::optional<double> heading;     // degrees about +Y, turned to after the points
        int  frameId = -1;                 // points and heading are in this object's frame (-1 = world)
        bool fromStart = false;            // ... or in the robot's own frame where the walk starts
                                           //     (x ahead, z to its right)
        gait::Params params;
    };
    struct MotionRequest {
        MotionKind kind = MotionKind::Joints;
        std::map<int, double> joints;   // Joints: joint id → target (deg or m)
        bool relativeJoints = false;    // Joints: offsets from where the joints are at the start
        int  tool = -1;                 // JointsToPose / Linear: the object whose frame moves
        std::vector<int> chain;         // the joints that move it; empty = kin::chainTo(tool)
        CartesianGoal goal;
        // Joints / JointsToPose: % of each actuator's speed (0 < s ≤ 100, 0 = 100);
        // Linear: tool speed in m/s (0 = 0.25)
        double speed = 0;
        QString label;                  // what to call it in messages ("moveL")
        WalkRequest walk;               // Walk
    };
    enum class MotionState { Queued, Running, Done, Failed, Cancelled };
    struct MotionStatus {
        MotionState state = MotionState::Queued;
        double  progress = 0;           // 0..1
        QString code;                   // Failed / Cancelled: out_of_reach, out_of_limits, cancelled, reset
        QString error;
        bool finished() const { return state == MotionState::Done || state == MotionState::Failed
                                    || state == MotionState::Cancelled; }
    };
    static QString motionStateName(MotionState s);

    // Checks the request and queues it; returns the motion's id (> 0), or 0
    // with an error code (not_running, bad_request, not_actuated,
    // out_of_limits) and message. Applies from the next scan.
    int  submitMotion(const MotionRequest& r, QString* code = nullptr, QString* error = nullptr);
    bool hasMotion(int id) const;
    MotionStatus motionStatus(int id) const;
    // Stops a queued or running motion; its joints brake and hold
    void cancelMotion(int id, const QString& why = {});
    // Cancels every unfinished motion that moves one of `joints` (every motion
    // when empty); returns how many
    int  cancelMotions(const std::set<int>& joints = {}, const QString& why = {},
                       const QString& code = "cancelled");
    // Runs an actuator in speed mode at `speed` (deg/s or m/s; 0 stops it),
    // cancelling motions that move its joint
    bool runActuator(int jointId, double speed, QString* error = nullptr);
    // Ends the walks of the robot at `root` after the step under way, the
    // feet together, and cancels its queued walks; returns how many walks
    int  stopWalking(int root);
    // A walk under way (from a program or a Walk operation), for status and drawing
    struct WalkView {
        int    root = -1;
        int    motion = 0;              // the motion's id; 0 for a Walk operation
        int    stepsDone = 0, steps = 0;
        double elapsed = 0, duration = 0;
        std::vector<gait::Walker::Print> prints;
        double footLength = 0, footWidth = 0;
        gait::Walker::Phase phase;      // the step under way
        gait::Place goal;               // where the walk ends
    };
    std::vector<WalkView> walks() const;
    // Checks a walk request and makes its walker from where the robot is now
    // (returns null with an error code and message)
    std::shared_ptr<gait::Walker> makeWalker(const WalkRequest& w, QString* code, QString* error) const;
    // Where `goal` puts `tool` now (world frame, unscaled)
    QMatrix4x4 goalFrame(int tool, const CartesianGoal& goal) const;
    // Compiles a condition against this run's symbols, and evaluates it on
    // this scan's values
    bool   compileExpression(const QString& source, expr::Expression& out, QString* error = nullptr) const;
    double evaluate(const expr::Expression& e) const { return e.eval(ctx()); }

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

    // A tool moving along a straight line: from → to over `duration`
    // seconds from `start`, eased in and out
    struct LinearPath {
        int              tool = -1;
        std::vector<int> chain;
        QMatrix4x4       from, to;
        bool             orient = false;
        double           start = 0, duration = 0;
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
        // MoveL: the tool path; to a part, `path.to` follows the part every scan
        LinearPath       path;
        bool             pathWarned = false;
        // Walk
        expr::Expression forward, sideways, turn;
        std::shared_ptr<gait::Walker> walker;
        double           walkStart = 0;
    };

    struct MotionRt {
        int           id = -1;
        MotionRequest req;
        MotionStatus  status;
        std::vector<int> joints;                           // every joint it moves
        std::vector<std::pair<int, double>> targets;       // Joints / JointsToPose: (actIdx, target)
        std::vector<double> from;
        LinearPath    path;
        std::shared_ptr<gait::Walker> walker;   // Walk
        double        walkStart = 0;
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
    // Synchronized point-to-point move of the joints to `values` at `percent`
    // of their speed; fills (actIdx, target) and the start positions
    void commandJoints(const std::map<int, double>& values, double percent,
                       std::vector<std::pair<int, double>>& targets, std::vector<double>& from);
    double jointProgress(const std::vector<std::pair<int, double>>& targets, const std::vector<double>& from,
                         bool* arrived) const;
    void beginPath(LinearPath& p, double speed);
    // Puts the chain's actuators at the next point of the path; returns the
    // path parameter (1 at the end)
    double stepPath(LinearPath& p, kin::Result* result);
    void advanceMotions();
    // One scan of a walk: the root and the legs where the plan has them now;
    // false when a foot can't reach (`error` m off)
    bool stepWalk(gait::Walker& w, double start, double* error, double* progress);
    bool startMotion(MotionRt& m);
    void finishMotion(MotionRt& m, MotionState state, const QString& code = {}, const QString& error = {});
    void holdJoints(const std::vector<int>& joints);
    int  actIndexForJoint(int jointId) const;
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

    std::vector<MotionRt>                 m_motions;      // queued and running, in submit order
    std::map<int, MotionStatus>           m_finishedMotions;
    int                                   m_nextMotionId = 1;   // never reset, so ids stay unique

    std::vector<std::pair<int, double>>   m_pendingWrites;
    std::map<int, std::deque<double>>     m_inputQueue;
    QStringList                           m_warnings;
};
