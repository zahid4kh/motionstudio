// Headless regression tests for MotionStudioCore. Build with
//   cmake -B build -DMOTIONSTUDIO_TESTS=ON && cmake --build build
// and run ./build/MotionStudioTests [filter]. Exit code 0 means all passed.
#include <QCoreApplication>
#include <QJsonDocument>
#include <cmath>
#include <cstdio>
#include <functional>
#include <vector>
#include "scene/SceneGraph.h"
#include "scene/PrimitiveMesh.h"
#include "scene/UndoManager.h"
#include "automation/AutomationEngine.h"
#include "automation/Sensors.h"
#include "automation/Kinematics.h"
#include "physics/SimulationEngine.h"
#include "serialization/ProjectSerializer.h"
#include "scene/Interference.h"
#include "io/ModbusServer.h"
#include "scene/SceneMerge.h"
#include "io/RobotFormats.h"
#include <QTemporaryDir>
#ifdef MUJOCO_ENABLED
#include <mujoco/mujoco.h>
#endif
#include <QDir>
#include <QElapsedTimer>
#include <QTcpSocket>
#include <cstring>

namespace {

int g_failures = 0;
int g_checks   = 0;

#define CHECK(cond) do { ++g_checks; if (!(cond)) { ++g_failures; \
    std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_NEAR(a, b, tol) do { ++g_checks; double _a = (a), _b = (b); \
    if (!(std::fabs(_a - _b) <= (tol))) { ++g_failures; \
    std::printf("  FAIL %s:%d: %s = %g, expected %g\n", __FILE__, __LINE__, #a, _a, _b); } } while (0)

struct Test { const char* name; std::function<void()> fn; };
std::vector<Test>& registry() { static std::vector<Test> t; return t; }
struct Register { Register(const char* n, std::function<void()> f) { registry().push_back({n, std::move(f)}); } };
#define TEST(name) static void name(); static Register reg_##name(#name, name); static void name()

int addBox(SceneGraph& g, const QString& name, QVector3D pos, QVector3D scale, int parent = -1) {
    int id = g.addObject(name, PrimitiveType::Cube, parent);
    g.getObject(id)->transform.position = pos;
    g.getObject(id)->transform.scale = scale;
    return id;
}

// Save to JSON and load into a fresh scene, as a file round trip does
void roundTrip(const SceneGraph& from, SceneGraph& to) {
    const QByteArray bytes = QJsonDocument(ProjectSerializer::toJson(from)).toJson();
    ProjectSerializer::fromJson(to, QJsonDocument::fromJson(bytes).object());
}

const Signal* signalNamed(const SceneGraph& g, const QString& name) {
    for (auto& [id, s] : g.allSignals()) if (s.name == name) return &s;
    return nullptr;
}


// A base with an arm turning about Y (actuator J1), a tool frame at the arm's
// tip (1, 0, 0) and a 0.1 m part sitting at the tip
struct Arm { int base, arm, tool, part, joint, motor; };
Arm makeArm(SceneGraph& g) {
    Arm a;
    a.base = addBox(g, "Base", {0, 0, 0}, {0.2f, 0.2f, 0.2f});
    a.arm  = addBox(g, "Arm", {0.5f, 0, 0}, {1, 0.1f, 0.1f}, a.base);
    // Children inherit scale: undo the base's 0.2 so positions are in metres
    g.getObject(a.arm)->transform.position = {2.5f, 0, 0};
    g.getObject(a.arm)->transform.scale = {5, 0.5f, 0.5f};
    a.tool = g.addObject("Tool", PrimitiveType::ImportedMesh, a.arm);
    g.getObject(a.tool)->transform.position = {0.5f, 0, 0};
    a.part = addBox(g, "Part", {1, 0, 0}, {0.1f, 0.1f, 0.1f});
    Joint j;
    j.type = JointType::Revolute;
    j.axis = {0, 1, 0};
    j.parentBodyId = a.base;
    j.childBodyId = a.arm;
    j.limitMin = -180;
    j.limitMax = 180;
    a.joint = g.addJoint(j);
    a.motor = g.addActuatorForJoint(a.joint);
    g.getMotor(a.motor)->velocity = 180;
    return a;
}

Operation makeOp(SceneGraph& g, OperationType t, std::vector<int> after = {}) {
    Operation op;
    op.id = g.allocateId();
    op.type = t;
    op.predecessors = std::move(after);
    return op;
}

QVector3D worldPos(const SceneGraph& g, int id) { return g.worldTransform(id).column(3).toVector3D(); }

void runFor(AutomationEngine& e, double seconds) {
    for (int i = 0; i < int(seconds / 0.016); ++i) e.scan(0.016);
}

// ── Appearance ──────────────────────────────────────────────────────────────

TEST(colorsInheritDuplicateAndSave) {
    SceneGraph g;
    int group = g.addObject("Assembly", PrimitiveType::ImportedMesh);
    int a = addBox(g, "A", {0, 0, 0}, {1, 1, 1}, group);
    int b = addBox(g, "B", {1, 0, 0}, {1, 1, 1}, group);
    CHECK(!g.colorOf(a).isValid());
    g.getObject(group)->color = QColor("#ff7a1a");
    CHECK(g.colorOf(a) == QColor("#ff7a1a"));
    CHECK(g.colorSource(a) == group);
    g.getObject(b)->color = QColor("#2f6fd6");
    CHECK(g.colorOf(b) == QColor("#2f6fd6"));
    int copy = g.duplicateObject(group);
    const SceneObject* c = g.getObject(copy);
    CHECK(c && c->color == QColor("#ff7a1a"));
    CHECK(c && g.colorOf(c->childIds[1]) == QColor("#2f6fd6"));
    SceneGraph h;
    roundTrip(g, h);
    int n = 0;
    for (auto& [id, o] : h.allObjects()) {
        if (o.name == "B") { CHECK(o.color == QColor("#2f6fd6")); ++n; }
        if (o.name == "A") { CHECK(!o.color.isValid() && h.colorOf(id) == QColor("#ff7a1a")); ++n; }
    }
    CHECK(n == 4);
}

// ── Sensors ─────────────────────────────────────────────────────────────────

TEST(sensorBeamDetectsAndMeasures) {
    SceneGraph g;
    int post = addBox(g, "Post", {0, 0.5f, 0}, {0.1f, 1, 0.1f});
    int part = addBox(g, "Part", {0.6f, 0.5f, 0}, {0.2f, 0.2f, 0.2f});
    SensorConfig c;
    c.objectId = post;
    c.direction = {1, 0, 0};
    c.range = 1.0f;
    sensors::Reading r = sensors::read(g, c);
    CHECK(r.active);
    CHECK(r.objectId == part);
    CHECK_NEAR(r.distance, 0.5, 1e-4);    // part's near face at x = 0.5

    c.range = 0.4f;                       // too short to reach it
    CHECK(!sensors::read(g, c).active);
    c.range = 1.0f;

    c.target = SensorTarget::RigidBodies; // part has no rigid body yet
    CHECK(!sensors::read(g, c).active);
    g.addRigidBody(part);
    CHECK(sensors::read(g, c).objectId == part);

    c.target = SensorTarget::Object;
    c.targetId = post;                    // the mount is never seen
    CHECK(!sensors::read(g, c).active);
    c.targetId = part;
    CHECK(sensors::read(g, c).active);

    // The beam turns with its mount
    g.getObject(post)->transform.rotation = {0, 90, 0};   // +X now points to −Z
    CHECK(!sensors::read(g, c).active);
}

TEST(sensorZoneAndContact) {
    SceneGraph g;
    int gripper = addBox(g, "Gripper", {0, 1, 0}, {0.1f, 0.1f, 0.1f});
    int part    = addBox(g, "Part", {0, 0.5f, 0}, {0.2f, 0.2f, 0.2f});
    SensorConfig c;
    c.type = SensorType::Zone;
    c.objectId = gripper;
    c.objectBounds = true;                // contact sensor
    CHECK(!sensors::read(g, c).active);
    g.getObject(gripper)->transform.position = {0, 0.64f, 0};   // gripper bottom 0.59, part top 0.6
    CHECK(sensors::read(g, c).active);
    CHECK(sensors::read(g, c).objectId == part);

    c.objectBounds = false;               // a 0.1 m box 0.3 m below the gripper
    c.offset = {0, -0.3f, 0};
    g.getObject(gripper)->transform.position = {0, 1, 0};
    CHECK(!sensors::read(g, c).active);
    c.offset = {0, -0.5f, 0};
    CHECK(sensors::read(g, c).active);
}

TEST(sensorSignalInEngineAndFile) {
    SceneGraph g;
    int post = addBox(g, "Post", {0, 0.5f, 0}, {0.1f, 1, 0.1f});
    int part = addBox(g, "Part", {0.6f, 0.5f, 0}, {0.2f, 0.2f, 0.2f});
    Signal s;
    s.name = "Present";
    s.kind = SignalKind::Sensor;
    s.type = SignalType::Real;            // forced to Bool
    s.sensor.objectId = post;
    s.sensor.range = 1.0f;
    g.addSignal(s);
    Signal d;
    d.name = "NotPresent";
    d.kind = SignalKind::Derived;
    d.expression = "!Present";
    g.addSignal(d);
    CHECK(signalNamed(g, "Present")->type == SignalType::Bool);

    AutomationEngine eng(&g);
    QStringList errs;
    CHECK(eng.start(&errs));
    eng.scan(0.016);
    CHECK(eng.value("Present") == 1.0);
    CHECK(eng.value("NotPresent") == 0.0);
    CHECK(eng.sensorDetection(signalNamed(g, "Present")->id) == part);
    // Sensors are read-only
    CHECK(!AutomationEngine::validateAssignments(g, "Present = true").isEmpty());
    g.getObject(part)->transform.position = {3, 0.5f, 0};
    eng.scan(0.016);
    CHECK(eng.value("Present") == 0.0);
    CHECK(eng.value("NotPresent") == 1.0);
    eng.stop();

    SceneGraph h;
    roundTrip(g, h);
    const Signal* hs = signalNamed(h, "Present");
    CHECK(hs && hs->kind == SignalKind::Sensor);
    CHECK(hs && h.getObject(hs->sensor.objectId) && h.getObject(hs->sensor.objectId)->name == "Post");
    CHECK(hs && std::fabs(hs->sensor.range - 1.0f) < 1e-6f);

    // A deleted mount is reported, not silently turned into the world
    g.removeObject(post);
    CHECK(!AutomationEngine::validateAll(g).isEmpty());
    SceneGraph k;
    roundTrip(g, k);
    CHECK(!AutomationEngine::validateAll(k).isEmpty());
}


// ── Gripping ────────────────────────────────────────────────────────────────

TEST(gripCarriesPartUntilRelease) {
    SceneGraph g;
    Arm a = makeArm(g);
    CHECK_NEAR((worldPos(g, a.tool) - QVector3D(1, 0, 0)).length(), 0, 1e-5);
    Sequence q;
    q.name = "Pick";
    Operation grip = makeOp(g, OperationType::Grip);
    grip.gripperId = a.tool;
    grip.partId = a.part;
    Operation move = makeOp(g, OperationType::Move, {grip.id});
    move.actuatorId = a.motor;
    move.target = "90";
    Operation release = makeOp(g, OperationType::Release, {move.id});
    release.gripperId = a.tool;
    Operation back = makeOp(g, OperationType::Move, {release.id});
    back.actuatorId = a.motor;
    back.target = "0";
    q.operations = {grip, move, release, back};
    g.addSequence(q);

    AutomationEngine e(&g);
    QStringList errs;
    CHECK(e.start(&errs));
    if (!errs.isEmpty()) std::printf("  %s\n", qPrintable(errs.join("; ")));
    e.scan(0.016);
    CHECK(e.holderOf(a.part) == a.tool);
    runFor(e, 0.25);                                       // halfway: 45°
    const QVector3D mid = worldPos(g, a.part);
    CHECK(mid.z() < -0.3f && mid.x() > 0.3f);
    runFor(e, 2.0);                                        // moved to 90, released, back to 0
    CHECK(e.heldObjects().empty());
    CHECK_NEAR((worldPos(g, a.part) - QVector3D(0, 0, -1)).length(), 0, 1e-3);
    CHECK_NEAR(g.getJoint(a.joint)->currentValue, 0, 1e-3);
    // Scale and orientation survive the trip (rotated a quarter turn)
    CHECK_NEAR(g.getObject(a.part)->transform.scale.x(), 0.1, 1e-4);
    CHECK_NEAR(std::fabs(g.getObject(a.part)->transform.rotation.y()), 90, 1e-2);
}

TEST(gripFromSensorWaitsForPart) {
    SceneGraph g;
    Arm a = makeArm(g);
    g.getObject(a.part)->transform.position = {3, 0, 0};   // not at the tool yet
    Signal sn;
    sn.name = "AtTool";
    sn.kind = SignalKind::Sensor;
    sn.sensor.type = SensorType::Zone;
    sn.sensor.objectId = a.tool;
    sn.sensor.size = {0.2f, 0.2f, 0.2f};
    const int sensorId = g.addSignal(sn);
    Sequence q;
    q.name = "Pick";
    Operation grip = makeOp(g, OperationType::Grip);
    grip.gripperId = a.tool;
    grip.sensorId = sensorId;
    q.operations = {grip};
    const int seqId = g.addSequence(q);
    const int gripId = g.getSequence(seqId)->operations[0].id;

    AutomationEngine e(&g);
    CHECK(e.start(nullptr));
    runFor(e, 0.1);
    CHECK(e.opStatus(seqId, gripId).state == AutomationEngine::OpState::Running);
    CHECK(e.opStatus(seqId, gripId).detail.contains("AtTool"));
    g.getObject(a.part)->transform.position = {1.05f, 0, 0};   // part arrives
    runFor(e, 0.05);
    CHECK(e.opStatus(seqId, gripId).state == AutomationEngine::OpState::Done);
    CHECK(e.holderOf(a.part) == a.tool);

    // Validation: a deleted sensor or a gripper gripping its own arm
    SceneGraph h;
    roundTrip(g, h);
    CHECK(AutomationEngine::validateAll(h).isEmpty());
    g.removeSignal(sensorId);
    CHECK(!AutomationEngine::validateAll(g).isEmpty());
    g.getSequence(seqId)->operations[0].sensorId = -1;
    g.getSequence(seqId)->operations[0].partId = a.arm;
    CHECK(!AutomationEngine::validateAll(g).isEmpty());
}

TEST(beltSpeedBinding) {
    SceneGraph g;
    int belt = addBox(g, "Belt", {0, 0.1f, 0}, {2, 0.2f, 0.6f});
    g.addConveyorBelt(belt);
    g.getConveyorBelt(belt)->velocity = 0.3f;
    Signal s;
    s.name = "Halt";
    s.kind = SignalKind::Input;
    g.addSignal(s);
    g.getConveyorBelt(belt)->speedExpr = "Halt ? 0 : 0.5";
    AutomationEngine e(&g);
    CHECK(e.start(nullptr));
    e.scan(0.016);
    CHECK_NEAR(e.beltSpeed(belt, 0.3), 0.5, 1e-9);
    e.setInput("Halt", 1);
    e.scan(0.016);
    CHECK_NEAR(e.beltSpeed(belt, 0.3), 0.0, 1e-9);
    g.getConveyorBelt(belt)->speedExpr = "Nope + 1";
    CHECK(!AutomationEngine::validateAll(g).isEmpty());
    g.getConveyorBelt(belt)->speedExpr = "Halt ? 0 : 0.5";
    g.renameSymbolReferences("Halt", "Stop");
    CHECK(g.getConveyorBelt(belt)->speedExpr == "Stop ? 0 : 0.5");
}


// ── Kinematics ──────────────────────────────────────────────────────────────

// A 6-axis arm of unscaled frames: yaw, shoulder, elbow, forearm roll, wrist
// pitch, flange roll; returns the tool frame's id and fills the joint ids
int makeSixAxis(SceneGraph& g, std::vector<int>* joints) {
    struct L { QVector3D pos; QVector3D axis; };
    const L links[6] = {{{0, 0.3f, 0}, {0, 1, 0}}, {{0, 0.2f, 0}, {0, 0, 1}}, {{0.4f, 0, 0}, {0, 0, 1}},
                        {{0.2f, 0, 0}, {1, 0, 0}}, {{0.2f, 0, 0}, {0, 0, 1}}, {{0.1f, 0, 0}, {1, 0, 0}}};
    int parent = g.addObject("Base", PrimitiveType::ImportedMesh);
    for (int i = 0; i < 6; ++i) {
        int link = g.addObject(QString("L%1").arg(i + 1), PrimitiveType::ImportedMesh, parent);
        g.getObject(link)->transform.position = links[i].pos;
        Joint j;
        j.axis = links[i].axis;
        j.pivot = links[i].pos;
        j.parentBodyId = parent;
        j.childBodyId = link;
        j.limitMin = -170;
        j.limitMax = 170;
        int jid = g.addJoint(j);
        g.addActuatorForJoint(jid);
        joints->push_back(jid);
        parent = link;
    }
    int tool = g.addObject("Tool", PrimitiveType::ImportedMesh, parent);
    g.getObject(tool)->transform.position = {0.05f, 0, 0};
    return tool;
}

void setJoints(SceneGraph& g, const std::vector<int>& joints, std::vector<float> q) {
    for (size_t i = 0; i < joints.size(); ++i) g.getJoint(joints[i])->currentValue = q[i];
}

TEST(ikReachesPoseWithOrientation) {
    SceneGraph g;
    std::vector<int> joints;
    int tool = makeSixAxis(g, &joints);
    CHECK(kin::chainTo(g, tool) == joints);
    setJoints(g, joints, {30, -20, 40, 25, -35, 60});
    const QMatrix4x4 goal = kin::toolFrame(g, tool);
    setJoints(g, joints, {0, 10, 30, 0, 20, 0});
    kin::Result r = kin::solve(g, tool, joints, kin::targetFrom(goal));
    std::printf("  6-axis: %d iterations, %.2g m, %.2g deg\n", r.iterations, r.positionError, r.angleError);
    CHECK(r.reached);
    CHECK(r.positionError < 1e-4f);
    CHECK(r.angleError < 0.05f);
    // Unreachable: 3 m away; the joints stay at the closest solution found
    kin::Target far = kin::targetFrom(goal);
    far.position = {3, 0, 0};
    r = kin::solve(g, tool, joints, far);
    CHECK(!r.reached);
    CHECK(r.positionError > 1.5f && r.positionError < 2.5f);
    for (int jid : joints) CHECK(std::fabs(g.getJoint(jid)->currentValue) <= 170.001f);
}

TEST(ikJogKeepsOrientation) {
    // Moving the tool while keeping its orientation exactly: the orientation
    // error starts at identity (once produced NaN)
    SceneGraph g;
    std::vector<int> joints;
    int tool = makeSixAxis(g, &joints);
    setJoints(g, joints, {10, 30, 60, 0, 40, 0});
    const std::vector<int> chain = kin::chainTo(g, tool);
    for (int i = 0; i < 4; ++i) {
        const kin::Target before = kin::targetFrom(kin::toolFrame(g, tool));
        kin::Target t = before;
        t.position += QVector3D(0, 0, 0.05f);
        kin::Result r = kin::solve(g, tool, chain, t, 60);
        CHECK(r.reached);
        const kin::Target after = kin::targetFrom(kin::toolFrame(g, tool));
        CHECK_NEAR((after.position - t.position).length(), 0, 1e-4);
        CHECK(std::fabs(QQuaternion::dotProduct(after.orientation, before.orientation)) > 0.99999f);
    }
}

TEST(ikPositionOnlyForShortChains) {
    SceneGraph g;
    Arm a = makeArm(g);                                      // one joint: a circle of radius 1
    std::vector<int> chain = kin::chainTo(g, a.tool);
    CHECK(chain.size() == 1);
    CHECK(!kin::canOrient(chain));
    kin::Target t;
    t.position = {0, 0, -1};
    t.useOrientation = false;
    kin::Result r = kin::solve(g, a.tool, chain, t);
    CHECK(r.reached);
    CHECK_NEAR(g.getJoint(a.joint)->currentValue, 90, 1e-2);
}

TEST(moveLFollowsAStraightLine) {
    SceneGraph g;
    std::vector<int> joints;
    int tool = makeSixAxis(g, &joints);
    Pose a, b;
    a.name = "A";
    b.name = "B";
    const std::vector<float> qa = {10, 30, 60, 0, 40, 0}, qb = {-35, 5, 70, 20, 30, -15};
    for (size_t i = 0; i < 6; ++i) { a.jointValues[joints[i]] = qa[i]; b.jointValues[joints[i]] = qb[i]; }
    int pa = g.addPose(a), pb = g.addPose(b);
    setJoints(g, joints, qb);
    const QMatrix4x4 frameB = kin::toolFrame(g, tool);
    setJoints(g, joints, qa);
    const QVector3D from = kin::toolFrame(g, tool).column(3).toVector3D();
    const QVector3D to = frameB.column(3).toVector3D();

    Sequence q;
    q.name = "Line";
    Operation op = makeOp(g, OperationType::MoveL);
    op.toolId = tool;
    op.poseId = pb;
    op.velocity = "0.2";
    q.operations = {op};
    int sid = g.addSequence(q);
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    AutomationEngine e(&g);
    CHECK(e.start(nullptr));
    double worst = 0;
    int scans = 0;
    while (e.seqStatus(sid).state != AutomationEngine::SeqState::Finished && scans < 2000) {
        e.scan(0.016);
        ++scans;
        const QVector3D p = kin::toolFrame(g, tool).column(3).toVector3D();
        worst = std::max(worst, double(p.distanceToLine(from, (to - from).normalized())));
    }
    const double expected = (to - from).length() / 0.2;
    std::printf("  MoveL: %.3f m in %.2f s (expected %.2f s), max %.3f mm off the line\n",
                (to - from).length(), scans * 0.016, expected, worst * 1000);
    CHECK(worst < 1e-3);
    CHECK_NEAR(scans * 0.016, expected, 0.05);
    CHECK_NEAR((kin::toolFrame(g, tool).column(3).toVector3D() - to).length(), 0, 1e-4);
    kin::Target end = kin::targetFrom(frameB);
    // The orientation at the end matches pose B's
    const QQuaternion got = kin::targetFrom(kin::toolFrame(g, tool)).orientation;
    CHECK(std::fabs(QQuaternion::dotProduct(got, end.orientation)) > 0.99999f);
    // Validation: a tool that no actuated joint moves
    g.getSequence(sid)->operations[0].toolId = g.rootObjects().front();
    CHECK(!AutomationEngine::validateAll(g).isEmpty());
    (void)pa;   // pose A documents the start; the joints are set to it directly
}

// ── Couplings ───────────────────────────────────────────────────────────────

TEST(couplingFollowsMasterEverywhere) {
    SceneGraph g;
    Arm a = makeArm(g);
    // A second wheel on the base, geared 2:1 the other way, 10° offset
    int wheel = addBox(g, "Wheel", {0, 0.5f, 0}, {0.5f, 0.5f, 0.5f}, a.base);
    Joint gear;
    gear.axis = {0, 1, 0};
    gear.parentBodyId = a.base;
    gear.childBodyId = wheel;
    gear.continuous = true;
    gear.coupledTo = a.joint;
    gear.ratio = -0.5f;
    gear.offset = 10;
    int gj = g.addJoint(gear);
    g.getJoint(a.joint)->currentValue = 40;
    g.notifyJointsMoved();                                      // jogging applies couplings
    CHECK_NEAR(g.getJoint(gj)->currentValue, -10, 1e-5);
    CHECK(AutomationEngine::validateAll(g).isEmpty());

    // In the engine, every scan
    Sequence q;
    q.name = "Turn";
    Operation move = makeOp(g, OperationType::Move);
    move.actuatorId = a.motor;
    move.target = "90";
    q.operations = {move};
    g.addSequence(q);
    g.getJoint(a.joint)->currentValue = 0;
    g.notifyJointsMoved();
    {
        AutomationEngine e(&g);
        CHECK(e.start(nullptr));
        runFor(e, 0.25);
        CHECK_NEAR(g.getJoint(gj)->currentValue, -0.5 * g.getJoint(a.joint)->currentValue + 10, 1e-4);
        runFor(e, 0.5);
        CHECK_NEAR(g.getJoint(gj)->currentValue, -35, 1e-3);
    }
#ifdef MUJOCO_ENABLED
    // With physics running, the follower is prescribed, not simulated
    int ball = g.addObject("Ball", PrimitiveType::Sphere);
    g.getObject(ball)->transform.position = {3, 1, 0};
    g.addRigidBody(ball);
    g.getJoint(a.joint)->currentValue = 0;
    g.notifyJointsMoved();
    {
        SimulationEngine sim(&g);
        for (int i = 0; i < 40; ++i) sim.step();
        CHECK_NEAR(g.getJoint(a.joint)->currentValue, 90, 1e-3);
        CHECK_NEAR(g.getJoint(gj)->currentValue, -35, 1e-3);
        sim.reset();
    }
#endif
    // Saved, and duplicated with the master: the copy follows the copy
    SceneGraph h;
    roundTrip(g, h);
    int followers = 0;
    for (auto& [id, j] : h.allJoints())
        if (j.coupledTo != -1) {
            ++followers;
            CHECK(h.getJoint(j.coupledTo) && h.motorForJoint(j.coupledTo));
            CHECK_NEAR(j.ratio, -0.5, 1e-6);
        }
    CHECK(followers == 1);
    const int copy = g.duplicateObject(a.base);
    int copiedFollowers = 0;
    for (auto& [id, j] : g.allJoints())
        if (j.coupledTo != -1 && scenequery::inSubtree(g, j.childBodyId, copy)) {
            ++copiedFollowers;
            CHECK(scenequery::inSubtree(g, g.getJoint(j.coupledTo)->childBodyId, copy));
        }
    CHECK(copiedFollowers == 1);

    // Problems: an actuator on a follower, a loop
    g.addActuatorForJoint(gj);
    CHECK(!AutomationEngine::validateAll(g).isEmpty());
    g.removeMotor(g.motorForJoint(gj)->id);
    g.getJoint(a.joint)->coupledTo = gj;
    CHECK(!AutomationEngine::validateAll(g).isEmpty());
    g.getJoint(a.joint)->coupledTo = -1;
    // Removing the master releases the follower
    g.removeJoint(a.joint);
    CHECK(g.getJoint(gj)->coupledTo == -1);
}

// ── Component library ───────────────────────────────────────────────────────

TEST(libraryComponentsLoadAndValidate) {
    const QStringList files = QDir(MS_SOURCE_DIR "/library").entryList({"*.mstudio"});
    CHECK(files.size() >= 7);
    for (const QString& f : files) {
        SceneGraph g;
        CHECK(ProjectSerializer::load(g, MS_SOURCE_DIR "/library/" + f));
        const QStringList errs = AutomationEngine::validateAll(g);
        if (!errs.isEmpty()) std::printf("  %s: %s\n", qPrintable(f), qPrintable(errs.join("; ")));
        CHECK(errs.isEmpty());
        CHECK(g.rootObjects().size() == 1);                     // one root to move it by
    }
}

TEST(mergeRenamesClashesAndRemapsReferences) {
    SceneGraph cell;
    CHECK(ProjectSerializer::load(cell, MS_SOURCE_DIR "/templates/sensor_cell.mstudio"));
    const size_t objectsBefore = cell.allObjects().size();
    SceneGraph gantry;
    CHECK(ProjectSerializer::load(gantry, MS_SOURCE_DIR "/library/gantry.mstudio"));
    const size_t gantryObjects = gantry.allObjects().size();
    const std::vector<int> roots = mergeScene(cell, gantry);
    CHECK(roots.size() == 1);
    CHECK(cell.allObjects().size() == objectsBefore + gantryObjects);
    // The cell already had X, Z and AtGripper: the copies are renamed
    QStringList motors;
    for (int id : cell.motorIdsSorted()) motors << cell.getMotor(id)->name;
    CHECK(motors.contains("X") && motors.contains("X_2") && motors.contains("Z_2"));
    const Signal* s2 = cell.findSignal("AtGripper_2");
    CHECK(s2 && s2->kind == SignalKind::Sensor);
    // The copy's sensor is mounted on the copy's gripper pad, not the cell's
    CHECK(s2 && scenequery::inSubtree(cell, s2->sensor.objectId, roots.front()));
    // The cell's own logic still uses its own names and objects
    CHECK(AutomationEngine::validateAll(cell).isEmpty());
    const Sequence& seq = cell.allSequences().begin()->second;
    for (const Operation& op : seq.operations)
        if (op.type == OperationType::Grip) {
            const Signal* used = cell.getSignal(op.sensorId);
            CHECK(used && used->name == "AtGripper");
        }

    // A gripper twice: the second copy's right finger follows its own left finger
    SceneGraph g;
    for (int k = 0; k < 2; ++k) {
        SceneGraph gripper;
        CHECK(ProjectSerializer::load(gripper, MS_SOURCE_DIR "/library/parallel_gripper.mstudio"));
        mergeScene(g, gripper);
    }
    CHECK(g.findSignal("InGripper") && g.findSignal("InGripper_2"));
    int coupled = 0;
    for (auto& [id, j] : g.allJoints()) {
        if (j.coupledTo == -1) continue;
        ++coupled;
        const Joint* master = g.getJoint(j.coupledTo);
        CHECK(master && g.getObject(master->parentBodyId) == g.getObject(j.parentBodyId));
    }
    CHECK(coupled == 2);
    CHECK(AutomationEngine::validateAll(g).isEmpty());
}

// ── URDF / MJCF ─────────────────────────────────────────────────────────────

// The 6-axis arm with a coloured box on each link, a cylinder, a mesh part
// and a coupled finger, so every kind of geometry and joint is exported
int makeDressedArm(SceneGraph& g, std::vector<int>* joints) {
    const int tool = makeSixAxis(g, joints);
    const int links[6] = {g.getJoint((*joints)[0])->childBodyId, g.getJoint((*joints)[1])->childBodyId,
                          g.getJoint((*joints)[2])->childBodyId, g.getJoint((*joints)[3])->childBodyId,
                          g.getJoint((*joints)[4])->childBodyId, g.getJoint((*joints)[5])->childBodyId};
    for (int i = 0; i < 6; ++i) {
        int box = addBox(g, QString("Box %1").arg(i + 1), {0.05f * float(i), 0.02f, 0.01f}, {0.08f, 0.05f + 0.01f * float(i), 0.06f}, links[i]);
        g.getObject(box)->transform.rotation = {10.0f * float(i), 5, -7};
        g.getObject(box)->color = QColor::fromHsv(i * 50, 200, 220);
    }
    int cyl = g.addObject("Shaft", PrimitiveType::Cylinder, links[2]);
    g.getObject(cyl)->transform.position = {0.2f, 0, 0.05f};
    g.getObject(cyl)->transform.scale = {0.04f, 0.3f, 0.04f};
    g.getObject(cyl)->transform.rotation = {0, 0, 90};
    int part = g.addObject("Housing", PrimitiveType::ImportedMesh, links[1]);
    auto mesh = std::make_shared<MeshGeometry>();
    mesh->interleaved = primitiveSoup(PrimitiveType::Sphere);
    mesh->aabbMin = {-0.5f, -0.5f, -0.5f};
    mesh->aabbMax = {0.5f, 0.5f, 0.5f};
    g.getObject(part)->mesh = mesh;
    g.getObject(part)->transform.scale = {0.1f, 0.1f, 0.1f};
    // A finger on the flange that follows J6 at half the angle
    int finger = addBox(g, "Finger", {0.02f, 0, 0}, {0.02f, 0.02f, 0.06f}, links[5]);
    Joint fj;
    fj.axis = {0, 0, 1};
    fj.pivot = {0.02f, 0, 0};
    fj.parentBodyId = links[5];
    fj.childBodyId = finger;
    fj.coupledTo = (*joints)[5];
    fj.ratio = 0.5f;
    fj.offset = 5;
    g.addJoint(fj);
    return tool;
}

// Sets joints by actuator name in both scenes and compares every part's
// world position (the imported scene names parts "<link> visual")
void compareScenes(SceneGraph& a, SceneGraph& b, const std::vector<std::vector<float>>& configs, const char* what) {
    double worst = 0;
    int compared = 0;
    for (const auto& q : configs) {
        for (SceneGraph* g : {&a, &b})
            for (int mid : g->motorIdsSorted()) {
                const QString n = g->getMotor(mid)->name;
                if (n.size() == 2 && n[0] == 'J') g->getJoint(g->getMotor(mid)->jointId)->currentValue = q[size_t(n[1].digitValue() - 1)];
            }
        a.applyCouplings();
        b.applyCouplings();
        for (auto& [id, o] : a.allObjects()) {
            if (!surfaceOf(o) && o.name != "Tool") continue;
            const QString name = o.name == "Tool" ? QString("Tool") : QString(o.name).replace(' ', '_') + " visual";
            int other = -1;
            for (auto& [bid, bo] : b.allObjects()) if (bo.name == name) other = bid;
            CHECK(other != -1);
            if (other == -1) continue;
            const QVector3D pa = surfaceOf(o) ? scenequery::worldBox(a, id).center : worldPos(a, id);
            const QVector3D pb = surfaceOf(*b.getObject(other)) ? scenequery::worldBox(b, other).center : worldPos(b, other);
            worst = std::max(worst, double((pa - pb).length()));
            ++compared;
        }
    }
    std::printf("  %s: %d part positions compared, worst %.2g m\n", what, compared, worst);
    CHECK(worst < 1e-4);
}

TEST(urdfAndMjcfRoundTrip) {
    SceneGraph g;
    std::vector<int> joints;
    makeDressedArm(g, &joints);
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    QTemporaryDir dir;
    const std::vector<std::vector<float>> configs = {{0, 0, 0, 0, 0, 0}, {30, -20, 40, 25, -35, 60}, {-90, 45, -30, 120, 15, -170}};
    for (bool mjcf : {false, true}) {
        const QString file = dir.filePath(mjcf ? "arm.xml" : "arm.urdf");
        const robotio::Result ex = mjcf ? robotio::exportMjcf(g, file) : robotio::exportUrdf(g, file);
        CHECK(ex.ok);
        CHECK(ex.files == 3);                                   // cube, cylinder, the mesh (sphere soup)
        SceneGraph h;
        const robotio::Result im = mjcf ? robotio::importMjcf(h, file) : robotio::importUrdf(h, file);
        if (!im.ok) std::printf("  import failed: %s\n", qPrintable(im.error));
        CHECK(im.ok);
        CHECK(h.allMotors().size() == 6);
        int couplings = 0;
        for (auto& [id, j] : h.allJoints()) if (j.coupledTo != -1) { ++couplings; CHECK_NEAR(j.ratio, 0.5, 1e-6); CHECK_NEAR(j.offset, 5, 1e-4); }
        CHECK(couplings == 1);
        CHECK(AutomationEngine::validateAll(h).isEmpty());
        compareScenes(g, h, configs, mjcf ? "MJCF" : "URDF");
        // Colours survive
        int coloured = 0;
        for (auto& [id, o] : h.allObjects()) if (o.color.isValid()) ++coloured;
        CHECK(coloured == 6);
    }
#ifdef MUJOCO_ENABLED
    // MuJoCo itself accepts the exported model: 6 actuated joints + the finger
    char err[1000] = {};
    mjModel* m = mj_loadXML(qPrintable(dir.filePath("arm.xml")), nullptr, err, sizeof err);
    if (!m) std::printf("  MuJoCo: %s\n", err);
    CHECK(m != nullptr);
    if (m) {
        CHECK(m->njnt == 7);
        CHECK(m->nu == 6);
        CHECK(m->neq == 1);
        mj_deleteModel(m);
    }
#endif
}

TEST(stlWriteAndRead) {
    QTemporaryDir dir;
    const std::vector<float>& soup = primitiveSoup(PrimitiveType::Cylinder);
    CHECK(robotio::writeStl(soup, dir.filePath("c.stl")));
    auto back = robotio::loadMesh(dir.filePath("c.stl"));
    CHECK(back && back->interleaved.size() == soup.size());
    if (back) {
        CHECK_NEAR(back->aabbMax.y(), 0.5, 1e-6);
        CHECK_NEAR(back->aabbMin.x(), -0.5, 1e-3);
    }
    QFile obj(dir.filePath("q.obj"));
    CHECK(obj.open(QIODevice::WriteOnly));
    obj.write("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n");
    obj.close();
    auto quad = robotio::loadMesh(dir.filePath("q.obj"));
    CHECK(quad && quad->interleaved.size() == 2 * 18);
}

// ── Templates ───────────────────────────────────────────────────────────────

TEST(everyTemplateValidatesAndRuns) {
    const QStringList files = QDir(MS_SOURCE_DIR "/templates").entryList({"*.mstudio"});
    CHECK(files.size() >= 7);
    for (const QString& f : files) {
        SceneGraph g;
        CHECK(ProjectSerializer::load(g, MS_SOURCE_DIR "/templates/" + f));
        const QStringList errs = AutomationEngine::validateAll(g);
        if (!errs.isEmpty()) std::printf("  %s: %s\n", qPrintable(f), qPrintable(errs.join("; ")));
        CHECK(errs.isEmpty());
        SimulationEngine sim(&g);
        bool moved = false;
        std::map<int, float> start;
        for (auto& [id, j] : g.allJoints()) start[id] = j.currentValue;
        // Switch every Bool input on (templates wait for Start)
        sim.step();
        for (auto& [id, s] : g.allSignals())
            if (s.kind == SignalKind::Input && s.type == SignalType::Bool && s.name != "Stop" && s.name != "Hold")
                sim.automation()->setInput(s.name, 1);
        for (int i = 0; i < 250; ++i) sim.step();
        for (auto& [id, j] : g.allJoints()) if (std::fabs(j.currentValue - start[id]) > 1e-3f) moved = true;
        if (!moved) std::printf("  %s: nothing moved\n", qPrintable(f));
        CHECK(moved);
        CHECK(sim.automation()->warnings().isEmpty());
        sim.reset();
        for (auto& [id, j] : g.allJoints()) CHECK_NEAR(j.currentValue, start[id], 1e-6);
    }
}

// ── Cycle time ──────────────────────────────────────────────────────────────

TEST(criticalPathThroughLongerBranch) {
    // 1 (0.5 s) → { 2 (0.3 s), 3 (0.6 s) } → 4 (0.1 s): 1 → 3 → 4 sets the cycle
    SceneGraph g;
    Sequence q;
    q.name = "Cell";
    Operation o1 = makeOp(g, OperationType::Wait);
    o1.duration = "0.5";
    Operation o2 = makeOp(g, OperationType::Wait, {o1.id});
    o2.duration = "0.3";
    Operation o3 = makeOp(g, OperationType::Wait, {o1.id});
    o3.duration = "0.6";
    Operation o4 = makeOp(g, OperationType::Wait, {o2.id, o3.id});
    o4.duration = "0.1";
    q.operations = {o1, o2, o3, o4};
    int sid = g.addSequence(q);
    AutomationEngine e(&g);
    CHECK(e.start(nullptr));
    runFor(e, 1.5);
    const auto st = e.seqStatus(sid);
    CHECK(st.cycles == 1);
    CHECK((st.criticalPath == std::vector<int>{o1.id, o3.id, o4.id}));
    CHECK_NEAR(st.lastCycleTime, 1.2, 0.05);
    CHECK_NEAR(st.criticalBusy, 1.2, 0.05);
}

// ── Interference ────────────────────────────────────────────────────────────

TEST(interferenceTrianglesExact) {
    SceneGraph g;
    int a = addBox(g, "A", {0, 0, 0}, {1, 1, 1});
    int b = addBox(g, "B", {0.9f, 0.3f, 0.2f}, {1, 1, 1});   // corners overlap 0.1 m
    int s = g.addObject("Ball", PrimitiveType::Sphere);
    g.getObject(s)->transform.position = {0, 0.95f, 0};   // a 0.5 m ball: its bottom pokes 0.05 into A
    InterferenceChecker c;
    CHECK(c.intersect(g, a, b));
    CHECK(c.intersect(g, a, s));
    g.getObject(b)->transform.position = {1.0f, 0.3f, 0.2f};   // faces touch exactly
    CHECK(!c.intersect(g, a, b));
    g.getObject(b)->transform.position = {1.2f, 0.3f, 0.2f};
    CHECK(!c.intersect(g, a, b));
    g.getObject(s)->transform.position = {0.9f, 0.9f, 0};   // boxes overlap, the ball misses the edge
    CHECK(scenequery::overlap(scenequery::worldBox(g, a), scenequery::worldBox(g, s)));
    CHECK(!c.intersect(g, a, s));
}

TEST(interferenceFoundDuringRun) {
    SceneGraph g;
    Arm a = makeArm(g);
    g.removeObject(a.part);
    // A post the arm sweeps through at about 45°
    const float r = 0.7f;
    int post = addBox(g, "Post", {r * 0.7071f, 0, -r * 0.7071f}, {0.06f, 1, 0.06f});
    Sequence q;
    q.name = "Sweep";
    Operation move = makeOp(g, OperationType::Move);
    move.actuatorId = a.motor;
    move.target = "90";
    q.operations = {move};
    g.addSequence(q);

    SimulationEngine sim(&g);
    sim.setInterferenceCheck(true);
    sim.step();
    CHECK(sim.interference().pairCount() > 0);
    for (int i = 0; i < 40; ++i) sim.step();                 // 0.66 s: done at 0.5 s
    const auto& ev = sim.interference().events();
    CHECK(ev.size() == 1);
    if (ev.size() == 1) {
        CHECK((std::set<int>{ev[0].a, ev[0].b} == std::set<int>{a.arm, post}));
        // The arm (0.1 thick) meets the 0.06 post around 45° = 0.25 s at 180°/s
        std::printf("  arm hits post from %.3f to %.3f s\n", ev[0].start, ev[0].end);
        CHECK(ev[0].start > 0.18 && ev[0].start < 0.25);
        CHECK(ev[0].end > 0.25 && ev[0].end < 0.33);
        CHECK(!ev[0].ongoing);
    }
    CHECK(sim.interference().interfering().empty());
    // The base and arm overlap from the start (assembled): not reported
    for (const auto& e : ev) CHECK(!(std::set<int>{e.a, e.b} == std::set<int>{a.base, a.arm}));
    sim.reset();
    CHECK(sim.interference().events().empty());
}

// ── Modbus TCP ──────────────────────────────────────────────────────────────

// Sends one request PDU and returns the response PDU (empty on timeout)
QByteArray modbus(QTcpSocket& sock, const QByteArray& pdu) {
    static quint16 tid = 0;
    QByteArray frame;
    ++tid;
    frame.append(char(tid >> 8)); frame.append(char(tid & 0xff));
    frame.append('\0'); frame.append('\0');
    frame.append(char((pdu.size() + 1) >> 8)); frame.append(char((pdu.size() + 1) & 0xff));
    frame.append(char(1));
    frame.append(pdu);
    sock.write(frame);
    QByteArray in;
    QElapsedTimer t;
    t.start();
    while (t.elapsed() < 2000) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        in.append(sock.readAll());
        if (in.size() >= 7 && in.size() >= 6 + ((quint8(in[4]) << 8) | quint8(in[5]))) break;
    }
    if (in.size() < 8 || ((quint8(in[0]) << 8) | quint8(in[1])) != tid) return {};
    return in.mid(7);
}
QByteArray pdu(std::initializer_list<int> bytes) {
    QByteArray b;
    for (int v : bytes) b.append(char(v));
    return b;
}
float wordsToFloat(const QByteArray& r, int at) {
    const quint32 bits = (quint32(quint8(r[at])) << 24) | (quint32(quint8(r[at + 1])) << 16) |
                         (quint32(quint8(r[at + 2])) << 8) | quint8(r[at + 3]);
    float f;
    std::memcpy(&f, &bits, 4);
    return f;
}

TEST(modbusServerReadsAndWritesSignals) {
    SceneGraph g;
    auto sig = [&](const char* name, SignalKind k, SignalType t, int addr, const char* expr = "", double init = 0) {
        Signal s;
        s.name = name; s.kind = k; s.type = t; s.ioAddress = addr; s.expression = expr; s.initialValue = init;
        return g.addSignal(s);
    };
    sig("Start", SignalKind::Input, SignalType::Bool, 0);
    sig("Speed", SignalKind::Input, SignalType::Real, 10, "", 1.5);
    sig("Running", SignalKind::Derived, SignalType::Bool, 0, "Start");
    sig("Temperature", SignalKind::Derived, SignalType::Real, 4, "Speed * 2");
    sig("Count", SignalKind::Internal, SignalType::Int, 0, "", -7);
    CHECK(AutomationEngine::validateAll(g).isEmpty());

    SimulationEngine sim(&g);
    ModbusServer server(&g, &sim);
    CHECK(server.start(0));                                    // any free port
    QTcpSocket sock;
    sock.connectToHost(QHostAddress::LocalHost, server.port());
    QElapsedTimer t;
    t.start();
    while (sock.state() != QAbstractSocket::ConnectedState && t.elapsed() < 2000)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
    CHECK(sock.state() == QAbstractSocket::ConnectedState);

    // Stopped: initial values
    QByteArray r = modbus(sock, pdu({3, 0, 10, 0, 2}));        // holding 10..11 = Speed
    CHECK(r.size() == 6 && r[0] == 3 && r[1] == 4);
    if (r.size() == 6) CHECK_NEAR(wordsToFloat(r, 2), 1.5, 1e-6);
    r = modbus(sock, pdu({4, 0, 0, 0, 1}));                    // input register 0 = Count
    CHECK(r.size() == 4 && qint16((quint8(r[2]) << 8) | quint8(r[3])) == -7);

    sim.step();                                                // simulation active
    r = modbus(sock, pdu({5, 0, 0, 0xFF, 0x00}));              // coil 0 (Start) ON
    CHECK(r == pdu({5, 0, 0, 0xFF, 0x00}));
    sim.step();
    CHECK(sim.automation()->value("Start") == 1.0);
    r = modbus(sock, pdu({2, 0, 0, 0, 8}));                    // discrete inputs 0..7
    CHECK(r == pdu({2, 1, 0x01}));                             // Running

    // Speed = 2.5 in one request (float32 high word first), then Temperature = 5
    float f = 2.5f;
    quint32 bits;
    std::memcpy(&bits, &f, 4);
    r = modbus(sock, pdu({16, 0, 10, 0, 2, 4, int(bits >> 24), int((bits >> 16) & 0xff), int((bits >> 8) & 0xff), int(bits & 0xff)}));
    CHECK(r == pdu({16, 0, 10, 0, 2}));
    sim.step();
    r = modbus(sock, pdu({4, 0, 4, 0, 2}));
    CHECK(r.size() == 6);
    if (r.size() == 6) CHECK_NEAR(wordsToFloat(r, 2), 5.0, 1e-6);

    // The halves of a Real in two single-register writes: 0.75
    f = 0.75f;
    std::memcpy(&bits, &f, 4);
    modbus(sock, pdu({6, 0, 10, int(bits >> 24), int((bits >> 16) & 0xff)}));
    modbus(sock, pdu({6, 0, 11, int((bits >> 8) & 0xff), int(bits & 0xff)}));
    sim.step();
    sim.step();
    CHECK_NEAR(sim.automation()->value("Speed"), 0.75, 1e-6);

    // Errors: unmapped coil, unknown function, bad coil value
    CHECK(modbus(sock, pdu({5, 0, 99, 0xFF, 0})) == pdu({0x85, 2}));
    CHECK(modbus(sock, pdu({43, 14, 1, 0})) == pdu({43 | 0x80, 1}));
    CHECK(modbus(sock, pdu({5, 0, 0, 0x12, 0x34})) == pdu({0x85, 3}));
    CHECK(server.requestCount() >= 10);
    server.stop();
    sim.reset();

    // Overlapping addresses: Speed takes holding 10 and 11
    sig("Limit", SignalKind::Input, SignalType::Int, 11);
    CHECK(!AutomationEngine::validateAll(g).isEmpty());
    g.findSignal("Limit")->ioAddress = -1;
    sig("Extra", SignalKind::Input, SignalType::Real, -1);
    CHECK(ModbusServer::assignAddresses(g) == 2);
    CHECK(g.findSignal("Limit")->ioAddress == 12);
    CHECK(g.findSignal("Extra")->ioAddress == 13);
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    SceneGraph h;
    roundTrip(g, h);
    CHECK(h.findSignal("Extra")->ioAddress == 13);
}

// ── Recording ───────────────────────────────────────────────────────────────

TEST(recordingScrubsAndReturnsLive) {
    SceneGraph g;
    Arm a = makeArm(g);
    Sequence q;
    q.name = "Swing";
    Operation grip = makeOp(g, OperationType::Grip);
    grip.gripperId = a.tool;
    grip.partId = a.part;
    Operation move = makeOp(g, OperationType::Move, {grip.id});
    move.actuatorId = a.motor;
    move.target = "90";
    q.operations = {grip, move};
    g.addSequence(q);

    SimulationEngine sim(&g);
    for (int i = 0; i < 50; ++i) sim.step();                 // 0.8 s: J1 reaches 90 at 0.5 s
    const Recording& rec = sim.recording();
    CHECK(rec.frames() == 51);                              // the start plus one per scan
    CHECK_NEAR(rec.duration(), 0.8, 1e-9);
    const float live = g.getJoint(a.joint)->currentValue;
    const QVector3D livePart = worldPos(g, a.part);
    CHECK_NEAR(live, 90, 1e-3);

    const int mid = rec.frameAt(0.25);                      // halfway through the move
    sim.seek(mid);
    CHECK(sim.isScrubbing());
    CHECK_NEAR(g.getJoint(a.joint)->currentValue, 180 * rec.time(mid), 1e-3);
    // Status slots are published at the start of a scan: one scan behind the joint
    CHECK_NEAR(sim.displayValue("J1.pos"), 180 * (rec.time(mid) - 0.016), 1e-3);
    CHECK_NEAR(sim.automation()->value("J1.pos"), 90, 1e-3);   // the engine itself stays live
    CHECK(worldPos(g, a.part).z() > -0.9f);                 // the gripped part was recorded too
    sim.seek(0);
    CHECK_NEAR(g.getJoint(a.joint)->currentValue, 0, 1e-6);
    CHECK_NEAR((worldPos(g, a.part) - QVector3D(1, 0, 0)).length(), 0, 1e-5);

    sim.seekLive();
    CHECK(!sim.isScrubbing());
    CHECK_NEAR(g.getJoint(a.joint)->currentValue, live, 1e-6);
    CHECK_NEAR((worldPos(g, a.part) - livePart).length(), 0, 1e-6);

    sim.seek(10);
    sim.step();                                             // Step resumes from the live state
    CHECK(!sim.isScrubbing());
    CHECK(rec.frames() == 52);
    CHECK_NEAR(g.getJoint(a.joint)->currentValue, 90, 1e-3);

    sim.reset();
    CHECK(rec.frames() == 0);
    CHECK_NEAR(g.getJoint(a.joint)->currentValue, 0, 1e-6);
}

#ifdef MUJOCO_ENABLED
TEST(gripLiftsRigidBodyAndDropsIt) {
    SceneGraph g;
    int frame = g.addObject("Frame", PrimitiveType::ImportedMesh);   // unscaled frame
    g.getObject(frame)->transform.position = {0, 1, 0};
    int slide = g.addObject("Slide", PrimitiveType::ImportedMesh, frame);
    g.getObject(slide)->transform.position = {0, -0.6f, 0};
    int part = addBox(g, "Part", {0, 0.05f, 0}, {0.1f, 0.1f, 0.1f});
    g.addRigidBody(part);
    Joint j;
    j.type = JointType::Prismatic;
    j.axis = {0, 1, 0};
    j.parentBodyId = frame;
    j.childBodyId = slide;
    j.limitMin = -1;
    j.limitMax = 1;
    int jid = g.addJoint(j);
    int lift = g.addActuatorForJoint(jid);
    g.getMotor(lift)->velocity = 1;
    Sequence q;
    q.name = "Lift";
    Operation grip = makeOp(g, OperationType::Grip);
    grip.gripperId = slide;
    grip.partId = part;
    Operation up = makeOp(g, OperationType::Move, {grip.id});
    up.actuatorId = lift;
    up.target = "0.5";
    Operation drop = makeOp(g, OperationType::Release, {up.id});
    drop.gripperId = slide;
    q.operations = {grip, up, drop};
    g.addSequence(q);

    SimulationEngine sim(&g);
    for (int i = 0; i < 40; ++i) sim.step();               // 0.64 s: lifting
    CHECK(g.getObject(part)->transform.position.y() > 0.4f);
    for (int i = 0; i < 25; ++i) sim.step();               // arrives, releases
    CHECK(sim.automation()->heldObjects().empty());
    for (int i = 0; i < 90; ++i) sim.step();               // falls back to the floor
    CHECK_NEAR(g.getObject(part)->transform.position.y(), 0.05, 0.02);
    sim.reset();
}

TEST(templateSensorCellPlacesAllParts) {
    SceneGraph g;
    CHECK(ProjectSerializer::load(g, MS_SOURCE_DIR "/templates/sensor_cell.mstudio"));
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    SimulationEngine sim(&g);
    int placed = 0;
    for (int i = 0; i < 1500 && placed < 3; ++i) {      // up to 24 s
        sim.step();
        placed = int(sim.automation()->value("Placed"));
    }
    std::printf("  placed %d parts in %.1f s\n", placed, sim.elapsedTime());
    CHECK(placed == 3);
    for (int i = 0; i < 60; ++i) sim.step();            // let the last one land
    // Every part ended up inside the bin (x 0.95..1.35, below its rim)
    for (auto& [id, o] : g.allObjects()) {
        if (!o.name.startsWith("Part")) continue;
        const QVector3D p = o.transform.position;
        CHECK(p.x() > 0.95f && p.x() < 1.35f && p.y() < 0.3f);
    }
    sim.reset();
}

TEST(sensorFollowsPhysics) {
    SceneGraph g;
    int belt = addBox(g, "Conveyor", {0, 0.1f, 0}, {2, 0.2f, 0.6f});
    int post = addBox(g, "Post", {0.5f, 0.25f, 0.4f}, {0.05f, 0.5f, 0.05f});
    int part = addBox(g, "Part", {0.5f, 0.3f, 0}, {0.15f, 0.15f, 0.15f});
    g.addRigidBody(part);
    g.addConveyorBelt(belt);
    g.getConveyorBelt(belt)->velocity = 0.5f;
    g.getConveyorBelt(belt)->direction = {1, 0, 0};
    Signal s;
    s.name = "PartAtSensor";
    s.kind = SignalKind::Sensor;
    s.sensor.objectId = post;
    s.sensor.offset = {0, 0.05f, 0};
    s.sensor.direction = {0, 0, -1};
    s.sensor.range = 0.8f;
    s.sensor.target = SensorTarget::RigidBodies;
    g.addSignal(s);

    SimulationEngine sim(&g);
    sim.step();
    CHECK(sim.automation()->value("PartAtSensor") == 1.0);
    for (int i = 0; i < 90; ++i) sim.step();          // 1.4 s at 0.5 m/s
    CHECK(g.getObject(part)->transform.position.x() > 0.7f);
    CHECK(sim.automation()->value("PartAtSensor") == 0.0);
    sim.reset();
    CHECK_NEAR(g.getObject(part)->transform.position.x(), 0.5, 1e-6);
}
#endif

} // namespace

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    const QString filter = argc > 1 ? QString(argv[1]) : QString();
    int run = 0;
    for (const Test& t : registry()) {
        if (!filter.isEmpty() && !QString(t.name).contains(filter)) continue;
        const int before = g_failures;
        t.fn();
        ++run;
        std::printf("%s %s\n", g_failures == before ? "ok  " : "FAIL", t.name);
    }
    std::printf("\n%d tests, %d checks, %d failures\n", run, g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
