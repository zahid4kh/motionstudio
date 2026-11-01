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
#include "scene/UndoManager.h"
#include "automation/AutomationEngine.h"
#include "automation/Sensors.h"
#include "physics/SimulationEngine.h"
#include "serialization/ProjectSerializer.h"

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
