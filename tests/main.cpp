// Headless regression tests for MotionStudioCore. Build with
//   cmake -B build -DMOTIONSTUDIO_TESTS=ON && cmake --build build
// and run ./build/MotionStudioTests [filter]. Exit code 0 means all passed.
#include <QCoreApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <cmath>
#include <cstdio>
#include <functional>
#include <vector>
#include <set>
#include "scene/SceneGraph.h"
#include "scene/PrimitiveMesh.h"
#include "scene/UndoManager.h"
#include "automation/AutomationEngine.h"
#include "automation/Sensors.h"
#include "automation/Kinematics.h"
#include "automation/Take.h"
#include "physics/SimulationEngine.h"
#include "serialization/ProjectSerializer.h"
#include "scene/Interference.h"
#include "io/ModbusServer.h"
#include "io/ApiServer.h"
#include "scene/SceneMerge.h"
#include "scene/Assembly.h"
#include "io/RobotFormats.h"
#include "import/StepImporter.h"
#include "robot/RobotBuilder.h"
#include "robot/RobotAnalysis.h"
#include "robot/HumanoidBuilder.h"
#include "robot/Gait.h"
#include <QTemporaryDir>
#include <QtMath>
#ifdef MUJOCO_ENABLED
#include <mujoco/mujoco.h>
#endif
#include <QDir>
#include <QFile>
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

TEST(opacityMultipliesDownDuplicatesAndSaves) {
    SceneGraph g;
    int group = g.addObject("Assembly", PrimitiveType::ImportedMesh);
    int a = addBox(g, "A", {0, 0, 0}, {1, 1, 1}, group);
    int b = addBox(g, "B", {1, 0, 0}, {1, 1, 1}, group);
    CHECK(g.opacityOf(a) == 1.0f);
    g.getObject(group)->opacity = 0.5f;
    g.getObject(b)->opacity = 0.4f;
    CHECK(std::abs(g.opacityOf(a) - 0.5f) < 1e-6f);
    CHECK(std::abs(g.opacityOf(b) - 0.2f) < 1e-6f);
    int copy = g.duplicateObject(group);
    const SceneObject* c = g.getObject(copy);
    CHECK(c && c->opacity == 0.5f);
    CHECK(c && std::abs(g.opacityOf(c->childIds[1]) - 0.2f) < 1e-6f);

    // Solid objects write nothing, so older files and solid scenes look the same
    const QJsonObject json = ProjectSerializer::toJson(g);
    int written = 0;
    for (const QJsonValue& v : json["objects"].toArray()) written += v.toObject().contains("opacity");
    CHECK(written == 4);   // two assemblies, two B parts

    SceneGraph h;
    roundTrip(g, h);
    int n = 0;
    for (auto& [id, o] : h.allObjects()) {
        if (o.name == "A") { CHECK(o.opacity == 1.0f && std::abs(h.opacityOf(id) - 0.5f) < 1e-6f); ++n; }
        if (o.name == "B") { CHECK(std::abs(o.opacity - 0.4f) < 1e-6f); ++n; }
    }
    CHECK(n == 4);

    // A slider drag is one undo step
    SceneGraph u;
    UndoManager undo(&u);
    const int x = addBox(u, "X", {0, 0, 0}, {1, 1, 1});
    undo.reset();
    for (float v : {0.9f, 0.6f, 0.3f}) { u.getObject(x)->opacity = v; emit u.objectChanged(x); }
    QElapsedTimer wait; wait.start();
    while (!undo.canUndo() && wait.elapsed() < 2000) QCoreApplication::processEvents();
    CHECK(undo.canUndo());
    undo.undo();
    CHECK(u.allObjects().size() == 1 && u.allObjects().begin()->second.opacity == 1.0f);
    CHECK(!undo.canUndo());
    undo.redo();
    CHECK(std::abs(u.allObjects().begin()->second.opacity - 0.3f) < 1e-6f);
}

TEST(hiddenPartsHideTheirSubtreeSaveAndStillSense) {
    SceneGraph g;
    int group = g.addObject("Assembly", PrimitiveType::ImportedMesh);
    int a = addBox(g, "A", {0, 0.5f, 0}, {0.2f, 0.2f, 0.2f}, group);
    int b = addBox(g, "B", {2, 0.5f, 0}, {0.2f, 0.2f, 0.2f});
    CHECK(!g.isHidden(a) && g.hiddenBy(a) == -1);

    // Alt+V on a shown selection hides it all, again shows it
    CHECK(g.toggleHidden({group, b}));
    CHECK(g.isHidden(a) && g.hiddenBy(a) == group && g.hiddenBy(b) == b);
    CHECK(g.opacityOf(a) == 0.0f && g.opacityOf(b) == 0.0f);
    CHECK(!g.getObject(a)->hidden);                  // the child's own flag is untouched
    CHECK(!g.toggleHidden({group, b}));
    CHECK(!g.isHidden(a) && !g.isHidden(b) && g.opacityOf(a) == 1.0f);

    // Mixed selection: hides the shown ones too
    g.setHidden(b, true);
    CHECK(g.toggleHidden({a, b}) && g.getObject(a)->hidden && g.getObject(b)->hidden);
    g.setHidden(a, false);
    g.setHidden(b, false);

    // Showing a part inside a hidden parent leaves it hidden by the parent
    g.setHidden(group, true);
    g.setHidden(a, true);
    CHECK(!g.toggleHidden({a}) && !g.getObject(a)->hidden && g.hiddenBy(a) == group);

    // Hidden is a view thing: picking skips it, sensors still see it
    g.setHidden(group, false);
    g.setHidden(b, true);
    int post = addBox(g, "Post", {1, 0.5f, 0}, {0.1f, 0.1f, 0.1f});
    SensorConfig c;
    c.objectId = post;
    c.direction = {1, 0, 0};
    c.range = 2.0f;
    CHECK(sensors::read(g, c).objectId == b);

    // Saved only when set; duplicates keep it
    int copy = g.duplicateObject(b);
    CHECK(g.getObject(copy) && g.getObject(copy)->hidden);
    const QJsonObject json = ProjectSerializer::toJson(g);
    int written = 0;
    for (const QJsonValue& v : json["objects"].toArray()) written += v.toObject().contains("hidden");
    CHECK(written == 2);
    SceneGraph h;
    roundTrip(g, h);
    int n = 0;
    for (auto& [id, o] : h.allObjects()) {
        if (o.name.startsWith("B")) { CHECK(o.hidden && h.isHidden(id)); ++n; }
        if (o.name == "A") { CHECK(!o.hidden && !h.isHidden(id)); ++n; }
    }
    CHECK(n == 3);

    // One Alt+V is one undo step
    SceneGraph u;
    UndoManager undo(&u);
    const int x = addBox(u, "X", {0, 0, 0}, {1, 1, 1});
    const int y = addBox(u, "Y", {2, 0, 0}, {1, 1, 1});
    undo.reset();
    u.toggleHidden({x, y});
    QElapsedTimer wait; wait.start();
    while (!undo.canUndo() && wait.elapsed() < 2000) QCoreApplication::processEvents();
    CHECK(undo.canUndo());
    undo.undo();
    for (auto& [id, o] : u.allObjects()) CHECK(!o.hidden);
    CHECK(!undo.canUndo());
    undo.redo();
    for (auto& [id, o] : u.allObjects()) CHECK(o.hidden);
}

TEST(undoKeepsTheSelectedPart) {
    SceneGraph u;
    UndoManager undo(&u);
    auto settle = [&]{
        QElapsedTimer wait; wait.start();
        while (wait.elapsed() < 700) QCoreApplication::processEvents();
    };
    addBox(u, "Base", {0, 0, 0}, {1, 1, 1});
    const int asm_ = u.addObject("Asm", PrimitiveType::ImportedMesh);
    addBox(u, "P", {0, 0, 0}, {1, 1, 1}, asm_);
    const int p2 = addBox(u, "P", {1, 0, 0}, {1, 1, 1}, asm_);
    addBox(u, "Q", {2, 0, 0}, {1, 1, 1}, asm_);
    undo.reset();

    // Deleting leaves a gap in the ids, which the restore closes up
    u.removeObject(u.rootObjects().front());
    settle();
    u.selectObject(p2);
    u.getObject(p2)->opacity = 0.3f;
    emit u.objectChanged(p2);
    settle();

    auto selected = [&]() -> const SceneObject* { return u.getObject(u.selectedObjectId()); };
    undo.undo();
    const SceneObject* s = selected();
    CHECK(s && s->name == "P" && s->opacity == 1.0f && u.getObject(s->parentId)->childIds[1] == s->id);
    undo.undo();   // Base is back, ids shift again
    s = selected();
    CHECK(!undo.canUndo());
    CHECK(u.allObjects().size() == 5);
    CHECK(s && s->name == "P" && u.getObject(s->parentId)->childIds[1] == s->id);
    undo.redo();
    undo.redo();
    s = selected();
    CHECK(s && s->name == "P" && std::abs(s->opacity - 0.3f) < 1e-6f);

    u.deselectAll();
    undo.undo();
    CHECK(u.selectedObjectId() == -1);
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

// A pose taught on a part: the robot goes where the tool was relative to
// the part, wherever the part is now (MoveToPose by IK, MoveL in a line)
TEST(poseTaughtOnPartFollowsThePart) {
    SceneGraph g;
    std::vector<int> joints;
    int tool = makeSixAxis(g, &joints);
    setJoints(g, joints, {10, 30, 60, 0, 40, 0});
    const QVector3D at = kin::toolFrame(g, tool).column(3).toVector3D();
    int part = addBox(g, "Part", at + QVector3D(0, -0.03f, 0), {0.05f, 0.05f, 0.05f});
    g.getObject(part)->transform.rotation = {0, 20, 0};
    Pose p;
    p.name = "Pick";
    kin::teachOnPart(g, &p, tool, part);
    CHECK(p.isRelative() && p.toolId == tool && p.jointValues.size() == 6);
    const int pid = g.addPose(p);
    // The part goes somewhere else; the robot starts elsewhere too
    g.getObject(part)->transform.position += QVector3D(0.04f, -0.03f, 0.05f);
    g.getObject(part)->transform.rotation = {0, 35, 0};
    setJoints(g, joints, {0, 10, 50, 0, 30, 0});
    const QMatrix4x4 goal = kin::poseTarget(g, *g.getPose(pid), part);

    Sequence q;
    q.name = "Go";
    Operation op = makeOp(g, OperationType::MoveToPose);
    op.poseId = pid;
    q.operations = {op};
    int sid = g.addSequence(q);
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    AutomationEngine e(&g);
    CHECK(e.start(nullptr));
    for (int i = 0; i < 400 && e.seqStatus(sid).state != AutomationEngine::SeqState::Finished; ++i) e.scan(0.016);
    CHECK(e.seqStatus(sid).state == AutomationEngine::SeqState::Finished);
    const QMatrix4x4 got = kin::toolFrame(g, tool);
    CHECK_NEAR((got.column(3) - goal.column(3)).toVector3D().length(), 0, 2e-4);
    CHECK(std::fabs(QQuaternion::dotProduct(kin::targetFrom(got).orientation, kin::targetFrom(goal).orientation)) > 0.9999f);
    e.stop();

    // MoveL to the same pose after the part moved again: a straight line onto it
    g.getObject(part)->transform.position += QVector3D(-0.03f, 0.02f, -0.04f);
    const QMatrix4x4 goal2 = kin::poseTarget(g, *g.getPose(pid), part);
    g.getSequence(sid)->operations[0].type = OperationType::MoveL;
    g.getSequence(sid)->operations[0].velocity = "0.2";
    CHECK(AutomationEngine::validateAll(g).isEmpty());            // the tool is left to the pose
    CHECK(e.start(nullptr));
    for (int i = 0; i < 400 && e.seqStatus(sid).state != AutomationEngine::SeqState::Finished; ++i) e.scan(0.016);
    CHECK_NEAR((kin::toolFrame(g, tool).column(3) - goal2.column(3)).toVector3D().length(), 0, 2e-4);
    e.stop();

    // A MoveL tool other than the pose's is reported; so is a deleted part
    g.getSequence(sid)->operations[0].toolId = joints.empty() ? -1 : g.getJoint(joints[4])->childBodyId;
    CHECK(!AutomationEngine::validateAll(g).isEmpty());
    g.getSequence(sid)->operations[0].toolId = tool;
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    SceneGraph h;
    roundTrip(g, h);
    CHECK(AutomationEngine::validateAll(h).isEmpty());
    const Pose& hp = h.allPoses().begin()->second;
    CHECK(hp.isRelative() && h.getObject(hp.frameId) && h.getObject(hp.frameId)->name == "Part");
    CHECK_NEAR((hp.relPosition - g.getPose(pid)->relPosition).length(), 0, 1e-6);
    g.removeObject(part);
    CHECK(!AutomationEngine::validateAll(g).isEmpty());
}

// A position sensor finds a copy made by a source; the robot goes to the
// copy (not the template it was taught on), and the sensor publishes where it is
TEST(robotPicksCopyFoundByPositionSensor) {
    SceneGraph g;
    std::vector<int> joints;
    int tool = makeSixAxis(g, &joints);
    setJoints(g, joints, {10, 30, 60, 0, 40, 0});
    const QVector3D at = kin::toolFrame(g, tool).column(3).toVector3D();
    int part = addBox(g, "Part", at + QVector3D(0, -0.03f, 0), {0.05f, 0.05f, 0.05f});
    Pose p;
    p.name = "Pick";
    kin::teachOnPart(g, &p, tool, part);
    const int pid = g.addPose(p);
    setJoints(g, joints, {0, 10, 50, 0, 30, 0});

    PartSource src;
    src.name = "Feeder";
    src.templateId = part;
    src.interval = "100";
    const int srcId = g.addSource(src);
    Signal cam;
    cam.name = "Cam";
    cam.kind = SignalKind::Sensor;
    cam.sensor.type = SensorType::Position;
    cam.sensor.offset = at;                      // a world-fixed zone around the spot
    cam.sensor.size = {0.4f, 0.4f, 0.4f};
    cam.sensor.target = SensorTarget::Object;    // the template matches its copies
    cam.sensor.targetId = part;
    const int camId = g.addSignal(cam);
    Sequence q;
    q.name = "Pick";
    Operation op = makeOp(g, OperationType::MoveL);
    op.poseId = pid;
    op.sensorId = camId;
    op.velocity = "0.3";
    q.operations = {op};
    int sid = g.addSequence(q);
    CHECK(AutomationEngine::validateAll(g).isEmpty());

    AutomationEngine e(&g);
    CHECK(e.start(nullptr));
    CHECK(g.isParked(part));
    e.scan(0.016);                               // the copy appears at the end of the scan
    CHECK(e.sourceCount(srcId) == 1);
    int copy = -1;
    for (auto& [id, o] : g.allObjects()) if (o.copyOf == part) copy = id;
    CHECK(copy != -1 && g.getObject(copy)->parentId == -1 && g.getObject(copy)->sourceId == srcId);
    CHECK_NEAR((worldPos(g, copy) - worldPos(g, part)).length(), 0, 1e-6);
    // It lands a little off the template's spot, turned
    g.getObject(copy)->transform.position += QVector3D(0.05f, 0, -0.04f);
    g.getObject(copy)->transform.rotation = {0, 25, 0};
    const QMatrix4x4 goal = kin::poseTarget(g, *g.getPose(pid), copy);
    for (int i = 0; i < 400 && e.seqStatus(sid).state != AutomationEngine::SeqState::Finished; ++i) e.scan(0.016);
    CHECK(e.seqStatus(sid).state == AutomationEngine::SeqState::Finished);
    CHECK_NEAR((kin::toolFrame(g, tool).column(3) - goal.column(3)).toVector3D().length(), 0, 2e-4);
    CHECK(e.sensorDetection(camId) == copy);
    CHECK(e.value("Cam") == 1.0);
    const QVector3D rel = worldPos(g, copy) - at;
    CHECK_NEAR(e.value("Cam.x"), rel.x(), 1e-5);
    CHECK_NEAR(e.value("Cam.y"), rel.y(), 1e-5);
    CHECK_NEAR(e.value("Cam.z"), rel.z(), 1e-5);
    CHECK_NEAR(e.value("Cam.ry"), 25, 1e-3);
    // Stopping removes the copy and brings the template back
    e.stop();
    CHECK(g.getObject(copy) == nullptr);
    CHECK(!g.isParked(part));
}

// ── Part flow ───────────────────────────────────────────────────────────────

TEST(sourceMakesCopiesAndSinkTakesThem) {
    SceneGraph g;
    int part = addBox(g, "Box", {0, 0.5f, 0}, {0.1f, 0.1f, 0.1f});
    g.getObject(part)->color = QColor(200, 30, 30);
    g.addRigidBody(part);
    int bin = addBox(g, "Bin", {2, 0.2f, 0}, {0.4f, 0.4f, 0.4f});
    PartSource src;
    src.name = "Feeder";
    src.templateId = part;
    src.interval = "0.5";
    src.whenClear = false;
    src.maxCount = 3;
    const int srcId = g.addSource(src);
    PartSink sink;
    sink.name = "Out";
    sink.zone.objectId = bin;
    sink.zone.objectBounds = true;
    sink.sourceId = srcId;
    const int sinkId = g.addSink(sink);
    Signal total;
    total.name = "InSystem";
    total.kind = SignalKind::Derived;
    total.type = SignalType::Int;
    total.expression = "Feeder.count - Out.count";
    g.addSignal(total);
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    CHECK(g.isSymbolNameTaken("Feeder") && g.isSymbolNameTaken("Out"));

    AutomationEngine e(&g);
    CHECK(e.start(nullptr));
    runFor(e, 1.2);                               // copies at 0, 0.5 and 1.0 s, then the limit
    CHECK(e.sourceCount(srcId) == 3);
    std::vector<int> copies;
    for (auto& [id, o] : g.allObjects()) if (o.copyOf == part) copies.push_back(id);
    std::sort(copies.begin(), copies.end());
    CHECK(copies.size() == 3);
    CHECK(g.getObject(copies[0])->name == "Box #1");
    CHECK(g.getRigidBody(copies[0]) != nullptr);  // copies are parts too
    CHECK(g.getObject(copies[0])->color == QColor(200, 30, 30));
    // The template is out of the run: not sensed, not picked, not drawn
    CHECK(g.isParked(part) && g.opacityOf(part) == 0.0f);
    SensorConfig look;
    look.objectId = -1;
    look.offset = {-1, 0.5f, 0};
    look.range = 2;
    CHECK(sensors::read(g, look).objectId == copies[0] || sensors::read(g, look).objectId == copies[1] ||
          sensors::read(g, look).objectId == copies[2]);
    // Saving while simulating leaves the copies out
    SceneGraph saved;
    roundTrip(g, saved);
    CHECK(saved.allObjects().size() == 2 && saved.allRigidBodies().size() == 1);
    CHECK(saved.allSources().size() == 1 && saved.allSinks().size() == 1);
    CHECK(saved.allSinks().begin()->second.sourceId == saved.allSources().begin()->first);

    // A copy carried into the bin is taken; one held by a gripper is not
    g.getObject(copies[0])->transform.position = {2, 0.3f, 0};
    e.scan(0.016);
    CHECK(e.sinkCount(sinkId) == 1 && g.isParked(copies[0]));
    e.scan(0.016);
    CHECK(e.value("Out.count") == 1 && e.value("Feeder.count") == 3 && e.value("InSystem") == 2);
    e.stop();
    CHECK(g.allObjects().size() == 2 && !g.isParked(part) && g.allRigidBodies().size() == 1);

    // Wait until the spot is clear: one copy, then blocked until it moves
    g.getSource(srcId)->whenClear = true;
    g.getSource(srcId)->maxCount = 0;
    CHECK(e.start(nullptr));
    runFor(e, 1.2);
    CHECK(e.sourceCount(srcId) == 1);
    CHECK(e.value("Feeder.blocked") == 1.0);
    int first = -1;
    for (auto& [id, o] : g.allObjects()) if (o.copyOf == part) first = id;
    g.getObject(first)->transform.position = {0.5f, 0.5f, 0};   // moved on
    runFor(e, 0.05);
    CHECK(e.sourceCount(srcId) == 2);
    e.stop();

    // Trigger mode: a copy per rising edge of an input
    Signal feed;
    feed.name = "Feed";
    feed.kind = SignalKind::Input;
    g.addSignal(feed);
    g.getSource(srcId)->mode = SourceMode::Trigger;
    g.getSource(srcId)->trigger = "Feed";
    g.getSource(srcId)->whenClear = false;
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    CHECK(e.start(nullptr));
    runFor(e, 0.2);
    CHECK(e.sourceCount(srcId) == 0);
    for (int k = 0; k < 2; ++k) {
        e.setInput("Feed", 1);
        e.setInput("Feed", 0);
    }
    runFor(e, 0.2);
    CHECK(e.sourceCount(srcId) == 2);
    e.stop();

    // Renames reach the source's expressions; a deleted template is reported
    g.renameSymbolReferences("Feed", "Load");
    CHECK(g.getSource(srcId)->trigger == "Load");
    g.removeObject(part);
    CHECK(!AutomationEngine::validateAll(g).isEmpty());
}

// Inserting a component remaps its sources, sinks and part poses, and
// renames clashing source / sink names; undo restores them
TEST(partFlowMergesAndUndoes) {
    SceneGraph comp;
    std::vector<int> joints;
    int tool = makeSixAxis(comp, &joints);
    setJoints(comp, joints, {10, 30, 60, 0, 40, 0});
    int part = addBox(comp, "Part", kin::toolFrame(comp, tool).column(3).toVector3D(), {0.05f, 0.05f, 0.05f});
    Pose p;
    p.name = "Pick";
    kin::teachOnPart(comp, &p, tool, part);
    comp.addPose(p);
    PartSource src;
    src.name = "Feeder";
    src.templateId = part;
    int srcId = comp.addSource(src);
    PartSink sink;
    sink.name = "Out";
    sink.zone.objectId = part;
    sink.zone.objectBounds = true;
    sink.sourceId = srcId;
    comp.addSink(sink);

    SceneGraph g;
    PartSource mine;
    mine.name = "Feeder";
    mine.templateId = addBox(g, "Other", {}, {0.1f, 0.1f, 0.1f});
    g.addSource(mine);
    UndoManager undo(&g);
    mergeScene(g, comp, -1);
    CHECK(g.allSources().size() == 2 && g.allSinks().size() == 1);
    const PartSink& gs = g.allSinks().begin()->second;
    const PartSource* merged = g.getSource(gs.sourceId);
    CHECK(merged && merged->name != "Feeder" && g.getObject(merged->templateId)->name == "Part");
    CHECK(g.getObject(gs.zone.objectId) && g.getObject(gs.zone.objectId)->name == "Part");
    const Pose& gp = g.allPoses().begin()->second;
    CHECK(gp.isRelative() && g.getObject(gp.frameId)->name == "Part" && g.getObject(gp.toolId)->name == "Tool");
    CHECK(gp.jointValues.size() == 6);
    for (auto& [jid, v] : gp.jointValues) CHECK(g.getJoint(jid) != nullptr);
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    QCoreApplication::processEvents();
    undo.undo();
    CHECK(g.allSources().size() == 1 && g.allSinks().empty() && g.allPoses().empty());
}

TEST(distanceSensorMeasuresExactly) {
    SceneGraph g;
    // A sphere of radius 0.15 straight ahead, centre 1 m away
    int ball = g.addObject("Ball", PrimitiveType::Sphere);
    g.getObject(ball)->transform.position = {1, 0, 0};
    g.getObject(ball)->transform.scale = {0.3f, 0.3f, 0.3f};
    SensorConfig c;
    c.type = SensorType::Distance;
    c.direction = {1, 0, 0};
    c.range = 2;
    sensors::Reading r = sensors::read(g, c);
    CHECK(r.active && r.objectId == ball);
    CHECK_NEAR(r.value(c), 0.85, 1e-6);
    // Off-centre the faceted mesh would be up to ~1 mm out; the true sphere is used
    c.offset = {0, 0.1f, 0};
    CHECK_NEAR(sensors::read(g, c).distance, 1 - std::sqrt(0.15 * 0.15 - 0.1 * 0.1), 1e-6);
    // Nothing in range: the range
    c.offset = {0, 0.5f, 0};
    r = sensors::read(g, c);
    CHECK(!r.active && r.value(c) == 2.0);

    // A cylinder's curved side (radius 0.2, axis Y) and its flat cap
    int can = g.addObject("Can", PrimitiveType::Cylinder);
    g.getObject(can)->transform.position = {0, 0, 3};
    g.getObject(can)->transform.scale = {0.4f, 0.4f, 0.4f};
    SensorConfig d;
    d.type = SensorType::Distance;
    d.offset = {0.07f, 0, 0};
    d.direction = {0, 0, 1};
    d.range = 5;
    CHECK_NEAR(sensors::read(g, d).distance, 3 - std::sqrt(0.2 * 0.2 - 0.07 * 0.07), 1e-6);
    d.offset = {0.05f, 1, 3};
    d.direction = {0, -1, 0};
    CHECK_NEAR(sensors::read(g, d).distance, 0.8, 1e-6);     // top cap at y = 0.2

    // A capsule's rounded end, turned on its side: radius 0.1, end centre 0.1 from its middle
    int pill = g.addObject("Pill", PrimitiveType::Capsule);
    g.getObject(pill)->transform.position = {0, 0, -2};
    g.getObject(pill)->transform.scale = {0.4f, 0.4f, 0.4f};
    g.getObject(pill)->transform.rotation = {0, 0, 90};       // its Y axis along −X
    SensorConfig e;
    e.type = SensorType::Distance;
    e.offset = {-1, 0.03f, -2};
    e.direction = {1, 0, 0};
    e.range = 3;
    CHECK_NEAR(sensors::read(g, e).distance, 1 - 0.1 - std::sqrt(0.01 - 0.0009), 1e-5);

    // In the engine and the file: a Real signal holding metres
    Signal s;
    s.name = "Gap";
    s.kind = SignalKind::Sensor;
    s.sensor = c;
    s.sensor.offset = {};
    g.addSignal(s);
    CHECK(signalNamed(g, "Gap")->type == SignalType::Real);
    AutomationEngine eng(&g);
    CHECK(eng.start(nullptr));
    eng.scan(0.016);
    CHECK_NEAR(eng.value("Gap"), 0.85, 1e-6);
    g.getObject(ball)->transform.position = {1.25f, 0, 0};
    eng.scan(0.016);
    CHECK_NEAR(eng.value("Gap"), 1.1, 1e-6);
    eng.stop();
    SceneGraph h;
    roundTrip(g, h);
    const Signal* hs = signalNamed(h, "Gap");
    CHECK(hs && hs->sensor.type == SensorType::Distance && hs->type == SignalType::Real);
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

// Copies from the source ride the belt; the gantry finds each with the
// position sensor, picks it where it stopped and the bin's sink takes it
TEST(templatePartFlowCellPicksFedParts) {
    SceneGraph g;
    CHECK(ProjectSerializer::load(g, MS_SOURCE_DIR "/templates/part_flow_cell.mstudio"));
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    // The help's examples for this cell compile against it
    QFile flow(MS_SOURCE_DIR "/resources/help/flow.md");
    CHECK(flow.open(QIODevice::ReadOnly));
    const QStringList blocks = QString::fromUtf8(flow.readAll()).split("```");
    QStringList examples;
    for (int i = 1; i < blocks.size(); i += 2) examples << blocks[i].trimmed();
    examples << "Height < 0.599 ? 0.6 - Height : 0" << "Height < 0.599 && abs(0.6 - Height - 0.12) > 0.002"
             << "Locator && Locator.z > 0.05";
    CHECK(examples.size() >= 6);
    for (const QString& ex : examples) {
        const QString err = AutomationEngine::validateExpression(g, ex);
        if (!err.isEmpty()) std::printf("  help example '%s': %s\n", qPrintable(ex), qPrintable(err));
        CHECK(err.isEmpty());
    }
    SimulationEngine sim(&g);
    int binned = 0;
    double height = 0;
    for (int i = 0; i < 2500 && binned < 3; ++i) {      // up to 40 s
        sim.step();
        binned = int(sim.automation()->value("Binned.count"));
        height = std::max(height, sim.automation()->value("PartHeight"));
    }
    std::printf("  %d parts binned in %.1f s, %g made; laser measured %.4f m high parts\n", binned,
                sim.elapsedTime(), sim.automation()->value("Feeder.count"), height);
    CHECK(binned == 3);
    CHECK(sim.automation()->value("Placed") >= 3);
    CHECK_NEAR(height, 0.12, 3e-3);                     // the boxes are 0.12 m; contact sinks them slightly
    CHECK(sim.automation()->warnings().isEmpty());
    if (!sim.automation()->warnings().isEmpty())
        std::printf("  %s\n", qPrintable(sim.automation()->warnings().join("; ")));
    sim.reset();
    CHECK(g.allObjects().size() == 13);
}

// The 6-axis robot finds each fed part, picks it from above and sets it on
// the outfeed, whose sink takes it; opening the door holds the robot
TEST(templateRobotCellPicksAndPlaces) {
    SceneGraph g;
    CHECK(ProjectSerializer::load(g, MS_SOURCE_DIR "/templates/robot_cell.mstudio"));
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    int tcp = -1;
    for (auto& [id, o] : g.allObjects()) if (o.name == "TCP") tcp = id;
    CHECK(kin::chainTo(g, tcp).size() == 6);
    // Every pose points the tool straight down and is reachable as taught
    for (auto& [id, p] : g.allPoses()) {
        if (p.name == "Home") continue;
        const QMatrix4x4 f = kin::toolFrameAt(g, tcp, p.jointValues);
        CHECK(f.mapVector({0, -1, 0}).y() < -0.9999f);
    }
    SimulationEngine sim(&g);
    sim.step();
    sim.automation()->setInput("Start", 1);
    int shipped = 0;
    float worstGrip = 0, worstTilt = 0;
    for (int i = 0; i < 2500 && shipped < 3; ++i) {     // up to 40 s
        sim.step();
        shipped = int(sim.automation()->value("Shipped.count"));
        // A held part sits centred between the fingertips; parts on the outfeed stay flat
        const QVector3D at = kin::toolFrame(g, tcp).column(3).toVector3D();
        for (auto& [id, o] : g.allObjects()) {
            if (o.copyOf == -1 || o.parked) continue;
            const QMatrix4x4 w = g.worldTransform(id);
            const QVector3D p = w.column(3).toVector3D();
            if (sim.automation()->holderOf(id) != -1) worstGrip = std::max(worstGrip, (p - at).length());
            else if (p.z() > 0.5f) worstTilt = std::max(worstTilt, 1 - w.mapVector({0, 1, 0}).normalized().y());
        }
    }
    std::printf("  %d parts shipped in %.1f s, %g placed; held %.1f mm off the TCP at worst\n", shipped,
                sim.elapsedTime(), sim.automation()->value("Cycles"), worstGrip * 1000);
    CHECK(worstGrip < 0.002f);
    CHECK(worstTilt < 1e-3f);
    CHECK(shipped == 3);
    CHECK(sim.automation()->warnings().isEmpty());
    if (!sim.automation()->warnings().isEmpty())
        std::printf("  %s\n", qPrintable(sim.automation()->warnings().join("; ")));

    // Door open: the robot finishes its move, then holds
    sim.automation()->setInput("OpenDoor", 1);
    for (int i = 0; i < 150; ++i) sim.step();
    CHECK(sim.automation()->value("DoorClosed") == 0.0);
    std::map<int, float> held;
    for (int jid : kin::chainTo(g, tcp)) held[jid] = g.getJoint(jid)->currentValue;
    for (int i = 0; i < 300; ++i) sim.step();
    for (auto& [jid, v] : held) CHECK_NEAR(g.getJoint(jid)->currentValue, v, 1e-4);
    sim.automation()->setInput("OpenDoor", 0);
    const double cycles = sim.automation()->value("Cycles");
    for (int i = 0; i < 600; ++i) sim.step();
    CHECK(sim.automation()->value("DoorClosed") == 1.0);
    CHECK(sim.automation()->value("Cycles") > cycles);
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

// Copies fall onto a belt, ride it into a sink and are taken; scrubbing back
// shows only the copies that existed then; Reset removes them
TEST(sourceBeltSinkWithPhysics) {
    SceneGraph g;
    int belt = addBox(g, "Conveyor", {0, 0.1f, 0}, {3, 0.2f, 0.6f});
    g.addConveyorBelt(belt);
    g.getConveyorBelt(belt)->velocity = 1.0f;
    g.getConveyorBelt(belt)->direction = {1, 0, 0};
    int box = addBox(g, "Box", {-1.2f, 0.35f, 0}, {0.15f, 0.15f, 0.15f});
    g.addRigidBody(box);
    int end = addBox(g, "End", {1.3f, 0.35f, 0}, {0.2f, 0.3f, 0.6f});
    g.getObject(end)->opacity = 0.3f;
    PartSource src;
    src.name = "Feeder";
    src.templateId = box;
    src.interval = "0.6";
    g.addSource(src);
    PartSink sink;
    sink.name = "Out";
    sink.zone.objectId = end;
    sink.zone.objectBounds = true;
    g.addSink(sink);
    Signal laser;
    laser.name = "Gap";
    laser.kind = SignalKind::Sensor;
    laser.sensor.type = SensorType::Distance;
    laser.sensor.offset = {0, 1, 0};
    laser.sensor.direction = {0, -1, 0};
    laser.sensor.range = 2;
    g.addSignal(laser);
    CHECK(AutomationEngine::validateAll(g).isEmpty());

    SimulationEngine sim(&g);
    sim.step();
    const int laserId = signalNamed(g, "Gap")->id;
    double nearest = 2, worstErr = 0;
    int measured = 0;
    for (int i = 0; i < 400; ++i) {                 // 6.4 s
        sim.step();
        nearest = std::min(nearest, sim.automation()->value("Gap"));
        // What the engine reads at the start of the next scan, on the pose now
        const SensorConfig& cfg = g.getSignal(laserId)->sensor;
        const sensors::Reading r = sensors::read(g, cfg);
        const double gap = r.value(cfg);
        const int hit = r.objectId;
        const SceneObject* o = g.getObject(hit);
        if (!o || o->copyOf == -1) continue;
        // The exact top face under the beam (x = z = 0) of the box as simulated
        const QMatrix4x4 w = g.worldTransform(hit);
        const QVector3D n = w.mapVector({0, 1, 0}).normalized(), top = w.map({0, 0.5f, 0});
        if (n.y() < 0.999f) continue;               // tipped: the beam may meet another face
        const QVector3D from(0, 1, 0);
        const double t = QVector3D::dotProduct(top - from, n) / QVector3D::dotProduct(QVector3D(0, -1, 0), n);
        worstErr = std::max(worstErr, std::fabs(gap - t));
        ++measured;
    }
    const double made = sim.automation()->value("Feeder.count");
    const double taken = sim.automation()->value("Out.count");
    std::printf("  %g copies made, %g taken by the sink; laser: nearest %.4f m, %d readings within %.2g m of the true top face\n",
                made, taken, nearest, measured, worstErr);
    CHECK(made >= 8);
    CHECK(taken >= made - 4 && taken >= 4);
    // The laser looks down at the belt (top 0.2) from 1 m: a box's top is about 0.65 below it
    CHECK_NEAR(nearest, 1 - (0.2 + 0.15), 5e-3);
    CHECK(measured > 10);
    CHECK(worstErr < 1e-5);
    int live = 0;
    for (auto& [id, o] : g.allObjects()) if (o.copyOf != -1 && !o.parked) ++live;
    CHECK(live == int(made - taken));
    // Copies ride the belt: an unparked one has moved along +X from the spot
    bool moved = false;
    for (auto& [id, o] : g.allObjects())
        if (o.copyOf != -1 && !o.parked && o.transform.position.x() > -0.8f) moved = true;
    CHECK(moved);

    sim.seek(0);                                    // only the first copy existed then
    int shown = 0;
    for (auto& [id, o] : g.allObjects()) if (o.copyOf != -1 && !o.parked) ++shown;
    CHECK(shown == 0 || shown == 1);
    sim.seekLive();
    int back = 0;
    for (auto& [id, o] : g.allObjects()) if (o.copyOf != -1 && !o.parked) ++back;
    CHECK(back == live);
    sim.reset();
    CHECK(g.allObjects().size() == 3 && !g.isParked(box));
    CHECK_NEAR(g.getObject(box)->transform.position.x(), -1.2, 1e-6);
}
#endif

} // namespace

// ── Assembly mates ───────────────────────────────────────────────────────────

MateFeature feat(int objectId, MateFeature::Kind kind, QVector3D point, QVector3D dir = {}) {
    MateFeature f;
    f.objectId = objectId;
    f.kind = kind;
    f.point = point;
    f.direction = dir;
    return f;
}

int addMate(SceneGraph& g, MateType type, const MateFeature& moving, const MateFeature& fixed, float offset = 0) {
    Mate m;
    m.name = assembly::nextName(g);
    m.type = type;
    m.moving = moving;
    m.fixed = fixed;
    m.offset = offset;
    m.aligned = assembly::defaultAlignment(g, m);
    return g.addMate(m);
}

TEST(mateFlushMovesLeast) {
    SceneGraph g;
    const int base = addBox(g, "Base", {0, 0, 0}, {1, 1, 1});
    const int block = addBox(g, "Block", {2, 1.3f, 0.4f}, {1, 1, 1});
    g.getObject(block)->transform.rotation = {0, 0, 10};
    using K = MateFeature::Kind;
    const int id = addMate(g, MateType::Flush, feat(block, K::Plane, {0, -0.5f, 0}, {0, -1, 0}),
                           feat(base, K::Plane, {0, 0.5f, 0}, {0, 1, 0}));
    CHECK(!g.getMate(id)->aligned);   // faces against each other
    CHECK(assembly::movingBody(g, *g.getMate(id)) == block);
    const auto r = assembly::solve(g);
    CHECK(r.ok());
    CHECK(r.moved.size() == 1 && r.moved[0] == block);
    const Transform& t = g.getObject(block)->transform;
    CHECK_NEAR(t.position.y(), 1.0, 1e-5);
    CHECK_NEAR(t.rotation.z(), 0.0, 1e-3);
    // Turned about the face centre, slid along the normal only
    CHECK_NEAR(t.position.x(), 2.0 + 0.5 * std::sin(qDegreesToRadians(10.0)), 1e-5);
    CHECK_NEAR(t.position.z(), 0.4, 1e-6);
    CHECK_NEAR(g.getObject(base)->transform.position.length(), 0.0, 1e-9);
    // Solving again moves nothing
    CHECK(assembly::solve(g).moved.empty());

    // The face is free to slide: a drag along it stays, a lift is undone
    g.getObject(block)->transform.position += QVector3D(0.7f, 0.5f, -0.2f);
    assembly::solve(g);
    CHECK_NEAR(g.getObject(block)->transform.position.y(), 1.0, 1e-5);
    CHECK_NEAR(g.getObject(block)->transform.position.z(), 0.2, 1e-5);

    // A gap, and level faces instead
    g.getMate(id)->offset = 0.1f;
    assembly::solve(g);
    CHECK_NEAR(g.getObject(block)->transform.position.y(), 1.1, 1e-5);
    g.getMate(id)->offset = 0;
    g.getMate(id)->aligned = true;
    assembly::solve(g);
    CHECK_NEAR(g.getObject(block)->transform.position.y(), 0.0, 1e-5);   // bottom faces level
    CHECK(assembly::residual(g, *g.getMate(id)).satisfied());
}

TEST(matesSolveTogether) {
    SceneGraph g;
    using K = MateFeature::Kind;
    const int base = addBox(g, "Base", {0, 0, 0}, {1, 1, 1});
    const int pin = g.addObject("Pin", PrimitiveType::Cylinder);
    g.getObject(pin)->transform.position = {1, 2, 1};
    g.getObject(pin)->transform.rotation = {30, 0, 15};
    g.getObject(pin)->transform.scale = {0.2f, 1, 0.2f};
    // Seat the pin's bottom edge on the top face, then line up the axes
    addMate(g, MateType::Flush, feat(pin, K::Circle, {0, -0.5f, 0}, {0, -1, 0}),
            feat(base, K::Plane, {0, 0.5f, 0}, {0, 1, 0}));
    assembly::solve(g);
    addMate(g, MateType::Concentric, feat(pin, K::Axis, {0, 0, 0}, {0, 1, 0}),
            feat(base, K::Axis, {0, 0, 0}, {0, 1, 0}));
    const auto r = assembly::solve(g);
    CHECK(r.ok());
    for (auto& [id, m] : g.allMates()) CHECK(assembly::residual(g, m).satisfied());
    const QMatrix4x4 w = g.worldTransform(pin);
    CHECK_NEAR(w.map(QVector3D(0, -0.5f, 0)).y(), 0.5, 1e-5);
    CHECK_NEAR(w.map(QVector3D(0, 0, 0)).x(), 0.0, 1e-5);
    CHECK_NEAR(w.map(QVector3D(0, 0, 0)).z(), 0.0, 1e-5);
    CHECK_NEAR(w.mapVector(QVector3D(0, 1, 0)).normalized().y(), 1.0, 1e-5);

    // Two flush mates on parallel faces at different heights can't both hold
    const int shelf = addBox(g, "Shelf", {3, 0.2f, 0}, {1, 1, 1});
    const int bad = addMate(g, MateType::Flush, feat(pin, K::Circle, {0, -0.5f, 0}, {0, -1, 0}),
                            feat(shelf, K::Plane, {0, 0.5f, 0}, {0, 1, 0}));
    const auto r2 = assembly::solve(g);
    CHECK(!r2.ok());
    CHECK(!r2.problems.isEmpty());
    g.removeMate(bad);
    CHECK(assembly::solve(g).ok());
}

TEST(mateInsertAndCoincident) {
    SceneGraph g;
    using K = MateFeature::Kind;
    const int plate = addBox(g, "Plate", {0, 0, 0}, {1, 0.1f, 1});
    const int bolt = g.addObject("Bolt", PrimitiveType::Cylinder);
    g.getObject(bolt)->transform.position = {0.4f, 1, -0.3f};
    g.getObject(bolt)->transform.rotation = {80, 0, 0};
    // The bolt's bottom rim into the hole's top rim (axis = plate normal), 5 mm proud
    const int id = addMate(g, MateType::Insert, feat(bolt, K::Circle, {0, -0.5f, 0}, {0, -1, 0}),
                           feat(plate, K::Circle, {0.2f, 0.5f, 0.1f}, {0, 1, 0}), 0.005f);
    CHECK(assembly::typesFor(K::Circle, K::Circle).front() == MateType::Insert);
    CHECK(assembly::solve(g).ok());
    const QVector3D rim = g.worldTransform(bolt).map(QVector3D(0, -0.5f, 0));
    CHECK_NEAR(rim.x(), 0.2, 1e-5);
    CHECK_NEAR(rim.y(), 0.055, 1e-5);
    CHECK(!g.getMate(id)->aligned);   // rims against each other: the bolt stands up out of the hole
    CHECK_NEAR(g.worldTransform(bolt).mapVector(QVector3D(0, 1, 0)).normalized().y(), 1.0, 1e-5);
    CHECK_NEAR(rim.z(), 0.1, 1e-5);

    const int cube = addBox(g, "Cube", {5, 5, 5}, {0.1f, 0.1f, 0.1f});
    addMate(g, MateType::Coincident, feat(cube, K::Point, {0.5f, 0.5f, 0.5f}),
            feat(plate, K::Point, {-0.5f, 0.5f, -0.5f}));
    CHECK(assembly::solve(g).ok());
    const QVector3D corner = g.worldTransform(cube).map(QVector3D(0.5f, 0.5f, 0.5f));
    CHECK_NEAR((corner - QVector3D(-0.5f, 0.05f, -0.5f)).length(), 0.0, 1e-5);
    CHECK_NEAR(g.getObject(cube)->transform.rotation.length(), 0.0, 1e-6);   // points don't turn it
}

TEST(mateMovesTheRightBranch) {
    SceneGraph g;
    using K = MateFeature::Kind;
    const int asmA = g.addObject("AssemblyA", PrimitiveType::ImportedMesh);
    const int partA = addBox(g, "PartA", {0, 0, 0}, {1, 1, 1}, asmA);
    const int partA2 = addBox(g, "PartA2", {2, 0, 0}, {1, 1, 1}, asmA);
    const int asmB = g.addObject("AssemblyB", PrimitiveType::ImportedMesh);
    const int sub = g.addObject("Sub", PrimitiveType::ImportedMesh, asmB);
    const int partB = addBox(g, "PartB", {0, 3, 0}, {1, 1, 1}, sub);

    Mate m;
    m.type = MateType::Flush;
    m.moving = feat(partB, K::Plane, {0, -0.5f, 0}, {0, -1, 0});
    m.fixed = feat(partA, K::Plane, {0, 0.5f, 0}, {0, 1, 0});
    CHECK(assembly::movingBody(g, m) == asmB);       // no shared ancestor: the whole tree
    m.fixed = feat(partA, K::Plane, {0, 0.5f, 0}, {0, 1, 0});
    m.moving = feat(partA2, K::Plane, {0, -0.5f, 0}, {0, -1, 0});
    CHECK(assembly::movingBody(g, m) == partA2);     // inside one assembly: just the part
    m.moving = feat(asmA, K::Plane, {0, 0, 0}, {0, 1, 0});
    QString why;
    CHECK(assembly::movingBody(g, m, &why) == -1);   // holds the fixed part
    CHECK(!why.isEmpty());
    m.moving = feat(partA, K::Plane, {0, 0, 0}, {0, 1, 0});
    CHECK(assembly::movingBody(g, m) == -1);          // same part
    (void)sub;
}

TEST(matesSaveUndoAndFollowEdits) {
    SceneGraph g;
    using K = MateFeature::Kind;
    const int base = addBox(g, "Base", {0, 0, 0}, {1, 1, 1});
    const int grp = g.addObject("Group", PrimitiveType::ImportedMesh);
    const int a = addBox(g, "A", {0, 2, 0}, {1, 1, 1}, grp);
    const int b = addBox(g, "B", {3, 2, 0}, {1, 1, 1}, grp);
    addMate(g, MateType::Flush, feat(a, K::Plane, {0, -0.5f, 0}, {0, -1, 0}), feat(base, K::Plane, {0, 0.5f, 0}, {0, 1, 0}), 0.25f);
    addMate(g, MateType::Flush, feat(b, K::Plane, {0, 0.5f, 0}, {0, 1, 0}), feat(a, K::Plane, {0, 0.5f, 0}, {0, 1, 0}));
    g.getMate(g.allMates().rbegin()->first)->enabled = false;
    CHECK(assembly::solve(g).ok());

    SceneGraph h;
    roundTrip(g, h);
    CHECK(h.allMates().size() == 2);
    const Mate& m1 = h.allMates().begin()->second;
    CHECK(m1.type == MateType::Flush && m1.name == "Mate1");
    CHECK_NEAR(m1.offset, 0.25, 1e-6);
    CHECK(h.getObject(m1.moving.objectId)->name == "A");
    CHECK(!h.allMates().rbegin()->second.enabled);
    CHECK(assembly::solve(h).moved.empty());   // loaded already solved
    // load → save is stable
    SceneGraph h2;
    roundTrip(h, h2);
    CHECK(QJsonDocument(ProjectSerializer::toJson(h)).toJson() == QJsonDocument(ProjectSerializer::toJson(h2)).toJson());

    // Moving the fixed part brings the mated one along
    g.getObject(base)->transform.position.setY(-1);
    assembly::solve(g);
    CHECK_NEAR(g.worldTransform(grp).map(QVector3D()).y() + g.getObject(a)->transform.position.y(), 0.25, 1e-5);

    // Duplicating the group copies the mate between its parts; removing a part drops its mates
    g.getMate(g.allMates().rbegin()->first)->enabled = true;
    const size_t before = g.allMates().size();
    g.duplicateObject(grp);
    CHECK(g.allMates().size() == before + 1);
    g.removeObject(a);
    CHECK(g.allMates().size() == 1);   // only the copy's internal mate is left

    // Undo brings a removed mate back
    SceneGraph u;
    UndoManager undo(&u);
    const int x = addBox(u, "X", {0, 0, 0}, {1, 1, 1});
    const int y = addBox(u, "Y", {0, 3, 0}, {1, 1, 1});
    undo.reset();
    const int mid = addMate(u, MateType::Flush, feat(y, K::Plane, {0, -0.5f, 0}, {0, -1, 0}), feat(x, K::Plane, {0, 0.5f, 0}, {0, 1, 0}));
    QCoreApplication::processEvents();
    QElapsedTimer wait; wait.start();
    while (!undo.canUndo() && wait.elapsed() < 2000) QCoreApplication::processEvents();
    CHECK(undo.canUndo());
    undo.undo();
    CHECK(u.allMates().empty());
    undo.redo();
    CHECK(u.allMates().size() == 1);
    (void)mid;
}

TEST(mateMergeRemapsAndRenames) {
    SceneGraph target, source;
    using K = MateFeature::Kind;
    const int t1 = addBox(target, "T1", {0, 0, 0}, {1, 1, 1});
    const int t2 = addBox(target, "T2", {0, 3, 0}, {1, 1, 1});
    addMate(target, MateType::Flush, feat(t2, K::Plane, {0, -0.5f, 0}, {0, -1, 0}), feat(t1, K::Plane, {0, 0.5f, 0}, {0, 1, 0}));
    const int root = source.addObject("Comp", PrimitiveType::ImportedMesh);
    const int s1 = addBox(source, "S1", {0, 0, 0}, {1, 1, 1}, root);
    const int s2 = addBox(source, "S2", {1, 0, 0}, {1, 1, 1}, root);
    addMate(source, MateType::Coincident, feat(s2, K::Point, {}), feat(s1, K::Point, {0.5f, 0, 0}));
    mergeScene(target, source);
    CHECK(target.allMates().size() == 2);
    const Mate& copy = target.allMates().rbegin()->second;
    CHECK(copy.name == "Mate2");
    CHECK(target.getObject(copy.moving.objectId)->name == "S2");
    CHECK(target.getObject(copy.fixed.objectId)->name == "S1");
}

TEST(mateFeatureFromWorldWithScale) {
    SceneGraph g;
    const int o = addBox(g, "Scaled", {1, 2, 3}, {2, 0.5f, 1});
    g.getObject(o)->transform.rotation = {0, 30, 0};
    // A slanted face normal must come back the same under non-uniform scale
    const QVector3D n = QVector3D(1, 1, 0).normalized();
    const QVector3D p(1.5f, 2.2f, 3.1f);
    const MateFeature f = assembly::fromWorld(g, o, MateFeature::Kind::Plane, p, n);
    const auto w = assembly::toWorld(g, f);
    CHECK_NEAR((w.point - p).length(), 0.0, 1e-5);
    CHECK_NEAR((w.direction - n).length(), 0.0, 1e-5);
    const MateFeature ax = assembly::fromWorld(g, o, MateFeature::Kind::Axis, p, n);
    CHECK_NEAR((assembly::toWorld(g, ax).direction - n).length(), 0.0, 1e-5);
}

TEST(stepImportEstimator) {
    using StepImporter::Progress;
    StepImporter::Estimator::Rates rates;
    rates.readSecPerMB = 0.25;
    rates.translateRatio = 2.5;
    rates.meshRatio = 1.0;
    StepImporter::Estimator est(4 * 1024 * 1024, rates);   // reading guessed at 1 s
    Progress p;
    est.update(p, 0.5);
    CHECK(!est.confident());                       // nothing measured in the first second
    CHECK_NEAR(est.fraction(), 0.5 / 4.5, 1e-6);   // read 1 + translate 2.5 + mesh 1
    est.update(p, 1.5);                            // reading runs past its guess
    CHECK(!est.confident());
    p.stage = Progress::Stage::Translating;
    p.stageFraction = 0.08;                        // OCCT's figure: ignored, it's too uneven
    est.update(p, 2.0);                            // read took 2 s → translate 5, mesh 2
    CHECK(est.confident());
    CHECK_NEAR(est.secondsLeft(), 7.0, 1e-6);
    const double before = est.fraction();
    p.stage = Progress::Stage::Meshing;
    p.stageFraction = 0.5;
    est.update(p, 7.0);                            // translate took 5 s; meshing starts
    CHECK_NEAR(est.secondsLeft(), 2.0, 1e-6);      // just started: the guess (2× reading)
    est.update(p, 8.0);                            // half done after 1 s → 1 s left
    CHECK(est.fraction() >= before);
    CHECK_NEAR(est.secondsLeft(), 1.0, 1e-6);
    CHECK(est.confident());
    est.update(p, 11.0);                           // stuck at 50% for 3 s (one big part)
    CHECK(!est.confident());
    CHECK(est.secondsLeft() >= 3.0);               // expects at least as long as it's been stuck
    CHECK(est.fraction() <= 0.99);
    p.stageFraction = 1.0;
    est.update(p, 12.0);
    const auto learned = est.learn(12.0);
    // Half the old rate, half this import's (read 0.5 s/MB, translate 2.5×, mesh 2.5×)
    CHECK_NEAR(learned.readSecPerMB, 0.375, 1e-6);
    CHECK_NEAR(learned.translateRatio, 2.5, 1e-6);
    CHECK_NEAR(learned.meshRatio, 1.75, 1e-6);

    CHECK(StepImporter::Estimator(0, rates).fraction() == 0.0);
}

#ifdef OCCT_ENABLED
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Trsf.hxx>
#include <Interface_Static.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <Message_PrinterOStream.hxx>

// An assembly: one 100 × 50 × 20 mm block used twice, and a pin
static QString writeTestStep(const QString& dir) {
    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
    Handle(TDocStd_Document) doc;
    app->NewDocument("MDTV-XCAF", doc);
    Handle(XCAFDoc_ShapeTool) tool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    const TDF_Label block = tool->AddShape(BRepPrimAPI_MakeBox(100, 50, 20).Shape(), false);
    TDataStd_Name::Set(block, "Block");
    const TDF_Label pin = tool->AddShape(BRepPrimAPI_MakeCylinder(5, 40).Shape(), false);
    TDataStd_Name::Set(pin, "Pin");
    const TDF_Label assy = tool->NewShape();
    TDataStd_Name::Set(assy, "Fixture");
    gp_Trsf up; up.SetTranslation(gp_Vec(0, 0, 20));
    gp_Trsf right; right.SetTranslation(gp_Vec(200, 0, 0));
    tool->AddComponent(assy, block, TopLoc_Location());
    tool->AddComponent(assy, block, TopLoc_Location(right));
    tool->AddComponent(assy, pin, TopLoc_Location(up));
    tool->UpdateAssemblies();
    // The writer prints transfer statistics; keep the test output clean
    Message::DefaultMessenger()->RemovePrinters(STANDARD_TYPE(Message_PrinterOStream));
    STEPCAFControl_Writer writer;
    Interface_Static::SetCVal("xstep.cascade.unit", "MM");   // the shapes are in mm (an import sets M)
    const QString path = dir + "/fixture.step";
    writer.Transfer(doc, STEPControl_AsIs);
    writer.Write(path.toUtf8().constData());
    app->Close(doc);
    return path;
}

TEST(stepImportReportsProgressAndCancels) {
    QTemporaryDir dir;
    const QString path = writeTestStep(dir.path());
    using StepImporter::Progress;

    std::vector<Progress> seen;
    StepImporter::Part root;
    QString error;
    const bool ok = StepImporter::importFile(path, root, error, [&](const Progress& p) {
        seen.push_back(p);
        return true;
    });
    CHECK(ok);
    CHECK(error.isEmpty());
    CHECK(root.name == "Fixture");
    CHECK(root.children.size() == 3);
    if (root.children.size() == 3) {
        // The two blocks share one mesh; geometry is in metres
        CHECK(root.children[0].mesh && root.children[0].mesh == root.children[1].mesh);
        CHECK(root.children[2].mesh && root.children[2].mesh != root.children[0].mesh);
        if (root.children[0].mesh) {
            const QVector3D size = root.children[0].mesh->aabbMax - root.children[0].mesh->aabbMin;
            CHECK_NEAR(size.x(), 0.1, 1e-4);
            CHECK_NEAR(size.z(), 0.02, 1e-4);
        }
        CHECK_NEAR(root.children[1].position.x(), 0.2, 1e-5);
    }
    // Stages arrive in order and meshing ends complete
    CHECK(!seen.empty());
    bool ordered = true;
    for (size_t i = 1; i < seen.size(); i++)
        if (int(seen[i].stage) < int(seen[i - 1].stage)) ordered = false;
    CHECK(ordered);
    if (!seen.empty()) {
        CHECK(seen.front().stage == Progress::Stage::Reading);
        CHECK(seen.back().stage == Progress::Stage::Meshing);
        CHECK(seen.back().total == 2);              // two distinct parts
        CHECK(seen.back().done == 2);
        CHECK_NEAR(seen.back().stageFraction, 1.0, 1e-9);
    }

    // Cancelling while reading stops at the next point the import can stop
    int calls = 0;
    StepImporter::Part none;
    const bool cancelled = StepImporter::importFile(path, none, error, [&](const Progress&) {
        ++calls;
        return false;
    });
    CHECK(!cancelled);
    CHECK(error == StepImporter::cancelledMessage());
    CHECK(none.children.empty() && !none.mesh);
    CHECK(calls <= 2);

    // Cancelling during meshing too
    const bool late = StepImporter::importFile(path, none, error, [&](const Progress& p) {
        return p.stage != Progress::Stage::Meshing;
    });
    CHECK(!late);
    CHECK(error == StepImporter::cancelledMessage());
}

// A grille: a 200 × 100 × 5 mm plate with 8 × 10 round holes of r 4 mm,
// so its faces have more holes than the default triangulator handles well
TEST(stepImportMeshesFacesWithManyHoles) {
    TopoDS_Wire outer = BRepBuilderAPI_MakePolygon(gp_Pnt(0, 0, 0), gp_Pnt(200, 0, 0),
                                                   gp_Pnt(200, 100, 0), gp_Pnt(0, 100, 0), true).Wire();
    BRepBuilderAPI_MakeFace face(gp_Pln(gp::XOY()), outer);
    for (int i = 0; i < 10; i++)
        for (int j = 0; j < 8; j++) {
            gp_Circ c(gp_Ax2(gp_Pnt(15 + i * 19, 15 + j * 10, 0), gp::DZ()), 4);
            TopoDS_Wire hole = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(c).Edge()).Wire();
            hole.Reverse();
            face.Add(hole);
        }
    const TopoDS_Shape plate = BRepPrimAPI_MakePrism(face.Face(), gp_Vec(0, 0, 5)).Shape();

    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
    Handle(TDocStd_Document) doc;
    app->NewDocument("MDTV-XCAF", doc);
    Handle(XCAFDoc_ShapeTool) tool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    TDataStd_Name::Set(tool->AddShape(plate, false), "Grille");
    QTemporaryDir dir;
    const QString path = dir.path() + "/grille.step";
    Message::DefaultMessenger()->RemovePrinters(STANDARD_TYPE(Message_PrinterOStream));
    STEPCAFControl_Writer writer;
    Interface_Static::SetCVal("xstep.cascade.unit", "MM");   // the shapes are in mm (an import sets M)
    writer.Transfer(doc, STEPControl_AsIs);
    writer.Write(path.toUtf8().constData());
    app->Close(doc);

    StepImporter::Part root;
    QString error;
    CHECK(StepImporter::importFile(path, root, error));
    CHECK(root.mesh != nullptr);
    if (!root.mesh) return;
    // Every face meshed and the holes left open: the triangles' area is the
    // plate's surface (two faces with holes, the outer sides, the hole walls)
    const auto& v = root.mesh->interleaved;
    double area = 0;
    for (size_t t = 0; t + 18 <= v.size(); t += 18) {
        const QVector3D a(v[t], v[t + 1], v[t + 2]), b(v[t + 6], v[t + 7], v[t + 8]),
                        c(v[t + 12], v[t + 13], v[t + 14]);
        area += 0.5 * QVector3D::crossProduct(b - a, c - a).length();
    }
    const double holes = 80 * M_PI * 0.004 * 0.004;
    const double expected = 2 * (0.2 * 0.1 - holes) + 2 * (0.2 + 0.1) * 0.005
                          + 80 * 2 * M_PI * 0.004 * 0.005;
    CHECK_NEAR(area, expected, expected * 0.02);
}
#endif

// ── Robot builder and analyser ──────────────────────────────────────────────

namespace {
double tcpLength(const SceneGraph& g, const robot::Built& b) { return -g.getObject(b.tcp)->transform.position.y(); }
void setArm(SceneGraph& g, const robot::Built& b, std::vector<float> v) {
    for (size_t i = 0; i < v.size(); ++i) g.getJoint(b.joints[i])->currentValue = v[i];
    g.applyCouplings();
}
bool hasWarning(const robot::Analysis& a, const char* text) {
    for (const QString& w : a.warnings) if (w.contains(text)) return true;
    return false;
}
} // namespace

// ── Motions commanded from outside (API) ─────────────────────────────────────

namespace {
// Runs scans until the motion is finished (or `limit` seconds pass)
AutomationEngine::MotionStatus runMotion(AutomationEngine& e, int id, double limit = 30,
                                         const std::function<void()>& each = {}) {
    for (int i = 0; i < int(limit / 0.016) && !e.motionStatus(id).finished(); ++i) {
        e.scan(0.016);
        if (each) each();
    }
    return e.motionStatus(id);
}
} // namespace

TEST(apiMotionsMoveJointsAndLines) {
    robot::Spec s = robot::defaultSpec(robot::Kind::Articulated);
    SceneGraph g;
    const robot::Built b = robot::build(g, s);
    AutomationEngine e(&g);
    using MK = AutomationEngine::MotionKind;
    using MS = AutomationEngine::MotionState;
    QString code, err;
    {   // Not running yet
        AutomationEngine::MotionRequest r;
        r.joints[b.joints[0]] = 10;
        CHECK(e.submitMotion(r, &code, &err) == 0 && code == "not_running");
    }
    CHECK(e.start(nullptr));

    // Joint move: both joints arrive together
    AutomationEngine::MotionRequest jm;
    jm.joints[b.joints[0]] = 30;
    jm.joints[b.joints[1]] = -10;
    jm.speed = 50;
    const int m1 = e.submitMotion(jm, &code, &err);
    CHECK(m1 > 0);
    CHECK(e.motionStatus(m1).state == MS::Queued);
    bool together = true;
    const auto st1 = runMotion(e, m1, 30, [&] {
        const double p0 = std::fabs(g.getJoint(b.joints[0])->currentValue) / 30.0;
        const double p1 = std::fabs(g.getJoint(b.joints[1])->currentValue) / 10.0;
        if (std::fabs(p0 - p1) > 0.02) together = false;
    });
    CHECK(st1.state == MS::Done && st1.progress == 1);
    CHECK(together);
    CHECK_NEAR(g.getJoint(b.joints[0])->currentValue, 30, 1e-3);
    CHECK_NEAR(g.getJoint(b.joints[1])->currentValue, -10, 1e-3);

    // Out of limits is refused at once
    AutomationEngine::MotionRequest bad;
    bad.joints[b.joints[1]] = 1000;
    CHECK(e.submitMotion(bad, &code, &err) == 0 && code == "out_of_limits");
    CHECK(err.contains("A2"));

    // Straight line 100 mm down in the robot's frame, keeping the orientation
    const QMatrix4x4 start = kin::toolFrame(g, b.tcp);
    AutomationEngine::MotionRequest lm;
    lm.kind = MK::Linear;
    lm.tool = b.tcp;
    lm.goal.frameId = b.root;
    lm.goal.relative = true;
    lm.goal.y = -0.1;
    lm.speed = 0.2;
    const int m2 = e.submitMotion(lm, &code, &err);
    CHECK(m2 > 0);
    double offLine = 0;
    const auto st2 = runMotion(e, m2, 30, [&] {
        const QVector3D d = (kin::toolFrame(g, b.tcp).column(3) - start.column(3)).toVector3D();
        offLine = std::max(offLine, double(std::hypot(d.x(), d.z())));
    });
    CHECK(st2.state == MS::Done);
    const QMatrix4x4 end = kin::toolFrame(g, b.tcp);
    CHECK_NEAR(end(1, 3), start(1, 3) - 0.1, 1e-4);
    CHECK_NEAR(end(0, 3), start(0, 3), 1e-4);
    CHECK(offLine < 1e-3);
    for (int c = 0; c < 3; ++c) CHECK_NEAR(end.column(c).toVector3D().distanceToPoint(start.column(c).toVector3D()), 0, 1e-3);

    // Absolute goal with only x given keeps y, z and the orientation
    AutomationEngine::MotionRequest ax = lm;
    ax.goal = {};
    ax.goal.frameId = b.root;
    ax.goal.x = double(end(0, 3)) - 0.05;
    const int m3 = e.submitMotion(ax);
    CHECK(runMotion(e, m3).state == MS::Done);
    CHECK_NEAR(kin::toolFrame(g, b.tcp)(0, 3), end(0, 3) - 0.05, 1e-4);
    CHECK_NEAR(kin::toolFrame(g, b.tcp)(1, 3), end(1, 3), 1e-4);

    // Motions sharing joints queue in order; the tool frame comes back
    const QMatrix4x4 before = kin::toolFrame(g, b.tcp);
    AutomationEngine::MotionRequest up = lm, down = lm;
    up.goal.y = 0.05;
    down.goal.y = -0.05;
    down.kind = MK::JointsToPose;
    down.speed = 100;
    const int q1 = e.submitMotion(up), q2 = e.submitMotion(down);
    e.scan(0.016);
    CHECK(e.motionStatus(q1).state == MS::Running && e.motionStatus(q2).state == MS::Queued);
    CHECK(runMotion(e, q2).state == MS::Done);
    CHECK(e.motionStatus(q1).state == MS::Done);
    CHECK(kin::toolFrame(g, b.tcp).column(3).toVector3D().distanceToPoint(before.column(3).toVector3D()) < 2e-4);

    // Out of reach fails when it starts, and cancels the queued motion behind it
    AutomationEngine::MotionRequest far = lm;
    far.goal.y = 5.0;
    const int f1 = e.submitMotion(far), f2 = e.submitMotion(up);
    e.scan(0.016);
    CHECK(e.motionStatus(f1).state == MS::Failed && e.motionStatus(f1).code == "out_of_reach");
    CHECK(e.motionStatus(f2).state == MS::Cancelled);
    CHECK(e.motionStatus(f2).error.contains("earlier"));

    // A 4-joint chain refuses an orientation
    AutomationEngine::MotionRequest rot = lm;
    rot.chain = {b.joints[0], b.joints[1], b.joints[2], b.joints[3]};
    rot.goal.rx = 10;
    CHECK(e.submitMotion(rot, &code, &err) == 0 && code == "bad_request");

    // Cancel stops a running motion where it is; reset cancels the rest
    AutomationEngine::MotionRequest swing;
    swing.joints[b.joints[0]] = -60;
    const int c1 = e.submitMotion(swing);
    for (int i = 0; i < 10; ++i) e.scan(0.016);
    CHECK(e.motionStatus(c1).state == MS::Running);
    e.cancelMotion(c1);
    CHECK(e.motionStatus(c1).state == MS::Cancelled);
    for (int i = 0; i < 20; ++i) e.scan(0.016);
    const float held = g.getJoint(b.joints[0])->currentValue;
    e.scan(0.016);
    CHECK(held > -60 && held < 30);
    CHECK_NEAR(g.getJoint(b.joints[0])->currentValue, held, 1e-6);
    const int r1 = e.submitMotion(up);
    e.stop();
    CHECK(e.motionStatus(r1).state == MS::Cancelled && e.motionStatus(r1).code == "reset");
    CHECK(e.motionStatus(9999).code == "not_found");
}

namespace {
// A line-based JSON client for the API server test: sends a request and
// processes events until the reply with its id arrives
struct ApiClient {
    QTcpSocket sock;
    QByteArray buf;
    int next = 1;
    bool connectTo(quint16 port) {
        sock.connectToHost(QHostAddress::LocalHost, port);
        QElapsedTimer t;
        t.start();
        while (sock.state() != QAbstractSocket::ConnectedState && t.elapsed() < 2000)
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        return sock.state() == QAbstractSocket::ConnectedState;
    }
    int send(const QString& method, const QJsonObject& params = {}) {
        const int id = next++;
        sock.write(QJsonDocument(QJsonObject{{"id", id}, {"method", method}, {"params", params}}).toJson(QJsonDocument::Compact) + '\n');
        sock.flush();
        return id;
    }
    QJsonObject reply(int id, int timeoutMs = 20000) {
        QElapsedTimer t;
        t.start();
        while (t.elapsed() < timeoutMs) {
            int nl;
            while ((nl = buf.indexOf('\n')) >= 0) {
                const QJsonObject o = QJsonDocument::fromJson(buf.left(nl)).object();
                buf.remove(0, nl + 1);
                if (o.value("id").toInt() == id) return o;
                pending.push_back(o);
            }
            for (size_t i = 0; i < pending.size(); ++i)
                if (pending[i].value("id").toInt() == id) { QJsonObject o = pending[i]; pending.erase(pending.begin() + long(i)); return o; }
            QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
            buf += sock.readAll();
        }
        return {};
    }
    QJsonObject call(const QString& method, const QJsonObject& params = {}) { return reply(send(method, params)); }
    std::vector<QJsonObject> pending;
};
QString errorCode(const QJsonObject& r) { return r.value("error").toObject().value("code").toString(); }
QJsonObject result(const QJsonObject& r) { return r.value("result").toObject(); }
} // namespace

TEST(apiMotionsLeaveAStretchedArm) {
    // A humanoid's arm hangs straight at home, the elbow at its limit: a
    // singular pose that inverse kinematics has to nudge out of
    SceneGraph g;
    const humanoid::Built h = humanoid::build(g, humanoid::defaultSpec());
    AutomationEngine e(&g);
    CHECK(e.start(nullptr));
    const QMatrix4x4 start = kin::toolFrame(g, h.rightHand);
    AutomationEngine::MotionRequest up;
    up.kind = AutomationEngine::MotionKind::Linear;
    up.tool = h.rightHand;
    up.goal.frameId = h.root;
    up.goal.relative = true;
    up.goal.y = 0.05;
    const int m = e.submitMotion(up);
    const auto st = runMotion(e, m);
    CHECK(st.state == AutomationEngine::MotionState::Done);
    if (!st.error.isEmpty()) std::printf("  %s\n", qPrintable(st.error));
    CHECK_NEAR(kin::toolFrame(g, h.rightHand)(1, 3), start(1, 3) + 0.05, 1e-4);
    CHECK_NEAR(kin::toolFrame(g, h.rightHand)(0, 3), start(0, 3), 1e-4);
}

TEST(apiServerFindsRobots) {
    SceneGraph g;
    const robot::Built a = robot::build(g, robot::defaultSpec(robot::Kind::Articulated));
    const robot::Built b = robot::build(g, robot::defaultSpec(robot::Kind::Scara));
    humanoid::Spec hs = humanoid::defaultSpec();
    const humanoid::Built h = humanoid::build(g, hs);
    // A cell holding a turntable next to the first robot
    const int cell = g.addObject("Cell", PrimitiveType::ImportedMesh);
    g.setParent(a.root, cell);
    const int table = addBox(g, "Table", {1, 0, 0}, {1, .1f, 1}, cell);
    Joint tj;
    tj.parentBodyId = cell;
    tj.childBodyId = table;
    tj.pivot = {1, 0, 0};
    tj.continuous = true;
    g.addActuatorForJoint(g.addJoint(tj));

    const auto robots = ApiServer::findRobots(g);
    CHECK(robots.size() == 4);
    std::map<QString, ApiServer::Robot> byPath;
    for (const auto& r : robots) byPath[r.path] = r;
    const QString ap = ApiServer::pathOf(g, a.root), bp = ApiServer::pathOf(g, b.root);
    CHECK(ap.startsWith("Cell/"));
    CHECK(byPath.count(ap) && byPath[ap].root == a.root && byPath[ap].tools == std::vector<int>{a.tcp});
    CHECK(byPath.count(bp) && byPath[bp].tools == std::vector<int>{b.tcp});
    // The humanoid is one machine with both hands as tools
    const auto& hr = byPath[ApiServer::pathOf(g, h.root)];
    CHECK(hr.root == h.root);
    CHECK(hr.joints.size() == h.motors.size());
    CHECK(hr.tools.size() == 2);
    CHECK(byPath.count("Cell/Table") && byPath["Cell/Table"].joints.size() == 1);
    CHECK(byPath["Cell/Table"].tools.empty());   // not the robot's TCP next to it
}

TEST(apiServerAnswersOverTcp) {
    SceneGraph g;
    const robot::Built b = robot::build(g, robot::defaultSpec(robot::Kind::Articulated));
    const humanoid::Built h = humanoid::build(g, humanoid::defaultSpec());
    Signal start;
    start.name = "Start"; start.kind = SignalKind::Input; start.type = SignalType::Bool;
    g.addSignal(start);
    Signal echo;
    echo.name = "Started"; echo.kind = SignalKind::Derived; echo.type = SignalType::Bool; echo.expression = "Start";
    g.addSignal(echo);
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    const QString rname = g.getObject(b.root)->name, hname = g.getObject(h.root)->name;

    SimulationEngine sim(&g);
    sim.setSpeedFactor(20);
    ApiServer server(&g, &sim);
    CHECK(server.start(0));
    ApiClient c;
    CHECK(c.connectTo(server.port()));

    QJsonObject r = c.call("hello");
    CHECK(result(r).value("protocol").toInt() == ApiServer::kProtocol);
    CHECK(result(r).value("units").toObject().value("length").toString() == "mm");
    CHECK(errorCode(c.call("no.such")) == "unknown_method");
    CHECK(c.call("scene.robots").value("result").toArray().size() == 2);

    // Joint and tool queries work before the simulation runs
    r = c.call("robot.info", {{"robot", rname}});
    CHECK(result(r).value("joints").toArray().size() >= 6);
    CHECK(result(r).value("tools").toArray().first().toObject().value("name").toString() == "TCP");
    const QJsonObject p0 = result(c.call("tool.pose", {{"robot", rname}}));
    CHECK_NEAR(p0.value("x").toDouble(), worldPos(g, b.tcp).x() * 1000, 1e-2);
    CHECK(!sim.isActive());
    // The humanoid has two hands: the tool must be named
    CHECK(errorCode(c.call("tool.pose", {{"robot", hname}})) == "ambiguous");
    CHECK(c.call("tool.pose", {{"robot", hname}, {"tool", "Right Hand TCP"}}).contains("result"));

    // A straight line 100 mm up starts the simulation and answers when done
    r = c.call("tool.moveL", {{"robot", rname}, {"target", QJsonObject{{"y", 100}}}, {"relative", true}, {"speed", 400}});
    CHECK(result(r).value("state").toString() == "done");
    CHECK(sim.isRunning());
    const QJsonObject p1 = result(c.call("tool.pose", {{"robot", rname}}));
    CHECK_NEAR(p1.value("y").toDouble(), p0.value("y").toDouble() + 100, 0.05);
    CHECK_NEAR(p1.value("x").toDouble(), p0.value("x").toDouble(), 0.05);

    // Without waiting: the motion's id, then motion.wait; joints in degrees
    r = c.call("robot.moveJoints", {{"robot", rname}, {"joints", QJsonObject{{"A1", 20}}}, {"wait", false}});
    const int motion = result(r).value("motion").toInt();
    CHECK(motion > 0);
    CHECK(result(c.call("motion.wait", {{"motion", motion}})).value("state").toString() == "done");
    CHECK_NEAR(result(c.call("robot.joints", {{"robot", rname}})).value("A1").toDouble(), 20, 1e-3);

    // Errors carry a code and a message
    r = c.call("robot.moveJoints", {{"robot", rname}, {"joints", QJsonObject{{"A2", 999}}}});
    CHECK(errorCode(r) == "out_of_limits");
    CHECK(errorCode(c.call("tool.moveL", {{"robot", rname}, {"target", QJsonObject{{"x", 9000}}}})) == "out_of_reach");
    CHECK(errorCode(c.call("robot.info", {{"robot", "Nobody"}})) == "not_found");
    CHECK(errorCode(c.call("tool.moveL", {{"robot", rname}, {"target", QJsonObject{{"q", 1}}}})) == "bad_request");

    // Signals: set an input, wait for a condition, read the derived signal
    CHECK(c.call("signal.set", {{"name", "Start"}, {"value", true}}).contains("result"));
    CHECK(c.call("signal.waitUntil", {{"condition", "Started"}, {"timeout", 5}}).contains("result"));
    CHECK(result(c.call("signal.get", {{"name", "Started"}})).value("value").toDouble() == 1);
    CHECK(errorCode(c.call("signal.set", {{"name", "Started"}, {"value", 1}})) == "bad_request");
    CHECK(errorCode(c.call("signal.waitUntil", {{"condition", "Start == 0"}, {"timeout", 0.2}})) == "timeout");
    const double t0 = result(c.call("sim.state")).value("time").toDouble();
    CHECK(result(c.call("sim.wait", {{"seconds", 0.5}})).value("time").toDouble() >= t0 + 0.5 - 1e-6);

    // Reset answers what still waits
    const int moving = result(c.call("robot.moveJoints", {{"robot", rname}, {"joints", QJsonObject{{"A1", -150}}},
                                                          {"speed", 5}, {"wait", false}})).value("motion").toInt();
    const int waitId = c.send("motion.wait", {{"motion", moving}});
    c.call("sim.reset");
    CHECK(errorCode(c.reply(waitId)) == "reset");
    CHECK(!sim.isActive());
}

TEST(apiServerActuatorValueInMillimetres) {
    SceneGraph g;
    const int frame = g.addObject("Frame", PrimitiveType::ImportedMesh);
    const int slide = g.addObject("Slide", PrimitiveType::ImportedMesh, frame);
    Joint j;
    j.type = JointType::Prismatic;
    j.axis = {0, -1, 0};
    j.parentBodyId = frame;
    j.childBodyId = slide;
    j.limitMin = 0;
    j.limitMax = 0.4;
    const int jid = g.addJoint(j);
    g.getMotor(g.addActuatorForJoint(jid))->name = "Press";

    SimulationEngine sim(&g);
    sim.setSpeedFactor(20);
    ApiServer server(&g, &sim);
    CHECK(server.start(0));
    ApiClient c;
    CHECK(c.connectTo(server.port()));
    CHECK(c.call("robot.moveJoints", {{"joints", QJsonObject{{"Press", 250}}}}).contains("result"));
    // actuator.get answers in API units; the expression symbol stays in metres
    const QJsonObject a = result(c.call("actuator.get", {{"name", "Press"}}));
    CHECK(a.value("unit").toString() == "mm");
    CHECK_NEAR(a.value("value").toDouble(), 250, 1e-3);
    CHECK_NEAR(result(c.call("signal.get", {{"name", "Press.pos"}})).value("value").toDouble(), 0.25, 1e-6);
    CHECK(errorCode(c.call("actuator.get", {{"name", "Nobody"}})) == "not_found");
}

humanoid::Built walkingHumanoid(SceneGraph& g, bool demo = false) {
    humanoid::Spec spec = humanoid::defaultSpec();
    spec.demo = demo;
    return humanoid::build(g, spec);
}

TEST(apiServerRecordsAndPlaysATake) {
    SceneGraph g;
    const humanoid::Built h = walkingHumanoid(g, false);
    SimulationEngine sim(&g);
    sim.setSpeedFactor(20);
    ApiServer server(&g, &sim);
    CHECK(server.start(0));
    ApiClient c;
    CHECK(c.connectTo(server.port()));
    const QString name = g.getObject(h.root)->name;
    auto place = [&] { return result(c.call("humanoid.state", {{"robot", name}})); };

    QJsonObject st = result(c.call("take.start", {{"robot", name}, {"rate", 20}}));
    const int take = st.value("take").toInt();
    CHECK(take > 0);
    CHECK(st.value("columns").toArray().contains(QJsonValue("LKnee")));
    CHECK(result(c.call("humanoid.walk", {{"robot", name}, {"forward", 500}})).value("state").toString() == "done");
    CHECK(result(c.call("sim.wait", {{"seconds", 0.2}})).contains("time"));
    st = result(c.call("take.status", {{"take", take}}));
    CHECK(st.value("frames").toInt() > 10);
    const QJsonObject t = result(c.call("take.stop", {{"take", take}}));
    CHECK(t.value("robot").toString() == name);
    CHECK_NEAR(t.value("rate").toDouble(), 20, 1e-9);
    const QJsonArray cols = t.value("columns").toArray(), rows = t.value("rows").toArray();
    CHECK(rows.size() >= st.value("frames").toInt());
    CHECK(rows.first().toArray().size() == cols.size());
    CHECK(errorCode(c.call("take.stop", {{"take", take}})) == "not_found");
    const int xCol = int(cols.toVariantList().indexOf("x"));
    CHECK_NEAR(rows.last().toArray()[xCol].toDouble(), 500, 0.1);

    // Again from where it stands, then the joints and its floor place only
    QJsonObject r = c.call("robot.play", {{"robot", name}, {"columns", cols}, {"rows", rows}, {"relative", true}});
    CHECK(result(r).value("state").toString() == "done");
    CHECK_NEAR(place().value("x").toDouble(), 1000, 0.1);
    QJsonArray few{"t", "x", "z", "heading", "LKnee", "RKnee", "Nonsense"}, fewRows;
    for (const QJsonValue& v : rows) {
        QJsonArray in = v.toArray(), out;
        for (const QJsonValue& col : few) {
            const int k = int(cols.toVariantList().indexOf(col.toString()));
            out.append(k >= 0 ? in[k] : QJsonValue(0));
        }
        fewRows.append(out);
    }
    r = c.call("robot.play", {{"robot", name}, {"columns", few}, {"rows", fewRows}, {"speed", 2}});
    CHECK(result(r).value("state").toString() == "done");
    CHECK_NEAR(place().value("x").toDouble(), 500, 0.1);
    CHECK_NEAR(place().value("z").toDouble(), 0, 0.1);

    // Refused: no joint and no place, rows of the wrong length
    CHECK(errorCode(c.call("robot.play", {{"robot", name}, {"columns", QJsonArray{"t", "foo"}},
                                           {"rows", QJsonArray{QJsonArray{0, 1}}}})) == "bad_request");
    CHECK(errorCode(c.call("robot.play", {{"robot", name}, {"columns", QJsonArray{"t", "LKnee"}},
                                           {"rows", QJsonArray{QJsonArray{0}}}})) == "bad_request");
    CHECK(errorCode(c.call("take.start", {{"robot", name}, {"rate", -1}})) == "bad_request");
}

TEST(apiServerWalksAHumanoid) {
    SceneGraph g;
    const humanoid::Built h = walkingHumanoid(g, false);
    humanoid::Spec ps = humanoid::defaultSpec();
    ps.base = humanoid::Base::Pedestal;
    ps.name = "Bust";
    ps.demo = false;
    humanoid::build(g, ps);
    const int spot = g.addObject("Spot", PrimitiveType::ImportedMesh);
    g.getObject(spot)->transform.position = {0, 0, 1};
    g.getObject(spot)->transform.rotation = {0, 90, 0};
    SimulationEngine sim(&g);
    sim.setSpeedFactor(20);
    ApiServer server(&g, &sim);
    CHECK(server.start(0));
    ApiClient c;
    CHECK(c.connectTo(server.port()));
    const QString name = g.getObject(h.root)->name;

    // Listed as walking; its state and gait before anything runs
    bool walks = false, bustWalks = true;
    for (auto v : c.call("scene.robots").value("result").toArray()) {
        if (v.toObject().value("name").toString() == name) walks = v.toObject().value("walks").toBool();
        if (v.toObject().value("name").toString() == "Bust") bustWalks = v.toObject().value("walks").toBool();
    }
    CHECK(walks && !bustWalks);
    CHECK(result(c.call("robot.info", {{"robot", name}})).value("walks").toBool());
    QJsonObject st = result(c.call("humanoid.state", {{"robot", name}}));
    CHECK_NEAR(st.value("x").toDouble(), 0, 1e-6);
    CHECK(!st.value("walking").toBool());
    CHECK(st.value("gait").toObject().value("stepLength").toDouble() > 100);
    CHECK(st.value("legs").toObject().value("left").toString() == "Left Foot");
    CHECK(!sim.isActive());

    // Half a metre ahead (mm), answered when it stands again
    QJsonObject r = c.call("humanoid.walk", {{"robot", name}, {"forward", 500}});
    CHECK(result(r).value("state").toString() == "done");
    st = result(c.call("humanoid.state", {{"robot", name}}));
    CHECK_NEAR(st.value("x").toDouble(), 500, 0.1);
    CHECK_NEAR(st.value("z").toDouble(), 0, 0.1);

    // To an object's spot, then a square in its own frame (z to its right)
    r = c.call("humanoid.walkPath", {{"robot", name}, {"points", QJsonArray{QJsonArray{0, 0}}}, {"frame", "Spot"},
                                     {"heading", 0}, {"speed", 400}});
    CHECK(result(r).value("state").toString() == "done");
    st = result(c.call("humanoid.state", {{"robot", name}}));
    CHECK_NEAR(st.value("x").toDouble(), 0, 0.1);
    CHECK_NEAR(st.value("z").toDouble(), 1000, 0.1);
    CHECK_NEAR(st.value("heading").toDouble(), 90, 1e-3);
    r = c.call("humanoid.walkPath", {{"robot", name}, {"frame", "robot"}, {"heading", 0},
                                     {"points", QJsonArray{QJsonObject{{"x", 400}, {"z", 0}}, QJsonObject{{"x", 400}, {"z", 400}},
                                                           QJsonObject{{"x", 0}, {"z", 400}}, QJsonObject{{"x", 0}, {"z", 0}}}}});
    CHECK(result(r).value("state").toString() == "done");
    st = result(c.call("humanoid.state", {{"robot", name}}));
    CHECK_NEAR(st.value("x").toDouble(), 0, 0.1);
    CHECK_NEAR(st.value("z").toDouble(), 1000, 0.1);
    CHECK_NEAR(st.value("heading").toDouble(), 90, 1e-3);
    r = c.call("humanoid.walkTo", {{"robot", name}, {"x", 300}, {"z", 1000}, {"heading", -90}});
    CHECK(result(r).value("state").toString() == "done");
    st = result(c.call("humanoid.state", {{"robot", name}}));
    CHECK_NEAR(st.value("x").toDouble(), 300, 0.1);
    CHECK_NEAR(st.value("heading").toDouble(), -90, 1e-3);

    // Without waiting: walking, then a stop that finishes the step
    r = c.call("humanoid.walk", {{"robot", name}, {"forward", 3000}, {"wait", false}});
    const int motion = result(r).value("motion").toInt();
    CHECK(motion > 0);
    c.call("sim.wait", {{"seconds", 1.0}});
    st = result(c.call("humanoid.state", {{"robot", name}}));
    CHECK(st.value("walking").toBool());
    CHECK(st.value("motion").toInt() == motion);
    CHECK(st.value("steps").toInt() > 5);
    CHECK(result(c.call("humanoid.stop", {{"robot", name}})).value("stopping").toInt() == 1);
    CHECK(result(c.call("motion.wait", {{"motion", motion}})).value("state").toString() == "done");
    st = result(c.call("humanoid.state", {{"robot", name}}));
    CHECK(!st.value("walking").toBool());
    CHECK(std::fabs(st.value("z").toDouble() - 1000) < 1500);

    // What it can't do
    CHECK(errorCode(c.call("humanoid.walk", {{"robot", "Bust"}, {"forward", 100}})) == "bad_request");
    CHECK(errorCode(c.call("humanoid.walk", {{"robot", name}, {"forward", 100}, {"stepLength", 5000}})) == "bad_request");
    CHECK(errorCode(c.call("humanoid.walkPath", {{"robot", name}, {"points", QJsonArray{1, 2}}})) == "bad_request");
    CHECK(errorCode(c.call("humanoid.walkTo", {{"robot", name}, {"x", 1}})) == "bad_request");
    CHECK(errorCode(c.call("humanoid.state", {{"robot", "Bust"}})) == "bad_request");
    CHECK(errorCode(c.call("humanoid.walk", {{"forward", 100}})) == "bad_request");
    // A reset puts it back where it started
    c.call("sim.reset");
    st = result(c.call("humanoid.state", {{"robot", name}}));
    CHECK_NEAR(st.value("x").toDouble(), 0, 1e-3);
    CHECK_NEAR(st.value("z").toDouble(), 0, 1e-3);
}

TEST(robotBuilderArticulatedMatchesDimensions) {
    robot::Spec s = robot::defaultSpec(robot::Kind::Articulated);
    s.dims = {{"h0", .2}, {"h1", .5}, {"a1", .17}, {"a2", .6}, {"a3", .05}, {"d4", .55}, {"d6", .09}};
    s.tool = robot::Tool::ParallelGripper;
    SceneGraph g;
    const robot::Built b = robot::build(g, s);
    CHECK(b.joints.size() == 6 && b.motors.size() == 6);
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    CHECK(kin::chainTo(g, b.tcp) == b.joints);
    CHECK(g.getMotor(b.motors[0])->name == "A1" && g.getMotor(b.motors[5])->name == "A6");
    const double tl = tcpLength(g, b);
    CHECK(tl > 0.05);
    // Home: forearm level, wrist bent down 90°, tool pointing at the floor
    QVector3D p = worldPos(g, b.tcp);
    CHECK_NEAR(p.x(), .17 + .55, 1e-4);
    CHECK_NEAR(p.y(), .2 + .5 + .6 + .05 - .09 - tl, 1e-4);
    CHECK_NEAR(p.z(), 0, 1e-4);
    CHECK_NEAR(kin::toolFrame(g, b.tcp).column(1).y(), 1, 1e-4);   // tool's −Y is world down
    // Wrist straight: the tool points along the forearm
    setArm(g, b, {0, 0, 0, 0, 0, 0});
    p = worldPos(g, b.tcp);
    CHECK_NEAR(p.x(), .17 + .55 + .09 + tl, 1e-4);
    CHECK_NEAR(p.y(), .2 + .5 + .6 + .05, 1e-4);
    // Directions: A2 + leans forward, A3 + lowers the forearm, A1 + turns +X towards −Z
    setArm(g, b, {0, 10, 0, 0, 90, 0});
    CHECK(worldPos(g, b.tcp).x() > .17f + .55f + .05f);
    setArm(g, b, {0, 0, 10, 0, 90, 0});
    CHECK(worldPos(g, b.tcp).y() < .2f + .5f + .6f + .05f - .09f - float(tl) - .05f);
    setArm(g, b, {90, 0, 0, 0, 90, 0});
    CHECK_NEAR(worldPos(g, b.tcp).z(), -(.17 + .55), 1e-4);
    setArm(g, b, {0, 0, 0, 0, 90, 0});

    // The analyser reads the entered dimensions back as DH parameters
    robot::Analysis a = robot::analyze(g, b.tcp, 8000);
    CHECK(a.error.isEmpty());
    CHECK(a.axes.size() == 6);
    CHECK(a.verticalBase);
    CHECK_NEAR(a.axes[0].d, .2 + .5, 1e-4);
    CHECK_NEAR(a.axes[0].a, .17, 1e-4);
    CHECK_NEAR(std::fabs(a.axes[0].alpha), 90, 1e-3);
    CHECK_NEAR(a.axes[1].a, .6, 1e-4);
    CHECK_NEAR(a.axes[1].alpha, 0, 1e-3);
    CHECK_NEAR(a.axes[2].a, .05, 1e-4);
    CHECK_NEAR(std::fabs(a.axes[2].alpha), 90, 1e-3);
    CHECK_NEAR(a.axes[3].d, .55, 1e-4);
    CHECK_NEAR(a.axes[3].a, 0, 1e-4);
    CHECK_NEAR(a.axes[4].a, 0, 1e-4);
    CHECK_NEAR(a.axes[4].d, 0, 1e-4);
    CHECK_NEAR(a.axes[5].d, .09 + tl, 1e-4);
    const double stretch = .17 + .6 + std::hypot(.05, .55) + .09 + tl;   // every link in line
    CHECK(a.maxReach <= stretch + 1e-3);
    CHECK(a.maxReach > 0.9 * stretch);
    CHECK(a.volume > 0.5);
    CHECK(a.warnings.isEmpty());
    for (const QString& w : a.warnings) std::printf("  warning: %s\n", qPrintable(w));
    CHECK(robot::findTool(g, b.root) == b.tcp);
    CHECK(robot::findTool(g, b.joints.size() ? g.getJoint(b.joints[2])->childBodyId : -1) == b.tcp);
    // Wrist singularity: A5 = 0 lines A4 up with A6
    setArm(g, b, {0, 0, 0, 0, 0, 0});
    robot::analyzePose(g, a);
    CHECK(hasWarning(a, "Wrist singularity"));
    CHECK(a.dexterity < 1e-3);
    setArm(g, b, {20, 10, 20, 30, 60, 0});
    robot::analyzePose(g, a);
    CHECK(a.warnings.isEmpty());
    CHECK(a.dexterity > 0.01);
    // Inverse kinematics reaches a frame it can reach
    setArm(g, b, {35, 20, 15, -30, 70, 40});
    const QMatrix4x4 goal = kin::toolFrame(g, b.tcp);
    setArm(g, b, {0, 0, 0, 0, 90, 0});
    const kin::Result r = kin::solve(g, b.tcp, b.joints, kin::targetFrom(goal));
    CHECK(r.reached);
}

TEST(robotBuilderScaraMatchesDimensions) {
    robot::Spec s = robot::defaultSpec(robot::Kind::Scara);
    s.dims = {{"H", .5}, {"L1", .3}, {"L2", .25}, {"S", .18}, {"Fz", .33}};
    s.tool = robot::Tool::SuctionCup;
    SceneGraph g;
    const robot::Built b = robot::build(g, s);
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    CHECK(kin::chainTo(g, b.tcp) == b.joints);
    const Joint* z = g.getJoint(b.joints[2]);
    CHECK(z->type == JointType::Prismatic);
    CHECK_NEAR(z->limitMax, .18, 1e-6);   // J3's range follows the stroke
    const double tl = tcpLength(g, b);
    QVector3D p = worldPos(g, b.tcp);
    CHECK_NEAR(p.x(), .55, 1e-4);
    CHECK_NEAR(p.y(), .33 - tl, 1e-4);
    setArm(g, b, {0, 0, .18f, 0});
    CHECK_NEAR(worldPos(g, b.tcp).y(), .33 - .18 - tl, 1e-4);
    setArm(g, b, {90, 0, 0, 0});
    p = worldPos(g, b.tcp);
    CHECK_NEAR(p.x(), 0, 1e-4);
    CHECK_NEAR(p.z(), -.55, 1e-4);
    setArm(g, b, {0, 90, 0, 0});
    p = worldPos(g, b.tcp);
    CHECK_NEAR(p.x(), .3, 1e-4);
    CHECK_NEAR(p.z(), -.25, 1e-4);
    setArm(g, b, {0, 0, 0, 77});
    CHECK_NEAR(worldPos(g, b.tcp).x(), .55, 1e-4);   // J4 turns the tool about its own axis
    setArm(g, b, {0, 0, 0, 0});

    robot::Analysis a = robot::analyze(g, b.tcp, 8000);
    CHECK_NEAR(a.maxReach, .55, 2e-3);
    const double inner = std::sqrt(.3 * .3 + .25 * .25 + 2 * .3 * .25 * std::cos(qDegreesToRadians(145.0)));
    CHECK_NEAR(a.minReach, inner, 0.01);
    CHECK_NEAR(a.maxHeight - a.minHeight, .18, 1e-3);
    CHECK_NEAR(a.axes[0].a, .3, 1e-4);
    CHECK_NEAR(a.axes[1].a, .25, 1e-4);
    CHECK_NEAR(a.axes[0].alpha, 0, 1e-3);
    CHECK(hasWarning(a, "stretched"));   // J2 = 0 is the SCARA's singularity
    setArm(g, b, {0, 40, .05f, 0});
    robot::analyzePose(g, a);
    CHECK(a.warnings.isEmpty());
    for (const QString& w : a.warnings) std::printf("  warning: %s\n", qPrintable(w));
}

TEST(robotBuilderPalletizerKeepsFlangeLevel) {
    robot::Spec s = robot::defaultSpec(robot::Kind::Palletizer);
    s.dims = {{"h0", .1}, {"h1", .9}, {"a1", .3}, {"a2", 1.1}, {"a3", 1.3}, {"d5", .3}};
    SceneGraph g;
    const robot::Built b = robot::build(g, s);
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    CHECK(kin::chainTo(g, b.tcp) == b.joints);   // the levelling joints follow, they are not axes
    const double tl = tcpLength(g, b);
    for (auto [q2, q3] : {std::pair{0.f, 0.f}, {30.f, -20.f}, {-25.f, 60.f}, {80.f, 70.f}}) {
        setArm(g, b, {15, q2, q3, 0});
        const QMatrix4x4 f = kin::toolFrame(g, b.tcp);
        CHECK_NEAR(f.column(1).y(), 1, 1e-4);   // flange always faces down
        const double r = .3 + 1.1 * std::sin(qDegreesToRadians(double(q2))) + 1.3 * std::cos(qDegreesToRadians(double(q3)));
        const double y = .1 + .9 + 1.1 * std::cos(qDegreesToRadians(double(q2))) - 1.3 * std::sin(qDegreesToRadians(double(q3))) - .3 - tl;
        const QVector3D p = f.column(3).toVector3D();
        CHECK_NEAR(std::hypot(p.x(), p.z()), r, 1e-4);
        CHECK_NEAR(p.y(), y, 1e-4);
    }
    setArm(g, b, {0, 10, 10, 0});
    const robot::Analysis a = robot::analyze(g, b.tcp, 6000);
    CHECK(a.error.isEmpty());
    CHECK_NEAR(a.axes[0].d, .1 + .9, 1e-4);
    CHECK_NEAR(a.axes[0].a, .3, 1e-4);
    CHECK_NEAR(a.axes[1].a, 1.1, 1e-4);
}

TEST(robotBuilderEveryPresetBuildsAndWorks) {
    CHECK(robot::presets().size() >= 25);
    std::set<robot::Tool> tools;
    for (const robot::Preset& pr : robot::presets()) {
        tools.insert(pr.spec.tool);
        robot::Spec spec = pr.spec;
        const QStringList problems = robot::validate(spec);
        if (!problems.isEmpty()) std::printf("  %s: %s\n", qPrintable(pr.name), qPrintable(problems.join("; ")));
        CHECK(problems.isEmpty());
        // JSON round trip
        const robot::Spec back = robot::fromJson(robot::toJson(pr.spec));
        CHECK(back.kind == pr.spec.kind && back.tool == pr.spec.tool && back.dims == pr.spec.dims && back.style == pr.spec.style);
        SceneGraph g;
        const robot::Built b = robot::build(g, pr.spec);
        const QStringList errs = AutomationEngine::validateAll(g);
        if (!errs.isEmpty()) std::printf("  %s: %s\n", qPrintable(pr.name), qPrintable(errs.join("; ")));
        CHECK(errs.isEmpty());
        const int n = robot::axisCount(pr.spec.kind);
        CHECK(int(b.joints.size()) == n);
        CHECK(int(kin::chainTo(g, b.tcp).size()) == n);
        CHECK(robot::findTool(g, b.root) == b.tcp);
        // IK to a frame a few axes away from home
        std::vector<float> q;
        for (int i = 0; i < n; ++i) {
            const robot::AxisSpec& ax = pr.spec.axes[i];
            q.push_back(float(ax.home + (ax.max - ax.home) * (i % 2 ? 0.2 : 0.25)));
        }
        setArm(g, b, q);
        kin::Target t = kin::targetFrom(kin::toolFrame(g, b.tcp));
        t.useOrientation = n >= 6;
        std::vector<float> home;
        for (int i = 0; i < n; ++i) home.push_back(float(pr.spec.axes[i].home));
        setArm(g, b, home);
        const kin::Result r = kin::solve(g, b.tcp, b.joints, t, 200);
        if (!r.reached) std::printf("  %s: IK %g m %g deg\n", qPrintable(pr.name), r.positionError, r.angleError);
        CHECK(r.reached);
        setArm(g, b, home);
        // Run it: every Bool input on; tool actuators (fingers, spindles) move
        std::map<int, float> start;
        for (auto& [id, j] : g.allJoints()) start[id] = j.currentValue;
        SimulationEngine sim(&g);
        sim.step();
        for (auto& [id, sg] : g.allSignals())
            if (sg.kind == SignalKind::Input && sg.type == SignalType::Bool) sim.automation()->setInput(sg.name, 1);
        for (int i = 0; i < 120; ++i) sim.step();
        CHECK(sim.automation()->warnings().isEmpty());
        for (auto& [id, m] : g.allMotors()) {
            if (std::find(b.motors.begin(), b.motors.end(), id) != b.motors.end()) continue;
            if (std::fabs(g.getJoint(m.jointId)->currentValue - start[m.jointId]) < 1e-4f)
                std::printf("  %s: tool actuator %s did not move\n", qPrintable(pr.name), qPrintable(m.name));
            CHECK(std::fabs(g.getJoint(m.jointId)->currentValue - start[m.jointId]) > 1e-4f);
        }
        sim.reset();
        // Saves and loads like any scene
        SceneGraph loaded;
        roundTrip(g, loaded);
        CHECK(AutomationEngine::validateAll(loaded).isEmpty());
        CHECK(loaded.allJoints().size() == g.allJoints().size());
        const robot::Analysis a = robot::analyze(g, b.tcp, 3000);
        CHECK(a.error.isEmpty());
        CHECK(a.maxReach > 0.1 && a.volume > 0);
    }
    CHECK(tools.size() >= 14);
}

TEST(robotBuilderSecondRobotGetsFreeNames) {
    SceneGraph g;
    robot::Spec s = robot::defaultSpec(robot::Kind::Articulated);
    s.tool = robot::Tool::ParallelGripper;
    robot::build(g, s);
    const robot::Built b2 = robot::build(g, s);
    CHECK(g.getMotor(b2.motors[0])->name == "A1_2");
    CHECK(signalNamed(g, "Grip") && signalNamed(g, "Grip_2"));
    CHECK(signalNamed(g, "InGripper_2"));
    bool bound = false;
    for (auto& [id, m] : g.allMotors()) if (m.targetExpr.startsWith("Grip_2 ?")) bound = true;
    CHECK(bound);
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    // Out-of-range values are clamped and reported
    robot::Spec bad = robot::defaultSpec(robot::Kind::Scara);
    bad.dims["Fz"] = 1.5;
    bad.dims["L1"] = 99;
    const QStringList problems = robot::validate(bad);
    CHECK(!problems.isEmpty());
    CHECK_NEAR(bad.dims["Fz"], bad.dims["H"], 1e-9);
    CHECK_NEAR(bad.dims["L1"], 1.5, 1e-9);
}

// ── Humanoid builder ────────────────────────────────────────────────────────

namespace {
void setJoints(SceneGraph& g, const humanoid::Built& b, const std::map<QString, float>& v) {
    for (auto& [id, j] : g.allJoints()) if (j.coupledTo < 0) g.getJoint(id)->currentValue = 0;
    for (auto& [name, val] : v) {
        auto it = b.joints.find(name);
        if (it != b.joints.end()) g.getJoint(it->second)->currentValue = val;
    }
    g.applyCouplings();
}
} // namespace

TEST(humanoidBuilderMatchesDimensions) {
    humanoid::Spec s = humanoid::defaultSpec();
    s.dims["Ha"] = .08; s.dims["Ls"] = .34; s.dims["Lt"] = .36; s.dims["Hp"] = .12; s.dims["Ht"] = .4;
    s.dims["Hn"] = .1; s.dims["Dh"] = .22; s.dims["Ws"] = .4; s.dims["Wh"] = .2;
    s.dims["Lu"] = .29; s.dims["Lf"] = .25; s.dims["Lh"] = .17;
    s.hand = humanoid::Hand::Fingers;
    CHECK(humanoid::validate(s).isEmpty());
    CHECK_NEAR(humanoid::height(s), 1.62, 1e-9);
    SceneGraph g;
    const humanoid::Built b = humanoid::build(g, s);
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    CHECK(humanoid::jointCount(s) == 28);
    CHECK(b.motors.size() == 30);   // + LHand, RHand
    for (const char* n : {"WaistYaw", "NeckPitch", "LShoulderPitch", "RShoulderPitch", "LElbow", "RKnee", "LAnkleRoll", "LHand", "RHand"})
        CHECK(b.joints.count(n) == 1);
    CHECK(b.joints.count("WaistRoll") == 0 && b.joints.count("LToe") == 0);
    // Home: the hands hang straight down at the shoulders' width
    const float shoulder = .08f + .34f + .36f + .12f + .4f;
    QVector3D p = worldPos(g, b.leftHand);
    CHECK_NEAR(p.x(), 0, 1e-4);
    CHECK_NEAR(p.z(), -.2, 1e-4);
    CHECK_NEAR(p.y(), shoulder - .29 - .25 - (.45 * .17 + .5 * .55 * .17), 1e-4);
    CHECK_NEAR(worldPos(g, b.rightHand).z(), .2, 1e-4);
    CHECK_NEAR(worldPos(g, b.leftFoot).y(), .08, 1e-4);
    CHECK_NEAR(worldPos(g, b.leftFoot).z(), -.1, 1e-4);
    CHECK_NEAR(worldPos(g, b.head).y(), shoulder + .1, 1e-4);
    CHECK_NEAR(kin::toolFrame(g, b.leftHand).column(1).y(), 1, 1e-4);   // fingers (−Y) point down
    // Directions
    setJoints(g, b, {{"LShoulderRoll", 90}});
    CHECK_NEAR(worldPos(g, b.leftHand).y(), shoulder, 1e-4);
    CHECK(worldPos(g, b.leftHand).z() < -.2f - .5f);        // left arm out to the left (−Z)
    setJoints(g, b, {{"RShoulderRoll", 90}});
    CHECK(worldPos(g, b.rightHand).z() > .2f + .5f);        // mirrored: right arm out to the right
    setJoints(g, b, {{"LShoulderPitch", 90}});
    CHECK(worldPos(g, b.leftHand).x() > .5f);               // forward
    setJoints(g, b, {{"LElbow", 90}});
    CHECK(worldPos(g, b.leftHand).x() > .2f);               // forearm forward
    setJoints(g, b, {{"LHipPitch", 45}});
    CHECK(worldPos(g, b.leftFoot).x() > .2f);               // leg forward
    setJoints(g, b, {{"LKnee", 90}});
    CHECK(worldPos(g, b.leftFoot).x() < -.3f);              // shin back
    setJoints(g, b, {{"NeckYaw", 90}});
    CHECK_NEAR(g.worldTransform(b.head).column(0).toVector3D().z(), -1, 1e-4);   // looks to its left
    setJoints(g, b, {{"WaistPitch", 30}});
    CHECK(worldPos(g, b.head).x() > .2f);                   // bends forward
    // Equal values on both sides give a mirror image
    std::map<QString, float> v;
    int k = 0;
    for (const humanoid::JointDef& d : humanoid::jointDefs()) {
        if (!d.sided) continue;
        const float val = float(10 + 7 * (k++ % 5));
        v["L" + d.key] = val;
        v["R" + d.key] = val;
    }
    setJoints(g, b, v);
    for (auto [l, r] : {std::pair{b.leftHand, b.rightHand}, {b.leftFoot, b.rightFoot}}) {
        const QVector3D a = worldPos(g, l), c = worldPos(g, r);
        CHECK_NEAR(a.x(), c.x(), 1e-4);
        CHECK_NEAR(a.y(), c.y(), 1e-4);
        CHECK_NEAR(a.z(), -c.z(), 1e-4);
    }
    // The hand actuator curls every finger
    setJoints(g, b, {{"LHand", 90}});
    int moved = 0;
    for (auto& [id, j] : g.allJoints())
        if (j.coupledTo == b.joints.at("LHand") && std::fabs(j.currentValue) > 1) ++moved;
    CHECK(moved == 9);   // 3 more fingers, 4 tips, thumb and its tip
}

TEST(humanoidBuilderEveryPresetBuildsAndWorks) {
    CHECK(humanoid::presets().size() >= 6);
    for (const humanoid::Preset& pr : humanoid::presets()) {
        humanoid::Spec spec = pr.spec;
        const QStringList problems = humanoid::validate(spec);
        if (!problems.isEmpty()) std::printf("  %s: %s\n", qPrintable(pr.name), qPrintable(problems.join("; ")));
        CHECK(problems.isEmpty());
        CHECK(humanoid::toJson(humanoid::fromJson(humanoid::toJson(pr.spec))) == humanoid::toJson(pr.spec));
        SceneGraph g;
        const humanoid::Built b = humanoid::build(g, pr.spec);
        const QStringList errs = AutomationEngine::validateAll(g);
        if (!errs.isEmpty()) std::printf("  %s: %s\n", qPrintable(pr.name), qPrintable(errs.join("; ")));
        CHECK(errs.isEmpty());
        const int hands = pr.spec.hand == humanoid::Hand::Mitten ? 0 : 2;
        CHECK(int(b.motors.size()) == humanoid::jointCount(pr.spec) + hands);
        CHECK((b.leftFoot >= 0) == (pr.spec.base == humanoid::Base::Legs));
        CHECK(b.sequence >= 0 && b.poses.size() >= 3);
        // Nothing below the floor at Home
        for (auto& [id, o] : g.allObjects()) {
            if (o.type == PrimitiveType::ImportedMesh) continue;
            const QMatrix4x4 m = g.worldTransform(id);
            for (const QVector3D& c : {QVector3D(-.5f, -.5f, -.5f), QVector3D(.5f, -.5f, .5f)})
                CHECK(m.map(c).y() > -1e-3f || o.type != PrimitiveType::Cube);
        }
        // IK: the left hand reaches a frame it had a few joints away
        const std::vector<int> chain = kin::chainTo(g, b.leftHand);
        CHECK(chain.size() >= 3);
        std::map<int, float> home;
        for (int jid : chain) home[jid] = g.getJoint(jid)->currentValue;
        for (int jid : chain) {
            Joint* j = g.getJoint(jid);
            j->currentValue = j->currentValue + (j->limitMax - j->currentValue) * 0.25f;
        }
        g.applyCouplings();
        kin::Target t = kin::targetFrom(kin::toolFrame(g, b.leftHand));
        t.useOrientation = chain.size() >= 6;
        for (int jid : chain) g.getJoint(jid)->currentValue = home[jid];
        g.applyCouplings();
        const kin::Result r = kin::solve(g, b.leftHand, chain, t, 200);
        if (!r.reached) std::printf("  %s: IK %g m %g deg\n", qPrintable(pr.name), r.positionError, r.angleError);
        CHECK(r.reached);
        for (int jid : chain) g.getJoint(jid)->currentValue = home[jid];
        g.applyCouplings();
        // Play: the demo sequence moves the arms, without warnings
        std::map<int, float> start;
        for (auto& [id, j] : g.allJoints()) start[id] = j.currentValue;
        SimulationEngine sim(&g);
        for (int i = 0; i < 90; ++i) sim.step();
        CHECK(sim.automation()->warnings().isEmpty());
        int moved = 0;
        for (auto& [id, j] : g.allJoints()) if (std::fabs(j.currentValue - start[id]) > 1) ++moved;
        CHECK(moved >= 2);
        sim.reset();
        SceneGraph loaded;
        roundTrip(g, loaded);
        CHECK(AutomationEngine::validateAll(loaded).isEmpty());
        CHECK(loaded.allJoints().size() == g.allJoints().size());
        CHECK(loaded.allSequences().size() == 1);
    }
}

TEST(humanoidBuilderJointChoiceAndNames) {
    humanoid::Spec s = humanoid::defaultSpec();
    for (const humanoid::JointDef& d : humanoid::jointDefs()) s.joints[d.key].enabled = false;
    s.joints["Elbow"].enabled = true;
    s.joints["NeckYaw"].enabled = true;
    s.hand = humanoid::Hand::Gripper;
    s.demo = false;
    SceneGraph g;
    const humanoid::Built b = humanoid::build(g, s);
    CHECK(b.motors.size() == 5);   // NeckYaw, LElbow, RElbow, LGrip, RGrip
    CHECK(g.getMotor(b.motors[0])->name == "NeckYaw");
    CHECK(g.getMotor(b.motors[1])->name == "LElbow" && g.getMotor(b.motors[2])->name == "RElbow");
    CHECK(g.getMotor(b.motors[3])->name == "LGrip");
    CHECK(b.poses.size() == 1 && b.sequence < 0);
    CHECK(g.getJoint(b.joints.at("LGrip"))->type == JointType::Prismatic);
    // The jaws close towards each other
    const int jawA = g.getJoint(b.joints.at("LGrip"))->childBodyId;
    const QVector3D a0 = worldPos(g, jawA);
    g.getJoint(b.joints.at("LGrip"))->currentValue = g.getJoint(b.joints.at("LGrip"))->limitMax;
    g.applyCouplings();
    CHECK(worldPos(g, jawA).x() < a0.x() - 1e-3f);
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    // A second humanoid gets free names
    s.name = "Twin";
    s.demo = true;
    const humanoid::Built b2 = humanoid::build(g, s);
    CHECK(b2.joints.count("LElbow_2") == 1);
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    // Pedestal: no legs, and the height scales every length
    humanoid::Spec p = humanoid::defaultSpec();
    p.base = humanoid::Base::Pedestal;
    const double lu = p.dim("Lu");
    humanoid::scaleToHeight(p, 2 * humanoid::height(p));
    CHECK_NEAR(p.dim("Lu"), 2 * lu, 1e-9);
    SceneGraph g2;
    const humanoid::Built b3 = humanoid::build(g2, p);
    CHECK(b3.leftFoot < 0 && b3.joints.count("LKnee") == 0);
    CHECK(AutomationEngine::validateAll(g2).isEmpty());
}

// ── Walking ──────────────────────────────────────────────────────────────────

namespace {
gait::Place placeNow(const SceneGraph& g, int root) { return gait::placeOf(kin::toolFrame(g, root)); }

// Runs a walk motion and checks that a foot always stands still on the floor
// (flat, or rolled on its toes or heel: one of them stays put) and that no
// sole goes through it
struct WalkWatch {
    SceneGraph& g;
    gait::Legs legs;
    QVector3D last[2][2];    // [foot][toe, heel]
    double maxSlip = 0;      // the least moving foot's motion in one scan, m
    double lowest = 1e9;     // the lowest toe or heel seen, m above the floor
    double floorY = 0;
    WalkWatch(SceneGraph& graph, const humanoid::Built& b) : g(graph) {
        QString why;
        gait::findLegs(g, b.root, &legs, &why);
        floorY = g.worldTransform(b.root)(1, 3);
        points(last);
    }
    void points(QVector3D out[2][2]) const {
        const QVector3D half(float(legs.footLength / 2), 0, 0);
        for (int s = 0; s < 2; ++s) {
            const QMatrix4x4 f = kin::toolFrame(g, legs.foot[s]);
            out[s][0] = f.map(legs.soleCentre + half);
            out[s][1] = f.map(legs.soleCentre - half);
        }
    }
    void operator()() {
        QVector3D now[2][2];
        points(now);
        double d[2];
        for (int s = 0; s < 2; ++s) {
            d[s] = std::min((now[s][0] - last[s][0]).length(), (now[s][1] - last[s][1]).length());
            for (int e = 0; e < 2; ++e) {
                lowest = std::min(lowest, double(now[s][e].y()) - floorY);
                last[s][e] = now[s][e];
            }
        }
        maxSlip = std::max(maxSlip, std::min(d[0], d[1]));
    }
};
} // namespace

TEST(walkPlansStepsAndPaths) {
    gait::Params p;
    p.stepLength = 0.3; p.sideStep = 0.1; p.turnPerStep = qDegreesToRadians(20.0); p.stepTime = 0.5;
    const gait::Place start{1, 2, 0};
    // 1 m ahead: four equal steps, then the other foot alongside
    std::vector<gait::Place> q = gait::relativePlaces(start, 1.0, 0, 0, p);
    CHECK(q.size() == 4);
    CHECK_NEAR(q.back().x, 2, 1e-9);
    CHECK_NEAR(q.back().z, 2, 1e-9);
    gait::Plan plan = gait::makePlan(start, q, p);
    CHECK(plan.steps.size() == 5);
    CHECK(plan.steps[0].side != plan.steps[1].side);
    CHECK(plan.steps[3].side != plan.steps[4].side);
    CHECK_NEAR(plan.end().x, 2, 1e-9);
    CHECK_NEAR(plan.duration(), 2 * plan.prep + 5 * 0.5, 1e-9);
    // Its left is −Z; a turn to the left is a positive heading and starts with the left foot
    q = gait::relativePlaces(start, 0, 0.25, 0, p);
    CHECK(q.size() == 3);
    CHECK_NEAR(q.back().z, 1.75, 1e-9);
    CHECK(gait::makePlan(start, q, p).steps[0].side == 0);
    q = gait::relativePlaces(start, 0, 0, qDegreesToRadians(-90.0), p);
    CHECK(q.size() == 5);
    CHECK_NEAR(q.back().heading, qDegreesToRadians(-90.0), 1e-9);
    CHECK(gait::makePlan(start, q, p).steps[0].side == 1);
    // A heading of 90° faces −Z
    const QMatrix4x4 f = gait::frameOf({0, 0, qDegreesToRadians(90.0)});
    CHECK_NEAR(f.map(QVector3D(1, 0, 0)).z(), -1, 1e-6);
    CHECK_NEAR(gait::placeOf(f).heading, qDegreesToRadians(90.0), 1e-6);

    // A square path: steps no longer than a step, turns no sharper than a step's turn
    const std::vector<QPointF> square{{2, 2}, {2, 3}, {1, 3}, {1, 2}};
    q = gait::pathPlaces(start, square, 0.0, p);
    CHECK(q.size() > 10);
    gait::Place prev = start;
    bool short_ = true, gentle = true;
    for (const gait::Place& x : q) {
        if (std::hypot(x.x - prev.x, x.z - prev.z) > p.stepLength + 1e-9) short_ = false;
        if (std::fabs(gait::angleDiff(x.heading, prev.heading)) > p.turnPerStep + 1e-9) gentle = false;
        prev = x;
    }
    CHECK(short_);
    CHECK(gentle);
    CHECK_NEAR(q.back().x, 1, 1e-9);
    CHECK_NEAR(q.back().z, 2, 1e-9);
    CHECK_NEAR(q.back().heading, 0, 1e-9);
    // Points behind it: it turns round on the spot first
    q = gait::pathPlaces(start, {{0, 2}}, std::nullopt, p);
    CHECK(std::fabs(gait::angleDiff(q[size_t(8)].heading, qDegreesToRadians(180.0))) < 1e-6);
    CHECK_NEAR(q.back().x, 0, 1e-9);
}

TEST(walkHumanoidFindsItsLegs) {
    SceneGraph g;
    const humanoid::Built b = walkingHumanoid(g);
    gait::Legs legs;
    QString why;
    CHECK(gait::findLegs(g, b.root, &legs, &why));
    CHECK(legs.foot[0] == b.leftFoot && legs.foot[1] == b.rightFoot);
    CHECK(legs.chain[0].size() == 6 && legs.chain[1].size() == 6);
    CHECK(legs.arm[0] == b.joints.at("LShoulderPitch") && legs.arm[1] == b.joints.at("RShoulderPitch"));
    const humanoid::Spec spec = humanoid::defaultSpec();
    CHECK_NEAR(legs.legLength, spec.dim("Lt") + spec.dim("Ls"), 1e-4);
    CHECK_NEAR(legs.hipWidth, spec.dim("Wh"), 1e-4);
    CHECK_NEAR(legs.ankleHeight, spec.dim("Ha"), 1e-4);
    CHECK(legs.footLength > 0.8 * spec.dim("Lft") && legs.footLength < 1.2 * spec.dim("Lft"));
    // The left foot is on its left (−Z)
    CHECK(legs.footHome[0](2, 3) < 0 && legs.footHome[1](2, 3) > 0);
    // A humanoid on a pedestal can't walk
    SceneGraph p;
    humanoid::Spec ps = humanoid::defaultSpec();
    ps.base = humanoid::Base::Pedestal;
    const humanoid::Built pb = humanoid::build(p, ps);
    CHECK(!gait::findLegs(p, pb.root, &legs, &why));
    CHECK(why.contains("Left Foot"));
    CHECK(!gait::canWalk(p, pb.root));
}

TEST(walkHumanoidForwardKeepsAFootPlanted) {
    SceneGraph g;
    const humanoid::Built b = walkingHumanoid(g);
    // Turned and moved: the walk is in its own frame
    g.getObject(b.root)->transform.position = {0.5f, 0, -1};
    g.getObject(b.root)->transform.rotation = {0, 30, 0};
    AutomationEngine e(&g);
    CHECK(e.start(nullptr));
    const gait::Place start = placeNow(g, b.root);
    std::map<int, float> legs0;
    for (auto& [name, jid] : b.joints) legs0[jid] = g.getJoint(jid)->currentValue;

    AutomationEngine::MotionRequest r;
    r.kind = AutomationEngine::MotionKind::Walk;
    r.walk.root = b.root;
    r.walk.forward = 1.0;
    QString code, err;
    const int id = e.submitMotion(r, &code, &err);
    if (!id) std::printf("  %s %s\n", qPrintable(code), qPrintable(err));
    CHECK(id > 0);
    WalkWatch watch(g, b);
    int walking = 0;
    const auto st = runMotion(e, id, 30, [&] {
        watch();
        if (!e.walks().empty()) ++walking;
    });
    if (st.state != AutomationEngine::MotionState::Done) std::printf("  %s\n", qPrintable(st.error));
    CHECK(st.state == AutomationEngine::MotionState::Done);
    CHECK(walking > 10);
    if (watch.maxSlip >= 2e-4 || watch.lowest < -1e-3)
        std::printf("  slip %g lowest %g\n", watch.maxSlip, watch.lowest);
    CHECK(watch.maxSlip < 2e-4);                       // one foot always stands
    CHECK(watch.lowest > -1e-3);                       // and no sole sinks into the floor
    const gait::Place end = placeNow(g, b.root);
    CHECK_NEAR(gait::angleDiff(end.heading, start.heading), 0, 1e-4);
    const gait::Place want = gait::advance(start, 1.0, 0, 0);
    CHECK_NEAR(end.x, want.x, 1e-4);
    CHECK_NEAR(end.z, want.z, 1e-4);
    CHECK_NEAR(g.worldTransform(b.root)(1, 3), 0, 1e-5);   // standing up again
    // Legs straight, arms back where they were
    for (auto& [jid, v] : legs0) CHECK_NEAR(g.getJoint(jid)->currentValue, v, 0.05);
    CHECK(e.walks().empty());
}

TEST(walkHumanoidPathTurnAndStop) {
    SceneGraph g;
    const humanoid::Built b = walkingHumanoid(g);
    AutomationEngine e(&g);
    CHECK(e.start(nullptr));
    using MS = AutomationEngine::MotionState;
    QString code, err;
    // A square back to where it started, facing the way it started
    AutomationEngine::MotionRequest r;
    r.kind = AutomationEngine::MotionKind::Walk;
    r.walk.root = b.root;
    r.walk.path = true;
    r.walk.points = {{1, 0}, {1, -1}, {0, -1}, {0, 0}};
    r.walk.heading = 0;
    const int id = e.submitMotion(r, &code, &err);
    CHECK(id > 0);
    WalkWatch watch(g, b);
    CHECK(runMotion(e, id, 120, [&] { watch(); }).state == MS::Done);
    CHECK(watch.maxSlip < 2e-4);
    gait::Place p = placeNow(g, b.root);
    CHECK_NEAR(p.x, 0, 1e-4);
    CHECK_NEAR(p.z, 0, 1e-4);
    CHECK_NEAR(p.heading, 0, 1e-4);

    // Sideways and a turn on the spot
    AutomationEngine::MotionRequest side = r;
    side.walk = {};
    side.walk.root = b.root;
    side.walk.left = 0.3;
    side.walk.turn = 90;
    CHECK(runMotion(e, e.submitMotion(side, &code, &err), 60).state == MS::Done);
    p = placeNow(g, b.root);
    CHECK_NEAR(qRadiansToDegrees(p.heading), 90, 1e-3);

    // A long walk stopped: it finishes the step under way and brings its feet together
    AutomationEngine::MotionRequest far = side;
    far.walk.left = 0;
    far.walk.turn = 0;
    far.walk.forward = 5;
    const int fid = e.submitMotion(far, &code, &err);
    const int queued = e.submitMotion(far, &code, &err);
    for (int i = 0; i < 100; ++i) e.scan(0.016);
    CHECK(e.walks().size() == 1 && e.walks()[0].stepsDone >= 1);
    const int total = e.walks()[0].steps;
    CHECK(e.stopWalking(b.root) == 2);
    CHECK(e.motionStatus(queued).state == MS::Cancelled);
    CHECK(e.walks()[0].steps < total);
    CHECK(runMotion(e, fid, 10).state == MS::Done);
    const gait::Place stopped = placeNow(g, b.root);
    CHECK(std::hypot(stopped.x - p.x, stopped.z - p.z) < 1.5);
    gait::Legs legs;
    CHECK(gait::findLegs(g, b.root, &legs, &err));
    // Feet together: both at their home spots around the root
    const QMatrix4x4 floor = gait::frameOf(stopped, 0);
    for (int s = 0; s < 2; ++s)
        CHECK_NEAR((worldPos(g, legs.foot[s]) - (floor * legs.footHome[s]).column(3).toVector3D()).length(), 0, 2e-3);
}

TEST(walkEveryLeggedPreset) {
    for (const humanoid::Preset& pr : humanoid::presets()) {
        if (pr.spec.base != humanoid::Base::Legs) continue;
        SceneGraph g;
        humanoid::Spec spec = pr.spec;
        spec.demo = false;
        const humanoid::Built b = humanoid::build(g, spec);
        AutomationEngine e(&g);
        CHECK(e.start(nullptr));
        QString code, err;
        AutomationEngine::MotionRequest r;
        r.kind = AutomationEngine::MotionKind::Walk;
        r.walk.root = b.root;
        r.walk.forward = 0.8;
        r.walk.turn = 60;
        const int id = e.submitMotion(r, &code, &err);
        if (!id) std::printf("  %s: %s\n", qPrintable(pr.name), qPrintable(err));
        WalkWatch watch(g, b);
        const auto st = runMotion(e, id, 60, [&] { watch(); });
        if (st.state != AutomationEngine::MotionState::Done || watch.maxSlip >= 2e-4 || watch.lowest < -1e-3)
            std::printf("  %s: %s slip %g lowest %g\n", qPrintable(pr.name), qPrintable(st.error), watch.maxSlip, watch.lowest);
        CHECK(st.state == AutomationEngine::MotionState::Done);
        CHECK(watch.maxSlip < 2e-4);
        CHECK(watch.lowest > -1e-3);
        CHECK_NEAR(qRadiansToDegrees(placeNow(g, b.root).heading), 60, 1e-3);
        // Short steps, ahead and backwards
        r.walk.turn = 0;
        r.walk.params.stepLength = 0.15;
        for (double d : {0.3, -0.3}) {
            r.walk.forward = d;
            const auto st2 = runMotion(e, e.submitMotion(r, &code, &err), 60, [&] { watch(); });
            if (st2.state != AutomationEngine::MotionState::Done)
                std::printf("  %s short steps %g: %s\n", qPrintable(pr.name), d, qPrintable(st2.error));
            CHECK(st2.state == AutomationEngine::MotionState::Done);
        }
        r.walk.params.stepLength = 0;
        // Backwards too
        r.walk.forward = -0.5;
        r.walk.turn = 0;
        const auto back = runMotion(e, e.submitMotion(r, &code, &err), 60, [&] { watch(); });
        if (back.state != AutomationEngine::MotionState::Done || watch.maxSlip >= 2e-4)
            std::printf("  %s backwards: %s slip %g\n", qPrintable(pr.name), qPrintable(back.error), watch.maxSlip);
        CHECK(back.state == AutomationEngine::MotionState::Done);
        CHECK(watch.maxSlip < 2e-4);
    }
}

TEST(walkHumanoidDemoComesBack) {
    SceneGraph g;
    const humanoid::Built b = walkingHumanoid(g, true);
    const Sequence* q = g.getSequence(b.sequence);
    CHECK(q && std::count_if(q->operations.begin(), q->operations.end(),
                             [](const Operation& op) { return op.type == OperationType::Walk; }) == 4);
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    AutomationEngine e(&g);
    CHECK(e.start(nullptr));
    bool walked = false;
    for (int i = 0; i < 6000 && e.seqStatus(b.sequence).cycles < 1; ++i) {
        e.scan(0.016);
        if (!e.walks().empty()) {
            walked = true;
            const gait::Place p = placeNow(g, b.root);
            CHECK(std::hypot(p.x, p.z) < 1.2);
        }
    }
    CHECK(e.seqStatus(b.sequence).cycles >= 1);
    CHECK(walked);
    CHECK(e.warnings().isEmpty());
    const gait::Place end = placeNow(g, b.root);
    CHECK_NEAR(end.x, 0, 1e-3);
    CHECK_NEAR(end.z, 0, 1e-3);
    CHECK_NEAR(std::fabs(gait::angleDiff(end.heading, 0)), 0, 1e-3);
}

TEST(walkPatrolTemplateMakesALap) {
    SceneGraph g;
    CHECK(ProjectSerializer::load(g, MS_SOURCE_DIR "/templates/humanoid_patrol.mstudio"));
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    int root = -1, patrol = -1;
    for (auto& [id, o] : g.allObjects()) if (o.name == "Humanoid" && o.parentId == -1) root = id;
    for (auto& [id, q] : g.allSequences()) if (q.name == "Patrol") patrol = id;
    CHECK(root != -1 && patrol != -1);
    CHECK(gait::canWalk(g, root));
    AutomationEngine e(&g);
    CHECK(e.start(nullptr));
    e.scan(0.016);
    CHECK(e.walks().empty());                     // waits for Start
    e.setInput("Start", 1);
    double far = 0;
    for (int i = 0; i < 12000 && e.seqStatus(patrol).cycles < 1; ++i) {
        e.scan(0.016);
        const gait::Place p = placeNow(g, root);
        far = std::max(far, std::hypot(p.x - 2, p.z - 2));
    }
    CHECK(e.seqStatus(patrol).cycles >= 1);
    CHECK(e.warnings().isEmpty());
    CHECK(far > 2.5);                              // it went round the spots
    const gait::Place end = placeNow(g, root);
    CHECK_NEAR(end.x, 0, 1e-3);
    CHECK_NEAR(end.z, 0, 1e-3);
    std::printf("  a patrol lap in %.1f s\n", e.seqStatus(patrol).lastCycleTime);
}

TEST(walkRejectsWhatItCantDo) {
    SceneGraph g;
    const humanoid::Built b = walkingHumanoid(g);
    humanoid::Spec ps = humanoid::defaultSpec();
    ps.base = humanoid::Base::Pedestal;
    ps.name = "Bust";
    const humanoid::Built pb = humanoid::build(g, ps);
    AutomationEngine e(&g);
    CHECK(e.start(nullptr));
    QString code, err;
    AutomationEngine::MotionRequest r;
    r.kind = AutomationEngine::MotionKind::Walk;
    r.walk.root = pb.root;
    r.walk.forward = 1;
    CHECK(e.submitMotion(r, &code, &err) == 0 && code == "bad_request");
    r.walk.root = b.root;
    r.walk.params.stepLength = 5;
    CHECK(e.submitMotion(r, &code, &err) == 0 && code == "bad_request" && err.contains("at most"));
    r.walk.params.stepLength = 0;
    r.walk.params.speed = 50;   // m/s
    CHECK(e.submitMotion(r, &code, &err) == 0 && err.contains("too fast"));
    r.walk.params.speed = 0;
    r.walk.forward = std::nan("");
    CHECK(e.submitMotion(r, &code, &err) == 0 && code == "bad_request");
    // A walk shares the leg joints: a joint move of a knee waits for it
    r.walk.forward = 0.5;
    const int w = e.submitMotion(r, &code, &err);
    AutomationEngine::MotionRequest knee;
    knee.joints[b.joints.at("LKnee")] = 20;
    const int k = e.submitMotion(knee, &code, &err);
    e.scan(0.016);
    CHECK(e.motionStatus(w).state == AutomationEngine::MotionState::Running);
    CHECK(e.motionStatus(k).state == AutomationEngine::MotionState::Queued);
    CHECK(runMotion(e, k, 30).state == AutomationEngine::MotionState::Done);
    CHECK(e.motionStatus(w).state == AutomationEngine::MotionState::Done);
}

TEST(walkOperationInASequence) {
    SceneGraph g;
    const humanoid::Built b = walkingHumanoid(g);
    const int mark = g.addObject("Spot", PrimitiveType::ImportedMesh);
    g.getObject(mark)->transform.position = {1.2f, 0, 0.6f};
    g.getObject(mark)->transform.rotation = {0, -90, 0};
    Sequence q;
    q.name = "Patrol";
    Operation a;
    a.id = g.allocateId();
    a.type = OperationType::Walk;
    a.walkerId = b.root;
    a.forward = "0.6";
    a.turn = "Angle";
    Operation c;
    c.id = g.allocateId();
    c.type = OperationType::Walk;
    c.walkerId = b.root;
    c.walkToId = mark;
    c.velocity = "0.5";
    c.predecessors = {a.id};
    q.operations = {a, c};
    Signal angle;
    angle.name = "Angle"; angle.kind = SignalKind::Internal; angle.type = SignalType::Real; angle.initialValue = 45;
    g.addSignal(angle);
    const int seq = g.addSequence(q);
    CHECK(AutomationEngine::validateAll(g).isEmpty());
    // Saved and loaded, renamed symbols follow
    SceneGraph loaded;
    roundTrip(g, loaded);
    CHECK(AutomationEngine::validateAll(loaded).isEmpty());
    CHECK(loaded.allSequences().begin()->second.operations[1].walkToId != -1);
    CHECK(loaded.allSequences().begin()->second.operations[0].turn == "Angle");
    g.renameSymbolReferences("Angle", "Heading");
    CHECK(g.getSequence(seq)->operations[0].turn == "Heading");
    g.renameSymbolReferences("Heading", "Angle");

    AutomationEngine e(&g);
    CHECK(e.start(nullptr));
    bool sawWalk = false;
    for (int i = 0; i < 2000 && e.seqStatus(seq).state != AutomationEngine::SeqState::Finished; ++i) {
        e.scan(0.016);
        if (!e.walks().empty()) sawWalk = true;
    }
    CHECK(e.seqStatus(seq).state == AutomationEngine::SeqState::Finished);
    CHECK(sawWalk);
    CHECK(e.warnings().isEmpty());
    const gait::Place p = placeNow(g, b.root);
    CHECK_NEAR(p.x, 1.2, 1e-4);
    CHECK_NEAR(p.z, 0.6, 1e-4);
    CHECK_NEAR(qRadiansToDegrees(p.heading), -90, 1e-3);

    // Nothing to walk, or a robot without legs, is reported
    g.getSequence(seq)->operations[0].forward.clear();
    g.getSequence(seq)->operations[0].turn.clear();
    CHECK(AutomationEngine::validateAll(g).join(" ").contains("nothing to walk"));
    g.getSequence(seq)->operations[0].walkerId = mark;
    CHECK(AutomationEngine::validateAll(g).join(" ").contains("no legs"));
}

TEST(takePlaysBackAWalk) {
    SceneGraph g;
    const humanoid::Built b = walkingHumanoid(g);
    AutomationEngine e(&g);
    CHECK(e.start(nullptr));
    std::vector<int> joints;
    for (auto& [name, jid] : b.joints) joints.push_back(jid);
    take::Recorder rec(g, b.root, joints);
    AutomationEngine::MotionRequest r;
    r.kind = AutomationEngine::MotionKind::Walk;
    r.walk.root = b.root;
    r.walk.forward = 0.6;
    r.walk.turn = 45;
    CHECK(runMotion(e, e.submitMotion(r), 30, [&] { rec.sample(e); }).state == AutomationEngine::MotionState::Done);
    const take::Take& t = rec.take();
    const gait::Place walked = placeNow(g, b.root);

    // A take as a play request: absolute frames start where it started
    auto request = [&](bool relative) {
        auto p = std::make_shared<AutomationEngine::PlayRequest>();
        p->root = b.root;
        p->joints = joints;
        p->relative = relative;
        for (const auto& row : t.rows) {
            p->times.push_back(row[0]);
            Transform f;
            f.position = QVector3D(float(row[1] / 1000), float(row[2] / 1000), float(row[3] / 1000));
            f.rotation = QVector3D(float(row[4]), float(row[5]), float(row[6]));
            p->frames.push_back(f);
            std::vector<double> v;
            for (size_t j = 0; j < joints.size(); ++j) v.push_back(row[15 + j]);
            p->values.push_back(v);
        }
        AutomationEngine::MotionRequest m;
        m.kind = AutomationEngine::MotionKind::Play;
        m.play = p;
        return m;
    };
    // Relative: the same walk again from where it stands
    take::Recorder again(g, b.root, joints);
    CHECK(runMotion(e, e.submitMotion(request(true)), 30, [&] { again.sample(e); }).state == AutomationEngine::MotionState::Done);
    gait::Place end = placeNow(g, b.root);
    CHECK_NEAR(gait::angleDiff(end.heading, walked.heading), qDegreesToRadians(45.0), 1e-4);
    // the second walk is the first one turned by 45° and moved to `walked`
    const double dx = walked.x, dz = walked.z, h = walked.heading;
    CHECK_NEAR(end.x, walked.x + dx * std::cos(h) + dz * std::sin(h), 1e-4);
    CHECK_NEAR(end.z, walked.z - dx * std::sin(h) + dz * std::cos(h), 1e-4);
    CHECK(std::abs(again.frames() - rec.frames()) <= 2);
    // the joints went the same way, scan by scan
    double worst = 0;
    for (int i = 1; i + 1 < std::min(again.frames(), rec.frames()); ++i)
        for (size_t j = 0; j < joints.size(); ++j) {
            const double a = again.take().rows[size_t(i)][15 + j];
            double best = 1e9;
            for (int k = i - 1; k <= i + 1; ++k) best = std::min(best, std::abs(a - t.rows[size_t(k)][15 + j]));
            worst = std::max(worst, best);
        }
    CHECK(worst < 1e-3);
    for (size_t j = 0; j < joints.size(); ++j) CHECK_NEAR(g.getJoint(joints[j])->currentValue, t.rows.back()[15 + j], 1e-3);

    // Absolute: back to where the take was recorded
    CHECK(runMotion(e, e.submitMotion(request(false)), 30).state == AutomationEngine::MotionState::Done);
    end = placeNow(g, b.root);
    CHECK_NEAR(end.x, walked.x, 1e-4);
    CHECK_NEAR(end.z, walked.z, 1e-4);
    CHECK_NEAR(gait::angleDiff(end.heading, walked.heading), 0, 1e-4);

    // Joints only, at twice the pace; a bad request is refused
    auto half = std::make_shared<AutomationEngine::PlayRequest>(*request(false).play);
    half->frames.clear();
    half->speed = 2;
    AutomationEngine::MotionRequest m;
    m.kind = AutomationEngine::MotionKind::Play;
    m.play = half;
    int scans = 0;
    CHECK(runMotion(e, e.submitMotion(m), 30, [&] { ++scans; }).state == AutomationEngine::MotionState::Done);
    CHECK(std::abs(scans - rec.frames() / 2) <= 2);
    end = placeNow(g, b.root);
    CHECK_NEAR(end.x, walked.x, 1e-4);
    half->times[3] = half->times[2];
    QString code, err;
    CHECK(e.submitMotion(m, &code, &err) == 0 && code == "bad_request");
}

TEST(takeRecordsAWalk) {
    SceneGraph g;
    const humanoid::Built b = walkingHumanoid(g);
    AutomationEngine e(&g);
    CHECK(e.start(nullptr));
    std::vector<int> joints;
    for (auto& [name, jid] : b.joints) joints.push_back(jid);
    take::Recorder every(g, b.root, joints), tenth(g, b.root, joints, 10);
    const take::Take& t = every.take();
    CHECK(t.column("t") == 0 && t.column("x") == 1 && t.column("heading") == 7);
    CHECK(t.column("goal_heading") == 14);
    CHECK(t.columns.size() == 15 + int(joints.size()));
    CHECK(t.column(g.motorForJoint(joints.front())->name) == 15);

    AutomationEngine::MotionRequest r;
    r.kind = AutomationEngine::MotionKind::Walk;
    r.walk.root = b.root;
    r.walk.forward = 1.0;
    const int id = e.submitMotion(r);
    const auto st = runMotion(e, id, 30, [&] { every.sample(e); tenth.sample(e); });
    CHECK(st.state == AutomationEngine::MotionState::Done);
    for (int i = 0; i < 20; ++i) { e.scan(0.016); every.sample(e); tenth.sample(e); }   // standing at the end

    CHECK_NEAR(t.rate, 62.5, 1e-9);
    CHECK(every.frames() > 100);
    CHECK_NEAR(t.duration(), 0.016 * (every.frames() - 1), 1e-6);
    CHECK_NEAR(tenth.take().duration(), t.duration(), 0.1);
    CHECK(std::abs(tenth.frames() - int(t.duration() * 10) - 1) <= 1);
    const int walking = t.column("walking"), step = t.column("step"), swing = t.column("swing");
    int walked = 0, maxStep = -1;
    std::set<int> sides;
    double lastT = -1;
    for (const auto& row : t.rows) {
        CHECK(row.size() == size_t(t.columns.size()));
        CHECK(row[0] > lastT);
        lastT = row[0];
        if (row[size_t(walking)] > 0) {
            ++walked;
            CHECK_NEAR(row[size_t(t.column("goal_x"))], 1000, 0.1);
            CHECK_NEAR(row[size_t(t.column("goal_z"))], 0, 0.1);
        }
        maxStep = std::max(maxStep, int(row[size_t(step)]));
        if (row[size_t(swing)] >= 0) sides.insert(int(row[size_t(swing)]));
        const double p = row[size_t(t.column("progress"))];
        CHECK(p >= 0 && p < 1);
    }
    CHECK(walked > 50);
    CHECK(sides.size() == 2);
    CHECK(maxStep >= 2);
    const auto& last = t.rows.back();
    CHECK_NEAR(last[1], 1000, 0.1);
    CHECK_NEAR(last[3], 0, 0.1);
    CHECK(last[size_t(walking)] == 0 && last[size_t(step)] == -1);
    CHECK_NEAR(last[size_t(t.column("goal_x"))], 1000, 0.1);   // standing: its own place

    const QString csv = t.toCsv();
    const QStringList lines = csv.split('\n', Qt::SkipEmptyParts);
    CHECK(lines.size() == every.frames() + 5);
    CHECK(lines[0] == "# MotionStudio take" && lines[2] == "# rate: 62.5");
    CHECK(lines[4].startsWith("t,x,y,z,rx,ry,rz,heading,walking"));
    CHECK(lines[5].split(',').size() == t.columns.size());
    QTemporaryDir dir;
    CHECK(t.save(dir.filePath("walk.csv")));
    QFile f(dir.filePath("walk.csv"));
    CHECK(f.open(QIODevice::ReadOnly) && QString::fromUtf8(f.readAll()) == csv);
}

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
