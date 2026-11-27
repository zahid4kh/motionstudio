// Live test against motionstudio-server (C++17, so the library works without
// C++20). tests/run.sh starts the server and runs this:
//   live_test <port>
#include <motionstudio/motionstudio.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

namespace ms = motionstudio;

static int failures = 0, checks = 0;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
#define NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

template <class E, class F> bool throws(F f) {
    try { f(); } catch (const E&) { return true; } catch (...) { return false; }
    return false;
}

// Indexing table: Press is a slide (mm) and Table turns (degrees).
// The expression symbol Press.pos stays in metres, and only once the simulation runs.
static int actuators(ms::Studio& studio) {
    ms::Actuator press = studio.actuator("Press");
    ms::Actuator table = studio.actuator("Table");
    NEAR(press.value(), 0, 1e-6);
    CHECK(throws<ms::SimulationError>([&] { studio.get("Press.pos"); }));
    press.moveTo(250);
    table.moveTo(90);
    NEAR(press.value(), 250, 1e-3);
    NEAR(studio.get("Press.pos"), 0.25, 1e-6);
    NEAR(table.value(), 90, 1e-3);
    NEAR(studio.get("Table.pos"), 90, 1e-3);
    CHECK(throws<ms::NotFound>([&] { studio.actuator("Nobody").value(); }));
    std::printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

// A humanoid on legs: walks, turns, paths, a stop that finishes the step
static bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
static bool sameWay(double a, double b) { return near(std::fmod(a - b + 540.0, 360.0) - 180.0, 0, 0.01); }

static int walking(ms::Studio& studio) {
    studio.reset();
    ms::Humanoid h = studio.humanoid();
    CHECK(h.walks());
    const ms::Gait g = h.defaultGait();
    CHECK(g.stepLength && *g.stepLength > 100);
    ms::Place p = h.place();
    NEAR(p.x, 0, 1e-6); NEAR(p.z, 0, 1e-6);

    h.walk(600);
    p = h.place();
    NEAR(p.x, 600, 0.5); NEAR(p.z, 0, 0.5);
    h.turn(90);
    CHECK(sameWay(h.place().heading, 90));
    ms::WalkOptions quick;
    quick.gait.speed = 450;
    quick.gait.armSwing = 0;
    h.walk(400, 0, 0, quick);                       // facing -Z now
    p = h.place();
    NEAR(p.x, 600, 0.5); NEAR(p.z, -400, 0.5);

    h.walkPath({{600, 400}, {0, 400}, {0, 0}}, 0.0);
    p = h.place();
    NEAR(p.x, 0, 0.5); NEAR(p.z, 0, 0.5);
    CHECK(sameWay(p.heading, 0));
    ms::WalkOptions robotFrame;
    robotFrame.frame = "robot";
    h.walkTo(300, 300, 180.0, robotFrame);         // x ahead, z to its right
    p = h.place();
    NEAR(p.x, 300, 0.5); NEAR(p.z, 300, 0.5);
    CHECK(sameWay(p.heading, 180));

    // Without waiting, then a stop that finishes the step under way
    ms::WalkOptions noWait;
    noWait.wait = false;
    ms::Motion m = h.walk(4000, 0, 0, noWait);
    studio.sleep(1.2);
    const ms::WalkState st = h.state();
    CHECK(st.walking && st.motion == m.id() && st.steps > 5);
    CHECK(h.stopWalking() == 1);
    m.wait();
    CHECK(m.state() == "done");
    CHECK(!h.walking());

    ms::WalkOptions tooLong;
    tooLong.gait.stepLength = 5000;
    CHECK(throws<ms::Error>([&] { h.walk(500, 0, 0, tooLong); }));
    std::printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

// An empty server: build a line (belt, part source, beam, ram, sink, a
// sequence), check it, run it, save and open it (C++17: options set by field)
static int building(ms::Studio& studio) {
    studio.reset();
    ms::Scene scene = studio.scene();
    scene.newScene();

    ms::ObjectOptions o;
    o.position = ms::Target().x(1000);
    ms::SceneObject cell = scene.addFrame("Cell", o);
    o = {};
    o.parent = cell.ref();
    o.position = ms::Target().y(700);
    o.color = "#336699";
    ms::SceneObject table = scene.addBox("Table", {800, 40, 600}, o);
    CHECK(table.path() == "Cell/Table");
    NEAR(table.world().x, 1000, 1e-3); NEAR(table.world().y, 700, 1e-3);
    const ms::Box b = table.bounds();
    NEAR(b.min.x, 600, 1e-2); NEAR(b.size.z, 600, 1e-2);
    CHECK(table.size() && std::fabs(table.size()->y - 40) < 1e-3);

    // A handle follows its object through a rename and a move
    o = {};
    o.parent = cell.ref();
    o.position = ms::Target().x(300).y(800);
    ms::SceneObject arm = scene.addFrame("Arm", o);
    arm.rename("Arm A").setParent("");
    CHECK(arm.path() == "Arm A");
    NEAR(arm.world().x, 1300, 1e-3);
    arm.setParent(cell.ref());
    CHECK(arm.path() == "Cell/Arm A" && cell.children().size() == 2);

    ms::JointOptions jo;
    jo.pivot = ms::Vec3{1000, 800, 0};
    jo.min = -170; jo.max = 170;
    jo.actuator = ms::ActuatorOptions{};
    jo.actuator->speed = 90;
    ms::Joint turn = scene.addJoint("Turn", cell.ref(), arm.ref(), {0, 1, 0}, jo);
    CHECK(turn.details().actuator == "Turn");
    turn.setValue(90);
    NEAR(arm.world().z, -300, 1e-2); NEAR(arm.world().x, 1000, 1e-2);
    CHECK(throws<ms::NotFound>([&] { scene.object("Nothing"); }));
    CHECK(throws<ms::Error>([&] { scene.addJoint("Bad", cell.ref(), arm.ref(), {0, 1, 0}); }));
    turn.remove();

    // The line
    scene.newScene();
    ms::SceneObject line = scene.addFrame("Line");
    o = {};
    o.parent = line.ref();
    o.position = ms::Target().y(450);
    ms::SceneObject belt = scene.addBox("Belt", {2000, 100, 400}, o);
    belt.setBelt("Run ? 0.3 : 0", ms::Vec3{1, 0, 0});
    o = {};
    o.position = ms::Target().x(-900).y(551);
    ms::SceneObject part = scene.addBox("Box", {100, 100, 100}, o);
    ms::BodyOptions body;
    body.mass = 0.5;
    part.setBody(body);
    o = {};
    o.parent = line.ref();
    o.position = ms::Target().x(1200);
    ms::SceneObject stand = scene.addFrame("Stand", o);
    o = {};
    o.parent = stand.ref();
    o.position = ms::Target().y(300);
    ms::SceneObject ram = scene.addBox("Ram", {50, 50, 50}, o);
    jo = {};
    jo.type = "prismatic";
    jo.min = 0; jo.max = 150;
    jo.actuator = ms::ActuatorOptions{};
    jo.actuator->speed = 300;
    scene.addJoint("Ram", stand.ref(), ram.ref(), {0, 1, 0}, jo);
    ms::SignalOptions run;
    run.kind = "input";
    run.value = 1;
    scene.addSignal("Run", run);
    ms::SignalOptions seen;
    seen.type = "int";
    scene.addSignal("Seen", seen);
    ms::SensorOptions beam;
    beam.mount = belt.ref();
    beam.frame = "world";
    beam.at = ms::Vec3{600, 560, -300};
    beam.direction = ms::Vec3{0, 0, 1};
    beam.range = 600;
    beam.detects = "bodies";
    scene.addSensor("AtEnd", "beam", beam);
    scene.addPose("Up", {{"Ram", 150}});
    ms::SourceOptions feed;
    feed.interval = 3;
    feed.max = 3;
    scene.addSource("Feeder", part.ref(), feed);
    ms::SinkOptions out;
    out.mount = belt.ref();
    out.at = ms::Vec3{950, 100, 0};
    out.size = ms::Vec3{100, 200, 400};
    scene.addSink("Out", out);
    ms::Ops ops;
    ms::Ops::Common afterCount;
    afterCount.after = {"Count"};
    ops.waitFor("Part", "AtEnd")
       .set("Count", "Seen = Seen + 1")
       .moveToPose("Raise", "Up")
       .move("Lower", "Ram", 0, ms::Value(600))
       .waitFor("Gone", "!AtEnd", afterCount);
    ms::SequenceOptions so;
    so.loop = true;
    so.enable = "Run";
    scene.addSequence("Counter", ops, so);
    CHECK(scene.sequenceJson("Counter").find("\"speed\":\"0.6\"") != std::string::npos);
    CHECK(scene.check().empty());
    for (const auto& p : scene.check()) std::printf("  problem: %s\n", p.c_str());

    const std::string path = scene.save("cpp_line_test.mstudio");
    scene.newScene();
    CHECK(scene.objects().empty());
    const ms::SceneInfo info = scene.open(path);
    CHECK(info.sequences.size() == 1 && info.sources.size() == 1 && info.sensors.size() == 1);
    std::remove(path.c_str());

    studio.sleep(12);
    CHECK(studio.get("Seen") >= 2);
    CHECK(studio.get("Out.count") >= 2);
    CHECK(throws<ms::SimulationError>([&] { scene.addFrame("Late"); }));
    studio.reset();
    CHECK(scene.remove("Seen") == "signal");
    std::printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

int main(int argc, char** argv) {
    const int port = argc > 1 ? std::atoi(argv[1]) : ms::kDefaultPort;
    ms::Studio studio = ms::Studio::connect("localhost", port);
    if (argc > 2 && std::string(argv[2]) == "actuators") return actuators(studio);
    if (argc > 2 && std::string(argv[2]) == "walking") return walking(studio);
    if (argc > 2 && std::string(argv[2]) == "building") return building(studio);
    studio.reset();
    ms::Robot robot = studio.robot();
    CHECK(robot.name() == "Robot");
    CHECK(robot.toolNames().size() == 1 && robot.toolNames()[0] == "TCP");
    CHECK(robot.jointInfo().size() >= 6);

    // Straight lines
    const ms::Pose start = robot.pose();
    ms::MoveOptions fast;
    fast.speed = 400;
    robot.moveL(ms::Target().z(100), fast);
    ms::Pose p = robot.pose();
    NEAR(p.z, 100, 0.05); NEAR(p.x, start.x, 0.05); NEAR(p.y, start.y, 0.05);
    robot.moveBy(ms::Target().y(-80));
    NEAR(robot.pose().y, start.y - 80, 0.05);
    robot.moveL(start);
    NEAR(robot.pose().z, start.z, 0.05);
    CHECK(studio.running());

    // Joints
    robot.moveJoints({{"A1", 30}, {"A2", -10}});
    auto j = robot.joints();
    NEAR(j["A1"], 30, 1e-3); NEAR(j["A2"], -10, 1e-3);
    robot.moveJointsBy({{"A1", 10}});
    NEAR(robot.joints()["A1"], 40, 1e-3);
    robot.home();
    NEAR(robot.joints()["A1"], 0, 1e-3);

    // Motions without waiting; queueing on shared joints
    ms::MoveOptions slow;
    slow.speed = 30;
    slow.wait = false;
    ms::Motion a = robot.moveJoints({{"A1", 40}}, slow);
    ms::MoveOptions later;
    later.wait = false;
    ms::Motion b = robot.moveJoints({{"A1", 0}}, later);
    b.wait();
    CHECK(a.state() == "done" && b.done());

    // Threads share the connection
    std::thread t1([&] { robot.moveJoints({{"A6", 20}}); });
    std::thread t2([&] { studio.sleep(0.2); });
    t1.join(); t2.join();
    NEAR(robot.joints()["A6"], 20, 1e-3);

    // Errors
    CHECK(throws<ms::OutOfReach>([&] { robot.moveL(ms::Target().x(5000)); }));
    CHECK(throws<ms::OutOfLimits>([&] { robot.moveJoints({{"A2", 999}}); }));
    CHECK(throws<ms::NotFound>([&] { robot.moveJoints({{"A9", 1}}); }));
    CHECK(throws<ms::NotFound>([&] { studio.robot("Nobody"); }));
    CHECK(throws<ms::MotionCancelled>([&] {
        ms::MoveOptions o; o.speed = 5; o.wait = false;
        ms::Motion m = robot.moveJoints({{"A1", -170}}, o);
        m.cancel();
        m.wait();
    }));
    CHECK(throws<ms::Timeout>([&] {
        ms::MoveOptions o; o.speed = 5; o.wait = false;
        robot.moveJoints({{"A1", 170}}, o).wait(0.2);
    }));
    robot.stop();

    // Signals
    studio.set("Grip", true);
    studio.waitUntil("Grip", 2);
    CHECK(studio.get("Grip") == 1);
    studio.set("Grip", false);
    CHECK(studio.sleep(0.3) > 0);
    CHECK(studio.get("Grip") == 0);
    CHECK(!studio.signals().empty());
    CHECK(throws<ms::Timeout>([&] { studio.waitUntil("Grip", 0.2); }));

    std::printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
