// Live test against motionstudio-server (C++17, so the library works without
// C++20). tests/run.sh starts the server and runs this:
//   live_test <port>
#include <motionstudio/motionstudio.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace ms = motionstudio;

static int failures = 0, checks = 0;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
#define NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

template <class E, class F> bool throws(F f) {
    try { f(); } catch (const E&) { return true; } catch (...) { return false; }
    return false;
}

int main(int argc, char** argv) {
    const int port = argc > 1 ? std::atoi(argv[1]) : ms::kDefaultPort;
    ms::Studio studio = ms::Studio::connect("localhost", port);
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
