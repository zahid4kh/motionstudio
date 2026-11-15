// Starts motions without waiting, waits for them, and stops one half-way.
#include <motionstudio/motionstudio.hpp>
#include <iostream>

namespace ms = motionstudio;

int main() try {
    auto studio = ms::Studio::connect();
    auto robot = studio.robot();
    const std::string a1 = robot.jointNames().front();

    ms::Motion turn = robot.moveJoints({{a1, 45}}, {.speed = 40, .wait = false});
    ms::Motion lift = robot.moveBy(ms::Target().y(50), {.wait = false});   // queued after the turn
    lift.wait();
    std::cout << "turn " << turn.state() << ", lift " << lift.state() << "\n";

    robot.moveJoints({{a1, -45}}, {.wait = false});
    studio.sleep(0.5);                                 // simulation seconds
    std::cout << "stopped " << robot.stop() << " at " << robot.joints()[a1] << "\n";
} catch (const ms::Error& e) {
    std::cerr << "MotionStudio: " << e.what() << " (" << e.code() << ")\n";
    return 1;
}
