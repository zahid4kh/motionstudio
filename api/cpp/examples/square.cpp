// Draws a 200 mm square with the tool in straight lines, then goes home.
// Run MotionStudio with a robot and the Robot API on, or:
//   motionstudio-server --robot "6-axis 0.9"
#include <motionstudio/motionstudio.hpp>
#include <iostream>

namespace ms = motionstudio;

int main() try {
    auto studio = ms::Studio::connect();
    auto robot = studio.robot();                       // the scene's only robot
    const ms::Pose start = robot.pose();
    std::cout << robot.name() << " tool at " << start << "\n";

    robot.moveL(start.offset(0, -150), {.speed = 300});    // 150 mm lower
    for (auto [dx, dz] : {std::pair{0, 200}, {-200, 0}, {0, -200}, {200, 0}})
        robot.moveBy(ms::Target().x(dx).z(dz), {.speed = 200});
    robot.moveL(start);
    robot.home();
    std::cout << "done at " << studio.time() << " s\n";
} catch (const ms::Error& e) {
    std::cerr << "MotionStudio: " << e.what() << " (" << e.code() << ")\n";
    return 1;
}
