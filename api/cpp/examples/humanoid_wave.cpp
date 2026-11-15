// Waves a humanoid's right hand and moves its left hand in straight lines.
//   motionstudio-server --humanoid "Full-size"
#include <motionstudio/motionstudio.hpp>
#include <iostream>

namespace ms = motionstudio;

int main() try {
    auto studio = ms::Studio::connect();
    auto humanoid = studio.robot("Humanoid");
    humanoid.home();
    humanoid.moveJoints({{"RShoulderRoll", 100}, {"RElbow", 60}}, {.speed = 60});
    for (int i = 0; i < 3; ++i) {
        humanoid.moveJoints({{"RShoulderYaw", 25}}, {.speed = 80});
        humanoid.moveJoints({{"RShoulderYaw", -25}}, {.speed = 80});
    }
    // The left hand as a tool, moved by its arm only (not the waist)
    auto hand = humanoid.tool("Left Hand TCP", "Torso");
    humanoid.moveJoints({{"LShoulderPitch", 40}, {"LElbow", 70}});   // a straight arm is at full reach
    hand.moveBy(ms::Target().x(60), {.speed = 150});
    hand.moveBy(ms::Target().z(-60), {.speed = 150});
    std::cout << "left hand at " << hand.pose() << "\n";
    humanoid.home();
} catch (const ms::Error& e) {
    std::cerr << "MotionStudio: " << e.what() << " (" << e.code() << ")\n";
    return 1;
}
