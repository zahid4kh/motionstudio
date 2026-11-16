// Walks a humanoid round a square, then once more as one path.
//
// Run with a humanoid on legs (Tools > Humanoid Builder) or:
//   motionstudio-server --humanoid "Full-size"
#include <motionstudio/motionstudio.hpp>
#include <iostream>

namespace ms = motionstudio;

int main() {
    auto studio = ms::Studio::connect();
    auto walker = studio.humanoid();               // the only robot with legs
    const ms::Gait gait = walker.defaultGait();
    std::cout << walker.name() << " steps " << *gait.stepLength << " mm at " << *gait.speed << " mm/s\n";

    // One side at a time: walk, then a quarter turn to the left
    for (int side = 0; side < 4; ++side) {
        walker.walk(1000);                         // mm ahead
        walker.turn(90);                           // degrees to its left
    }
    std::cout << "back at " << walker.place() << "\n";

    // The same square as one path, in its own frame (x ahead, z to its right), faster
    walker.walkPath({{1000, 0}, {1000, -1000}, {0, -1000}, {0, 0}}, 0.0,
                    {.gait = {.speed = 500, .armSwing = 0}, .frame = "robot"});
    std::cout << "and again, at " << walker.place() << "\n";
}
