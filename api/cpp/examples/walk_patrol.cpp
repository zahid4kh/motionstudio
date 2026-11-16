// A humanoid patrols through waypoints while the program watches it, and
// stops gracefully (finishing its step) after two laps.
//
// Run with a humanoid on legs (Tools > Humanoid Builder) or:
//   motionstudio-server --humanoid "Full-size"
#include <motionstudio/motionstudio.hpp>
#include <iostream>

namespace ms = motionstudio;

int main() {
    auto studio = ms::Studio::connect();
    auto walker = studio.humanoid();
    const std::vector<ms::Point> waypoints{{1500, 0}, {1500, -1200}, {0, -1200}, {0, 0}};
    ms::WalkOptions options{.gait = {.stepLength = 280, .speed = 450}, .wait = false};

    for (int lap = 1; lap <= 3; ++lap) {
        ms::Motion walk = walker.walkPath(waypoints, 0.0, options);
        while (!walk.done()) {
            const ms::WalkState s = walker.state();
            std::cout << "lap " << lap << ": step " << s.step << "/" << s.steps << " at " << s.place << "\n";
            studio.sleep(0.5);
            if (lap == 3 && s.step > s.steps / 2) {     // half way round the third lap: stop
                walker.stopWalking();
                std::cout << "stopping\n";
                break;
            }
        }
        walk.wait();
    }
    std::cout << "stands at " << walker.place() << "\n";
}
