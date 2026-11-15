# MotionStudio C++ API

Control MotionStudio robots and stations from C++17. It has no dependencies beyond the standard library and sockets, and works on Linux, macOS and Windows.

```cmake
add_subdirectory(path/to/MotionStudio/api/cpp)      # or FetchContent / install + find_package(motionstudio)
target_link_libraries(my_app PRIVATE motionstudio::motionstudio)
```

```cpp
#include <motionstudio/motionstudio.hpp>
namespace ms = motionstudio;

int main() {
    auto studio = ms::Studio::connect();                 // localhost:5800
    auto robot = studio.robot("Robot");
    std::cout << robot.pose() << "\n";
    robot.moveL(ms::Target().z(100));                    // straight line, only z changes
    robot.moveBy(ms::Target().y(-50), {.speed = 100});   // C++20; in C++17: {100}
    robot.moveJoints({{"A1", 30}, {"A2", -10}});
    robot.home();
}
```

Units are mm, degrees and seconds, and Y is up. See [../README.md](../README.md) and [../PROTOCOL.md](../PROTOCOL.md).

## Reference

| `Studio` | |
|---|---|
| `Studio::connect(host, port)` | Copies share the connection; it closes with the last one |
| `robots()`, `robot(name = "")`, `actuator(name)` | |
| `get(signal)`, `set(signal, double \| bool)`, `signals()` | Signals |
| `waitUntil(condition, timeout = -1)` | Waits for an expression |
| `play()`, `pause()`, `reset()`, `sleep(s)`, `time()`, `running()`, `setSpeed(f)` | Simulation |

| `Robot` | |
|---|---|
| `name()`, `path()`, `jointNames()`, `toolNames()` | |
| `joints()`, `jointInfo()`, `poses()` | |
| `moveJoints(map, opts)`, `moveJointsBy(map, opts)` | Joint moves (speed in %) |
| `moveToPose(name, opts)`, `home(opts)`, `stop()` | |
| `tool(name = "", base = "")` | → `Tool` |
| `pose()`, `moveL()`, `moveJ()`, `moveBy()` | The only tool's methods |

| `Tool` | |
|---|---|
| `pose(frame = "base")` | → `Pose` |
| `moveL(Target, opts)` | Straight line (speed in mm/s, default 250) |
| `moveJ(Target, opts)` | Joint move to the target (speed in %) |
| `moveBy(Target, opts)`, `moveJBy(Target, opts)` | Relative; `opts.frame = "tool"` for the tool's own axes |

- `Target`: `ms::Target().x(…).z(…)`, or any `Pose`. In an absolute move, the values left out stay as they are.
- `MoveOptions{speed = 0, frame = "base", wait = true}`.
- `Motion`: `wait(timeout)`, `cancel()`, `state()`, `done()`, `progress()`.
- `Actuator`: `value()`, `moveTo()`, `moveBy()`, `run(speed)`, `stop()`.

Errors are `ms::Error` (`code()`, `what()`), with the subclasses `NotFound`, `OutOfReach`, `OutOfLimits`, `MotionCancelled`, `Timeout`, `SimulationError` and `ConnectionError`. Calls are thread-safe.

## Build and test

```sh
cmake -B build && cmake --build build      # library, examples (square, two_motions, humanoid_wave), live_test
./tests/run.sh                             # starts motionstudio-server and runs the live test
```
