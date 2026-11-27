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
| `humanoid(name = "")` | A robot with legs → `Humanoid`; without a name the only one |
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

| `Humanoid` (a `Robot` that walks) | |
|---|---|
| `walk(forward, sideways = 0, turn = 0, WalkOptions)` | mm ahead, mm to its left, degrees to its left; a turn with a distance makes an arc |
| `turn(degrees, opts)`, `sideStep(mm, opts)` | On the spot / sideways |
| `walkTo(x, z, heading = nullopt, opts)` | To a spot on the floor (mm), then face `heading` |
| `walkPath({{x, z}, …}, heading = nullopt, opts)` | Through points; `opts.frame = "robot"`: x ahead, z to its right |
| `stopWalking()` | Finishes the step under way, feet together |
| `state()` → `WalkState{place, walking, motion, step, steps, gait}` | Also `place()`, `walking()`, `defaultGait()` |

- `WalkOptions{gait, frame = "world", wait = true}`; `Gait{stepLength, sideStep, turnPerStep, stepHeight, stepTime, speed, crouch, sway, armSwing}` (optionals; mm, degrees, s, mm/s). C++20: `walker.walk(2000, 0, 0, {.gait = {.speed = 500}})`.
- `Place{x, z, heading}`: where it stands; heading 0 faces +X, positive to its left. `Point{x, z}` for paths.

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

Errors are `ms::Error` (`code()`, `what()`), with the subclasses `NotFound`, `OutOfReach`, `OutOfLimits`, `MotionCancelled`, `Timeout`, `SimulationError`, `UnsavedChanges` and `ConnectionError`. Calls are thread-safe.

## Building scenes (`studio.scene()`)

```cpp
ms::Scene scene = studio.scene();
studio.reset();                                          // changes need the simulation reset
ms::SceneObject station = scene.importFile("station.step");   // Z-up CAD stood upright
ms::ObjectOptions o;
o.parent = station.ref();
ms::SceneObject arm = scene.addFrame("Arm", o);
scene.object("arm1").setParent(arm.ref());               // keeps where it is (and its joint)
ms::JointOptions j;
j.frame = station.ref();
j.min = -170; j.max = 170;
scene.addJoint("J1", station.ref(), arm.ref(), {0, 0, 1}, j);
scene.addSequence("Cycle", ms::Ops().move("Out", "J1", 90).wait("Hold", 1).move("Back", "J1", 0));
scene.save("station.mstudio");
```

- `Scene`: `newScene`, `open`, `save`, `info`, `check`, `importFile`, `objects`, `object`, `exists`, `add`, `addFrame`, `addBox`, `addCylinder`, `group`, `addJoint`, `joint`, `addSignal`, `addSensor`, `addPose`, `addSequence`, `sequenceJson`, `addSource`, `addSink`, `remove`.
- `SceneObject` (found by `ref()`, `"#id"`, so it survives renames and moves): `path`, `name`, `type`, `position`, `world`, `poseIn`, `size`, `children`, `set(ObjectOptions)`, `moveTo`, `rename`, `setParent("" for the top)`, `add`, `duplicate`, `remove`, `bounds` → `Box`, `features` → `Feature`s, `setBody`, `setBelt`.
- `Joint`: `details()`, `value()`, `setValue`, `setRange`, `remove`.
- `Ops`: `move`, `moveToPose`, `run`, `stop`, `set`, `wait`, `waitFor`, `grip`, `release`, `moveL`, `raw(json)`, each with `OpOptions` (`after`, `first`, `onStart`, `onFinish`).
- `ms::Value` is a number (mm, mm/s, degrees) or an expression string in the automation's units (m, m/s).

## Build and test

```sh
cmake -B build && cmake --build build      # library, examples (square, two_motions, humanoid_wave, walk_square, walk_patrol), live_test
./tests/run.sh                             # starts motionstudio-server and runs the live tests (moves, actuators, walking, building)
```
