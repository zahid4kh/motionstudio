# Robot API (code)

Control robots and stations from your own programs, in **Python**, **C++** or **Kotlin**. A program can move a robot's tool in straight lines, make joint moves, go to poses you taught, walk a humanoid along a path, and read and write signals. You can test a robot program here before it runs a real machine, or let outside software drive the model.

## Switching it on

**Tools › Robot API…** runs the server that programs connect to. It is on by default, on port **5800**, for programs on this computer only. The window lists the robots in the scene with their joints and tools, and shows a quick start for this scene in each language (**Copy Code**). While programs are connected, the status bar shows **API :5800 · 1 program**.

The libraries are in the **api/** folder of the MotionStudio source:

| Language | Folder | Use it |
|---|---|---|
| Python 3.8+ | `api/python` | `pip install api/python` |
| C++17 | `api/cpp` | `add_subdirectory(api/cpp)`, link `motionstudio::motionstudio` |
| Kotlin / JVM | `api/kotlin` | from Maven Central: `implementation("io.github.zahid4kh:motionstudio:0.2.0")` |

None of them needs anything else. To run programs without the app window (on a server, or in automated tests), use `motionstudio-server project.mstudio`. It can also build a robot from a preset: `motionstudio-server --robot "6-axis 0.9"`, or `--humanoid "Full-size"`.

## Units and frames

- Lengths are **millimetres**, angles are **degrees**, times are **seconds**.
- **Y is up**, as everywhere in MotionStudio.
- A pose is `x, y, z` plus a rotation `rx, ry, rz` (Euler XYZ, like an object's rotation in the Properties panel).
- Cartesian targets are in the robot's **base frame** by default: the frame of the robot's top object. So a program keeps working when you move the robot in the scene. Use `frame="world"` for the scene's frame, or an object's name for that object's frame (a fixture or a pallet, say).
- Joint values are degrees for turning joints and millimetres for sliding joints. Joints are named after their **actuators** (`A1`, `J2`, `LShoulderPitch`).

## Connecting and finding robots

```python
import motionstudio as ms

studio = ms.connect()                 # localhost:5800
robot = studio.robot("Robot")         # by name, or path: "Cell/Robot"
print(robot.joint_names, robot.tool_names)
```

```cpp
auto studio = ms::Studio::connect();
auto robot = studio.robot("Robot");
```

```kotlin
val studio = MotionStudio.connect()
val robot = studio.robot("Robot")
```

A **robot** is any machine in the scene: the actuated joints below one object, such as a robot arm, a humanoid, a gantry or a turntable. `studio.robot()` without a name returns the scene's only robot. A robot's **tool** is its frame named **TCP**, so `robot.move_l(...)` moves that frame. A humanoid has two tools, so pick one: `robot.tool("Right Hand TCP")`.

## Straight lines and joint moves

```python
robot.move_l(z=100)                   # straight line; values you leave out stay as they are
robot.move_l(ms.Pose(500, 300, 100, 0, 0, 0), speed=200)    # mm/s (default 250)
robot.move_by(y=-50)                  # 50 mm down from where it is
robot.move_by(y=20, frame="tool")     # along the tool's own Y axis
robot.move_j(x=400)                   # to a pose with a joint move: faster, not straight
robot.move_joints(A1=30, A2=-10, speed=50)   # % of the joints' speed (default 100)
robot.move_to_pose("Pick")            # a pose taught in Actuators & Poses
robot.home()                          # the pose "<robot> Home", or "Home"
print(robot.pose(), robot.joints)
```

```cpp
robot.moveL(ms::Target().z(100));
robot.moveL(ms::Pose{500, 300, 100}, {.speed = 200});
robot.moveBy(ms::Target().y(-50));
robot.moveJoints({{"A1", 30}, {"A2", -10}}, {.speed = 50});
robot.home();
```

```kotlin
robot.moveL(z = 100)
robot.moveL(Pose(500, 300, 100), speed = 200)
robot.moveBy(y = -50)
robot.moveJoints("A1" to 30, "A2" to -10, speed = 50)
robot.home()
```

A straight-line move goes along a line, with the tool's orientation turning evenly along the way. It eases in and out. Joint moves bring every joint to its target at the same moment.

A robot with fewer than six joints (a SCARA, a palletizer) can't hold an orientation. Give it only `x`, `y` and `z`. To move only part of a robot, give the tool a `base`: `humanoid.tool("Left Hand TCP", base="Torso")` moves the arm without the waist.

## Waiting, or not

Every move waits until it is done. Pass `wait=False` to get a **Motion** back straight away:

```python
a = robot.move_joints(A1=90, wait=False)
b = gantry.move_l(x=200, wait=False)          # a different robot: runs at the same time
a.wait(); b.wait()                            # Kotlin: a.await()
print(a.state, a.progress)                    # queued, running, done, failed, cancelled
robot.stop()                                  # stops every motion of the robot
```

Motions that share a joint run **one after another**, in the order you sent them. Motions with no joint in common run at the same time. If a motion fails, the motions queued behind it on the same joints are cancelled, the way a robot program stops at a fault.

## Walking a humanoid

A humanoid built on its legs (see [Humanoid Builder](help:humanoids)) walks: the whole robot is carried along while its feet step. `studio.humanoid()` returns the scene's only robot with legs (or give its name).

```python
walker = studio.humanoid()
walker.walk(1000)                       # 1 m ahead (negative: backwards)
walker.turn(90)                         # on the spot, to its left (negative: right)
walker.side_step(-300)                  # 300 mm to its right
walker.walk(1500, turn=90)              # an arc: the turn is spread over the walk
walker.walk_to(2000, 500, heading=180)  # to x, z (mm, world), then face -X
walker.walk_path([(1000, 0), (1000, 1000), (0, 1000), (0, 0)], heading=0)
walker.walk_path([(500, 0), (500, 500)], frame="robot")   # x ahead, z to its right
walker.walk(3000, speed=600, step_length=300, arm_swing=0)
print(walker.place)                     # Place(x, z, heading): where it stands
```

```cpp
auto walker = studio.humanoid();
walker.walk(1000);
walker.turn(90);
walker.walkTo(2000, 500, 180.0);
walker.walkPath({{1000, 0}, {1000, 1000}, {0, 1000}, {0, 0}}, 0.0);
walker.walk(3000, 0, 0, {.gait = {.stepLength = 300, .speed = 600}});
std::cout << walker.place() << "\n";
```

```kotlin
val walker = studio.humanoid()
walker.walk(1000)
walker.turn(90)
walker.walkTo(2000, 500, heading = 180)
walker.walkPath(1000 to 0, 1000 to 1000, 0 to 1000, 0 to 0, heading = 0)
walker.walk(3000, gait = Gait(stepLength = 300, speed = 600))
println(walker.place)
```

- **Distances** are millimetres, angles degrees. Its left is −Z. A heading is the robot's rotation about Y: 0 faces +X, 90 faces −Z.
- **Points** of `walk_to` and `walk_path` are on the floor: `x` and `z`. `frame="world"` (the default), `"robot"` (where it stands when the walk starts: x ahead, z to its right), or an object's name (that object's spot on the floor, turned with it). The path's corners are rounded, so it turns while it walks. A point behind it makes it turn round on the spot first.
- **The gait**: `step_length`, `side_step`, `step_height`, `crouch` and `sway` in mm, `turn_per_step` and `arm_swing` in degrees, `step_time` in seconds or `speed` in mm/s. Anything you leave out comes from the size of its legs; `walker.default_gait` shows the values. Steps that are too long for the legs are refused before the robot moves.
- **Waiting**: a walk waits until the humanoid stands again. With `wait=False` you get a **Motion**. Walks of one humanoid queue one after another, and a joint move of a leg waits for the walk. `walker.state()` says whether it walks and which step it is on.
- **Stopping**: `walker.stop_walking()` finishes the step under way and brings the feet together. `walker.stop()` halts at once, mid-step.

## Signals and the simulation

```python
studio.signals["Start"] = True                # Input and Internal signals
print(studio.signals["Box.count"])            # any symbol: J1.pos, Seq.running, …
studio.wait_until("PartSeen && !Robot1.moving", timeout=10)
studio.sleep(2)                               # simulation seconds
studio.play(); studio.pause(); studio.reset()
studio.speed = 5                              # 5× real time
```

Moves, signal writes and waits **start the simulation** if it isn't running. Timeouts and `sleep` count **simulation** seconds, so a program behaves the same at any simulation speed. **Reset** cancels every motion, and anything waiting on one gets an error.

Your program's moves and the scene's own sequences drive the same actuators. If a sequence moves a joint while a program moves it too, the one that commands last wins.

## When something goes wrong

A refused or failed command raises an error with a **code** and a message that says what was wrong:

| Code | Python | Kotlin | C++ | Meaning |
|---|---|---|---|---|
| `not_found`, `ambiguous` | `NotFoundError` | `NotFoundException` | `ms::NotFound` | No such robot, tool, joint, pose or signal, or several share the name (the message lists them) |
| `out_of_reach` | `OutOfReachError` | `OutOfReachException` | `ms::OutOfReach` | The target, or a point on the straight line, is out of the robot's reach |
| `out_of_limits` | `OutOfLimitsError` | `OutOfLimitsException` | `ms::OutOfLimits` | A joint value is outside the joint's range |
| `cancelled`, `reset` | `MotionCancelled` | `MotionCancelledException` | `ms::MotionCancelled` | Stopped, cancelled because an earlier motion failed, or the simulation was reset |
| `timeout` | `WaitTimeout` | `WaitTimeoutException` | `ms::Timeout` | A wait ran out of time; the motion or condition carries on |
| `simulation` | `SimulationError` | `SimulationException` | `ms::SimulationError` | The simulation can't start (the automation has errors) |
| `bad_request` | `MotionStudioError` | `MotionStudioException` | `ms::Error` | A parameter is wrong: a robot without legs asked to walk, a step too long for its legs |

They all derive from one base class (`MotionStudioError`, `MotionStudioException`, `ms::Error`), which carries the `code`.

The target of a move is checked before the robot moves, so an unreachable goal fails at once instead of halfway. A straight line that leaves the reach on the way stops where it left it.

## Writing a client in another language

The protocol is one JSON object per line over TCP: `{"id": 1, "method": "tool.moveL", "params": {...}}`. Replies are `{"id": 1, "result": ...}` or `{"id": 1, "error": {"code", "message"}}`. `api/PROTOCOL.md` lists every method.
