# Robot API (code)

Control robots and stations from your own programs, in **Python**, **C++** or **Kotlin**. A program can move a robot's tool in straight lines, make joint moves, go to poses you taught, walk a humanoid along a path, and read and write signals. It can also build the scene itself: import CAD and add joints, sensors and logic (see *Building scenes*). You can test a robot program here before it runs a real machine, or let outside software drive the model.

## Switching it on

**Tools › Robot API…** runs the server that programs connect to. It is on by default, on port **5800**, for programs on this computer only. The window lists the robots in the scene with their joints and tools, and shows a quick start for this scene in each language (**Copy Code**). While programs are connected, the status bar shows **API :5800 · 1 program**.

## Getting the libraries

None of the libraries needs anything else. Their source is in the **api/** folder of the MotionStudio source.

**Python 3.8+.** Install it from PyPI:

```sh
pip install motionstudio
```

`pip install "motionstudio[learn]"` also installs NumPy, for learning motion. A wheel file you were given installs the same way: `pip install motionstudio-0.3.0-py3-none-any.whl`.

**Kotlin and Java (JVM 17+).** Add the dependency from Maven Central to your Gradle build:

```kotlin
dependencies { implementation("io.github.zahid4kh:motionstudio:0.3.0") }
```

**C++17.** The library comes as a source archive, `motionstudio-cpp-0.3.0.tar.gz`. CMake can download and build it with your project:

```cmake
include(FetchContent)
FetchContent_Declare(motionstudio
    URL <where the archive is>/motionstudio-cpp-0.3.0.tar.gz
    URL_HASH SHA256=<its hash>
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(motionstudio)
target_link_libraries(my_app PRIVATE motionstudio::motionstudio)
```

You can also install it once, then use it from any project:

```sh
tar xzf motionstudio-cpp-0.3.0.tar.gz && cd motionstudio-cpp-0.3.0
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
sudo cmake --install build
```

After that, use `find_package(motionstudio 0.3 REQUIRED)` in CMake. Without CMake, compile with `g++ -std=c++17 main.cpp $(pkg-config --cflags --libs --static motionstudio)`.

## Running without the app window

On a server, or in automated tests, use `motionstudio-server project.mstudio`, or `motionstudio-server` alone for an empty scene to build in. It can also build a robot from a preset: `motionstudio-server --robot "6-axis 0.9"`, or `--humanoid "Full-size"`.

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
walker.walk_steps([(200, 0, 0), (400, 0, 0), (550, -50, 20)])   # footsteps you choose
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

## Recording and learning motion

A **take** is what a robot did, as numbers: its joint values and where it stands, every scan. **Record Values** saves one from the app (see [Running & Debugging](help:runtime)); a program records one around anything it does, and plays takes back:

```python
with walker.record() as rec:                 # every scan; record(rate=25) for 25 a second
    walker.walk_to(2000, 500)
take = rec.take                              # ms.Take: columns, rows, take["x"], take.frame(i)
take.save("a-to-b.csv")                      # the same CSV as Record Values
walker.play(ms.Take.load("a-to-b.csv"), relative=True)   # again, from where it stands
walker.play(take, speed=2, place=False)      # only the joints, twice as fast
walker.walk_steps([(200, 0, 0), (400, 0, 0)])  # exactly these footsteps (x, z, heading)
```

`play` sets the joints named in the take's columns and moves the robot to its `x y z rx ry rz` (or `x z heading` on the floor), interpolating between rows. With `relative=True` the motion starts where the robot stands.

The app itself records, trains and plays motions without any code (see [Motions](help:motions)). **Python only:** `motionstudio.learn` (needs NumPy) teaches small neural networks from takes, for your own experiments:

- `AnimationModel().fit({"wave": take1, "bow": take2})` learns animations and poses. `generate("wave", duration=3)` plays one at any length, `generate({"wave": 0.5, "bow": 0.5})` blends them, and `pose("bow", 0.5)` gives the joint values half way through.
- `WalkPolicy().fit(takes)` learns to walk from takes of a humanoid walking to many goals. Its **step policy** decides each footstep towards a goal, and its **motion model** moves every joint through a step. `policy.walk(walker, x, z, heading)` makes the robot step where the policy decides, so its feet land exactly. `policy.motion(x, z, heading, start=walker)` makes the whole motion from the networks, as a take to play.

The Python library's README describes them. The test project's scripts 09 to 13 show the whole round: record, collect, train, walk.

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

## Building scenes

A program can also **build** the scene, as you would with the mouse but repeatably: import CAD, group its parts, put joints through its own geometry, and add belts, rigid bodies, sensors, part sources and sinks, poses and sequences. Then it saves the project. What a program builds shows in the app at once, and **Ctrl+Z** takes it back.

```python
scene = studio.scene
studio.reset()                                   # the scene only changes while the simulation is reset
station = scene.import_file("station.step")      # Z-up CAD is stood upright (up_axis="y" if it is Y-up)
S = station                                      # CAD coordinates (mm, Z up) are in the station's frame

table = scene.add_frame("Turntable", parent=station)
for name in ["circle", "static", "slider_holder"]:
    scene.object(name).set_parent(table)         # keeps where it is
centre = scene.object("circle").bounds(S).center
scene.add_joint("Turn", station, table, (0, 0, 1), frame=S, pivot=(centre.x, centre.y, 0),
                min=-5, max=95, actuator={"speed": 60})

scene.object("belt").set_belt("Run ? 0.2 : 0", (1, 0, 0), frame=S)    # m/s as an expression
scene.add_signal("Run", "input", value=True)
scene.add_sensor("AtGate", "beam", mount=table, frame=S, at=(2132, -180, 159), direction=(0, 1, 0), range=360)
scene.add_sequence("Sorter", ms.Ops()
                   .wait_for("Part there", "AtGate")
                   .move("Turn", "Turn", 90)                # degrees (mm for a slide)
                   .wait("Settle", 0.5)
                   .move("Back", "Turn", 0), loop=True, enable="Run")
print(scene.check())                             # what would stop Play: [] when it runs
scene.save("station.mstudio")
```

The C++ and Kotlin libraries have the same calls: `studio.scene()` / `studio.scene`, `importFile`, `addFrame`, `addJoint`, `Ops`, `save`.

- **Where things go.** Positions are in the parent's frame unless you give `frame`, which is `"world"` or any object. For CAD, giving the model's top object as the frame lets you use the coordinates you see in the CAD system.
- **Finding parts.** Use `scene.objects("legs:*")` (wildcards), `scene.object("Station/Sorter/Lift")` (name or path) and `bounds(frame)` (a part's box). `features(["circle", "cylinder"])` finds the round edges and faces of a CAD part, with their centre, axis and radius, so a joint can go exactly through a bearing or a shaft.
- **Joints** take an axis and a pivot. The child goes under the parent, and it gets an actuator named after the joint unless `follows` couples it to another joint (gears, mirrored gates). `min` and `max` are in degrees, or millimetres for a `prismatic` joint.
- **Values for the logic.** A number is in mm, mm/s, degrees or seconds. A string is an expression, in the units the automation uses: metres for slides and m/s for belts, the same as you type in the Automation panel.
- **Handles.** An object handle keeps working after you rename or move the object, because it is found by its id. Two parts with the same name (CAD files have them) can be told apart that way too.
- **Unsaved work.** `scene.new()` and `scene.open()` won't throw away unsaved changes in the app: save first, or pass `discard=True`.

`api/python/examples/sorting_station.py` builds a whole sorting station twin from its STEP file this way: a SCARA robot, conveyors, a turning and lifting sorter, bins, sensors and the logic.

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
| `simulation_active` | `SimulationError` | `SimulationException` | `ms::SimulationError` | The scene can't change while the simulation is active: reset it first |
| `unsaved` | `UnsavedChangesError` | `UnsavedChangesException` | `ms::UnsavedChanges` | The project open in the app has unsaved changes: save, or pass discard |
| `bad_request` | `MotionStudioError` | `MotionStudioException` | `ms::Error` | A parameter is wrong: a robot without legs asked to walk, a step too long for its legs |

They all derive from one base class (`MotionStudioError`, `MotionStudioException`, `ms::Error`), which carries the `code`.

The target of a move is checked before the robot moves, so an unreachable goal fails at once instead of halfway. A straight line that leaves the reach on the way stops where it left it.

## Writing a client in another language

The protocol is one JSON object per line over TCP: `{"id": 1, "method": "tool.moveL", "params": {...}}`. Replies are `{"id": 1, "result": ...}` or `{"id": 1, "error": {"code", "message"}}`. `api/PROTOCOL.md` lists every method.
