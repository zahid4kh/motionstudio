# motionstudio (Python)

Control MotionStudio robots and stations from Python 3.8+. It uses only the standard library.

```sh
pip install .            # from api/python
```

```python
import motionstudio as ms

with ms.connect() as studio:                  # localhost:5800; ms.connect("host", 5801)
    robot = studio.robot("Robot")             # or studio.robot() for the only one
    print(robot.pose())                       # Pose(x=570.00, y=715.00, z=0.00, ...)
    robot.move_l(z=100)                       # straight line, only z changes
    robot.move_by(y=-50, speed=100)           # relative, 100 mm/s
    robot.move_joints(A1=30, A2=-10)          # joints arrive together
    robot.home()
```

Units are mm, degrees and seconds, and Y is up. See [../README.md](../README.md) for the model and [../PROTOCOL.md](../PROTOCOL.md) for the protocol.

## Reference

### `connect(host="localhost", port=5800, timeout=10) → Studio`

| `Studio` | |
|---|---|
| `robots()` / `robot(name=None)` | Robots in the scene; a robot by name or path (a `Humanoid` when it walks) |
| `humanoid(name=None)` | A robot with legs → `Humanoid`; without a name the only one |
| `actuator(name)` | One actuator (a door, a turntable) → `Actuator` |
| `signals[name]`, `signals[name] = v` | Read any symbol; write Input/Internal signals. `signals.list()` |
| `get(name)` / `set(name, value)` | The same as functions |
| `wait_until(condition, timeout=None)` | Waits until an expression is true; returns the time |
| `play()`, `pause()`, `reset()` | Simulation control |
| `sleep(seconds)` | Waits for simulation time to pass |
| `time`, `running`, `speed` (settable) | Simulation state |
| `close()` / `with` | Ends the connection |

| `Robot` | |
|---|---|
| `name`, `path`, `joint_names`, `tool_names` | |
| `joints` | `{name: value}` now |
| `joint_info()` | `[JointInfo(name, type, unit, value, min, max, speed, acceleration)]` |
| `move_joints(values=None, *, speed=None, relative=False, wait=True, **joints)` | Joint move (speed in %) |
| `move_to_pose(name, *, speed, wait)` / `home(...)` | Taught poses |
| `stop()` | Stops this robot's motions |
| `tool(name=None, *, base=None)` | A tool frame → `Tool` |
| `pose()`, `move_l()`, `move_j()`, `move_by()` | The only tool's methods |
| `record(rate=None)` → `Recording` | Records a take: joint values and place after every scan (or `rate`/s); `.stop()` → `Take`, or use `with` |
| `play(take, *, relative=False, speed=None, place=True, wait=True)` | Plays a `Take` (or a CSV path) back; `relative`: from where it stands |

| `Humanoid` (a `Robot` that walks) | |
|---|---|
| `walk(forward=0, sideways=0, turn=0, *, gait=None, wait=True, **gait)` | mm ahead, mm to its left, degrees to its left; a turn with a distance makes an arc |
| `turn(degrees, …)`, `side_step(mm, …)` | On the spot / sideways |
| `walk_to(x, z, heading=None, *, frame="world", …)` | To a spot on the floor (mm), then face `heading` |
| `walk_path(points, heading=None, *, frame="world", …)` | Through `[(x, z), …]`; `frame="robot"`: x ahead, z to its right; or an object's name |
| `walk_steps(steps, *, first=None, frame="world", …)` | Exactly these footsteps `[(x, z, heading), …]`, then the other foot alongside |
| `stop_walking()` | Finishes the step under way, feet together |
| `state()` → `WalkState(place, walking, motion, step, steps, gait)` | Also `place`, `walking`, `default_gait` |

Gait options, as keywords (`step_length=250, speed=400`) or `gait=Gait(...)`: `step_length`, `side_step`, `step_height`, `crouch`, `sway` (mm), `turn_per_step`, `arm_swing` (degrees), `step_time` (s), `speed` (mm/s). What you leave out comes from the legs. `Place(x, z, heading)` is where it stands (heading 0 faces +X, positive to its left).

| `Tool` | |
|---|---|
| `pose(frame="base")` | → `Pose` |
| `move_l(target=None, *, x, y, z, rx, ry, rz, speed=None, frame="base", wait=True)` | Straight line (mm/s, default 250) |
| `move_j(...)` | The same target, by a joint move (speed in %) |
| `move_by(x=0, y=0, z=0, rx=0, ry=0, rz=0, *, speed, frame="base", linear=True, wait=True)` | Relative; `frame="tool"` for the tool's own axes |

| `Motion` (from `wait=False`) | |
|---|---|
| `wait(timeout=None)` | Raises if it failed or was cancelled |
| `cancel()`, `state`, `done`, `progress`, `id` | |

| `Actuator` | |
|---|---|
| `value`, `move_to(v)`, `move_by(d)`, `run(speed)`, `stop()` | |

`Pose(x, y, z, rx, ry, rz)` is frozen, with `.offset(...)` and `.replace(...)`.

| `Take` (a recording) | |
|---|---|
| `columns`, `rows`, `robot`, `rate`, `duration`, `len(take)` | `t`, `x y z rx ry rz`, for legs `heading walking step swing progress step_x step_z step_heading goal_x goal_z goal_heading`, then the joints |
| `take["x"]`, `frame(i)`, `joints(i)`, `joint_names`, `walks` | Columns and rows |
| `between(t0, t1)`, `select(columns)` | Parts of it |
| `save(path)`, `Take.load(path)` | The CSV that MotionStudio's **Record Values** writes |
| `array(columns=None)` | A NumPy array (needs NumPy) |

```python
with walker.record() as rec:
    walker.walk_to(2000, 500)
rec.take.save("a-to-b.csv")
walker.play(ms.Take.load("a-to-b.csv"), relative=True)
```

Examples (`examples/`): `square.py`, `two_motions.py`, `station.py`, `humanoid_wave.py`, `walk_square.py`, `walk_patrol.py`, `learn_to_walk.py`, `sorting_station.py`.

Errors derive from `MotionStudioError` (`.code`, `.message`). The subclasses are `NotFoundError`, `OutOfReachError`, `OutOfLimitsError`, `MotionCancelled`, `WaitTimeout`, `SimulationError` and `UnsavedChangesError`. Losing the connection raises the built-in `ConnectionError`.

Handles are thread-safe. A background thread reads the replies, so threads can share one `Studio`.

## Building scenes (`studio.scene`)

A program can build the scene: import CAD, group it, put joints through it, add belts, bodies, sensors, poses, sequences and part flow, and save the project. Changes need the simulation reset (`studio.reset()`).

```python
scene = studio.scene
station = scene.import_file("station.step")         # Z-up CAD stood upright; up_axis="y" if not
arm = scene.add_frame("Arm", parent=station)
scene.object("arm1").set_parent(arm)                 # keeps where it is (and its joint)
hub = max(scene.object("arm1").features(["circle"], frame=station), key=lambda f: f.radius)
scene.add_joint("J1", station, arm, (0, 0, 1), frame=station, pivot=hub.origin, min=-170, max=170)
scene.add_sequence("Cycle", ms.Ops().move("Out", "J1", 90).wait("Hold", 1).move("Back", "J1", 0), loop=True)
scene.save("station.mstudio")
```

| `Scene` | |
|---|---|
| `new(discard=False)`, `open(path, discard=False)`, `save(path=None)` | Projects (in the app, unsaved changes need `discard=True`) |
| `info()`, `check()` | What it holds by name; the automation's problems (empty: it runs) |
| `import_file(path, parent=None, name=None, up_axis=None)` | STEP, URDF, MJCF or a project (merged) → its top `SceneObject` |
| `objects(name=None, under=None, type=None, depth=None)`, `object(name)`, `exists(name)` | Find objects (wildcards, paths) |
| `add(type, name, parent, position, frame, size, diameter, height, color)`, `add_frame`, `add_box`, `add_cylinder`, `add_sphere` | New objects (mm; positions in the parent's frame unless `frame`) |
| `group(objects, name)` | A frame at their middle, with them under it |
| `add_joint(name, parent, child, axis, type, pivot, frame, min, max, continuous, value, actuator, follows)`, `joint(name)` | Joints (`actuator={"speed": 90}`, `follows={"joint": "Gate", "ratio": -1}`) |
| `add_signal`, `add_sensor`, `add_pose`, `add_sequence(name, Ops, loop, enable)`, `sequence(name)`, `add_source`, `add_sink`, `remove(name)` | The logic |

`SceneObject` (found by id, so it survives renames and moves): `path`, `name`, `type`, `position`, `world`, `pose_in(frame)`, `size`, `children`, `set(...)`, `move_to(x=…, frame=…)`, `rename`, `set_parent(None for the top)`, `add(...)`, `duplicate`, `remove`, `bounds(frame)` → `Box`, `features(kinds, frame)` → `[Feature]`, `set_body(mass)`, `set_belt(speed, direction)`.

`Ops` builds a sequence: `move`, `move_to_pose`, `run`, `stop`, `set`, `wait`, `wait_for`, `grip`, `release`, `move_l`, `walk`, `play_motion`, each with `after`, `on_start`, `on_finish`. A number is in mm, mm/s or degrees; a string is an expression in the automation's units (m, m/s): `move("Down", "J3", "Depth - 0.1")`.

`examples/sorting_station.py` builds a whole sorting station twin from its STEP file.

## Learning motion (`motionstudio.learn`)

`pip install "motionstudio[learn]"` (it needs NumPy, nothing else). Small neural networks, trained on your CPU in seconds to minutes, learn from takes:

| | |
|---|---|
| `AnimationModel(hidden=(128, 128)).fit({name: take, …}, epochs=400)` | Learns each take as a clip: joint values (and how the robot moves on the floor) over the clip's time |
| `.generate(clip, *, duration=None, rate=50, place=True)` → `Take` | A clip at any length, or a blend `{"wave": 0.7, "bow": 0.3}`; play it with `robot.play(take)` |
| `.pose(clip, at=0.5)` | The joint values at a moment of a clip (0 = start, 1 = end) |
| `WalkPolicy(hidden=(512, 512)).fit(takes, epochs=300, motion_epochs=150, verbose=True)` | Learns to walk from takes of a humanoid walking to many goals |
| `.walk(walker, x, z, heading=None)` | The robot steps where the policy decides; its own walking plants each foot exactly |
| `.motion(x, z, heading=None, *, start=walker)` → `Take` | The whole walk from the networks, every joint included; `robot.play(take)` |
| `.footsteps(start, x, z, heading=None, *, exact=True)` | Only the footsteps `[(x, z, heading), …]` |
| `.save(path)` / `.load(path)` | Both models save to one `.npz` |

The walk policy is two networks:
- The **step policy** sees where the goal is from the last footstep, and the step before, and decides the next footstep and whether it is the last.
- The **motion model** gives every joint's value and how the body sways through a step, from the phase in the step and the steps around it.

The policy ends within a few centimetres of a goal on its own. With `exact=True` (the default), its last footstep is then put on the goal, when that is less than half a step away.

The learned motion (`motion`) has no foot contacts, so its feet slide slightly. Playing the policy's footsteps through the robot's own walking (`walk`) does not.

```python
import motionstudio as ms
from motionstudio.learn import WalkPolicy

takes = []
with ms.connect() as studio:
    walker = studio.humanoid()
    for x, z in goals:                          # many places, near and far
        with walker.record(rate=25) as rec:
            studio.sleep(0.2)                   # standing before and after each walk
            walker.walk_to(x, z)
            studio.sleep(0.3)
        takes.append(rec.take)
    policy = WalkPolicy().fit(takes, verbose=True)
    policy.walk(walker, 1500, 800, heading=90)  # a goal it never saw
```

`motionstudio.learn.walk_steps(take)` splits a walk take into its footsteps and steps, and `MLP` is the network underneath, if you want to build your own.

## Tests

`python3 -m unittest discover -s tests` starts `motionstudio-server` (`$MOTIONSTUDIO_SERVER` or `../../build/motionstudio-server`).
