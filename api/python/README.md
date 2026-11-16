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

Examples (`examples/`): `square.py`, `two_motions.py`, `station.py`, `humanoid_wave.py`, `walk_square.py`, `walk_patrol.py`, `learn_to_walk.py`.

Errors derive from `MotionStudioError` (`.code`, `.message`). The subclasses are `NotFoundError`, `OutOfReachError`, `OutOfLimitsError`, `MotionCancelled`, `WaitTimeout` and `SimulationError`. Losing the connection raises the built-in `ConnectionError`.

Handles are thread-safe. A background thread reads the replies, so threads can share one `Studio`.

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
