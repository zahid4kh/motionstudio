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
| `robots()` / `robot(name=None)` | Robots in the scene; a robot by name or path |
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

Errors derive from `MotionStudioError` (`.code`, `.message`). The subclasses are `NotFoundError`, `OutOfReachError`, `OutOfLimitsError`, `MotionCancelled`, `WaitTimeout` and `SimulationError`. Losing the connection raises the built-in `ConnectionError`.

Handles are thread-safe. A background thread reads the replies, so threads can share one `Studio`.

## Tests

`python3 -m unittest discover -s tests` starts `motionstudio-server` (`$MOTIONSTUDIO_SERVER` or `../../build/motionstudio-server`).
