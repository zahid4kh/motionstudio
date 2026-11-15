# MotionStudio robot API

Control MotionStudio's robots and stations from code in **Python**, **C++** or **Kotlin**. Your program can:

- move a robot's tool in straight lines and make joint moves;
- go to the poses you taught;
- run several machines at once;
- read and write the automation's signals.

Every library speaks the same small protocol ([PROTOCOL.md](PROTOCOL.md)) and needs nothing else.

```python
import motionstudio as ms

with ms.connect() as studio:
    robot = studio.robot("Robot")
    robot.move_l(z=100)                 # straight line; only z changes
    robot.move_by(y=-50, speed=100)     # 50 mm down at 100 mm/s
    robot.move_joints(A1=30)            # a joint move
    studio.signals["Start"] = True
```

| | Folder | Get it |
|---|---|---|
| Python 3.8+ | [python/](python/) | `pip install ./python` |
| C++17 | [cpp/](cpp/) | `add_subdirectory(cpp)` + `target_link_libraries(app PRIVATE motionstudio::motionstudio)` |
| Kotlin / JVM 17+ | [kotlin/](kotlin/) | `implementation("io.github.zahid4kh:motionstudio:0.1.1")` from Maven Central |

## What to connect to

- **The app.** **Tools › Robot API** runs the server. It is on by default, on port 5800, for this computer only. The window lists the robots in the open scene and shows a quick start for it.
- **`motionstudio-server`.** This serves a project without the app window, for automated tests, CI, or a machine without a display. It is built next to the app (`build/motionstudio-server`).

```sh
motionstudio-server cell.mstudio            # a project
motionstudio-server --robot "6-axis 0.9"     # a robot from a Robot Builder preset
motionstudio-server --humanoid "Full-size"   # a humanoid from a Humanoid Builder preset
motionstudio-server --presets                # list them
motionstudio-server --speed 10 --port 5801 --any   # 10× real time, another port, every network interface
```

## The model

- **Robots.** A robot is the set of actuated joints below one object: an arm, a humanoid, a gantry, a turntable. Address it by its name (or its path, `"Cell/Robot"`).
- **Tools.** A robot's tool is its frame named TCP. A humanoid has two, `Left Hand TCP` and `Right Hand TCP`.
- **Units.** Millimetres, degrees and seconds. **Y is up**, as in MotionStudio.
- **Poses.** A pose is `x y z rx ry rz`, with the rotation as Euler XYZ in degrees.
- **Frames.** Cartesian targets are in the robot's base frame unless you pass `frame="world"`, `"tool"` or an object's name.
- **Moves.** `move_l` goes in a straight line, `move_j` makes a joint move to a pose, `move_by` moves relative to where the tool is, and `move_joints` sets joint values. For absolute targets, the coordinates you leave out stay as they are.
- **Waiting.** Every move waits until it is done. With `wait=False` you get a `Motion` back to wait for later. Motions that share joints run in order; the others run together.
- **Errors.** A target out of reach or a joint out of its range fails before anything moves, with a message that says why.
- **Simulation.** The simulation starts by itself when a program moves something. Waits and timeouts count simulation seconds.

## Examples

Each library has the same examples:

- `square`: a 200 mm square in straight lines;
- `two_motions`: motions without waiting, queueing and stopping;
- `humanoid_wave`: joint moves and a hand moved as a tool.

## Tests

Each library has a live test that starts `motionstudio-server` and drives it:

```sh
cd python && python3 -m unittest discover -s tests
cd cpp && ./tests/run.sh
cd kotlin && ./gradlew test
```

Set `MOTIONSTUDIO_SERVER` if the server isn't at `../build/motionstudio-server`.
