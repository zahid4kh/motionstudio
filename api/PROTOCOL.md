# MotionStudio robot API — wire protocol (version 1)

The Python, C++ and Kotlin libraries in this folder all speak this protocol. Use it directly to write a client in another language.

## Transport

- TCP. The default port is **5800** (Tools › Robot API in the app, `--port` for `motionstudio-server`).
- Each message is **one JSON object on one line** (UTF-8, ending in `\n`), in both directions.
- A request has an `id` (a number or a string), a `method`, and optional `params` (an object):

```json
{"id": 7, "method": "tool.moveL", "params": {"robot": "Robot", "target": {"z": 100}, "speed": 200}}
```

- The reply carries the same `id`, with either `result` or `error`:

```json
{"id": 7, "result": {"motion": 3, "state": "done", "progress": 1}}
{"id": 7, "error": {"code": "out_of_reach", "message": "the target is out of reach (12.0 mm off)", "motion": 3}}
```

- **Replies can come in any order.** A move with `"wait": true` (the default) is answered when the motion ends, and the server answers other requests in the meantime. Match replies to requests by `id`. A client can keep many requests open at once, for example from several threads.
- Closing the connection drops the replies it was waiting for. Its motions carry on.

## Units and frames

- Lengths are **mm**, angles are **degrees**, times are **seconds**. Speeds are per second. **Y is up.**
- A pose is `{"x", "y", "z", "rx", "ry", "rz"}`, where the rotation is Euler XYZ in degrees (the same as object transforms in MotionStudio).
- Joint values are degrees, or mm for prismatic joints. Joints are named after their actuators.
- `frame` (on Cartesian methods) can be:
  - `"base"` (the default): the robot's root object;
  - `"world"`;
  - `"tool"`: for relative moves, along and about the tool's own axes;
  - any object's name or path.

## Names

- **Robots** are named by their root object's name or its path (`"Cell/Robot"`). `scene.robots` lists them. Any object that has actuated joints below it can be used as a robot.
- **Tools** are object names or paths, looked up under the robot first. Without a `tool`, the robot's only tool is used: an object named `…TCP…`, otherwise the end of its longest chain.
- **Objects** with a name that appears more than once must be given by path. The error lists the paths.

## Simulation

Commands that need the simulation start it when it isn't running. These are the moves, `signal.set`, `signal.waitUntil`, `sim.wait` and `actuator.run`. `timeout` and `seconds` are in **simulation** time. `sim.reset` cancels every motion, and every pending wait is answered with the error `reset`.

## Error codes

| Code | Meaning |
|---|---|
| `bad_request` | Malformed request, or a parameter of the wrong type or out of range |
| `unknown_method` | No such method |
| `not_found` | No such robot, tool, frame, joint, pose, signal or motion |
| `ambiguous` | Several objects (or tools) match; the message lists them |
| `not_actuated` | A joint in the move has no actuator |
| `out_of_limits` | A joint target is outside the joint's range |
| `out_of_reach` | The target, or a point on the straight line, can't be reached |
| `cancelled` | The motion was stopped, or cancelled because an earlier motion on its joints failed |
| `reset` | The simulation was reset |
| `timeout` | A wait ran out (the motion or condition carries on) |
| `simulation` | The simulation can't start; the message lists the automation's errors |
| `not_running` | The value exists only while the simulation runs |

## Motions

Every move creates a **motion** with an id.

- **Queueing.** Motions that share a joint run one after another in the order they were received. Motions that share no joint run at the same time.
- **Start-time checks.** A motion's start position, limits and reach are resolved and checked when it starts. A relative move therefore counts from where the previous one ended.
- **Failure.** When a motion fails, the queued motions that share a joint with it are cancelled.
- **Results.** A move with `"wait": false` returns `{"motion", "state", "progress"}` at once. A move with `"wait": true` returns the same object when the motion is done, or an error if it fails.
- **States:** `queued`, `running`, `done`, `failed`, `cancelled`.

## Methods

### Session and simulation

| Method | Params | Result |
|---|---|---|
| `hello` | `client?` | `{server, version, protocol, units: {length, angle, time}}` |
| `sim.state` | — | `{active, running, time, speed}` |
| `sim.play` / `sim.pause` / `sim.reset` | — | the state |
| `sim.speed` | `factor` (0.05 … 20) | the state |
| `sim.wait` | `seconds` | `{time}` once that much simulation time has passed |

### Scene and robots

| Method | Params | Result |
|---|---|---|
| `scene.robots` | — | `[{name, path, joints: [names], tools: [names]}]` |
| `robot.info` | `robot` | `{name, path, joints: [{name, type, unit, value, min, max \| continuous, speed, acceleration}], tools: [{name, path, joints}], poses: [names]}` |
| `robot.joints` | `robot` | `{jointName: value, …}` |
| `tool.pose` | `robot?`, `tool?`, `frame?`, `base?` | a pose |

### Moves

| Method | Params |
|---|---|
| `robot.moveJoints` | `robot?`, `joints: {name: value}`, `relative?` (false), `speed?` (% of each actuator's speed, ≤ 100), `wait?` (true) |
| `robot.moveToPose` | `robot?`, `pose` (a pose name), `speed?` (%), `wait?` |
| `robot.home` | `robot`, `speed?`, `wait?`: moves to the pose `"<robot> Home"`, else `"Home"` |
| `tool.moveL` | `robot?`, `tool?`, `base?`, `target: {x?, y?, z?, rx?, ry?, rz?}`, `frame?`, `relative?` (false), `speed?` (mm/s, default 250), `wait?` |
| `tool.moveJ` | the same as `tool.moveL`, with `speed?` in % of the joints' speed |
| `robot.stop` | `robot?` (none: every motion) → `{stopped: n}` |
| `actuator.run` | `name`, `speed` (deg/s or mm/s; 0 stops) → `{name, speed}` |

Details for the moves:

- **robot.moveJoints:** without `robot`, any actuator in the scene can be named.
- **tool.moveL target:** in an absolute move, the fields left out keep the tool's current values. If no rotation field is given, the orientation is kept. In a relative move, missing fields are 0.
- **tool.moveL base:** only the joints below that object move.
- **tool.moveL straight line:** the tool moves along a line, and its orientation turns evenly (slerp) on the way. The motion eases in and out. Rotation is timed so that 1 rad takes as long as 300 mm of travel.
- **tool.moveJ:** solves the target first (trying several start configurations if needed), then makes a synchronized joint move.
- **Fewer than six joints:** a tool with fewer than six joints above it can't hold an orientation, and giving `rx`/`ry`/`rz` is a `bad_request`.

### Motions

| Method | Params | Result |
|---|---|---|
| `motion.status` | `motion` | `{motion, state, progress, code?, error?}` |
| `motion.wait` | `motion`, `timeout?` | the status once it is done; an error if it failed, was cancelled or timed out |
| `motion.cancel` | `motion` | the status (the joints brake and hold) |

### Signals

| Method | Params | Result |
|---|---|---|
| `signal.list` | — | `[{name, kind, type, writable, value}]` |
| `signal.get` | `name` (any symbol: `Start`, `Box.count`, `J1.pos` …) | `{value}` |
| `signal.set` | `name` (an Input or Internal signal), `value` (number or bool) | `{value}` (as coerced to the signal's type) |
| `signal.waitUntil` | `condition` (an expression), `timeout?` | `{time}` once it is true |

Notes:

- `signal.get` returns a signal's initial value while the simulation is stopped.
- `signal.set` is applied at the start of the next scan, one write per scan per signal.
- `signal.waitUntil` uses the same expression language as the automation, so `"PartSeen && !Robot.moving"` works.

## Example session

```
→ {"id":1,"method":"hello","params":{"client":"netcat"}}
← {"id":1,"result":{"protocol":1,"server":"MotionStudio","units":{"angle":"deg","length":"mm","time":"s"},"version":"0.1.0"}}
→ {"id":2,"method":"tool.pose","params":{"robot":"Robot"}}
← {"id":2,"result":{"rx":0,"ry":0,"rz":0,"x":570,"y":715,"z":0}}
→ {"id":3,"method":"tool.moveL","params":{"robot":"Robot","target":{"y":-50},"relative":true,"wait":false}}
← {"id":3,"result":{"motion":1,"progress":0,"state":"queued"}}
→ {"id":4,"method":"motion.wait","params":{"motion":1}}
← {"id":4,"result":{"motion":1,"progress":1,"state":"done"}}
```
