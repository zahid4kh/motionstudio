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

Commands that need the simulation start it when it isn't running. These are the moves (walks included), `signal.set`, `signal.waitUntil`, `sim.wait` and `actuator.run`. `timeout` and `seconds` are in **simulation** time. `sim.reset` cancels every motion, and every pending wait is answered with the error `reset`.

## Error codes

| Code | Meaning |
|---|---|
| `bad_request` | Malformed request, or a parameter of the wrong type or out of range (a robot without legs asked to walk) |
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
| `scene.robots` | — | `[{name, path, joints: [names], tools: [names], walks}]` |
| `robot.info` | `robot` | `{name, path, joints: [{name, type, unit, value, min, max \| continuous, speed, acceleration}], tools: [{name, path, joints}], poses: [names], walks}` |
| `robot.joints` | `robot` | `{jointName: value, …}` |
| `actuator.get` | `name` (any actuator in the scene) | `{name, unit, value}` (deg or mm) |
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

### Walking

`walks` in `scene.robots` and `robot.info` is true for a robot that can walk: objects called `Left Foot` and `Right Foot` below its root, each moved by at least three actuated joints of its own (a humanoid the Humanoid Builder makes on legs). A walk carries the robot's root object along the floor while the legs step: each foot is set down where the plan puts it and stays there until it lifts. It is kinematic and can't fall. The floor is the root's height when the walk starts.

A place on the floor is `{x, z, heading}`: mm, and degrees about +Y (0 faces +X; positive turns to the robot's left, which is its −Z side).

| Method | Params | Result |
|---|---|---|
| `humanoid.walk` | `robot`, `forward?` (mm, negative: back), `sideways?` (mm to its left), `turn?` (degrees to its left), gait, `wait?` | a motion |
| `humanoid.walkTo` | `robot`, `x`, `z` (mm), `heading?` (degrees), `frame?`, gait, `wait?` | a motion |
| `humanoid.walkPath` | `robot`, `points: [{x, z} \| [x, z], …]` (mm), `heading?`, `frame?`, gait, `wait?` | a motion |
| `humanoid.stop` | `robot` | `{stopping: n}`: walks that end after the step under way, feet together (queued walks are cancelled) |
| `humanoid.state` | `robot` | `{x, z, heading, walking, motion?, step?, steps?, legs: {left, right, length, hipWidth}, gait: {…}}` |

Details:

- **walk:** a turn given with a distance is spread over the walk, so the robot walks an arc. All zero: done at once.
- **walkTo / walkPath:** the robot faces the way it goes; corners are rounded so it turns while it walks; a point behind it makes it turn on the spot first. With `heading`, it turns on the spot at the end to face that way.
- **frame:** `"world"` (the default); `"robot"`: the robot's place when the walk starts (x ahead, z to its right; so queued walks chain); or any object's name or path: points are on the floor below that object, turned with it, and `heading` is relative to its heading.
- **gait** (all optional, flat in `params`): `stepLength`, `sideStep`, `stepHeight`, `crouch`, `sway` (mm), `turnPerStep`, `armSwing` (degrees; 0 keeps the arms still), `stepTime` (s), `speed` (mm/s, sets the step time from the step length), `leftFoot` / `rightFoot` (object names, for a model whose feet are named differently). What is left out comes from the legs' size; `humanoid.state` returns the values.
- **Queueing:** a walk moves the leg joints (and the shoulder pitch joints while the arms swing), so walks of one robot queue one after another, and a joint move of a leg waits for the walk.
- **Errors:** a robot without legs, or a gait the legs can't do (steps too long, too fast), is a `bad_request` before anything moves. A foot that can't reach its spot on the way fails the walk with `out_of_reach`.
- `robot.stop` and `motion.cancel` stop a walk at once, mid-step; `humanoid.stop` finishes the step first.
- `humanoid.state` works while the simulation is stopped (`walking` is false then). `step` (1-based) and `steps` describe the walk under way; `motion` is absent for a sequence's Walk operation.

### Takes: recording and playing motion

| Method | Params | Result |
|---|---|---|
| `take.start` | `robot`, `rate?` (samples per second; 0 or absent: every scan) | `{take, robot, columns}` |
| `take.status` | `take` | `{take, robot, frames, duration}` |
| `take.stop` | `take` | `{take, robot, rate, columns, rows}` |
| `robot.play` | `robot`, `columns`, `rows`, `relative?`, `speed?`, `place?`, `wait?` | a motion (see Motions) |

Details:

- **A take** samples the robot after every scan while the simulation runs (a stopped simulation adds nothing). Its rows hold one number per column, in API units (mm, degrees, seconds, rounded to 0.001):
  - `t`: seconds since the first sample;
  - `x`, `y`, `z`, `rx`, `ry`, `rz`: the robot's top object in the world;
  - for a robot with legs: `heading`, `walking` (1 while it walks), `step` (the step under way, from 0; -1 while it crouches, stands up or stands), `swing` (the foot in the air: 0 left, 1 right, -1 none), `progress` (0–1 through the step), `goal_x`, `goal_z`, `goal_heading` (where the walk under way ends; its own place when standing);
  - then one column per joint, named after its actuator.
- A take belongs to the connection that started it, and is dropped when that connection closes.
- **robot.play** plays rows like a take's: `t` (s, increasing) is required. Columns named after the robot's actuators set those joints; other columns are ignored. With `x y z rx ry rz` it also moves the robot's top object to that frame; with only `x`, `z`, `heading` it moves it on the floor, keeping its height and tilt. `place: false` plays the joints only. Between rows it interpolates. Joint values beyond a joint's range are clamped.
- **relative:** the place columns start where the robot stands now (its x, z and heading on the floor), so a recorded walk can be repeated from anywhere.
- **speed:** × the recorded pace (default 1).
- A play moves its joints and, when it moves the top object, the legs, so it queues behind walks of the same robot (and they behind it).

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
| `signal.get` | `name` (any symbol: `Start`, `Box.count`, `J1.pos` …) | `{value}` (in the automation's units: a prismatic `.pos` is in metres; use `actuator.get` for mm) |
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
