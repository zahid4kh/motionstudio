# MotionStudio — Usage Guide

## Launch

```bash
./run.sh
```

---

## Core concept

**Objects added while something is selected become children of it.**
Parent-child relationships are what joints connect.

```
Base (Cube)              ← root, no joint
 └── Arm1 (Cylinder)     ← child, has revolute joint to Base
      └── Arm2 (Cylinder) ← child, has revolute joint to Arm1
```

---

## Quick start: spinning roller

1. Click **`+ Cube`** → machine frame appears at origin
2. Cube is selected → click **`+ Cylinder`** → roller added as child
3. In **Properties**, set roller **Position Y = 1** to lift it above the base
4. Roller still selected → click **`Add Joint`**, tick **Continuous (no limits)**
5. Roller still selected → click **`Add Actuator`**; in the **Actuator** group set **Mode = Speed**, **Speed = 90**
6. Click **`▶ Play`** → roller spins

A Speed-mode actuator runs on its own. Position-mode actuators stay still until logic moves them (see below).

---

## Quick start: a sequenced robot arm

1. **`+ Cube`** → Base
2. Select Base → **`+ Cylinder`** → Arm1, set Position Y = 1
3. Select Arm1 → **`+ Cylinder`** → Arm2, set Position Y = 1
4. Select Arm1 → **`Add Joint`** → **`Add Actuator`** (named **J1**)
5. Select Arm2 → **`Add Joint`** → **`Add Actuator`** (named **J2**)
6. **Automation › Signals** → **`+ Input`**, name it `Start`
7. **Automation › Sequences** → **`+`** (new sequence), set **Start when** = `Start`, tick **Loop**
8. **`+ Operation ▾` › Move**: Actuator J1, Target `60`
9. **`+ Operation ▾` › Move**: Actuator J2, Target `-45`. It is placed after #1 automatically.
10. **`+ Operation ▾` › Move**: Actuator J1, Target `0`. Then set **After** to include #2.
11. **`▶ Play`**, open **Automation › Runtime** and switch **Start** ON

The templates (**File › Open Template…**) are complete working examples. *Robotic Arm — Pick & Place* and *Indexing Table* use most of the features. *Sensor & Gripper Cell* shows sensors, a belt that stops at a sensor, and Grip/Release.

---

## Camera controls

| Action | Control |
|--------|---------|
| Orbit  | Alt + Left drag |
| Pan    | Right drag |
| Zoom   | Scroll wheel |
| Select | Left click on object |

### Viewport buttons (top right)

| Button | Does |
|--------|------|
| **IK** | Dragging a robot tool's gizmo moves its joints by inverse kinematics instead of moving the part |
| **Section** | Cuts the model with a plane (X, Y or Z, at a position you slide) to see inside assemblies; cut faces show in a flat colour. *Keep the other side* flips it |
| **Lighting** | Ambient, key, fill and specular light and the light's direction, per theme |
| **Floor** | Turns the floor collider on or off |

---

## Toolbar reference

| Button | What it does |
|--------|-------------|
| `+ Cube / Cylinder / Sphere / Capsule / Basket` | Add primitive (child of selection, or root if nothing selected) |
| `Add Joint` | Add revolute joint between selected object and its parent |
| `Add Actuator` | Add an actuator (J1, J2…) to the selected object's joint |
| `Delete` | Remove selected object (and its children) |
| `▶ Play` | Run the automation logic (Space). Fails with a list of problems if any definition is invalid |
| `⏸ Pause` | Pause (Space) |
| `⏭ Step` | Run exactly one 16 ms scan, then pause (F10) |
| `↺ Reset` | Stop and restore the scene, joints and signals to how they were before Play |
| `1×` | Playback speed (0.25×–4×). Results are identical at every speed |
| `New / Open / Save` | File operations. A project path can also be passed on the command line |

While the simulation is active (from the first Play/Step until Reset), automation definitions, joints and actuators are locked. Undo/redo are disabled.

---

## Properties panel

### Transform
- **Position** — local position relative to parent
- **Rotation** — Euler angles in degrees (XYZ order)
- **Scale** — local scale
- **Colour** — click the swatch for any colour, or pick a preset (steel, aluminium, anthracite, safety orange, signal yellow, blue, green, red); **×** removes it. A part without its own colour uses its parent's, so colouring an assembly colours all its parts. STEP imports keep the colours stored in the file.

### Joint (visible when object has a joint)

| Field | Meaning |
|-------|---------|
| Type | **Revolute** = rotates, **Prismatic** = slides |
| Axis | Direction of rotation/translation (e.g. `0,1,0` = Y-axis) |
| Continuous | Revolute only: unlimited rotation (limits ignored) |
| Limit Min / Max | Angle range (degrees) or displacement range (m). Motion stops at a limit |
| Init angle | Value the joint is set to when the project loads |
| Follows | Couple this joint to another: value = **Ratio** × that joint + **Offset** (gears, belts, rack and pinion, mirrored gripper fingers). A follower needs no actuator; it moves whenever its master does, also while jogging and in physics |
| Position | The joint's current value. Jog it to pose the mechanism; Play starts from here |
| Rotation Origin | Pivot of a revolute joint, in the parent's local frame |

### Conveyor Belt

**Add Conveyor Belt** makes the object's top surface push rigid bodies resting on it (MuJoCo only).

| Field | Meaning |
|-------|---------|
| Velocity | Surface speed in m/s |
| Direction | Push direction in the object's frame |
| Belt enabled | Off: the surface doesn't push |
| Speed expr | Optional expression evaluated every scan while simulating, overriding Velocity, e.g. `PartAtEnd ? 0 : 0.4` |

### Actuator (visible when the joint has an actuator)

| Field | Meaning |
|-------|---------|
| Name | Symbol used in expressions (`J1.pos`, `J1.inPosition`…). Rename it in the Automation panel |
| Mode | **Position**: holds still until commanded to a target. **Speed**: runs at *Speed* from the start |
| Speed | Default max speed (Position) or signed speed (Speed), deg/s or m/s |
| Accel | Acceleration/deceleration for a trapezoidal profile; `instant` = 0 |
| Active at start | When off, the actuator holds still until activated |

---

## Automation panel

The panel at the bottom works like the signal and operation editors of a mechatronics concept designer. Joints only move when logic tells them to.

### Signals tab

| Kind | Meaning |
|------|---------|
| **Input** | Set by you while the simulation runs (Runtime tab buttons, switches, numbers) or by operation actions |
| **Internal** | State or flags written by operation actions, e.g. `Gripper = true`, `Count = Count + 1` |
| **Derived** | Recomputed every scan from an expression, e.g. `Ready = J1.inPosition && !EStop` |
| **Sensor** | Read from the scene every scan: true while something is in its beam or zone |

Types are **Bool**, **Int** or **Real**. Renaming a signal rewrites every expression that uses it.

**Sensors** close the loop between physics and logic. Add one with **New Sensor** (mounted on the selected object), or right-click an object in the Hierarchy and choose **Add Sensor Here…**. **Configure…** sets:

| Setting | Meaning |
|---------|---------|
| Type | **Beam**: a ray along a direction, up to a range (light barrier, part present). **Zone**: a box, or the mounted object's own bounds (contact sensor) |
| Mounted on | The sensor moves with this object; *World* keeps it fixed. It never detects its mount, the mount's children or the objects the mount hangs from |
| Offset / Direction / Range / Size | In metres, in the mount's frame (its scale is ignored) |
| Detects | Anything, rigid bodies only (parts), or one specific object |

The viewport draws beams and zones green while clear and orange while detecting. Example: set a conveyor belt's *Speed expr* to `PartAtEnd ? 0 : 0.4`, so the belt stops when a part reaches the end.

### Actuators & Poses tab

- **Actuator table + form** — mode, speed and acceleration, plus optional **bindings**. A bound **Target**, **Velocity** or **Active** expression is re-evaluated every scan and overrides operations. Example: `Target = Gripper ? 0 : 0.04`.
- **Jog joints** — sliders that pose the mechanism at edit time. **All to Init** and **Current → Init** move between the current pose and the saved initial values.
- **Cartesian jog** — choose a **Tool** (a part the actuated joints move), then **X− … Z+** move it along the world axes by inverse kinematics; **Keep orientation** holds its angle (6+ joints). The **IK** button on the viewport does the same by dragging the tool's gizmo.
- **Poses** — named joint snapshots (teach points). **Capture** saves the current pose, **Update** overwrites one, **Go to** jogs to one. Use poses with *MoveToPose* operations.

### Sequences tab

A sequence is a set of **operations** with **predecessors**. An operation starts when all operations listed in its **After** field are done and its **Start when** condition is true. Operations that share a predecessor run **in parallel**.

| Operation | Does | Done when |
|-----------|------|-----------|
| **Move** | Actuator → Target (Position mode), optional max speed | actuator is at the target and stopped |
| **MoveToPose** | All actuated joints of a pose, synchronized to arrive together; Speed % scales them | all have arrived |
| **MoveL** | A *Tool* part moves in a straight line (eased, at *Speed* m/s) to where it is in a pose; the joints follow by inverse kinematics | tool arrived |
| **Run** | Actuator → Speed mode at the given signed speed | immediately |
| **Stop** | Actuator decelerates to rest | it has stopped |
| **Set** | Only runs its *On start* actions | immediately |
| **Wait** | Waits *Duration* seconds | time elapsed |
| **WaitFor** | Waits until *Until* is true | condition true |
| **Grip** | Attaches a part to a *Gripper* object so it moves with it. *Part* is a fixed object or **Detected by** a sensor (then it waits for a detection) | part held |
| **Release** | The *Gripper* lets go of everything it holds; rigid bodies fall again | immediately |

Every operation can also run **On start** and **On finish** actions: `Sig = expr; Other = expr`.

Sequence settings:
- **Enabled when** — while false, no new operations start, but running ones finish. Use it for Stop or Auto modes.
- **Start when** — gate for starting each cycle, e.g. `Start` or `rising(Start)`.
- **Loop** — start a new cycle after each one completes. Each sequence publishes `<Name>.running`, `<Name>.done` and `<Name>.cycles`.

The **Gantt** chart under the operation table shows each operation's timing in the current cycle, with live progress. The previous cycle is shown faded. After each cycle the sequence panel shows the **cycle time** and the **critical path** (the chain of operations that set it, outlined in the Gantt), split into working and waiting time. Speed up operations on that path to shorten the cycle.

### Runtime tab

- **Inputs** — **Push** (momentary, seen by at least one scan) and **ON/OFF** (latching) for Bool inputs, number boxes for Int/Real
- **Watch** — every signal and status value, live; tick **Plot** to trace it
- **Sequences** — state of every operation, with the reason it is waiting (`after 3`, `start condition: Safe`…)
- **Trace** — scrolling plot of the ticked signals
- **Check interference between moving parts** — reports parts that pass through each other while mechanisms move (tinted red, named in the viewport overlay, listed with times). Parts touching at the start and rigid bodies are ignored; applies from the next Play after Reset

The ✓/⚠ indicator in the panel's top-right corner validates all definitions as you edit. Fields with errors get a red outline, and their tooltip explains the problem.

### Timeline (under the viewport while simulating)

Every scan is recorded until **Reset**. Pause, then drag or click the timeline to see any earlier moment: the 3D view, the watch values, the Signals tab and the trace playhead follow. **Live** (or Play/Step) returns to the latest scan. **Export Video…** saves the run, or a time range of it, as MP4, an animated GIF (both need `ffmpeg`) or PNG frames. Frames are rendered from the current camera at 15–60 fps, at 50–100 % of the viewport size.

### Expressions

```
literals    1.5   true   false
names       Start   J1.pos   PickPlace.cycles
operators   + - * / %   == != < <= > >=   && || !   cond ? a : b   ( )
functions   abs min max clamp sin cos tan sqrt floor ceil round sign
            rising(x)  falling(x)  changed(x)  time()
```

- Everything is a number; booleans are 0/1.
- `sin`, `cos` and `tan` take degrees.
- `rising(x)` is true for the one scan where `x` becomes true.
- Field autocomplete suggests names as you type.

**Actuator status names** (for actuator `J1`): `J1.pos`, `J1.vel`, `J1.target`, `J1.inPosition`, `J1.moving`, `J1.atMin`, `J1.atMax`, `J1.active`.

### How a scan works (every 16 ms)

1. Apply action writes from the previous scan and queued Runtime inputs
2. Read sensors, then publish actuator and sequence status signals
3. Evaluate derived signals (in dependency order)
4. Advance sequences: finish operations, then start ready ones
5. Evaluate actuator bindings
6. Integrate the motion profiles and move the joints

Because of step 1, signals written by actions are visible from the next scan, as in a PLC.

With MuJoCo installed, scenes that contain rigid bodies also simulate physics. Actuated joints are still driven kinematically, exactly along their profiles. Free rigid bodies and joints without an actuator are handled by MuJoCo.

### External I/O (Tools › External I/O, Modbus TCP)

A real PLC (or CODESYS, OpenPLC…) can drive the model over Modbus TCP. Give signals an address (the Signals tab's *Modbus* column, or **Assign Free Addresses** in the dialog) and tick **Run the server** (default port 5020; *All network interfaces* for a PLC on another machine). The PLC writes **Input** signals (Bool → coils, Int/Real → holding registers) and reads every other addressed signal, sensors included (discrete inputs, input registers). Int is one 16-bit register; Real is float32 in two registers, high word first. Values are live while simulating. The status bar shows the server and its clients.

---

## Hierarchy panel

- Shows parent-child tree of all objects
- **Click** an item to select it
- **Double-click** to rename
- **Right-click** → Rename, Duplicate, **Add Sensor Here…**, Delete

---

## Save / Load

- **Ctrl+S** — save (prompts for filename on first save)
- **Ctrl+O** — open project
- **Ctrl+N** — new project
- Format: `.mstudio` (JSON)

---

## Joint axis cheat sheet

| Axis value | Effect |
|------------|--------|
| `0, 1, 0` | Spin around Y (vertical) — default |
| `1, 0, 0` | Tilt around X |
| `0, 0, 1` | Tilt around Z |
| `1, 0, 0` (prismatic) | Slide left/right |
| `0, 1, 0` (prismatic) | Slide up/down |
| `0, 0, 1` (prismatic) | Slide forward/back |
