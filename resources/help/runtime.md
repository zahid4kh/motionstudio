# Running & Debugging

## Play, Step, Reset

| Button | Key | Effect |
|---|---|---|
| **Play** / **Pause** | Space | Starts the automation. It runs a scan every 16 ms, like a PLC. |
| **Step** | F10 | Runs exactly one scan, then pauses. Use it to follow a sequence step by step. |
| **Reset** | | Stops everything and puts the scene back exactly as it was before Play. |

Definitions are **locked** while the simulation is active (a lock appears in the Automation header). Press **Reset** to edit again.

## Timeline: look back and export

Every scan is recorded from the first Play or Step until **Reset**: the joints, every part's position, and every signal value. While the simulation is active, a timeline bar is shown under the viewport.

- **Look back:** pause, then drag the timeline, or click anywhere on it. The scene shows that moment. The Values list, the Signals tab and the trace plot's playhead show the values at that moment too. The overlay in the viewport says that you are looking at the recording.
- **Continue:** **Live** returns to the latest scan. **Play** or **Step** also continue from the latest scan, never from the moment on display.
- **Export Video…** (while paused) renders the run, or part of it, offscreen at the **Resolution** you pick: **Full HD 1920 × 1080** by default, or 4K, 1440p, HD, square, vertical or the viewport's own size. Open panels never crop it. **Framing**: *Fit the whole scene* uses the viewport's view direction zoomed to every part; *Everything the viewport shows* keeps its camera and backs off until the whole picture fits. Selection, hover and the gizmo are left out; sensor labels stay. You can save an **MP4** video (high-quality H.264), an animated **GIF**, or a folder of **PNG** frames. MP4 and GIF need `ffmpeg` installed.

Sequence and operation states always show the latest scan. The recording holds up to 20 minutes; after that, the bar shows *FULL*.

## Record Values: a robot's motion as numbers

**Record Values** (on the Simulation toolbar, and in the Tools menu) writes what one robot does to a CSV file, a **take**: its joint values and where it stands, at every scan or at a rate you pick. Use it to analyse a motion, or to teach a model to move like it (see [Robot API](help:api)).

1. Press **Record Values**, pick the **Robot**, how often to sample (**Samples**) and the **File**.
2. Press **Record**. The simulation starts if it isn't running; make the robot move (its sequence, a Walk operation, or a program).
3. Press **Stop Recording** to save. With **Stop when it stands still again** ticked, it saves by itself half a second after the robot stops moving, so recording a walk from A to B ends when it arrives. **Reset** saves too.

Each row has the time `t` (seconds), the robot's top object `x y z rx ry rz` (mm, degrees), then one column per joint, named after its actuator (degrees, or mm for sliding joints). A robot with legs also gets `heading`, `walking`, `step`, `swing` (the foot in the air: 0 left, 1 right), `progress` through the step, `step_x step_z step_heading` (where that foot lands) and `goal_x goal_z goal_heading` (where the walk ends). The `#` lines at the top say which robot it is and the rate; spreadsheets, NumPy and pandas read the file as it is.

## Interference check

Tick **Check interference between moving parts** (under Sequences in the Runtime tab) to be warned when parts pass through each other, for example a robot clipping a fixture. It takes effect at the next Play after Reset. While two parts interfere, both are tinted red and the viewport overlay names them. The Runtime tab lists every pass, with its start time and how long it lasted.

- Only parts that move relative to each other are checked: parts on different joints, or a moving part against the static scene.
- Parts that already touch when you press Play count as assembled that way (a link in its housing), so they are not reported.
- Rigid bodies are left to the physics, which handles their contacts.
- The test uses the actual triangles. A part completely inside another, with no surfaces crossing, isn't reported.

## Before you press Play

Check the **problems indicator** in the Automation header (top right). *No problems* means every expression and reference is valid. Otherwise click it to see the list in "Where: problem" form. Play refuses to start while there are problems.

## The Runtime tab

- **Inputs:** switch Bool inputs with **OFF/ON** (latching) or **Push** (momentary), and type values into Int and Real inputs.
- **Values:** live values of every signal, actuator and sequence. Type in *Filter…* to narrow the list.
- **Trace plot:** values over time, like an oscilloscope. **Clear** restarts it.

The viewport tints the parts that are moving and lists the operations currently running in its top-left corner.

## Something doesn't move?

| Symptom | Usual cause |
|---|---|
| Nothing moves on Play | The sequence's *Start when* is false (switch `Start` ON), or there is no sequence at all. Position actuators wait to be commanded. |
| Nothing moves, although the sequence runs | The machine is already at the pose it is told to move to. With a single pose that is always the case; capture a second one (see [Poses](help:poses)). |
| A joint spins forever | It is in **Speed** mode with *Active at start* on. Switch it to **Position**. |
| A move never finishes | The target is outside the joint's limits, so the joint stops at the limit and is never *in position*. Check the limits in Properties. |
| An operation is skipped or ignored | The actuator has a **Target =** binding. Bindings override operations. |
| The sequence stops after one cycle | **Loop** is off. |
| The sequence pauses | *Enabled when* became false, for example `!Stop` while Stop is ON. |
| A joint turns the wrong way | Flip the joint axis in Properties (*Flip*), or negate the targets. |

Watching `<Actuator>.target` and `<Actuator>.inPosition` in the Runtime tab usually shows why an operation is waiting.
