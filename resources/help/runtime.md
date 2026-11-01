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
- **Export Video…** (while paused) renders the run, or part of it, from the viewport as it looks now: the same camera, size and labels. You can save an **MP4** video, an animated **GIF**, or a folder of **PNG** frames. MP4 and GIF need `ffmpeg` installed.

Sequence and operation states always show the latest scan. The recording holds up to 20 minutes; after that, the bar shows *FULL*.

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
