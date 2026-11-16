# Getting Started

MotionStudio turns a 3D model into a working machine in five steps. Each step builds on the one before.

| Step | Where | What you do |
|---|---|---|
| 1. **Model** | File › Import STEP, or the toolbar | Bring in parts, put them together with **Tools › Assemble** (flush faces, aligned axes) and arrange the hierarchy (child parts move with their parent) |
| 2. **Joints** | Properties › *Add Joint to Parent* | Say *how* a part moves relative to its parent: revolute (turns) or prismatic (slides) |
| 3. **Actuators** | Automation › Actuators & Poses | Put a motor on each joint that the automation should drive |
| 4. **Poses** | Automation › Actuators & Poses | Jog the joints and *Capture* the positions the machine visits (at least two, e.g. `Home` and a target) |
| 5. **Sequences** | Automation › Sequences | Chain operations: move to a pose, wait, set a signal, repeat. *Sequence from Poses* builds a simple cycle in one click |

Then press **Play** (Space) and drive the machine from the **Runtime** tab.

> **Tip:** to reuse a motion a robot made (a wave, a bow, a handover), press **Record**, let it move, and play it again from any sequence with a **Play Motion** operation. **Train** in Automation › Motions teaches a model several of them, to play at any length or blended (see [Motions](help:motions)).

> **Tip:** **File › Insert Component…** (Ctrl+Shift+I) adds ready-made pieces to your scene: a gripper, a conveyor with an end sensor, a gantry, a turntable, a light barrier, parts and a bin. Each comes with its actuators and sensors.

> **Tip:** the fastest way to learn is to open a working example. Use **File › Open Template…**, load *Robotic Arm — Pick & Place*, press Play, then switch **Start** ON in the Runtime tab. Every tab shows how the pieces fit together.

## Finding your way around a big model

- **Click a part in the viewport** and the Hierarchy opens its assembly and scrolls to it, so you see where it sits in the tree.
- **Collapse assemblies** you aren't working on; they stay collapsed. **Undo** (Ctrl+Z) keeps the part you had selected.
- **See inside:** select a housing, panel or door and lower its **Opacity** in Properties (slider, or type 0–1). Fade an assembly to fade all its parts. Parts below 0.5 are clicked through, so the next click picks what's behind them. **Section** in the viewport cuts the model open instead.
- **Hide a part:** select it and press **Alt+V** (**Edit › Hide / Show**); Alt+V again shows it. Hiding an assembly hides all its parts. Hidden parts are listed dimmed in the Hierarchy with a crossed-out eye, where you can select them again; **Alt+Shift+V** shows everything. Hidden parts still move, collide and are seen by sensors when you press Play: hiding only changes the view. It is saved with the project and can be undone.

## The mental model

- **Joints** describe what is *possible*: the axis, the pivot and the limits.
- **Actuators** decide what is *driven*. A joint without an actuator never moves on Play, unless it is part of a physics body.
- **Poses** are named snapshots of joint values, the "teach points" of a robot.
- **Sequences** are the program. Each one is a list of operations, and each operation starts when the operations it comes *after* are done.
- **Signals** are variables: buttons you press (Input), flags the sequence sets (Internal) and formulas (Derived).

Nothing moves on Play unless something commands it. A **Position** actuator holds still until an operation or a binding gives it a target. Only an active **Speed** actuator runs by itself.

## Where to go next

- [Assembling parts: flush faces, aligned axes, bolts in holes](help:assembly)
- [Joints & Actuators: what to do next](help:actuators)
- [Building a sequence](help:sequences)
- [A ready-made robot from your dimensions: the Robot Builder](help:robots)
- [Copy & paste recipes](help:recipes)
