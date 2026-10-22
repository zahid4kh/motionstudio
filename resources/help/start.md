# Getting Started

MotionStudio turns a 3D model into a working machine in five steps. Each step builds on the one before.

| Step | Where | What you do |
|---|---|---|
| 1. **Model** | File › Import STEP, or the toolbar | Bring in parts and arrange the hierarchy (child parts move with their parent) |
| 2. **Joints** | Properties › *Add Joint to Parent* | Say *how* a part moves relative to its parent: revolute (turns) or prismatic (slides) |
| 3. **Actuators** | Automation › Actuators & Poses | Put a motor on each joint that the automation should drive |
| 4. **Poses** | Automation › Actuators & Poses | Jog the joints and *Capture* the positions the machine visits (at least two, e.g. `Home` and a target) |
| 5. **Sequences** | Automation › Sequences | Chain operations: move to a pose, wait, set a signal, repeat. *Sequence from Poses* builds a simple cycle in one click |

Then press **Play** (Space) and drive the machine from the **Runtime** tab.

> **Tip:** the fastest way to learn is to open a working example. Use **File › Open Template…**, load *Robotic Arm — Pick & Place*, press Play, then switch **Start** ON in the Runtime tab. Every tab shows how the pieces fit together.

## The mental model

- **Joints** describe what is *possible*: the axis, the pivot and the limits.
- **Actuators** decide what is *driven*. A joint without an actuator never moves on Play, unless it is part of a physics body.
- **Poses** are named snapshots of joint values, the "teach points" of a robot.
- **Sequences** are the program. Each one is a list of operations, and each operation starts when the operations it comes *after* are done.
- **Signals** are variables: buttons you press (Input), flags the sequence sets (Internal) and formulas (Derived).

Nothing moves on Play unless something commands it. A **Position** actuator holds still until an operation or a binding gives it a target. Only an active **Speed** actuator runs by itself.

## Where to go next

- [Joints & Actuators: what to do next](help:actuators)
- [Building a sequence](help:sequences)
- [Copy & paste recipes](help:recipes)
