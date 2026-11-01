# Robot Tools & Inverse Kinematics

Joint moves (Move, MoveToPose) say where each joint goes. For a robot it is often easier to say where the **tool** goes and let the joints follow. That is inverse kinematics (IK).

A **tool** is any part that actuated joints move: the flange, the gripper, or an empty frame placed at the tool centre point (TCP). Its own frame (origin and axes) is what IK positions. To put the TCP somewhere else, add a child frame there (an empty imported group works) and use it as the tool.

IK uses the joints that have an **actuator**, from the tool up to the root. A chain of 6 or more joints keeps the tool's orientation too. Shorter chains (a SCARA, a gantry, a 3-axis arm) position the tool only.

## Cartesian jog

In **Actuators & Poses**, under the joint sliders:

1. Choose the **Tool**. The part with the most joints above it is chosen for you.
2. Pick a step (1 to 50 mm), then press **X− X+ Y− Y+ Z− Z+** to move the tool along the world axes. Hold a button to keep moving.
3. **Keep orientation** holds the tool's angle while it moves (6+ joints).

The line below shows where the tool is. If a move would leave the robot's reach or pass a joint limit, the robot stays put and the status bar says so. Jog to each place, then **Capture** a pose there, as usual.

## Drag the tool in the viewport

Turn on **IK** at the top of the viewport, select the tool part, and drag an arrow of its gizmo. The joints follow instead of the part being moved. Turn **IK** off to move parts by their transform again. IK is off while a simulation runs.

## MoveL: straight-line moves

**MoveL** moves a tool in a straight line, at a constant speed, to where it is in a pose:

| Setting | Meaning |
|---|---|
| **Tool** | The part whose frame follows the line |
| **Pose** | The target. The tool goes where it would be with the pose's joint values |
| **Speed m/s** | Tool speed along the line (default 0.25). The move eases in and out |

The joints follow by IK every scan. The orientation turns smoothly from start to end (6+ joints). MoveL finishes when the tool arrives. If part of the line is out of reach, the Runtime tab shows a warning.

Use **MoveToPose** to get somewhere quickly, and **MoveL** where the path matters, such as the approach and retract around a pick:

| # | Operation | Settings |
|---|---|---|
| 1 | MoveToPose | `AbovePick` |
| 2 | MoveL | Tool `Gripper`, Pose `Pick`, Speed `0.1` |
| 3 | Grip | Gripper `Gripper`, Part *Detected by AtGripper* |
| 4 | MoveL | Tool `Gripper`, Pose `AbovePick`, Speed `0.1` |

## Good to know

- IK starts from the joints' current values and moves them as little as it can, so the robot keeps its elbow and wrist configuration.
- Joint limits are respected. A continuous joint has no limits.
- Near a singularity (for example a straight wrist), or at the edge of the reach, the solution may be approximate. Capture poses a little away from those spots.
