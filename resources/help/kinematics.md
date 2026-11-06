# Robot Tools & Inverse Kinematics

Joint moves (Move, MoveToPose) say where each joint goes. For a robot it is often easier to say where the **tool** goes and let the joints follow. That is inverse kinematics (IK).

A **tool** is any part that actuated joints move: the flange, the gripper, or an empty frame placed at the tool centre point (TCP). Its own frame (origin and axes) is what IK positions. To put the TCP somewhere else, add a child frame there (an empty imported group works) and use it as the tool.

To get a ready-made robot with a TCP, use **Tools › Robot Builder…**; see [Robot Builder & Analyzer](help:robots).

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

## Poses taught on a part

A normal pose is a set of joint values, so the robot always goes to the same place. When parts don't arrive in exactly the same place (a conveyor, a feeder, a sloppy fixture), teach the pose **on the part** instead. It then stores where the tool is *relative to the part*, and the robot goes there relative to wherever the part is at run time.

1. Place the part (or the template of a [source](help:flow)) where you can reach it.
2. Under **Cartesian jog**, choose the **Tool** and jog it to the grip position on the part.
3. In the Poses list, set **On part** to the part, then press **Capture**. The pose shows a crosshair icon; its tooltip says which part and tool.

**Update** re-teaches it on the same part with the same tool. **Go to** solves the joints for where the part is now.

In a **MoveL** or **MoveToPose** to such a pose, the **On part** field picks the part this time:

| On part | The robot goes to |
|---|---|
| *(the part), where it was taught* | That object, wherever it is now |
| **Detected by** a sensor | Whatever the sensor sees when the operation starts. It waits until the sensor finds something. A copy of the taught part counts, and so does a piece of one (the copy's own frame is used) |
| A fixed object | That object |

**MoveL** follows the part: the end of its straight line is recomputed every scan, so the tool meets a part that is still moving. **MoveToPose** solves the joints once, when it starts (from the pose's taught joint values, so the robot keeps its configuration), then moves all joints together. A MoveL to a pose taught on a part can leave *Tool* empty: it uses the pose's tool.

A typical vision pick: a **Position** sensor (see [Sensors & Gripping](help:sensors)) watches where parts stop, and

| # | Operation | Settings |
|---|---|---|
| 1 | WaitFor | `Locator` |
| 2 | MoveL | Pose `Pick` (taught on `Box`), On part *Detected by Locator* |
| 3 | Grip | Gripper `Gripper`, Part *Detected by AtGripper* |

The template **Part Flow Cell** runs exactly this on parts a source feeds onto a belt. The template **6-Axis Robot Cell** does the same with a six-axis arm, which also keeps the tool pointing down along its straight lines.

## Good to know

- IK starts from the joints' current values and moves them as little as it can, so the robot keeps its elbow and wrist configuration.
- Joint limits are respected. A continuous joint has no limits.
- Near a singularity (for example a straight wrist), or at the edge of the reach, the solution may be approximate. Capture poses a little away from those spots.
