# Robot Builder & Analyzer

**Tools › Robot Builder…** (Ctrl+Shift+R, or **Robot** on the toolbar) builds a complete, working robot from a few dimensions. **Tools › Analyze Robot…** measures any robot in the scene: its workspace, its Denavit–Hartenberg parameters, and how close it is to a singularity.

## Building a robot

1. Pick a **robot type**: SCARA, Palletizer or Articulated.
2. Pick a **preset**. Each one sets the dimensions, axis ranges, speeds, end effector and look of a typical robot.
3. Change whatever you like. The **Drawing** tab follows every edit. Point at a dimension to find it on the drawing, or click a value on the drawing to edit it.
4. Press **Create Robot**. The robot is added at the centre of the view, standing on the floor, and selected.

The builder remembers the last robot you created and opens with it next time.

## Robot types

Each robot faces **+X** when it is at its Home pose and turns about the vertical **Y** axis at its base.

| Type | Axes | Positive direction |
|---|---|---|
| **SCARA** (4 axes) | J1 turns arm 1; J2 turns arm 2; J3 lowers the quill (Z stroke); J4 turns the tool | J1, J2 and J4 turn anticlockwise seen from above; J3 + goes down |
| **Palletizer** (4 axes) | J1 base; J2 lower arm; J3 upper arm; J4 turns the tool | J2 + leans forward; J3 + tilts the upper arm down from level |
| **Articulated** (6 axes) | A1 base; A2 lower arm; A3 forearm; A4 forearm roll; A5 wrist bend; A6 flange roll | A2 + leans forward; A3 + and A5 + bend down |

On a palletizer, parallel links keep the elbow and the wrist level. MotionStudio models them as joints that follow J2 and J3. Their names end in **Level** (`J2 Level`, `J3 Level`), they have no actuator, and they are drawn as hollow circles. So J3 is the upper arm's angle to the horizontal, and the flange always faces down.

The articulated robot is built with its wrist bent down (A5 = 90°), so the tool points at the floor.

## Dimensions

All lengths are in millimetres.

| Type | Symbol | Measured from … to … |
|---|---|---|
| SCARA | **H** | the floor to the underside of arm 1 |
| | **L1** | the J1 axis to the J2 axis |
| | **L2** | the J2 axis to the quill (J3/J4) axis |
| | **S** | the Z stroke: how far J3 lowers the flange (J3's range) |
| | **Fz** | the floor to the flange at the top of the stroke |
| Palletizer, Articulated | **h0** | the floor to the mounting face (a pedestal; 0 = on the floor) |
| | **h1** | the mounting face to the shoulder axis (J2 / A2) |
| | **a1** | the base axis to the shoulder axis, horizontally |
| | **a2** | the shoulder axis to the elbow axis (lower arm) |
| Palletizer | **a3** | the elbow axis to the wrist axis (upper arm) |
| | **d5** | the wrist axis down to the flange |
| Articulated | **a3** | the A3 axis to the forearm (A4) axis, across the forearm (elbow offset) |
| | **d4** | the A3 axis to the wrist centre, along the forearm |
| | **d6** | the wrist centre to the flange face |

If a value makes the robot impractical, it is listed in orange under the form. For example, a SCARA flange that would go below the floor at full stroke (Fz − S < 0). Values out of range are clamped.

## Axes

The **Axes** tab sets each axis's **Min** and **Max**, its **Home** value and its actuator's **Speed** and **Acceleration**. The robot is built at its Home values, and they are saved as the pose `<name> Home`. A SCARA's J3 range always follows the stroke S. The **Axes** field next to the name sets the actuators' prefix (`A` → A1 … A6).

## End effectors

The tool sits on the flange. Its **TCP** (tool centre point) frame is where it works, and its **−Y** axis points the way the tool faces. **Tool size** scales the tool beyond the size that suits the robot.

| Tool | Adds | TCP |
|---|---|---|
| Bare flange | — | the flange centre |
| Suction cup | input `Vacuum`, zone sensor `AtCup` | the cup's lip |
| Vacuum pad (4 cups) | input `Vacuum`, zone sensor `AtPad` | the cups' lips |
| Magnetic gripper | input `Magnet`, zone sensor `AtMagnet` | the pole face |
| Parallel finger gripper | actuator `Fingers` (the second finger is coupled), input `Grip`, zone sensor `InGripper` | between the fingertips |
| Three-finger gripper | actuator `Jaws` (two fingers coupled), input `Grip`, zone sensor `InGripper` | between the fingers |
| Angular gripper | actuator `Jaws` in degrees (one jaw coupled), input `Grip`, zone sensor `InGripper` | between the jaws |
| Fork / bag gripper | actuator `Clamp`, input `Grip`, zone sensor `OnFork` | on the tines |
| Drill | speed actuator `Spindle` (runs while input `SpindleOn` is on) | the bit tip |
| Screwdriver | speed actuator `Screw`, input `ScrewOn` | the bit tip |
| Deburring spindle | speed actuator `Spindle`, input `SpindleOn` | the ball cutter's centre |
| MIG welding torch | input `Weld` | the wire tip, pointing along the 25° nozzle |
| Glue dispenser | input `Dispense` | the needle tip |
| Spray gun | input `Spray` | the spray spot, 0.2 m in front of the nozzle |
| Laser cutting head | input `Laser` | the focal point |
| Vision camera | distance sensor `Range` along the view | the working distance in front of the lens |

A gripper's actuator target is bound to `Grip`. For example, `Fingers` has `Grip ? 0.03 : 0`, so switching **Grip** on in the Runtime tab closes the fingers. The zone sensors see parts with a rigid body. To pick a part, use a **Grip** operation with the tool's top part (*Parallel finger gripper*, *Suction cup* …) as the gripper and the zone sensor as the part's source; see [Sensors & Gripping](help:sensors).

When a name is already used in the scene, the new robot's gets a suffix: a second robot's actuators are `A1_2` …, and its gripper input is `Grip_2`, with the bindings updated to match.

## Appearance

| Style | Look |
|---|---|
| Industrial | Cast, boxy links and a turning base with a shoulder yoke |
| Cobot (rounded) | Round links and joint housings with coloured rings |
| Slim (lightweight) | Thin rods and ball joints |
| Heavy-duty | Thicker links, a counterbalance cylinder and a bolted base plate |

**Colours** sets the paint and the joint colour; **Link thickness** makes the links thicker or thinner. Neither changes the kinematics.

## The drawing

**Side** shows the robot from the front (X to the right, Y up); **Top** looks down (X to the right, Z down). Blue lines are the dimensions you entered, measured on the robot that will be built. The line through the joints is the kinematic chain:

| Mark | Meaning |
|---|---|
| ● | an axis that turns in the drawing's plane (its axis points at you) |
| ▭ on a dashed line | an axis that turns about the dashed line |
| ■ | an axis that slides |
| ○ | a joint that follows another one |
| ⊕ TCP | the tool centre point |

Wheel zooms, dragging pans, and a double-click fits the drawing again.

## Analysis

The **Analysis** tab (and **Tools › Analyze Robot…**) samples tens of thousands of axis combinations within their ranges and shows:

- **Workspace**: the shaded area is everywhere the TCP reaches, seen from the side (turned about the base axis) or from the top. It also gives the max and min reach from the base axis, the height range, the volume and how far the base turns.
- **Current pose**: where the TCP is and its **dexterity**, how evenly it can move in every direction (0 % = singular). Any **singularities** and axes at their limits are listed, for example: *Wrist singularity: A4 and A6 are in line (A5 ≈ 0°)*, *Arm stretched out at J2*, *Shoulder singularity: the wrist centre is on the A1 axis*.
- **Axes**: each axis's range, current value, speed, the time for a 90° (or 100 mm) move, and its **DH parameters** (a, α, d). These are measured from the model's joint axes, so they confirm the dimensions: a 6-axis robot shows a1, a2 and a3 in column *a* of A1 to A3, d4 in column *d* of A4, and d6 plus the tool's length for A6.

## Analyzing any robot

Select a robot (any part of it, or its TCP) and choose **Tools › Analyze Robot…**, or right-click it in the Hierarchy and choose **Analyze Robot…**. This works for robots from the builder, templates, URDF/MJCF imports or your own joints. The analyzer uses the object named **TCP** below the selection (or the deepest part with the most actuated joints above it); the **Tool** list switches between robots. The pose, dexterity and warnings follow the robot while you jog it or the simulation runs. **Recompute** samples the workspace again after you change the robot.

## Using the robot

- Jog it in **Automation › Actuators & Poses** (Ctrl+J), including Cartesian jog with the TCP as the tool; see [Robot Tools & IK](help:kinematics).
- Capture poses and build a sequence: **MoveToPose** to get close, **MoveL** with the TCP for straight approaches, **Set** `Grip` (or `Vacuum`) and **Grip** / **Release** to carry parts.
- Turn a spindle on with its input (`SpindleOn`) from a **Set** operation, or from the Runtime tab.

For a humanoid, use **Tools › Humanoid Builder…** instead; see [Humanoid Builder](help:humanoids).
