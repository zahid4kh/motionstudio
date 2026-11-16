# Humanoid Builder

**Tools › Humanoid Builder…** (Ctrl+Shift+U, or **Humanoid** on the toolbar) builds a humanoid robot from its body dimensions and the joints you choose. Each joint gets an actuator, so you can jog the humanoid, teach poses and drive it from sequences like any other machine.

## Building a humanoid

1. Pick a **preset**. Each one sets the body, the joints, the hands and the look of a typical humanoid: a full-size humanoid, a kid-size biped, a dual-arm worker on a pedestal, a research humanoid with every joint, a heavy-duty humanoid, a slim skeleton or a social robot.
2. Choose the **body**. **On its legs** builds a full humanoid that stands on the floor. **On a pedestal** builds only the upper body, on a column.
3. Set the **dimensions**. **Overall height** scales every dimension at once; the dimensions below set each one separately. Point at a value to find it on the drawing, or click a value on the drawing to edit it.
4. Choose the **joints** in the **Joints** tab (see below).
5. Choose the **hands**, the **style**, the colours and the thickness.
6. Press **Create Humanoid**. The humanoid is added at the centre of the view, standing on the floor, and selected.

The builder remembers the last humanoid you created and opens with it next time.

## Orientation and dimensions

The humanoid faces **+X** with **Y** up. Its left side is **−Z**. Every dimension is measured between joint axes:

| Symbol | Measured |
|---|---|
| Dh | Neck joint up to the top of the head |
| Hn | Shoulder joints up to the neck joint |
| Ws | Between the left and right shoulder joints |
| Ht | Waist joint up to the shoulder joints |
| Lu, Lf, Lh | Upper arm, forearm and hand: shoulder → elbow → wrist → fingertips |
| Hp | Hip joints up to the waist joint |
| Wh | Between the left and right hip joints |
| Lt, Ls | Thigh and shin: hip → knee → ankle |
| Ha | Sole up to the ankle joint |
| Lft, Wft | Foot length and width (the ankle sits at 30 % of the length from the heel) |
| Hb | Pedestal only: floor up to the hip joints |

The drawing has **Front**, **Side** and **Top** views. Widths and the arm appear in the front view; heights and the leg appear in the side view. Tick **Joint names** to label the joints.

## Joints

The Joints tab lists every joint the builder knows, grouped by waist, neck, arms and legs. Tick a joint to give it an actuator. A joint you leave out is rigid. **All**, **Typical** and **None** set every joint at once.

Arm and leg joints are built on both sides, named **L…** and **R…** (`LShoulderPitch`, `RShoulderPitch`). The right side's axes are mirrored, so the same value moves both sides the same way: `LShoulderRoll = 90` and `RShoulderRoll = 90` raise both arms sideways.

| Joint | Positive value |
|---|---|
| WaistYaw, WaistPitch, WaistRoll | Turns the torso to its left, bends forward, leans to its right |
| NeckYaw, NeckPitch, NeckRoll | Looks left, looks down, tilts the head to its right |
| ShoulderPitch, ShoulderRoll, ShoulderYaw | Raises the arm forward, raises it sideways, turns it outward |
| Elbow | Bends the elbow |
| WristYaw, WristPitch, WristRoll | Turns the palm forward, bends the hand forward, bends it outward |
| HipYaw, HipRoll, HipPitch | Turns the toes outward, spreads the leg, swings the leg forward |
| Knee | Bends the knee |
| AnklePitch, AnkleRoll, Toe | Lifts the toes, turns the sole outward, bends the toes up |

Joints that meet at one point (the three shoulder joints) turn in the order listed, each one carrying the next. For each joint you set its range (**Min**, **Max**), its **Home** value (the value it is built at, saved as its Home pose) and its actuator's speed and acceleration.

The humanoid's pelvis stays where it is built while you jog it: it is a kinematic model and does not balance. Lifting a leg lifts its foot off the floor. To move the whole humanoid, make it walk (see below).

## Hands

- **Mitten**: a solid hand with a thumb, and no actuators.
- **Parallel gripper**: two jaws per hand, closed by `LGrip` and `RGrip` (0 = open).
- **Five fingers**: a thumb and four two-part fingers per hand. `LHand` and `RHand` curl them all together (0 = open, 90 = a fist); the other finger joints follow it.

Each palm has a **Left Hand TCP** / **Right Hand TCP** frame, with −Y along the fingers. Use it as the tool for the Cartesian jog, the viewport's IK gizmo and **MoveL** (see [Robot Tools & IK](help:kinematics)). To pick up a part, use the hand (the **Left Hand** frame) as the gripper of a **Grip** operation (see [Sensors & Gripping](help:sensors)).

## Poses and the demo sequence

Every humanoid gets a **Home** pose. With **Demo poses and a looping sequence** ticked, it also gets the poses **T-Pose**, **Wave A**, **Wave B**, **Step L**, **Step R** and **Bow**, and a sequence (`HumanoidDemo` for a humanoid named Humanoid) that runs through them on Play and loops. A humanoid on legs then also walks out, turns round, walks back and turns to face the way it started. A pose that needs a joint the humanoid lacks is left out, and so are the step poses on a pedestal.

Every pose holds a value for every body joint, so **MoveToPose** always gives the same posture. Hand actuators are not in the poses: open and close the hands with **Move** operations. To make your own pose, jog the joints in **Automation › Actuators & Poses** and press **Capture** (see [Poses](help:poses)).

## Walking

A humanoid built **on its legs** can walk. It crouches a little, steps with alternate feet, and stands up again when it arrives. While it walks:

- the humanoid's top object is carried along, so everything on it (arms, hands, a part it holds) comes too;
- each foot is set down on the floor exactly where its step puts it and stays there until it lifts again. The back foot rolls onto its toes before it swings, and the arms swing against the legs;
- its planned footprints are drawn on the floor (the left foot blue, the right one yellow; faint once taken).

It is kinematic: the legs follow the plan and the humanoid can't fall. The floor is the height the humanoid stands at when it starts walking. **Reset** puts it back where it was.

**From a sequence**, add a **Walk** operation (see [Sequences](help:sequences)):

- **Forward**, **Sideways** and **Turn**: metres ahead, metres to its left and degrees to its left. Negative values walk backwards, step right or turn right. A turn given with a distance makes the walk an arc.
- **Walk to** an object: it walks to where the object stands on the floor and turns to face the way it faces (the object's +X). Add an empty frame as a marker, place it, and walk to it. Move the marker and the humanoid walks there instead.
- **Speed** (m/s) is optional; without it the pace comes from the size of the legs.

**From a program**, call `walk`, `turn`, `walk_to` and `walk_path` on a humanoid (see [Robot API](help:api)). A program can set the step length, the step height, the speed and the arm swing, and stop a walk gracefully after the step under way.

**File › Open Template… › Humanoid Patrol** is a humanoid that walks round four spots, waving at one and bowing at another.

**Recording it:** **Record Values** saves the humanoid's joint values, where it stands and its steps while it walks, as a CSV take (see [Running & Debugging](help:runtime)). From Python, takes like that teach a model to walk on its own (see [Robot API](help:api)).

Any model can walk if it has objects called **Left Foot** and **Right Foot** below one top object, each moved by at least a hip, a knee and an ankle joint with actuators, and its feet resting on the floor at the top object's height.
