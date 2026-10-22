# Poses (teach points)

A pose stores the value of **every joint** under a name. It is the robot-programming idea of a *teach point*: you move the machine by hand once, and the program replays it.

## Teaching a pose

1. In **Jog joints**, drag the sliders (or type values) until the machine is where you want it.
2. Press **Capture**. A new pose appears and its name is ready to edit.
3. Give it a short name with no spaces, for example `Home` or `AbovePick`.

**Update** overwrites the selected pose with the current joint values. **Go to** jogs the machine to a pose so you can check it. Hover over a pose to see its stored values.

> **You need at least two poses to see motion.** A pose is only a stored position; nothing moves to it until a sequence says so. On Play the machine starts from wherever the jog sliders are, and a pose captured from there *is* that position, so moving to it is a move of zero. Capture a second one, such as `Home`, and let the sequence move between them.

## Sequence from Poses

Once you have two or more poses, press **Sequence from Poses** under the pose list. It builds a ready-to-run sequence in one click:

- named `PoseCycle` (rename it in the Sequences tab),
- one **MoveToPose** step per pose, in the order of the list, each running after the previous one,
- **Loop** on, so after the last pose it goes back to the first and repeats.

The Sequences tab opens with the new sequence selected. Press **Play** to watch it. It is an ordinary sequence, so you can edit it afterwards: set a *Speed %*, insert a **Wait** between steps, add *Start when* = `Start`, or turn Loop off for a single pass.

The list is in capture order. To visit the poses in a different order, select a step in the Sequences tab and change its **Pose**. The steps keep running 1, 2, 3, ….

Example: poses `Home`, `AbovePick`, `Pick` give

```
1  MoveToPose → Home        After: start
2  MoveToPose → AbovePick   After: 1
3  MoveToPose → Pick        After: 2
   (Loop: back to Home)
```

## Using a pose by hand

**Sequence from Poses** covers the simple cycle. For anything more, add a **MoveToPose** operation in a sequence and choose the pose. All actuated joints move at once and **arrive together**: the slowest axis sets the pace and the others are scaled to match. The operation is done when every joint is in position.

Its **Speed %** field scales the actuators' default speeds. Leave it empty for 100 %, or bind it to an input:

```
SpeedPct
```

```
SpeedPct * 0.4
```

The second form is a slow, careful approach for the last few centimetres.

## Tips

- Nothing moves on Play? Check that the machine isn't already sitting at the pose the first step moves to. Press **Go to** on a different pose, or jog away, and try again.
- Use an **approach pose** above every pick or place position (`AbovePick` → `Pick` → `AbovePick`). Straight-down moves avoid collisions.
- Joints without an actuator are stored in the pose but ignored when moving to it.
- If you later change a joint's limits, press **Go to** and then **Update** on each pose to re-teach it inside the new range.
