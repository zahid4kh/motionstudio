# Poses (teach points)

A pose stores the value of **every joint** under a name. It is the robot-programming idea of a *teach point*: you move the machine by hand once, and the program replays it.

## Teaching a pose

1. In **Jog joints**, drag the sliders (or type values) until the machine is where you want it.
2. Press **Capture**. A new pose appears and its name is ready to edit.
3. Give it a short name with no spaces, for example `Home` or `AbovePick`.

**Update** overwrites the selected pose with the current joint values. **Go to** jogs the machine to a pose so you can check it. Hover over a pose to see its stored values.

## Using a pose

Add a **MoveToPose** operation in a sequence and choose the pose. All actuated joints move at once and **arrive together**: the slowest axis sets the pace and the others are scaled to match. The operation is done when every joint is in position.

Its **Speed %** field scales the actuators' default speeds. Leave it empty for 100 %, or bind it to an input:

```
SpeedPct
```

```
SpeedPct * 0.4
```

The second form is a slow, careful approach for the last few centimetres.

## Tips

- Use an **approach pose** above every pick or place position (`AbovePick` → `Pick` → `AbovePick`). Straight-down moves avoid collisions.
- Joints without an actuator are stored in the pose but ignored when moving to it.
- If you later change a joint's limits, press **Go to** and then **Update** on each pose to re-teach it inside the new range.
