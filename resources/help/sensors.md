# Sensors & Gripping

A **sensor** is a signal that the scene sets for you: true while something is in its beam or zone, or the exact distance a laser measures. Use sensors so that your logic reacts to what is happening in the scene. For example, stop the conveyor when a part reaches the end, start the pick when a part is present, or check a part's height.

Create one with **New Sensor** in the Signals tab, or right-click an object in the Hierarchy and choose **Add Sensor Here…**. **Configure…** opens its settings. The dialog shows what the sensor sees in the current pose.

## Sensor types

| Type | Tests | Value | Use it as |
|---|---|---|---|
| **Beam** | A ray from the sensor along a direction, up to its range | Bool | Light barrier, part-present switch |
| **Zone** | A box of a given size | Bool | Presence check, area guard |
| **Zone** with *Use the mounted object's bounding box* | The object's own bounds | Bool | Contact sensor ("the gripper touches the part") |
| **Distance** | A ray like a beam | Real: metres to the first surface, the range when clear | Laser distance sensor: height, gap, position checks |
| **Position** | A zone | Bool, plus where the part is | Vision / locating sensor: where to pick |

A sensor is **mounted** on an object and moves and turns with it. *Offset* and *Size* are in metres in that object's frame, and the object's scale is ignored. Mounted on *World*, the sensor stays fixed. A sensor never detects the object it is mounted on, that object's children, or the objects it hangs from.

**Detects** narrows what triggers the sensor:

- **Anything**: any object that has geometry.
- **Rigid bodies (parts)**: only objects with a rigid body, or objects inside a rigid-body group. This is usually what you want for parts on a conveyor.
- **A specific object**: only that object and its children, and the copies a [source](help:flow) makes of it.

In the viewport, a beam is drawn green while it is clear and orange while something blocks it. A zone is drawn as a wire box in the same colours. The label shows ● while the sensor is detecting something and ○ while it is not; a distance sensor's label shows the distance in mm.

## Distance sensor

A **Distance** sensor is a laser rangefinder: a Real signal holding the distance in metres from the sensor to the first surface along its beam, or its *Range* when nothing is in range. The distance is computed exactly, not sampled: spheres, cylinders and capsules are intersected as true curved surfaces, boxes and baskets on their faces, and imported parts on their own triangles (the CAD tessellation). In a physics test with boxes riding a conveyor, every reading matched the true top face of the simulated box to within a micrometre.

Part height under a laser 0.6 m above the belt (a *Derived* Real signal):

```
Height < 0.599 ? 0.6 - Height : 0
```

Reject parts that are too short or too tall (a *Derived* Bool):

```
Height < 0.599 && abs(0.6 - Height - 0.12) > 0.002
```

## Position sensor

A **Position** sensor is a zone that also reports **where** the part it sees is, like a vision system. Besides its Bool value it publishes six Real values for the detected part's frame, in the sensor's own frame (its mount's frame moved by *Offset*; world coordinates when mounted on the world):

| Name | Meaning |
|---|---|
| `Locator.x` `Locator.y` `Locator.z` | position, m |
| `Locator.rx` `Locator.ry` `Locator.rz` | rotation, degrees (Euler XYZ, like Properties) |

They are 0 while nothing is detected. The viewport marks the found part's frame with small axes.

Use the values in logic, for example to sort by where the part lies:

```
Locator && Locator.z > 0.05
```

To have a robot go **to** the part, teach a pose on it and choose the sensor as the operation's *On part*: see [Robot Tools & IK](help:kinematics). Any sensor works for that; a Position sensor also tells your logic where the part is.

## Using a sensor

A sensor is read at the start of every scan, before derived signals are evaluated. You can use it anywhere you would use a Bool signal:

Stop the belt while a part is at the end. Select the conveyor and, under **Conveyor Belt** in Properties, set *Speed expr* (m/s):

```
PartAtEnd ? 0 : 0.4
```

Wait for a part before picking (a *WaitFor* operation, *Until*):

```
PartPresent
```

Count parts as they pass the barrier (an *Internal* counter, set in an operation's *On finish*):

```
Count = Count + 1
```

Start each cycle on a new part (the sequence's *Start when*):

```
rising(PartPresent)
```

Sensors are read-only: actions can't assign to them.

## Gripping

Two operations pick parts up and put them down:

| Operation | Settings | Does | Done when |
|---|---|---|---|
| **Grip** | *Gripper*: the object that holds; *Part*: **Detected by** a sensor, or a fixed object | Attaches the part to the gripper: from then on it moves and turns with it | the part is held. With a sensor, it waits until the sensor detects something |
| **Release** | *Gripper* | Lets go of everything the gripper holds | immediately |

A gripped rigid body ignores gravity and the conveyor. Once it is released it falls and is simulated again. Without MuJoCo, a released part stays where it was let go.

The usual setup is a **Zone** sensor mounted on the gripper, just under its pad, that detects **Rigid bodies**. A pick then reads:

| # | Operation | Settings |
|---|---|---|
| 1 | WaitFor | Until `PartAtEnd` |
| 2 | Move | Lift down to the part |
| 3 | Grip | Gripper `Gripper`, Part *Detected by AtGripper* |
| 4 | Move | Lift up |
| 5 | Move | Carriage to the drop point |
| 6 | Release | Gripper `Gripper`, On finish `Placed = Placed + 1` |

The template **Sensor & Gripper Cell** (File › Open Template…) is this cell, running in a loop. Its drop point is an expression (`0.75 + 0.15 * (Placed % 3)`), so each part lands in a different spot.

## Tips

- Scenes with rigid bodies need MuJoCo for parts to fall and ride conveyors. Without MuJoCo, sensors still see actuated mechanisms move.
- Zones and contact sensors test bounding boxes, so a zone around a round or hollow part triggers at its box. Beams and distance sensors test the actual surface.
- To sense a part on a moving carrier, mount the sensor on the carrier.
