# Sensors

A **sensor** is a Bool signal that the scene sets for you: it is true while something is in its beam or zone. Use sensors so that your logic reacts to what is happening in the scene. For example, stop the conveyor when a part reaches the end, or start the pick when a part is present.

Create one with **New Sensor** in the Signals tab, or right-click an object in the Hierarchy and choose **Add Sensor Here…**. **Configure…** opens its settings. The dialog shows what the sensor sees in the current pose.

## Beam and zone

| Type | Tests | Use it as |
|---|---|---|
| **Beam** | A ray from the sensor along a direction, up to its range | Light barrier, part-present switch, distance check |
| **Zone** | A box of a given size | Presence check, area guard |
| **Zone** with *Use the mounted object's bounding box* | The object's own bounds | Contact sensor ("the gripper touches the part") |

A sensor is **mounted** on an object and moves and turns with it. *Offset* and *Size* are in metres in that object's frame, and the object's scale is ignored. Mounted on *World*, the sensor stays fixed. A sensor never detects the object it is mounted on, that object's children, or the objects it hangs from.

**Detects** narrows what triggers the sensor:

- **Anything**: any object that has geometry.
- **Rigid bodies (parts)**: only objects with a rigid body, or objects inside a rigid-body group. This is usually what you want for parts on a conveyor.
- **A specific object**: only that object and its children.

In the viewport, a beam is drawn green while it is clear and orange while something blocks it. A zone is drawn as a wire box in the same colours. The label shows ● while the sensor is detecting something and ○ while it is not.

## Using a sensor

A sensor is read at the start of every scan, before derived signals are evaluated. You can use it anywhere you would use a Bool signal:

Stop the belt while a part is at the end (a binding on the conveyor's actuator, *Active*):

```
!PartAtEnd
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

## Tips

- Scenes with rigid bodies need MuJoCo for parts to fall and ride conveyors. Without MuJoCo, sensors still see actuated mechanisms move.
- Zones and contact sensors test bounding boxes, so a zone around a round or hollow part triggers at its box. Beams test the actual triangles.
- To sense a part on a moving carrier, mount the sensor on the carrier.
