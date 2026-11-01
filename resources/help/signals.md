# Signals

Signals are the variables of your automation. Signals, actuators and sequences share one namespace, so every name must be unique.

| Kind | Set by | Typical use |
|---|---|---|
| **Input** | You, in the Runtime tab (buttons, switches, number fields) | `Start`, `Stop`, `EStop`, `SpeedPct`, `Amplitude` |
| **Internal** | Operation actions (*On start* / *On finish*) | `Gripper`, `PartCount`, `State` |
| **Derived** | A formula, recomputed every scan | `Safe`, `GripperDone`, `AllHome` |
| **Sensor** | The scene: true while something is in its beam or zone ([Sensors](help:sensors)) | `PartPresent`, `PartAtEnd`, `Touching` |

Each signal has a type: **Bool** (true/false), **Int** (rounded) or **Real**. Sensors are always Bool.

## Examples

Derived **Bool**, true when both gripper fingers have arrived:

```
GripL.inPosition && GripR.inPosition
```

Derived **Bool** interlock, safe to index the table only when the press is up:

```
Press.pos < 0.005 && !Press.moving
```

Derived **Bool**, every axis is home:

```
abs(Base.pos) < 0.5 && abs(Shoulder.pos) < 0.5 && abs(Elbow.pos) < 0.5
```

Internal **Int** counter, as the *On finish* action of the last operation:

```
PartCount = PartCount + 1
```

Detecting a button press (true for exactly one scan):

```
rising(Start)
```

## In the Runtime tab

A **Bool** input gets two controls: **Push** (momentary, true while held) and **OFF/ON** (latching). **Int** and **Real** inputs get a number field. You can change them while the simulation runs.
