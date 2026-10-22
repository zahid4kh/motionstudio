# Recipes

Complete, working setups to copy. Rename the actuators, signals and poses in the examples to match your project. Each **Copy** link copies the block above it.

## Pick & place robot arm

**Signals:**

| Name | Kind | Type | Initial |
|---|---|---|---|
| `Start` | Input | Bool | false |
| `Stop` | Input | Bool | false |
| `SpeedPct` | Input | Real | 100 |
| `Gripper` | Internal | Bool | false |
| `Cycles` | Internal | Int | 0 |
| `GripperDone` | Derived | Bool | see below |

`GripperDone` expression:

```
GripL.inPosition && GripR.inPosition
```

**Actuators:** every arm axis in **Position** mode, with the gripper fingers bound to the signal. *Target =* on `GripL`:

```
Gripper ? 0.02 : 0
```

**Poses:** `Home`, `AbovePick`, `Pick`, `AbovePlace`, `Place`.

**Sequence** `PickPlace`, with Loop on:

```
Enabled when:  !Stop
Start when:    Start
```

**Operations**, each one *after* the previous:

| # | Name | Type | Settings |
|---|---|---|---|
| 1 | Home | MoveToPose | Pose `Home`, Speed % `SpeedPct` |
| 2 | Above pick | MoveToPose | Pose `AbovePick`, Speed % `SpeedPct` |
| 3 | Pick | MoveToPose | Pose `Pick`, Speed % `SpeedPct * 0.4` |
| 4 | Close gripper | Set | On start `Gripper = true` |
| 5 | Gripped | WaitFor | Until `Gripper && GripperDone` |
| 6 | Above place | MoveToPose | Pose `AbovePlace` |
| 7 | Place | MoveToPose | Pose `Place`, Speed % `SpeedPct * 0.4` |
| 8 | Open gripper | Set | On start `Gripper = false` |
| 9 | Released | WaitFor | Until `!Gripper && GripperDone`, On finish `Cycles = Cycles + 1` |

The template **File › Open Template › Robotic Arm** contains exactly this setup.

## Back-and-forth (pendulum or wiper)

One Position actuator `Swing` and a Real input `Amplitude` (initial value 30). Sequence `Osc`, with Loop on:

| # | Type | Target |
|---|---|---|
| 1 | Move | `Amplitude` |
| 2 | Move | `-Amplitude` |

Without a sequence, the same motion as a binding (*Target =* on `Swing`):

```
Amplitude * sin(time() * 90)
```

## Indexing table with a press

Actuators: `Table` (revolute, Position) and `Press` (prismatic, Position). Derived Bool `Safe`:

```
Press.pos < 0.001 && !Press.moving
```

Sequence `Index`, with Loop on and Start when `Start`:

| # | Type | Settings |
|---|---|---|
| 1 | Move | `Table`, Target `Table.target + 90`, Start when `Safe` |
| 2 | Move | `Press`, Target `0.05` |
| 3 | Wait | Duration `0.3` |
| 4 | Move | `Press`, Target `0`, On finish `Parts = Parts + 1` |

## Conveyor, fan or wheel

One **Speed**-mode actuator with *Active at start* on, so it runs on Play. To control it from inputs instead, bind it.

*Active =*

```
Run && !EStop
```

*Velocity =* (signed, so `Reverse` flips the direction)

```
Reverse ? -Speed : Speed
```

## Two-speed spindle with ramps

Speed mode, *Accel* 360 °/s², and a Bool input `Fast`. *Velocity =*

```
Fast ? 720 : 180
```

## Run a motor for a fixed time

Sequence operations on a Speed-capable actuator `Mixer`:

| # | Type | Settings |
|---|---|---|
| 1 | Run | `Mixer`, Speed `180` |
| 2 | Wait | Duration `5` |
| 3 | Stop | `Mixer` |

## Wait for a part, then act

A WaitFor on a rising edge, so a held button doesn't retrigger the operation:

```
rising(PartSensor)
```
