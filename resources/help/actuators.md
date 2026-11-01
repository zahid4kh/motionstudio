# Joints & Actuators

You have joints and an actuator on each of them. This page covers what to set next and how to get from there to a running machine.

## 1. Pick the right mode for each actuator

| Mode | On Play | Use it for |
|---|---|---|
| **Position** | Holds still until it is given a target, then moves there and stops | Robot axes, grippers, slides, lifts, indexing tables: anything that goes *to a place* |
| **Speed** | If **Active at start** is on, spins at *Speed* from the first moment and never stops | Wheels, conveyors, fans, spindles, planets: anything that *keeps turning* |

> **Common mistake:** setting every joint of a robot arm to *Speed* with *Active at start* on. The arm then spins its joints forever as soon as you press Play. A robot arm wants **Position** mode on every axis.

## 2. Set speed and acceleration

- **Max speed** (Position mode) is how fast the joint travels to a target, in °/s or m/s. An operation can override it.
- **Accel** gives a smooth trapezoidal ramp. Leave it at *instant* for snappy motion, or use roughly 2–4× the speed for a realistic one:

```
Max speed  90 °/s
Accel     270 °/s²     (reaches full speed in a third of a second)
```

## 3. Name the actuators meaningfully

The name is how expressions refer to the actuator. `Base`, `Shoulder`, `Elbow`, `Wrist` and `Gripper` read much better than `J1`–`J6`. Renaming updates every expression that uses the old name.

Every actuator publishes these values, which you can read in any expression:

| Symbol | Meaning |
|---|---|
| `J1.pos` | current position (° or m) |
| `J1.vel` | current velocity |
| `J1.target` | commanded target |
| `J1.inPosition` | true when at its target and stopped |
| `J1.moving` | true while the velocity is non-zero |
| `J1.atMin`, `J1.atMax` | true at a joint limit |
| `J1.active` | true when the actuator is active |

## 4. Set the start position

In the middle column (**Jog joints**), drag the sliders until the machine is in its rest position, then press **Current → Init**. *Reset* always returns here, and the project saves it.

## 5. Teach poses

Jog to each place the machine has to visit and press **Capture** under *Poses*. Double-click a pose to rename it, for example `Home`, `AbovePick`, `Pick`, `AbovePlace` or `Place`. See [Poses](help:poses).

## 6. Write the program

Open the **Sequences** tab and build the motion from the poses. See [Sequences & Operations](help:sequences), or copy a ready-made program from [Recipes](help:recipes).

## Bindings: the alternative to sequences

The three **=** fields in the actuator form are evaluated *every scan* and override operations. Leave them empty unless you want the actuator wired directly to a formula.

**Target =** makes a joint follow a signal, like a servo:

```
Pick ? -45 : 0
```

**Target =** can also oscillate with time (a pendulum without a sequence):

```
30 * sin(time() * 90)
```

**Velocity =** scales speed from a Real input:

```
Speed * 0.5
```

**Active =** is an emergency stop or a run switch:

```
Run && !EStop
```

## Coupled joints

A joint can **follow** another joint instead of having its own actuator. Under **Joint** in Properties, set *Follows*, a *Ratio* and an *Offset*. This is how gears, belts, a rack and pinion, or two mirrored gripper fingers are modelled. See [Recipes](help:recipes) for typical ratios.
