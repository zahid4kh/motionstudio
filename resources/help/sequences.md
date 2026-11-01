# Sequences & Operations

A **sequence** is a program: a set of operations linked by *After* (predecessors). An operation starts as soon as every operation it comes after is done. That gives you plain step-by-step programs and parallel branches, as in a Gantt chart.

## Building one

1. In the **Sequences** tab, press **+** to add a sequence and name it, for example `Cycle`.
2. Press **Operation** and choose a type. A new operation is placed *after* the selected one, so adding them in order builds a chain.
3. Fill in the fields, then press **Play**.

**Shortcut:** with two or more poses captured, **Sequence from Poses** in the Actuators & Poses tab builds a looping sequence of MoveToPose steps for you. See [Poses](help:poses).

## Sequence settings

| Field | Meaning |
|---|---|
| **Enabled when** | While false, the sequence finishes its running operations but starts no new ones (a pause or stop). Empty = always. |
| **Start when** | Gate for starting a cycle. Empty = start immediately on Play. |
| **Loop** | Start a new cycle each time one completes. |

The sequence publishes `Cycle.running`, `Cycle.done` and `Cycle.cycles` for other expressions.

## Operation types

| Type | What it does | Done when |
|---|---|---|
| **Move** | One actuator to *Target* (absolute) | it is in position |
| **MoveToPose** | Every actuated joint to a pose, synchronised | all are in position |
| **MoveL** | A *Tool* in a straight line to where it is in a pose; joints by inverse kinematics ([Robot Tools & IK](help:kinematics)) | the tool has arrived |
| **Run** | Switches an actuator to Speed mode at *Speed* (signed) | immediately |
| **Stop** | Decelerates an actuator to rest | it has stopped |
| **Set** | Only runs its *On start* actions | immediately |
| **Wait** | Pauses for *Duration* seconds | the time has passed |
| **WaitFor** | Waits for the *Until* condition | the condition is true |
| **Grip** | Attaches a *Part* (fixed, or whatever a sensor detects) to a *Gripper*, so it moves with it | the part is held |
| **Release** | The *Gripper* lets go of everything it holds | immediately |

See [Sensors & Gripping](help:sensors) for a complete pick and place.

Every operation also has:

- **Start when**, an extra condition checked after its predecessors are done.
- **On start** and **On finish**, actions that write signals.

Actions are assignments separated by `;`:

```
Gripper = true; Cycles = Cycles + 1
```

## Relative moves

*Target* is an expression, so a relative move is just the current target plus an offset:

```
Table.target + 90
```

## Parallel branches

Make two operations come **After** the same one and they run at the same time. Make the next operation come after *both* (tick both in its *After* menu) to join the branches again:

```
Gripped ──┬── Lift        (Move Shoulder → -40)
          └── Turn wrist  (Move Wrist → 90)
                  └── Above place   (After: Lift + Turn wrist)
```

## Start, stop and loop with inputs

A typical machine cycle uses two Input signals, `Start` and `Stop`:

```
Enabled when:  !Stop
Start when:    Start
Loop:          on
```

Press Play, then switch **Start** ON in the Runtime tab. Switching **Stop** ON lets the current step finish and then holds.

See [Recipes](help:recipes) for complete programs to copy.
