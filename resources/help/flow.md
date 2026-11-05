# Part Sources & Sinks

A cell rarely handles just one part. A **source** keeps making new parts while the simulation runs, and a **sink** takes them away when they are done, so a line can run for as long as you like: parts arrive, get handled and leave. Count them to get throughput.

Both are in the **Part Flow** tab of the Automation panel. You can also right-click an object in the Hierarchy and choose **Make Part Source…** or **Add Part Sink Here…**.

## Sources

A source copies a **template** part. Build the part once, place it where new parts should appear (for example just above the start of a conveyor), and make it the source's template:

1. Give the part a **rigid body** in Properties, so its copies fall and ride belts.
2. Select it and press **New Source** (or right-click it, **Make Part Source…**).
3. Choose when to make copies and press **OK**.

| Setting | Meaning |
|---|---|
| **Copies** | The template: the part and everything under it |
| **When** | *Every few seconds*: a copy every *Interval* seconds, the first at the start. *Each time a condition becomes true*: a copy on each rising edge of *Trigger* |
| **Interval** | Seconds between copies. An expression, read as each copy is made |
| **Trigger** | A copy each time this becomes true |
| **Enabled** | No copies while this is false. Empty = always |
| **At most** | Stop after this many copies. *No limit* = keep going |
| **Wait until the spot is clear** | Don't make a copy while another part still overlaps the template's place, so parts never appear inside each other |

While simulating, the template itself is **out of the run**: it is hidden, sensors don't see it and it has no physics. Its place is outlined in blue, and each copy appears there as a new free part named `Box #1`, `Box #2`, …. Copies behave like any other part: they fall, ride conveyors, trigger sensors and can be gripped. They are never saved and are removed on **Reset**, and the template comes back.

A source publishes `Feeder.count` (copies made) and `Feeder.blocked` (a copy is due but the spot is taken).

## Sinks

A sink is a zone that **takes parts away** when they touch it, like the end of a line or a bin. It takes copies made by sources and free parts (rigid bodies at the top of the tree). A part held by a gripper is left alone until it is released.

| Setting | Meaning |
|---|---|
| **Mounted on** | The zone moves with this object. The object itself is never taken |
| **Use the mounted object's bounding box** | The zone is the object's bounds, or else a box of *Size* at *Offset* |
| **Takes** | *Any part*, or only copies from one source |
| **Enabled** | Takes nothing while this is false |

The zone is drawn as a red wire box. A part that is taken disappears; edit-time parts come back on **Reset**. A sink publishes `Binned.count`, the parts taken so far.

## Counting and throughput

The counts are ordinary values. Parts in the system:

```
Feeder.count - Binned.count
```

Parts per minute so far (a *Derived* Real signal):

```
time() > 0 ? Binned.count / time() * 60 : 0
```

Feed a part only when the operator asks for one: make an *Input* `Feed`, set the source to *Each time a condition becomes true* and use `Feed` as the trigger. Or feed from the line itself: `rising(PartAtEnd)`.

Pause feeding while the buffer is full (the source's *Enabled*):

```
Feeder.count - Binned.count < 5
```

The Part Flow tab shows the live counts, and the viewport overlay lists them while the simulation runs.

## An example

The template **Part Flow Cell** (File › Open Template…) is a complete line:

| Part | What it does |
|---|---|
| Source `Feeder` | A box every 4 s at the start of the belt, when the spot is clear |
| Distance sensor `Height` | A laser above the belt; `PartHeight = Height < 0.599 ? 0.6 - Height : 0` is each box's height |
| Beam `PartAtEnd` | Stops the belt when a box reaches the end |
| Position sensor `Locator` | Finds the waiting box |
| MoveL `Pick` | Puts the gripper on the box *where it stopped*, using a pose taught on the part ([Robot Tools & IK](help:kinematics)) |
| Sink `Binned` | In the bin: takes each box that lands there |

## Tips

- Copies need MuJoCo to fall and ride belts. Without it they appear and stay where they are until gripped.
- Every copy and every part a sink takes rebuilds the physics model, keeping every body's position and speed. On very large imported assemblies that rebuild takes a moment.
- A sensor that detects **A specific object** also detects that object's copies, so a sensor can watch for "any Box".
- Scrubbing the timeline shows only the parts that existed at that moment.
