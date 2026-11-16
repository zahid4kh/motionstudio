# Motions: record, train, play

A **motion** is something a robot did, kept in the project: its joint values and where it stood, sample by sample. You record it once, then any sequence plays it again with a **Play Motion** operation. Train a model on several motions and it plays each of them at any length, or blends two.

Everything lives in **Automation › Motions** (Ctrl+J opens the Automation panel) and is saved in the `.mstudio` file.

## 1. Record a motion

1. Press **Record** (in the Motions tab, on the Simulation toolbar, or Tools › Record Values).
2. Pick the **Robot**, how often to sample, and a **Name**. Tick **Also save it as a CSV file** if you want the numbers outside the app too.
3. Press **Record**. The simulation starts; make the robot move. Use its sequence, a Walk operation, the jog in **Actuators & Poses** or a program.
4. Press **Stop**. With **Stop when it stands still again** ticked, recording ends by itself half a second after the robot stops moving. **Reset** ends it too.

The motion appears in the Motions tab. Select it to see how long it is and which actuators it moves. Press **Play on the robot** (or double-click it) to watch it: the simulation starts and the robot plays it from where it stands. **Reset** puts everything back.

A good way to make a motion: teach a few poses in **Actuators & Poses**, put **MoveToPose** operations in a sequence, record while it runs, then delete the sequence. The recording keeps the motion.

Other things you can do in the Motions tab:

- **Import CSV…**: add takes recorded with `robot.record()` or made by your own program.
- **Export CSV…**: save a motion for spreadsheets, NumPy or pandas.
- **Robot**: give a motion to another robot with actuators of the same names.

## 2. Play it in a sequence

Add a **Play Motion** operation (see [Sequences](help:sequences)):

| Field | Meaning |
|---|---|
| **Motion** | the motion to play (a film icon), or a trained model (a light bulb) |
| **Played by** | the robot; by default the one it was recorded on |
| **Start where the robot stands** | on: the motion is moved to start at the robot's place and heading on the floor. Off: the robot goes where the motion was recorded |
| **Speed ×** | 1 plays it as recorded, 2 twice as fast, 0.5 at half speed |

The operation is done when the motion ends. Joints the motion doesn't name stay where they are, so a hand motion recorded on its own can play while the robot walks.

## 3. Train a model

Press **Train…** in the Motions tab. Pick the robot, tick the motions to learn and choose how long to train (**Quick**, **Standard**, **Thorough**). A few short motions take seconds. Training runs in the background with a progress bar, and **Cancel** stops it.

The result is a **learned model** in the list (the light bulb). It has one **clip** per motion it learned, named after the motion. A small neural network learns every joint and how the robot moves over the floor, over the clip's time. That lets it:

- play a clip **at any length**: a 2 s wave as a slow 5 s wave;
- **blend** two clips: half a wave and half a bow, or 30 % of a limp in a walk;
- play smoothly, without the jitter of a hand-made recording.

Record the same action a few times, or several different actions, and train them together. In a **Play Motion** operation, choose the model and then:

| Field | Meaning |
|---|---|
| **Clip** | which motion it learned |
| **Blend with** | a second clip to mix in (optional) |
| **Blend** | how much of the second clip, 0 to 1 (0.5 = half of each); an expression, so a signal can change it |
| **Length s** | how long it takes; empty = the clip's own length (or the blend's) |

A learned clip always starts where the robot stands. Select a model in the Motions tab to see its clips, play one, or export it as CSV.

## Which motion is which?

- A **Walk** operation (or Humanoid Patrol) uses the built-in walking planner: it computes each step as it goes, so it walks anywhere.
- A **Play Motion** of a recorded motion replays exactly what was recorded.
- A **Play Motion** of a learned model plays what the network learned from your recordings.

The viewport's overlay names the operation that is running ("playing Wave" or "walking"), so you can always tell.

## Walking somewhere new

To walk to a new spot, use a **Walk** operation, not a recording: choose **Walk to › Points on the floor**, press **Pick on floor** and click the floor in the viewport, once per point. The route is drawn as you go. **Enter** or **Esc** ends picking. **Face °** turns it at the end (0 = +X, 90 = −Z). See [Humanoid Builder](help:humanoids).

## From a program

The [Robot API](help:api) records and plays takes too (`robot.record()`, `robot.play(take)`). Its Python library also has `motionstudio.learn`, with the same animation model and a walking policy, for your own experiments.
