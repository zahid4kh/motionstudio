# MotionStudio Kotlin API

Control MotionStudio robots and stations from Kotlin, or any JVM language (JVM 17+). It has no runtime dependencies.

```kotlin
dependencies { implementation("io.github.zahid4kh:motionstudio:0.3.0") }
```

Or build it from this folder: `./gradlew publishToMavenLocal`, then add `mavenLocal()` to your repositories.

API reference: https://zahid4kh.github.io/motionstudio/ (made by Dokka from the KDoc comments: `./gradlew dokkaGenerate` writes it to `build/dokka/html/`).

```kotlin
import io.motionstudio.MotionStudio
import io.motionstudio.Pose

fun main() = MotionStudio.connect().use { studio ->        // localhost:5800
    val robot = studio.robot("Robot")
    println(robot.pose())
    robot.moveL(z = 100)                                  // straight line, only z changes
    robot.moveBy(y = -50, speed = 100)                    // relative, 100 mm/s
    robot.moveJoints("A1" to 30, "A2" to -10)
    robot.home()
}
```

Numbers can be `Int` or `Double` (`z = 100`). Units are mm, degrees and seconds, and Y is up. See [../README.md](../README.md) and [../PROTOCOL.md](../PROTOCOL.md).

## Reference

| `Studio` | |
|---|---|
| `MotionStudio.connect(host, port, timeoutMs)` | `Closeable`: use it with `use { }` |
| `robots()`, `robot(name = null)`, `actuator(name)` | |
| `humanoid(name = null)` | A robot with legs → `Humanoid`; without a name the only one |
| `signals["Start"] = true`, `signals["Box.count"]`, `signals.list()` | Signals; also `get`/`set` |
| `waitUntil(condition, timeout = null)` | Waits for an expression |
| `play()`, `pause()`, `reset()`, `sleep(s)`, `time`, `running`, `speed` (var) | Simulation |

| `Robot` | |
|---|---|
| `name`, `path`, `jointNames`, `toolNames`, `joints`, `poses`, `jointInfo()` | |
| `moveJoints(vararg "A1" to 30, speed, relative, wait)` / `moveJoints(map, …)` | Joint moves (speed in %) |
| `moveToPose(name)`, `home()`, `stop()` | |
| `tool(name = null, base = null)` | → `Tool` |
| `pose()`, `moveL()`, `moveJ()`, `moveBy()` | The only tool's methods |

| `Humanoid` (a `Robot` that walks) | |
|---|---|
| `walk(forward = 0, sideways = 0, turn = 0, gait = Gait(), wait = true)` | mm ahead, mm to its left, degrees to its left; a turn with a distance makes an arc |
| `turn(degrees, …)`, `sideStep(mm, …)` | On the spot / sideways |
| `walkTo(x, z, heading = null, frame = "world", …)` | To a spot on the floor (mm), then face `heading` |
| `walkPath(1000 to 0, 1000 to 1000, heading = null, frame = "world", …)` | Through points (also a `List`); `frame = "robot"`: x ahead, z to its right |
| `stopWalking()` | Finishes the step under way, feet together |
| `state()` → `WalkState(place, walking, motion, step, steps, gait)` | Also `place`, `walking`, `defaultGait` |

- `Gait(stepLength, sideStep, turnPerStep, stepHeight, stepTime, speed, crouch, sway, armSwing)`: all optional (mm, degrees, s, mm/s); the rest comes from the legs.
- `Place(x, z, heading)`: where it stands; heading 0 faces +X, positive to its left.

| `Tool` | |
|---|---|
| `pose(frame = "base")` | → `Pose` |
| `moveL(target: Pose? = null, x, y, z, rx, ry, rz, speed, frame = "base", wait = true)` | Straight line (mm/s, default 250) |
| `moveJ(…)` | Joint move to the target (speed in %) |
| `moveBy(x, y, z, rx, ry, rz, speed, frame = "base", linear = true, wait = true)` | Relative; `frame = "tool"` for the tool's own axes |

- `Motion` (from `wait = false`): `await(timeout)`, `cancel()`, `state`, `done`, `progress`.
- `Actuator`: `value`, `moveTo()`, `moveBy()`, `run(speed)`, `stop()`.
- `Pose` is a data class with `offset(...)` and `copy(...)`.

Errors are `MotionStudioException` (`code`, `message`), with the subclasses `NotFoundException`, `OutOfReachException`, `OutOfLimitsException`, `MotionCancelledException`, `WaitTimeoutException`, `SimulationException`, `UnsavedChangesException` and `ConnectionException`. Calls are thread-safe.

## Building scenes (`studio.scene`)

```kotlin
val scene = studio.scene
studio.reset()                                              // changes need the simulation reset
val station = scene.importFile("station.step")              // Z-up CAD stood upright
val arm = scene.addFrame("Arm", parent = station)
scene["arm1"].setParent(arm)                                // keeps where it is (and its joint)
scene.addJoint("J1", station, arm, Vec3(0, 0, 1), frame = station, min = -170, max = 170,
               actuator = mapOf("speed" to 150))
scene.addSequence("Cycle", Ops().move("Out", "J1", 90).wait("Hold", 1).move("Back", "J1", 0), loop = true)
scene.save("station.mstudio")
```

- `Scene`: `newScene`, `open`, `save`, `info`, `check`, `importFile`, `objects`, `scene[name]`, `exists`, `add`, `addFrame`, `addBox`, `addCylinder`, `group`, `addJoint`, `joint`, `addSignal`, `addSensor`, `addPose`, `addSequence`, `sequence`, `addSource`, `addSink`, `remove`.
- `SceneObject` (found by its id, so it survives renames and moves): `path`, `name`, `type`, `position`, `world`, `poseIn`, `size`, `children`, `set(...)`, `moveTo(x = …, frame = …)`, `rename`, `setParent(Top for the top)`, `add`, `duplicate`, `remove`, `bounds` → `Box`, `features` → `Feature`s, `setBody`, `setBelt`.
- `Joint`: `info`, `value` (get/set), `actuator`, `set(...)`, `remove`. `Ops`: `move`, `moveToPose`, `run`, `stop`, `set`, `wait`, `waitFor`, `grip`, `release`, `moveL`, `add(type, …)`.
- Objects can be given as a `SceneObject` or by name, path or ref. A number is in mm, mm/s or degrees; a `String` is an expression in the automation's units (m, m/s).

## Build, examples and tests

```sh
./gradlew build
./gradlew runExample -Pexample=Square          # or TwoMotions, HumanoidWave, WalkSquare, WalkPatrol (start a server first)
./gradlew test                                 # starts ../../build/motionstudio-server (or $MOTIONSTUDIO_SERVER)
```

## Publishing to Maven Central

The group `io.github.zahid4kh` is the Central Portal namespace. To release:

1. Raise `version` in `build.gradle.kts`. A published version can never be changed.
2. Set the signing key once in `~/.gradle/gradle.properties`: `signing.gnupg.keyName=<gpg key fingerprint>`. Its public key must be on a keyserver (`gpg --keyserver keyserver.ubuntu.com --send-keys <fingerprint>`).
3. `./gradlew centralBundle` builds `build/distributions/motionstudio-<version>-bundle.zip`: the jar, sources, javadoc, POM and Gradle module, each signed and with checksums.
4. At central.sonatype.com › Publish Component, upload the zip, wait for *Validated*, then press **Publish**.
