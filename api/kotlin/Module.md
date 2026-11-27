# Module motionstudio

Build and drive MotionStudio digital twins from Kotlin or Java over TCP: move robots in straight lines and joint moves, go to taught poses, walk humanoids, read and write signals, and build whole scenes (CAD import, joints, belts, physics, sensors, sequences, part flow).

```kotlin
implementation("io.github.zahid4kh:motionstudio:0.3.0")
```

```kotlin
MotionStudio.connect().use { studio ->
    val robot = studio.robot()                         // the scene's only robot
    val start = robot.pose()
    robot.moveL(start.offset(y = -150), speed = 300)   // 150 mm lower, in a straight line
    robot.moveBy(x = 200, speed = 200)
    robot.home()
}
```

Start with [MotionStudio.connect][io.motionstudio.MotionStudio.connect], which returns a [Studio][io.motionstudio.Studio]. Units are millimetres, degrees and seconds, with Y up. The program talks to a running MotionStudio (**Tools › Robot API**, port 5800) or to `motionstudio-server`.

# Package io.motionstudio

Everything in the library: the connection ([Studio]), robots and humanoids ([Robot], [Humanoid]), moves ([Motion], [Pose]) and scene building ([Scene], [SceneObject], [Joint], [Ops]).
