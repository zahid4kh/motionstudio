// A humanoid patrols through waypoints while the program watches it, and
// stops gracefully (finishing its step) half way round the third lap.
//
// Run with a humanoid on legs (Tools › Humanoid Builder) or:
//   motionstudio-server --humanoid "Full-size"
//   ./gradlew runExample -Pexample=WalkPatrol
package examples

import io.motionstudio.Gait
import io.motionstudio.MotionStudio

fun main() {
    MotionStudio.connect().use { studio ->
        val walker = studio.humanoid()
        val waypoints = listOf(1500 to 0, 1500 to -1200, 0 to -1200, 0 to 0)
        val gait = Gait(stepLength = 280, speed = 450)
        for (lap in 1..3) {
            val walk = walker.walkPath(waypoints, heading = 0, gait = gait, wait = false)
            while (!walk.done) {
                val s = walker.state()
                println("lap $lap: step ${s.step}/${s.steps} at ${s.place}")
                studio.sleep(0.5)
                if (lap == 3 && s.step > s.steps / 2) {
                    walker.stopWalking()
                    println("stopping")
                    break
                }
            }
            walk.await()
        }
        println("stands at ${walker.place}")
    }
}
