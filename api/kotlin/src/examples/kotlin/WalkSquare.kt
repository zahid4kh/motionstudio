// Walks a humanoid round a square, then once more as one path.
//
// Run with a humanoid on legs (Tools › Humanoid Builder) or:
//   motionstudio-server --humanoid "Full-size"
//   ./gradlew runExample -Pexample=WalkSquare
package examples

import io.motionstudio.Gait
import io.motionstudio.MotionStudio

fun main() {
    MotionStudio.connect().use { studio ->
        val walker = studio.humanoid()            // the only robot with legs
        println("$walker walks with ${walker.defaultGait}")

        // One side at a time: walk, then a quarter turn to the left
        repeat(4) {
            walker.walk(1000)                     // mm ahead
            walker.turn(90)                       // degrees to its left
        }
        println("back at ${walker.place}")

        // The same square as one path, in its own frame (x ahead, z to its right), faster
        walker.walkPath(1000 to 0, 1000 to -1000, 0 to -1000, 0 to 0, heading = 0, frame = "robot",
                        gait = Gait(speed = 500, armSwing = 0))
        println("and again, at ${walker.place}")
    }
}
