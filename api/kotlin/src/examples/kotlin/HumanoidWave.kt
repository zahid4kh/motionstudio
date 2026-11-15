// Waves a humanoid's right hand and moves its left hand in straight lines.
//   motionstudio-server --humanoid "Full-size"   then   ./gradlew runExample -Pexample=HumanoidWave
package examples

import io.motionstudio.MotionStudio

fun main() {
    MotionStudio.connect().use { studio ->
        val humanoid = studio.robot("Humanoid")
        humanoid.home()
        humanoid.moveJoints("RShoulderRoll" to 100, "RElbow" to 60, speed = 60)
        repeat(3) {
            humanoid.moveJoints("RShoulderYaw" to 25, speed = 80)
            humanoid.moveJoints("RShoulderYaw" to -25, speed = 80)
        }
        // The left hand as a tool, moved by its arm only (not the waist)
        val hand = humanoid.tool("Left Hand TCP", base = "Torso")
        humanoid.moveJoints("LShoulderPitch" to 40, "LElbow" to 70)   // a straight arm is at full reach
        hand.moveBy(x = 60, speed = 150)
        hand.moveBy(z = -60, speed = 150)
        println("left hand at ${hand.pose()}")
        humanoid.home()
    }
}
