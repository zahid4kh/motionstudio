// Starts motions without waiting, waits for them, and stops one half-way.
package examples

import io.motionstudio.MotionStudio

fun main() {
    MotionStudio.connect().use { studio ->
        val robot = studio.robot()
        val a1 = robot.jointNames.first()
        val turn = robot.moveJoints(a1 to 45, speed = 40, wait = false)
        val lift = robot.moveBy(y = 50, wait = false)   // queued: it shares joints with the turn
        lift.await()
        println("turn ${turn.state}, lift ${lift.state}")

        robot.moveJoints(a1 to -45, wait = false)
        studio.sleep(0.5)                                // simulation seconds
        println("stopped ${robot.stop()} at ${robot.joints[a1]}")
    }
}
