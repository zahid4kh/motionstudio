// Draws a 200 mm square with the tool in straight lines, then goes home.
//   motionstudio-server --robot "6-axis 0.9"      then   ./gradlew runExample -Pexample=Square
package examples

import io.motionstudio.MotionStudio

fun main() {
    MotionStudio.connect().use { studio ->
        val robot = studio.robot()                   // the scene's only robot
        val start = robot.pose()
        println("$robot tool at $start")

        robot.moveL(start.offset(y = -150), speed = 300)   // 150 mm lower
        for ((dx, dz) in listOf(0 to 200, -200 to 0, 0 to -200, 200 to 0))
            robot.moveBy(x = dx, z = dz, speed = 200)
        robot.moveL(start)
        robot.home()
        println("done at %.2f s".format(studio.time))
    }
}
