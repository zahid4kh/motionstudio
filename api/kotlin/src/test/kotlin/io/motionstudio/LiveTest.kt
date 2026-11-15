package io.motionstudio

import java.io.File
import java.net.ServerSocket
import kotlin.math.abs
import kotlin.test.AfterTest
import kotlin.test.BeforeTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertTrue

// Starts motionstudio-server with a 6-axis robot and drives it through the library
class LiveTest {
    private lateinit var server: Process
    private lateinit var studio: Studio
    private lateinit var robot: Robot

    @BeforeTest
    fun start() {
        val path = System.getProperty("motionstudio.server")
        check(File(path).exists()) { "motionstudio-server is not built at $path" }
        val port = ServerSocket(0).use { it.localPort }
        server = ProcessBuilder(path, "--robot", "6-axis 0.9", "--port", "$port", "--speed", "20")
            .redirectErrorStream(true).start()
        val first = server.inputStream.bufferedReader().readLine() ?: ""
        check("Serving" in first) { "the server did not start: $first" }
        studio = MotionStudio.connect(port = port)
        robot = studio.robot()
    }

    @AfterTest
    fun stop() {
        studio.close()
        server.destroy()
    }

    private fun near(a: Double, b: Double, tol: Double = 0.05) = assertTrue(abs(a - b) <= tol, "$a vs $b")

    @Test
    fun findsTheRobot() {
        assertEquals("Robot", robot.name)
        assertEquals(listOf("TCP"), robot.toolNames)
        assertTrue(robot.jointInfo().size >= 6)
    }

    @Test
    fun straightLinesAndJoints() {
        val start = robot.pose()
        robot.moveL(z = 100, speed = 400)
        near(robot.pose().z, 100.0)
        near(robot.pose().x, start.x)
        robot.moveBy(y = -80)
        near(robot.pose().y, start.y - 80)
        robot.moveL(start)
        near(robot.pose().z, start.z)

        robot.moveJoints("A1" to 30, "A2" to -10)
        near(robot.joints.getValue("A1"), 30.0, 1e-3)
        robot.moveJoints(mapOf("A1" to 10), relative = true)
        near(robot.joints.getValue("A1"), 40.0, 1e-3)
        robot.home()
        near(robot.joints.getValue("A1"), 0.0, 1e-3)
    }

    @Test
    fun motionsWithoutWaiting() {
        val a = robot.moveJoints("A1" to 40, speed = 30, wait = false)
        val b = robot.moveJoints("A1" to 0, wait = false)
        b.await()
        assertEquals("done", a.state)
        val threads = listOf(Thread { robot.moveJoints("A6" to 20) }, Thread { studio.sleep(0.2) })
        threads.forEach { it.start() }
        threads.forEach { it.join(10_000) }
        near(robot.joints.getValue("A6"), 20.0, 1e-3)
    }

    @Test
    fun errors() {
        assertFailsWith<OutOfReachException> { robot.moveL(x = 5000) }
        assertFailsWith<OutOfLimitsException> { robot.moveJoints("A2" to 999) }
        val e = assertFailsWith<NotFoundException> { robot.moveJoints("A9" to 1) }
        assertTrue("A1" in e.message!!)
        assertFailsWith<MotionCancelledException> {
            val m = robot.moveJoints("A1" to -170, speed = 5, wait = false)
            m.cancel()
            m.await()
        }
        assertFailsWith<WaitTimeoutException> { robot.moveJoints("A1" to 170, speed = 5, wait = false).await(0.2) }
    }

    @Test
    fun signals() {
        assertTrue("Grip" in studio.signals)
        studio.signals["Grip"] = true
        studio.waitUntil("Grip", timeout = 2)
        assertEquals(1.0, studio.signals["Grip"])
        studio.set("Grip", false)
        assertTrue(studio.sleep(0.3) > 0)
        assertEquals(0.0, studio.get("Grip"))
        assertFailsWith<WaitTimeoutException> { studio.waitUntil("Grip", timeout = 0.2) }
    }
}
