package io.motionstudio

import java.io.File
import java.net.ServerSocket
import kotlin.math.abs
import kotlin.test.AfterTest
import kotlin.test.BeforeTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertFalse
import kotlin.test.assertTrue

// Starts motionstudio-server with a humanoid on legs and walks it through the library
class WalkingLiveTest {
    private lateinit var server: Process
    private lateinit var studio: Studio
    private lateinit var walker: Humanoid

    @BeforeTest
    fun start() {
        val path = System.getProperty("motionstudio.server")
        check(File(path).exists()) { "motionstudio-server is not built at $path" }
        val port = ServerSocket(0).use { it.localPort }
        server = ProcessBuilder(path, "--humanoid", "Full-size", "--port", "$port", "--speed", "20")
            .redirectErrorStream(true).start()
        val first = server.inputStream.bufferedReader().readLine() ?: ""
        check("Serving" in first) { "the server did not start: $first" }
        studio = MotionStudio.connect(port = port)
        walker = studio.humanoid()
    }

    @AfterTest
    fun stop() {
        studio.close()
        server.destroy()
    }

    private fun near(a: Double, b: Double, tol: Double = 0.5) = assertTrue(abs(a - b) <= tol, "$a vs $b")
    private fun sameWay(a: Double, b: Double) = near(((a - b + 540) % 360) - 180, 0.0, 0.01)

    @Test
    fun itIsAHumanoidWithAGait() {
        assertTrue(walker.walks)
        assertTrue(studio.robot("Humanoid") is Humanoid)
        val gait = walker.defaultGait
        assertTrue(gait.stepLength!!.toDouble() > 100)
        assertTrue(gait.speed!!.toDouble() > 0)
        assertEquals(Place(), walker.place)
        assertFalse(walker.walking)
    }

    @Test
    fun walksTurnsAndSideSteps() {
        assertEquals("done", walker.walk(600).state)
        near(walker.place.x, 600.0)
        walker.turn(90)
        sameWay(walker.place.heading, 90.0)
        walker.walk(400, gait = Gait(speed = 450, armSwing = 0))      // facing -Z now
        near(walker.place.z, -400.0)
        walker.sideStep(-200)                                         // to its right: +X when facing -Z
        near(walker.place.x, 800.0)
        walker.walk(-300, gait = Gait(stepLength = 150))              // backwards in short steps
        near(walker.place.z, -100.0)
    }

    @Test
    fun pathsAndPlaces() {
        walker.walkPath(800 to 0, 800 to 800, 0 to 800, 0 to 0, heading = 0)
        val p = walker.place
        near(p.x, 0.0); near(p.z, 0.0); sameWay(p.heading, 0.0)
        walker.walkTo(300, 300, heading = 180, frame = "robot")       // x ahead, z to its right
        near(walker.place.x, 300.0); near(walker.place.z, 300.0); sameWay(walker.place.heading, 180.0)
    }

    @Test
    fun withoutWaitingAndStopping() {
        val m = walker.walk(4000, wait = false)
        studio.sleep(1.2)
        val st = walker.state()
        assertTrue(st.walking)
        assertEquals(m.id, st.motion)
        assertTrue(st.steps > 5)
        assertEquals(1, walker.stopWalking())
        m.await()
        assertEquals("done", m.state)
        assertTrue(walker.place.x < 2000)
        assertFailsWith<MotionStudioException> { walker.walk(500, gait = Gait(stepLength = 5000)) }
    }
}
