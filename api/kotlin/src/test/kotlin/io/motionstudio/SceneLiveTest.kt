package io.motionstudio

import java.io.File
import java.net.ServerSocket
import kotlin.math.abs
import kotlin.test.AfterTest
import kotlin.test.BeforeTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertNull
import kotlin.test.assertTrue

// An empty server: objects, joints and a line built through the library, run, saved and opened
class SceneLiveTest {
    private lateinit var server: Process
    private lateinit var studio: Studio

    @BeforeTest
    fun start() {
        val path = System.getProperty("motionstudio.server")
        check(File(path).exists()) { "motionstudio-server is not built at $path" }
        val port = ServerSocket(0).use { it.localPort }
        server = ProcessBuilder(path, "--port", "$port", "--speed", "20").redirectErrorStream(true).start()
        val first = server.inputStream.bufferedReader().readLine() ?: ""
        check("Serving" in first) { "the server did not start: $first" }
        studio = MotionStudio.connect(port = port)
    }

    @AfterTest
    fun stop() {
        studio.close()
        server.destroy()
    }

    private fun near(a: Double, b: Double, tol: Double = 0.01) = assertTrue(abs(a - b) <= tol, "$a is not $b")

    @Test
    fun objectsAndJoints() {
        val scene = studio.scene
        scene.newScene()
        val cell = scene.addFrame("Cell", position = Pose(x = 1000.0))
        val table = scene.addBox("Table", Vec3(800, 40, 600), parent = cell, position = Pose(y = 700.0), color = "#336699")
        assertEquals("Cell/Table", table.path)
        near(table.world.y, 700.0)
        assertEquals(Vec3(800, 40, 600), table.size)
        near(table.bounds().min.x, 600.0)
        // In another frame: a Z-up frame stood upright
        val cad = scene.addFrame("CAD", position = Pose(rx = -90.0))
        val pin = scene.addFrame("Pin", frame = cad, position = Pose(x = 10.0, z = 250.0))
        near(pin.world.y, 250.0)
        near(pin.poseIn(cad).z, 250.0)
        // A handle follows its object through a rename and moves
        pin.rename("Pin A").setParent(cell)
        assertEquals("Cell/Pin A", pin.path)
        pin.setParent(Top)
        assertEquals("Pin A", pin.path)
        near(pin.world.y, 250.0)
        pin.moveTo(z = -40)
        near(pin.position.z, -40.0)
        assertEquals(listOf("Pin A"), scene.objects("Pin*").map { it.name })

        val arm = scene.addFrame("Arm", parent = cell, position = Pose(x = 300.0, y = 800.0))
        val turn = scene.addJoint("Turn", cell, arm, Vec3(0, 1, 0), pivot = Vec3(1000, 800, 0), min = -170, max = 170,
                                  actuator = mapOf("speed" to 90))
        assertEquals("Turn", turn.actuator)
        turn.value = 90.0
        near(arm.world.z, -300.0)
        val slide = arm.add("frame", "Slide")
        val lift = scene.addJoint("Lift", arm, slide, Vec3(0, 1, 0), type = "prismatic", min = 0, max = 200)
        lift.value = 100.0
        near(slide.world.y, 900.0)
        val wheel = scene.addFrame("Wheel", parent = cell)
        val follower = scene.addJoint("Wheel", cell, wheel, Vec3(0, 0, 1), continuous = true, follows = "Turn", ratio = -2)
        assertNull(follower.actuator)
        near(follower.value, -180.0)
        assertFailsWith<NotFoundException> { scene["Nothing"] }
        assertFailsWith<MotionStudioException> { scene.addJoint("Bad", cell, arm, Vec3(0, 1, 0)) }
    }

    @Test
    fun lineRunsSavesAndOpens() {
        val scene = studio.scene
        studio.reset()
        scene.newScene()
        val line = scene.addFrame("Line")
        val belt = scene.addBox("Belt", Vec3(2000, 100, 400), parent = line, position = Pose(y = 450.0))
        belt.setBelt("Run ? 0.3 : 0", Vec3(1, 0, 0))
        val part = scene.addBox("Box", Vec3(100, 100, 100), position = Pose(x = -900.0, y = 551.0)).setBody(mass = 0.5)
        val stand = scene.addFrame("Stand", parent = line, position = Pose(x = 1200.0))
        val ram = scene.addBox("Ram", Vec3(50, 50, 50), parent = stand, position = Pose(y = 300.0))
        scene.addJoint("Ram", stand, ram, Vec3(0, 1, 0), type = "prismatic", min = 0, max = 150, actuator = mapOf("speed" to 300))
        scene.addSignal("Run", "input", value = true)
        scene.addSignal("Seen", type = "int")
        scene.addSensor("AtEnd", "beam", mount = belt, frame = "world", at = Vec3(600, 560, -300), direction = Vec3(0, 0, 1),
                        range = 600, detects = "bodies")
        scene.addPose("Up", mapOf("Ram" to 150))
        scene.addSource("Feeder", part, interval = 3, max = 3)
        scene.addSink("Out", mount = belt, at = Vec3(950, 100, 0), size = Vec3(100, 200, 400))
        val ops = Ops()
            .waitFor("Part", "AtEnd")
            .set("Count", "Seen = Seen + 1")
            .moveToPose("Raise", "Up")
            .move("Lower", "Ram", 0, speed = 600)
            .waitFor("Gone", "!AtEnd", after = listOf("Count"))
        val seq = scene.addSequence("Counter", ops, loop = true, enable = "Run")
        val got = seq["operations"] as List<*>
        assertEquals(listOf(1.0), (got[4] as Map<*, *>)["after"])
        assertEquals("0.6", (got[3] as Map<*, *>)["speed"])
        assertEquals(emptyList(), scene.check())

        val file = File.createTempFile("kotlin_line", ".mstudio")
        try {
            scene.save(file.path)
            scene.newScene()
            assertTrue(scene.objects().isEmpty())
            val info = scene.open(file.path)
            assertEquals(listOf("Counter"), info["sequences"])
        } finally {
            file.delete()
        }
        studio.sleep(12)
        assertTrue(studio["Seen"] >= 2, "seen ${studio["Seen"]}")
        assertTrue(studio["Out.count"] >= 2)
        assertFailsWith<SimulationException> { scene.addFrame("Late") }
        studio.reset()
        assertEquals("signal", scene.remove("Seen"))
    }
}

private operator fun Studio.get(signal: String) = this.get(signal)
