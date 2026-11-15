package io.motionstudio

import java.io.File
import java.net.ServerSocket
import kotlin.math.abs
import kotlin.test.AfterTest
import kotlin.test.BeforeTest
import kotlin.test.Test
import kotlin.test.assertFailsWith
import kotlin.test.assertTrue

// Indexing table: Press is a slide (mm). Press.pos stays in metres.
class ActuatorLiveTest {
    private lateinit var server: Process
    private lateinit var studio: Studio

    @BeforeTest
    fun start() {
        val path = System.getProperty("motionstudio.server")
        val template = System.getProperty("motionstudio.template")
        check(File(path).exists()) { "motionstudio-server is not built at $path" }
        val port = ServerSocket(0).use { it.localPort }
        server = ProcessBuilder(path, template, "--port", "$port", "--speed", "20")
            .redirectErrorStream(true).start()
        val first = server.inputStream.bufferedReader().readLine() ?: ""
        check("Serving" in first) { "the server did not start: $first" }
        studio = MotionStudio.connect(port = port)
    }

    @AfterTest
    fun stop() {
        studio.close()
        server.destroy()
    }

    @Test
    fun slideIsMillimetresAndTableIsDegrees() {
        val press = studio.actuator("Press")
        val table = studio.actuator("Table")
        assertTrue(abs(press.value) < 1e-6)   // readable before the simulation runs
        assertFailsWith<SimulationException> { studio.signals["Press.pos"] }
        press.moveTo(250)
        table.moveTo(90)
        assertTrue(abs(press.value - 250) < 1e-3, "${press.value}")
        assertTrue(abs(studio.signals["Press.pos"] - 0.25) < 1e-6)
        assertTrue(abs(table.value - 90) < 1e-3, "${table.value}")
        assertTrue(abs(studio.signals["Table.pos"] - 90) < 1e-3)
        assertFailsWith<NotFoundException> { studio.actuator("Nobody").value }
    }
}
