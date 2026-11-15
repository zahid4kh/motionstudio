// MotionStudio robot API for Kotlin (JVM, no dependencies).
//
//   MotionStudio.connect().use { studio ->
//       val robot = studio.robot("Robot")
//       robot.moveL(z = 100)                     // straight line; only z changes
//       robot.moveBy(y = -50)                    // 50 mm down
//       robot.moveJoints("A1" to 30)             // a joint move
//       studio.signals["Start"] = true
//   }
//
// Units are millimetres, degrees and seconds; Y is up, as in MotionStudio.
package io.motionstudio

import java.io.BufferedReader
import java.io.Closeable
import java.io.InputStreamReader
import java.io.OutputStream
import java.net.InetSocketAddress
import java.net.Socket
import java.nio.charset.StandardCharsets
import java.util.concurrent.CompletableFuture
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.ExecutionException
import java.util.concurrent.atomic.AtomicInteger

// ── Errors ──────────────────────────────────────────────────────────────────

/** An error reported by MotionStudio; [code] says what kind it is. */
open class MotionStudioException(val code: String, message: String) : RuntimeException(message) {
    override fun toString(): String = "${javaClass.simpleName}: $message ($code)"
}
/** No robot, tool, joint, pose or signal has that name (or several do). */
class NotFoundException(code: String, message: String) : MotionStudioException(code, message)
/** The tool can't get there: the target, or a point on the straight line. */
class OutOfReachException(code: String, message: String) : MotionStudioException(code, message)
/** A joint value is outside the joint's range. */
class OutOfLimitsException(code: String, message: String) : MotionStudioException(code, message)
/** The motion was stopped, cancelled by an earlier failure, or the simulation was reset. */
class MotionCancelledException(code: String, message: String) : MotionStudioException(code, message)
/** A wait ran out of time (the motion or condition carries on). */
class WaitTimeoutException(code: String, message: String) : MotionStudioException(code, message)
/** The simulation could not start, or is not running. */
class SimulationException(code: String, message: String) : MotionStudioException(code, message)
/** The connection failed or was closed. */
class ConnectionException(message: String) : MotionStudioException("connection", message)

private fun errorOf(err: Map<*, *>): MotionStudioException {
    val code = err["code"] as? String ?: "error"
    val msg = err["message"] as? String ?: code
    return when (code) {
        "not_found", "ambiguous" -> NotFoundException(code, msg)
        "out_of_reach" -> OutOfReachException(code, msg)
        "out_of_limits" -> OutOfLimitsException(code, msg)
        "cancelled", "reset" -> MotionCancelledException(code, msg)
        "timeout" -> WaitTimeoutException(code, msg)
        "simulation", "not_running" -> SimulationException(code, msg)
        else -> MotionStudioException(code, msg)
    }
}

// ── Values ──────────────────────────────────────────────────────────────────

/** A position (mm) and orientation (Euler XYZ, degrees) of a tool frame. */
data class Pose(
    val x: Double = 0.0, val y: Double = 0.0, val z: Double = 0.0,
    val rx: Double = 0.0, val ry: Double = 0.0, val rz: Double = 0.0,
) {
    constructor(x: Number, y: Number, z: Number, rx: Number = 0, ry: Number = 0, rz: Number = 0) :
        this(x.toDouble(), y.toDouble(), z.toDouble(), rx.toDouble(), ry.toDouble(), rz.toDouble())

    /** This pose with the amounts added to its values. */
    fun offset(x: Number = 0, y: Number = 0, z: Number = 0, rx: Number = 0, ry: Number = 0, rz: Number = 0) =
        Pose(this.x + x.toDouble(), this.y + y.toDouble(), this.z + z.toDouble(),
             this.rx + rx.toDouble(), this.ry + ry.toDouble(), this.rz + rz.toDouble())

    internal fun toMap() = mapOf("x" to x, "y" to y, "z" to z, "rx" to rx, "ry" to ry, "rz" to rz)

    override fun toString() =
        "Pose(x=%.2f, y=%.2f, z=%.2f, rx=%.2f, ry=%.2f, rz=%.2f)".format(x, y, z, rx, ry, rz)

    companion object {
        internal fun of(m: Map<*, *>) = Pose(m.num("x"), m.num("y"), m.num("z"), m.num("rx"), m.num("ry"), m.num("rz"))
    }
}

/** One joint of a robot: its actuator's name, range and speed. */
data class JointInfo(
    val name: String,
    val type: String,          // "revolute" or "prismatic"
    val unit: String,          // "deg" or "mm"
    val value: Double,
    val min: Double?,          // null for a continuous joint
    val max: Double?,
    val speed: Double,         // the actuator's speed (100 %), per second
    val acceleration: Double,
)

/** A signal of the automation. */
data class SignalInfo(val name: String, val kind: String, val type: String, val writable: Boolean, val value: Double)

private fun Map<*, *>.num(key: String): Double = (this[key] as? Number)?.toDouble() ?: 0.0
private fun Map<*, *>.names(key: String): List<String> =
    (this[key] as? List<*>)?.map { if (it is Map<*, *>) it["name"] as String else it as String } ?: emptyList()
private fun Map<*, *>.obj(key: String): Map<*, *> = this[key] as? Map<*, *> ?: emptyMap<String, Any>()

// ── Connection ──────────────────────────────────────────────────────────────

internal class Connection(host: String, port: Int, timeoutMs: Int) : Closeable {
    private val socket = Socket()
    private val out: OutputStream
    private val ids = AtomicInteger(1)
    private val waiting = ConcurrentHashMap<Int, CompletableFuture<Map<*, *>>>()
    @Volatile private var closed: String? = null

    init {
        try {
            socket.connect(InetSocketAddress(host, port), timeoutMs)
        } catch (e: java.io.IOException) {
            throw ConnectionException("can't reach MotionStudio at $host:$port (${e.message}). Is the app running " +
                "with Tools › Robot API switched on, or motionstudio-server started?")
        }
        socket.tcpNoDelay = true
        out = socket.getOutputStream()
        Thread(::read, "motionstudio-reader").apply { isDaemon = true; start() }
    }

    private fun read() {
        try {
            val reader = BufferedReader(InputStreamReader(socket.getInputStream(), StandardCharsets.UTF_8))
            while (true) {
                val line = reader.readLine() ?: break
                if (line.isBlank()) continue
                val msg = Json.parse(line) as? Map<*, *> ?: continue
                val id = (msg["id"] as? Number)?.toInt() ?: continue
                waiting.remove(id)?.complete(msg)
            }
        } catch (_: Exception) {
        }
        if (closed == null) closed = "the connection to MotionStudio was closed"
        val failed = ConnectionException(closed!!)
        waiting.values.forEach { it.completeExceptionally(failed) }
        waiting.clear()
    }

    fun call(method: String, params: Map<String, Any?> = emptyMap()): Map<*, *> = callAny(method, params) as? Map<*, *>
        ?: emptyMap<String, Any>()

    fun callList(method: String, params: Map<String, Any?> = emptyMap()): List<*> =
        callAny(method, params) as? List<*> ?: emptyList<Any>()

    private fun callAny(method: String, params: Map<String, Any?>): Any? {
        closed?.let { throw ConnectionException(it) }
        val id = ids.getAndIncrement()
        val future = CompletableFuture<Map<*, *>>()
        waiting[id] = future
        val line = Json.write(mapOf("id" to id, "method" to method, "params" to params.filterValues { it != null })) + "\n"
        try {
            synchronized(out) {
                out.write(line.toByteArray(StandardCharsets.UTF_8))
                out.flush()
            }
        } catch (e: java.io.IOException) {
            waiting.remove(id)
            throw ConnectionException("can't send to MotionStudio: ${e.message}")
        }
        val reply = try {
            future.get()
        } catch (e: ExecutionException) {
            throw e.cause ?: e
        }
        (reply["error"] as? Map<*, *>)?.let { throw errorOf(it) }
        return reply["result"]
    }

    override fun close() {
        if (closed == null) closed = "the connection was closed"
        try { socket.close() } catch (_: java.io.IOException) {}
    }
}

private fun target(pose: Pose?, vararg parts: Pair<String, Number?>): Map<String, Double> {
    val t = LinkedHashMap<String, Double>()
    pose?.let { t.putAll(it.toMap()) }
    for ((k, v) in parts) if (v != null) t[k] = v.toDouble()
    return t
}

// ── Motions ─────────────────────────────────────────────────────────────────

/** A motion MotionStudio is running (a move with `wait = false`). */
class Motion internal constructor(private val conn: Connection, reply: Map<*, *>) {
    val id: Int = (reply["motion"] as Number).toInt()

    /**
     * Waits until the motion ends; throws if it failed or was cancelled.
     * [timeout] is in simulation seconds; on timeout the motion carries on.
     */
    fun await(timeout: Number? = null): Motion {
        conn.call("motion.wait", mapOf("motion" to id, "timeout" to timeout?.toDouble()))
        return this
    }

    /** Stops the motion; the joints brake and hold where they are. */
    fun cancel() { conn.call("motion.cancel", mapOf("motion" to id)) }

    /** "queued", "running", "done", "failed" or "cancelled". */
    val state: String get() = conn.call("motion.status", mapOf("motion" to id))["state"] as String
    val done: Boolean get() = state in setOf("done", "failed", "cancelled")
    /** 0 … 1 */
    val progress: Double get() = conn.call("motion.status", mapOf("motion" to id)).num("progress")

    override fun toString() = "Motion($id)"
}

// ── Robots and tools ────────────────────────────────────────────────────────

/** A tool frame (TCP) of a robot: what moves to Cartesian targets. */
class Tool internal constructor(val robot: Robot, val name: String, val base: String?) {
    private val conn get() = robot.conn

    private fun params(vararg more: Pair<String, Any?>) =
        mapOf("robot" to robot.path, "tool" to name, "base" to base) + more

    /** Where the tool is: in the robot's base frame, "world" or an object's frame. */
    fun pose(frame: String = "base"): Pose = Pose.of(conn.call("tool.pose", params("frame" to frame)))

    /**
     * Moves the tool along a straight line. Give a [Pose] or any of x y z rx ry rz:
     * the values left out stay as they are (`moveL(z = 100)` only changes z).
     * [speed] in mm/s (default 250); [frame] "base", "world" or an object's name.
     */
    fun moveL(target: Pose? = null, x: Number? = null, y: Number? = null, z: Number? = null,
              rx: Number? = null, ry: Number? = null, rz: Number? = null,
              speed: Number? = null, frame: String = "base", wait: Boolean = true): Motion =
        move("tool.moveL", target(target, "x" to x, "y" to y, "z" to z, "rx" to rx, "ry" to ry, "rz" to rz),
             speed, frame, false, wait)

    /** Like [moveL], but a joint move (faster, not straight); [speed] is a percentage (default 100). */
    fun moveJ(target: Pose? = null, x: Number? = null, y: Number? = null, z: Number? = null,
              rx: Number? = null, ry: Number? = null, rz: Number? = null,
              speed: Number? = null, frame: String = "base", wait: Boolean = true): Motion =
        move("tool.moveJ", target(target, "x" to x, "y" to y, "z" to z, "rx" to rx, "ry" to ry, "rz" to rz),
             speed, frame, false, wait)

    /**
     * Moves by an offset from where the tool is, in a straight line ([linear] = false: a joint move).
     * [frame] = "tool" moves along and turns about the tool's own axes; "base" (default), "world"
     * or an object's name uses that frame's axes.
     */
    fun moveBy(x: Number = 0, y: Number = 0, z: Number = 0, rx: Number = 0, ry: Number = 0, rz: Number = 0,
               speed: Number? = null, frame: String = "base", linear: Boolean = true, wait: Boolean = true): Motion {
        val offset = mapOf("x" to x, "y" to y, "z" to z, "rx" to rx, "ry" to ry, "rz" to rz)
            .mapValues { it.value.toDouble() }.filterValues { it != 0.0 }
        return move(if (linear) "tool.moveL" else "tool.moveJ", offset, speed, frame, true, wait)
    }

    private fun move(method: String, target: Map<String, Double>, speed: Number?, frame: String,
                     relative: Boolean, wait: Boolean) =
        Motion(conn, conn.call(method, params("target" to target, "speed" to speed?.toDouble(), "frame" to frame,
                                              "relative" to relative, "wait" to wait)))

    override fun toString() = "Tool($name of ${robot.path})"
}

/** A robot or other machine in the scene: the joints below one object. */
class Robot internal constructor(internal val conn: Connection, info: Map<*, *>) {
    val name: String = info["name"] as String
    val path: String = info["path"] as String
    val jointNames: List<String> = info.names("joints")
    val toolNames: List<String> = info.names("tools")

    /** The joints' values now (degrees, or mm for sliding joints), by actuator name. */
    val joints: Map<String, Double>
        get() = conn.call("robot.joints", mapOf("robot" to path)).entries
            .associate { it.key as String to (it.value as Number).toDouble() }

    /** Every joint with its range, unit and speed. */
    fun jointInfo(): List<JointInfo> = (conn.call("robot.info", mapOf("robot" to path))["joints"] as List<*>).map {
        val j = it as Map<*, *>
        JointInfo(j["name"] as String, j["type"] as String, j["unit"] as String, j.num("value"),
                  (j["min"] as? Number)?.toDouble(), (j["max"] as? Number)?.toDouble(), j.num("speed"),
                  j.num("acceleration"))
    }

    /** Names of the poses taught for this robot. */
    val poses: List<String> get() = conn.call("robot.info", mapOf("robot" to path)).names("poses")

    /** Moves joints to values, arriving together: `moveJoints("A1" to 30, "A2" to -10, speed = 50)`. */
    fun moveJoints(vararg joints: Pair<String, Number>, speed: Number? = null, relative: Boolean = false,
                   wait: Boolean = true): Motion = moveJoints(joints.toMap(), speed, relative, wait)

    /** Moves joints to values ([relative]: by the values); [speed] is a percentage (default 100). */
    fun moveJoints(joints: Map<String, Number>, speed: Number? = null, relative: Boolean = false,
                   wait: Boolean = true): Motion =
        Motion(conn, conn.call("robot.moveJoints", mapOf(
            "robot" to path, "joints" to joints.mapValues { it.value.toDouble() },
            "speed" to speed?.toDouble(), "relative" to relative, "wait" to wait)))

    /** Moves the joints to a pose taught in MotionStudio. */
    fun moveToPose(pose: String, speed: Number? = null, wait: Boolean = true): Motion =
        Motion(conn, conn.call("robot.moveToPose",
                               mapOf("robot" to path, "pose" to pose, "speed" to speed?.toDouble(), "wait" to wait)))

    /** Moves to the home pose ("<robot> Home" or "Home"). */
    fun home(speed: Number? = null, wait: Boolean = true): Motion =
        Motion(conn, conn.call("robot.home", mapOf("robot" to path, "speed" to speed?.toDouble(), "wait" to wait)))

    /** Stops every motion of this robot; returns how many were stopped. */
    fun stop(): Int = conn.call("robot.stop", mapOf("robot" to path)).num("stopped").toInt()

    /**
     * A tool frame; without a name the robot's only tool. [base] limits the joints that move to those
     * below that object (a humanoid's arm without its waist: `tool("Right Hand TCP", base = "Torso")`).
     */
    fun tool(name: String? = null, base: String? = null): Tool {
        val n = name ?: when (toolNames.size) {
            1 -> toolNames[0]
            0 -> throw NotFoundException("not_found", "$this has no tool; give the name of an object to use")
            else -> throw NotFoundException("ambiguous",
                "${this.name} has ${toolNames.size} tools: pick one of ${toolNames.joinToString()}")
        }
        return Tool(this, n, base)
    }

    // The only tool's moves, so a one-tool robot reads naturally
    fun pose(frame: String = "base"): Pose = tool().pose(frame)
    fun moveL(target: Pose? = null, x: Number? = null, y: Number? = null, z: Number? = null,
              rx: Number? = null, ry: Number? = null, rz: Number? = null,
              speed: Number? = null, frame: String = "base", wait: Boolean = true) =
        tool().moveL(target, x, y, z, rx, ry, rz, speed, frame, wait)
    fun moveJ(target: Pose? = null, x: Number? = null, y: Number? = null, z: Number? = null,
              rx: Number? = null, ry: Number? = null, rz: Number? = null,
              speed: Number? = null, frame: String = "base", wait: Boolean = true) =
        tool().moveJ(target, x, y, z, rx, ry, rz, speed, frame, wait)
    fun moveBy(x: Number = 0, y: Number = 0, z: Number = 0, rx: Number = 0, ry: Number = 0, rz: Number = 0,
               speed: Number? = null, frame: String = "base", linear: Boolean = true, wait: Boolean = true) =
        tool().moveBy(x, y, z, rx, ry, rz, speed, frame, linear, wait)

    override fun toString() = "Robot($path, ${jointNames.size} joints)"
}

/** One actuator by name, on any machine: a door, a turntable, a drive. */
class Actuator internal constructor(private val conn: Connection, val name: String) {
    /** Its position (degrees or mm). */
    val value: Double get() = conn.call("actuator.get", mapOf("name" to name)).num("value")

    /** Moves to a position; [speed] is a percentage (default 100). */
    fun moveTo(value: Number, speed: Number? = null, wait: Boolean = true): Motion =
        Motion(conn, conn.call("robot.moveJoints",
                               mapOf("joints" to mapOf(name to value.toDouble()), "speed" to speed?.toDouble(), "wait" to wait)))

    /** Moves by an amount from where it is. */
    fun moveBy(amount: Number, speed: Number? = null, wait: Boolean = true): Motion =
        Motion(conn, conn.call("robot.moveJoints", mapOf("joints" to mapOf(name to amount.toDouble()),
                                                         "relative" to true, "speed" to speed?.toDouble(), "wait" to wait)))

    /** Runs at a speed (deg/s or mm/s, negative for reverse) until stopped. */
    fun run(speed: Number) { conn.call("actuator.run", mapOf("name" to name, "speed" to speed.toDouble())) }

    fun stop() = run(0)

    override fun toString() = "Actuator($name)"
}

/**
 * The automation's signals: `studio.signals["Start"] = true`, `studio.signals["Box.count"]`.
 * Every symbol can be read; Input and Internal signals can be written.
 */
class Signals internal constructor(private val conn: Connection) : Iterable<SignalInfo> {
    operator fun get(name: String): Double = conn.call("signal.get", mapOf("name" to name)).num("value")
    operator fun set(name: String, value: Number) { conn.call("signal.set", mapOf("name" to name, "value" to value.toDouble())) }
    operator fun set(name: String, value: Boolean) { conn.call("signal.set", mapOf("name" to name, "value" to value)) }
    operator fun contains(name: String): Boolean = any { it.name == name }

    fun list(): List<SignalInfo> = conn.callList("signal.list").map {
        val s = it as Map<*, *>
        SignalInfo(s["name"] as String, s["kind"] as String, s["type"] as String, s["writable"] == true, s.num("value"))
    }

    override fun iterator(): Iterator<SignalInfo> = list().iterator()
}

// ── Studio ──────────────────────────────────────────────────────────────────

/**
 * A connection to MotionStudio (the app with Tools › Robot API on, or motionstudio-server).
 * Calls may come from several threads. Make one with [MotionStudio.connect].
 */
class Studio internal constructor(private val conn: Connection) : Closeable {
    val version: String
    val signals = Signals(conn)

    init {
        val hello = conn.call("hello", mapOf("client" to "kotlin"))
        version = hello["version"] as? String ?: ""
        val protocol = (hello["protocol"] as? Number)?.toInt()
        if (protocol != MotionStudio.PROTOCOL) {
            conn.close()
            throw MotionStudioException("protocol",
                "MotionStudio speaks protocol $protocol, this library ${MotionStudio.PROTOCOL}: update one of them")
        }
    }

    /** Every robot and machine in the scene. */
    fun robots(): List<Robot> = conn.callList("scene.robots").map { Robot(conn, it as Map<*, *>) }

    /** A robot by name or path ("Cell/Robot"); without a name the only robot. */
    fun robot(name: String? = null): Robot {
        if (name == null) {
            val all = robots()
            return all.singleOrNull() ?: throw NotFoundException(
                if (all.isEmpty()) "not_found" else "ambiguous",
                if (all.isEmpty()) "the scene has no robot"
                else "the scene has several robots: pick one of ${all.joinToString { it.path }}")
        }
        val info = conn.call("robot.info", mapOf("robot" to name))
        return Robot(conn, mapOf("name" to info["name"], "path" to info["path"],
                                 "joints" to info["joints"], "tools" to info["tools"]))
    }

    /** An actuator by name, e.g. a door or a turntable. */
    fun actuator(name: String) = Actuator(conn, name)

    fun get(signal: String): Double = signals[signal]
    fun set(signal: String, value: Number) { signals[signal] = value }
    fun set(signal: String, value: Boolean) { signals[signal] = value }

    /**
     * Waits until an expression is true ("PartSensor && !Busy"), in MotionStudio's expression
     * language. [timeout] in simulation seconds. Returns the simulation time.
     */
    fun waitUntil(condition: String, timeout: Number? = null): Double =
        conn.call("signal.waitUntil", mapOf("condition" to condition, "timeout" to timeout?.toDouble())).num("time")

    /** Starts or resumes the simulation (moves and signal writes do this themselves). */
    fun play() { conn.call("sim.play") }
    fun pause() { conn.call("sim.pause") }
    /** Stops the simulation and puts everything back where it started. */
    fun reset() { conn.call("sim.reset") }
    /** Waits for simulation time to pass; returns the simulation time. */
    fun sleep(seconds: Number): Double = conn.call("sim.wait", mapOf("seconds" to seconds.toDouble())).num("time")

    /** Simulation time in seconds. */
    val time: Double get() = conn.call("sim.state").num("time")
    val running: Boolean get() = conn.call("sim.state")["running"] == true
    /** Simulation speed factor (1 = real time, up to 20). */
    var speed: Double
        get() = conn.call("sim.state").num("speed")
        set(v) { conn.call("sim.speed", mapOf("factor" to v)) }

    /** Closes the connection; motions that are running carry on. */
    override fun close() = conn.close()

    override fun toString() = "Studio(MotionStudio $version)"
}

object MotionStudio {
    const val DEFAULT_HOST = "localhost"
    const val DEFAULT_PORT = 5800
    const val PROTOCOL = 1

    /** Connects to MotionStudio. [timeoutMs] limits only the connection attempt. */
    @JvmStatic
    @JvmOverloads
    fun connect(host: String = DEFAULT_HOST, port: Int = DEFAULT_PORT, timeoutMs: Int = 10_000): Studio =
        Studio(Connection(host, port, timeoutMs))
}
