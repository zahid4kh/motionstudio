// Building scenes: objects, joints, physics, signals, sensors, poses,
// sequences and part flow, plus opening, saving and importing models.
//
//   val scene = studio.scene
//   studio.reset()                                  // changes need the simulation reset
//   scene.newScene()
//   val cell = scene.addFrame("Cell")
//   val table = scene.addBox("Table", Vec3(800, 40, 600), parent = cell, position = Pose(y = 700.0))
//   val arm = scene.addFrame("Arm", parent = cell, position = Pose(x = 300.0, y = 800.0))
//   scene.addJoint("Turn", cell, arm, Vec3(0, 1, 0), min = -170, max = 170)
//   scene.save("cell.mstudio")
//
// Positions are in the parent's frame unless `frame` names another ("world"
// or an object). Objects are given as a SceneObject or by name, path or ref
// ("#12"). A number given for an automation value is in mm, mm/s or degrees;
// a String is an expression in the automation's own units (m, m/s, degrees).
package io.motionstudio

import java.io.File

/** A point or a direction (mm for points). */
data class Vec3(val x: Double = 0.0, val y: Double = 0.0, val z: Double = 0.0) {
    constructor(x: Number, y: Number, z: Number) : this(x.toDouble(), y.toDouble(), z.toDouble())

    operator fun plus(o: Vec3) = Vec3(x + o.x, y + o.y, z + o.z)
    operator fun minus(o: Vec3) = Vec3(x - o.x, y - o.y, z - o.z)
    internal fun toList() = listOf(x, y, z)

    companion object {
        internal fun of(m: Any?): Vec3 {
            val o = m as? Map<*, *> ?: return Vec3()
            return Vec3(o.d("x"), o.d("y"), o.d("z"))
        }
    }
}

/** An axis-aligned box in some frame (mm). */
data class Box(val min: Vec3, val max: Vec3, val center: Vec3, val size: Vec3)

/**
 * A round edge (`circle`), round face (`cylinder`), straight edge (`line`) or flat face (`plane`)
 * of a CAD part: where a joint's axis can come from. [origin] is the centre, [axis] the direction.
 */
data class Feature(val objectPath: String, val kind: String, val origin: Vec3, val axis: Vec3,
                   val radius: Double?, val length: Double?)

private fun Map<*, *>.d(key: String): Double = (this[key] as? Number)?.toDouble() ?: 0.0
private fun Map<*, *>.list(key: String): List<String> =
    (this[key] as? List<*>)?.map { if (it is Map<*, *>) it["name"] as String else it as String } ?: emptyList()

/** An object given as a [SceneObject], or by name, path or ref. */
private fun ref(o: Any?): String? = when (o) {
    null -> null
    is SceneObject -> o.ref
    else -> o.toString()
}
/** An automation value: a number (API units) or an expression. */
private fun value(v: Any?): Any? = when (v) {
    null -> null
    is Number -> v.toDouble()
    is Boolean -> v
    else -> v.toString()
}
private fun Pose.toPartial(): Map<String, Double> = toMap()

/** An object in the scene, found by its id ([ref]), so it stays the same object when renamed or moved. */
class SceneObject internal constructor(private val conn: Connection, info: Map<*, *>) {
    val id: Int = (info["id"] as Number).toInt()
    val ref: String = "#$id"
    var path: String = info["path"] as String
        private set

    private fun call(method: String, params: Map<String, Any?> = emptyMap()): Map<*, *> =
        conn.call(method, mapOf("object" to ref) + params).also { r -> (r["path"] as? String)?.let { path = it } }

    /** Everything about it: type, parent, children, position, world, size, colour, body, belt, joint. */
    fun info(): Map<*, *> = call("object.get")
    val name: String get() = info()["name"] as String
    /** frame, mesh, box, cylinder, sphere, capsule or basket */
    val type: String get() = info()["type"] as String
    val parentPath: String? get() = info()["parent"] as? String
    /** Where it is in its parent's frame (the world at the top). */
    val position: Pose get() = Pose.of(info()["position"] as Map<*, *>)
    val world: Pose get() = Pose.of(info()["world"] as Map<*, *>)
    fun poseIn(frame: Any): Pose = Pose.of(conn.call("object.get", mapOf("object" to ref, "frame" to ref(frame)))["inFrame"] as Map<*, *>)
    /** A box's, cylinder's or sphere's size in mm (null for frames and meshes). */
    val size: Vec3? get() = info()["size"]?.let { Vec3.of(it) }
    val children: List<SceneObject>
        get() = conn.callList("scene.objects", mapOf("under" to ref, "depth" to 1)).map { SceneObject(conn, it as Map<*, *>) }

    /**
     * Changes it: what is not null. [parent]: `Top` moves it to the top. [position] is in its parent's
     * frame unless [frame]; [color] "#rrggbb".
     */
    fun set(name: String? = null, parent: Any? = null, position: Pose? = null, frame: Any? = null, size: Vec3? = null,
            diameter: Number? = null, height: Number? = null, color: String? = null, opacity: Number? = null,
            hidden: Boolean? = null): SceneObject {
        val p = mutableMapOf<String, Any?>("name" to name, "position" to position?.toPartial(), "frame" to ref(frame),
            "size" to size?.toList(), "diameter" to diameter?.toDouble(), "height" to height?.toDouble(),
            "color" to color, "opacity" to opacity?.toDouble(), "hidden" to hidden)
        if (parent === Top) p["parent"] = JsonNull else if (parent != null) p["parent"] = ref(parent)
        call("object.set", p)
        return this
    }

    /** Puts it somewhere; values left out stay: `moveTo(z = 100)`. */
    fun moveTo(x: Number? = null, y: Number? = null, z: Number? = null, rx: Number? = null, ry: Number? = null,
               rz: Number? = null, frame: Any? = null): SceneObject {
        val pos = listOf("x" to x, "y" to y, "z" to z, "rx" to rx, "ry" to ry, "rz" to rz)
            .filter { it.second != null }.associate { it.first to it.second!!.toDouble() }
        call("object.set", mapOf("position" to pos, "frame" to ref(frame)))
        return this
    }

    fun rename(name: String) = set(name = name)
    /** Hangs it under another object ([Top]: the top), keeping where it is and its joint. */
    fun setParent(parent: Any) = set(parent = parent)
    /** Adds a child: `obj.add("box", "Plate", size = Vec3(100, 10, 100))`. */
    fun add(type: String, name: String, position: Pose? = null, frame: Any? = null, size: Vec3? = null,
            color: String? = null): SceneObject =
        Scene(conn).add(type, name, parent = this, position = position, frame = frame, size = size, color = color)
    /** Removes it and what is below it; returns how many objects went. */
    fun remove(): Int = (conn.call("object.remove", mapOf("object" to ref))["removed"] as Number).toInt()
    /** A copy with its children, joints and actuators (renamed). */
    fun duplicate(name: String? = null): SceneObject = SceneObject(conn, conn.call("object.duplicate", mapOf("object" to ref, "name" to name)))

    /** Its box (with its children's unless [children] is false) in a frame, mm. */
    fun bounds(frame: Any = "world", children: Boolean = true): Box {
        val b = conn.call("object.bounds", mapOf("object" to ref, "frame" to ref(frame), "children" to children))
        return Box(Vec3.of(b["min"]), Vec3.of(b["max"]), Vec3.of(b["center"]), Vec3.of(b["size"]))
    }

    /** The round edges and faces (or "line", "plane") of its CAD geometry. */
    fun features(kinds: List<String> = listOf("circle", "cylinder"), frame: Any = "world", minRadius: Number? = null,
                 maxRadius: Number? = null): List<Feature> {
        val r = conn.call("object.features", mapOf("object" to ref, "kinds" to kinds, "frame" to ref(frame),
            "minRadius" to minRadius?.toDouble(), "maxRadius" to maxRadius?.toDouble()))
        return (r["features"] as List<*>).map {
            val f = it as Map<*, *>
            Feature(f["object"] as String, f["kind"] as String, Vec3.of(f["origin"]), Vec3.of(f["axis"]),
                    (f["radius"] as? Number)?.toDouble(), (f["length"] as? Number)?.toDouble())
        }
    }

    /** A rigid body (falls, rides belts, can be gripped); mass in kg. */
    fun setBody(mass: Number? = null, gravity: Boolean? = null, collision: Boolean? = null): SceneObject {
        call("body.set", mapOf("mass" to mass?.toDouble(), "gravity" to gravity, "collision" to collision))
        return this
    }
    fun removeBody(): SceneObject { call("body.set", mapOf("remove" to true)); return this }

    /** A conveyor belt: [speed] mm/s or an expression in m/s; [direction] in its own frame unless [frame]. */
    fun setBelt(speed: Any? = null, direction: Vec3? = null, frame: Any? = null, enabled: Boolean? = null): SceneObject {
        call("belt.set", mapOf("speed" to value(speed), "direction" to direction?.toList(), "frame" to ref(frame),
                               "enabled" to enabled))
        return this
    }
    fun removeBelt(): SceneObject { call("belt.set", mapOf("remove" to true)); return this }

    override fun equals(other: Any?) = other is SceneObject && other.id == id
    override fun hashCode() = id
    override fun toString() = "SceneObject($path)"
}

/** Pass as `parent` to move an object to the top of the scene: `obj.setParent(Top)`. */
object Top

/** A joint, by its name (or its actuator's, or the object it moves). */
class Joint internal constructor(private val conn: Connection, name: String) {
    var name: String = name
        private set

    /** Type, parent, child, axis and pivot (in [frame]), range, value, actuator, follows. */
    fun info(frame: Any = "world"): Map<*, *> =
        conn.call("joint.get", mapOf("joint" to name, "frame" to ref(frame))).also { name = it["name"] as String }
    val actuator: String? get() = (info()["actuator"] as? Map<*, *>)?.get("name") as? String
    /** Degrees, or mm for a prismatic joint. */
    var value: Double
        get() = info().d("value")
        set(v) { set(value = v) }

    /** Changes what is not null; [actuator] true / false / a map (name, speed, acceleration, mode, target). */
    fun set(name: String? = null, axis: Vec3? = null, pivot: Vec3? = null, frame: Any? = null, min: Number? = null,
            max: Number? = null, continuous: Boolean? = null, value: Number? = null, actuator: Any? = null,
            follows: String? = null, ratio: Number = 1, offset: Number = 0): Joint {
        val r = conn.call("joint.set", mapOf("joint" to this.name, "name" to name, "axis" to axis?.toList(),
            "pivot" to pivot?.toList(), "frame" to ref(frame), "min" to min?.toDouble(), "max" to max?.toDouble(),
            "continuous" to continuous, "value" to value?.toDouble(), "actuator" to actuator,
            "follows" to follows?.let { mapOf("joint" to it, "ratio" to ratio.toDouble(), "offset" to offset.toDouble()) }))
        this.name = r["name"] as String
        return this
    }

    /** Removes it (and its actuator); the part stays where it is now. */
    fun remove() { conn.call("joint.remove", mapOf("joint" to name)) }

    override fun toString() = "Joint($name)"
}

/**
 * The operations of a sequence, in order. Each runs after the one before unless [after] names others
 * (names or 0-based indices; an empty list: none). Every operation takes onStart / onFinish actions
 * ("Count = Count + 1; Busy = true") and a startCondition.
 */
class Ops {
    internal val operations = mutableListOf<Map<String, Any?>>()

    /** Any operation type with its fields as the protocol names them. */
    fun add(type: String, name: String? = null, after: List<Any>? = null, onStart: String? = null,
            onFinish: String? = null, startCondition: String? = null, fields: Map<String, Any?> = emptyMap()): Ops {
        val op = mutableMapOf<String, Any?>("type" to type, "name" to name, "onStart" to onStart, "onFinish" to onFinish,
                                            "startCondition" to startCondition, "after" to after)
        op.putAll(fields)
        operations += op.filterValues { it != null }
        return this
    }

    fun move(name: String, actuator: String, target: Any, speed: Any? = null, after: List<Any>? = null,
             onStart: String? = null, onFinish: String? = null) =
        add("move", name, after, onStart, onFinish, fields = mapOf("actuator" to actuator, "target" to value(target), "speed" to value(speed)))
    fun moveToPose(name: String, pose: String, speed: Any? = null, after: List<Any>? = null, onStart: String? = null,
                   onFinish: String? = null) =
        add("moveToPose", name, after, onStart, onFinish, fields = mapOf("pose" to pose, "speed" to value(speed)))
    fun run(name: String, actuator: String, speed: Any, after: List<Any>? = null) =
        add("run", name, after, fields = mapOf("actuator" to actuator, "speed" to value(speed)))
    fun stop(name: String, actuator: String, after: List<Any>? = null) =
        add("stop", name, after, fields = mapOf("actuator" to actuator))
    /** Only its actions: `set("Reset", "Count = 0; Busy = false")`. */
    fun set(name: String, actions: String, after: List<Any>? = null) = add("set", name, after, onStart = actions)
    fun wait(name: String, seconds: Any, after: List<Any>? = null, onStart: String? = null, onFinish: String? = null) =
        add("wait", name, after, onStart, onFinish, fields = mapOf("seconds" to value(seconds)))
    fun waitFor(name: String, condition: String, after: List<Any>? = null, onStart: String? = null, onFinish: String? = null) =
        add("waitFor", name, after, onStart, onFinish, fields = mapOf("condition" to condition))
    /** The gripper holds what [sensor] sees (or [part]). */
    fun grip(name: String, gripper: Any, sensor: String? = null, part: Any? = null, after: List<Any>? = null) =
        add("grip", name, after, fields = mapOf("gripper" to ref(gripper), "sensor" to sensor, "part" to ref(part)))
    fun release(name: String, gripper: Any, after: List<Any>? = null) =
        add("release", name, after, fields = mapOf("gripper" to ref(gripper)))
    /** The tool in a straight line to where it is in a pose; [speed] mm/s. */
    fun moveL(name: String, tool: Any, pose: String, speed: Any? = null, after: List<Any>? = null) =
        add("moveL", name, after, fields = mapOf("tool" to ref(tool), "pose" to pose, "speed" to value(speed)))
}

/** The scene: `studio.scene`. Changes need the simulation reset; in the app they can be undone. */
class Scene internal constructor(private val conn: Connection) {
    private fun obj(r: Map<*, *>) = SceneObject(conn, r)
    private fun abs(path: String) = File(path).absolutePath

    /** An empty scene; in the app, unsaved changes need [discard]. */
    fun newScene(discard: Boolean = false) { conn.call("scene.new", mapOf("discard" to discard)) }
    /** Opens a project in place of the scene; returns [info]. */
    fun open(path: String, discard: Boolean = false): Map<*, *> = conn.call("scene.open", mapOf("path" to abs(path), "discard" to discard))
    /** Saves (to where it came from without a path); returns the path. */
    fun save(path: String? = null): String = conn.call("scene.save", mapOf("path" to path?.let(::abs)))["path"] as String
    /** Top objects, robots, joints, signals, sensors, sequences, poses, sources, sinks, motions by name. */
    fun info(): Map<*, *> = conn.call("scene.info")
    /** The automation's problems; empty when it can run. */
    fun check(): List<String> = conn.call("scene.check").list("problems")
    /** A STEP (Z-up stood upright unless [upAxis] "y"), URDF, MJCF or project file (merged in); the new top object. */
    fun importFile(path: String, parent: Any? = null, name: String? = null, upAxis: String? = null): SceneObject =
        obj(conn.call("scene.import", mapOf("path" to abs(path), "parent" to ref(parent), "name" to name, "upAxis" to upAxis))["root"] as Map<*, *>)

    /** Objects in tree order: all, or names matching a wildcard ("legs:*"), below [under], of a [type]. */
    fun objects(name: String? = null, under: Any? = null, type: String? = null, depth: Int? = null): List<SceneObject> =
        conn.callList("scene.objects", mapOf("name" to name, "under" to ref(under), "type" to type, "depth" to depth))
            .map { obj(it as Map<*, *>) }
    /** An object by name, path or ref: `scene["Station/Sorter/Lift"]`. */
    operator fun get(name: String): SceneObject = obj(conn.call("object.get", mapOf("object" to name)))
    fun exists(name: String): Boolean = try { get(name); true } catch (_: NotFoundException) { false }

    /** Adds a frame, box, cylinder, sphere, capsule or basket (sizes mm, default 100). */
    fun add(type: String, name: String? = null, parent: Any? = null, position: Pose? = null, frame: Any? = null,
            size: Vec3? = null, diameter: Number? = null, height: Number? = null, color: String? = null): SceneObject =
        obj(conn.call("object.add", mapOf("type" to type, "name" to name, "parent" to ref(parent),
            "position" to position?.toPartial(), "frame" to ref(frame), "size" to size?.toList(),
            "diameter" to diameter?.toDouble(), "height" to height?.toDouble(), "color" to color)))
    fun addFrame(name: String, parent: Any? = null, position: Pose? = null, frame: Any? = null) =
        add("frame", name, parent, position, frame)
    fun addBox(name: String, size: Vec3, parent: Any? = null, position: Pose? = null, frame: Any? = null, color: String? = null) =
        add("box", name, parent, position, frame, size = size, color = color)
    /** A cylinder standing along its own Y axis. */
    fun addCylinder(name: String, diameter: Number, height: Number, parent: Any? = null, position: Pose? = null,
                    frame: Any? = null, color: String? = null) =
        add("cylinder", name, parent, position, frame, diameter = diameter, height = height, color = color)
    /** A new frame at the middle of the objects, with them under it. */
    fun group(objects: List<Any>, name: String = "Group"): SceneObject =
        obj(conn.call("object.group", mapOf("objects" to objects.map { ref(it) }, "name" to name)))

    /**
     * A joint moving [child] on [parent]. [axis] and [pivot] (mm; default the child's origin) are in the world
     * unless [frame]. [min]/[max] degrees, or mm for type "prismatic" (needed then). It gets an actuator named after
     * it unless [actuator] is false or it [follows] another joint; [actuator] may be a map of name, speed,
     * acceleration, mode, target.
     */
    fun addJoint(name: String?, parent: Any, child: Any, axis: Vec3, type: String = "revolute", pivot: Vec3? = null,
                 frame: Any? = null, min: Number? = null, max: Number? = null, continuous: Boolean? = null,
                 value: Number? = null, actuator: Any? = null, follows: String? = null, ratio: Number = 1,
                 offset: Number = 0): Joint {
        val r = conn.call("joint.add", mapOf("name" to name, "parent" to ref(parent), "child" to ref(child),
            "axis" to axis.toList(), "type" to type, "pivot" to pivot?.toList(), "frame" to ref(frame),
            "min" to min?.toDouble(), "max" to max?.toDouble(), "continuous" to continuous, "value" to value?.toDouble(),
            "actuator" to actuator,
            "follows" to follows?.let { mapOf("joint" to it, "ratio" to ratio.toDouble(), "offset" to offset.toDouble()) }))
        return Joint(conn, r["name"] as String)
    }
    fun joint(name: String): Joint = Joint(conn, conn.call("joint.get", mapOf("joint" to name))["name"] as String)

    /** A signal: kind "input", "internal" or "derived" (an [expression]); type "bool", "int" or "real". */
    fun addSignal(name: String, kind: String = "internal", type: String = "bool", value: Any = 0,
                  expression: String? = null, description: String? = null): Map<*, *> =
        conn.call("signal.add", mapOf("name" to name, "kind" to kind, "type" to type, "value" to value,
                                      "expression" to expression, "description" to description))

    /**
     * A sensor: "beam" (from [at] along [direction] up to [range] mm), "distance", "zone" (a box of [size] at
     * [at], or the mount's box with [bounds]) or "position". It moves with [mount]; positions are in the
     * mount's frame unless [frame]. [detects]: "any", "bodies" or an object.
     */
    fun addSensor(name: String, type: String, mount: Any? = null, at: Vec3? = null, direction: Vec3? = null,
                  range: Number? = null, size: Vec3? = null, bounds: Boolean? = null, frame: Any? = null,
                  detects: Any? = null, description: String? = null): Map<*, *> =
        conn.call("sensor.add", mapOf("name" to name, "type" to type, "mount" to ref(mount), "at" to at?.toList(),
            "direction" to direction?.toList(), "range" to range?.toDouble(), "size" to size?.toList(), "bounds" to bounds,
            "frame" to ref(frame), "detects" to ref(detects), "description" to description))

    /** A pose: actuator values (degrees or mm), or where [robot] stands now. */
    fun addPose(name: String, joints: Map<String, Number>? = null, robot: Any? = null, replace: Boolean = false): Map<*, *> =
        conn.call("pose.add", mapOf("name" to name, "joints" to joints?.mapValues { it.value.toDouble() },
                                    "robot" to ref(robot), "replace" to replace))

    /** A sequence; [loop] restarts it, [enable] pauses it while false. Returns [sequence]. */
    fun addSequence(name: String, ops: Ops, loop: Boolean = false, enable: String? = null, start: String? = null,
                    replace: Boolean = false): Map<*, *> =
        conn.call("sequence.add", mapOf("name" to name, "operations" to ops.operations, "loop" to loop,
                                        "enable" to enable, "start" to start, "replace" to replace))
    /** A sequence as stored: predecessors as indices, values as expressions (automation units). */
    fun sequence(name: String): Map<*, *> = conn.call("sequence.get", mapOf("name" to name))

    /** Copies of [template] while simulating: every [interval] s, or each time [trigger] becomes true. */
    fun addSource(name: String, template: Any, interval: Any? = null, trigger: String? = null, enable: String? = null,
                  max: Int? = null, whenClear: Boolean? = null): Map<*, *> =
        conn.call("source.add", mapOf("name" to name, "template" to ref(template), "interval" to value(interval),
            "trigger" to trigger, "mode" to (if (trigger != null) "trigger" else null), "enable" to enable, "max" to max,
            "whenClear" to whenClear))
    /** Takes away the parts that touch its zone. */
    fun addSink(name: String, mount: Any? = null, at: Vec3? = null, size: Vec3? = null, bounds: Boolean? = null,
                frame: Any? = null, source: String? = null, enable: String? = null): Map<*, *> =
        conn.call("sink.add", mapOf("name" to name, "mount" to ref(mount), "at" to at?.toList(), "size" to size?.toList(),
            "bounds" to bounds, "frame" to ref(frame), "source" to source, "enable" to enable))
    /** Removes a signal, sensor, sequence, source, sink, pose or motion; says which it was. */
    fun remove(name: String): String = conn.call("automation.remove", mapOf("name" to name))["removed"] as String
}
