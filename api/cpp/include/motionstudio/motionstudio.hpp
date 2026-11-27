// MotionStudio robot API for C++ (C++17, no dependencies).
//
//   #include <motionstudio/motionstudio.hpp>
//   namespace ms = motionstudio;
//
//   auto studio = ms::Studio::connect();          // localhost:5800
//   auto robot  = studio.robot("Robot");
//   robot.moveL(ms::Target().z(100));             // straight line; only z changes
//   robot.moveBy(ms::Target().y(-50));            // 50 mm down
//   robot.moveJoints({{"A1", 30}});               // a joint move
//   studio.set("Start", true);
//
//   auto walker = studio.humanoid();              // a humanoid on legs
//   walker.walk(1000);                            // a metre ahead
//   walker.turn(90);                              // a quarter turn to its left
//   walker.walkPath({{1000, 0}, {1000, 1000}});   // through points (x, z in mm)
//
//   ms::Scene scene = studio.scene();             // building scenes (see below)
//   ms::SceneObject station = scene.importFile("station.step");
//
// Units are millimetres, degrees and seconds; Y is up, as in MotionStudio.
// Every call throws ms::Error (or a subclass) when MotionStudio refuses it.
#pragma once

#include <iosfwd>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace motionstudio {

constexpr const char* kDefaultHost = "localhost";
constexpr int         kDefaultPort = 5800;
constexpr int         kProtocol    = 1;

// ── Errors ──────────────────────────────────────────────────────────────────

// An error reported by MotionStudio; code() says what kind it is
// (not_found, ambiguous, out_of_reach, out_of_limits, cancelled, reset,
// timeout, simulation, bad_request …)
class Error : public std::runtime_error {
public:
    Error(std::string code, const std::string& message)
        : std::runtime_error(message), code_(std::move(code)) {}
    const std::string& code() const { return code_; }
private:
    std::string code_;
};
// No robot, tool, joint, pose or signal has that name (or several do)
struct NotFound : Error { using Error::Error; };
// The tool can't get there: the target, or a point on the straight line
struct OutOfReach : Error { using Error::Error; };
// A joint value is outside the joint's range
struct OutOfLimits : Error { using Error::Error; };
// The motion was stopped, cancelled by an earlier failure, or the simulation was reset
struct MotionCancelled : Error { using Error::Error; };
// A wait ran out of time (the motion or condition carries on)
struct Timeout : Error { using Error::Error; };
// The simulation could not start, is not running, or is active while the
// scene is changed (simulation_active: reset it first)
struct SimulationError : Error { using Error::Error; };
// The project open in the app has unsaved changes: save, or pass discard
struct UnsavedChanges : Error { using Error::Error; };
// The connection failed or was closed
struct ConnectionError : Error { using Error::Error; };

// ── Values ──────────────────────────────────────────────────────────────────

// A position (mm) and orientation (Euler XYZ, degrees) of a tool frame
struct Pose {
    double x = 0, y = 0, z = 0;
    double rx = 0, ry = 0, rz = 0;

    // This pose with the amounts added to its values
    Pose offset(double dx, double dy = 0, double dz = 0, double drx = 0, double dry = 0, double drz = 0) const {
        return {x + dx, y + dy, z + dz, rx + drx, ry + dry, rz + drz};
    }
};
std::ostream& operator<<(std::ostream& os, const Pose& p);

// A Cartesian target: any of x y z rx ry rz. In an absolute move the values
// left out stay as they are; in a relative move (moveBy) they are 0.
//   ms::Target().z(100)          ms::Target(pose)          ms::Target().x(10).y(-5)
class Target {
public:
    Target() = default;
    Target(const Pose& p) : x_(p.x), y_(p.y), z_(p.z), rx_(p.rx), ry_(p.ry), rz_(p.rz) {}   // NOLINT: implicit on purpose
    Target& x(double v)  { x_ = v;  return *this; }
    Target& y(double v)  { y_ = v;  return *this; }
    Target& z(double v)  { z_ = v;  return *this; }
    Target& rx(double v) { rx_ = v; return *this; }
    Target& ry(double v) { ry_ = v; return *this; }
    Target& rz(double v) { rz_ = v; return *this; }
    std::optional<double> x_, y_, z_, rx_, ry_, rz_;
};

// How to move. `speed`: mm/s for straight lines (0 = 250), a percentage of
// the joints' speed for joint moves (0 = 100). `frame`: "base" (the
// robot's), "world", "tool" (moveBy only) or an object's name. With `wait`
// false a move returns at once; wait for it with Motion::wait().
//   robot.moveL(target, {200})                       C++17
//   robot.moveL(target, {.speed = 200, .wait = false})   C++20
struct MoveOptions {
    double      speed = 0;
    std::string frame = "base";
    bool        wait  = true;
};

struct JointInfo {
    std::string name;
    std::string type;          // "revolute" or "prismatic"
    std::string unit;          // "deg" or "mm"
    double      value = 0;
    std::optional<double> min, max;   // none for a continuous joint
    double      speed = 0;     // the actuator's speed (100 %), per second
    double      acceleration = 0;
};

// Where a walking robot stands: x and z (mm) on the floor and its heading
// (degrees about +Y: 0 faces +X, positive turns to its left, towards −Z)
struct Place {
    double x = 0, z = 0;
    double heading = 0;
};
std::ostream& operator<<(std::ostream& os, const Place& p);

// A point on the floor (mm) for walkTo / walkPath
struct Point {
    double x = 0, z = 0;
};

// How a humanoid walks; what is left out comes from the size of its legs.
// Lengths in mm, angles in degrees, stepTime in s, speed in mm/s (it sets
// the step time from the step length). armSwing 0 keeps the arms still.
//   ms::Gait{.stepLength = 250, .speed = 400}          C++20
struct Gait {
    std::optional<double> stepLength, sideStep, turnPerStep, stepHeight, stepTime, speed, crouch, sway, armSwing;
};

// Options of a walk. `frame` (walkTo / walkPath): "world", "robot" (where
// it stands when the walk starts: x ahead, z to its right) or an object's name.
struct WalkOptions {
    Gait        gait;
    std::string frame = "world";
    bool        wait  = true;
};

// A humanoid's place, whether it walks, and the gait it uses by default
struct WalkState {
    Place place;
    bool  walking = false;
    int   motion = 0;          // the walk under way (0: none, or a sequence's Walk operation)
    int   step = 0, steps = 0; // the step under way (1-based) and the walk's steps
    Gait  gait;                // every value set
};

struct SignalInfo {
    std::string name, kind, type;   // kind: Input, Internal, Derived, Sensor; type: Bool, Int, Real
    bool        writable = false;
    double      value = 0;
};

namespace detail { class Connection; }
class Studio;
class Robot;

// ── Motions ─────────────────────────────────────────────────────────────────

// A motion MotionStudio is running (a move with wait = false)
class Motion {
public:
    int id() const { return id_; }
    // Waits until it ends; throws if it failed or was cancelled. `timeout`
    // is in simulation seconds (< 0: none); on timeout the motion carries on.
    void wait(double timeout = -1) const;
    // Stops it; the joints brake and hold where they are
    void cancel() const;
    // "queued", "running", "done", "failed" or "cancelled"
    std::string state() const;
    bool done() const;
    double progress() const;   // 0 … 1

private:
    friend class Robot; friend class Tool; friend class Actuator; friend class Humanoid;
    Motion(std::shared_ptr<detail::Connection> c, int id) : conn_(std::move(c)), id_(id) {}
    std::shared_ptr<detail::Connection> conn_;
    int id_ = 0;
};

// ── Robots and tools ────────────────────────────────────────────────────────

// A tool frame (TCP) of a robot: what moves to Cartesian targets
class Tool {
public:
    const std::string& name() const { return name_; }
    // Where the tool is: in the robot's base frame, "world" or an object's frame
    Pose pose(const std::string& frame = "base") const;
    // Straight line to a target (values left out stay as they are)
    Motion moveL(const Target& target, const MoveOptions& o = {}) const;
    // Joint move to a target: faster, not straight
    Motion moveJ(const Target& target, const MoveOptions& o = {}) const;
    // Straight line by an offset from where the tool is; o.frame = "tool"
    // moves along and turns about the tool's own axes
    Motion moveBy(const Target& offset, const MoveOptions& o = {}) const;
    // The same offset as a joint move
    Motion moveJBy(const Target& offset, const MoveOptions& o = {}) const;

private:
    friend class Robot;
    Tool(std::shared_ptr<detail::Connection> c, std::string robot, std::string name, std::string base)
        : conn_(std::move(c)), robot_(std::move(robot)), name_(std::move(name)), base_(std::move(base)) {}
    Motion move(const char* method, const Target& t, const MoveOptions& o, bool relative) const;
    std::shared_ptr<detail::Connection> conn_;
    std::string robot_, name_, base_;
};

// A robot or other machine in the scene: the joints below one object
class Robot {
public:
    const std::string& name() const { return name_; }
    const std::string& path() const { return path_; }
    const std::vector<std::string>& jointNames() const { return joints_; }
    const std::vector<std::string>& toolNames() const { return tools_; }
    bool walks() const { return walks_; }   // it has legs: use it as a Humanoid

    // The joints' values now (degrees, or mm for sliding joints)
    std::map<std::string, double> joints() const;
    std::vector<JointInfo> jointInfo() const;
    std::vector<std::string> poses() const;
    // Moves joints to values, arriving together: moveJoints({{"A1", 30}, {"A2", -10}})
    Motion moveJoints(const std::map<std::string, double>& values, const MoveOptions& o = {}) const;
    // Moves joints by amounts from where they are
    Motion moveJointsBy(const std::map<std::string, double>& amounts, const MoveOptions& o = {}) const;
    // A pose taught in MotionStudio, and the home pose ("<robot> Home" or "Home")
    Motion moveToPose(const std::string& pose, const MoveOptions& o = {}) const;
    Motion home(const MoveOptions& o = {}) const;
    // Stops every motion of this robot; returns how many
    int stop() const;

    // A tool frame; without a name the robot's only tool. `base` limits the
    // joints that move to those below that object (a humanoid's arm without
    // its waist: tool("Right Hand TCP", "Torso")).
    Tool tool(const std::string& name = "", const std::string& base = "") const;
    // The only tool's moves
    Pose   pose(const std::string& frame = "base") const { return tool().pose(frame); }
    Motion moveL(const Target& t, const MoveOptions& o = {}) const { return tool().moveL(t, o); }
    Motion moveJ(const Target& t, const MoveOptions& o = {}) const { return tool().moveJ(t, o); }
    Motion moveBy(const Target& t, const MoveOptions& o = {}) const { return tool().moveBy(t, o); }

protected:
    friend class Studio;
    Robot(std::shared_ptr<detail::Connection> c) : conn_(std::move(c)) {}
    Motion jointMove(const std::map<std::string, double>& v, const MoveOptions& o, bool relative) const;
    std::shared_ptr<detail::Connection> conn_;
    std::string name_, path_;
    std::vector<std::string> joints_, tools_;
    bool walks_ = false;
};

// A robot that walks: a humanoid on legs (Tools > Humanoid Builder) or any
// model with a "Left Foot" and a "Right Foot". Its root object is carried
// along while the legs step. Distances are mm, angles degrees; its left is −Z.
class Humanoid : public Robot {
public:
    // Walks `forward` mm (negative: back) and `sideways` mm to its left
    // (negative: right) while turning `turn` degrees to its left; a turn with
    // a distance is spread over the walk, so it follows an arc
    Motion walk(double forward, double sideways = 0, double turn = 0, const WalkOptions& o = {}) const;
    Motion turn(double degrees, const WalkOptions& o = {}) const;       // on the spot, positive to its left
    Motion sideStep(double mm, const WalkOptions& o = {}) const;        // positive to its left
    // To (x, z) mm, facing the way it goes, then to `heading` (degrees) if given
    Motion walkTo(double x, double z, std::optional<double> heading = std::nullopt, const WalkOptions& o = {}) const;
    // Through points on the floor, turning gradually at the corners, then to `heading` if given
    Motion walkPath(const std::vector<Point>& points, std::optional<double> heading = std::nullopt,
                    const WalkOptions& o = {}) const;
    // Ends its walks after the step under way, the feet together; returns how many
    int stopWalking() const;

    WalkState state() const;
    Place place() const { return state().place; }
    bool  walking() const { return state().walking; }
    Gait  defaultGait() const { return state().gait; }

private:
    friend class Studio;
    explicit Humanoid(const Robot& r) : Robot(r) {}
};

// One actuator by name, on any machine: a door, a turntable, a drive
class Actuator {
public:
    const std::string& name() const { return name_; }
    double value() const;                                         // degrees or mm
    Motion moveTo(double value, const MoveOptions& o = {}) const;  // o.speed in %
    Motion moveBy(double amount, const MoveOptions& o = {}) const;
    void   run(double speed) const;                               // deg/s or mm/s until stopped
    void   stop() const;

private:
    friend class Studio;
    Actuator(std::shared_ptr<detail::Connection> c, std::string n) : conn_(std::move(c)), name_(std::move(n)) {}
    std::shared_ptr<detail::Connection> conn_;
    std::string name_;
};

// ── Building scenes ─────────────────────────────────────────────────────────
//
//   ms::Scene scene = studio.scene();
//   studio.reset();                                   // changes need the simulation reset
//   scene.newScene();
//   ms::SceneObject cell  = scene.addFrame("Cell");
//   ms::SceneObject table = scene.addBox("Table", {800, 40, 600}, {.parent = cell.ref(), .position = ms::Target().y(700)});
//   ms::SceneObject arm   = scene.addFrame("Arm", {.parent = cell.ref(), .position = ms::Target().x(300).y(800)});
//   scene.addJoint("Turn", cell.ref(), arm.ref(), {0, 1, 0}, {.min = -170, .max = 170});
//   scene.save("cell.mstudio");
//
// Positions are in the parent's frame unless `frame` names another
// ("world" or an object). Objects are named by name, path or ref() ("#12",
// stable through renames and moves until the scene is opened again). A
// number given for an automation value is in mm, mm/s or degrees; a string
// is an expression in the automation's own units (m, m/s, degrees).

// A point or direction (mm for points)
struct Vec3 {
    double x = 0, y = 0, z = 0;
};
// An axis-aligned box in some frame (mm)
struct Box {
    Vec3 min, max, center, size;
};
// A round edge ("circle"), round face ("cylinder"), straight edge ("line")
// or flat face ("plane") of a CAD part: where a joint's axis can come from
struct Feature {
    std::string object, kind;
    Vec3 origin, axis;                       // centre and direction (a plane's normal)
    std::optional<double> radius, length;
};

// An automation value: a number in API units, or an expression as a string
class Value {
public:
    Value(double v) : number_(v) {}                       // NOLINT: implicit on purpose
    Value(int v) : number_(v) {}                          // NOLINT
    Value(const char* e) : expr_(e) {}                    // NOLINT
    Value(std::string e) : expr_(std::move(e)) {}         // NOLINT
    bool isNumber() const { return !expr_; }
    double number() const { return number_; }
    const std::string& expression() const { return *expr_; }
private:
    double number_ = 0;
    std::optional<std::string> expr_;
};

struct ObjectOptions {
    std::string parent;                     // name, path or ref; empty: the top
    std::optional<Target> position;         // what is left out stays (0 for a new object)
    std::string frame;                      // the position's frame; empty: the parent's
    std::optional<Vec3> size;               // boxes, cylinders, spheres (mm; default 100)
    std::optional<double> diameter, height; // cylinders, capsules
    std::string color;                      // "#rrggbb"; empty: unchanged / the parent's
    std::optional<double> opacity;
    std::optional<bool> hidden;
};

struct BodyOptions {
    std::optional<double> mass;             // kg
    std::optional<bool> gravity, collision;
};

class Scene;

// An object in the scene. Copies refer to the same object.
class SceneObject {
public:
    int id() const { return id_; }
    std::string ref() const { return "#" + std::to_string(id_); }
    const std::string& path() const { return path_; }
    std::string name() const;
    std::string type() const;               // frame, mesh, box, cylinder, sphere, capsule, basket
    std::string parentPath() const;         // empty at the top
    Pose position() const;                  // in its parent's frame (the world at the top)
    Pose world() const;
    Pose poseIn(const std::string& frame) const;
    std::optional<Vec3> size() const;       // boxes, cylinders, spheres
    std::vector<SceneObject> children() const;

    // Changes: what is set in `o` (parent "" leaves it; use setParent("") for the top)
    SceneObject& set(const ObjectOptions& o);
    SceneObject& moveTo(const Target& position, const std::string& frame = "");
    SceneObject& rename(const std::string& name);
    SceneObject& setParent(const std::string& parent);   // "" = the top; keeps where it is and its joint
    SceneObject  add(const std::string& type, const std::string& name, ObjectOptions o = {}) const;
    int          remove() const;                          // with its children; returns how many
    SceneObject  duplicate(const std::string& name = "") const;

    Box bounds(const std::string& frame = "world", bool children = true) const;
    std::vector<Feature> features(const std::vector<std::string>& kinds = {"circle", "cylinder"},
                                  const std::string& frame = "world", std::optional<double> minRadius = std::nullopt,
                                  std::optional<double> maxRadius = std::nullopt) const;

    SceneObject& setBody(const BodyOptions& o = {});      // a rigid body (falls, rides belts, is gripped)
    SceneObject& removeBody();
    // A conveyor belt: speed mm/s or an expression in m/s; direction in its own frame unless `frame`
    SceneObject& setBelt(const Value& speed, std::optional<Vec3> direction = std::nullopt, const std::string& frame = "");
    SceneObject& removeBelt();

private:
    friend class Scene;
    SceneObject(std::shared_ptr<detail::Connection> c, int id, std::string path)
        : conn_(std::move(c)), id_(id), path_(std::move(path)) {}
    std::shared_ptr<detail::Connection> conn_;
    int id_ = 0;
    std::string path_;
};

struct ActuatorOptions {
    std::string name;                       // default: the joint's
    std::optional<double> speed, acceleration;   // deg/s(²) or mm/s(²)
    std::string mode;                       // "position" (default) or "speed"
    std::string target;                     // an expression binding its target (automation units)
};

struct JointOptions {
    std::string type = "revolute";          // or "prismatic"
    std::optional<Vec3> pivot;              // mm; default: the child's origin
    std::string frame;                      // axis and pivot frame; empty: the world
    std::optional<double> min, max;         // degrees, or mm (needed for prismatic)
    std::optional<bool> continuous;
    std::optional<double> value;
    std::optional<ActuatorOptions> actuator;   // default: one named after the joint
    bool noActuator = false;
    std::string follows;                    // a joint this one follows (no actuator then)
    double ratio = 1, offset = 0;
};

// Facts about a joint (axis and pivot in the world)
struct JointDetails {
    std::string name, type, parent, child;
    Vec3 axis, pivot;
    std::optional<double> min, max;
    double value = 0;
    std::string actuator, follows;          // empty when none
};

class Joint {
public:
    const std::string& name() const { return name_; }
    JointDetails details() const;
    double value() const { return details().value; }
    void   setValue(double v) const;        // degrees or mm, while the simulation is reset
    void   setRange(double min, double max) const;
    void   remove() const;                  // the part stays where it is

private:
    friend class Scene;
    Joint(std::shared_ptr<detail::Connection> c, std::string n) : conn_(std::move(c)), name_(std::move(n)) {}
    std::shared_ptr<detail::Connection> conn_;
    std::string name_;
};

// What every operation can have
struct OpOptions {
    std::vector<std::string> after;         // operation names it waits for (default: the one before)
    bool first = false;                     // waits for none: runs at the start of the cycle
    std::string onStart, onFinish, startCondition;
};

// The operations of a sequence, in order; each runs after the one before
// unless its options' `after` names others (`first`: none)
class Ops {
public:
    using Common = OpOptions;
    Ops& move(const std::string& name, const std::string& actuator, const Value& target,
              std::optional<Value> speed = std::nullopt, const Common& c = {});
    Ops& moveToPose(const std::string& name, const std::string& pose, std::optional<Value> speed = std::nullopt,
                    const Common& c = {});
    Ops& run(const std::string& name, const std::string& actuator, const Value& speed, const Common& c = {});
    Ops& stop(const std::string& name, const std::string& actuator, const Common& c = {});
    Ops& set(const std::string& name, const std::string& actions, const Common& c = {});
    Ops& wait(const std::string& name, const Value& seconds, const Common& c = {});
    Ops& waitFor(const std::string& name, const std::string& condition, const Common& c = {});
    // The gripper holds what `sensor` sees (or, with bySensor false, the object `what`)
    Ops& grip(const std::string& name, const std::string& gripper, const std::string& what, bool bySensor = true,
              const Common& c = {});
    Ops& release(const std::string& name, const std::string& gripper, const Common& c = {});
    Ops& moveL(const std::string& name, const std::string& tool, const std::string& pose,
               std::optional<Value> speed = std::nullopt, const Common& c = {});
    // Any operation as the protocol writes it: a JSON object
    Ops& raw(const std::string& json);
    size_t size() const { return ops_.size(); }

private:
    friend class Scene;
    Ops& push(std::string json);
    std::vector<std::string> ops_;          // JSON objects
};

struct ImportOptions {
    std::string parent, name;
    std::string upAxis;                     // STEP: "z" (default, most CAD) or "y"
};
struct SignalOptions {
    std::string kind = "internal";          // "input", "internal" or "derived"
    std::string type = "bool";              // "bool", "int" or "real"
    double value = 0;
    std::string expression, description;
};
struct SensorOptions {
    std::string mount;                      // what it moves with; empty: the world
    std::optional<Vec3> at, direction, size;
    std::optional<double> range;            // beams, mm
    bool bounds = false;                    // zones: the mount's box
    std::string frame;                      // at/direction/size frame; empty: the mount's
    std::string detects = "any";            // "any", "bodies" or an object
    std::string description;
};
struct SourceOptions {
    std::optional<Value> interval;          // seconds
    std::string trigger, enable;            // a trigger makes a copy each time it becomes true
    int max = 0;
    bool whenClear = true;
};
struct SinkOptions {
    std::string mount;
    std::optional<Vec3> at, size;
    bool bounds = false;
    std::string frame, source, enable;
};
struct SequenceOptions {
    bool loop = false;
    std::string enable, start;
    bool replace = false;
};
// What a scene holds, by name
struct SceneInfo {
    std::string path;
    int objects = 0;
    std::vector<std::string> top, robots, joints, signals, sensors, sequences, poses, sources, sinks, motions;
};

class Scene {
public:
    void newScene(bool discard = false) const;           // in the app: unsaved changes need discard
    SceneInfo open(const std::string& path, bool discard = false) const;
    std::string save(const std::string& path = "") const;   // "" = where it came from; returns the path
    SceneInfo info() const;
    std::vector<std::string> check() const;               // the automation's problems; empty = it runs
    // STEP (Z-up stood upright unless upAxis "y"), URDF, MJCF or a project merged in
    SceneObject importFile(const std::string& path, const ImportOptions& o = {}) const;

    std::vector<SceneObject> objects(const std::string& pattern = "", const std::string& under = "",
                                     const std::string& type = "", int depth = -1) const;
    SceneObject object(const std::string& name) const;
    bool exists(const std::string& name) const;
    SceneObject add(const std::string& type, const std::string& name, ObjectOptions o = {}) const;
    SceneObject addFrame(const std::string& name, ObjectOptions o = {}) const { return add("frame", name, std::move(o)); }
    SceneObject addBox(const std::string& name, Vec3 size, ObjectOptions o = {}) const;
    SceneObject addCylinder(const std::string& name, double diameter, double height, ObjectOptions o = {}) const;
    SceneObject group(const std::vector<std::string>& objects, const std::string& name = "Group") const;

    Joint addJoint(const std::string& name, const std::string& parent, const std::string& child, Vec3 axis,
                   const JointOptions& o = {}) const;
    Joint joint(const std::string& name) const;

    void addSignal(const std::string& name, const SignalOptions& o = {}) const;
    void addSensor(const std::string& name, const std::string& type, const SensorOptions& o) const;
    void addPose(const std::string& name, const std::map<std::string, double>& joints, bool replace = false) const;
    void addSequence(const std::string& name, const Ops& ops, const SequenceOptions& o = {}) const;
    std::string sequenceJson(const std::string& name) const;   // as stored (automation units)
    void addSource(const std::string& name, const std::string& templateObject, const SourceOptions& o = {}) const;
    void addSink(const std::string& name, const SinkOptions& o) const;
    std::string remove(const std::string& name) const;         // a signal, sensor, sequence, source, sink, pose …

private:
    friend class Studio;
    friend class SceneObject;
    explicit Scene(std::shared_ptr<detail::Connection> c) : conn_(std::move(c)) {}
    std::shared_ptr<detail::Connection> conn_;
};

// ── Studio ──────────────────────────────────────────────────────────────────

// A connection to MotionStudio (the app with Tools › Robot API on, or
// motionstudio-server). Copies share the connection; it closes with the last.
// Calls may come from several threads.
class Studio {
public:
    static Studio connect(const std::string& host = kDefaultHost, int port = kDefaultPort);

    const std::string& version() const { return version_; }

    std::vector<Robot> robots() const;
    // A robot by name or path ("Cell/Robot"); without a name the only robot
    Robot robot(const std::string& name = "") const;
    // A robot that walks, by name or path; without a name the only one
    Humanoid humanoid(const std::string& name = "") const;
    Actuator actuator(const std::string& name) const;
    // Building: objects, joints, sensors, sequences, import, open and save
    Scene scene() const { return Scene(conn_); }

    // Signals: any symbol can be read (Box.count, J1.pos …); Input and
    // Internal signals can be written
    double get(const std::string& signal) const;
    void   set(const std::string& signal, double value) const;
    void   set(const std::string& signal, bool value) const { set(signal, value ? 1.0 : 0.0); }
    std::vector<SignalInfo> signals() const;
    // Waits until an expression is true ("PartSensor && !Busy"); `timeout`
    // in simulation seconds (< 0: none). Returns the simulation time.
    double waitUntil(const std::string& condition, double timeout = -1) const;

    // Simulation. Moves and signal writes start it themselves.
    void   play() const;
    void   pause() const;
    void   reset() const;
    double sleep(double seconds) const;   // simulation seconds; returns the time
    double time() const;
    bool   running() const;
    void   setSpeed(double factor) const; // 1 = real time, up to 20

    void close();

private:
    explicit Studio(std::shared_ptr<detail::Connection> c) : conn_(std::move(c)) {}
    std::shared_ptr<detail::Connection> conn_;
    std::string version_;
};

} // namespace motionstudio
