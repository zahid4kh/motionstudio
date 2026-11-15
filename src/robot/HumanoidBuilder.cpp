#include "HumanoidBuilder.h"
#include "automation/Automation.h"
#include "scene/SceneGraph.h"
#include <QJsonArray>
#include <QMatrix4x4>
#include <QQuaternion>
#include <algorithm>
#include <cmath>

namespace humanoid {

namespace {

const QVector3D X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);
const QColor kDark("#1d2024"), kRubber("#26292d"), kMetal("#b9bfc7");

JointSpec J(bool on, double min, double max) { return {on, min, max, 0, 120, 480}; }

// Each joint's axis at its frame (all joints 0). Sided axes are mirrored on
// the right (−axis) so a value moves both sides the same way.
struct AxisInfo { QVector3D axis; bool mirrored; };
AxisInfo axisOf(const QString& key) {
    static const std::map<QString, AxisInfo> a = {
        {"WaistYaw", {Y, false}},  {"WaistPitch", {-Z, false}}, {"WaistRoll", {X, false}},
        {"NeckYaw", {Y, false}},   {"NeckPitch", {-Z, false}},  {"NeckRoll", {X, false}},
        {"ShoulderPitch", {Z, false}}, {"ShoulderRoll", {X, true}}, {"ShoulderYaw", {Y, true}},
        {"Elbow", {Z, false}},
        {"WristYaw", {Y, true}}, {"WristPitch", {Z, false}}, {"WristRoll", {X, true}},
        {"HipYaw", {Y, true}}, {"HipRoll", {X, true}}, {"HipPitch", {Z, false}},
        {"Knee", {-Z, false}}, {"AnklePitch", {Z, false}}, {"AnkleRoll", {X, true}}, {"Toe", {Z, false}},
    };
    auto it = a.find(key);
    return it == a.end() ? AxisInfo{Z, false} : it->second;
}

QVector3D eulerFromY(const QVector3D& dir) {
    const QVector3D d = dir.normalized();
    if (QVector3D::dotProduct(d, Y) < -0.99999f) return {180, 0, 0};
    QMatrix4x4 m;
    m.rotate(QQuaternion::rotationTo(Y, d));
    return SceneGraph::toTransform(m).rotation;
}

QString sidePrefix(int side) { return side < 0 ? "L" : side > 0 ? "R" : ""; }
QString sideName(int side) { return side < 0 ? "Left " : side > 0 ? "Right " : ""; }

} // namespace

// ── Catalogue ────────────────────────────────────────────────────────────────

std::vector<robot::DimensionDef> dimensionDefs(Base base) {
    std::vector<robot::DimensionDef> d = {
        {"Dh", "Dh", "Head", "Neck joint up to the top of the head", 0.06, 0.6, 0.22},
        {"Hn", "Hn", "Neck", "Shoulder joints up to the neck joint", 0, 0.4, 0.10},
        {"Ws", "Ws", "Shoulder width", "Between the left and right shoulder joints", 0.1, 1.4, 0.40},
        {"Ht", "Ht", "Torso", "Waist joint up to the shoulder joints", 0.1, 1.4, 0.40},
        {"Lu", "Lu", "Upper arm", "Shoulder joint to elbow joint", 0.04, 1.2, 0.29},
        {"Lf", "Lf", "Forearm", "Elbow joint to wrist joint", 0.04, 1.2, 0.25},
        {"Lh", "Lh", "Hand", "Wrist joint to the fingertips", 0.03, 0.5, 0.17},
        {"Hp", "Hp", "Pelvis", "Hip joints up to the waist joint", 0, 0.6, 0.12},
        {"Wh", "Wh", "Hip width", "Between the left and right hip joints", 0.04, 1.0, 0.20},
    };
    if (base == Base::Pedestal) {
        d.push_back({"Hb", "Hb", "Pedestal", "Floor up to the hip joints' line (the pelvis sits on the column)", 0.1, 3.0, 0.80});
    } else {
        d.push_back({"Lt", "Lt", "Thigh", "Hip joint to knee joint", 0.04, 1.4, 0.36});
        d.push_back({"Ls", "Ls", "Shin", "Knee joint to ankle joint", 0.04, 1.4, 0.34});
        d.push_back({"Ha", "Ha", "Ankle height", "Sole up to the ankle joint", 0.02, 0.4, 0.08});
        d.push_back({"Lft", "Lft", "Foot length", "Heel to toe (the ankle sits at 30 % from the heel)", 0.04, 0.8, 0.24});
        d.push_back({"Wft", "Wft", "Foot width", "Across the foot", 0.02, 0.4, 0.10});
    }
    return d;
}

const std::vector<JointDef>& jointDefs() {
    static const std::vector<JointDef> d = {
        {"WaistYaw",   "Waist yaw",   "Waist", false, "Turns the torso to its left", J(true, -90, 90)},
        {"WaistPitch", "Waist pitch", "Waist", false, "Bends the torso forward", J(true, -20, 60)},
        {"WaistRoll",  "Waist roll",  "Waist", false, "Leans the torso to its right", J(false, -30, 30)},
        {"NeckYaw",    "Neck yaw",    "Neck", false, "Looks to its left", J(true, -90, 90)},
        {"NeckPitch",  "Neck pitch",  "Neck", false, "Looks down", J(true, -30, 45)},
        {"NeckRoll",   "Neck roll",   "Neck", false, "Tilts the head to its right", J(false, -30, 30)},
        {"ShoulderPitch", "Shoulder pitch", "Arms", true, "Raises the arm forward", J(true, -60, 180)},
        {"ShoulderRoll",  "Shoulder roll",  "Arms", true, "Raises the arm sideways", J(true, -15, 180)},
        {"ShoulderYaw",   "Shoulder yaw",   "Arms", true, "Turns the arm outward", J(true, -90, 90)},
        {"Elbow",         "Elbow",          "Arms", true, "Bends the elbow", J(true, 0, 145)},
        {"WristYaw",      "Wrist twist",    "Arms", true, "Turns the palm forward", J(true, -120, 120)},
        {"WristPitch",    "Wrist pitch",    "Arms", true, "Bends the hand forward", J(true, -70, 70)},
        {"WristRoll",     "Wrist roll",     "Arms", true, "Bends the hand outward", J(false, -30, 30)},
        {"HipYaw",     "Hip yaw",     "Legs", true, "Turns the toes outward", J(true, -30, 45)},
        {"HipRoll",    "Hip roll",    "Legs", true, "Spreads the leg sideways", J(true, -20, 45)},
        {"HipPitch",   "Hip pitch",   "Legs", true, "Swings the leg forward", J(true, -30, 120)},
        {"Knee",       "Knee",        "Legs", true, "Bends the knee", J(true, 0, 140)},
        {"AnklePitch", "Ankle pitch", "Legs", true, "Lifts the toes", J(true, -40, 30)},
        {"AnkleRoll",  "Ankle roll",  "Legs", true, "Turns the sole outward", J(true, -20, 20)},
        {"Toe",        "Toes",        "Legs", true, "Bends the toes up", J(false, -10, 60)},
    };
    return d;
}

const JointDef* jointDef(const QString& key) {
    for (const JointDef& d : jointDefs())
        if (d.key == key) return &d;
    return nullptr;
}

QStringList baseNames() { return {"Legs (standing)", "Pedestal (upper body)"}; }
QStringList handNames() { return {"Mitten (no joints)", "Parallel gripper", "Five fingers"}; }
QStringList styleNames() { return {"Android (rounded)", "Mechanical", "Slim (skeleton)"}; }

double Spec::dim(const QString& key) const {
    if (dims.contains(key)) return dims.value(key);
    for (Base b : {Base::Legs, Base::Pedestal})
        for (const robot::DimensionDef& d : dimensionDefs(b))
            if (d.key == key) return d.value;
    return 0;
}

JointSpec Spec::joint(const QString& key) const {
    if (joints.contains(key)) return joints.value(key);
    const JointDef* d = jointDef(key);
    return d ? d->defaults : JointSpec{false};
}

Spec defaultSpec() {
    Spec s;
    for (Base b : {Base::Legs, Base::Pedestal})
        for (const robot::DimensionDef& d : dimensionDefs(b)) s.dims[d.key] = d.value;
    for (const JointDef& d : jointDefs()) s.joints[d.key] = d.defaults;
    return s;
}

double height(const Spec& s) {
    const double below = s.base == Base::Legs ? s.dim("Ha") + s.dim("Ls") + s.dim("Lt") : s.dim("Hb");
    return below + s.dim("Hp") + s.dim("Ht") + s.dim("Hn") + s.dim("Dh");
}

int jointCount(const Spec& s) {
    int n = 0;
    for (const JointDef& d : jointDefs()) {
        if (!s.joint(d.key).enabled) continue;
        if (d.group == "Legs" && s.base != Base::Legs) continue;
        n += d.sided ? 2 : 1;
    }
    return n;
}

void scaleToHeight(Spec& s, double metres) {
    const double h = height(s);
    if (h < 1e-6 || metres <= 0) return;
    const double k = metres / h;
    for (auto it = s.dims.begin(); it != s.dims.end(); ++it) it.value() *= k;
}

namespace {

// Limb and body widths, from the robot's size
struct Body {
    double S;   // 1 = a 1.62 m robot
    double thigh, shin, upper, fore, neck, torsoD, chestW, palmW, palmT;
};

Body body(const Spec& s) {
    Body w;
    w.S = std::max(0.15, height(s) / 1.62);
    const double b = w.S * s.bulk;
    w.thigh = std::min(0.13 * b, 0.95 * s.dim("Wh"));
    w.shin = std::min(0.105 * b, w.thigh);
    w.upper = 0.09 * b;
    w.fore = 0.08 * b;
    w.neck = 0.075 * b;
    w.torsoD = 0.21 * b;
    w.chestW = std::max(0.08 * w.S, s.dim("Ws") - 1.7 * w.upper);
    w.palmW = std::max(0.45 * s.dim("Lh"), 0.8 * w.fore);
    w.palmT = 0.38 * w.palmW;
    return w;
}

} // namespace

QStringList validate(Spec& s) {
    QStringList problems;
    s.name = s.name.trimmed();
    if (s.name.isEmpty()) s.name = "Humanoid";
    for (Base b : {Base::Legs, Base::Pedestal})
        for (const robot::DimensionDef& d : dimensionDefs(b))
            s.dims[d.key] = std::clamp(s.dims.value(d.key, d.value), d.min, d.max);
    for (const JointDef& d : jointDefs()) {
        JointSpec j = s.joints.value(d.key, d.defaults);
        j.min = std::clamp(j.min, -360.0, 360.0);
        j.max = std::clamp(j.max, -360.0, 360.0);
        if (j.min > j.max) std::swap(j.min, j.max);
        j.home = std::clamp(j.home, j.min, j.max);
        j.speed = std::clamp(j.speed, 1.0, 10000.0);
        j.accel = std::clamp(j.accel, 0.0, 200000.0);
        s.joints[d.key] = j;
    }
    s.scheme = std::clamp(s.scheme, 0, int(robot::colorSchemes().size()) - 1);
    s.bulk = std::clamp(s.bulk, 0.4, 2.5);

    const Body w = body(s);
    if (jointCount(s) == 0) problems << "No joints are chosen, so the humanoid cannot move.";
    if (s.dim("Ws") < 2 * w.upper + 0.03)
        problems << "The shoulders (Ws) are too close together for the arms to clear the torso.";
    if (s.base == Base::Legs && s.dim("Wh") < 0.13 * w.S * s.bulk)
        problems << "The hips (Wh) are narrower than the thighs; they are made thinner to fit.";
    if (s.base == Base::Pedestal && s.dim("Hb") < s.dim("Lu") + s.dim("Lf") + s.dim("Lh"))
        problems << "The hanging hands reach below the floor (Lu + Lf + Lh > Hb).";
    if (s.base == Base::Legs && s.dim("Lft") < s.dim("Wft"))
        problems << "The feet are wider (Wft) than they are long (Lft).";
    return problems;
}

// ── Presets ──────────────────────────────────────────────────────────────────

namespace {

void enable(Spec& s, std::initializer_list<const char*> keys, bool on) {
    for (const char* k : keys) s.joints[k].enabled = on;
}

Spec make(double h, Style style, int scheme, Hand hand, double bulk, Base base = Base::Legs) {
    Spec s = defaultSpec();
    s.base = base;
    s.style = style;
    s.scheme = scheme;
    s.hand = hand;
    s.bulk = bulk;
    scaleToHeight(s, h);
    return s;
}

} // namespace

const std::vector<Preset>& presets() {
    static const std::vector<Preset> p = [] {
        std::vector<Preset> v;
        {
            Spec s = make(1.62, Style::Android, 2, Hand::Fingers, 1.0);
            s.name = "Humanoid";
            v.push_back({"Full-size humanoid · 28 joints",
                         "An adult-sized general-purpose humanoid: waist, neck, 6-joint arms with five-finger hands "
                         "and 6-joint legs.", s});
        }
        {
            Spec s = make(0.58, Style::Mechanical, 4, Hand::Mitten, 1.15);
            s.name = "Mini Biped";
            s.dims["Dh"] *= 1.35;
            s.dims["Lft"] *= 1.15;
            s.dims["Wft"] *= 1.3;
            enable(s, {"WaistYaw", "WaistPitch", "ShoulderYaw", "WristYaw", "WristPitch"}, false);
            v.push_back({"Kid-size biped · 20 joints",
                         "A small servo-driven biped like the ones in robot football: big head, 3-joint arms, 6-joint "
                         "legs, no waist.", s});
        }
        {
            Spec s = make(1.55, Style::Mechanical, 0, Hand::Gripper, 1.2, Base::Pedestal);
            s.name = "Dual-Arm Worker";
            s.dims["Hb"] = 0.95;
            s.dims["Ws"] = 0.52;
            enable(s, {"WaistPitch", "WristRoll"}, false);
            enable(s, {"WristYaw", "WristPitch"}, true);
            v.push_back({"Dual-arm worker on a pedestal · 17 joints",
                         "An upper body on a column for bench work: waist turn, neck, two 6-joint arms with parallel "
                         "grippers.", s});
        }
        {
            Spec s = make(1.75, Style::Android, 5, Hand::Fingers, 1.0);
            s.name = "Research Humanoid";
            for (const JointDef& d : jointDefs()) s.joints[d.key].enabled = true;
            v.push_back({"Research humanoid · 34 joints",
                         "Every joint in the catalogue: 3-joint waist and neck, 7-joint arms and 7-joint legs with "
                         "toes.", s});
        }
        {
            Spec s = make(1.85, Style::Mechanical, 1, Hand::Gripper, 1.35);
            s.name = "Heavy Humanoid";
            s.dims["Ws"] *= 1.15;
            s.dims["Wh"] *= 1.1;
            s.dims["Wft"] *= 1.2;
            enable(s, {"WristYaw"}, false);
            v.push_back({"Heavy-duty humanoid · 26 joints",
                         "A broad, tall machine for carrying loads: thick limbs, grippers, no wrist twist.", s});
        }
        {
            Spec s = make(1.5, Style::Slim, 6, Hand::Fingers, 0.8);
            s.name = "Skeleton";
            v.push_back({"Slim skeleton · 28 joints",
                         "The full-size joint set on a light frame of rods, so every joint is easy to see.", s});
        }
        {
            Spec s = make(1.3, Style::Android, 7, Hand::Mitten, 1.1, Base::Pedestal);
            s.name = "Receptionist";
            s.dims["Hb"] = 0.7;
            s.dims["Dh"] *= 1.2;
            enable(s, {"WaistPitch", "ShoulderYaw", "WristYaw", "WristPitch"}, false);
            enable(s, {"NeckRoll"}, true);
            v.push_back({"Social robot · 13 joints",
                         "A friendly upper body for greeting: expressive 3-joint neck, simple arms with mitten hands.", s});
        }
        for (Preset& pr : v) validate(pr.spec);
        return v;
    }();
    return p;
}

// ── JSON ─────────────────────────────────────────────────────────────────────

QJsonObject toJson(const Spec& s) {
    QJsonObject o;
    o["name"] = s.name;
    o["base"] = s.base == Base::Legs ? "legs" : "pedestal";
    o["hand"] = s.hand == Hand::Mitten ? "mitten" : s.hand == Hand::Gripper ? "gripper" : "fingers";
    o["style"] = int(s.style);
    o["scheme"] = s.scheme;
    o["bulk"] = s.bulk;
    o["demo"] = s.demo;
    QJsonObject dims;
    for (auto it = s.dims.begin(); it != s.dims.end(); ++it) dims[it.key()] = it.value();
    o["dims"] = dims;
    QJsonObject joints;
    for (auto it = s.joints.begin(); it != s.joints.end(); ++it) {
        const JointSpec& j = it.value();
        joints[it.key()] = QJsonObject{{"on", j.enabled}, {"min", j.min}, {"max", j.max}, {"home", j.home},
                                       {"speed", j.speed}, {"accel", j.accel}};
    }
    o["joints"] = joints;
    return o;
}

Spec fromJson(const QJsonObject& o) {
    Spec s = defaultSpec();
    s.name = o.value("name").toString(s.name);
    s.base = o.value("base").toString() == "pedestal" ? Base::Pedestal : Base::Legs;
    const QString hand = o.value("hand").toString();
    s.hand = hand == "mitten" ? Hand::Mitten : hand == "gripper" ? Hand::Gripper : Hand::Fingers;
    s.style = Style(std::clamp(o.value("style").toInt(0), 0, 2));
    s.scheme = o.value("scheme").toInt(s.scheme);
    s.bulk = o.value("bulk").toDouble(s.bulk);
    s.demo = o.value("demo").toBool(true);
    const QJsonObject dims = o.value("dims").toObject();
    for (auto it = dims.begin(); it != dims.end(); ++it) s.dims[it.key()] = it.value().toDouble();
    const QJsonObject joints = o.value("joints").toObject();
    for (auto it = joints.begin(); it != joints.end(); ++it) {
        if (!jointDef(it.key())) continue;
        const QJsonObject j = it.value().toObject();
        JointSpec& js = s.joints[it.key()];
        js.enabled = j.value("on").toBool(js.enabled);
        js.min = j.value("min").toDouble(js.min);
        js.max = j.value("max").toDouble(js.max);
        js.home = j.value("home").toDouble(js.home);
        js.speed = j.value("speed").toDouble(js.speed);
        js.accel = j.value("accel").toDouble(js.accel);
    }
    validate(s);
    return s;
}

// ── Building ─────────────────────────────────────────────────────────────────

namespace {

struct Maker {
    SceneGraph& g;
    const Spec& s;
    Built& b;
    Body w;
    QColor primary, secondary, accent;
    std::map<QString, int> pending;   // actuator name to be → joint id
    std::map<QString, std::pair<double, double>> handSpeeds;   // hand actuator name → speed, accel

    int frame(int parent, const QString& name, QVector3D pos, QVector3D rot = {}) {
        const int id = g.addObject(name, PrimitiveType::ImportedMesh, parent);
        SceneObject* o = g.getObject(id);
        o->transform.position = pos;
        o->transform.rotation = rot;
        return id;
    }
    int prim(int parent, const QString& name, PrimitiveType t, QVector3D c, QVector3D size, const QColor& col,
             QVector3D rot = {}) {
        const int id = g.addObject(name, t, parent);
        SceneObject* o = g.getObject(id);
        o->transform.position = c;
        o->transform.rotation = rot;
        o->transform.scale = size;
        o->color = col;
        return id;
    }
    int box(int parent, const QString& name, QVector3D c, QVector3D size, const QColor& col) {
        return prim(parent, name, PrimitiveType::Cube, c, size, col);
    }
    int ball(int parent, const QString& name, QVector3D c, QVector3D size, const QColor& col) {
        return prim(parent, name, PrimitiveType::Sphere, c, size, col);
    }
    int cyl(int parent, const QString& name, QVector3D c, QVector3D axis, double dia, double len, const QColor& col) {
        return prim(parent, name, PrimitiveType::Cylinder, c, QVector3D(dia, len, dia), col, eulerFromY(axis));
    }
    int rod(int parent, const QString& name, QVector3D a, QVector3D bb, double dia, const QColor& col) {
        return cyl(parent, name, (a + bb) / 2, bb - a, dia, (bb - a).length(), col);
    }

    // A limb segment from a to b
    void limb(int parent, const QString& name, QVector3D a, QVector3D bb, double width) {
        const double len = (bb - a).length();
        if (len < 1e-4) return;
        switch (s.style) {
        case Style::Android:
            rod(parent, name, a, bb, width, primary);
            break;
        case Style::Mechanical:
            prim(parent, name, PrimitiveType::Cube, (a + bb) / 2, QVector3D(width, len, width * 0.85), primary,
                 eulerFromY(bb - a));
            break;
        case Style::Slim:
            rod(parent, name, a, bb, width * 0.4, secondary);
            break;
        }
    }
    // A joint housing turning about `axis`
    void hub(int parent, const QString& name, QVector3D c, QVector3D axis, double dia) {
        switch (s.style) {
        case Style::Android:
            ball(parent, name, c, QVector3D(dia, dia, dia), secondary);
            break;
        case Style::Mechanical:
            cyl(parent, name, c, axis, dia, dia * 1.1, secondary);
            break;
        case Style::Slim:
            ball(parent, name, c, QVector3D(dia, dia, dia) * 0.7f, accent);
            break;
        }
    }

    // The joints in `keys` that are chosen, all at `at` in `parent`, each in
    // its own frame; the last frame is the segment (named `segment`) that
    // carries the geometry. No joint: the segment is fixed to the parent.
    int group(int parent, QVector3D at, const QString& segment, std::initializer_list<const char*> keys, int side) {
        std::vector<QString> on;
        for (const char* k : keys)
            if (s.joint(k).enabled) on.push_back(k);
        const QString seg = sideName(side) + segment;
        if (on.empty()) return frame(parent, seg, at);
        int cur = parent;
        QVector3D pos = at;
        for (size_t i = 0; i < on.size(); ++i) {
            const QString name = sidePrefix(side) + on[i];
            const int f = frame(cur, i + 1 == on.size() ? seg : name + " Link", pos);
            const JointSpec js = s.joint(on[i]);
            const AxisInfo ax = axisOf(on[i]);
            Joint j;
            j.name = name;
            j.type = JointType::Revolute;
            j.axis = ax.mirrored && side > 0 ? -ax.axis : ax.axis;
            j.pivot = pos;
            j.limitMin = float(js.min);
            j.limitMax = float(js.max);
            j.initialAngle = float(js.home);
            j.parentBodyId = cur;
            j.childBodyId = f;
            pending[name] = g.addJoint(j);
            cur = f;
            pos = {};
        }
        return cur;
    }

    int follower(const QString& name, int parent, int child, JointType type, QVector3D axis, QVector3D pivot,
                 int master, double ratio, double min, double max) {
        Joint j;
        j.name = name;
        j.type = type;
        j.axis = axis;
        j.pivot = pivot;
        j.limitMin = float(min);
        j.limitMax = float(max);
        j.parentBodyId = parent;
        j.childBodyId = child;
        j.coupledTo = master;
        j.ratio = float(ratio);
        return g.addJoint(j);
    }

    void buildPelvis(int pelvis, bool onPedestal) {
        const double Wh = s.dim("Wh"), Hp = s.dim("Hp"), S = w.S;
        const double pw = Wh + w.thigh;
        const double top = std::max(0.03 * S, 0.65 * Hp), bot = -0.06 * S;
        const QVector3D c(0, float((top + bot) / 2), 0);
        switch (s.style) {
        case Style::Android:
            ball(pelvis, "Pelvis", c, QVector3D(float(w.torsoD * 0.85), float((top - bot) * 1.3), float(pw * 1.05)), primary);
            break;
        case Style::Mechanical:
            box(pelvis, "Pelvis", c, QVector3D(float(w.torsoD * 0.8), float(top - bot), float(pw)), primary);
            break;
        case Style::Slim:
            box(pelvis, "Pelvis", c, QVector3D(float(w.torsoD * 0.45), float((top - bot) * 0.6), float(pw * 0.9)), secondary);
            break;
        }
        if (Hp > top + 0.005) rod(pelvis, "Spine", {0, float(top), 0}, {0, float(Hp), 0}, w.neck * 1.2, secondary);
        if (onPedestal) cyl(pelvis, "Mount Flange", {0, float(bot - 0.01 * S), 0}, Y, 0.18 * S, 0.02 * S, secondary);
    }

    void buildLeg(int pelvis, int side) {
        const double Wh = s.dim("Wh"), Lt = s.dim("Lt"), Ls = s.dim("Ls"), Ha = s.dim("Ha"), Lft = s.dim("Lft"),
                     Wft = s.dim("Wft");
        const float z = float(side * Wh / 2);
        const int thigh = group(pelvis, {0, 0, z}, "Thigh", {"HipYaw", "HipRoll", "HipPitch"}, side);
        hub(thigh, "Hip", {}, Z, w.thigh * 1.15);
        limb(thigh, "Thigh", {0, float(-0.3 * w.thigh), 0}, {0, float(-Lt), 0}, w.thigh);
        const int shin = group(thigh, {0, float(-Lt), 0}, "Shin", {"Knee"}, side);
        hub(shin, "Knee", {}, Z, w.shin * 1.2);
        limb(shin, "Shin", {}, {0, float(-Ls), 0}, w.shin);
        const int foot = group(shin, {0, float(-Ls), 0}, "Foot", {"AnklePitch", "AnkleRoll"}, side);
        hub(foot, "Ankle", {}, Z, w.shin * 0.95);
        const double ft = 0.7 * Ha;                    // sole to the top of the foot
        const double x0 = -0.3 * Lft, x1 = 0.7 * Lft;
        const bool toe = s.joint("Toe").enabled;
        const double xs = toe ? x0 + 0.75 * Lft : x1;
        if (Ha - ft > 0.005) rod(foot, "Ankle Post", {}, {0, float(-Ha + ft), 0}, w.shin * 0.6, secondary);
        const QColor shell = s.style == Style::Slim ? secondary : primary;
        const double sole = std::min(0.012 * w.S, ft * 0.3);
        box(foot, "Foot", {float((x0 + xs) / 2), float(-Ha + sole + (ft - sole) / 2), 0},
            QVector3D(float(xs - x0), float(ft - sole), float(Wft)), shell);
        box(foot, "Sole", {float((x0 + xs) / 2), float(-Ha + sole / 2), 0}, QVector3D(float(xs - x0), float(sole), float(Wft)), kRubber);
        if (toe) {
            const int toes = group(foot, {float(xs), float(-Ha + ft / 2), 0}, "Toes", {"Toe"}, side);
            const double tl = x1 - xs;
            box(toes, "Toes", {float(tl / 2), float(-ft / 2 + (sole + ft * 0.8) / 2), 0},
                QVector3D(float(tl * 0.96), float(ft * 0.8 - sole), float(Wft * 0.95)), shell);
            box(toes, "Toe Sole", {float(tl / 2), float(-ft / 2 + sole / 2), 0}, QVector3D(float(tl * 0.96), float(sole), float(Wft * 0.95)), kRubber);
        }
        (side < 0 ? b.leftFoot : b.rightFoot) = foot;
    }

    int buildTorso(int pelvis) {
        const double Hp = s.dim("Hp"), Ht = s.dim("Ht"), Hn = s.dim("Hn"), Ws = s.dim("Ws"), S = w.S;
        const int torso = group(pelvis, {0, float(Hp), 0}, "Torso", {"WaistYaw", "WaistPitch", "WaistRoll"}, 0);
        const double cw = w.chestW, d = w.torsoD;
        switch (s.style) {
        case Style::Android:
            ball(torso, "Waist", {}, QVector3D(float(w.neck * 1.7), float(w.neck * 1.7), float(w.neck * 1.7)), secondary);
            ball(torso, "Abdomen", {0, float(0.24 * Ht), 0}, QVector3D(float(d * 0.8), float(0.5 * Ht), float(cw * 0.68)), secondary);
            ball(torso, "Chest", {0, float(0.66 * Ht), 0}, QVector3D(float(d), float(0.8 * Ht), float(cw)), primary);
            cyl(torso, "Core Light", {float(d * 0.47), float(0.68 * Ht), 0}, X, 0.075 * S, 0.02 * S, accent);
            for (int side : {-1, 1})
                rod(torso, sideName(side) + "Shoulder Bridge", {0, float(Ht), float(side * cw * 0.3)},
                    {0, float(Ht), float(side * (Ws / 2 - w.upper * 0.5))}, w.upper * 0.95, primary);
            break;
        case Style::Mechanical:
            cyl(torso, "Waist", {}, Y, d * 0.6, 0.07 * S, secondary);
            box(torso, "Abdomen", {0, float(0.25 * Ht), 0}, QVector3D(float(d * 0.75), float(0.5 * Ht), float(cw * 0.65)), secondary);
            box(torso, "Chest", {0, float(0.67 * Ht), 0}, QVector3D(float(d), float(0.66 * Ht), float(cw)), primary);
            box(torso, "Chest Plate", {float(d / 2 + 0.004), float(0.7 * Ht), 0}, QVector3D(0.008f, float(0.3 * Ht), float(cw * 0.5)), accent);
            for (int side : {-1, 1})
                rod(torso, sideName(side) + "Shoulder Housing", {0, float(Ht), float(side * cw * 0.35)},
                    {0, float(Ht), float(side * (Ws / 2 - w.upper * 0.6))}, w.upper * 1.15, secondary);
            break;
        case Style::Slim:
            ball(torso, "Waist", {}, QVector3D(float(w.neck), float(w.neck), float(w.neck)), accent);
            rod(torso, "Spine", {}, {0, float(Ht), 0}, w.neck * 0.7, secondary);
            box(torso, "Chest", {0, float(0.7 * Ht), 0}, QVector3D(float(d * 0.6), float(0.42 * Ht), float(cw * 0.8)), primary);
            rod(torso, "Shoulder Bar", {0, float(Ht), float(-Ws / 2)}, {0, float(Ht), float(Ws / 2)}, w.upper * 0.45, secondary);
            break;
        }
        if (Hn > 0.005) rod(torso, "Neck", {0, float(Ht * 0.9), 0}, {0, float(Ht + Hn), 0}, w.neck, secondary);
        return torso;
    }

    void buildHead(int torso) {
        const double Ht = s.dim("Ht"), Hn = s.dim("Hn"), Dh = s.dim("Dh");
        const int head = group(torso, {0, float(Ht + Hn), 0}, "Head", {"NeckYaw", "NeckPitch", "NeckRoll"}, 0);
        b.head = head;
        hub(head, "Neck Joint", {}, Z, w.neck * 1.15);
        const float h = float(Dh);
        switch (s.style) {
        case Style::Android:
            ball(head, "Skull", {0, h * 0.53f, 0}, QVector3D(h * 0.85f, h * 0.94f, h * 0.76f), primary);
            ball(head, "Visor", {h * 0.2f, h * 0.58f, 0}, QVector3D(h * 0.5f, h * 0.3f, h * 0.62f), kDark);
            for (int side : {-1, 1})
                ball(head, sideName(side) + "Eye", {h * 0.43f, h * 0.6f, side * h * 0.13f}, QVector3D(h, h, h) * 0.065f, accent);
            break;
        case Style::Mechanical:
            box(head, "Skull", {0, h * 0.5f, 0}, QVector3D(h * 0.8f, h * 0.86f, h * 0.72f), primary);
            box(head, "Visor", {h * 0.4f + 0.004f, h * 0.58f, 0}, QVector3D(0.012f, h * 0.2f, h * 0.56f), kDark);
            for (int side : {-1, 1})
                cyl(head, sideName(side) + "Ear", {0, h * 0.52f, side * h * 0.38f}, Z, h * 0.32, h * 0.06, secondary);
            break;
        case Style::Slim:
            ball(head, "Skull", {0, h * 0.5f, 0}, QVector3D(h, h, h) * 0.82f, primary);
            box(head, "Visor", {h * 0.36f, h * 0.55f, 0}, QVector3D(0.012f, h * 0.14f, h * 0.42f), kDark);
            break;
        }
    }

    void buildArm(int torso, int side) {
        const double Ht = s.dim("Ht"), Ws = s.dim("Ws"), Lu = s.dim("Lu"), Lf = s.dim("Lf");
        const int upper = group(torso, {0, float(Ht), float(side * Ws / 2)}, "Upper Arm",
                                {"ShoulderPitch", "ShoulderRoll", "ShoulderYaw"}, side);
        hub(upper, "Shoulder", {}, Z, w.upper * 1.3);
        limb(upper, "Upper Arm", {0, float(-0.4 * w.upper), 0}, {0, float(-Lu), 0}, w.upper);
        const int fore = group(upper, {0, float(-Lu), 0}, "Forearm", {"Elbow"}, side);
        hub(fore, "Elbow", {}, Z, w.fore * 1.15);
        limb(fore, "Forearm", {}, {0, float(-Lf), 0}, w.fore);
        const int hand = group(fore, {0, float(-Lf), 0}, "Hand", {"WristYaw", "WristPitch", "WristRoll"}, side);
        hub(hand, "Wrist", {}, Z, w.fore * 0.85);
        buildHand(hand, side);
    }

    // Hand frame: wrist at the origin, fingers along −Y, palm facing the body
    // (+Z on the left, −Z on the right), thumb at +X
    void buildHand(int hand, int side) {
        const double Lh = s.dim("Lh"), pw = w.palmW, pt = w.palmT;
        const double pl = 0.45 * Lh, fl = Lh - pl;
        const QString sn = sideName(side), sp = sidePrefix(side);
        const QColor shell = s.style == Style::Slim ? secondary : primary;
        const float inward = float(-side);   // towards the palm side
        (side < 0 ? b.leftGripper : b.rightGripper) = hand;
        box(hand, "Palm", {0, float(-pl / 2), 0}, QVector3D(float(pw), float(pl), float(pt)), shell);
        const int tcp = frame(hand, sn + "Hand TCP", {0, float(-(pl + 0.5 * fl)), 0});
        (side < 0 ? b.leftHand : b.rightHand) = tcp;

        switch (s.hand) {
        case Hand::Mitten:
            box(hand, "Fingers", {0, float(-pl - fl / 2), 0}, QVector3D(float(pw * 0.95), float(fl), float(pt * 0.85)), secondary);
            box(hand, "Thumb", {float(pw * 0.55), float(-pl * 0.6), inward * float(pt * 0.25)},
                QVector3D(float(pw * 0.28), float(pl * 0.6), float(pt * 0.7)), secondary);
            break;
        case Hand::Gripper: {
            const double jw = pw * 0.18, travel = pw * 0.32;
            const float jx = float(pw / 2 - jw / 2);
            const int a = frame(hand, sn + "Jaw A", {jx, float(-pl), 0});
            const int c = frame(hand, sn + "Jaw B", {-jx, float(-pl), 0});
            for (int jaw : {a, c})
                box(jaw, "Jaw", {0, float(-fl / 2), 0}, QVector3D(float(jw), float(fl), float(pt * 0.9)), secondary);
            Joint j;
            j.name = sp + "Grip";
            j.type = JointType::Prismatic;
            j.axis = -X;
            j.limitMin = 0;
            j.limitMax = float(travel);
            j.parentBodyId = hand;
            j.childBodyId = a;
            const int master = g.addJoint(j);
            pending[j.name] = master;
            handSpeeds[j.name] = {0.1 * w.S, 1.0 * w.S};
            follower(sp + "Grip Follower", hand, c, JointType::Prismatic, X, {}, master, 1, 0, travel);
            break;
        }
        case Hand::Fingers: {
            const double fw = pw * 0.21, ft = pt * 0.8;
            const QVector3D curl = X * float(side);   // fingertips towards the palm
            const char* names[] = {"Index", "Middle", "Ring", "Little"};
            const double len[] = {0.95, 1.0, 0.95, 0.8};
            int master = -1;
            for (int i = 0; i < 4; ++i) {
                const double l1 = 0.55 * fl * len[i], l2 = 0.45 * fl * len[i];
                const QVector3D at(float(pw * (0.375 - 0.25 * i)), float(-pl), 0);
                const int f1 = frame(hand, sn + names[i], at);
                box(f1, "Phalanx 1", {0, float(-l1 / 2), 0}, QVector3D(float(fw), float(l1 * 0.94), float(ft)), secondary);
                const int f2 = frame(f1, sn + names[i] + " Tip", {0, float(-l1), 0});
                box(f2, "Phalanx 2", {0, float(-l2 / 2), 0}, QVector3D(float(fw * 0.92), float(l2 * 0.94), float(ft * 0.9)), secondary);
                if (master < 0) {
                    Joint j;
                    j.name = sp + "Hand";
                    j.axis = curl;
                    j.pivot = at;
                    j.limitMin = 0;
                    j.limitMax = 90;
                    j.parentBodyId = hand;
                    j.childBodyId = f1;
                    master = g.addJoint(j);
                    pending[j.name] = master;
                    handSpeeds[j.name] = {180, 900};
                } else {
                    follower(sp + names[i], hand, f1, JointType::Revolute, curl, at, master, 1, 0, 90);
                }
                follower(sp + names[i] + " Tip", f1, f2, JointType::Revolute, curl, {0, float(-l1), 0}, master, 1, 0, 90);
            }
            // Thumb: leans forward out of the palm's front edge and closes towards it
            const double tl = 0.42 * fl;
            const QVector3D at(float(pw * 0.5), float(-pl * 0.3), inward * float(pt * 0.3));
            const int t1 = frame(hand, sn + "Thumb", at, {0, 0, 40});
            box(t1, "Thumb 1", {0, float(-tl / 2), 0}, QVector3D(float(fw * 1.05), float(tl * 0.94), float(ft)), secondary);
            const int t2 = frame(t1, sn + "Thumb Tip", {0, float(-tl), 0});
            box(t2, "Thumb 2", {0, float(-tl / 2), 0}, QVector3D(float(fw), float(tl * 0.94), float(ft * 0.9)), secondary);
            follower(sp + "Thumb", hand, t1, JointType::Revolute, -Z, at, master, 0.5, 0, 45);
            follower(sp + "Thumb Tip", t1, t2, JointType::Revolute, -Z, {0, float(-tl), 0}, master, 0.6, 0, 54);
            break;
        }
        }
    }

    // Actuators in catalogue order: waist, neck, left arm, right arm, left
    // leg, right leg, then the hands
    void actuators() {
        auto add = [&](const QString& name, double speed, double accel) {
            auto it = pending.find(name);
            if (it == pending.end()) return;
            Joint* j = g.getJoint(it->second);
            Motor m;
            m.name = g.makeSymbolName(name);
            m.jointId = j->id;
            m.velocity = float(speed);
            m.acceleration = float(accel);
            const int mid = g.addMotor(m);
            j->name = m.name;
            b.joints[m.name] = j->id;
            b.motors.push_back(mid);
        };
        for (const char* group : {"Waist", "Neck", "Arms", "Legs"})
            for (int side : {-1, 1})
                for (const JointDef& d : jointDefs()) {
                    if (d.group != group || (!d.sided && side > 0)) continue;
                    const JointSpec js = s.joint(d.key);
                    add((d.sided ? sidePrefix(side) : QString()) + d.key, js.speed, js.accel);
                }
        for (int side : {-1, 1})
            for (auto& [name, sa] : handSpeeds)
                if (name.startsWith(sidePrefix(side))) add(name, sa.first, sa.second);
    }
};

// Joint values by name as specified (L…/R… or centre keys); sided values
// given without a prefix apply to both sides
using Values = std::map<QString, double>;

} // namespace

Built build(SceneGraph& scene, Spec spec) {
    validate(spec);
    const robot::ColorScheme& c = robot::colorSchemes()[spec.scheme];
    Built b;
    Maker m{scene, spec, b, body(spec), c.primary, c.secondary, c.accent, {}, {}};
    const bool legs = spec.base == Base::Legs;
    const double hipY = legs ? spec.dim("Ha") + spec.dim("Ls") + spec.dim("Lt") : spec.dim("Hb");
    const double S = m.w.S;

    b.root = m.frame(-1, spec.name, {});
    if (!legs) {
        const double r = std::max(spec.dim("Wh") * 1.6, 0.35 * S);
        m.cyl(b.root, "Pedestal Foot", {0, float(0.02 * S), 0}, Y, r, 0.04 * S, m.secondary);
        const double top = hipY - 0.08 * S;
        if (top > 0.04 * S)
            m.cyl(b.root, "Pedestal", {0, float((0.04 * S + top) / 2), 0}, Y, 0.13 * S, top - 0.04 * S,
                  spec.style == Style::Slim ? kMetal : m.primary);
    }
    const int pelvis = m.frame(b.root, "Pelvis", {0, float(hipY), 0});
    m.buildPelvis(pelvis, !legs);
    if (legs)
        for (int side : {-1, 1}) m.buildLeg(pelvis, side);
    const int torso = m.buildTorso(pelvis);
    m.buildHead(torso);
    for (int side : {-1, 1}) m.buildArm(torso, side);
    m.actuators();
    scene.applyCouplings();

    // Poses: every body joint in each (so a MoveToPose fully defines the
    // posture), starting from Home
    auto value = [&](const QString& name) -> std::pair<bool, double> {
        for (const JointDef& d : jointDefs()) {
            for (int side : {-1, 0, 1}) {
                if ((side == 0) == d.sided) continue;
                if ((d.sided ? sidePrefix(side) : QString()) + d.key == name) return {true, spec.joint(d.key).home};
            }
        }
        return {false, 0};
    };
    std::map<QString, int> bodyJoints;   // name as specified → joint id (hands left out)
    for (auto& [name, jid] : m.pending)
        if (value(name).first) bodyJoints[name] = jid;

    auto makePose = [&](const QString& label, const Values& v) -> int {
        Pose p;
        p.name = spec.name + " " + label;
        bool differs = label == "Home";
        for (auto& [name, jid] : bodyJoints) {
            const Joint* j = scene.getJoint(jid);
            double val = value(name).second;
            QString base = name;
            if (name.startsWith('L') || name.startsWith('R'))
                if (jointDef(name.mid(1)) && jointDef(name.mid(1))->sided) base = name.mid(1);
            if (auto it = v.find(name); it != v.end()) val = it->second;
            else if (auto it2 = v.find(base); it2 != v.end()) val = it2->second;
            val = std::clamp(val, double(j->limitMin), double(j->limitMax));
            if (std::fabs(val - value(name).second) > 1e-6) differs = true;
            p.jointValues[jid] = float(val);
        }
        if (!differs || p.jointValues.empty()) return -1;
        const int id = scene.addPose(p);
        b.poses.push_back(id);
        return id;
    };

    const int home = makePose("Home", {});
    if (spec.demo && home >= 0) {
        std::vector<std::pair<QString, int>> steps;
        const int tpose = makePose("T-Pose", {{"ShoulderRoll", 90}});
        const int waveA = makePose("Wave A", {{"RShoulderRoll", 150}, {"RElbow", 35}, {"NeckYaw", -25}, {"NeckRoll", 8}});
        const int waveB = makePose("Wave B", {{"RShoulderRoll", 115}, {"RElbow", 35}, {"NeckYaw", -25}, {"NeckRoll", 8}});
        const int stepL = !legs ? -1 : makePose("Step L", {{"LHipPitch", 50}, {"LKnee", 75}, {"LAnklePitch", 25},
                                              {"RShoulderPitch", 30}, {"LShoulderPitch", -20}, {"Elbow", 20}});
        const int stepR = !legs ? -1 : makePose("Step R", {{"RHipPitch", 50}, {"RKnee", 75}, {"RAnklePitch", 25},
                                              {"LShoulderPitch", 30}, {"RShoulderPitch", -20}, {"Elbow", 20}});
        const int bow = makePose("Bow", {{"WaistPitch", 30}, {"NeckPitch", 20}});
        for (auto [label, pose] : std::initializer_list<std::pair<QString, int>>{
                 {"T-Pose", tpose}, {"Home", home}, {"Wave A", waveA}, {"Wave B", waveB}, {"Wave A", waveA},
                 {"Wave B", waveB}, {"Home", home}, {"Step L", stepL}, {"Home", home}, {"Step R", stepR},
                 {"Home", home}, {"Bow", bow}, {"Home", home}})
            if (pose >= 0 && (steps.empty() || steps.back().second != pose)) steps.push_back({label, pose});
        if (steps.size() > 2) {
            Sequence seq;
            seq.name = scene.makeSymbolName(spec.name + "Demo");
            seq.loop = true;
            int prev = -1;
            for (auto& [label, pose] : steps) {
                Operation op;
                op.id = scene.allocateId();
                op.name = label;
                op.type = OperationType::MoveToPose;
                op.poseId = pose;
                if (prev >= 0) op.predecessors = {prev};
                prev = op.id;
                seq.operations.push_back(op);
            }
            b.sequence = scene.addSequence(seq);
        }
    }
    scene.notifyAutomationChanged();
    emit scene.objectChanged(b.root);
    return b;
}

// ── Drawing ──────────────────────────────────────────────────────────────────

std::vector<robot::DimensionLine> dimensionLines(const Spec& in) {
    Spec s = in;
    validate(s);
    const Body w = body(s);
    std::vector<robot::DimensionLine> d;
    auto add = [&](const QString& sym, QVector3D a, QVector3D b, QVector3D side, double off, bool front = false) {
        robot::DimensionLine l;
        l.symbol = sym;
        l.from = a;
        l.to = b;
        l.offsetDir = side;
        l.offset = off;
        l.front = front;
        d.push_back(l);
    };
    const QVector3D back(-1, 0, 0), fwd(1, 0, 0), up(0, 1, 0), down(0, -1, 0), left(0, 0, -1);
    const bool legs = s.base == Base::Legs;
    const double Ha = s.dim("Ha"), Ls = s.dim("Ls"), Lt = s.dim("Lt"), Hp = s.dim("Hp"), Ht = s.dim("Ht"),
                 Hn = s.dim("Hn"), Dh = s.dim("Dh"), Ws = s.dim("Ws"), Wh = s.dim("Wh"), Lu = s.dim("Lu"),
                 Lf = s.dim("Lf"), Lh = s.dim("Lh"), Lft = s.dim("Lft"), Wft = s.dim("Wft");
    const float hip = float(legs ? Ha + Ls + Lt : s.dim("Hb"));
    const float waist = hip + float(Hp), sh = waist + float(Ht), neck = sh + float(Hn), top = neck + float(Dh);
    const double off = 0.1 * w.S;

    // Side view: legs behind, body in front
    if (legs) {
        const double rear = 0.3 * Lft + off;
        add("Ha", {0, 0, 0}, {0, float(Ha), 0}, back, rear);
        add("Ls", {0, float(Ha), 0}, {0, float(Ha + Ls), 0}, back, rear);
        add("Lt", {0, float(Ha + Ls), 0}, {0, hip, 0}, back, rear);
        add("Lft", {float(-0.3 * Lft), 0, 0}, {float(0.7 * Lft), 0, 0}, down, 0.6 * off);
    } else {
        add("Hb", {0, 0, 0}, {0, hip, 0}, back, 0.2 * w.S + off);
    }
    const double front = w.torsoD / 2 + off;
    if (Hp > 0.001) add("Hp", {0, hip, 0}, {0, waist, 0}, fwd, front);
    add("Ht", {0, waist, 0}, {0, sh, 0}, fwd, front);
    if (Hn > 0.001) add("Hn", {0, sh, 0}, {0, neck, 0}, fwd, front);
    add("Dh", {0, neck, 0}, {0, top, 0}, fwd, front);

    // Front view: widths, and the left arm (on the drawing's right)
    add("Ws", {0, sh, float(-Ws / 2)}, {0, sh, float(Ws / 2)}, up, top - sh + 0.6 * off, true);
    add("Wh", {0, hip, float(-Wh / 2)}, {0, hip, float(Wh / 2)}, down, legs ? 0.35 * Lt : 0.15 * w.S, true);
    const float az = float(-Ws / 2);
    const double out = w.upper / 2 + off;
    add("Lu", {0, sh, az}, {0, float(sh - Lu), az}, left, out, true);
    add("Lf", {0, float(sh - Lu), az}, {0, float(sh - Lu - Lf), az}, left, out, true);
    add("Lh", {0, float(sh - Lu - Lf), az}, {0, float(sh - Lu - Lf - Lh), az}, left, out, true);
    if (legs)
        add("Wft", {0, 0, float(Wh / 2 - Wft / 2)}, {0, 0, float(Wh / 2 + Wft / 2)}, down, 0.6 * off, true);
    return d;
}

} // namespace humanoid
