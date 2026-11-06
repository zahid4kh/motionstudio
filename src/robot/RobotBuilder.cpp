#include "RobotBuilder.h"
#include "../scene/SceneGraph.h"
#include <QJsonArray>
#include <QQuaternion>
#include <QtMath>
#include <algorithm>
#include <cmath>

namespace robot {

// ── Catalogue ────────────────────────────────────────────────────────────────

QString kindName(Kind k) {
    switch (k) {
    case Kind::Scara:       return "SCARA (4-axis)";
    case Kind::Palletizer:  return "Palletizer (4-axis)";
    case Kind::Articulated: return "Articulated (6-axis)";
    }
    return {};
}

int axisCount(Kind k) { return k == Kind::Articulated ? 6 : 4; }

std::vector<DimensionDef> dimensionDefs(Kind k) {
    switch (k) {
    case Kind::Scara: return {
        {"H",  "H",  "Arm height",     "Floor to the underside of arm 1 (the plane J1 and J2 turn in)", 0.15, 2.0, 0.45},
        {"L1", "L1", "Arm 1 length",   "J1 axis to J2 axis",                                            0.06, 1.5, 0.325},
        {"L2", "L2", "Arm 2 length",   "J2 axis to the quill (J3/J4) axis",                             0.06, 1.2, 0.275},
        {"S",  "S",  "Z stroke",       "How far J3 lowers the flange",                                  0.02, 0.8, 0.20},
        {"Fz", "Fz", "Flange height",  "Floor to the flange at the top of the stroke (J3 = 0)",         0.02, 2.0, 0.27},
    };
    case Kind::Palletizer: return {
        {"h0", "h0", "Pedestal",        "Floor to the robot's mounting face",                0.0,  2.0, 0.0},
        {"h1", "h1", "Shoulder height", "Mounting face to the J2 axis",                      0.2,  3.0, 0.90},
        {"a1", "a1", "Shoulder offset", "J1 axis to the J2 axis, horizontally",              0.0,  1.0, 0.30},
        {"a2", "a2", "Lower arm",       "J2 axis to the J3 (elbow) axis",                    0.2,  3.0, 1.10},
        {"a3", "a3", "Upper arm",       "J3 axis to the wrist axis",                         0.2,  3.5, 1.30},
        {"d5", "d5", "Wrist drop",      "Wrist axis down to the flange (the J4 face)",       0.04, 1.0, 0.30},
    };
    case Kind::Articulated: return {
        {"h0", "h0", "Pedestal",        "Floor to the robot's mounting face",                         0.0,  2.0, 0.0},
        {"h1", "h1", "Shoulder height", "Mounting face to the A2 axis",                               0.12, 2.5, 0.45},
        {"a1", "a1", "Shoulder offset", "A1 axis to the A2 axis, horizontally",                       0.0,  1.0, 0.15},
        {"a2", "a2", "Lower arm",       "A2 axis to the A3 axis",                                     0.1,  3.0, 0.45},
        {"a3", "a3", "Elbow offset",    "A3 axis to the forearm (A4) axis, across the forearm",       0.0,  0.8, 0.035},
        {"d4", "d4", "Forearm",         "A3 axis to the wrist centre (A5 axis), along the forearm",   0.1,  3.0, 0.42},
        {"d6", "d6", "Wrist to flange", "Wrist centre to the flange face",                            0.03, 0.8, 0.08},
    };
    }
    return {};
}

std::vector<AxisDef> axisDefs(Kind k) {
    switch (k) {
    case Kind::Scara: return {
        {"J1", "Arm 1 rotation",            false, {-132, 132, 375, 1500, 0}},
        {"J2", "Arm 2 rotation",            false, {-145, 145, 600, 2400, 0}},
        {"J3", "Z stroke (+ down)",         true,  {0, 0.2, 1.1, 5, 0}},
        {"J4", "Tool rotation",             false, {-360, 360, 2000, 8000, 0}},
    };
    case Kind::Palletizer: return {
        {"J1", "Base rotation",             false, {-180, 180, 125, 400, 0}},
        {"J2", "Lower arm (+ forward)",     false, {-40, 85, 110, 400, 0}},
        {"J3", "Upper arm (+ down, level = 0)", false, {-60, 75, 110, 400, 0}},
        {"J4", "Tool rotation",             false, {-360, 360, 300, 1200, 0}},
    };
    case Kind::Articulated: return {
        {"A1", "Base rotation",             false, {-170, 170, 175, 700, 0}},
        {"A2", "Lower arm (+ forward)",     false, {-95, 155, 175, 700, 0}},
        {"A3", "Forearm (+ down)",          false, {-180, 75, 175, 700, 0}},
        {"A4", "Forearm roll",              false, {-200, 200, 360, 1400, 0}},
        {"A5", "Wrist bend (+ down)",       false, {-125, 125, 360, 1400, 90}},
        {"A6", "Flange roll",               false, {-360, 360, 500, 2000, 0}},
    };
    }
    return {};
}

QStringList styleNames() { return {"Industrial", "Cobot (rounded)", "Slim (lightweight)", "Heavy-duty"}; }

const std::vector<ColorScheme>& colorSchemes() {
    static const std::vector<ColorScheme> s = {
        {"Industrial orange", QColor("#ef7d22"), QColor("#363a41"), QColor("#c9ced6")},
        {"Signal yellow",     QColor("#f1bf2c"), QColor("#2f3237"), QColor("#d6d9de")},
        {"Cobot white",       QColor("#e9ecef"), QColor("#59616c"), QColor("#2f7fd8")},
        {"Factory red",       QColor("#cf3b30"), QColor("#2c2f34"), QColor("#d5d8dc")},
        {"Steel blue",        QColor("#3d6fb6"), QColor("#2a2e35"), QColor("#d0d6de")},
        {"Graphite",          QColor("#50555e"), QColor("#25282d"), QColor("#ef7d22")},
        {"Mint",              QColor("#3fb39b"), QColor("#2b3036"), QColor("#e9ecef")},
        {"Cream",             QColor("#ece3cf"), QColor("#3b3f45"), QColor("#c0533a")},
    };
    return s;
}

const std::vector<ToolDef>& toolDefs() {
    static const std::vector<ToolDef> t = {
        {Tool::None,               "Bare flange",          "No tool: the TCP is the flange centre.", "circle-dot"},
        {Tool::SuctionCup,         "Suction cup",          "One bellows cup. Adds input Vacuum and zone sensor AtCup at the cup.", "circle-dot"},
        {Tool::VacuumPad,          "Vacuum pad (4 cups)",  "A plate with four cups for boxes, sheets and layers. Adds input Vacuum and zone sensor AtPad.", "grid-2x2"},
        {Tool::MagneticGripper,    "Magnetic gripper",     "An electro-magnet for steel parts. Adds input Magnet and zone sensor AtMagnet.", "magnet"},
        {Tool::ParallelGripper,    "Parallel finger gripper", "Two fingers on one actuator Fingers (the right one is coupled). Input Grip closes them; zone sensor InGripper sees the part.", "hand-grab"},
        {Tool::ThreeFingerGripper, "Three-finger gripper", "Three centring fingers on actuator Jaws (two coupled). Input Grip closes them; zone sensor InGripper.", "hand-grab"},
        {Tool::AngularGripper,     "Angular gripper",      "Two pivoting jaws on actuator Jaws (degrees, one coupled). Input Grip closes them; zone sensor InGripper.", "hand-grab"},
        {Tool::ForkGripper,        "Fork / bag gripper",   "Tines under the load and a clamp plate on actuator Clamp. Input Grip clamps; zone sensor OnFork.", "forklift"},
        {Tool::Drill,              "Drill",                "A drill motor with a spinning chuck: speed actuator Spindle runs while input SpindleOn is on. TCP at the bit tip.", "drill"},
        {Tool::Screwdriver,        "Screwdriver",          "A nut runner: speed actuator Screw runs while input ScrewOn is on. TCP at the bit tip.", "wrench"},
        {Tool::DeburringSpindle,   "Deburring spindle",    "A finned spindle with a ball cutter: speed actuator Spindle, input SpindleOn. TCP at the ball centre.", "cog"},
        {Tool::WeldingTorch,       "MIG welding torch",    "A torch with a 25° gooseneck; the TCP sits at the wire tip, pointing along the nozzle. Adds input Weld.", "flame"},
        {Tool::Dispenser,          "Glue dispenser",       "A syringe with a needle; TCP at the needle tip. Adds input Dispense.", "droplet"},
        {Tool::SprayGun,           "Spray gun",            "A paint gun; the TCP is the spray spot 0.2 m in front of the nozzle. Adds input Spray.", "spray-can"},
        {Tool::LaserHead,          "Laser cutting head",   "A focusing head; TCP at the focal point just below the nozzle. Adds input Laser.", "zap"},
        {Tool::Camera,             "Vision camera",        "A camera with a ring light; the TCP is at its working distance. Adds distance sensor Range along the view.", "camera"},
    };
    return t;
}

const ToolDef& toolDef(Tool t) {
    for (const ToolDef& d : toolDefs()) if (d.tool == t) return d;
    return toolDefs().front();
}

double Spec::dim(const QString& key) const { return dims.value(key, 0.0); }

Spec defaultSpec(Kind k) {
    Spec s;
    s.kind = k;
    s.prefix = k == Kind::Articulated ? "A" : "J";
    for (const DimensionDef& d : dimensionDefs(k)) s.dims[d.key] = d.value;
    for (const AxisDef& a : axisDefs(k)) s.axes.push_back(a.defaults);
    s.tool = k == Kind::Scara ? Tool::SuctionCup : k == Kind::Palletizer ? Tool::VacuumPad : Tool::ParallelGripper;
    return s;
}

namespace {

Spec preset(Kind k, std::initializer_list<std::pair<const char*, double>> dims, Style style, int scheme,
            Tool tool, double speedFactor = 1.0, double linkScale = 1.0, double toolScale = 1.0) {
    Spec s = defaultSpec(k);
    for (auto& [key, v] : dims) s.dims[key] = v;
    for (AxisSpec& a : s.axes) { a.speed *= speedFactor; a.accel *= speedFactor; }
    s.style = style;
    s.scheme = scheme;
    s.tool = tool;
    s.linkScale = linkScale;
    s.toolScale = toolScale;
    validate(s);
    return s;
}

} // namespace

const std::vector<Preset>& presets() {
    using K = Kind;
    using S = Style;
    using T = Tool;
    static const std::vector<Preset> p = {
        // SCARA
        {"SCARA 400 · Suction cup", "Compact pick & place of light parts from a tray.",
         preset(K::Scara, {{"H", .38}, {"L1", .225}, {"L2", .175}, {"S", .15}, {"Fz", .22}}, S::Industrial, 2, T::SuctionCup)},
        {"SCARA 600 · Finger gripper", "General assembly: grips parts by their sides.",
         preset(K::Scara, {{"H", .45}, {"L1", .325}, {"L2", .275}, {"S", .2}, {"Fz", .27}}, S::Industrial, 2, T::ParallelGripper)},
        {"SCARA 600 · Screwdriving", "Drives screws straight down; J4 holds the bit's angle.",
         preset(K::Scara, {{"H", .45}, {"L1", .325}, {"L2", .275}, {"S", .2}, {"Fz", .3}}, S::Industrial, 4, T::Screwdriver)},
        {"SCARA 700 · PCB drilling", "Drills holes in boards on a fixture below.",
         preset(K::Scara, {{"H", .5}, {"L1", .4}, {"L2", .3}, {"S", .2}, {"Fz", .32}}, S::Industrial, 5, T::Drill)},
        {"SCARA 500 · Glue dispensing", "Lays glue beads along a part's edge.",
         preset(K::Scara, {{"H", .42}, {"L1", .25}, {"L2", .25}, {"S", .15}, {"Fz", .25}}, S::Slim, 6, T::Dispenser)},
        {"SCARA 1000 · Vacuum pad (heavy)", "Long reach, 0.4 m stroke, a four-cup pad for boxes.",
         preset(K::Scara, {{"H", .75}, {"L1", .55}, {"L2", .45}, {"S", .4}, {"Fz", .52}}, S::HeavyDuty, 1, T::VacuumPad, 0.7)},
        {"SCARA 300 · Vision inspection", "A small, fast arm that moves a camera over parts.",
         preset(K::Scara, {{"H", .3}, {"L1", .17}, {"L2", .13}, {"S", .1}, {"Fz", .2}}, S::Slim, 5, T::Camera)},
        {"SCARA 600 · Magnetic pick", "Picks steel blanks with an electro-magnet.",
         preset(K::Scara, {{"H", .45}, {"L1", .325}, {"L2", .275}, {"S", .2}, {"Fz", .27}}, S::Cobot, 3, T::MagneticGripper)},
        {"SCARA 800 · Three-finger gripper", "Centres and grips round parts.",
         preset(K::Scara, {{"H", .55}, {"L1", .45}, {"L2", .35}, {"S", .3}, {"Fz", .32}}, S::Industrial, 0, T::ThreeFingerGripper)},
        // Palletizers
        {"Palletizer 2.4 m · Layer vacuum pad", "Stacks whole layers of boxes on a pallet.",
         preset(K::Palletizer, {{"h1", .9}, {"a1", .3}, {"a2", 1.1}, {"a3", 1.3}, {"d5", .3}}, S::HeavyDuty, 1, T::VacuumPad, 1.0, 1.0, 1.6)},
        {"Palletizer 3.1 m · Bag fork gripper", "Lifts sacks on tines and clamps them from above.",
         preset(K::Palletizer, {{"h1", 1.1}, {"a1", .35}, {"a2", 1.4}, {"a3", 1.6}, {"d5", .35}}, S::HeavyDuty, 0, T::ForkGripper, 0.8)},
        {"Palletizer 1.6 m · Box clamp", "Clamps single boxes between two long fingers.",
         preset(K::Palletizer, {{"h1", .6}, {"a1", .2}, {"a2", .7}, {"a3", .85}, {"d5", .22}}, S::Industrial, 4, T::ParallelGripper, 1.0, 1.0, 1.6)},
        {"Palletizer 1.4 m · Suction cup", "A light end-of-line palletizer for cartons.",
         preset(K::Palletizer, {{"h1", .55}, {"a1", .15}, {"a2", .6}, {"a3", .75}, {"d5", .2}}, S::Cobot, 2, T::SuctionCup, 1.0, 1.0, 1.5)},
        {"Palletizer 2.0 m · Magnetic sheets", "Stacks steel sheets with a magnet head.",
         preset(K::Palletizer, {{"h1", .8}, {"a1", .25}, {"a2", .9}, {"a3", 1.1}, {"d5", .26}}, S::Industrial, 5, T::MagneticGripper, 1.0, 1.0, 1.5)},
        // Articulated
        {"6-axis 0.9 m · Finger gripper", "A general-purpose handling robot.",
         preset(K::Articulated, {{"h1", .45}, {"a1", .15}, {"a2", .45}, {"a3", .035}, {"d4", .42}, {"d6", .08}}, S::Industrial, 0, T::ParallelGripper)},
        {"Cobot 0.85 m · Finger gripper", "A rounded collaborative arm without shoulder offset.",
         preset(K::Articulated, {{"h1", .35}, {"a1", 0}, {"a2", .42}, {"a3", 0}, {"d4", .39}, {"d6", .1}}, S::Cobot, 2, T::ParallelGripper, 0.6)},
        {"6-axis 1.4 m · MIG welding torch", "Arc welding on a 0.3 m riser; the TCP is at the wire tip.",
         preset(K::Articulated, {{"h0", .3}, {"h1", .5}, {"a1", .15}, {"a2", .7}, {"a3", .115}, {"d4", .795}, {"d6", .085}}, S::Industrial, 4, T::WeldingTorch)},
        {"6-axis 1.8 m · Spray painting", "A slim painting arm; the TCP is the spray spot.",
         preset(K::Articulated, {{"h0", .2}, {"h1", .5}, {"a1", .15}, {"a2", .9}, {"a3", .1}, {"d4", .9}, {"d6", .1}}, S::Slim, 7, T::SprayGun)},
        {"6-axis 2.6 m · Heavy handling", "Moves heavy boxes with a four-cup vacuum pad.",
         preset(K::Articulated, {{"h1", .75}, {"a1", .35}, {"a2", 1.15}, {"a3", .25}, {"d4", 1.2}, {"d6", .215}}, S::HeavyDuty, 1, T::VacuumPad, 0.6)},
        {"6-axis 0.7 m · Glue dispensing", "A small arm drawing glue beads in 3D.",
         preset(K::Articulated, {{"h1", .33}, {"a1", .05}, {"a2", .33}, {"a3", .03}, {"d4", .33}, {"d6", .07}}, S::Slim, 6, T::Dispenser)},
        {"6-axis 1.1 m · Deburring spindle", "Follows edges with a ball cutter.",
         preset(K::Articulated, {{"h1", .45}, {"a1", .15}, {"a2", .56}, {"a3", .03}, {"d4", .52}, {"d6", .1}}, S::Industrial, 5, T::DeburringSpindle)},
        {"6-axis 1.4 m · Laser cutting", "Cuts sheet parts with a focusing head.",
         preset(K::Articulated, {{"h1", .5}, {"a1", .15}, {"a2", .7}, {"a3", .115}, {"d4", .795}, {"d6", .085}}, S::Industrial, 3, T::LaserHead)},
        {"6-axis 0.6 m · Three-finger gripper", "A compact arm that centres round parts.",
         preset(K::Articulated, {{"h1", .3}, {"a1", .03}, {"a2", .27}, {"a3", .02}, {"d4", .29}, {"d6", .07}}, S::Cobot, 4, T::ThreeFingerGripper)},
        {"6-axis 1.2 m · Drilling", "Drills holes at any angle.",
         preset(K::Articulated, {{"h1", .48}, {"a1", .15}, {"a2", .6}, {"a3", .04}, {"d4", .6}, {"d6", .09}}, S::Industrial, 0, T::Drill)},
        {"6-axis 1.0 m · Angular gripper", "Grips with two pivoting jaws.",
         preset(K::Articulated, {{"h1", .45}, {"a1", .1}, {"a2", .5}, {"a3", .03}, {"d4", .45}, {"d6", .08}}, S::Industrial, 7, T::AngularGripper)},
        {"Cobot 1.3 m · Camera inspection", "Moves a camera around a part to inspect it.",
         preset(K::Articulated, {{"h1", .4}, {"a1", 0}, {"a2", .61}, {"a3", 0}, {"d4", .57}, {"d6", .12}}, S::Cobot, 5, T::Camera, 0.6)},
        {"Cobot 1.0 m · Screwdriving", "Drives screws at any angle.",
         preset(K::Articulated, {{"h1", .38}, {"a1", 0}, {"a2", .5}, {"a3", 0}, {"d4", .45}, {"d6", .1}}, S::Cobot, 2, T::Screwdriver, 0.6)},
        {"6-axis 1.6 m · Magnetic gripper", "Handles steel parts with a magnet.",
         preset(K::Articulated, {{"h1", .55}, {"a1", .2}, {"a2", .75}, {"a3", .1}, {"d4", .8}, {"d6", .12}}, S::HeavyDuty, 3, T::MagneticGripper, 0.8)},
    };
    return p;
}

QStringList validate(Spec& s) {
    QStringList problems;
    const std::vector<DimensionDef> defs = dimensionDefs(s.kind);
    QMap<QString, double> dims;
    for (const DimensionDef& d : defs) dims[d.key] = std::clamp(s.dims.value(d.key, d.value), d.min, d.max);
    s.dims = dims;
    const std::vector<AxisDef> adefs = axisDefs(s.kind);
    s.axes.resize(adefs.size());
    for (size_t i = 0; i < adefs.size(); ++i) {
        AxisSpec& a = s.axes[i];
        if (a.speed <= 0) a.speed = adefs[i].defaults.speed;
        if (a.accel < 0) a.accel = 0;
        if (a.min > a.max) std::swap(a.min, a.max);
        if (a.max - a.min < 1e-6) {
            problems << QString("%1 has no range: set its minimum below its maximum").arg(adefs[i].name);
            a.max = a.min + (adefs[i].prismatic ? 0.01 : 1);
        }
        if (a.home < a.min || a.home > a.max) {
            problems << QString("%1's home value is outside its range: it is clamped").arg(adefs[i].name);
            a.home = std::clamp(a.home, a.min, a.max);
        }
    }
    if (s.kind == Kind::Scara) {
        AxisSpec& z = s.axes[2];
        z.min = 0;
        z.max = s.dims["S"];
        z.home = std::clamp(z.home, z.min, z.max);
        if (s.dims["Fz"] > s.dims["H"]) {
            problems << "The flange must hang below arm 1 (Fz ≤ H): Fz is set to H";
            s.dims["Fz"] = s.dims["H"];
        }
        if (s.dims["Fz"] - s.dims["S"] < 0)
            problems << QString("At full stroke the flange goes %1 mm below the floor (Fz − S < 0)")
                            .arg(std::round((s.dims["S"] - s.dims["Fz"]) * 1000));
        if (s.dims["L2"] > s.dims["L1"] * 1.6)
            problems << "Arm 2 is much longer than arm 1: the robot cannot reach near its own base";
    }
    if (s.kind == Kind::Articulated && s.dims["d4"] + s.dims["d6"] < 0.15 * s.dims["a2"])
        problems << "The forearm is very short for the lower arm: the reach is mostly below the shoulder";
    s.linkScale = std::clamp(s.linkScale, 0.4, 2.5);
    s.toolScale = std::clamp(s.toolScale, 0.3, 4.0);
    s.scheme = std::clamp(s.scheme, 0, int(colorSchemes().size()) - 1);
    if (s.name.trimmed().isEmpty()) s.name = "Robot";
    QString prefix;
    for (QChar c : s.prefix) if (c.isLetterOrNumber() || c == '_') prefix += c;
    if (prefix.isEmpty() || prefix[0].isDigit()) prefix = s.kind == Kind::Articulated ? "A" : "J";
    s.prefix = prefix;
    return problems;
}

// ── JSON ─────────────────────────────────────────────────────────────────────

namespace {
const char* kindKey(Kind k) { return k == Kind::Scara ? "scara" : k == Kind::Palletizer ? "palletizer" : "articulated"; }
const char* toolKeys[] = {"none", "suctionCup", "vacuumPad", "magnet", "parallelGripper", "threeFinger",
                          "angularGripper", "fork", "drill", "screwdriver", "deburring", "weldingTorch",
                          "dispenser", "sprayGun", "laser", "camera"};
const char* styleKeys[] = {"industrial", "cobot", "slim", "heavy"};
} // namespace

QJsonObject toJson(const Spec& s) {
    QJsonObject o;
    o["kind"] = kindKey(s.kind);
    o["name"] = s.name;
    o["prefix"] = s.prefix;
    QJsonObject dims;
    for (auto it = s.dims.begin(); it != s.dims.end(); ++it) dims[it.key()] = it.value();
    o["dims"] = dims;
    QJsonArray axes;
    for (const AxisSpec& a : s.axes)
        axes.append(QJsonObject{{"min", a.min}, {"max", a.max}, {"speed", a.speed}, {"accel", a.accel}, {"home", a.home}});
    o["axes"] = axes;
    o["style"] = styleKeys[int(s.style)];
    o["scheme"] = s.scheme;
    o["linkScale"] = s.linkScale;
    o["tool"] = toolKeys[int(s.tool)];
    o["toolScale"] = s.toolScale;
    return o;
}

Spec fromJson(const QJsonObject& o) {
    const QString kind = o["kind"].toString();
    Spec s = defaultSpec(kind == "scara" ? Kind::Scara : kind == "palletizer" ? Kind::Palletizer : Kind::Articulated);
    s.name = o["name"].toString(s.name);
    s.prefix = o["prefix"].toString(s.prefix);
    const QJsonObject dims = o["dims"].toObject();
    for (auto it = dims.begin(); it != dims.end(); ++it)
        if (s.dims.contains(it.key())) s.dims[it.key()] = it.value().toDouble();
    const QJsonArray axes = o["axes"].toArray();
    for (int i = 0; i < axes.size() && i < int(s.axes.size()); ++i) {
        const QJsonObject a = axes[i].toObject();
        AxisSpec& x = s.axes[i];
        x.min = a["min"].toDouble(x.min);
        x.max = a["max"].toDouble(x.max);
        x.speed = a["speed"].toDouble(x.speed);
        x.accel = a["accel"].toDouble(x.accel);
        x.home = a["home"].toDouble(x.home);
    }
    for (int i = 0; i < 4; ++i) if (o["style"].toString() == styleKeys[i]) s.style = Style(i);
    s.scheme = o["scheme"].toInt(s.scheme);
    s.linkScale = o["linkScale"].toDouble(s.linkScale);
    for (int i = 0; i < int(std::size(toolKeys)); ++i) if (o["tool"].toString() == toolKeys[i]) s.tool = Tool(i);
    s.toolScale = o["toolScale"].toDouble(s.toolScale);
    validate(s);
    return s;
}

// ── Building ─────────────────────────────────────────────────────────────────

namespace {

const QVector3D X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);
const QColor kMetal("#b9bfc7"), kChrome("#dde2e8"), kDark("#2b2e33"), kRubber("#1f2124"),
             kGold("#d4a24c"), kCopper("#b87333");

QVector3D eulerFromY(const QVector3D& dir) {
    const QVector3D d = dir.normalized();
    if (QVector3D::dotProduct(d, Y) < -0.99999f) return {180, 0, 0};
    QMatrix4x4 m;
    m.rotate(QQuaternion::rotationTo(Y, d));
    return SceneGraph::toTransform(m).rotation;
}

// Adds frames, primitives, joints and actuators, with the robot's look
struct Maker {
    SceneGraph& g;
    Style style;
    QColor primary, secondary, accent;

    int frame(int parent, const QString& name, QVector3D pos, QVector3D rot = {}) {
        const int id = g.addObject(name, PrimitiveType::ImportedMesh, parent);
        SceneObject* o = g.getObject(id);
        o->transform.position = pos;
        o->transform.rotation = rot;
        return id;
    }
    int prim(int parent, const QString& name, PrimitiveType t, QVector3D center, QVector3D size,
             const QColor& c, QVector3D rot = {}) {
        const int id = g.addObject(name, t, parent);
        SceneObject* o = g.getObject(id);
        o->transform.position = center;
        o->transform.rotation = rot;
        o->transform.scale = size;
        o->color = c;
        return id;
    }
    int box(int parent, const QString& name, QVector3D c, QVector3D size, const QColor& col) {
        return prim(parent, name, PrimitiveType::Cube, c, size, col);
    }
    int cyl(int parent, const QString& name, QVector3D c, QVector3D axis, double dia, double len, const QColor& col) {
        return prim(parent, name, PrimitiveType::Cylinder, c, QVector3D(dia, len, dia), col, eulerFromY(axis));
    }
    int rod(int parent, const QString& name, QVector3D a, QVector3D b, double dia, const QColor& col) {
        return cyl(parent, name, (a + b) / 2, b - a, dia, (b - a).length(), col);
    }
    int ball(int parent, const QString& name, QVector3D c, double dia, const QColor& col) {
        return prim(parent, name, PrimitiveType::Sphere, c, QVector3D(dia, dia, dia), col);
    }

    // A link from a to b: `across` is its size across the motion plane's
    // normal side (in the local frame it becomes the box's X after turning
    // Y onto a→b), `depth` along Z
    void link(int parent, const QString& name, QVector3D a, QVector3D b, double across, double depth) {
        const double len = (b - a).length();
        if (len < 1e-4) return;
        switch (style) {
        case Style::Industrial:
        case Style::HeavyDuty:
            prim(parent, name, PrimitiveType::Cube, (a + b) / 2, QVector3D(across, len, depth), primary, eulerFromY(b - a));
            break;
        case Style::Cobot:
            rod(parent, name, a, b, std::min(across, depth) * 0.9, primary);
            break;
        case Style::Slim:
            rod(parent, name, a, b, std::min(across, depth) * 0.5, primary);
            break;
        }
    }
    // A joint housing turning about `axis`
    void hub(int parent, const QString& name, QVector3D c, QVector3D axis, double dia, double len) {
        switch (style) {
        case Style::Industrial:
        case Style::HeavyDuty:
            cyl(parent, name, c, axis, dia, len, secondary);
            break;
        case Style::Cobot: {
            cyl(parent, name, c, axis, dia, len, primary);
            const QVector3D n = axis.normalized();
            cyl(parent, name + " Ring", c + n * (len * 0.5 - len * 0.04), axis, dia * 1.03, len * 0.08, accent);
            cyl(parent, name + " Cap", c - n * (len * 0.5 + 0.004), axis, dia * 0.7, 0.008, secondary);
            break;
        }
        case Style::Slim:
            ball(parent, name, c, dia * 0.75, secondary);
            break;
        }
    }
    // A cast housing (motor covers, turrets)
    void housing(int parent, const QString& name, QVector3D c, QVector3D size) {
        switch (style) {
        case Style::Industrial:
        case Style::HeavyDuty:
            box(parent, name, c, size, primary);
            break;
        case Style::Cobot:
            cyl(parent, name, c, Y, std::max(size.x(), size.z()) * 0.9, size.y(), primary);
            break;
        case Style::Slim:
            box(parent, name, c, size * 0.7f, primary);
            break;
        }
    }

    int joint(const QString& name, int parent, int child, JointType type, QVector3D axis, QVector3D pivot,
              double min, double max, double home, bool continuous = false) {
        Joint j;
        j.name = name;
        j.type = type;
        j.axis = axis;
        j.pivot = pivot;
        j.limitMin = float(min);
        j.limitMax = float(max);
        j.initialAngle = float(home);
        j.parentBodyId = parent;
        j.childBodyId = child;
        j.continuous = continuous;
        return g.addJoint(j);
    }
    int actuator(const QString& base, int jointId, double speed, double accel) {
        Motor m;
        m.name = g.makeSymbolName(base);
        m.jointId = jointId;
        m.velocity = float(speed);
        m.acceleration = float(accel);
        return g.addMotor(m);
    }
    // A follower that keeps a frame level: value = −master
    int follower(const QString& name, int parent, int child, QVector3D axis, QVector3D pivot, int master) {
        const int id = joint(name, parent, child, JointType::Revolute, axis, pivot, -360, 360, 0, true);
        Joint* j = g.getJoint(id);
        j->coupledTo = master;
        j->ratio = -1;
        j->offset = 0;
        return id;
    }
    QString input(const QString& base, const QString& description) {
        Signal s;
        s.name = g.makeSymbolName(base);
        s.kind = SignalKind::Input;
        s.type = SignalType::Bool;
        s.description = description;
        g.addSignal(s);
        return s.name;
    }
    void zone(const QString& base, int mount, QVector3D offset, QVector3D size, const QString& description) {
        Signal s;
        s.name = g.makeSymbolName(base);
        s.kind = SignalKind::Sensor;
        s.description = description;
        s.sensor.type = SensorType::Zone;
        s.sensor.objectId = mount;
        s.sensor.offset = offset;
        s.sensor.size = size;
        s.sensor.target = SensorTarget::RigidBodies;
        s.type = sensorSignalType(s.sensor.type);
        g.addSignal(s);
    }
};

QString num(double v) { return QString::number(v, 'g', 5); }

// The end effector, built in a frame whose −Y is the approach direction and
// whose origin is the flange face. Returns the TCP frame.
int buildTool(Maker& m, Tool tool, int mount, QVector3D mountRot, double k, Built& b) {
    const ToolDef& def = toolDef(tool);
    const int root = m.frame(mount, tool == Tool::None ? QString("Flange Mount") : def.name, {}, mountRot);
    b.toolRoot = root;
    QVector3D tcp(0, 0, 0);
    QVector3D tcpRot;
    auto P = [k](double x, double y, double z = 0) { return QVector3D(float(x * k), float(y * k), float(z * k)); };
    auto cylY = [&](int parent, const QString& n, double y, double dia, double len, const QColor& c) {
        return m.cyl(parent, n, P(0, y), Y, dia * k, len * k, c);
    };
    // Grippers' fingers: one actuator, the rest coupled to the first
    auto gripActuator = [&](const QString& base, int jointId, double closed, double speed, double accel) {
        const QString grip = m.input("Grip", QString("Close the %1").arg(def.name.toLower()));
        const int mid = m.actuator(base, jointId, speed, accel);
        m.g.getMotor(mid)->targetExpr = QString("%1 ? %2 : 0").arg(grip, num(closed));
    };
    auto spindle = [&](const QString& base, const QString& on, int jointId, double speed) {
        const QString in = m.input(on, QString("Run the %1").arg(def.name.toLower()));
        const int mid = m.actuator(base, jointId, speed, speed * 2);
        Motor* mo = m.g.getMotor(mid);
        mo->mode = ActuatorMode::Speed;
        mo->activeExpr = in;
    };

    switch (tool) {
    case Tool::None:
        break;
    case Tool::SuctionCup:
        cylY(root, "Adapter", -0.01, 0.05, 0.02, kDark);
        cylY(root, "Tube", -0.055, 0.016, 0.07, kMetal);
        cylY(root, "Bellows 1", -0.096, 0.034, 0.012, kRubber);
        cylY(root, "Bellows 2", -0.11, 0.03, 0.012, kRubber);
        cylY(root, "Cup", -0.123, 0.05, 0.014, kRubber);
        tcp = P(0, -0.13);
        m.input("Vacuum", "Suction on: Grip operations hold what AtCup sees");
        break;
    case Tool::VacuumPad: {
        cylY(root, "Column", -0.025, 0.05, 0.05, kDark);
        m.box(root, "Pad Plate", P(0, -0.0625), P(0.24, 0.025, 0.16), kMetal);
        int n = 1;
        for (double x : {-0.085, 0.085})
            for (double z : {-0.05, 0.05}) {
                m.cyl(root, QString("Stem %1").arg(n), P(x, -0.085, z), Y, 0.012 * k, 0.02 * k, kMetal);
                m.cyl(root, QString("Cup %1").arg(n++), P(x, -0.1, z), Y, 0.045 * k, 0.014 * k, kRubber);
            }
        tcp = P(0, -0.107);
        m.input("Vacuum", "Suction on: Grip operations hold what AtPad sees");
        break;
    }
    case Tool::MagneticGripper:
        cylY(root, "Magnet Body", -0.035, 0.09, 0.07, QColor("#b8322a"));
        cylY(root, "Pole Face", -0.075, 0.1, 0.01, kDark);
        tcp = P(0, -0.08);
        m.input("Magnet", "Magnet on: Grip operations hold what AtMagnet sees");
        break;
    case Tool::ParallelGripper: {
        m.box(root, "Gripper Body", P(0, -0.035), P(0.13, 0.07, 0.06), kDark);
        m.box(root, "Rail", P(0, -0.075), P(0.15, 0.01, 0.04), kMetal);
        const int fl = m.frame(root, "Finger L Slide", P(0, -0.08));
        const int fr = m.frame(root, "Finger R Slide", P(0, -0.08));
        m.box(fl, "Finger L", P(-0.05, -0.04), P(0.016, 0.08, 0.03), kMetal);
        m.box(fr, "Finger R", P(0.05, -0.04), P(0.016, 0.08, 0.03), kMetal);
        const double travel = 0.03 * k;
        const int jl = m.joint("Finger L", root, fl, JointType::Prismatic, X, {}, 0, travel, 0);
        const int jr = m.joint("Finger R", root, fr, JointType::Prismatic, X, {}, -travel, 0, 0);
        Joint* r = m.g.getJoint(jr);
        r->coupledTo = jl;
        r->ratio = -1;
        gripActuator("Fingers", jl, travel, 0.1 * k, 1.0 * k);
        tcp = P(0, -0.14);
        break;
    }
    case Tool::ThreeFingerGripper: {
        cylY(root, "Gripper Body", -0.04, 0.11, 0.08, kDark);
        cylY(root, "Face", -0.085, 0.09, 0.01, kMetal);
        const double travel = 0.025 * k;
        int first = -1, n = 1;
        for (double deg : {90.0, 210.0, 330.0}) {
            const QVector3D dir(float(std::cos(qDegreesToRadians(deg))), 0, float(std::sin(qDegreesToRadians(deg))));
            const int f = m.frame(root, QString("Finger %1 Slide").arg(n), P(0, -0.09));
            m.box(f, QString("Finger %1").arg(n), dir * float(0.04 * k) + P(0, -0.035), P(0.018, 0.07, 0.018), kMetal);
            const int j = m.joint(QString("Finger %1").arg(n), root, f, JointType::Prismatic, -dir, {}, 0, travel, 0);
            if (first == -1) first = j;
            else m.g.getJoint(j)->coupledTo = first;
            ++n;
        }
        gripActuator("Jaws", first, travel, 0.08 * k, 1.0 * k);
        tcp = P(0, -0.15);
        break;
    }
    case Tool::AngularGripper: {
        m.box(root, "Gripper Body", P(0, -0.035), P(0.07, 0.07, 0.05), kDark);
        const int jl = m.frame(root, "Jaw L Pivot", P(-0.018, -0.07));
        const int jr = m.frame(root, "Jaw R Pivot", P(0.018, -0.07));
        m.cyl(jl, "Jaw L Hub", {}, Z, 0.016 * k, 0.04 * k, kMetal);
        m.cyl(jr, "Jaw R Hub", {}, Z, 0.016 * k, 0.04 * k, kMetal);
        m.box(jl, "Jaw L", P(-0.012, -0.045), P(0.012, 0.09, 0.03), kMetal);
        m.box(jr, "Jaw R", P(0.012, -0.045), P(0.012, 0.09, 0.03), kMetal);
        const int a = m.joint("Jaw L", root, jl, JointType::Revolute, Z, P(-0.018, -0.07), 0, 25, 0);
        const int c = m.joint("Jaw R", root, jr, JointType::Revolute, Z, P(0.018, -0.07), -25, 0, 0);
        Joint* r = m.g.getJoint(c);
        r->coupledTo = a;
        r->ratio = -1;
        gripActuator("Jaws", a, 20, 90, 600);
        tcp = P(0, -0.15);
        break;
    }
    case Tool::ForkGripper: {
        m.box(root, "Top Plate", P(0, -0.02), P(0.5, 0.04, 0.4), kDark);
        m.box(root, "Back Plate", P(-0.24, -0.21), P(0.02, 0.38, 0.4), kMetal);
        int n = 1;
        for (double z : {-0.15, -0.05, 0.05, 0.15})
            m.box(root, QString("Tine %1").arg(n++), P(0, -0.39, z), P(0.48, 0.02, 0.04), kMetal);
        const int clamp = m.frame(root, "Clamp Slide", P(0, -0.04));
        m.box(clamp, "Clamp Plate", P(0, -0.02), P(0.42, 0.02, 0.36), m.secondary);
        const double travel = 0.2 * k;
        const int j = m.joint("Clamp", root, clamp, JointType::Prismatic, -Y, {}, 0, travel, 0);
        gripActuator("Clamp", j, travel, 0.25 * k, 1.5 * k);
        tcp = P(0, -0.38);
        break;
    }
    case Tool::Drill: {
        cylY(root, "Drill Motor", -0.06, 0.075, 0.12, kDark);
        cylY(root, "Motor Band", -0.02, 0.078, 0.012, m.accent);
        cylY(root, "Gearbox", -0.135, 0.055, 0.03, kMetal);
        const int chuck = m.frame(root, "Chuck", P(0, -0.15));
        cylY(chuck, "Chuck Body", -0.02, 0.038, 0.04, kChrome);
        cylY(chuck, "Drill Bit", -0.08, 0.008, 0.08, kGold);
        const int j = m.joint("Chuck", root, chuck, JointType::Revolute, -Y, P(0, -0.15), -180, 180, 0, true);
        spindle("Spindle", "SpindleOn", j, 3600);
        tcp = P(0, -0.27);
        break;
    }
    case Tool::Screwdriver: {
        cylY(root, "Nut Runner", -0.07, 0.05, 0.14, kDark);
        cylY(root, "Clutch", -0.15, 0.04, 0.02, m.accent);
        const int bit = m.frame(root, "Bit Holder", P(0, -0.16));
        cylY(bit, "Sleeve", -0.025, 0.022, 0.05, kChrome);
        cylY(bit, "Bit", -0.07, 0.006, 0.04, kMetal);
        const int j = m.joint("Bit", root, bit, JointType::Revolute, -Y, P(0, -0.16), -180, 180, 0, true);
        spindle("Screw", "ScrewOn", j, 1800);
        tcp = P(0, -0.25);
        break;
    }
    case Tool::DeburringSpindle: {
        cylY(root, "Spindle Body", -0.08, 0.085, 0.16, QColor("#d0d4da"));
        int n = 1;
        for (double y : {-0.03, -0.06, -0.09, -0.12})
            cylY(root, QString("Fin %1").arg(n++), y, 0.096, 0.006, kDark);
        cylY(root, "Nose", -0.175, 0.05, 0.03, kDark);
        const int cutter = m.frame(root, "Collet", P(0, -0.19));
        cylY(cutter, "Shank", -0.025, 0.012, 0.05, kChrome);
        m.ball(cutter, "Ball Cutter", P(0, -0.055), 0.02 * k, kGold);
        const int j = m.joint("Collet", root, cutter, JointType::Revolute, -Y, P(0, -0.19), -180, 180, 0, true);
        spindle("Spindle", "SpindleOn", j, 6000);
        tcp = P(0, -0.245);
        break;
    }
    case Tool::WeldingTorch: {
        m.box(root, "Torch Bracket", P(0, -0.015), P(0.06, 0.03, 0.06), kDark);
        cylY(root, "Torch Body", -0.1, 0.045, 0.14, kDark);
        const double a = qDegreesToRadians(25.0);
        const QVector3D d(float(std::sin(a)), float(-std::cos(a)), 0);
        const QVector3D p0 = P(0, -0.17), p1 = p0 + d * float(0.12 * k), p2 = p1 + d * float(0.035 * k),
                        p3 = p2 + d * float(0.015 * k);
        m.rod(root, "Gooseneck", p0, p1, 0.028 * k, kDark);
        m.rod(root, "Nozzle", p1, p2, 0.024 * k, kCopper);
        m.rod(root, "Wire", p2, p3, 0.003 * k, kChrome);
        tcp = p3;
        tcpRot = {0, 0, 25};   // −Y along the nozzle
        m.input("Weld", "Arc on");
        break;
    }
    case Tool::Dispenser: {
        m.box(root, "Bracket", P(0, -0.015), P(0.05, 0.03, 0.04), kDark);
        cylY(root, "Cap", -0.036, 0.036, 0.012, kDark);
        const int barrel = cylY(root, "Barrel", -0.09, 0.032, 0.1, QColor("#e3e7ec"));
        m.g.getObject(barrel)->opacity = 0.55f;
        cylY(root, "Glue", -0.105, 0.026, 0.065, QColor("#e0a030"));
        cylY(root, "Needle Hub", -0.145, 0.014, 0.01, m.accent);
        m.rod(root, "Needle", P(0, -0.15), P(0, -0.18), 0.0025 * k, kChrome);
        tcp = P(0, -0.18);
        m.input("Dispense", "Glue flowing");
        break;
    }
    case Tool::SprayGun: {
        m.box(root, "Bracket", P(0, -0.02), P(0.05, 0.04, 0.05), kDark);
        m.box(root, "Gun Body", P(0, -0.09), P(0.06, 0.1, 0.05), kMetal);
        cylY(root, "Fluid Nozzle", -0.155, 0.03, 0.03, kDark);
        cylY(root, "Air Cap", -0.175, 0.04, 0.01, kChrome);
        const int cup = m.cyl(root, "Paint Cup", P(0.055, -0.07), Y, 0.045 * k, 0.07 * k, m.accent);
        m.g.getObject(cup)->opacity = 0.85f;
        tcp = P(0, -0.18) + QVector3D(0, -0.2f * float(std::sqrt(k)), 0);
        m.input("Spray", "Spray on");
        break;
    }
    case Tool::LaserHead:
        cylY(root, "Head Body", -0.07, 0.065, 0.14, kDark);
        cylY(root, "Focus Lens", -0.155, 0.05, 0.03, kMetal);
        cylY(root, "Cone", -0.18, 0.036, 0.02, kMetal);
        cylY(root, "Nozzle Tip", -0.197, 0.02, 0.014, kCopper);
        tcp = P(0, -0.207);
        m.input("Laser", "Beam on");
        break;
    case Tool::Camera: {
        m.box(root, "Bracket", P(0, -0.015), P(0.04, 0.03, 0.04), kDark);
        m.box(root, "Camera Body", P(0, -0.07), P(0.06, 0.08, 0.06), QColor("#3c4048"));
        cylY(root, "Lens", -0.125, 0.04, 0.03, kRubber);
        cylY(root, "Ring Light", -0.145, 0.1, 0.012, QColor("#e8f1ff"));
        tcp = P(0, -0.3);
        Signal s;
        s.name = m.g.makeSymbolName("Range");
        s.kind = SignalKind::Sensor;
        s.description = "Distance from the camera to the first surface it looks at (m)";
        s.sensor.type = SensorType::Distance;
        s.sensor.objectId = root;
        s.sensor.offset = P(0, -0.152);
        s.sensor.direction = -Y;
        s.sensor.range = float(0.8 * k);
        s.type = sensorSignalType(s.sensor.type);
        m.g.addSignal(s);
        break;
    }
    }
    const int tcpId = m.frame(root, "TCP", tcp, tcpRot);
    // Presence sensors at the TCP for the grippers that hold by touch
    const double zk = k;
    switch (tool) {
    case Tool::SuctionCup:
        m.zone("AtCup", tcpId, {0, float(-0.005 * zk), 0}, QVector3D(0.06f, 0.04f, 0.06f) * float(zk), "A part touches the cup");
        break;
    case Tool::VacuumPad:
        m.zone("AtPad", tcpId, {0, float(-0.01 * zk), 0}, QVector3D(0.26f, 0.04f, 0.18f) * float(zk), "A part touches the pad");
        break;
    case Tool::MagneticGripper:
        m.zone("AtMagnet", tcpId, {0, float(-0.008 * zk), 0}, QVector3D(0.1f, 0.03f, 0.1f) * float(zk), "A part touches the magnet");
        break;
    case Tool::ParallelGripper:
    case Tool::ThreeFingerGripper:
    case Tool::AngularGripper:
        m.zone("InGripper", tcpId, {}, QVector3D(0.06f, 0.06f, 0.04f) * float(zk), "A part between the fingers");
        break;
    case Tool::ForkGripper:
        m.zone("OnFork", tcpId, {0, float(0.08 * zk), 0}, QVector3D(0.44f, 0.16f, 0.36f) * float(zk), "A load on the tines");
        break;
    default:
        break;
    }
    return tcpId;
}

// Size the end effector is drawn at for this robot (×1 suits a 5 kg arm)
double toolSize(const Spec& s) {
    double k = 1;
    switch (s.kind) {
    case Kind::Scara:       k = std::clamp((s.dim("L1") + s.dim("L2")) / 0.6, 0.5, 2.0); break;
    case Kind::Palletizer:  k = std::clamp(s.dim("a2") / 0.6, 1.0, 3.5); break;
    case Kind::Articulated: k = std::clamp(s.dim("a2") / 0.45, 0.6, 3.0); break;
    }
    return k * s.toolScale;
}

double styleWidth(const Spec& s, double w) {
    w *= s.linkScale;
    if (s.style == Style::HeavyDuty) w *= 1.3;
    if (s.style == Style::Slim) w *= 0.85;
    return w;
}

// Pedestal and the fixed base plinth (on the root, which does not turn)
double buildBase(Maker& m, const Spec& s, int root, double w, double h1, double a1) {
    const double h0 = s.dim("h0");
    const double db = 1.9 * w + 0.6 * a1;
    if (h0 > 0.001) {
        if (s.style == Style::Cobot || s.style == Style::Slim)
            m.cyl(root, "Pedestal", {0, float(h0 / 2), 0}, Y, db * 0.85, h0, m.secondary);
        else
            m.box(root, "Pedestal", {0, float(h0 / 2), 0}, QVector3D(db * 0.9, h0, db * 0.9), m.secondary);
        m.box(root, "Pedestal Plate", {0, float(h0 - 0.01), 0}, QVector3D(db * 1.15, 0.02f, db * 1.15), kMetal);
    }
    const double hb = std::clamp(0.28 * h1, 0.05, std::max(0.05, h1 - 0.6 * w));
    switch (s.style) {
    case Style::Industrial:
        m.box(root, "Base Foot", {0, float(h0 + 0.0125), 0}, QVector3D(db * 1.1, 0.025f, db * 1.1), m.secondary);
        m.cyl(root, "Base", {0, float(h0 + hb / 2), 0}, Y, db, hb, m.secondary);
        break;
    case Style::HeavyDuty:
        m.box(root, "Base Plate", {0, float(h0 + 0.02), 0}, QVector3D(db * 1.25, 0.04f, db * 1.25), m.secondary);
        m.cyl(root, "Base", {0, float(h0 + hb / 2), 0}, Y, db * 1.05, hb, m.secondary);
        break;
    case Style::Cobot:
        m.cyl(root, "Base", {0, float(h0 + hb / 2), 0}, Y, db * 0.8, hb, m.primary);
        m.cyl(root, "Base Ring", {0, float(h0 + hb - 0.01), 0}, Y, db * 0.82, 0.012, m.accent);
        break;
    case Style::Slim:
        m.cyl(root, "Base", {0, float(h0 + hb / 2), 0}, Y, db * 0.75, hb, m.secondary);
        break;
    }
    return hb;
}

// The turning base above the plinth: a drum, and a yoke on the side of the
// lower arm carrying the shoulder axis (on both sides with `twoSided`)
void buildYoke(Maker& m, const Spec& s, int f1, double w, double hb, double h1, double a1, bool twoSided) {
    const double drumTop = std::max(hb + 0.02, h1 - 0.7 * w);
    const double drumDia = std::max(1.5 * w, 2 * a1 + 0.8 * w);
    if (s.style == Style::Slim)
        m.cyl(f1, "Turret", {0, float((hb + drumTop) / 2), 0}, Y, drumDia * 0.6, drumTop - hb, m.primary);
    else
        m.cyl(f1, "Turret", {0, float((hb + drumTop) / 2), 0}, Y, drumDia, drumTop - hb, m.primary);
    const double top = h1 + 0.55 * w;
    const double x0 = -0.4 * w, x1 = a1 + 0.5 * w;
    const QVector3D size(float(x1 - x0), float(top - drumTop), float(0.45 * w));
    const std::vector<int> sides = twoSided ? std::vector<int>{-1, 1} : std::vector<int>{-1};
    for (int side : sides) {
        const QVector3D c(float((x0 + x1) / 2), float((drumTop + top) / 2), float(side * 0.68 * w));
        m.housing(f1, side < 0 ? QString("Shoulder Yoke") : QString("Shoulder Yoke R"), c, size);
    }
    if (!twoSided && s.style != Style::Slim)
        m.cyl(f1, "Shoulder Motor", {float(a1), float(h1), float(0.78 * w)}, Z, 0.6 * w, 0.35 * w, m.secondary);
}

// The arm's actuators, J1 first (before the tool's own)
void armActuators(Maker& m, const Spec& s, Built& b) {
    for (size_t i = 0; i < b.joints.size(); ++i) {
        Joint* j = m.g.getJoint(b.joints[i]);
        const int mid = m.actuator(j->name, j->id, s.axes[i].speed, s.axes[i].accel);
        j->name = m.g.getMotor(mid)->name;
        b.motors.push_back(mid);
    }
}

void buildArticulated(Maker& m, const Spec& s, Built& b) {
    const double h0 = s.dim("h0"), h1 = s.dim("h1"), a1 = s.dim("a1"), a2 = s.dim("a2"), a3 = s.dim("a3"),
                 d4 = s.dim("d4"), d6 = s.dim("d6");
    const double w = styleWidth(s, 0.06 + 0.12 * a2);
    const QString p = s.prefix;
    const int root = b.root;
    const double hb = buildBase(m, s, root, w, h1, a1);

    const int f1 = m.frame(root, p + "1 Turret", {0, float(h0), 0});
    if (s.style == Style::Cobot) {
        m.cyl(f1, "Turret", {0, float((hb + h1) / 2), 0}, Y, 1.2 * w, h1 - hb, m.primary);
        if (a1 > 0.3 * w)
            m.rod(f1, "Shoulder Arm", {0, float(h1), 0}, {float(a1), float(h1), 0}, 0.9 * w, m.primary);
    } else {
        buildYoke(m, s, f1, w, hb, h1, a1, false);
    }

    const int f2 = m.frame(f1, p + "2 Lower Arm", {float(a1), float(h1), 0});
    m.hub(f2, "Shoulder", {}, Z, 1.15 * w, 1.55 * w);
    m.link(f2, "Lower Arm", {}, {0, float(a2), 0}, w, 0.85 * w);
    if (s.style == Style::HeavyDuty)
        m.rod(f2, "Counterbalance", {float(-0.6 * w), float(0.08 * a2), float(0.55 * w)},
              {float(-0.6 * w), float(0.6 * a2), float(0.55 * w)}, 0.45 * w, m.secondary);

    const int f3 = m.frame(f2, p + "3 Elbow", {0, float(a2), 0});
    m.hub(f3, "Elbow", {}, Z, 1.0 * w, 1.35 * w);
    m.housing(f3, "Elbow Housing", {float(-0.1 * w), float(a3 / 2), 0}, QVector3D(float(1.45 * w), float(a3 + 0.95 * w), float(1.05 * w)));
    if (s.style != Style::Slim)
        m.box(f3, "Forearm Motor", {float(-0.85 * w), float(a3 / 2), 0}, QVector3D(float(0.5 * w), float(0.65 * w), float(0.8 * w)), m.secondary);

    const int f4 = m.frame(f3, p + "4 Forearm", {0, float(a3), 0});
    if (d4 - 0.45 * w > 0.55 * w)
        m.link(f4, "Forearm", {float(0.55 * w), 0, 0}, {float(d4 - 0.45 * w), 0, 0}, 0.75 * w, 0.7 * w);
    m.housing(f4, "Wrist Fork", {float(d4 - 0.22 * w), 0, 0}, QVector3D(float(0.6 * w), float(0.6 * w), float(0.85 * w)));

    const int f5 = m.frame(f4, p + "5 Wrist", {float(d4), 0, 0});
    m.hub(f5, "Wrist", {}, Z, 0.6 * w, 0.95 * w);
    if (d6 - 0.025 > 0.2 * w)
        m.rod(f5, "Wrist Body", {float(0.2 * w), 0, 0}, {float(d6 - 0.02), 0, 0}, 0.5 * w,
              s.style == Style::Cobot ? m.primary : m.secondary);

    const int f6 = m.frame(f5, p + "6 Flange", {float(d6), 0, 0});
    m.cyl(f6, "Flange Plate", {-0.01f, 0, 0}, X, 0.5 * w, 0.02, kMetal);

    const std::vector<AxisSpec>& ax = s.axes;
    const QVector3D mz(0, 0, -1);
    b.joints = {
        m.joint(p + "1", root, f1, JointType::Revolute, Y, {0, float(h0), 0}, ax[0].min, ax[0].max, ax[0].home),
        m.joint(p + "2", f1, f2, JointType::Revolute, mz, {float(a1), float(h1), 0}, ax[1].min, ax[1].max, ax[1].home),
        m.joint(p + "3", f2, f3, JointType::Revolute, mz, {0, float(a2), 0}, ax[2].min, ax[2].max, ax[2].home),
        m.joint(p + "4", f3, f4, JointType::Revolute, X, {0, float(a3), 0}, ax[3].min, ax[3].max, ax[3].home),
        m.joint(p + "5", f4, f5, JointType::Revolute, mz, {float(d4), 0, 0}, ax[4].min, ax[4].max, ax[4].home),
        m.joint(p + "6", f5, f6, JointType::Revolute, X, {float(d6), 0, 0}, ax[5].min, ax[5].max, ax[5].home),
    };
    armActuators(m, s, b);
    b.tcp = buildTool(m, s.tool, f6, {0, 0, 90}, toolSize(s), b);   // tool's −Y along the flange's +X
}

void buildScara(Maker& m, const Spec& s, Built& b) {
    const double H = s.dim("H"), L1 = s.dim("L1"), L2 = s.dim("L2"), S = s.dim("S"), Fz = s.dim("Fz");
    const double w = styleWidth(s, 0.07 + 0.11 * (L1 + L2));
    const double t1 = 0.5 * w, t2 = 0.45 * w;
    const QString p = s.prefix;
    const int root = b.root;

    switch (s.style) {
    case Style::Industrial:
    case Style::HeavyDuty: {
        const double f = s.style == Style::HeavyDuty ? 1.2 : 1.0;
        m.box(root, "Base Foot", {float(-0.2 * w), 0.01f, 0}, QVector3D(float(2.1 * w * f), 0.02f, float(1.8 * w * f)), m.secondary);
        m.box(root, "Base", {float(-0.2 * w), float(H / 2), 0}, QVector3D(float(1.5 * w * f), float(H), float(1.3 * w * f)), m.primary);
        m.box(root, "Cable Duct", {float(-1.0 * w * f), float(H * 0.72), 0}, QVector3D(float(0.3 * w), float(0.3 * H), float(0.4 * w)), m.secondary);
        break;
    }
    case Style::Cobot:
        m.cyl(root, "Base Foot", {0, 0.01f, 0}, Y, 1.8 * w, 0.02, m.secondary);
        m.cyl(root, "Base", {0, float(H / 2), 0}, Y, 1.2 * w, H, m.primary);
        m.cyl(root, "Base Ring", {0, float(H - 0.015), 0}, Y, 1.23 * w, 0.012, m.accent);
        break;
    case Style::Slim:
        m.box(root, "Base Foot", {0, 0.01f, 0}, QVector3D(float(1.6 * w), 0.02f, float(1.6 * w)), m.secondary);
        m.cyl(root, "Base", {0, float(H / 2), 0}, Y, 0.75 * w, H, m.primary);
        break;
    }

    const int f1 = m.frame(root, p + "1 Arm 1", {0, float(H), 0});
    m.link(f1, "Arm 1", {0, float(t1 / 2), 0}, {float(L1), float(t1 / 2), 0}, t1, w);
    if (s.style != Style::Slim) {
        m.cyl(f1, "Arm 1 Root", {0, float(t1 / 2), 0}, Y, 1.05 * w, t1 * 1.02, s.style == Style::Cobot ? m.primary : m.secondary);
        m.cyl(f1, "Arm 1 End", {float(L1), float(t1 / 2), 0}, Y, w, t1, m.primary);
    } else {
        m.ball(f1, "J1 Hub", {0, float(t1 / 2), 0}, 0.6 * w, m.secondary);
    }

    const int f2 = m.frame(f1, p + "2 Arm 2", {float(L1), float(t1), 0});
    m.link(f2, "Arm 2", {0, float(t2 / 2), 0}, {float(L2), float(t2 / 2), 0}, t2, 0.85 * w);
    if (s.style != Style::Slim) {
        m.cyl(f2, "Elbow", {0, float(t2 / 2), 0}, Y, 0.9 * w, t2 * 1.02, s.style == Style::Cobot ? m.accent : m.secondary);
        m.cyl(f2, "Arm 2 End", {float(L2), float(t2 / 2), 0}, Y, 0.85 * w, t2, m.primary);
        const double x0 = 0.35 * L2, x1 = L2 + 0.42 * w;
        m.housing(f2, "Z Cover", {float((x0 + x1) / 2), float(t2 + 0.4 * w), 0}, QVector3D(float(x1 - x0), float(0.8 * w), float(0.8 * w)));
    } else {
        m.ball(f2, "J2 Hub", {0, float(t2 / 2), 0}, 0.55 * w, m.secondary);
        m.box(f2, "Z Motor", {float(L2), float(t2 + 0.25 * w), 0}, QVector3D(float(0.45 * w), float(0.5 * w), float(0.45 * w)), m.primary);
    }

    // Quill: J3 lowers it (+ down); the flange sits at Fz above the floor at J3 = 0
    const int f3 = m.frame(f2, p + "3 Quill", {float(L2), 0, 0});
    const double yf = Fz - (H + t1);
    m.rod(f3, "Ball Screw Spline", {0, float(yf), 0}, {0, float(t2 + 0.3 * w + S), 0}, 0.22 * w, kChrome);

    const int f4 = m.frame(f3, p + "4 Tool Flange", {0, float(yf), 0});
    m.cyl(f4, "Flange Plate", {0, 0.01f, 0}, Y, 0.4 * w, 0.02, kMetal);

    const std::vector<AxisSpec>& ax = s.axes;
    b.joints = {
        m.joint(p + "1", root, f1, JointType::Revolute, Y, {0, float(H), 0}, ax[0].min, ax[0].max, ax[0].home),
        m.joint(p + "2", f1, f2, JointType::Revolute, Y, {float(L1), float(t1), 0}, ax[1].min, ax[1].max, ax[1].home),
        m.joint(p + "3", f2, f3, JointType::Prismatic, -Y, {}, ax[2].min, ax[2].max, ax[2].home),
        m.joint(p + "4", f3, f4, JointType::Revolute, Y, {0, float(yf), 0}, ax[3].min, ax[3].max, ax[3].home),
    };
    armActuators(m, s, b);
    b.tcp = buildTool(m, s.tool, f4, {}, toolSize(s), b);
}

void buildPalletizer(Maker& m, const Spec& s, Built& b) {
    const double h0 = s.dim("h0"), h1 = s.dim("h1"), a1 = s.dim("a1"), a2 = s.dim("a2"), a3 = s.dim("a3"),
                 d5 = s.dim("d5");
    const double w = styleWidth(s, 0.08 + 0.12 * a2);
    const QString p = s.prefix;
    const int root = b.root;
    const double hb = buildBase(m, s, root, w, h1, a1);

    const int f1 = m.frame(root, p + "1 Turret", {0, float(h0), 0});
    buildYoke(m, s, f1, w, hb, h1, a1, true);

    const int f2 = m.frame(f1, p + "2 Lower Arm", {float(a1), float(h1), 0});
    m.hub(f2, "Shoulder", {}, Z, 1.2 * w, 1.6 * w);
    m.link(f2, "Lower Arm", {}, {0, float(a2), 0}, w, 0.85 * w);
    m.rod(f2, "Parallel Link", {float(-0.55 * w), 0, float(0.6 * w)}, {float(-0.55 * w), float(a2), float(0.6 * w)},
          0.22 * w, m.secondary);

    // The elbow frame stays level (follows −J2), so J3 is the upper arm's
    // angle to the horizontal, like on a parallel-link palletizer
    const int elbow = m.frame(f2, "Elbow Level", {0, float(a2), 0});
    m.hub(elbow, "Elbow", {}, Z, 1.0 * w, 1.4 * w);

    const int f3 = m.frame(elbow, p + "3 Upper Arm", {});
    m.link(f3, "Upper Arm", {float(-0.22 * a3), 0, 0}, {float(a3), 0, 0}, 0.8 * w, 0.75 * w);
    m.box(f3, "Counterweight", {float(-0.22 * a3), 0, 0}, QVector3D(float(0.6 * w), float(0.95 * w), float(0.95 * w)), m.secondary);

    // The wrist stays level too (follows −J3), so the flange always faces down
    const int wrist = m.frame(f3, "Wrist Level", {float(a3), 0, 0});
    m.hub(wrist, "Wrist", {}, Z, 0.75 * w, 1.0 * w);
    if (d5 > 0.05)
        m.housing(wrist, "Wrist Body", {0, float(-(d5 - 0.02) / 2), 0}, QVector3D(float(0.6 * w), float(d5 - 0.02), float(0.65 * w)));

    const int f4 = m.frame(wrist, p + "4 Tool Flange", {0, float(-d5), 0});
    m.cyl(f4, "Flange Plate", {0, 0.01f, 0}, Y, 0.5 * w, 0.02, kMetal);

    const std::vector<AxisSpec>& ax = s.axes;
    const QVector3D mz(0, 0, -1);
    const int j1 = m.joint(p + "1", root, f1, JointType::Revolute, Y, {0, float(h0), 0}, ax[0].min, ax[0].max, ax[0].home);
    const int j2 = m.joint(p + "2", f1, f2, JointType::Revolute, mz, {float(a1), float(h1), 0}, ax[1].min, ax[1].max, ax[1].home);
    m.follower(p + "2 Level", f2, elbow, mz, {0, float(a2), 0}, j2);
    const int j3 = m.joint(p + "3", elbow, f3, JointType::Revolute, mz, {}, ax[2].min, ax[2].max, ax[2].home);
    m.follower(p + "3 Level", f3, wrist, mz, {float(a3), 0, 0}, j3);
    const int j4 = m.joint(p + "4", wrist, f4, JointType::Revolute, Y, {0, float(-d5), 0}, ax[3].min, ax[3].max, ax[3].home);
    b.joints = {j1, j2, j3, j4};
    armActuators(m, s, b);
    b.tcp = buildTool(m, s.tool, f4, {}, toolSize(s), b);
}

} // namespace

Built build(SceneGraph& scene, Spec spec) {
    validate(spec);
    const ColorScheme& c = colorSchemes()[spec.scheme];
    Maker m{scene, spec.style, c.primary, c.secondary, c.accent};
    Built b;
    b.root = m.frame(-1, spec.name, {});
    switch (spec.kind) {
    case Kind::Articulated: buildArticulated(m, spec, b); break;
    case Kind::Scara:       buildScara(m, spec, b); break;
    case Kind::Palletizer:  buildPalletizer(m, spec, b); break;
    }
    scene.applyCouplings();
    Pose home;
    home.name = spec.name + " Home";
    for (size_t i = 0; i < b.joints.size(); ++i) home.jointValues[b.joints[i]] = float(spec.axes[i].home);
    b.homePose = scene.addPose(home);
    scene.notifyAutomationChanged();
    emit scene.objectChanged(b.root);
    return b;
}

// ── Drawing ──────────────────────────────────────────────────────────────────

std::vector<DimensionLine> dimensionLines(const Spec& in) {
    Spec s = in;
    validate(s);
    std::vector<DimensionLine> d;
    auto add = [&](const QString& sym, QVector3D a, QVector3D b, QVector3D side, double off, bool top = false) {
        d.push_back({sym, a, b, side, off, top});
    };
    const QVector3D left(-1, 0, 0), right(1, 0, 0), up(0, 1, 0), down(0, -1, 0), back(0, 0, -1);
    switch (s.kind) {
    case Kind::Scara: {
        const double H = s.dim("H"), L1 = s.dim("L1"), L2 = s.dim("L2"), S = s.dim("S"), Fz = s.dim("Fz");
        const double r = L1 + L2, sc = std::max(0.3, r);
        add("H", {0, 0, 0}, {0, float(H), 0}, left, 0.18 * sc);
        add("L1", {0, float(H), 0}, {float(L1), float(H), 0}, up, 0.14 * sc);
        add("L2", {float(L1), float(H), 0}, {float(r), float(H), 0}, up, 0.14 * sc);
        add("Fz", {float(r), 0, 0}, {float(r), float(Fz), 0}, right, 0.12 * sc);
        add("S", {float(r), float(Fz - S), 0}, {float(r), float(Fz), 0}, right, 0.24 * sc);
        add("L1", {0, 0, 0}, {float(L1), 0, 0}, back, 0.16 * sc, true);
        add("L2", {float(L1), 0, 0}, {float(r), 0, 0}, back, 0.16 * sc, true);
        break;
    }
    case Kind::Palletizer: {
        const double h0 = s.dim("h0"), h1 = s.dim("h1"), a1 = s.dim("a1"), a2 = s.dim("a2"), a3 = s.dim("a3"), d5 = s.dim("d5");
        const double sc = std::max(0.5, a2 + a3) * 0.5;
        const float ys = float(h0 + h1), ye = float(h0 + h1 + a2);
        if (h0 > 0.001) add("h0", {0, 0, 0}, {0, float(h0), 0}, left, 0.35 * sc);
        add("h1", {0, float(h0), 0}, {float(a1), ys, 0}, left, 0.35 * sc);
        add("a1", {0, ys, 0}, {float(a1), ys, 0}, down, 0.2 * sc);
        add("a2", {float(a1), ys, 0}, {float(a1), ye, 0}, left, 0.2 * sc);
        add("a3", {float(a1), ye, 0}, {float(a1 + a3), ye, 0}, up, 0.2 * sc);
        add("d5", {float(a1 + a3), ye, 0}, {float(a1 + a3), float(ye - d5), 0}, right, 0.2 * sc);
        break;
    }
    case Kind::Articulated: {
        const double h0 = s.dim("h0"), h1 = s.dim("h1"), a1 = s.dim("a1"), a2 = s.dim("a2"), a3 = s.dim("a3"),
                     d4 = s.dim("d4"), d6 = s.dim("d6");
        const double sc = std::max(0.4, a2 + d4) * 0.5;
        const float ys = float(h0 + h1), ye = float(h0 + h1 + a2), yw = float(h0 + h1 + a2 + a3);
        const float xw = float(a1 + d4);
        if (h0 > 0.001) add("h0", {0, 0, 0}, {0, float(h0), 0}, left, 0.35 * sc);
        add("h1", {0, float(h0), 0}, {float(a1), ys, 0}, left, 0.35 * sc);
        if (a1 > 0.001) add("a1", {0, ys, 0}, {float(a1), ys, 0}, down, 0.25 * sc);
        add("a2", {float(a1), ys, 0}, {float(a1), ye, 0}, left, 0.2 * sc);
        if (a3 > 0.001) add("a3", {float(a1), ye, 0}, {float(a1), yw, 0}, left, 0.1 * sc);
        add("d4", {float(a1), yw, 0}, {xw, yw, 0}, up, 0.25 * sc);
        add("d6", {xw, yw, 0}, {xw, float(yw - d6), 0}, right, 0.2 * sc);
        break;
    }
    }
    return d;
}

} // namespace robot
