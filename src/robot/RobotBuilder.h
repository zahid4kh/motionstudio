#pragma once
#include <QColor>
#include <QJsonObject>
#include <QMap>
#include <QString>
#include <QStringList>
#include <QVector3D>
#include <vector>

class SceneGraph;

// Builds a working 4- or 6-axis robot into a scene from a few dimensions:
// frames with exact pivots and axes, geometry in a chosen style, actuators,
// an end effector (with its own actuators, signals and sensors) and a Home
// pose. Everything is in metres and degrees; the robot stands on the floor at
// its root's origin and faces +X.
namespace robot {

enum class Kind {
    Scara,        // 4 axes: J1, J2 turn about vertical axes, J3 lifts (Z), J4 turns the tool
    Palletizer,   // 4 axes: J1 base, J2 shoulder, J3 arm; the wrist stays level (parallel links), J4 turns the tool
    Articulated,  // 6 axes: base, shoulder, elbow, then a spherical wrist (A4, A5, A6)
};

enum class Style { Industrial, Cobot, Slim, HeavyDuty };

enum class Tool {
    None, SuctionCup, VacuumPad, MagneticGripper, ParallelGripper, ThreeFingerGripper,
    AngularGripper, ForkGripper, Drill, Screwdriver, DeburringSpindle, WeldingTorch,
    Dispenser, SprayGun, LaserHead, Camera,
};

// One length the user enters (shown on the dimensioned drawing by `symbol`)
struct DimensionDef {
    QString key;           // stored in Spec::dims
    QString symbol;        // H, L1, a2 …
    QString label;
    QString description;   // between which two points it is measured
    double  min, max, value;   // m (value = default)
};

// One axis's range and speed. For a prismatic axis the range is in metres
// (a SCARA's J3 range follows its stroke).
struct AxisSpec {
    double min = -180, max = 180;
    double speed = 180;       // deg/s or m/s
    double accel = 720;       // deg/s² or m/s²
    double home = 0;          // value the robot is built at (and its Home pose)
};

struct AxisDef {
    QString name;        // J1 … / A1 …
    QString role;        // "Base rotation" …
    bool    prismatic = false;
    AxisSpec defaults;
};

struct ColorScheme { QString name; QColor primary, secondary, accent; };

struct ToolDef {
    Tool    tool;
    QString name;
    QString description;   // what it adds (actuators, signals, sensor)
    QString icon;          // Lucide icon name
};

struct Spec {
    Kind    kind   = Kind::Articulated;
    QString name   = "Robot";
    QString prefix = "A";            // actuators are <prefix>1 … <prefix>N
    QMap<QString, double> dims;      // key → metres (see dimensionDefs)
    std::vector<AxisSpec> axes;      // one per axis (see axisDefs)
    Style   style      = Style::Industrial;
    int     scheme     = 0;          // index into colorSchemes()
    double  linkScale  = 1.0;        // link thickness × this
    Tool    tool       = Tool::ParallelGripper;
    double  toolScale  = 1.0;        // tool size × this (on top of the size that suits the robot)

    double dim(const QString& key) const;
};

struct Preset {
    QString name;
    QString description;
    Spec    spec;
};

// ── Catalogue ────────────────────────────────────────────────────────────────
QString                     kindName(Kind k);
int                         axisCount(Kind k);
std::vector<DimensionDef>   dimensionDefs(Kind k);
std::vector<AxisDef>        axisDefs(Kind k);
QStringList                 styleNames();
const std::vector<ColorScheme>& colorSchemes();
const std::vector<ToolDef>& toolDefs();
const ToolDef&              toolDef(Tool t);
const std::vector<Preset>&  presets();

// A spec of `kind` with every default
Spec defaultSpec(Kind k);
// Clamps every value into range and keeps the axes' count and the SCARA
// stroke consistent; returns the problems that make the robot impractical
// (empty = fine). Building always uses the clamped spec.
QStringList validate(Spec& spec);

QJsonObject toJson(const Spec& spec);
Spec        fromJson(const QJsonObject& o);

// ── Building ─────────────────────────────────────────────────────────────────
struct Built {
    int root = -1;               // the robot's top object
    int tcp  = -1;               // tool centre point frame (the IK tool)
    int toolRoot = -1;           // end effector's top object (the gripper for Grip operations)
    std::vector<int> joints;     // the arm's axes, J1 first
    std::vector<int> motors;     // their actuators
    int homePose = -1;
};

// Builds `spec` (validated first) into `scene` as a new top-level object.
// Actuators, signals and sensors get free names (Grip → Grip_2 when taken).
Built build(SceneGraph& scene, Spec spec);

// Dimension lines for the drawing, in the robot's own frame at its home pose
// (side view = XY plane, top view = XZ plane, front view = ZY plane)
struct DimensionLine {
    QString   symbol;
    QVector3D from, to;
    QVector3D offsetDir;    // where the dimension line sits, away from the robot
    double    offset = 0.08;
    bool      top = false;  // drawn in the top view
    bool      front = false;  // drawn in the front view
};
std::vector<DimensionLine> dimensionLines(const Spec& spec);

} // namespace robot
