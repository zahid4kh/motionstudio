#pragma once
#include <QJsonObject>
#include <QMap>
#include <QString>
#include <QStringList>
#include <map>
#include <vector>
#include "robot/RobotBuilder.h"

class SceneGraph;

// Builds a humanoid robot into a scene from body dimensions and a choice of
// joints: frames with exact pivots, an actuator per chosen joint (left and
// right mirrored, so equal values give a symmetric posture), hands, a set of
// poses and a looping demo sequence. Metres and degrees; the robot faces +X,
// its left side is −Z, it stands on the floor at its root's origin and its
// pelvis stays where it is built (kinematic, it does not balance).
namespace humanoid {

enum class Base  { Legs, Pedestal };              // stands on its legs / upper body on a column
enum class Hand  { Mitten, Gripper, Fingers };    // no joints / parallel jaws / thumb and four fingers
enum class Style { Android, Mechanical, Slim };

// One joint's settings; sided joints share them (mirrored)
struct JointSpec {
    bool   enabled = true;
    double min = -90, max = 90;
    double home = 0;
    double speed = 120;     // deg/s
    double accel = 480;     // deg/s²
};

struct JointDef {
    QString key;          // "ShoulderPitch"; sided joints are named L<key> and R<key>
    QString label;        // "Shoulder pitch"
    QString group;        // Waist, Neck, Arms, Legs
    bool    sided = false;
    QString positive;     // what a positive value does
    JointSpec defaults;
};

struct Spec {
    QString name  = "Humanoid";
    Base    base  = Base::Legs;
    QMap<QString, double>    dims;     // key → metres (see dimensionDefs)
    QMap<QString, JointSpec> joints;   // key → settings (see jointDefs)
    Hand    hand   = Hand::Fingers;
    Style   style  = Style::Android;
    int     scheme = 0;                // index into robot::colorSchemes()
    double  bulk   = 1.0;              // limb thickness × this
    bool    demo   = true;             // poses beyond Home and a looping demo sequence

    double dim(const QString& key) const;
    JointSpec joint(const QString& key) const;
};

struct Preset {
    QString name;
    QString description;
    Spec    spec;
};

// ── Catalogue ────────────────────────────────────────────────────────────────
std::vector<robot::DimensionDef> dimensionDefs(Base base);   // keys are the drawing's symbols
const std::vector<JointDef>&     jointDefs();
const JointDef*                  jointDef(const QString& key);
QStringList baseNames();
QStringList handNames();
QStringList styleNames();
const std::vector<Preset>& presets();

Spec defaultSpec();
// Clamps every value into range; returns what makes the robot impractical
// (empty = fine). Building always uses the clamped spec.
QStringList validate(Spec& spec);

double height(const Spec& spec);      // floor to the top of the head at Home (all joints 0)
int    jointCount(const Spec& spec);  // actuated body joints, both sides counted
// Scales every length so the robot is `metres` tall
void   scaleToHeight(Spec& spec, double metres);

QJsonObject toJson(const Spec& spec);
Spec        fromJson(const QJsonObject& o);

// ── Building ─────────────────────────────────────────────────────────────────
struct Built {
    int root = -1;
    std::map<QString, int> joints;       // actuator name as built (LShoulderPitch …) → joint id
    std::vector<int> motors;             // body joints first, then the hands'
    int leftHand = -1, rightHand = -1;   // TCP frames in the palms (−Y along the fingers)
    int leftGripper = -1, rightGripper = -1;   // hand frames (Grip operations)
    int leftFoot = -1, rightFoot = -1;   // foot frames
    int head = -1;
    std::vector<int> poses;              // Home first
    int sequence = -1;                   // demo sequence (-1 without)
};

// Builds `spec` (validated first) into `scene` as a new top-level object.
// Actuators and the sequence get free names (LElbow → LElbow_2 when taken).
Built build(SceneGraph& scene, Spec spec);

// Dimension lines for the drawing in the robot's frame with every joint at 0
std::vector<robot::DimensionLine> dimensionLines(const Spec& spec);

} // namespace humanoid
