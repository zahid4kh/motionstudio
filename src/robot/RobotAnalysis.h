#pragma once
#include <QString>
#include <QStringList>
#include <QVector3D>
#include <cstdint>
#include <vector>

class SceneGraph;

// Measures a robot in the scene through its own joint tree: the actuated
// joints above a tool (TCP) object, whatever built them.
namespace robot {

struct AxisInfo {
    int     jointId = -1;
    QString name;              // actuator's name
    bool    prismatic = false;
    bool    continuous = false;
    double  min = 0, max = 0;  // deg or m
    double  value = 0;         // now
    double  speed = 0, accel = 0;
    double  moveTime = 0;      // s for a 90° (or 100 mm) move with the actuator's speed and acceleration
    QVector3D point, axis;     // the axis line in the world, now
    // Denavit–Hartenberg parameters from the axis lines (standard convention):
    // a = common normal to the next axis (to the TCP for the last), alpha =
    // twist to the next axis, d = offset along this axis between the normals
    double  a = 0, alpha = 0, d = 0;   // m, deg, m
};

// A grid of cells the TCP reached (side: r × height, top: x × z)
struct EnvelopeGrid {
    int    w = 0, h = 0;
    double x0 = 0, y0 = 0, cell = 0.01;
    std::vector<uint8_t> hit;
    bool at(int i, int j) const { return i >= 0 && j >= 0 && i < w && j < h && hit[size_t(j) * w + i]; }
};

struct Analysis {
    QString error;               // why it could not be analysed; empty = fine
    int     toolId = -1;
    std::vector<AxisInfo> axes;
    QVector3D basePoint, baseAxis = {0, 1, 0};   // J1's axis (or world up)
    bool    verticalBase = false;

    // Workspace from sampling every axis within its range
    int     samples = 0;
    double  maxReach = 0, minReach = 0;          // m, TCP distance from the base axis
    double  maxHeight = 0, minHeight = 0;        // m, TCP height above the floor
    double  volume = 0;                          // m³
    double  sweep = 360;                         // deg the base axis turns
    EnvelopeGrid side, top;                      // side: r ≥ 0 vs height; top: world x vs z

    // At the current pose
    QVector3D tcp;
    double  reachNow = 0;
    double  dexterity = 0;       // smallest / largest singular value of the Jacobian (0 = singular)
    double  manipulability = 0;
    QStringList warnings;        // singularities and axes at their limits
};

// The tool to analyse for `objectId`: an object named TCP in its subtree
// (or its ancestors' subtrees, nearest first) with the longest chain of
// actuated joints, else the deepest object with the longest chain; -1 if none
int findTool(const SceneGraph& scene, int objectId);

// Samples the workspace and evaluates the current pose; the scene's joints
// are left as they were
Analysis analyze(SceneGraph& scene, int toolId, int samples = 24000);
// Re-evaluates only the current-pose part (cheap: for jogging)
void analyzePose(SceneGraph& scene, Analysis& a);

// Time for a move of `distance` with a trapezoidal profile
double moveTime(double distance, double speed, double accel);

} // namespace robot
