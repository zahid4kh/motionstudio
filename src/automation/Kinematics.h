#pragma once
#include <QMatrix4x4>
#include <QQuaternion>
#include <QString>
#include <QVector3D>
#include <map>
#include <vector>

class SceneGraph;

// Inverse kinematics on the scene's own joint tree: a tool (any object, its
// frame being the tool centre point) and the joints between it and the root.
// Generic damped least squares with a numerical Jacobian, so any mix of
// revolute and prismatic joints works.
namespace kin {

// Joints above `toolId`, root first. With `actuatedOnly`, only joints that
// have an actuator (the ones automation can move).
std::vector<int> chainTo(const SceneGraph& scene, int toolId, bool actuatedOnly = true);

// Tool frame in the world: the object's world transform without scale
QMatrix4x4 toolFrame(const SceneGraph& scene, int toolId);
// Tool frame with the given joint values (others as they are now); the
// scene is left unchanged
QMatrix4x4 toolFrameAt(SceneGraph& scene, int toolId, const std::map<int, float>& jointValues);

struct Target {
    QVector3D   position;
    QQuaternion orientation;
    bool        useOrientation = true;
};
Target targetFrom(const QMatrix4x4& frame);

struct Result {
    bool  reached = false;
    float positionError = 0;      // m
    float angleError    = 0;      // degrees (0 when orientation is not used)
    int   iterations    = 0;
};

// Moves the chain's joints (Joint::currentValue, within limits) so the tool
// frame reaches `target`, starting from their current values. The joints are
// left at the best solution found, even when it does not reach.
Result solve(SceneGraph& scene, int toolId, const std::vector<int>& chain, const Target& target,
             int maxIterations = 100);

// Whether `chain` can hold an orientation (6 or more joints)
inline bool canOrient(const std::vector<int>& chain) { return chain.size() >= 6; }

} // namespace kin
