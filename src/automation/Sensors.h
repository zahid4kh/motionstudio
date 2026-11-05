#pragma once
#include <QMatrix4x4>
#include <QString>
#include "Automation.h"
#include "../scene/SceneQuery.h"

class SceneGraph;

// Evaluation of sensor signals on the scene's current pose. Used every scan
// by AutomationEngine, and by the viewport to draw sensors live at edit time.
namespace sensors {

struct Reading {
    bool  active   = false;
    // What was detected, as something a gripper can hold: the rigid-body
    // group for SensorTarget::RigidBodies, the target for Object, else the
    // part that was hit
    int   objectId = -1;
    float distance = -1;     // Beam / Distance: metres to the hit (range when clear)
    // The signal's value: the distance for a Distance sensor, else 1 / 0
    double value(const SensorConfig& cfg) const {
        return cfg.type == SensorType::Distance ? double(distance) : (active ? 1.0 : 0.0);
    }
};

// Position sensor fields, in symbol order (<name>.x …)
constexpr int kPositionFieldCount = 6;
extern const char* const kPositionFields[kPositionFieldCount];
// Where `objectId`'s frame is in the sensor's frame: x, y, z (m), then
// rx, ry, rz (degrees, Euler XYZ like Transform::rotation)
void partPose(const SceneGraph& scene, const SensorConfig& cfg, int objectId, double out[kPositionFieldCount]);

// World frame of the sensor (mount frame without scale, moved to `offset`)
QMatrix4x4 frame(const SceneGraph& scene, const SensorConfig& cfg);
// Beam / Distance: world start point and unit direction
void beam(const SceneGraph& scene, const SensorConfig& cfg, QVector3D* from, QVector3D* dir);
// Zone / Position: the box that is tested
scenequery::Obb zone(const SceneGraph& scene, const SensorConfig& cfg);

Reading read(const SceneGraph& scene, const SensorConfig& cfg);

// Empty when the configuration is usable, else what is wrong with it
QString problem(const SceneGraph& scene, const SensorConfig& cfg);
// One-line description for tables ("Beam on Conveyor, 0.50 m → rigid bodies")
QString summary(const SceneGraph& scene, const SensorConfig& cfg);

} // namespace sensors
