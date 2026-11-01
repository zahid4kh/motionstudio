#pragma once
#include <QString>
#include <QStringList>
#include <memory>
#include "../scene/SceneObject.h"

class SceneGraph;

// Exchange with ROS and MuJoCo: URDF and MJCF import and export.
//
// MotionStudio is Y-up; URDF and MJCF are Z-up. Imports are wrapped in a
// root frame turned −90° about X, exports put the scene under a base frame
// turned +90° about X, so models stand upright in both directions.
namespace robotio {

struct Result {
    bool        ok = false;
    QString     error;
    QStringList warnings;   // things that were approximated or skipped
    int         rootId = -1;   // import: the new root object
    int         files = 0;     // export: mesh files written
};

// Imports add to the scene (the current scene is kept). Meshes may be STL
// (binary or ASCII) or OBJ; package:// paths are looked up in the folders
// above the file. Movable joints get actuators; mimic joints and joint
// equalities become couplings.
Result importUrdf(SceneGraph& scene, const QString& path);
Result importMjcf(SceneGraph& scene, const QString& path);

// Exports write the file plus a "meshes" folder of binary STL files next to
// it. Joint values are taken as 0 (the definition pose). MJCF also gets
// position/velocity actuators for actuated joints, couplings as joint
// equalities, free joints for rigid bodies, and a floor.
Result exportUrdf(SceneGraph& scene, const QString& path);
Result exportMjcf(SceneGraph& scene, const QString& path);

// Triangle meshes as pos+normal soups (flat normals), in the file's units
std::shared_ptr<MeshGeometry> loadMesh(const QString& path, QString* error = nullptr);
bool writeStl(const std::vector<float>& soup, const QString& path);

} // namespace robotio
