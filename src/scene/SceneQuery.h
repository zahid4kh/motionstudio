#pragma once
#include <QMatrix4x4>
#include <QVector3D>
#include <functional>
#include <limits>
#include <vector>

class SceneGraph;

// Geometric queries on the scene's current pose, shared by viewport picking,
// sensors and the interference check. Everything is in world space (metres).
namespace scenequery {

// Whether a ray from `o` along `d` meets the box (an origin inside counts);
// `tEnter` gets the ray parameter where it enters (0 when inside)
bool rayMeetsBox(const QVector3D& o, const QVector3D& d, const QVector3D& mn, const QVector3D& mx,
                 float* tEnter = nullptr);

// Nearest hit (0 < t < maxT, either face side) of a ray on a pos+normal
// triangle soup; Möller–Trumbore
bool rayHitsSoup(const std::vector<float>& soup, const QVector3D& o, const QVector3D& d,
                 float maxT, float* tOut);

struct RayHit {
    int   objectId = -1;
    float t        = std::numeric_limits<float>::max();
    bool  hit() const { return objectId != -1; }
};

// The object whose drawn surface the ray meets first, among those `accept`
// lets through (all objects with geometry when null). Bounding boxes only
// prefilter: the triangles decide, so overlapping boxes don't hide what is
// actually in front. `t` is in units of `dir`.
RayHit rayCast(const SceneGraph& scene, const QVector3D& orig, const QVector3D& dir,
               float maxT = std::numeric_limits<float>::max(),
               const std::function<bool(int)>& accept = {});

// Oriented box: centre, unit axes, half extents along each axis
struct Obb {
    QVector3D center;
    QVector3D axis[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    QVector3D half;
};

// An object's local bounding box placed in the world (scale applied)
Obb worldBox(const SceneGraph& scene, int objectId);
// A box of half extents `half` centred on `frame`'s origin (frame unscaled)
Obb boxInFrame(const QMatrix4x4& frame, const QVector3D& half);
// Separating-axis test
bool overlap(const Obb& a, const Obb& b);

// `m` with each basis column normalised (removes scale, keeps rotation)
QMatrix4x4 stripScale(const QMatrix4x4& m);

// True when `id` is `root` or one of its descendants
bool inSubtree(const SceneGraph& scene, int id, int root);

} // namespace scenequery
