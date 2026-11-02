#pragma once
#include "Mate.h"
#include <QQuaternion>
#include <QStringList>
#include <vector>

class SceneGraph;

// Assembly mates: placing parts against each other by their geometry (flush
// faces, aligned axes, bolts in holes) instead of typing coordinates.
//
// Each mate moves one body: the part its `moving` feature belongs to, or the
// branch of the tree that part hangs from (see movingBody). The correction is
// the smallest one: the minimal rotation about the moving feature, then a
// translation only along the directions the mate fixes, so a part that is
// already nearly in place barely moves and whatever the mate leaves free
// (sliding along a face, turning about an axis) is kept. Mates are solved
// together (repeated projection), so adding one keeps the earlier ones.
namespace assembly {

// A feature in world space
struct WorldFeature {
    MateFeature::Kind kind = MateFeature::Kind::Point;
    QVector3D point, direction;
};
WorldFeature toWorld(const SceneGraph& scene, const MateFeature& feature);
// A feature picked in world space, stored in the object's own frame
MateFeature fromWorld(const SceneGraph& scene, int objectId, MateFeature::Kind kind,
                      const QVector3D& worldPoint, const QVector3D& worldDirection, float radius = 0.0f);

// The mate types that apply to two features; the usual one first
std::vector<MateType> typesFor(MateFeature::Kind moving, MateFeature::Kind fixed);
// What the type means for these two features ("Flush", "Point on face", …)
QString typeLabel(MateType type, MateFeature::Kind moving, MateFeature::Kind fixed);
// Whether the offset applies (Flush gap, Insert depth)
bool usesOffset(MateType type, MateFeature::Kind moving, MateFeature::Kind fixed);
// Whether "aligned" (flip) applies: both features have a direction and the
// type keeps them parallel
bool usesAlignment(MateType type, MateFeature::Kind moving, MateFeature::Kind fixed);

// The object the mate moves: the moving feature's object, or its ancestor
// right below the lowest ancestor it shares with the fixed feature's object
// (so a part inside an imported assembly moves with its subassembly, and
// two parts of one assembly move relative to each other). -1 when the mate
// can't move anything (same part, or the fixed part is inside the moving one);
// `why` then says so.
int movingBody(const SceneGraph& scene, const Mate& mate, QString* why = nullptr);

// The starting `aligned` setting: opposed for two faces or round edges
// (their directions point out of the part, so they go against each other),
// otherwise whichever needs the least rotation from the current pose
bool defaultAlignment(const SceneGraph& scene, const Mate& mate);

// How far the mate is from being satisfied
struct Residual {
    float distance = 0.0f;   // m
    float angle = 0.0f;      // rad
    bool  valid = true;      // false: the features can't take this type, or nothing can move
    bool  satisfied() const { return valid && distance < 1e-5f && angle < 1e-4f; }
};
Residual residual(const SceneGraph& scene, const Mate& mate);

struct SolveResult {
    std::vector<int> moved;          // bodies whose transforms changed
    std::vector<int> unsatisfied;    // enabled mates still off (conflicting, or invalid)
    QStringList      problems;       // "Mate3: …" for the unsatisfied ones
    int              iterations = 0;
    bool ok() const { return unsatisfied.empty(); }
};
// Moves bodies until every enabled mate holds (or `maxIterations` sweeps).
// Emits objectChanged for each moved body.
SolveResult solve(SceneGraph& scene, int maxIterations = 200);

// "Flush: Plate → Base"
QString describe(const SceneGraph& scene, const Mate& mate);
// The next free "Mate<n>" name
QString nextName(const SceneGraph& scene);

} // namespace assembly
