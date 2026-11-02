#pragma once
#include <QString>
#include <QVector3D>

// A piece of geometry on a part that a mate refers to, in the part's own
// (mesh) frame so it follows the part wherever it moves.
struct MateFeature {
    enum class Kind {
        Plane,    // a flat face: a point on it and its outward normal
        Axis,     // a line: a cylinder's centre line or a straight edge
        Circle,   // a round edge: its centre and the axis through it
        Point     // a corner or any point on a surface
    };
    int       objectId = -1;
    Kind      kind = Kind::Plane;
    QVector3D point;
    // Unit; unused for Point. Planes and circles point out of the part (out
    // of a face, out of the end a rim is on); an axis's sign is arbitrary.
    QVector3D direction;
    float     radius = 0.0f;   // circles and cylinders as picked (world, m); for display

    bool hasDirection() const { return kind != Kind::Point; }
};

// What a mate holds between its two features
enum class MateType {
    Flush,          // planes (or a circle on a plane) coincide, faces against each other or level
    Concentric,     // axes on one line
    Insert,         // round edges: on one axis with their centres together (bolt in a hole)
    Coincident,     // points together
    Parallel,       // directions parallel, no position change
    Perpendicular   // directions at right angles, no position change
};

// An assembly constraint: the part owning `moving` (or the branch of the
// tree it hangs from, see assembly::movingBody) is moved by the smallest
// rotation and translation that satisfies the mate; `fixed` stays put.
struct Mate {
    int         id = -1;
    QString     name;
    MateType    type = MateType::Flush;
    MateFeature moving, fixed;
    // Directions pointing the same way (level faces, same-way axes) instead
    // of opposite ways (faces against each other)
    bool        aligned = false;
    // Flush: gap along the fixed normal; Insert: along the axis (m)
    float       offset = 0.0f;
    bool        enabled = true;
};

const char* mateTypeName(MateType t);
MateType    mateTypeFromName(const QString& name);
const char* mateFeatureKindName(MateFeature::Kind k);
MateFeature::Kind mateFeatureKindFromName(const QString& name);
