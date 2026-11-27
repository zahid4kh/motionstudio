#pragma once
#include <QVector3D>
#include <vector>

// Geometry a joint axis can be taken from: a line in space (origin + axis)
// recognised on a triangle mesh. Everything is in the mesh's local frame.
struct AxisFeature {
    enum class Kind { None, CircularEdge, StraightEdge, CylindricalFace, FlatFace };
    Kind      kind = Kind::None;
    QVector3D origin;                // circle / cylinder centre, edge midpoint, face centroid
    QVector3D axis;                  // unit; its sign is arbitrary
    float     radius = 0.0f;         // circles and cylinders
    std::vector<QVector3D> outline;  // line segments (pairs of points) to highlight
};

// Recovers the faces and edges of a CAD part from its tessellation. STEP
// meshes arrive as triangle soup, so vertices are welded, creases sharper
// than ~40° become edges, and the smooth patches between them become faces.
// Faces are classified as flat or cylindrical and edge chains as straight
// or circular, the shapes a revolute or prismatic axis is defined by.
class MeshFeatures {
public:
    // pos.xyz + normal.xyz, 6 floats per vertex, GL_TRIANGLES
    explicit MeshFeatures(const std::vector<float>& interleaved);

    int triangleCount() const { return int(m_tris.size()); }
    // Nearest triangle hit by a local-space ray (either side), or -1
    int intersect(const QVector3D& origin, const QVector3D& dir, float* t) const;

    // The classified face the triangle belongs to (kind None if it is neither
    // flat nor cylindrical)
    const AxisFeature& faceFeature(int triangle) const;
    // The straight and circular edges bounding the triangle's face
    std::vector<const AxisFeature*> edgesAround(int triangle) const;
    // Every face and edge chain found (faces of kind None included)
    const std::vector<AxisFeature>& faces() const { return m_faces; }
    const std::vector<AxisFeature>& edges() const { return m_edges; }

private:
    struct Triangle { int v[3]; QVector3D normal; float area; };

    void buildRegions(const std::vector<std::vector<int>>& edgeTris, const std::vector<std::pair<int, int>>& edges,
                      const std::vector<bool>& crease);
    void buildChains(const std::vector<std::pair<int, int>>& edges, const std::vector<std::vector<int>>& edgeTris,
                     const std::vector<bool>& crease);
    AxisFeature classifyRegion(const std::vector<int>& tris) const;
    AxisFeature classifyChain(const std::vector<int>& points, bool closed) const;

    std::vector<QVector3D> m_points;       // welded vertices
    std::vector<Triangle>  m_tris;
    std::vector<int>       m_regionOf;     // triangle → face
    std::vector<AxisFeature> m_faces;
    std::vector<AxisFeature> m_edges;
    std::vector<std::vector<int>> m_faceEdges;   // face → indices into m_edges
};
