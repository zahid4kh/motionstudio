#pragma once
#include <QString>
#include <QVector3D>
#include <memory>
#include <vector>

enum class PrimitiveType { Cube, Cylinder, Sphere, Capsule, Basket, ImportedMesh };

// Triangle mesh for imported CAD parts (STEP). Interleaved layout matches the
// object shader: pos.xyz + normal.xyz, 6 floats per vertex, GL_TRIANGLES.
// Shared between SceneObjects so assembly instances of the same part reuse one mesh.
struct MeshGeometry {
    std::vector<float> interleaved;
    QVector3D aabbMin{-0.5f, -0.5f, -0.5f};
    QVector3D aabbMax{ 0.5f,  0.5f,  0.5f};
    int vertexCount() const { return int(interleaved.size()) / 6; }
};

struct Transform {
    QVector3D position{0.0f, 0.0f, 0.0f};
    QVector3D rotation{0.0f, 0.0f, 0.0f}; // Euler angles in degrees (XYZ order)
    QVector3D scale{1.0f, 1.0f, 1.0f};
};

struct SceneObject {
    int         id       = -1;
    QString     name;
    PrimitiveType type   = PrimitiveType::Cube;
    Transform   transform;
    int         parentId = -1;
    std::vector<int> childIds;
    bool        selected = false;

    // Only set for ImportedMesh; null for assembly group nodes (tree-only, no geometry)
    std::shared_ptr<MeshGeometry> mesh;

    // Local-space AABB used for picking/overlays (unit ±0.5 for primitives)
    QVector3D aabbMin() const { return mesh ? mesh->aabbMin : QVector3D(-0.5f, -0.5f, -0.5f); }
    QVector3D aabbMax() const { return mesh ? mesh->aabbMax : QVector3D( 0.5f,  0.5f,  0.5f); }
};
