#pragma once
#include <QString>
#include <QVector3D>
#include <vector>

enum class PrimitiveType { Cube, Cylinder, Sphere, Capsule };

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
};
