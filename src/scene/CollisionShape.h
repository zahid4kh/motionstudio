#pragma once
#include <QVector3D>

enum class CollisionType { Box, Cylinder, Sphere, Capsule };

struct CollisionShape {
    CollisionType type   = CollisionType::Box;
    // Box half-extents (m)
    float halfW  = 0.5f;
    float halfH  = 0.5f;
    float halfD  = 0.5f;
    // Cylinder / Capsule / Sphere
    float radius = 0.5f;
    float height = 1.0f;
    // Offset from object origin (m)
    QVector3D offset{0.0f, 0.0f, 0.0f};
};
