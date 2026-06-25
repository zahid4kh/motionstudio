#pragma once
#include "CollisionShape.h"

struct RigidBody {
    int   objectId       = -1;
    float mass           = 1.0f;  // kg
    bool  gravityEnabled = true;
    bool  hasCollision   = false;
    CollisionShape collision;
};
