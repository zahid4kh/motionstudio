#pragma once
#include <QString>
#include "scene/SceneObject.h"

// Lucide icon (see Theme::icon) for a scene object's primitive type.
// Imported CAD: an assembly node has no mesh, a part has one.
inline QString primitiveIconName(PrimitiveType type, bool hasMesh) {
    switch (type) {
        case PrimitiveType::Cube:         return "box";
        case PrimitiveType::Cylinder:     return "cylinder";
        case PrimitiveType::Sphere:       return "globe";
        case PrimitiveType::Capsule:      return "pill";
        case PrimitiveType::Basket:       return "shopping-basket";
        case PrimitiveType::ImportedMesh: return hasMesh ? "component" : "boxes";
    }
    return "box";
}
