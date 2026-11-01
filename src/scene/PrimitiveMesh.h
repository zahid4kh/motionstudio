#pragma once
#include <vector>
#include "SceneObject.h"

// Triangle soups (pos.xyz + normal.xyz per vertex) of the built-in
// primitives in their unit local space, as drawn by the viewport. Built once
// on first use. ImportedMesh returns an empty soup (its geometry is per object).
const std::vector<float>& primitiveSoup(PrimitiveType type);

// The triangles an object is drawn with: its mesh, the primitive's soup, or
// null for a group node / frame without geometry
const std::vector<float>* surfaceOf(const SceneObject& obj);
