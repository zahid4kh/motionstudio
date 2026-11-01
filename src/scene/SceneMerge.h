#pragma once
#include <vector>

class SceneGraph;

// Copies everything in `source` into `target` (inserting a component from
// the library): objects (under `parentId`, -1 for the root), joints and
// couplings, actuators, rigid bodies, conveyor belts, signals and sensors,
// poses and sequences, with every reference remapped. Signals, actuators and
// sequences whose names are already used in `target` are renamed (J1 → J1_2)
// and the component's expressions are rewritten to match; `source` is
// modified by those renames. Modbus addresses are not copied. Returns the
// ids of the copied root objects.
std::vector<int> mergeScene(SceneGraph& target, SceneGraph& source, int parentId = -1);
