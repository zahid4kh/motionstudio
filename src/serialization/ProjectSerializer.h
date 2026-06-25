#pragma once
#include <QString>
#include "../scene/SceneGraph.h"

class ProjectSerializer {
public:
    static bool save(const SceneGraph& scene, const QString& filePath);
    static bool load(SceneGraph& scene, const QString& filePath);
};
