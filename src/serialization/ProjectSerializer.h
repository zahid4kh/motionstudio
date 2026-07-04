#pragma once
#include <QString>
#include <QJsonObject>
#include "../scene/SceneGraph.h"

class ProjectSerializer {
public:
    static bool save(const SceneGraph& scene, const QString& filePath);
    static bool load(SceneGraph& scene, const QString& filePath);

    // In-memory equivalents (no file I/O) — shared by save/load above and by
    // UndoManager's snapshot-based history.
    static QJsonObject toJson(const SceneGraph& scene);
    static bool        fromJson(SceneGraph& scene, const QJsonObject& root);
};
