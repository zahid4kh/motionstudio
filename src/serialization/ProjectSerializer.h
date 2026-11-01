#pragma once
#include <QString>
#include <QJsonObject>
#include <memory>
#include <set>
#include <unordered_map>
#include "../scene/SceneGraph.h"

// Keeps mesh geometry for in-memory snapshots. Imported meshes never change
// after import, so a snapshot only needs to name one: the pool holds the
// geometry and hands the same shared_ptr back on restore. Undo then copies
// no triangle data, and the viewport's GPU buffers and feature caches (keyed
// by MeshGeometry) stay valid across undo/redo.
class MeshPool {
public:
    int add(const std::shared_ptr<MeshGeometry>& mesh);   // stable key per mesh
    std::shared_ptr<MeshGeometry> get(int key) const;
    // Drops every mesh whose key is not in `keys`
    void retainOnly(const std::set<int>& keys);
    void clear();

    // The mesh keys a snapshot made with a pool refers to
    static void collectKeys(const QJsonObject& snapshot, std::set<int>* keys);

private:
    std::unordered_map<const MeshGeometry*, int> m_keys;
    std::unordered_map<int, std::shared_ptr<MeshGeometry>> m_meshes;
    int m_nextKey = 0;
};

class ProjectSerializer {
public:
    static bool save(const SceneGraph& scene, const QString& filePath);
    static bool load(SceneGraph& scene, const QString& filePath);

    // In-memory equivalents (no file I/O) — shared by save/load above and by
    // UndoManager's snapshot-based history. With a pool, meshes are written
    // as references into it instead of as vertex data (snapshots only; such
    // JSON must be read back with the same pool).
    static QJsonObject toJson(const SceneGraph& scene, MeshPool* pool = nullptr);
    static bool        fromJson(SceneGraph& scene, const QJsonObject& root, const MeshPool* pool = nullptr);
};
