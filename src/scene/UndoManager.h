#pragma once
#include <QObject>
#include <QJsonObject>
#include <QTimer>
#include <vector>
#include "../serialization/ProjectSerializer.h"

class SceneGraph;

// In-memory undo/redo history for the whole SceneGraph.
//
// Rather than modeling every mutation as its own Command subclass (add
// object, edit transform, add joint, combine into rigid body, reparent...),
// this takes full-scene JSON snapshots (reusing ProjectSerializer's
// in-memory toJson/fromJson) and swaps the whole scene back in on undo/redo.
// A single desktop-sized scene graph serializes to JSON in well under a
// millisecond, so the cost of "just snapshot everything" is negligible next
// to the correctness win: every mutation type is covered automatically,
// with no risk of a hand-written Command forgetting to touch some field.
// Mesh geometry is not copied: snapshots reference it through a MeshPool, so
// a checkpoint costs the same on a large imported assembly as on a cube.
//
// Rapid-fire signals from one user gesture (dragging the translate gizmo
// emits objectChanged on every mouse-move) are coalesced by a short idle
// timer so one drag becomes one undo step instead of one per frame.
class UndoManager : public QObject {
    Q_OBJECT
public:
    explicit UndoManager(SceneGraph* scene, QObject* parent = nullptr);

    bool canUndo() const { return !m_undoStack.empty(); }
    bool canRedo() const { return !m_redoStack.empty(); }

public slots:
    void undo();
    void redo();

    // Drops all history and re-baselines on the scene's current state. Call
    // this after New/Open/Template — those replace the whole scene, so
    // "undo" shouldn't reach back into the project that was just closed.
    void reset();

signals:
    void historyChanged();

private slots:
    void onSceneMutated();
    void commitCheckpoint();

private:
    QJsonObject snapshot();
    void        restore(const QJsonObject& snapshot);
    void        pruneMeshPool();

    SceneGraph*          m_scene;
    MeshPool             m_meshPool;
    QTimer               m_coalesceTimer;
    QJsonObject          m_baseline;
    std::vector<QJsonObject> m_undoStack;
    std::vector<QJsonObject> m_redoStack;
    bool                 m_restoring = false;

    static constexpr int kCoalesceMs  = 500;
    static constexpr size_t kMaxDepth = 100;
};
