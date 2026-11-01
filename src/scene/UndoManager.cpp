#include "UndoManager.h"
#include "SceneGraph.h"

UndoManager::UndoManager(SceneGraph* scene, QObject* parent)
    : QObject(parent), m_scene(scene)
{
    m_coalesceTimer.setSingleShot(true);
    connect(&m_coalesceTimer, &QTimer::timeout, this, &UndoManager::commitCheckpoint);

    // Every signal the scene emits when something the user would want to
    // undo has changed. selectionChanged is deliberately excluded — picking
    // an object isn't an edit.
    connect(scene, &SceneGraph::objectAdded,        this, &UndoManager::onSceneMutated);
    connect(scene, &SceneGraph::objectRemoved,      this, &UndoManager::onSceneMutated);
    connect(scene, &SceneGraph::objectChanged,      this, &UndoManager::onSceneMutated);
    connect(scene, &SceneGraph::jointAdded,         this, &UndoManager::onSceneMutated);
    connect(scene, &SceneGraph::motorAdded,         this, &UndoManager::onSceneMutated);
    connect(scene, &SceneGraph::rigidBodyAdded,     this, &UndoManager::onSceneMutated);
    connect(scene, &SceneGraph::rigidBodyRemoved,   this, &UndoManager::onSceneMutated);
    connect(scene, &SceneGraph::rigidBodyChanged,   this, &UndoManager::onSceneMutated);
    connect(scene, &SceneGraph::conveyorBeltAdded,  this, &UndoManager::onSceneMutated);
    connect(scene, &SceneGraph::conveyorBeltRemoved,this, &UndoManager::onSceneMutated);
    connect(scene, &SceneGraph::conveyorBeltChanged,this, &UndoManager::onSceneMutated);
    connect(scene, &SceneGraph::floorChanged,       this, &UndoManager::onSceneMutated);
    connect(scene, &SceneGraph::automationChanged,  this, &UndoManager::onSceneMutated);

    m_baseline = snapshot();
}

QJsonObject UndoManager::snapshot() {
    return ProjectSerializer::toJson(*m_scene, &m_meshPool);
}

void UndoManager::restore(const QJsonObject& snapshot) {
    m_restoring = true;
    ProjectSerializer::fromJson(*m_scene, snapshot, &m_meshPool);
    m_restoring = false;
}

// Releases meshes no snapshot refers to any more (deleted parts that fell
// off the end of the history, or the previous project after reset())
void UndoManager::pruneMeshPool() {
    std::set<int> keys;
    MeshPool::collectKeys(m_baseline, &keys);
    for (auto& s : m_undoStack) MeshPool::collectKeys(s, &keys);
    for (auto& s : m_redoStack) MeshPool::collectKeys(s, &keys);
    m_meshPool.retainOnly(keys);
}

void UndoManager::onSceneMutated() {
    if (m_restoring) return;
    m_coalesceTimer.start(kCoalesceMs);
}

void UndoManager::commitCheckpoint() {
    if (m_restoring) return;
    QJsonObject current = snapshot();
    if (current == m_baseline) return;

    m_undoStack.push_back(m_baseline);
    if (m_undoStack.size() > kMaxDepth)
        m_undoStack.erase(m_undoStack.begin());
    m_redoStack.clear();
    m_baseline = current;
    pruneMeshPool();
    emit historyChanged();
}

void UndoManager::undo() {
    m_coalesceTimer.stop();
    commitCheckpoint();
    if (m_undoStack.empty()) return;

    QJsonObject prev = m_undoStack.back();
    m_undoStack.pop_back();
    m_redoStack.push_back(m_baseline);

    restore(prev);

    m_baseline = prev;
    emit historyChanged();
}

void UndoManager::redo() {
    m_coalesceTimer.stop();
    commitCheckpoint();
    if (m_redoStack.empty()) return;

    QJsonObject next = m_redoStack.back();
    m_redoStack.pop_back();
    m_undoStack.push_back(m_baseline);

    restore(next);

    m_baseline = next;
    emit historyChanged();
}

void UndoManager::reset() {
    m_coalesceTimer.stop();
    m_undoStack.clear();
    m_redoStack.clear();
    m_meshPool.clear();
    m_baseline = snapshot();
    emit historyChanged();
}
