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
    connect(scene, &SceneGraph::matesChanged,       this, &UndoManager::onSceneMutated);

    m_baseline = snapshot();
}

QJsonObject UndoManager::snapshot() {
    return ProjectSerializer::toJson(*m_scene, &m_meshPool);
}

namespace {

// Where an object sits in the tree, as names from the root down; the index
// tells same-named siblings apart. Ids are reallocated on restore, so this is
// how the selection finds its object again.
using TreePath = std::vector<std::pair<QString, int>>;

TreePath pathOf(const SceneGraph& scene, int id) {
    TreePath path;
    for (const SceneObject* o = scene.getObject(id); o; o = scene.getObject(o->parentId)) {
        const SceneObject* parent = scene.getObject(o->parentId);
        const std::vector<int>& siblings = parent ? parent->childIds : scene.rootObjects();
        int n = 0;
        for (int s : siblings) {
            if (s == o->id) break;
            if (const SceneObject* so = scene.getObject(s); so && so->name == o->name) n++;
        }
        path.insert(path.begin(), {o->name, n});
    }
    return path;
}

int objectAt(const SceneGraph& scene, const TreePath& path) {
    int id = -1;
    for (const auto& [name, index] : path) {
        const SceneObject* parent = scene.getObject(id);
        const std::vector<int>& siblings = parent ? parent->childIds : scene.rootObjects();
        int n = 0, found = -1;
        for (int s : siblings) {
            const SceneObject* so = scene.getObject(s);
            if (so && so->name == name && n++ == index) { found = s; break; }
        }
        if (found == -1) return -1;
        id = found;
    }
    return id;
}

} // namespace

void UndoManager::restore(const QJsonObject& snapshot) {
    const TreePath selected = pathOf(*m_scene, m_scene->selectedObjectId());
    m_restoring = true;
    ProjectSerializer::fromJson(*m_scene, snapshot, &m_meshPool);
    m_restoring = false;
    // Keep the selection on the same part, so its panels stay open
    m_scene->selectObject(selected.empty() ? -1 : objectAt(*m_scene, selected));
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

    // Re-taken rather than prev: loading closes gaps in the ids, so the
    // scene no longer serializes to prev exactly, and the next undo would
    // record that difference as an edit of its own
    m_baseline = snapshot();
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

    // Re-taken rather than next: loading closes gaps in the ids, so the
    // scene no longer serializes to next exactly, and the next undo would
    // record that difference as an edit of its own
    m_baseline = snapshot();
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
