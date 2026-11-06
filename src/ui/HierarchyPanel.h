#pragma once
#include <QWidget>
#include <QTreeWidget>
#include <QMap>
#include <QSet>
#include <QIcon>
#include "../scene/SceneGraph.h"

class HierarchyPanel : public QWidget {
    Q_OBJECT
public:
    explicit HierarchyPanel(SceneGraph* scene, QWidget* parent = nullptr);
    // Objects selected in the tree (several with Ctrl/Shift), else the scene's selection
    std::vector<int> selectedIds() const;

signals:
    void objectSelected(int id);
    void duplicateRequested(int id);
    void addSensorRequested(int id);
    void addSourceRequested(int id);   // the object becomes an object source's template
    void addSinkRequested(int id);     // a sink on the object's bounds
    void toggleHiddenRequested();      // Hide / Show on the selected items (Alt+V)

private slots:
    void onObjectAdded(int id);
    void onObjectRemoved(int id);
    void onSelectionChanged(int id);
    void onTreeSelectionChanged();
    void onItemDoubleClicked(QTreeWidgetItem* item, int col);
    void showContextMenu(const QPoint& pos);
    void onSceneCleared();
    void onRigidBodyChanged(int objectId);

private:
    void rebuild();
    void scheduleRebuild();
    void revealSelection(int id, bool centre);
    void addItem(int objectId, QTreeWidgetItem* parent, const QString& parentPath);
    void applyRigidBodyIcon(QTreeWidgetItem* item, int objectId);
    void refreshHidden(QTreeWidgetItem* item);   // the item and its subtree
    void combineSelectedIntoRigidBody(const QList<QTreeWidgetItem*>& items);
    void deleteSelectedItems(const QList<QTreeWidgetItem*>& items);
    void setRelation(int childId, int parentId);

    QTreeWidget*              m_tree;
    SceneGraph*               m_scene;
    QMap<int, QTreeWidgetItem*> m_items;
    bool                       m_ignoreSelection = false;
    // Scene changes come in bursts (an undo re-adds every object), so the
    // tree is rebuilt once, after the burst, instead of once per object
    bool                       m_rebuildPending = false;
    // Assemblies the user collapsed, by name path, kept across rebuilds
    QSet<QString>              m_collapsed;
};
