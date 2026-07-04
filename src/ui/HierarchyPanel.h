#pragma once
#include <QWidget>
#include <QTreeWidget>
#include <QMap>
#include <QIcon>
#include "../scene/SceneGraph.h"

class HierarchyPanel : public QWidget {
    Q_OBJECT
public:
    explicit HierarchyPanel(SceneGraph* scene, QWidget* parent = nullptr);

signals:
    void objectSelected(int id);

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
    void addItem(int objectId, QTreeWidgetItem* parent);
    void applyRigidBodyIcon(QTreeWidgetItem* item, int objectId);
    void combineSelectedIntoRigidBody(const QList<QTreeWidgetItem*>& items);
    void deleteSelectedItems(const QList<QTreeWidgetItem*>& items);

    QTreeWidget*              m_tree;
    SceneGraph*               m_scene;
    QMap<int, QTreeWidgetItem*> m_items;
    bool                       m_ignoreSelection = false;
    QIcon                      m_rigidBodyIcon;
};
