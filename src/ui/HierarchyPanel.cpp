#include "HierarchyPanel.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QHeaderView>
#include <QMenu>
#include <QInputDialog>

HierarchyPanel::HierarchyPanel(SceneGraph* scene, QWidget* parent)
    : QWidget(parent), m_scene(scene)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    auto* header = new QLabel("  Hierarchy", this);
    header->setFixedHeight(28);
    header->setStyleSheet("background:#1e2128; color:#aaaaaa; font-size:11px; font-weight:bold;");
    layout->addWidget(header);

    m_tree = new QTreeWidget(this);
    m_tree->setHeaderHidden(true);
    m_tree->setIndentation(16);
    m_tree->setStyleSheet(
        "QTreeWidget { background:#1a1c22; color:#d0d0d0; border:none; }"
        "QTreeWidget::item:selected { background:#3a4a6a; }"
        "QTreeWidget::item:hover { background:#252830; }"
    );
    m_tree->setContextMenuPolicy(Qt::CustomContextMenu);
    layout->addWidget(m_tree);

    connect(m_tree, &QTreeWidget::itemSelectionChanged,
            this, &HierarchyPanel::onTreeSelectionChanged);
    connect(m_tree, &QTreeWidget::itemDoubleClicked,
            this, &HierarchyPanel::onItemDoubleClicked);
    connect(m_tree, &QTreeWidget::customContextMenuRequested,
            this, &HierarchyPanel::showContextMenu);

    connect(scene, &SceneGraph::objectAdded,      this, &HierarchyPanel::onObjectAdded);
    connect(scene, &SceneGraph::objectRemoved,     this, &HierarchyPanel::onObjectRemoved);
    connect(scene, &SceneGraph::selectionChanged,  this, &HierarchyPanel::onSelectionChanged);
    connect(scene, &SceneGraph::sceneCleared,      this, &HierarchyPanel::onSceneCleared);
}

void HierarchyPanel::addItem(int objectId, QTreeWidgetItem* parentItem) {
    const SceneObject* obj = m_scene->getObject(objectId);
    if(!obj) return;

    QString icon;
    switch(obj->type){
        case PrimitiveType::Cube:     icon = "[C]"; break;
        case PrimitiveType::Cylinder: icon = "[Y]"; break;
        case PrimitiveType::Sphere:   icon = "[S]"; break;
        case PrimitiveType::Capsule:  icon = "[K]"; break;
    }
    auto* item = new QTreeWidgetItem();
    item->setText(0, icon + " " + obj->name);
    item->setData(0, Qt::UserRole, objectId);

    if(parentItem)
        parentItem->addChild(item);
    else
        m_tree->addTopLevelItem(item);

    m_items[objectId] = item;
    item->setExpanded(true);

    for(int childId : obj->childIds)
        addItem(childId, item);
}

void HierarchyPanel::rebuild() {
    m_ignoreSelection = true;
    m_tree->clear();
    m_items.clear();
    for(int rootId : m_scene->rootObjects())
        addItem(rootId, nullptr);
    m_ignoreSelection = false;
}

void HierarchyPanel::onObjectAdded(int) { rebuild(); }
void HierarchyPanel::onObjectRemoved(int) { rebuild(); }
void HierarchyPanel::onSceneCleared() { rebuild(); }

void HierarchyPanel::onSelectionChanged(int id) {
    m_ignoreSelection = true;
    m_tree->clearSelection();
    if(id != -1 && m_items.contains(id)){
        m_items[id]->setSelected(true);
    }
    m_ignoreSelection = false;
}

void HierarchyPanel::onTreeSelectionChanged() {
    if(m_ignoreSelection) return;
    auto selected = m_tree->selectedItems();
    if(selected.isEmpty()){
        m_scene->deselectAll();
    } else {
        int id = selected.first()->data(0, Qt::UserRole).toInt();
        m_scene->selectObject(id);
        emit objectSelected(id);
    }
}

void HierarchyPanel::onItemDoubleClicked(QTreeWidgetItem* item, int) {
    int id = item->data(0, Qt::UserRole).toInt();
    SceneObject* obj = m_scene->getObject(id);
    if(!obj) return;
    bool ok;
    QString name = QInputDialog::getText(this, "Rename", "Name:", QLineEdit::Normal, obj->name, &ok);
    if(ok && !name.trimmed().isEmpty()){
        obj->name = name.trimmed();
        rebuild();
        emit m_scene->objectChanged(id);
    }
}

void HierarchyPanel::showContextMenu(const QPoint& pos) {
    QTreeWidgetItem* item = m_tree->itemAt(pos);
    int selectedId = item ? item->data(0, Qt::UserRole).toInt() : -1;

    QMenu menu(this);

    if(item){
        menu.addAction("Rename", [this, item](){
            onItemDoubleClicked(item, 0);
        });
        menu.addSeparator();
        menu.addAction("Delete", [this, selectedId](){
            m_scene->removeObject(selectedId);
        });
    } else {
        menu.addAction("(select an object to act on it)");
    }
    menu.exec(m_tree->viewport()->mapToGlobal(pos));
}
