#include "HierarchyPanel.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QHeaderView>
#include <QMenu>
#include <QInputDialog>
#include <QMessageBox>
#include <QPainter>
#include <QPixmap>
#include <QShortcut>

// Small filled-circle icon marking objects that carry a RigidBody, drawn once
// and reused for every tree item (see applyRigidBodyIcon).
static QIcon makeRigidBodyIcon() {
    QPixmap pm(14, 14);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(QPen(QColor("#c07000"), 1.0));
    p.setBrush(QColor("#e8963c"));
    p.drawEllipse(QRectF(1.5, 1.5, 11, 11));
    p.end();
    return QIcon(pm);
}

HierarchyPanel::HierarchyPanel(SceneGraph* scene, QWidget* parent)
    : QWidget(parent), m_scene(scene)
{
    m_rigidBodyIcon = makeRigidBodyIcon();

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    auto* header = new QLabel("  Hierarchy", this);
    header->setFixedHeight(28);
    header->setStyleSheet("background:#e8ecf2; color:#333333; font-size:11px; font-weight:bold; border-bottom:1px solid #d0d0d0;");
    layout->addWidget(header);

    auto* card = new QWidget(this);
    card->setStyleSheet("QWidget#hierarchyCard { background:#ffffff; border:1px solid #d4dae8; border-radius:5px; }");
    card->setObjectName("hierarchyCard");
    auto* cardLayout = new QVBoxLayout(card);
    cardLayout->setContentsMargins(6, 6, 6, 6);
    cardLayout->setSpacing(0);
    layout->addWidget(card, 1);

    m_tree = new QTreeWidget(card);
    m_tree->setHeaderHidden(true);
    m_tree->setIndentation(16);
    m_tree->setIconSize(QSize(12, 12));
    m_tree->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_tree->setStyleSheet(
        "QTreeWidget { background:#ffffff; color:#222222; border:none; font-size:12px; outline:0; }"
        "QTreeWidget::item { padding:4px 3px; border-radius:4px; border-bottom:1px solid #f0f2f7; }"
        "QTreeWidget::item:selected { background:#cce0ff; color:#111111; border-bottom:1px solid #cce0ff; }"
        "QTreeWidget::item:hover:!selected { background:#eef4ff; }"
        "QTreeView::branch { background:transparent; }"
    );
    m_tree->setContextMenuPolicy(Qt::CustomContextMenu);
    m_tree->setCursor(Qt::PointingHandCursor);
    cardLayout->addWidget(m_tree);

    connect(m_tree, &QTreeWidget::itemSelectionChanged,
            this, &HierarchyPanel::onTreeSelectionChanged);
    connect(m_tree, &QTreeWidget::itemDoubleClicked,
            this, &HierarchyPanel::onItemDoubleClicked);
    connect(m_tree, &QTreeWidget::customContextMenuRequested,
            this, &HierarchyPanel::showContextMenu);

    auto* renameShortcut = new QShortcut(QKeySequence(Qt::Key_F2), m_tree);
    renameShortcut->setContext(Qt::WidgetShortcut);
    connect(renameShortcut, &QShortcut::activated, this, [this](){
        auto selected = m_tree->selectedItems();
        if(selected.size() == 1)
            onItemDoubleClicked(selected.first(), 0);
    });

    connect(scene, &SceneGraph::objectAdded,      this, &HierarchyPanel::onObjectAdded);
    connect(scene, &SceneGraph::objectRemoved,     this, &HierarchyPanel::onObjectRemoved);
    connect(scene, &SceneGraph::selectionChanged,  this, &HierarchyPanel::onSelectionChanged);
    connect(scene, &SceneGraph::sceneCleared,      this, &HierarchyPanel::onSceneCleared);
    connect(scene, &SceneGraph::rigidBodyAdded,    this, &HierarchyPanel::onRigidBodyChanged);
    connect(scene, &SceneGraph::rigidBodyRemoved,  this, &HierarchyPanel::onRigidBodyChanged);
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
        case PrimitiveType::Basket:   icon = "[B]"; break;
        // Imported CAD: [A] assembly node (no geometry), [P] part with mesh
        case PrimitiveType::ImportedMesh: icon = obj->mesh ? "[P]" : "[A]"; break;
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
    applyRigidBodyIcon(item, objectId);

    for(int childId : obj->childIds)
        addItem(childId, item);
}

void HierarchyPanel::applyRigidBodyIcon(QTreeWidgetItem* item, int objectId) {
    item->setIcon(0, m_scene->getRigidBody(objectId) ? m_rigidBodyIcon : QIcon());
}

void HierarchyPanel::onRigidBodyChanged(int objectId) {
    if(m_items.contains(objectId))
        applyRigidBodyIcon(m_items[objectId], objectId);
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
    // If this fired because the tree itself just changed m_scene's selection
    // (see onTreeSelectionChanged), don't fight the tree's own selection —
    // that would collapse a multi-selection back down to one item.
    if(m_ignoreSelection) return;

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

    // Updating m_scene's (single-object) selection below fires
    // selectionChanged synchronously, which would otherwise reach
    // onSelectionChanged() and clear/collapse the tree's own multi-selection.
    m_ignoreSelection = true;
    if(selected.isEmpty()){
        m_scene->deselectAll();
    } else {
        int id = selected.first()->data(0, Qt::UserRole).toInt();
        m_scene->selectObject(id);
        emit objectSelected(id);
    }
    m_ignoreSelection = false;
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

    // Right-clicking an item outside the current selection re-selects just
    // that item, matching standard tree-view convention.
    if(item && !item->isSelected()){
        m_tree->clearSelection();
        item->setSelected(true);
    }

    QList<QTreeWidgetItem*> selected = m_tree->selectedItems();

    QMenu menu(this);

    if(selected.isEmpty()){
        menu.addAction("(select an object to act on it)");
    } else if(selected.size() == 1){
        QTreeWidgetItem* only = selected.first();
        menu.addAction("Rename", [this, only](){
            onItemDoubleClicked(only, 0);
        });
        menu.addSeparator();
        menu.addAction("Delete", [this, selected](){
            deleteSelectedItems(selected);
        });
    } else if(selected.size() == 2){
        int idA = selected[0]->data(0, Qt::UserRole).toInt();
        int idB = selected[1]->data(0, Qt::UserRole).toInt();
        const SceneObject* a = m_scene->getObject(idA);
        const SceneObject* b = m_scene->getObject(idB);
        QString nameA = a ? a->name : "?";
        QString nameB = b ? b->name : "?";

        menu.addAction(QString("Make \"%1\" the parent of \"%2\"").arg(nameA, nameB), [this, idA, idB](){
            setRelation(/*child=*/idB, /*parent=*/idA);
        });
        menu.addAction(QString("Make \"%1\" the parent of \"%2\"").arg(nameB, nameA), [this, idA, idB](){
            setRelation(/*child=*/idA, /*parent=*/idB);
        });
        menu.addSeparator();
        menu.addAction("No relation \xE2\x80\x94 just combine", [this, selected](){
            combineSelectedIntoRigidBody(selected);
        });
        menu.addSeparator();
        menu.addAction("Delete Selected (2)", [this, selected](){
            deleteSelectedItems(selected);
        });
    } else {
        menu.addAction(QString("Combine into Rigid Body (%1)").arg(selected.size()), [this, selected](){
            combineSelectedIntoRigidBody(selected);
        });
        menu.addSeparator();
        menu.addAction(QString("Delete Selected (%1)").arg(selected.size()), [this, selected](){
            deleteSelectedItems(selected);
        });
    }
    menu.exec(m_tree->viewport()->mapToGlobal(pos));
}

void HierarchyPanel::deleteSelectedItems(const QList<QTreeWidgetItem*>& items) {
    for(QTreeWidgetItem* item : items)
        m_scene->removeObject(item->data(0, Qt::UserRole).toInt());
}

void HierarchyPanel::setRelation(int childId, int parentId) {
    auto result = m_scene->setParent(childId, parentId);
    switch(result){
        case SceneGraph::ReparentResult::WouldCreateCycle:
            QMessageBox::warning(this, "Cannot Set Relation",
                "That would make an object its own ancestor — pick the other direction instead.");
            break;
        case SceneGraph::ReparentResult::SameObject:
        case SceneGraph::ReparentResult::Ok:
            break;
    }
}

void HierarchyPanel::combineSelectedIntoRigidBody(const QList<QTreeWidgetItem*>& items) {
    std::vector<int> ids;
    for(QTreeWidgetItem* item : items)
        ids.push_back(item->data(0, Qt::UserRole).toInt());

    bool ok;
    QString name = QInputDialog::getText(this, "Combine into Rigid Body", "Group name:",
                                          QLineEdit::Normal, "RigidBodyGroup", &ok);
    if(!ok || name.trimmed().isEmpty()) return;

    int groupId;
    auto result = m_scene->combineIntoRigidBody(ids, name.trimmed(), groupId);
    switch(result){
        case SceneGraph::CombineResult::TooFewEligible:
            QMessageBox::warning(this, "Cannot Combine",
                "Fewer than two of the selected parts are eligible.\n"
                "Parts with their own joint (moving parts) can't be merged into a rigid body.");
            break;
        case SceneGraph::CombineResult::MixedParents:
            QMessageBox::warning(this, "Cannot Combine",
                "The selected parts must share the same parent in the hierarchy to be combined.");
            break;
        case SceneGraph::CombineResult::Ok:
            break;
    }
}
