#include "HierarchyPanel.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QHeaderView>
#include <QMenu>
#include "robot/RobotAnalysis.h"
#include <QInputDialog>
#include <QMessageBox>
#include <QShortcut>
#include <QScrollBar>
#include "Theme.h"
#include "PrimitiveIcons.h"
#include <QDropEvent>
#include <functional>

namespace {
// The tree lets items be dragged onto others. The drop only reports where
// they went; the scene moves them and the tree is rebuilt from it.
class HierarchyTree : public QTreeWidget {
public:
    using QTreeWidget::QTreeWidget;
    std::function<void(const QList<QTreeWidgetItem*>&, int)> dropped;   // items, new parent id (-1 = top level)
protected:
    void dropEvent(QDropEvent* e) override {
        if(e->source() != this){ e->ignore(); return; }
        QTreeWidgetItem* target = itemAt(e->position().toPoint());
        int parent = -1;
        if(target){
            switch(dropIndicatorPosition()){
                case OnItem:   parent = target->data(0, Qt::UserRole).toInt(); break;
                case AboveItem:
                case BelowItem: parent = target->parent() ? target->parent()->data(0, Qt::UserRole).toInt() : -1; break;
                case OnViewport: break;
            }
        }
        // Not a move for Qt: it would delete the dragged items itself
        e->setDropAction(Qt::IgnoreAction);
        e->accept();
        stopAutoScroll();
        setState(NoState);
        viewport()->update();
        if(dropped) dropped(selectedItems(), parent);
    }
};
}

static const char* typeName(const SceneObject& obj) {
    switch(obj.type){
        case PrimitiveType::Cube:         return "Cube";
        case PrimitiveType::Cylinder:     return "Cylinder";
        case PrimitiveType::Sphere:       return "Sphere";
        case PrimitiveType::Capsule:      return "Capsule";
        case PrimitiveType::Basket:       return "Basket";
        case PrimitiveType::ImportedMesh: return obj.mesh ? "Imported part" : "Assembly";
    }
    return "";
}

HierarchyPanel::HierarchyPanel(SceneGraph* scene, QWidget* parent)
    : QWidget(parent), m_scene(scene)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    auto* tree = new HierarchyTree(this);
    m_tree = tree;
    m_tree->setObjectName("hierarchyTree");
    m_tree->setHeaderHidden(true);
    m_tree->setIndentation(16);
    m_tree->setIconSize(QSize(16, 16));
    m_tree->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_tree->setContextMenuPolicy(Qt::CustomContextMenu);
    m_tree->setCursor(Qt::PointingHandCursor);
    // Drag items onto another to put them under it, between items to put them
    // next to it, or onto the empty space below to make them top level
    m_tree->setDragDropMode(QAbstractItemView::InternalMove);
    m_tree->setDragEnabled(true);
    m_tree->setAcceptDrops(true);
    m_tree->setDropIndicatorShown(true);
    tree->dropped = [this](const QList<QTreeWidgetItem*>& items, int parentId){
        std::vector<int> ids;
        for(QTreeWidgetItem* item : items) ids.push_back(item->data(0, Qt::UserRole).toInt());
        if(!ids.empty()) emit moveRequested(ids, parentId);
    };
    layout->addWidget(m_tree, 1);

    connect(m_tree, &QTreeWidget::itemSelectionChanged,
            this, &HierarchyPanel::onTreeSelectionChanged);
    connect(m_tree, &QTreeWidget::itemDoubleClicked,
            this, &HierarchyPanel::onItemDoubleClicked);
    connect(m_tree, &QTreeWidget::customContextMenuRequested,
            this, &HierarchyPanel::showContextMenu);
    auto remember = [this](QTreeWidgetItem* item, bool collapsed){
        const QString path = item->data(0, Qt::UserRole + 1).toString();
        if(collapsed) m_collapsed.insert(path); else m_collapsed.remove(path);
    };
    connect(m_tree, &QTreeWidget::itemCollapsed, this, [remember](QTreeWidgetItem* i){ remember(i, true); });
    connect(m_tree, &QTreeWidget::itemExpanded,  this, [remember](QTreeWidgetItem* i){ remember(i, false); });

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
    // Hiding (Alt+V) dims the object and its subtree
    connect(scene, &SceneGraph::objectChanged, this, [this](int id){
        if(m_rebuildPending || !m_items.contains(id)) return;
        QTreeWidgetItem* item = m_items[id];
        if(item->data(0, Qt::UserRole + 2).toBool() != m_scene->isHidden(id)) refreshHidden(item);
    });
    connect(Theme::notifier(), &Theme::Notifier::changed, this, &HierarchyPanel::scheduleRebuild);
}

std::vector<int> HierarchyPanel::selectedIds() const {
    std::vector<int> ids;
    for(QTreeWidgetItem* item : m_tree->selectedItems())
        ids.push_back(item->data(0, Qt::UserRole).toInt());
    if(ids.empty() && m_scene->selectedObjectId() != -1) ids.push_back(m_scene->selectedObjectId());
    return ids;
}

void HierarchyPanel::refreshHidden(QTreeWidgetItem* item) {
    applyRigidBodyIcon(item, item->data(0, Qt::UserRole).toInt());
    for(int i = 0; i < item->childCount(); ++i) refreshHidden(item->child(i));
}

void HierarchyPanel::addItem(int objectId, QTreeWidgetItem* parentItem, const QString& parentPath) {
    const SceneObject* obj = m_scene->getObject(objectId);
    if(!obj || obj->copyOf != -1) return;   // copies made while simulating aren't listed

    auto* item = new QTreeWidgetItem();
    item->setText(0, obj->name);
    item->setData(0, Qt::UserRole, objectId);
    const QString path = parentPath + '/' + obj->name;
    item->setData(0, Qt::UserRole + 1, path);

    if(parentItem)
        parentItem->addChild(item);
    else
        m_tree->addTopLevelItem(item);

    m_items[objectId] = item;
    item->setExpanded(!m_collapsed.contains(path));
    applyRigidBodyIcon(item, objectId);

    for(int childId : obj->childIds)
        addItem(childId, item, path);
}

// The item's icon shows the primitive type; objects that carry a RigidBody are
// drawn in the physics colour.
void HierarchyPanel::applyRigidBodyIcon(QTreeWidgetItem* item, int objectId) {
    const SceneObject* obj = m_scene->getObject(objectId);
    if(!obj) return;
    const bool rigid = m_scene->getRigidBody(objectId) != nullptr;
    item->setIcon(0, obj->hidden ? Theme::icon("eye-off", Theme::Tint::Muted)
                                 : Theme::icon(primitiveIconName(obj->type, obj->mesh != nullptr),
                                               rigid ? Theme::Tint::Physics : Theme::Tint::Muted));
    // A hidden object and everything under it is listed dimmed
    const int hiddenBy = m_scene->hiddenBy(objectId);
    item->setData(0, Qt::UserRole + 2, hiddenBy != -1);
    if(hiddenBy != -1) item->setForeground(0, Theme::colors().muted);
    else item->setData(0, Qt::ForegroundRole, QVariant());
    QString tip = QString(typeName(*obj)) + (rigid ? "  ·  Rigid body" : "");
    if(obj->hidden) tip += "  ·  Hidden (Alt+V shows it)";
    else if(hiddenBy != -1) tip += "  ·  Hidden with " + m_scene->getObject(hiddenBy)->name;
    item->setToolTip(0, tip);
}

void HierarchyPanel::onRigidBodyChanged(int objectId) {
    if(m_items.contains(objectId))
        applyRigidBodyIcon(m_items[objectId], objectId);
}

void HierarchyPanel::rebuild() {
    m_rebuildPending = false;
    m_ignoreSelection = true;
    const int scroll = m_tree->verticalScrollBar()->value();
    m_tree->setUpdatesEnabled(false);
    m_tree->clear();
    m_items.clear();
    for(int rootId : m_scene->rootObjects())
        addItem(rootId, nullptr, QString());
    m_tree->verticalScrollBar()->setValue(scroll);
    m_tree->setUpdatesEnabled(true);
    m_ignoreSelection = false;
    // Keep showing the scene's selection; the tree is new, so select it again
    revealSelection(m_scene->selectedObjectId(), false);
}

void HierarchyPanel::scheduleRebuild() {
    if(m_rebuildPending) return;
    m_rebuildPending = true;
    QMetaObject::invokeMethod(this, [this]{ if(m_rebuildPending) rebuild(); }, Qt::QueuedConnection);
}

void HierarchyPanel::onObjectAdded(int) { scheduleRebuild(); }
void HierarchyPanel::onObjectRemoved(int) { scheduleRebuild(); }
void HierarchyPanel::onSceneCleared() { scheduleRebuild(); }

void HierarchyPanel::onSelectionChanged(int id) {
    // If this fired because the tree itself just changed m_scene's selection
    // (see onTreeSelectionChanged), don't fight the tree's own selection —
    // that would collapse a multi-selection back down to one item.
    if(m_ignoreSelection) return;
    if(m_rebuildPending){ rebuild(); return; }   // rebuild() selects it
    revealSelection(id, true);
}

// Selects the object's item; picked in the viewport, maybe deep in a
// collapsed assembly, it opens the way to it and brings it into view
void HierarchyPanel::revealSelection(int id, bool centre) {
    m_ignoreSelection = true;
    m_tree->clearSelection();
    if(id != -1 && m_items.contains(id)){
        QTreeWidgetItem* item = m_items[id];
        bool opened = false;
        for(QTreeWidgetItem* p = item->parent(); p; p = p->parent())
            if(!p->isExpanded()){ p->setExpanded(true); opened = true; }
        if(opened) m_tree->doItemsLayout();   // so the scroll range includes the opened rows
        // Centred when it has to scroll, so the parts around it show too;
        // left alone when it's already in view
        const bool inView = m_tree->viewport()->rect().contains(m_tree->visualItemRect(item));
        m_tree->setCurrentItem(item, 0, QItemSelectionModel::ClearAndSelect);
        if(!inView) m_tree->scrollToItem(item, centre ? QAbstractItemView::PositionAtCenter
                                                      : QAbstractItemView::EnsureVisible);
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

    // Hide / Show for the whole selection, the same as Alt+V
    auto addHideShow = [&]{
        bool anyShown = false;
        for(QTreeWidgetItem* s : selected)
            if(!m_scene->isHidden(s->data(0, Qt::UserRole).toInt())) anyShown = true;
        menu.addAction(Theme::icon(anyShown ? "eye-off" : "eye"), anyShown ? "Hide\tAlt+V" : "Show\tAlt+V",
                       this, [this]{ emit toggleHiddenRequested(); });
    };

    std::vector<int> selectedIdList;
    for(QTreeWidgetItem* s : selected) selectedIdList.push_back(s->data(0, Qt::UserRole).toInt());
    // Group and Move to Top Level, for any selection
    auto addGrouping = [&]{
        menu.addAction(Theme::icon("group"), selected.size() == 1 ? "Group\tCtrl+G"
                                             : QString("Group %1 Objects\tCtrl+G").arg(selected.size()),
                       this, [this, selectedIdList]{ emit groupRequested(selectedIdList); });
        bool nested = false;
        for(QTreeWidgetItem* s : selected) if(s->parent()) nested = true;
        if(nested)
            menu.addAction(Theme::icon("arrow-up-to-line"), "Move to Top Level",
                           this, [this, selectedIdList]{ emit moveRequested(selectedIdList, -1); });
    };

    if(selected.isEmpty()){
        menu.addAction("(select an object to act on it)");
    } else if(selected.size() == 1){
        QTreeWidgetItem* only = selected.first();
        menu.addAction(Theme::icon("pencil"), "Rename", [this, only](){
            onItemDoubleClicked(only, 0);
        });
        menu.addAction(Theme::icon("copy"), "Duplicate\tCtrl+D", [this, only](){
            emit duplicateRequested(only->data(0, Qt::UserRole).toInt());
        });
        menu.addAction(Theme::icon("radio"), "Add Sensor Here…", [this, only](){
            emit addSensorRequested(only->data(0, Qt::UserRole).toInt());
        });
        menu.addAction(Theme::icon("boxes"), "Make Part Source…", [this, only](){
            emit addSourceRequested(only->data(0, Qt::UserRole).toInt());
        });
        menu.addAction(Theme::icon("log-out"), "Add Part Sink Here…", [this, only](){
            emit addSinkRequested(only->data(0, Qt::UserRole).toInt());
        });
        addHideShow();
        addGrouping();
        if(robot::findTool(*m_scene, only->data(0, Qt::UserRole).toInt()) != -1)
            menu.addAction(Theme::icon("gauge"), "Analyze Robot…", [this, only](){
                emit analyzeRobotRequested(only->data(0, Qt::UserRole).toInt());
            });
        menu.addSeparator();
        menu.addAction(Theme::icon("trash-2", Theme::Tint::Danger), "Delete", [this, selected](){
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
        menu.addAction(Theme::icon("weight"), "No relation \xE2\x80\x94 just combine", [this, selected](){
            combineSelectedIntoRigidBody(selected);
        });
        addHideShow();
        addGrouping();
        menu.addSeparator();
        menu.addAction(Theme::icon("trash-2", Theme::Tint::Danger), "Delete Selected (2)", [this, selected](){
            deleteSelectedItems(selected);
        });
    } else {
        menu.addAction(Theme::icon("weight"), QString("Combine into Rigid Body (%1)").arg(selected.size()), [this, selected](){
            combineSelectedIntoRigidBody(selected);
        });
        addHideShow();
        addGrouping();
        menu.addSeparator();
        menu.addAction(Theme::icon("trash-2", Theme::Tint::Danger), QString("Delete Selected (%1)").arg(selected.size()), [this, selected](){
            deleteSelectedItems(selected);
        });
    }
    menu.exec(m_tree->viewport()->mapToGlobal(pos));
}

void HierarchyPanel::deleteSelectedItems(const QList<QTreeWidgetItem*>& items) {
    std::vector<int> ids;
    for(QTreeWidgetItem* item : items)
        ids.push_back(item->data(0, Qt::UserRole).toInt());
    for(int id : ids)
        m_scene->removeObject(id);
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
