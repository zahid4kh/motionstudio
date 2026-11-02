#include "AssemblePanel.h"
#include "scene/Assembly.h"
#include "scene/SceneGraph.h"
#include "ui/Theme.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <algorithm>

AssemblePanel::AssemblePanel(SceneGraph* scene, QWidget* parent) : QFrame(parent), m_scene(scene) {
    setObjectName("assemblePanel");
    setFixedWidth(310);
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(14, 12, 14, 12);
    root->setSpacing(8);

    auto* header = new QHBoxLayout;
    header->setSpacing(8);
    auto* icon = new QLabel;
    Theme::setLabelIcon(icon, "puzzle", 16);
    auto* title = new QLabel("ASSEMBLE");
    title->setObjectName("islandTitle");
    auto* close = new QToolButton;
    close->setIcon(Theme::icon("x", Theme::Tint::Muted));
    close->setAutoRaise(true);
    close->setToolTip("Leave Assemble mode (Esc)");
    header->addWidget(icon);
    header->addWidget(title, 1);
    header->addWidget(close);
    root->addLayout(header);

    m_step = new QLabel;
    m_step->setWordWrap(true);
    root->addWidget(m_step);

    // The mate being made or edited
    m_editor = new QWidget;
    auto* ev = new QVBoxLayout(m_editor);
    ev->setContentsMargins(0, 4, 0, 0);
    ev->setSpacing(8);
    m_mateTitle = new QLabel;
    m_mateTitle->setObjectName("fieldLabel");
    m_mateTitle->setWordWrap(true);
    ev->addWidget(m_mateTitle);
    auto* grid = new QGridLayout;
    grid->setHorizontalSpacing(10);
    grid->setVerticalSpacing(6);
    grid->setColumnStretch(1, 1);
    auto* typeLabel = new QLabel("Mate");
    typeLabel->setObjectName("fieldLabel");
    m_type = new QComboBox;
    m_type->setToolTip("What the two picked features hold. The list depends on what you picked.");
    grid->addWidget(typeLabel, 0, 0);
    grid->addWidget(m_type, 0, 1);
    m_offsetLabel = new QLabel("Offset");
    m_offsetLabel->setObjectName("fieldLabel");
    m_offset = new QDoubleSpinBox;
    m_offset->setRange(-100000, 100000);
    m_offset->setDecimals(2);
    m_offset->setSingleStep(1);
    m_offset->setSuffix(" mm");
    m_offset->setToolTip("Gap between the faces (Flush), or how far along the axis (Insert)");
    grid->addWidget(m_offsetLabel, 1, 0);
    grid->addWidget(m_offset, 1, 1);
    ev->addLayout(grid);
    m_flip = new QCheckBox;
    ev->addWidget(m_flip);
    auto* editButtons = new QHBoxLayout;
    auto* remove = new QPushButton(Theme::icon("trash-2", Theme::Tint::Danger), "Remove");
    remove->setProperty("danger", true);
    remove->setToolTip("Delete this mate and put the part back where it was");
    auto* done = new QPushButton(Theme::icon("check"), "Done");
    done->setToolTip("Keep the mate and pick the next one (Enter)");
    editButtons->addWidget(remove);
    editButtons->addStretch();
    editButtons->addWidget(done);
    ev->addLayout(editButtons);
    root->addWidget(m_editor);
    m_status = new QLabel;
    m_status->setWordWrap(true);
    root->addWidget(m_status);

    // Every mate in the scene
    m_listTitle = new QLabel("MATES");
    m_listTitle->setObjectName("islandTitle");
    root->addSpacing(4);
    root->addWidget(m_listTitle);
    m_list = new QListWidget;
    m_list->setToolTip("Click a mate to edit it; untick it to switch it off");
    root->addWidget(m_list);
    auto* listButtons = new QHBoxLayout;
    auto* solve = new QPushButton(Theme::icon("refresh-cw"), "Solve");
    solve->setToolTip("Move parts until every mate holds again");
    m_delete = new QPushButton(Theme::icon("trash-2", Theme::Tint::Danger), "Delete");
    m_delete->setProperty("danger", true);
    m_delete->setToolTip("Delete the selected mate; parts stay where they are");
    listButtons->addWidget(solve);
    listButtons->addStretch();
    listButtons->addWidget(m_delete);
    root->addLayout(listButtons);
    m_hint = new QLabel("Flat face → plane · round edge → hole or shaft rim · round face → axis · "
                            "straight edge → line · edge end → corner. The first part you click moves.");
    m_hint->setObjectName("hint");
    m_hint->setWordWrap(true);
    root->addWidget(m_hint);

    connect(close, &QToolButton::clicked, this, &AssemblePanel::closeRequested);
    connect(done, &QPushButton::clicked, this, &AssemblePanel::finishEdit);
    connect(remove, &QPushButton::clicked, this, [this]{
        const int id = m_editId;
        if (!m_scene->getMate(id)) return;
        // Parts go back to where they were before this mate
        for (auto& [oid, t] : m_snapshot) {
            if (SceneObject* o = m_scene->getObject(oid)) {
                o->transform = t;
                emit m_scene->objectChanged(oid);
            }
        }
        m_editId = -1;
        m_snapshot.clear();
        m_scene->removeMate(id);
        assembly::solve(*m_scene);
        syncEditor();
        emit changed();
    });
    connect(m_type, &QComboBox::currentIndexChanged, this, [this]{
        Mate* m = m_scene->getMate(m_editId);
        if (m_syncing || !m) return;
        m->type = MateType(m_type->currentData().toInt());
        resolveFromSnapshot();
    });
    connect(m_flip, &QCheckBox::toggled, this, [this](bool on){
        Mate* m = m_scene->getMate(m_editId);
        if (m_syncing || !m) return;
        m->aligned = on;
        resolveFromSnapshot();
    });
    connect(m_offset, &QDoubleSpinBox::valueChanged, this, [this](double v){
        Mate* m = m_scene->getMate(m_editId);
        if (m_syncing || !m) return;
        m->offset = float(v / 1000.0);
        resolveFromSnapshot();
    });
    connect(m_list, &QListWidget::currentItemChanged, this, [this](QListWidgetItem* item){
        if (m_syncing) return;
        if (item) editMate(item->data(Qt::UserRole).toInt());
        emit changed();
    });
    connect(m_list, &QListWidget::currentRowChanged, this, [this]{ m_delete->setEnabled(selectedMate() >= 0); });
    connect(m_list, &QListWidget::itemChanged, this, [this](QListWidgetItem* item){
        if (m_syncing) return;
        Mate* m = m_scene->getMate(item->data(Qt::UserRole).toInt());
        if (!m) return;
        m->enabled = item->checkState() == Qt::Checked;
        assembly::solve(*m_scene);
        m_scene->notifyMatesChanged();
    });
    connect(solve, &QPushButton::clicked, this, [this]{
        const assembly::SolveResult r = assembly::solve(*m_scene);
        showStatus(r.ok() ? QString("✓ Every mate holds%1.").arg(r.moved.empty() ? "" : " again")
                          : "⚠ " + r.problems.join("\n⚠ "), !r.ok());
        rebuildList();
    });
    connect(m_delete, &QPushButton::clicked, this, [this]{
        const int id = selectedMate();
        if (id < 0) return;
        if (id == m_editId) { m_editId = -1; m_snapshot.clear(); }
        m_scene->removeMate(id);
        syncEditor();
        emit changed();
    });

    connect(m_scene, &SceneGraph::matesChanged, this, &AssemblePanel::rebuildList);
    connect(m_scene, &SceneGraph::sceneCleared, this, [this]{
        // Undo, redo and Open replace the scene (and its ids)
        m_editId = -1;
        m_snapshot.clear();
        syncEditor();
        rebuildList();
    });
    // Mate status changes as parts move; refresh the list once things settle
    auto* refresh = new QTimer(this);
    refresh->setSingleShot(true);
    refresh->setInterval(150);
    connect(refresh, &QTimer::timeout, this, [this]{ if (isVisible()) rebuildList(); });
    connect(m_scene, &SceneGraph::objectChanged, refresh, qOverload<>(&QTimer::start));
    connect(m_scene, &SceneGraph::objectRemoved, refresh, qOverload<>(&QTimer::start));
    connect(Theme::notifier(), &Theme::Notifier::changed, this, &AssemblePanel::rebuildList);

    // Clicks on the panel leave keyboard focus in the viewport, so Enter
    // finishes the mate and Esc steps back there (only the offset takes typing)
    for (QWidget* w : findChildren<QWidget*>())
        if (w != m_offset && !m_offset->isAncestorOf(w)) w->setFocusPolicy(Qt::NoFocus);

    setStage(0);
    showStatus({}, false);
    syncEditor();
    rebuildList();
}

void AssemblePanel::setStage(int stage, const QString& firstPick) {
    if (stage == 0)
        m_step->setText(m_editId >= 0 ? "Adjust the mate below, or click a feature on the next part to move."
                                      : "<b>1.</b> Click a face, edge or hole on the part to <b>move</b>.");
    else
        m_step->setText(QString("<b>2.</b> Moving <b>%1</b>. Now click where it goes, on another part. "
                                "Esc to pick again.").arg(firstPick.toHtmlEscaped()));
}

bool AssemblePanel::createMate(const MateFeature& moving, const MateFeature& fixed) {
    finishEdit();
    Mate m;
    m.name = assembly::nextName(*m_scene);
    m.moving = moving;
    m.fixed = fixed;
    m.type = assembly::typesFor(moving.kind, fixed.kind).front();
    m.aligned = assembly::defaultAlignment(*m_scene, m);
    QString why;
    if (assembly::movingBody(*m_scene, m, &why) < 0) {
        showStatus("Can't make this mate: " + why + ".", true);
        return false;
    }
    takeSnapshot();
    m_editId = m_scene->addMate(m);
    resolveFromSnapshot();
    return true;
}

void AssemblePanel::editMate(int id) {
    if (id == m_editId) return;
    showStatus({}, false);
    m_editId = m_scene->getMate(id) ? id : -1;
    m_snapshot.clear();
    if (m_editId >= 0) takeSnapshot();
    syncEditor();
    setStage(0);
    emit changed();
}

void AssemblePanel::finishEdit() {
    showStatus({}, false);
    if (m_editId < 0) return;
    m_editId = -1;
    m_snapshot.clear();
    syncEditor();
    setStage(0);
    emit changed();
}

int AssemblePanel::selectedMate() const {
    QListWidgetItem* item = m_list->currentItem();
    return item ? item->data(Qt::UserRole).toInt() : -1;
}

void AssemblePanel::takeSnapshot() {
    m_snapshot.clear();
    for (auto& [id, o] : m_scene->allObjects()) m_snapshot[id] = o.transform;
}

void AssemblePanel::resolveFromSnapshot() {
    Mate* mate = m_scene->getMate(m_editId);
    if (!mate) return;
    std::vector<int> restored;
    for (auto& [id, t] : m_snapshot) {
        SceneObject* o = m_scene->getObject(id);
        if (!o) continue;
        if (o->transform.position != t.position || o->transform.rotation != t.rotation || o->transform.scale != t.scale) {
            o->transform = t;
            restored.push_back(id);
        }
    }
    const assembly::SolveResult r = assembly::solve(*m_scene);
    for (int id : restored)
        if (std::find(r.moved.begin(), r.moved.end(), id) == r.moved.end()) emit m_scene->objectChanged(id);

    const assembly::Residual res = assembly::residual(*m_scene, *mate);
    if (res.satisfied()) {
        const SceneObject* body = m_scene->getObject(assembly::movingBody(*m_scene, *mate));
        showStatus(QString("✓ %1 is in place.").arg(body ? body->name : QString("The part")), false);
    } else {
        showStatus("⚠ This mate fights another one. Flip it, pick another mate type, or remove one of them.", true);
    }
    syncEditor();
    m_scene->notifyMatesChanged();
    emit changed();
}

void AssemblePanel::showStatus(const QString& text, bool error) {
    m_status->setText(text);
    m_status->setVisible(!text.isEmpty());
    m_status->setProperty("error", error);
    Theme::repolish(m_status);
    adjustSize();
}

void AssemblePanel::syncEditor() {
    const Mate* m = m_scene->getMate(m_editId);
    m_editor->setVisible(m);
    m_hint->setVisible(!m);
    if (!m) { adjustSize(); return; }
    m_syncing = true;
    const SceneObject* a = m_scene->getObject(m->moving.objectId);
    const SceneObject* b = m_scene->getObject(m->fixed.objectId);
    m_mateTitle->setText(QString("%1 · %2 → %3").arg(m->name, a ? a->name : "?", b ? b->name : "?"));
    m_type->clear();
    for (MateType t : assembly::typesFor(m->moving.kind, m->fixed.kind)) {
        m_type->addItem(assembly::typeLabel(t, m->moving.kind, m->fixed.kind), int(t));
        if (t == m->type) m_type->setCurrentIndex(m_type->count() - 1);
    }
    m_type->setEnabled(m_type->count() > 1);
    const bool flip = assembly::usesAlignment(m->type, m->moving.kind, m->fixed.kind);
    m_flip->setVisible(flip);
    const bool planes = m->type == MateType::Flush || m->type == MateType::Parallel;
    m_flip->setText(planes && m->moving.kind == MateFeature::Kind::Plane && m->fixed.kind == MateFeature::Kind::Plane
                        ? "Level (faces point the same way)" : "Flip (point the same way)");
    m_flip->setChecked(m->aligned);
    const bool offset = assembly::usesOffset(m->type, m->moving.kind, m->fixed.kind);
    m_offsetLabel->setVisible(offset);
    m_offset->setVisible(offset);
    m_offset->setValue(double(m->offset) * 1000.0);
    m_syncing = false;
    adjustSize();
}

void AssemblePanel::rebuildList() {
    m_syncing = true;
    const int keep = m_editId >= 0 ? m_editId : selectedMate();
    m_list->clear();
    for (auto& [id, m] : m_scene->allMates()) {
        const bool ok = !m.enabled || assembly::residual(*m_scene, m).satisfied();
        auto* item = new QListWidgetItem(Theme::icon(ok ? "link" : "triangle-alert",
                                                     ok ? (m.enabled ? Theme::Tint::Text : Theme::Tint::Muted)
                                                        : Theme::Tint::Danger),
                                         m.name + "   " + assembly::describe(*m_scene, m));
        item->setData(Qt::UserRole, id);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(m.enabled ? Qt::Checked : Qt::Unchecked);
        item->setToolTip(ok ? (m.enabled ? "Holds" : "Switched off") : "Not satisfied: it conflicts with another mate");
        m_list->addItem(item);
        if (id == keep) m_list->setCurrentItem(item);
    }
    m_list->setVisible(m_list->count() > 0);
    m_listTitle->setText(m_list->count() > 0 ? QString("MATES (%1)").arg(m_list->count()) : "MATES: NONE YET");
    m_list->setFixedHeight(std::min(150, m_list->count() * 26 + 6));
    adjustSize();
    m_delete->setEnabled(selectedMate() >= 0);
    m_syncing = false;
}
