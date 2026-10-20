#include "ActuatorsPage.h"
#include "automation/Expression.h"
#include "physics/SimulationEngine.h"
#include "ui/Theme.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QSplitter>
#include <QTableWidget>
#include <QToolButton>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

namespace {
enum Col { C_Name, C_Joint, C_Mode, C_Vel, C_Acc, C_Active, C_Bindings, C_Pos, C_Count };

QString jointLabel(const SceneGraph& scene, const Joint& j) {
    const SceneObject* child = scene.getObject(j.childBodyId);
    return child ? QString("%1  (%2)").arg(j.name, child->name) : j.name;
}

QString bindingSummary(const Motor& m) {
    QStringList b;
    if (!m.targetExpr.trimmed().isEmpty())   b << "target = " + m.targetExpr.trimmed();
    if (!m.velocityExpr.trimmed().isEmpty()) b << "vel = " + m.velocityExpr.trimmed();
    if (!m.activeExpr.trimmed().isEmpty())   b << "active = " + m.activeExpr.trimmed();
    return b.join(";  ");
}
} // namespace

ActuatorsPage::ActuatorsPage(SceneGraph* scene, SimulationEngine* sim, QWidget* parent)
    : Page(scene, sim, parent)
{
    auto* root = new QHBoxLayout(this);
    root->setContentsMargins(6, 6, 6, 6);
    auto* split = new QSplitter(Qt::Horizontal);
    root->addWidget(split);

    // ── Left: actuator table ────────────────────────────────────────────────
    auto* left = new QWidget;
    auto* lv = new QVBoxLayout(left);
    lv->setContentsMargins(0, 0, 0, 0);
    lv->setSpacing(4);
    auto* bar = new QHBoxLayout;
    m_addBtn = new QToolButton;
    m_addBtn->setText("Actuator for joint…");
    m_addBtn->setIcon(Theme::icon("plus"));
    m_addBtn->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    m_addBtn->setObjectName("menuButton");
    m_addBtn->setToolTip("Attach an actuator to a joint that doesn't have one");
    m_delBtn = new QPushButton(Theme::icon("trash-2", Theme::Tint::Danger), "Delete");
    m_delBtn->setProperty("danger", true);
    bar->addWidget(m_addBtn);
    bar->addStretch();
    bar->addWidget(m_delBtn);
    lv->addLayout(bar);

    m_table = new QTableWidget(0, C_Count);
    m_table->setHorizontalHeaderLabels({"Name", "Joint", "Mode", "Velocity", "Accel", "Active", "Bindings", "Position"});
    m_table->verticalHeader()->hide();
    m_table->setAlternatingRowColors(true);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->verticalHeader()->setDefaultSectionSize(24);
    m_table->setColumnWidth(C_Name, 110);
    m_table->setColumnWidth(C_Joint, 150);
    m_table->setColumnWidth(C_Mode, 65);
    m_table->setColumnWidth(C_Vel, 70);
    m_table->setColumnWidth(C_Acc, 60);
    m_table->setColumnWidth(C_Active, 50);
    m_table->setColumnWidth(C_Bindings, 160);
    m_table->horizontalHeader()->setSectionResizeMode(C_Bindings, QHeaderView::Stretch);
    auto* vsplit = new QSplitter(Qt::Vertical);
    vsplit->setChildrenCollapsible(false);
    vsplit->addWidget(m_table);
    lv->addWidget(vsplit, 1);

    // Detail form
    m_form = new QGroupBox("Actuator");
    auto* fl = new QFormLayout(m_form);
    fl->setLabelAlignment(Qt::AlignRight);
    fl->setContentsMargins(8, 12, 8, 6);
    fl->setVerticalSpacing(4);
    m_name = new QLineEdit;
    m_name->setToolTip("Used in expressions: <name>.pos, <name>.inPosition, …");
    m_jointLabel = new QLabel;
    m_mode = new QComboBox;
    m_mode->addItems({"Position", "Speed"});
    m_mode->setToolTip("Position: holds still until commanded to a target.\n"
                       "Speed: runs continuously at the velocity below from the start (when active).");
    m_velocity = new QDoubleSpinBox;
    m_velocity->setRange(-1e5, 1e5);
    m_velocity->setDecimals(3);
    m_accel = new QDoubleSpinBox;
    m_accel->setRange(0, 1e6);
    m_accel->setDecimals(3);
    m_accel->setSpecialValueText("instant");
    m_accel->setToolTip("Acceleration/deceleration for a trapezoidal profile; 0 = instant");
    m_active = new QCheckBox("Active at start");
    m_bindTarget   = autoui::makeExprEdit(scene, autoui::ExprKind::Value, "optional — e.g. Pick ? -45 : 0");
    m_bindVelocity = autoui::makeExprEdit(scene, autoui::ExprKind::Value, "optional — e.g. Speed * 0.5");
    m_bindActive   = autoui::makeExprEdit(scene, autoui::ExprKind::Value, "optional — e.g. !EStop");
    m_bindTarget->setHelpText("Bound target: evaluated every scan, forces Position mode and overrides operations.");
    m_bindVelocity->setHelpText("Bound velocity: max speed (Position) or signed speed (Speed), every scan.");
    m_bindActive->setHelpText("Bound active flag: when false the actuator decelerates and holds.");
    m_velLabel = new QLabel("Velocity");
    auto* nameRow = new QHBoxLayout;
    m_name->setMaximumWidth(180);
    nameRow->addWidget(m_name);
    nameRow->addWidget(m_jointLabel, 1);
    fl->addRow("Name", nameRow);
    auto* modeRow = new QHBoxLayout;
    modeRow->addWidget(m_mode);
    modeRow->addWidget(m_active);
    modeRow->addStretch();
    fl->addRow("Mode", modeRow);
    auto* velRow = new QHBoxLayout;
    velRow->addWidget(m_velocity);
    velRow->addWidget(new QLabel("Accel"));
    velRow->addWidget(m_accel);
    velRow->addStretch();
    fl->addRow(m_velLabel, velRow);
    fl->addRow("Target =", m_bindTarget);
    fl->addRow("Velocity =", m_bindVelocity);
    fl->addRow("Active =", m_bindActive);
    vsplit->addWidget(m_form);
    vsplit->setStretchFactor(0, 1);
    vsplit->setStretchFactor(1, 0);
    split->addWidget(left);

    // ── Middle: jog ─────────────────────────────────────────────────────────
    auto* jogBox = new QGroupBox("Jog joints (edit pose)");
    auto* jv = new QVBoxLayout(jogBox);
    jv->setContentsMargins(6, 12, 6, 6);
    auto* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    m_jogHost = new QWidget;
    m_jogLayout = new QVBoxLayout(m_jogHost);
    m_jogLayout->setContentsMargins(0, 0, 0, 0);
    m_jogLayout->setSpacing(2);
    scroll->setWidget(m_jogHost);
    jv->addWidget(scroll, 1);
    auto* jb = new QHBoxLayout;
    m_toInitBtn  = new QPushButton(Theme::icon("house"), "All to Init");
    m_setInitBtn = new QPushButton(Theme::icon("pin"),   "Current → Init");
    m_toInitBtn->setToolTip("Move every joint back to its saved initial value");
    m_setInitBtn->setToolTip("Store the current joint values as each joint's initial value (saved with the project)");
    jb->addWidget(m_toInitBtn);
    jb->addWidget(m_setInitBtn);
    jv->addLayout(jb);
    split->addWidget(jogBox);

    // ── Right: poses ────────────────────────────────────────────────────────
    auto* poseBox = new QGroupBox("Poses (teach points)");
    auto* pv = new QVBoxLayout(poseBox);
    pv->setContentsMargins(6, 12, 6, 6);
    m_poses = new QListWidget;
    m_poses->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed);
    m_poses->setToolTip("Double-click to rename. Use MoveToPose operations to move to a pose.");
    pv->addWidget(m_poses, 1);
    auto* pb1 = new QHBoxLayout;
    m_poseCapture = new QPushButton(Theme::icon("camera"),     "Capture");
    m_poseUpdate  = new QPushButton(Theme::icon("refresh-cw"), "Update");
    m_poseCapture->setToolTip("Save the current joint values as a new pose");
    m_poseUpdate->setToolTip("Overwrite the selected pose with the current joint values");
    pb1->addWidget(m_poseCapture);
    pb1->addWidget(m_poseUpdate);
    pv->addLayout(pb1);
    auto* pb2 = new QHBoxLayout;
    m_poseGo     = new QPushButton(Theme::icon("navigation"), "Go to");
    m_poseDelete = new QPushButton(Theme::icon("trash-2", Theme::Tint::Danger), "Delete");
    m_poseDelete->setProperty("danger", true);
    m_poseGo->setToolTip("Jog all joints to the selected pose (edit mode)");
    pb2->addWidget(m_poseGo);
    pb2->addWidget(m_poseDelete);
    pv->addLayout(pb2);
    split->addWidget(poseBox);

    split->setStretchFactor(0, 5);
    split->setStretchFactor(1, 3);
    split->setStretchFactor(2, 2);

    // ── Wiring ──────────────────────────────────────────────────────────────
    connect(m_addBtn, &QToolButton::clicked, this, &ActuatorsPage::addActuatorMenu);
    connect(m_delBtn, &QPushButton::clicked, this, &ActuatorsPage::deleteActuator);
    connect(m_table, &QTableWidget::currentCellChanged, this, [this](int row, int, int, int){
        if (m_loading || row < 0) return;
        auto* it = m_table->item(row, C_Name);
        m_selectedMotor = it ? it->data(Qt::UserRole).toInt() : -1;
        loadForm();
    });
    connect(m_name, &QLineEdit::editingFinished, this, &ActuatorsPage::renameActuator);
    connect(m_mode, &QComboBox::currentIndexChanged, this, &ActuatorsPage::commitForm);
    connect(m_velocity, &QDoubleSpinBox::valueChanged, this, &ActuatorsPage::commitForm);
    connect(m_accel, &QDoubleSpinBox::valueChanged, this, &ActuatorsPage::commitForm);
    connect(m_active, &QCheckBox::toggled, this, &ActuatorsPage::commitForm);
    for (auto* e : {m_bindTarget, m_bindVelocity, m_bindActive})
        connect(e, &ExpressionEdit::committed, this, &ActuatorsPage::commitForm);

    connect(m_toInitBtn, &QPushButton::clicked, this, [this]{
        for (auto& [jid, j] : m_scene->allJoints())
            if (Joint* jj = m_scene->getJoint(jid)) jj->currentValue = jj->initialAngle;
        m_scene->notifyJointsMoved();
        rebuildJog();
    });
    connect(m_setInitBtn, &QPushButton::clicked, this, [this]{
        edit([&]{
            for (auto& [jid, j] : m_scene->allJoints())
                if (Joint* jj = m_scene->getJoint(jid)) jj->initialAngle = jj->currentValue;
            m_scene->notifyAutomationChanged();
        });
        emit statusMessage("Current joint values stored as initial values", true);
    });

    connect(m_poseCapture, &QPushButton::clicked, this, &ActuatorsPage::capturePose);
    connect(m_poseUpdate,  &QPushButton::clicked, this, &ActuatorsPage::updatePose);
    connect(m_poseGo,      &QPushButton::clicked, this, &ActuatorsPage::goToPose);
    connect(m_poseDelete,  &QPushButton::clicked, this, &ActuatorsPage::deletePose);
    connect(m_poses, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem*){ /* rename via editor */ });
    connect(m_poses, &QListWidget::itemChanged, this, [this](QListWidgetItem* it){
        if (m_loading) return;
        Pose* p = m_scene->getPose(it->data(Qt::UserRole).toInt());
        if (!p || p->name == it->text().trimmed() || it->text().trimmed().isEmpty()) return;
        edit([&]{ p->name = it->text().trimmed(); m_scene->notifyAutomationChanged(); });
    });

    refresh();
}

// ── Build ────────────────────────────────────────────────────────────────────

void ActuatorsPage::refresh() {
    rebuildTable();
    rebuildJog();
    rebuildPoses();
    loadForm();
}

void ActuatorsPage::rebuildTable() {
    m_loading = true;
    m_table->setRowCount(0);
    int row = 0, selRow = -1;
    for (int mid : m_scene->motorIdsSorted()) {
        const Motor* m = m_scene->getMotor(mid);
        const Joint* j = m_scene->getJoint(m->jointId);
        m_table->insertRow(row);
        auto* name = new QTableWidgetItem(m->name);
        name->setData(Qt::UserRole, mid);
        QFont f = name->font(); f.setBold(true); name->setFont(f);
        m_table->setItem(row, C_Name, name);
        m_table->setItem(row, C_Joint, new QTableWidgetItem(j ? jointLabel(*m_scene, *j) : "(missing)"));
        m_table->setItem(row, C_Mode, new QTableWidgetItem(m->mode == ActuatorMode::Speed ? "Speed" : "Position"));
        m_table->setItem(row, C_Vel, new QTableWidgetItem(QString::number(m->velocity, 'g', 4) + autoui::velUnit(j)));
        m_table->setItem(row, C_Acc, new QTableWidgetItem(m->acceleration > 0 ? QString::number(m->acceleration, 'g', 4) : "—"));
        m_table->setItem(row, C_Active, new QTableWidgetItem(m->enabled ? "yes" : "no"));
        m_table->setItem(row, C_Bindings, new QTableWidgetItem(bindingSummary(*m)));
        auto* pos = new QTableWidgetItem(j ? QString::number(j->currentValue, 'f', 2) + autoui::posUnit(j) : "");
        pos->setFont(QFont("monospace"));
        m_table->setItem(row, C_Pos, pos);
        if (mid == m_selectedMotor) selRow = row;
        ++row;
    }
    if (selRow < 0 && row > 0) {
        selRow = 0;
        m_selectedMotor = m_table->item(0, C_Name)->data(Qt::UserRole).toInt();
    }
    if (row == 0) m_selectedMotor = -1;
    if (selRow >= 0) m_table->selectRow(selRow);
    m_loading = false;

    // Joints that can still get an actuator
    bool any = false;
    for (auto& [jid, j] : m_scene->allJoints())
        if (!m_scene->motorForJoint(jid)) any = true;
    m_addBtn->setEnabled(any && !m_locked);
    m_delBtn->setEnabled(row > 0 && !m_locked);
}

void ActuatorsPage::rebuildJog() {
    // Clear
    m_jogRows.clear();
    while (QLayoutItem* it = m_jogLayout->takeAt(0)) {
        if (it->widget()) it->widget()->deleteLater();
        delete it;
    }
    std::vector<int> jids;
    for (auto& [jid, _] : m_scene->allJoints()) jids.push_back(jid);
    std::sort(jids.begin(), jids.end());

    if (jids.empty()) {
        auto* l = new QLabel("No joints yet — select a part with a parent and use Add Joint.");
        l->setWordWrap(true);
        l->setObjectName("subtle");
        m_jogLayout->addWidget(l);
    }
    for (int jid : jids) {
        const Joint* j = m_scene->getJoint(jid);
        bool prism = j->type == JointType::Prismatic;
        double scale = prism ? 1000.0 : 10.0;
        double lo = j->limitMin, hi = j->limitMax;
        if (!prism && j->continuous) { lo = -360; hi = 360; }
        if (lo > hi) std::swap(lo, hi);

        auto* row = new QWidget;
        auto* h = new QHBoxLayout(row);
        h->setContentsMargins(0, 0, 0, 0);
        h->setSpacing(4);
        const Motor* m = m_scene->motorForJoint(jid);
        auto* lbl = new QLabel(m ? m->name : j->name);
        lbl->setToolTip(jointLabel(*m_scene, *j) + (m ? "" : "\n(no actuator — cannot be moved by automation)"));
        lbl->setFixedWidth(90);
        if (!m) lbl->setObjectName("subtle");
        auto* sl = new QSlider(Qt::Horizontal);
        sl->setRange(int(std::floor(lo * scale)), int(std::ceil(hi * scale)));
        sl->setValue(int(std::lround(j->currentValue * scale)));
        auto* sp = new QDoubleSpinBox;
        sp->setRange(lo, hi);
        sp->setDecimals(prism ? 4 : 2);
        sp->setSingleStep(prism ? 0.005 : 1.0);
        sp->setSuffix(autoui::posUnit(j));
        sp->setValue(j->currentValue);
        sp->setFixedWidth(108);
        sl->setEnabled(!m_locked);
        sp->setEnabled(!m_locked);
        h->addWidget(lbl);
        h->addWidget(sl, 1);
        h->addWidget(sp);
        m_jogLayout->addWidget(row);

        auto apply = [this, jid](double v) {
            Joint* jj = m_scene->getJoint(jid);
            if (!jj || m_locked) return;
            jj->currentValue = float(v);
            m_scene->notifyJointsMoved();
        };
        connect(sl, &QSlider::valueChanged, this, [sp, scale, apply](int v){
            QSignalBlocker b(sp);
            sp->setValue(v / scale);
            apply(v / scale);
        });
        connect(sp, &QDoubleSpinBox::valueChanged, this, [sl, scale, apply](double v){
            QSignalBlocker b(sl);
            sl->setValue(int(std::lround(v * scale)));
            apply(v);
        });
        m_jogRows.push_back({jid, sl, sp, scale});
    }
    m_jogLayout->addStretch();
    m_toInitBtn->setEnabled(!m_locked && !jids.empty());
    m_setInitBtn->setEnabled(!m_locked && !jids.empty());
}

void ActuatorsPage::rebuildPoses() {
    m_loading = true;
    int keep = currentPoseId();
    m_poses->clear();
    for (auto& [pid, p] : m_scene->allPoses()) {
        auto* it = new QListWidgetItem(p.name);
        it->setData(Qt::UserRole, pid);
        QStringList tip;
        for (auto& [jid, v] : p.jointValues) {
            const Joint* j = m_scene->getJoint(jid);
            if (!j) continue;
            const Motor* m = m_scene->motorForJoint(jid);
            tip << QString("%1: %2%3").arg(m ? m->name : j->name).arg(v, 0, 'f', 2).arg(autoui::posUnit(j));
        }
        it->setToolTip(tip.join("\n"));
        if (!m_locked) it->setFlags(it->flags() | Qt::ItemIsEditable);
        m_poses->addItem(it);
        if (pid == keep) m_poses->setCurrentItem(it);
    }
    bool has = m_poses->count() > 0;
    m_poseCapture->setEnabled(!m_locked && !m_scene->allJoints().empty());
    m_poseUpdate->setEnabled(!m_locked && has);
    m_poseGo->setEnabled(!m_locked && has);
    m_poseDelete->setEnabled(!m_locked && has);
    m_loading = false;
}

void ActuatorsPage::loadForm() {
    const Motor* m = m_scene->getMotor(m_selectedMotor);
    m_form->setEnabled(m != nullptr && !m_locked);
    if (!m) {
        m_form->setTitle("Actuator");
        return;
    }
    m_loading = true;
    const Joint* j = m_scene->getJoint(m->jointId);
    m_form->setTitle("Actuator — " + m->name);
    m_name->setText(m->name);
    m_jointLabel->setText(j ? "on " + jointLabel(*m_scene, *j) : "");
    m_jointLabel->setObjectName("subtle");
    m_mode->setCurrentIndex(m->mode == ActuatorMode::Speed ? 1 : 0);
    m_velLabel->setText(m->mode == ActuatorMode::Speed ? "Speed" : "Max speed");
    m_velocity->setSuffix(autoui::velUnit(j));
    m_accel->setSuffix(j && j->type == JointType::Prismatic ? " m/s²" : "°/s²");
    m_velocity->setValue(m->velocity);
    m_accel->setValue(m->acceleration);
    m_active->setChecked(m->enabled);
    m_bindTarget->setExpression(m->targetExpr);
    m_bindVelocity->setExpression(m->velocityExpr);
    m_bindActive->setExpression(m->activeExpr);
    m_loading = false;
}

int ActuatorsPage::currentMotorId() const { return m_selectedMotor; }

void ActuatorsPage::selectActuatorForJoint(int jointId) {
    if (const Motor* m = m_scene->motorForJoint(jointId)) {
        m_selectedMotor = m->id;
        rebuildTable();
        loadForm();
    }
}

void ActuatorsPage::refreshRuntime() {
    for (int r = 0; r < m_table->rowCount(); ++r) {
        const Motor* m = m_scene->getMotor(m_table->item(r, C_Name)->data(Qt::UserRole).toInt());
        const Joint* j = m ? m_scene->getJoint(m->jointId) : nullptr;
        if (!j) continue;
        m_table->item(r, C_Pos)->setText(QString::number(j->currentValue, 'f', 2) + autoui::posUnit(j));
    }
    if (m_locked) {
        for (auto& row : m_jogRows) {
            const Joint* j = m_scene->getJoint(row.jointId);
            if (!j) continue;
            QSignalBlocker b1(row.slider), b2(row.spin);
            row.slider->setValue(int(std::lround(j->currentValue * row.scale)));
            row.spin->setValue(j->currentValue);
        }
    }
}

void ActuatorsPage::setLocked(bool locked) {
    Page::setLocked(locked);
    refresh();
}

// ── Edits ────────────────────────────────────────────────────────────────────

void ActuatorsPage::commitForm() {
    if (m_loading || m_locked) return;
    Motor* m = m_scene->getMotor(m_selectedMotor);
    if (!m) return;
    edit([&]{
        m->mode         = m_mode->currentIndex() == 1 ? ActuatorMode::Speed : ActuatorMode::Position;
        m->velocity     = float(m_velocity->value());
        m->acceleration = float(m_accel->value());
        m->enabled      = m_active->isChecked();
        m->targetExpr   = m_bindTarget->text();
        m->velocityExpr = m_bindVelocity->text();
        m->activeExpr   = m_bindActive->text();
        m_scene->notifyAutomationChanged();
    });
    m_velLabel->setText(m->mode == ActuatorMode::Speed ? "Speed" : "Max speed");
    // Refresh the summary row without disturbing the form
    m_loading = true;
    for (int r = 0; r < m_table->rowCount(); ++r) {
        if (m_table->item(r, C_Name)->data(Qt::UserRole).toInt() != m->id) continue;
        const Joint* j = m_scene->getJoint(m->jointId);
        m_table->item(r, C_Mode)->setText(m->mode == ActuatorMode::Speed ? "Speed" : "Position");
        m_table->item(r, C_Vel)->setText(QString::number(m->velocity, 'g', 4) + autoui::velUnit(j));
        m_table->item(r, C_Acc)->setText(m->acceleration > 0 ? QString::number(m->acceleration, 'g', 4) : "—");
        m_table->item(r, C_Active)->setText(m->enabled ? "yes" : "no");
        m_table->item(r, C_Bindings)->setText(bindingSummary(*m));
    }
    m_loading = false;
}

void ActuatorsPage::renameActuator() {
    if (m_loading || m_locked) return;
    Motor* m = m_scene->getMotor(m_selectedMotor);
    if (!m) return;
    QString n = m_name->text().trimmed();
    if (n == m->name) return;
    if (!expr::isValidName(n)) {
        emit statusMessage("'" + n + "' is not a valid name — use letters, digits and _", false);
        m_name->setText(m->name);
        return;
    }
    if (m_scene->isSymbolNameTaken(n, m->id)) {
        emit statusMessage("The name '" + n + "' is already used", false);
        m_name->setText(m->name);
        return;
    }
    QString old = m->name;
    edit([&]{
        m->name = n;
        m_scene->renameSymbolReferences(old, n);
    });
    emit statusMessage(QString("Renamed %1 → %2 (expressions updated)").arg(old, n), true);
    scheduleRefresh();
}

void ActuatorsPage::addActuatorMenu() {
    QMenu menu;
    std::vector<int> jids;
    for (auto& [jid, _] : m_scene->allJoints()) jids.push_back(jid);
    std::sort(jids.begin(), jids.end());
    for (int jid : jids) {
        if (m_scene->motorForJoint(jid)) continue;
        const Joint* j = m_scene->getJoint(jid);
        QAction* a = menu.addAction(Theme::icon("rotate-3d"), jointLabel(*m_scene, *j));
        connect(a, &QAction::triggered, this, [this, jid]{
            const Joint* jj = m_scene->getJoint(jid);
            if (!jj) return;
            int n = 1;
            while (m_scene->isSymbolNameTaken(QString("J%1").arg(n))) ++n;
            Motor m;
            m.name     = QString("J%1").arg(n);
            m.jointId  = jid;
            m.velocity = jj->type == JointType::Prismatic ? 0.1f : 45.0f;
            int id = 0;
            edit([&]{ id = m_scene->addMotor(m); });
            m_selectedMotor = id;
            refresh();
            emit statusMessage("Actuator " + m.name + " added", true);
        });
    }
    menu.exec(m_addBtn->mapToGlobal(QPoint(0, m_addBtn->height())));
}

void ActuatorsPage::deleteActuator() {
    if (m_locked || m_selectedMotor < 0) return;
    edit([&]{ m_scene->removeMotor(m_selectedMotor); });
    m_selectedMotor = -1;
    refresh();
}

// ── Poses ────────────────────────────────────────────────────────────────────

int ActuatorsPage::currentPoseId() const {
    auto* it = m_poses->currentItem();
    return it ? it->data(Qt::UserRole).toInt() : -1;
}

void ActuatorsPage::capturePose() {
    Pose p;
    int n = int(m_scene->allPoses().size()) + 1;
    p.name = QString("Pose%1").arg(n);
    for (auto& [jid, j] : m_scene->allJoints()) p.jointValues[jid] = j.currentValue;
    int id = 0;
    edit([&]{ id = m_scene->addPose(p); });
    rebuildPoses();
    for (int i = 0; i < m_poses->count(); ++i)
        if (m_poses->item(i)->data(Qt::UserRole).toInt() == id) {
            m_poses->setCurrentRow(i);
            m_poses->editItem(m_poses->item(i));
        }
    emit statusMessage("Pose captured — double-click to rename", true);
}

void ActuatorsPage::updatePose() {
    Pose* p = m_scene->getPose(currentPoseId());
    if (!p) return;
    edit([&]{
        p->jointValues.clear();
        for (auto& [jid, j] : m_scene->allJoints()) p->jointValues[jid] = j.currentValue;
        m_scene->notifyAutomationChanged();
    });
    rebuildPoses();
    emit statusMessage("Pose " + p->name + " updated", true);
}

void ActuatorsPage::goToPose() {
    const Pose* p = m_scene->getPose(currentPoseId());
    if (!p) return;
    for (auto& [jid, v] : p->jointValues)
        if (Joint* j = m_scene->getJoint(jid)) j->currentValue = v;
    m_scene->notifyJointsMoved();
    rebuildJog();
}

void ActuatorsPage::deletePose() {
    int id = currentPoseId();
    if (id < 0) return;
    edit([&]{ m_scene->removePose(id); });
    rebuildPoses();
}
