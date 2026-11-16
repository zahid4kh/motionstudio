#include "SequencesPage.h"
#include "GanttView.h"
#include "automation/Expression.h"
#include "io/ApiServer.h"
#include "physics/SimulationEngine.h"
#include "robot/Gait.h"
#include "ui/Theme.h"
#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QTableWidget>
#include <QToolButton>
#include <QVBoxLayout>
#include <algorithm>

namespace {
enum Col { C_Num, C_Name, C_Type, C_What, C_After, C_When, C_State, C_Count };

const OperationType kTypes[] = {
    OperationType::Move, OperationType::MoveToPose, OperationType::MoveL, OperationType::Walk, OperationType::Run,
    OperationType::Stop,
    OperationType::Set, OperationType::Wait, OperationType::WaitFor,
    OperationType::Grip, OperationType::Release,
};

QString typeHelp(OperationType t) {
    switch (t) {
        case OperationType::Move:       return "Move an actuator to a target position; finishes when it arrives";
        case OperationType::MoveToPose: return "Move all actuated joints to a pose together (synchronized); finishes when all arrive";
        case OperationType::Run:        return "Switch an actuator to constant speed; finishes immediately";
        case OperationType::Stop:       return "Bring an actuator to rest; finishes when it has stopped";
        case OperationType::Set:        return "Only run the 'On start' actions (set signals); finishes immediately";
        case OperationType::Wait:       return "Wait for a time; finishes after the duration";
        case OperationType::WaitFor:    return "Wait until a condition becomes true";
        case OperationType::MoveL:      return "Move a tool in a straight line to where it is in a pose; the joints "
                                               "follow by inverse kinematics. Finishes when it arrives";
        case OperationType::Grip:       return "Attach a part to a gripper so it moves with it: the part a sensor "
                                               "detects, or a fixed object; finishes when the part is held";
        case OperationType::Release:    return "Let go of everything the gripper holds (a rigid body then falls); "
                                               "finishes immediately";
        case OperationType::Walk:       return "A humanoid on legs walks: ahead, sideways and turning by the amounts "
                                               "given, or to an object's place, facing its way. It crouches, steps "
                                               "and stands up; finishes when it stands";
    }
    return {};
}

QIcon typeIcon(OperationType t) {
    switch (t) {
        case OperationType::Move:       return Theme::icon("move-right");
        case OperationType::MoveToPose: return Theme::icon("navigation");
        case OperationType::Run:        return Theme::icon("fast-forward");
        case OperationType::Stop:       return Theme::icon("circle-stop");
        case OperationType::Set:        return Theme::icon("equal");
        case OperationType::Wait:       return Theme::icon("hourglass");
        case OperationType::WaitFor:    return Theme::icon("eye");
        case OperationType::Grip:       return Theme::icon("hand-grab");
        case OperationType::MoveL:      return Theme::icon("move-diagonal-2");
        case OperationType::Release:    return Theme::icon("hand");
        case OperationType::Walk:       return Theme::icon("footprints");
    }
    return {};
}

// Objects that can walk: robots with a left and a right foot
std::vector<int> walkers(SceneGraph& scene) {
    std::vector<int> out;
    for (const ApiServer::Robot& r : ApiServer::findRobots(scene))
        if (gait::canWalk(scene, r.root)) out.push_back(r.root);
    return out;
}

QString opDisplayName(const Operation& op, int index) {
    return op.name.isEmpty() ? QString("#%1").arg(index + 1) : op.name;
}
} // namespace

SequencesPage::SequencesPage(SceneGraph* scene, SimulationEngine* sim, QWidget* parent)
    : Page(scene, sim, parent)
{
    auto* root = new QHBoxLayout(this);
    root->setContentsMargins(6, 6, 6, 6);
    auto* split = new QSplitter(Qt::Horizontal);
    root->addWidget(split);

    // ── Left: sequences ─────────────────────────────────────────────────────
    auto* left = new QWidget;
    auto* lv = new QVBoxLayout(left);
    lv->setContentsMargins(0, 0, 0, 0);
    lv->setSpacing(4);
    auto* lbar = new QHBoxLayout;
    auto* lt = new QLabel("SEQUENCES");
    lt->setObjectName("subheading");
    m_seqAdd = new QPushButton(Theme::icon("plus"), "");
    m_seqDel = new QPushButton(Theme::icon("minus", Theme::Tint::Danger), "");
    m_seqAdd->setObjectName("iconButton");
    m_seqDel->setObjectName("iconButton");
    m_seqAdd->setToolTip("New sequence");
    m_seqDel->setToolTip("Delete sequence");
    m_seqDel->setProperty("danger", true);
    m_seqAdd->setFixedWidth(30);
    m_seqDel->setFixedWidth(30);
    lbar->addWidget(lt);
    lbar->addStretch();
    lbar->addWidget(m_seqAdd);
    lbar->addWidget(m_seqDel);
    lv->addLayout(lbar);
    m_seqList = new QListWidget;
    m_seqList->setMaximumHeight(110);
    lv->addWidget(m_seqList);

    m_seqForm = new QGroupBox("Sequence");
    auto* sf = new QFormLayout(m_seqForm);
    sf->setContentsMargins(6, 12, 6, 6);
    sf->setVerticalSpacing(4);
    m_seqName = new QLineEdit;
    m_seqName->setToolTip("Identifier; publishes <name>.running, <name>.done, <name>.cycles");
    m_seqEnable = autoui::makeExprEdit(scene, autoui::ExprKind::Value, "always");
    m_seqEnable->setHelpText("While false, no new operations start (running ones finish). E.g. Auto && !Stop");
    m_seqStart = autoui::makeExprEdit(scene, autoui::ExprKind::Value, "immediately");
    m_seqStart->setHelpText("Gate for starting each cycle. E.g. rising(Start)  or  PartPresent");
    m_seqLoop = new QCheckBox("Loop (start a new cycle after each one completes)");
    m_seqState = new QLabel;
    m_seqState->setWordWrap(true);
    m_seqState->setObjectName("hint");
    sf->addRow("Name", m_seqName);
    sf->addRow("Enabled when", m_seqEnable);
    sf->addRow("Start when", m_seqStart);
    sf->addRow(m_seqLoop);
    sf->addRow(m_seqState);
    lv->addWidget(m_seqForm);
    lv->addStretch();
    split->addWidget(left);

    // ── Middle: operations ──────────────────────────────────────────────────
    auto* mid = new QWidget;
    auto* mv = new QVBoxLayout(mid);
    mv->setContentsMargins(0, 0, 0, 0);
    mv->setSpacing(4);
    auto* obar = new QHBoxLayout;
    m_opAdd = new QToolButton;
    m_opAdd->setText("Operation");
    m_opAdd->setIcon(Theme::icon("plus"));
    m_opAdd->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    m_opAdd->setObjectName("menuButton");
    m_opAdd->setPopupMode(QToolButton::InstantPopup);
    auto* addMenu = new QMenu(m_opAdd);
    for (OperationType t : kTypes) {
        QAction* a = addMenu->addAction(typeIcon(t), operationTypeName(t));
        a->setToolTip(typeHelp(t));
        connect(a, &QAction::triggered, this, [this, t]{ addOperation(t); });
    }
    addMenu->setToolTipsVisible(true);
    m_opAdd->setMenu(addMenu);
    m_opDup  = new QPushButton(Theme::icon("copy"), "Duplicate");
    m_opUp   = new QPushButton(Theme::icon("arrow-up"), "");
    m_opDown = new QPushButton(Theme::icon("arrow-down"), "");
    m_opDel  = new QPushButton(Theme::icon("trash-2", Theme::Tint::Danger), "Delete");
    m_opUp->setObjectName("iconButton");
    m_opDown->setObjectName("iconButton");
    m_opUp->setToolTip("Move operation up");
    m_opDown->setToolTip("Move operation down");
    m_opDel->setProperty("danger", true);
    m_opUp->setFixedWidth(30);
    m_opDown->setFixedWidth(30);
    obar->addWidget(m_opAdd);
    obar->addWidget(m_opDup);
    obar->addWidget(m_opUp);
    obar->addWidget(m_opDown);
    obar->addStretch();
    obar->addWidget(m_opDel);
    mv->addLayout(obar);

    auto* vsplit = new QSplitter(Qt::Vertical);
    m_ops = new QTableWidget(0, C_Count);
    m_ops->setHorizontalHeaderLabels({"#", "Name", "Type", "What", "After", "Start when", "State"});
    m_ops->verticalHeader()->hide();
    m_ops->setAlternatingRowColors(true);
    m_ops->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_ops->setSelectionMode(QAbstractItemView::SingleSelection);
    m_ops->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_ops->verticalHeader()->setDefaultSectionSize(22);
    m_ops->setColumnWidth(C_Num, 28);
    m_ops->setColumnWidth(C_Name, 110);
    m_ops->setColumnWidth(C_Type, 124);
    m_ops->setColumnWidth(C_What, 170);
    m_ops->setColumnWidth(C_After, 60);
    m_ops->setColumnWidth(C_When, 110);
    m_ops->horizontalHeader()->setSectionResizeMode(C_State, QHeaderView::Stretch);
    vsplit->addWidget(m_ops);
    auto* ganttScroll = new QScrollArea;
    ganttScroll->setWidgetResizable(true);
    ganttScroll->setFrameShape(QFrame::StyledPanel);
    m_gantt = new GanttView;
    ganttScroll->setWidget(m_gantt);
    vsplit->addWidget(ganttScroll);
    vsplit->setStretchFactor(0, 3);
    vsplit->setStretchFactor(1, 2);
    mv->addWidget(vsplit, 1);
    split->addWidget(mid);

    // ── Right: operation editor ─────────────────────────────────────────────
    m_opForm = new QGroupBox("Operation");
    // The wrapping status text sits below the form, not in it: a
    // height-for-width row makes QFormLayout (Qt 6.4) squeeze a shown row
    // next to hidden ones
    auto* opBox = new QVBoxLayout(m_opForm);
    opBox->setContentsMargins(6, 12, 6, 6);
    opBox->setSpacing(6);
    m_opLayout = new QFormLayout;
    opBox->addLayout(m_opLayout);
    m_opLayout->setVerticalSpacing(4);
    m_opLayout->setLabelAlignment(Qt::AlignRight);
    m_opName = new QLineEdit;
    m_opName->setPlaceholderText("optional");
    m_opType = new QComboBox;
    for (OperationType t : kTypes) m_opType->addItem(typeIcon(t), operationTypeName(t));
    m_opActuator = new QComboBox;
    m_opPose = new QComboBox;
    m_opGripper = new QComboBox;
    m_opGripper->setToolTip("The object that holds the part, e.g. the robot's tool flange or gripper body");
    m_opPart = new QComboBox;
    m_opOnPart = new QComboBox;
    m_opOnPart->setToolTip("The pose was taught on a part: go to it relative to this part. A sensor finds it at "
                           "that moment (the operation waits until it finds something; copies of the taught part "
                           "count), or a fixed object");
    // Long item names (sensors, objects deep in the tree) must not widen the form
    for (QComboBox* c : {m_opPart, m_opOnPart}) {
        c->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        c->setMinimumContentsLength(12);
    }
    m_opPart->setToolTip("What to pick up: whatever a sensor detects at that moment (waits until it detects "
                         "something), or always the same object");
    m_opTarget    = autoui::makeExprEdit(scene, autoui::ExprKind::Value, "e.g. 90  or  Pick ? -45 : 0");
    m_opVelocity  = autoui::makeExprEdit(scene, autoui::ExprKind::Value, "actuator default");
    m_opDuration  = autoui::makeExprEdit(scene, autoui::ExprKind::Value, "seconds, e.g. 0.5");
    m_opCondition = autoui::makeExprEdit(scene, autoui::ExprKind::Value, "e.g. PartPresent && !EStop");
    m_opWalker = new QComboBox;
    m_opWalker->setToolTip("The humanoid that walks: a robot with a Left Foot and a Right Foot (Tools › Humanoid "
                           "Builder, built on legs)");
    m_opWalkTo = new QComboBox;
    m_opWalkTo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    m_opWalkTo->setMinimumContentsLength(12);
    m_opWalkTo->setToolTip("Walk to where this object stands on the floor and face the way it faces (its +X). "
                           "Add an empty frame as a marker and place it. Or walk by the distances below");
    m_opForward  = autoui::makeExprEdit(scene, autoui::ExprKind::Value, "metres ahead, e.g. 1.5");
    m_opSideways = autoui::makeExprEdit(scene, autoui::ExprKind::Value, "metres to its left (− right)");
    m_opTurn     = autoui::makeExprEdit(scene, autoui::ExprKind::Value, "degrees to its left (− right)");
    m_opForward->setHelpText("Metres ahead; negative walks backwards. A turn given with it makes the walk an arc");
    m_opSideways->setHelpText("Metres to its left (its −Z); negative steps to its right");
    m_opTurn->setHelpText("Degrees to its left (counter-clockwise seen from above); negative turns right");
    m_opAfter = new QToolButton;
    m_opAfter->setPopupMode(QToolButton::InstantPopup);
    m_opAfter->setObjectName("menuButton");
    m_opAfter->setToolTip("Operations that must be finished before this one starts.\n"
                          "Several operations after the same predecessor run in parallel.");
    m_opAfter->setMinimumWidth(120);
    m_opStartCond = autoui::makeExprEdit(scene, autoui::ExprKind::Value, "immediately");
    m_opStartCond->setHelpText("Extra condition checked after the predecessors are done");
    m_opOnStart  = autoui::makeExprEdit(scene, autoui::ExprKind::Assignments, "e.g. Gripper = true");
    m_opOnFinish = autoui::makeExprEdit(scene, autoui::ExprKind::Assignments, "e.g. Cycles = Cycles + 1");
    m_opOnStart->setHelpText("Signal assignments run when the operation starts; separate several with ';'");
    m_opOnFinish->setHelpText("Signal assignments run when the operation finishes; separate several with ';'");
    m_opVelLabel = new QLabel("Velocity");
    m_opStatus = new QLabel;
    m_opStatus->setWordWrap(true);
    m_opStatus->setObjectName("hint");
    m_opLayout->addRow("Name", m_opName);
    m_opLayout->addRow("Type", m_opType);
    m_opLayout->addRow("Actuator", m_opActuator);
    m_opLayout->addRow("Pose", m_opPose);
    m_opLayout->addRow("On part", m_opOnPart);
    m_opLayout->addRow("Gripper", m_opGripper);
    m_opLayout->addRow("Part", m_opPart);
    m_opLayout->addRow("Robot", m_opWalker);
    m_opLayout->addRow("Walk to", m_opWalkTo);
    m_opLayout->addRow("Forward m", m_opForward);
    m_opLayout->addRow("Sideways m", m_opSideways);
    m_opLayout->addRow("Turn °", m_opTurn);
    m_opLayout->addRow("Target", m_opTarget);
    m_opLayout->addRow(m_opVelLabel, m_opVelocity);
    m_opLayout->addRow("Duration", m_opDuration);
    m_opLayout->addRow("Until", m_opCondition);
    m_opLayout->addRow("After", m_opAfter);
    m_opLayout->addRow("Start when", m_opStartCond);
    m_opLayout->addRow("On start", m_opOnStart);
    m_opLayout->addRow("On finish", m_opOnFinish);
    opBox->addWidget(m_opStatus);
    opBox->addStretch();
    auto* formScroll = new QScrollArea;
    formScroll->setWidgetResizable(true);
    formScroll->setFrameShape(QFrame::NoFrame);
    formScroll->setWidget(m_opForm);
    split->addWidget(formScroll);

    split->setStretchFactor(0, 2);
    split->setStretchFactor(1, 4);
    split->setStretchFactor(2, 3);

    // ── Wiring ──────────────────────────────────────────────────────────────
    connect(m_seqAdd, &QPushButton::clicked, this, &SequencesPage::addSequence);
    connect(m_seqDel, &QPushButton::clicked, this, &SequencesPage::deleteSequence);
    connect(m_seqList, &QListWidget::currentRowChanged, this, [this](int row){
        if (m_loading || row < 0) return;
        m_seqId = m_seqList->item(row)->data(Qt::UserRole).toInt();
        m_opId = -1;
        loadSequenceForm();
        rebuildOpTable();
        loadOpForm();
    });
    connect(m_seqName, &QLineEdit::editingFinished, this, &SequencesPage::renameSequence);
    connect(m_seqEnable, &ExpressionEdit::committed, this, &SequencesPage::commitSequenceForm);
    connect(m_seqStart,  &ExpressionEdit::committed, this, &SequencesPage::commitSequenceForm);
    connect(m_seqLoop,   &QCheckBox::toggled,        this, &SequencesPage::commitSequenceForm);

    connect(m_opDup,  &QPushButton::clicked, this, &SequencesPage::duplicateOperation);
    connect(m_opDel,  &QPushButton::clicked, this, &SequencesPage::deleteOperation);
    connect(m_opUp,   &QPushButton::clicked, this, [this]{ moveOperation(-1); });
    connect(m_opDown, &QPushButton::clicked, this, [this]{ moveOperation(+1); });
    connect(m_ops, &QTableWidget::currentCellChanged, this, [this](int row, int, int, int){
        if (m_loading || row < 0) return;
        Sequence* s = currentSequence();
        if (!s || row >= int(s->operations.size())) return;
        m_opId = s->operations[size_t(row)].id;
        m_gantt->setSelectedRow(row);
        loadOpForm();
    });
    connect(m_gantt, &GanttView::rowClicked, this, [this](int row){ m_ops->selectRow(row); });

    connect(m_opName, &QLineEdit::editingFinished, this, &SequencesPage::commitOpForm);
    connect(m_opType, &QComboBox::currentIndexChanged, this, [this](int i){
        if (m_loading) return;
        updateOpFormVisibility(kTypes[i]);
        commitOpForm();
    });
    connect(m_opActuator, &QComboBox::currentIndexChanged, this, &SequencesPage::commitOpForm);
    connect(m_opPose,     &QComboBox::currentIndexChanged, this, [this]{
        if (m_loading) return;
        // A pose taught on a part brings its tool along
        const Pose* p = m_scene->getPose(m_opPose->currentData().toInt());
        if (p && p->isRelative() && kTypes[std::max(0, m_opType->currentIndex())] == OperationType::MoveL &&
            m_opGripper->currentData().toInt() == -1) {
            const QSignalBlocker block(m_opGripper);
            m_opGripper->setCurrentIndex(std::max(0, m_opGripper->findData(p->toolId)));
        }
        updateOpFormVisibility(kTypes[std::max(0, m_opType->currentIndex())]);
        commitOpForm();
    });
    connect(m_opGripper,  &QComboBox::currentIndexChanged, this, &SequencesPage::commitOpForm);
    connect(m_opPart,     &QComboBox::currentIndexChanged, this, &SequencesPage::commitOpForm);
    connect(m_opOnPart,   &QComboBox::currentIndexChanged, this, &SequencesPage::commitOpForm);
    connect(m_opWalker,   &QComboBox::currentIndexChanged, this, &SequencesPage::commitOpForm);
    connect(m_opWalkTo,   &QComboBox::currentIndexChanged, this, [this]{
        if (m_loading) return;
        updateOpFormVisibility(kTypes[std::max(0, m_opType->currentIndex())]);
        commitOpForm();
    });
    for (auto* e : {m_opTarget, m_opVelocity, m_opDuration, m_opCondition, m_opStartCond, m_opOnStart, m_opOnFinish,
                    m_opForward, m_opSideways, m_opTurn})
        connect(e, &ExpressionEdit::committed, this, &SequencesPage::commitOpForm);

    refresh();
}

// ── Lookup ───────────────────────────────────────────────────────────────────

Sequence* SequencesPage::currentSequence() { return m_scene->getSequence(m_seqId); }

int SequencesPage::currentOpIndex() const {
    const Sequence* s = const_cast<SequencesPage*>(this)->m_scene->getSequence(m_seqId);
    if (!s) return -1;
    for (int i = 0; i < int(s->operations.size()); ++i)
        if (s->operations[size_t(i)].id == m_opId) return i;
    return -1;
}

Operation* SequencesPage::currentOperation() {
    Sequence* s = currentSequence();
    int i = currentOpIndex();
    return (s && i >= 0) ? &s->operations[size_t(i)] : nullptr;
}

// ── Build ────────────────────────────────────────────────────────────────────

void SequencesPage::refresh() {
    if (!m_scene->getSequence(m_seqId)) {
        m_seqId = m_scene->allSequences().empty() ? -1 : m_scene->allSequences().begin()->first;
        m_opId = -1;
    }
    rebuildSequenceList();
    loadSequenceForm();
    rebuildOpTable();
    loadOpForm();
}

void SequencesPage::rebuildSequenceList() {
    m_loading = true;
    m_seqList->clear();
    for (auto& [id, q] : m_scene->allSequences()) {
        auto* it = new QListWidgetItem(q.name + (q.loop ? "  ⟳" : ""));
        it->setData(Qt::UserRole, id);
        m_seqList->addItem(it);
        if (id == m_seqId) m_seqList->setCurrentItem(it);
    }
    m_seqAdd->setEnabled(!m_locked);
    m_seqDel->setEnabled(!m_locked && m_seqId >= 0);
    m_loading = false;
}

void SequencesPage::loadSequenceForm() {
    Sequence* s = currentSequence();
    m_seqForm->setEnabled(s && !m_locked);
    m_loading = true;
    m_seqName->setText(s ? s->name : QString());
    m_seqEnable->setExpression(s ? s->enableCondition : QString());
    m_seqStart->setExpression(s ? s->startCondition : QString());
    m_seqLoop->setChecked(s && s->loop);
    m_seqState->clear();
    m_loading = false;
    bool hasSeq = s != nullptr;
    m_opAdd->setEnabled(hasSeq && !m_locked);
}

QString SequencesPage::predecessorText(const Sequence& seq, const Operation& op) const {
    QStringList out;
    for (int p : op.predecessors)
        for (int i = 0; i < int(seq.operations.size()); ++i)
            if (seq.operations[size_t(i)].id == p) out << QString::number(i + 1);
    return out.isEmpty() ? QString("start") : out.join(", ");
}

QString SequencesPage::partText(const Operation& op) const {
    const Pose* p = m_scene->getPose(op.poseId);
    if (!p || !p->isRelative()) return {};
    if (op.sensorId != -1) {
        const Signal* sg = m_scene->getSignal(op.sensorId);
        return " on part at " + (sg ? sg->name : QString("?"));
    }
    const SceneObject* o = m_scene->getObject(op.partId != -1 ? op.partId : p->frameId);
    return " on " + (o ? o->name : QString("?"));
}

QString SequencesPage::describe(const Operation& op) const {
    const Motor* m = m_scene->getMotor(op.actuatorId);
    QString act = m ? m->name : QString("?");
    switch (op.type) {
        case OperationType::Move:
            return QString("%1 → %2%3").arg(act, op.target.trimmed().isEmpty() ? "?" : op.target.trimmed(),
                       op.velocity.trimmed().isEmpty() ? QString() : " @ " + op.velocity.trimmed());
        case OperationType::MoveToPose: {
            const Pose* p = m_scene->getPose(op.poseId);
            return QString("→ %1%2%3").arg(p ? p->name : "?", partText(op),
                       op.velocity.trimmed().isEmpty() ? QString() : QString(" @ %1 %").arg(op.velocity.trimmed()));
        }
        case OperationType::Run:     return QString("%1 at %2").arg(act, op.velocity.trimmed());
        case OperationType::Stop:    return "stop " + act;
        case OperationType::Set:     return op.onStart.trimmed();
        case OperationType::Wait:    return op.duration.trimmed() + " s";
        case OperationType::WaitFor: return op.condition.trimmed();
        case OperationType::Grip: {
            const SceneObject* g = m_scene->getObject(op.gripperId);
            QString what = "?";
            if (op.sensorId != -1) {
                const Signal* sg = m_scene->getSignal(op.sensorId);
                what = "part at " + (sg ? sg->name : QString("?"));
            } else if (const SceneObject* p = m_scene->getObject(op.partId)) {
                what = p->name;
            }
            return QString("%1 grips %2").arg(g ? g->name : "?", what);
        }
        case OperationType::Release: {
            const SceneObject* g = m_scene->getObject(op.gripperId);
            return QString("%1 releases").arg(g ? g->name : "?");
        }
        case OperationType::MoveL: {
            const Pose* p = m_scene->getPose(op.poseId);
            const SceneObject* t = m_scene->getObject(op.toolId != -1 || !p ? op.toolId : p->toolId);
            return QString("%1 → %2%3 linear%4").arg(t ? t->name : "?", p ? p->name : "?", partText(op),
                       op.velocity.trimmed().isEmpty() ? QString() : QString(" @ %1 m/s").arg(op.velocity.trimmed()));
        }
        case OperationType::Walk: {
            const SceneObject* w = m_scene->getObject(op.walkerId);
            const QString speed = op.velocity.trimmed().isEmpty() ? QString() : QString(" @ %1 m/s").arg(op.velocity.trimmed());
            if (const SceneObject* to = m_scene->getObject(op.walkToId))
                return QString("%1 walks to %2%3").arg(w ? w->name : "?", to->name, speed);
            QStringList parts;
            if (!op.forward.trimmed().isEmpty()) parts << op.forward.trimmed() + " m";
            if (!op.sideways.trimmed().isEmpty()) parts << op.sideways.trimmed() + " m left";
            if (!op.turn.trimmed().isEmpty()) parts << "turns " + op.turn.trimmed() + "°";
            return QString("%1 walks %2%3").arg(w ? w->name : "?", parts.isEmpty() ? QString("?") : parts.join(", "), speed);
        }
    }
    return {};
}

void SequencesPage::rebuildOpTable() {
    m_loading = true;
    m_ops->setRowCount(0);
    Sequence* s = currentSequence();
    int selRow = -1;
    if (s) {
        for (int i = 0; i < int(s->operations.size()); ++i) {
            const Operation& op = s->operations[size_t(i)];
            m_ops->insertRow(i);
            auto* num = new QTableWidgetItem(QString::number(i + 1));
            num->setTextAlignment(Qt::AlignCenter);
            m_ops->setItem(i, C_Num, num);
            m_ops->setItem(i, C_Name, new QTableWidgetItem(op.name));
            m_ops->setItem(i, C_Type, new QTableWidgetItem(typeIcon(op.type), operationTypeName(op.type)));
            m_ops->setItem(i, C_What, new QTableWidgetItem(describe(op)));
            m_ops->setItem(i, C_After, new QTableWidgetItem(predecessorText(*s, op)));
            m_ops->setItem(i, C_When, new QTableWidgetItem(op.startCondition.trimmed()));
            m_ops->setItem(i, C_State, new QTableWidgetItem(""));
            if (op.id == m_opId) selRow = i;
        }
        if (selRow < 0 && !s->operations.empty()) {
            selRow = 0;
            m_opId = s->operations[0].id;
        }
    }
    if (selRow >= 0) m_ops->selectRow(selRow);
    m_gantt->setSelectedRow(selRow);
    bool hasOp = selRow >= 0;
    m_opDup->setEnabled(hasOp && !m_locked);
    m_opDel->setEnabled(hasOp && !m_locked);
    m_opUp->setEnabled(hasOp && !m_locked && selRow > 0);
    m_opDown->setEnabled(hasOp && !m_locked && s && selRow < int(s->operations.size()) - 1);
    m_loading = false;
    refreshRuntime();
}

void SequencesPage::updateOpFormVisibility(OperationType t) {
    bool act  = t == OperationType::Move || t == OperationType::Run || t == OperationType::Stop;
    bool pose = t == OperationType::MoveToPose || t == OperationType::MoveL;
    m_opLayout->setRowVisible(m_opActuator, act);
    m_opLayout->setRowVisible(m_opPose, pose);
    m_opLayout->setRowVisible(m_opTarget, t == OperationType::Move);
    m_opLayout->setRowVisible(m_opVelocity, t == OperationType::Move || t == OperationType::Run || pose);
    m_opLayout->setRowVisible(m_opDuration, t == OperationType::Wait);
    m_opLayout->setRowVisible(m_opCondition, t == OperationType::WaitFor);
    m_opLayout->setRowVisible(m_opGripper, t == OperationType::Grip || t == OperationType::Release ||
                                           t == OperationType::MoveL);
    if (auto* l = qobject_cast<QLabel*>(m_opLayout->labelForField(m_opGripper)))
        l->setText(t == OperationType::MoveL ? "Tool" : "Gripper");
    m_opGripper->setToolTip(t == OperationType::MoveL
        ? "The object whose frame is the tool centre point; the actuated joints above it move it"
        : "The object that holds the part, e.g. the robot's tool flange or gripper body");
    m_opLayout->setRowVisible(m_opPart, t == OperationType::Grip);
    const bool walk = t == OperationType::Walk;
    const bool byDistance = walk && m_opWalkTo->currentData().toInt() == -1;
    m_opLayout->setRowVisible(m_opWalker, walk);
    m_opLayout->setRowVisible(m_opWalkTo, walk);
    m_opLayout->setRowVisible(m_opForward, byDistance);
    m_opLayout->setRowVisible(m_opSideways, byDistance);
    m_opLayout->setRowVisible(m_opTurn, byDistance);
    if (walk) m_opLayout->setRowVisible(m_opVelocity, true);
    // MoveL / MoveToPose to a pose taught on a part: which part this time
    const Pose* chosen = m_scene->getPose(m_opPose->currentData().toInt());
    const bool onPart = pose && chosen && chosen->isRelative();
    m_opLayout->setRowVisible(m_opOnPart, onPart);
    if (onPart) {
        const SceneObject* taught = m_scene->getObject(chosen->frameId);
        m_opOnPart->setItemText(0, QString("%1, where it was taught").arg(taught ? taught->name : QString("The part")));
    }
    // MoveL to such a pose may leave the tool to the pose
    const SceneObject* poseTool = onPart ? m_scene->getObject(chosen->toolId) : nullptr;
    m_opGripper->setItemText(0, t == OperationType::MoveL && poseTool
                                    ? QString("%1 (the pose's tool)").arg(poseTool->name) : QString("(choose)"));
    if (t == OperationType::Run) {
        m_opVelLabel->setText("Speed");
        m_opVelocity->setPlaceholderText("signed speed, e.g. 90 or -0.2");
    } else if (t == OperationType::MoveL) {
        m_opVelLabel->setText("Speed m/s");
        m_opVelocity->setPlaceholderText("0.25");
    } else if (walk) {
        m_opVelLabel->setText("Speed m/s");
        m_opVelocity->setPlaceholderText("from its legs");
    } else if (pose) {
        m_opVelLabel->setText("Speed %");
        m_opVelocity->setPlaceholderText("100");
    } else {
        m_opVelLabel->setText("Max speed");
        m_opVelocity->setPlaceholderText("actuator default");
    }
    m_opType->setToolTip(typeHelp(t));
}

void SequencesPage::rebuildPredecessorMenu() {
    Sequence* s = currentSequence();
    Operation* op = currentOperation();
    auto* menu = new QMenu(m_opAfter);
    if (s && op) {
        auto* none = menu->addAction("(sequence start)");
        none->setCheckable(true);
        none->setChecked(op->predecessors.empty());
        connect(none, &QAction::triggered, this, [this]{
            if (Operation* o = currentOperation()) {
                edit([&]{ o->predecessors.clear(); m_scene->notifyAutomationChanged(); });
                rebuildOpTable();
                loadOpForm();
            }
        });
        menu->addSeparator();
        for (int i = 0; i < int(s->operations.size()); ++i) {
            const Operation& other = s->operations[size_t(i)];
            if (other.id == op->id) continue;
            auto* a = menu->addAction(QString("%1  %2  —  %3").arg(i + 1)
                                          .arg(opDisplayName(other, i), describe(other)));
            a->setCheckable(true);
            a->setChecked(std::find(op->predecessors.begin(), op->predecessors.end(), other.id) != op->predecessors.end());
            const int otherId = other.id;
            connect(a, &QAction::toggled, this, [this, otherId](bool on){
                Operation* o = currentOperation();
                if (!o) return;
                edit([&]{
                    auto& p = o->predecessors;
                    p.erase(std::remove(p.begin(), p.end(), otherId), p.end());
                    if (on) p.push_back(otherId);
                    m_scene->notifyAutomationChanged();
                });
                rebuildOpTable();
                Sequence* sq = currentSequence();
                if (sq && o) m_opAfter->setText(predecessorText(*sq, *o));
            });
        }
    }
    if (QMenu* old = m_opAfter->menu()) old->deleteLater();
    m_opAfter->setMenu(menu);
}

void SequencesPage::loadOpForm() {
    Sequence* s = currentSequence();
    Operation* op = currentOperation();
    m_opForm->setEnabled(op && !m_locked);
    m_loading = true;

    // Actuator / pose choices
    m_opActuator->clear();
    m_opActuator->addItem("(choose)", -1);
    for (int mid : m_scene->motorIdsSorted())
        m_opActuator->addItem(m_scene->getMotor(mid)->name, mid);
    m_opPose->clear();
    m_opPose->addItem("(choose)", -1);
    for (auto& [pid, p] : m_scene->allPoses()) m_opPose->addItem(p.name, pid);
    autoui::fillObjectCombo(m_opGripper, *m_scene,
                            !op ? -1 : op->type == OperationType::MoveL ? op->toolId : op->gripperId, "(choose)");
    // Walk: robots with legs, and where to
    {
        const QSignalBlocker block(m_opWalker);
        m_opWalker->clear();
        m_opWalker->addItem("(choose)", -1);
        {
            for (int id : walkers(*m_scene))
                m_opWalker->addItem(Theme::icon("person-standing"), m_scene->getObject(id)->name, id);
            const int cur = op ? op->walkerId : -1;
            int idx = m_opWalker->findData(cur);
            if (idx < 0 && cur != -1) {
                const SceneObject* o = m_scene->getObject(cur);
                m_opWalker->addItem(o ? o->name + " (no legs)" : QString("(deleted object)"), cur);
                idx = m_opWalker->count() - 1;
            }
            m_opWalker->setCurrentIndex(std::max(0, idx));
        }
    }
    autoui::fillObjectCombo(m_opWalkTo, *m_scene, op ? op->walkToId : -1, "(by distance)");
    // Part: sensors first ("s<id>"), then objects ("o<id>")
    {
        const QSignalBlocker block(m_opPart);
        m_opPart->clear();
        m_opPart->addItem("(choose)", QString());
        for (auto& [sid, sg] : m_scene->allSignals())
            if (sg.kind == SignalKind::Sensor)
                m_opPart->addItem(Theme::icon("radio"), "Detected by " + sg.name, QString("s%1").arg(sid));
        QComboBox objects;
        autoui::fillObjectCombo(&objects, *m_scene, -1);
        for (int i = 0; i < objects.count(); ++i)
            m_opPart->addItem(objects.itemText(i), QString("o%1").arg(objects.itemData(i).toInt()));
        QString cur = !op ? QString() : op->sensorId != -1 ? QString("s%1").arg(op->sensorId)
                                      : op->partId != -1 ? QString("o%1").arg(op->partId) : QString();
        int idx = m_opPart->findData(cur);
        if (idx < 0 && !cur.isEmpty()) { m_opPart->addItem("(deleted)", cur); idx = m_opPart->count() - 1; }
        m_opPart->setCurrentIndex(std::max(0, idx));

        // The same choices for a pose taught on a part; the first is the taught part itself
        const QSignalBlocker block2(m_opOnPart);
        m_opOnPart->clear();
        for (int i = 0; i < m_opPart->count(); ++i)
            m_opOnPart->addItem(m_opPart->itemIcon(i), m_opPart->itemText(i), m_opPart->itemData(i));
        m_opOnPart->setCurrentIndex(std::max(0, idx));
    }

    if (!op) {
        m_opForm->setTitle("Operation");
        m_opName->clear();
        for (auto* e : {m_opTarget, m_opVelocity, m_opDuration, m_opCondition, m_opStartCond, m_opOnStart, m_opOnFinish,
                        m_opForward, m_opSideways, m_opTurn})
            e->setExpression({});
        m_opAfter->setText("");
        m_opStatus->setText(s ? "Add an operation with “+ Operation”." : "Create a sequence with “+”.");
        m_loading = false;
        return;
    }
    int idx = currentOpIndex();
    m_opForm->setTitle(QString("Operation %1").arg(idx + 1));
    m_opName->setText(op->name);
    m_opType->setCurrentIndex(int(std::find(std::begin(kTypes), std::end(kTypes), op->type) - std::begin(kTypes)));
    m_opActuator->setCurrentIndex(std::max(0, m_opActuator->findData(op->actuatorId)));
    m_opPose->setCurrentIndex(std::max(0, m_opPose->findData(op->poseId)));
    m_opTarget->setExpression(op->target);
    m_opVelocity->setExpression(op->velocity);
    m_opDuration->setExpression(op->duration);
    m_opCondition->setExpression(op->condition);
    m_opForward->setExpression(op->forward);
    m_opSideways->setExpression(op->sideways);
    m_opTurn->setExpression(op->turn);
    m_opStartCond->setExpression(op->startCondition);
    m_opOnStart->setExpression(op->onStart);
    m_opOnFinish->setExpression(op->onFinish);
    m_opAfter->setText(predecessorText(*s, *op));
    rebuildPredecessorMenu();
    updateOpFormVisibility(op->type);
    m_opStatus->setText(typeHelp(op->type));
    m_loading = false;
}

void SequencesPage::refreshRuntime() {
    Sequence* s = currentSequence();
    AutomationEngine* ae = m_sim->automation();
    bool live = ae->isActive();
    std::vector<GanttView::Bar> bars;
    if (s) {
        if (live) {
            auto ss = ae->seqStatus(s->id);
            static const char* names[] = {"idle", "running", "finished"};
            QString critical;
            if (!ss.criticalPath.empty()) {
                QStringList steps;
                for (int opId : ss.criticalPath)
                    for (int i = 0; i < int(s->operations.size()); ++i)
                        if (s->operations[size_t(i)].id == opId) steps << QString::number(i + 1);
                const double waiting = std::max(0.0, ss.lastCycleTime - ss.criticalBusy);
                critical = QString("<br><span style='color:%1'>Critical path</span> %2 — %3 s working, %4 s waiting")
                               .arg(Theme::css(Theme::colors().warning), steps.join(" → "))
                               .arg(ss.criticalBusy, 0, 'f', 2).arg(waiting, 0, 'f', 2);
            }
            m_seqState->setText(QString("State: <b>%1</b> — cycles: %2%3%4<br>%5")
                .arg(names[int(ss.state)]).arg(ss.cycles)
                .arg(ss.lastCycleTime > 0 ? QString(", last cycle <b>%1 s</b>").arg(ss.lastCycleTime, 0, 'f', 2) : QString())
                .arg(critical)
                .arg(ss.detail.toHtmlEscaped()));
        } else {
            m_seqState->setText("Press Play or Step to run.");
        }
        double now = live ? ae->time() - ae->seqStatus(s->id).cycleStart : 0;
        bool running = live && ae->seqStatus(s->id).state == AutomationEngine::SeqState::Running;
        for (int i = 0; i < int(s->operations.size()) && i < m_ops->rowCount(); ++i) {
            const Operation& op = s->operations[size_t(i)];
            GanttView::Bar b;
            b.label = QString("%1 %2").arg(i + 1).arg(op.name.isEmpty() ? describe(op) : op.name);
            auto* stateItem = m_ops->item(i, C_State);
            if (!live) {
                stateItem->setText("");
                stateItem->setBackground(QBrush());
                bars.push_back(b);
                continue;
            }
            auto st = ae->opStatus(s->id, op.id);
            QString txt;
            switch (st.state) {
                case AutomationEngine::OpState::Waiting:
                    txt = "waiting" + (st.detail.isEmpty() ? QString() : " — " + st.detail);
                    break;
                case AutomationEngine::OpState::Running:
                    txt = QString("running %1%").arg(int(st.progress * 100)) +
                          (st.detail.isEmpty() ? QString() : " — " + st.detail);
                    break;
                case AutomationEngine::OpState::Done:
                    txt = QString("done (%1 s)").arg(st.endTime - st.startTime, 0, 'f', 2);
                    break;
            }
            stateItem->setText(txt);
            stateItem->setBackground(autoui::opStateBrush(st.state));
            const auto& crit = ae->seqStatus(s->id).criticalPath;
            b.critical = std::find(crit.begin(), crit.end(), op.id) != crit.end();
            b.state = int(st.state);
            b.progress = st.progress;
            b.detail = st.detail;
            if (st.startTime >= 0) {
                b.start = st.startTime;
                b.end = st.state == AutomationEngine::OpState::Done ? st.endTime : -1;
            } else if (st.prevStart >= 0) {
                b.start = st.prevStart;
                b.end = st.prevEnd;
                b.fromPreviousCycle = true;
            }
            bars.push_back(b);
        }
        m_gantt->setBars(std::move(bars), now, running);
    } else {
        m_seqState->clear();
        m_gantt->setBars({}, 0, false);
    }
}

void SequencesPage::setLocked(bool locked) {
    Page::setLocked(locked);
    refresh();
}

void SequencesPage::selectSequence(int sequenceId) {
    m_seqId = sequenceId;
    m_opId = -1;
    refresh();
}

// ── Sequence edits ───────────────────────────────────────────────────────────

void SequencesPage::addSequence() {
    Sequence q;
    q.name = m_scene->makeSymbolName("Sequence" + QString::number(m_scene->allSequences().size() + 1));
    int id = 0;
    edit([&]{ id = m_scene->addSequence(q); });
    m_seqId = id;
    m_opId = -1;
    refresh();
    m_seqName->setFocus();
    m_seqName->selectAll();
}

void SequencesPage::deleteSequence() {
    if (m_seqId < 0) return;
    edit([&]{ m_scene->removeSequence(m_seqId); });
    m_seqId = -1;
    refresh();
}

void SequencesPage::commitSequenceForm() {
    if (m_loading || m_locked) return;
    Sequence* s = currentSequence();
    if (!s) return;
    edit([&]{
        s->enableCondition = m_seqEnable->text();
        s->startCondition  = m_seqStart->text();
        s->loop            = m_seqLoop->isChecked();
        m_scene->notifyAutomationChanged();
    });
    rebuildSequenceList();
}

void SequencesPage::renameSequence() {
    if (m_loading || m_locked) return;
    Sequence* s = currentSequence();
    if (!s) return;
    QString n = m_seqName->text().trimmed();
    if (n == s->name) return;
    if (!expr::isValidName(n) || m_scene->isSymbolNameTaken(n, s->id)) {
        emit statusMessage(expr::isValidName(n) ? "The name '" + n + "' is already used"
                                                : "'" + n + "' is not a valid name — use letters, digits and _", false);
        m_seqName->setText(s->name);
        return;
    }
    QString old = s->name;
    edit([&]{
        s->name = n;
        m_scene->renameSymbolReferences(old, n);
    });
    rebuildSequenceList();
}

// ── Operation edits ──────────────────────────────────────────────────────────

void SequencesPage::addOperation(OperationType t) {
    Sequence* s = currentSequence();
    if (!s || m_locked) return;
    Operation op;
    op.id   = m_scene->allocateId();
    op.type = t;
    // Default: run after the currently selected operation (or the last one)
    int after = currentOpIndex();
    if (after < 0 && !s->operations.empty()) after = int(s->operations.size()) - 1;
    if (after >= 0) op.predecessors.push_back(s->operations[size_t(after)].id);
    switch (t) {
        case OperationType::Move:
        case OperationType::Run:
        case OperationType::Stop: {
            auto mids = m_scene->motorIdsSorted();
            if (!mids.empty()) op.actuatorId = mids.front();
            if (t == OperationType::Move) op.target = "0";
            if (t == OperationType::Run)  op.velocity = "45";
            break;
        }
        case OperationType::MoveToPose:
            if (!m_scene->allPoses().empty()) op.poseId = m_scene->allPoses().begin()->first;
            break;
        case OperationType::MoveL:
            if (!m_scene->allPoses().empty()) op.poseId = m_scene->allPoses().begin()->first;
            op.toolId = m_scene->selectedObjectId();
            break;
        case OperationType::Walk: {
            const std::vector<int> w = walkers(*m_scene);
            if (!w.empty()) op.walkerId = w.front();
            op.forward = "1";
            break;
        }
        case OperationType::Wait:    op.duration = "1"; break;
        case OperationType::WaitFor: op.condition = "true"; break;
        case OperationType::Set:     break;
        case OperationType::Grip:
        case OperationType::Release:
            op.gripperId = m_scene->selectedObjectId();
            if (t == OperationType::Grip)
                for (auto& [sid, sg] : m_scene->allSignals())
                    if (sg.kind == SignalKind::Sensor) { op.sensorId = sid; break; }
            break;
    }
    int insertAt = after >= 0 ? after + 1 : int(s->operations.size());
    edit([&]{
        // Operations that followed `after` now follow the new one, so inserting
        // in the middle of a chain keeps the chain intact.
        if (after >= 0) {
            int afterId = s->operations[size_t(after)].id;
            for (auto& o : s->operations) {
                if (o.predecessors.size() == 1 && o.predecessors[0] == afterId)
                    o.predecessors[0] = op.id;
            }
        }
        s->operations.insert(s->operations.begin() + insertAt, op);
        m_scene->notifyAutomationChanged();
    });
    m_opId = op.id;
    rebuildOpTable();
    loadOpForm();
}

void SequencesPage::duplicateOperation() {
    Sequence* s = currentSequence();
    int i = currentOpIndex();
    if (!s || i < 0 || m_locked) return;
    Operation copy = s->operations[size_t(i)];
    copy.id = m_scene->allocateId();
    if (!copy.name.isEmpty()) copy.name += " copy";
    edit([&]{
        s->operations.insert(s->operations.begin() + i + 1, copy);
        m_scene->notifyAutomationChanged();
    });
    m_opId = copy.id;
    rebuildOpTable();
    loadOpForm();
}

void SequencesPage::deleteOperation() {
    Sequence* s = currentSequence();
    int i = currentOpIndex();
    if (!s || i < 0 || m_locked) return;
    Operation removed = s->operations[size_t(i)];
    edit([&]{
        s->operations.erase(s->operations.begin() + i);
        // Successors inherit the deleted operation's predecessors
        for (auto& o : s->operations) {
            auto it = std::find(o.predecessors.begin(), o.predecessors.end(), removed.id);
            if (it == o.predecessors.end()) continue;
            o.predecessors.erase(it);
            for (int p : removed.predecessors)
                if (std::find(o.predecessors.begin(), o.predecessors.end(), p) == o.predecessors.end())
                    o.predecessors.push_back(p);
        }
        m_scene->notifyAutomationChanged();
    });
    m_opId = s->operations.empty() ? -1 : s->operations[size_t(std::min(i, int(s->operations.size()) - 1))].id;
    rebuildOpTable();
    loadOpForm();
}

void SequencesPage::moveOperation(int delta) {
    Sequence* s = currentSequence();
    int i = currentOpIndex();
    int j = i + delta;
    if (!s || i < 0 || j < 0 || j >= int(s->operations.size()) || m_locked) return;
    edit([&]{
        std::swap(s->operations[size_t(i)], s->operations[size_t(j)]);
        m_scene->notifyAutomationChanged();
    });
    rebuildOpTable();
    loadOpForm();
}

void SequencesPage::commitOpForm() {
    if (m_loading || m_locked) return;
    Operation* op = currentOperation();
    if (!op) return;
    edit([&]{
        op->name           = m_opName->text().trimmed();
        op->type           = kTypes[std::max(0, m_opType->currentIndex())];
        op->actuatorId     = m_opActuator->currentData().toInt();
        op->poseId         = m_opPose->currentData().toInt();
        const int object   = m_opGripper->currentData().toInt();
        op->gripperId      = op->type == OperationType::MoveL ? -1 : object;
        op->toolId         = op->type == OperationType::MoveL ? object : -1;
        const bool onPart  = op->type == OperationType::MoveL || op->type == OperationType::MoveToPose;
        const QString part = (onPart ? m_opOnPart : m_opPart)->currentData().toString();
        op->sensorId       = part.startsWith('s') ? part.mid(1).toInt() : -1;
        op->partId         = part.startsWith('o') ? part.mid(1).toInt() : -1;
        op->target         = m_opTarget->text();
        op->velocity       = m_opVelocity->text();
        op->duration       = m_opDuration->text();
        op->condition      = m_opCondition->text();
        op->startCondition = m_opStartCond->text();
        op->onStart        = m_opOnStart->text();
        op->onFinish       = m_opOnFinish->text();
        const bool walk    = op->type == OperationType::Walk;
        // Switched to Walk: the first robot with legs
        if (walk && m_opWalker->currentData().toInt() == -1 && m_opWalker->count() > 1) {
            const QSignalBlocker block(m_opWalker);
            m_opWalker->setCurrentIndex(1);
        }
        op->walkerId       = walk ? m_opWalker->currentData().toInt() : -1;
        op->walkToId       = walk ? m_opWalkTo->currentData().toInt() : -1;
        op->forward        = walk ? m_opForward->text() : QString();
        op->sideways       = walk ? m_opSideways->text() : QString();
        op->turn           = walk ? m_opTurn->text() : QString();
        m_scene->notifyAutomationChanged();
    });
    // Update the row in place so the form keeps focus
    Sequence* s = currentSequence();
    int i = currentOpIndex();
    if (s && i >= 0 && i < m_ops->rowCount()) {
        m_loading = true;
        m_ops->item(i, C_Name)->setText(op->name);
        m_ops->item(i, C_Type)->setText(operationTypeName(op->type));
        m_ops->item(i, C_Type)->setIcon(typeIcon(op->type));
        m_ops->item(i, C_What)->setText(describe(*op));
        m_ops->item(i, C_When)->setText(op->startCondition.trimmed());
        m_loading = false;
    }
    m_opStatus->setText(typeHelp(op->type));
}
