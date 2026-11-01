#include "SignalsPage.h"
#include "SensorDialog.h"
#include "automation/Sensors.h"
#include "automation/Expression.h"
#include "physics/SimulationEngine.h"
#include "ui/Theme.h"
#include <QComboBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>

namespace {
enum Col { C_Name, C_Kind, C_Type, C_Initial, C_Expr, C_Desc, C_Value, C_Count };

bool parseValue(const QString& text, double& out) {
    QString t = text.trimmed().toLower();
    if (t == "true" || t == "on")  { out = 1; return true; }
    if (t == "false" || t == "off") { out = 0; return true; }
    bool ok = false;
    out = t.toDouble(&ok);
    return ok;
}
} // namespace

SignalsPage::SignalsPage(SceneGraph* scene, SimulationEngine* sim, QWidget* parent)
    : Page(scene, sim, parent)
{
    auto* vb = new QVBoxLayout(this);
    vb->setContentsMargins(6, 6, 6, 6);
    vb->setSpacing(4);

    auto* bar = new QHBoxLayout;
    m_addInput    = new QPushButton(Theme::icon("log-in"),   "New Input");
    m_addInternal = new QPushButton(Theme::icon("variable"), "New Internal");
    m_addDerived  = new QPushButton(Theme::icon("sigma"),    "New Derived");
    m_addSensor   = new QPushButton(Theme::icon("radio"),    "New Sensor");
    m_delete      = new QPushButton(Theme::icon("trash-2", Theme::Tint::Danger), "Delete");
    m_delete->setProperty("danger", true);
    m_addInput->setToolTip("A signal you set by hand from the Runtime tab (buttons, switches, set-points)");
    m_addInternal->setToolTip("A state/flag written by operation actions (e.g. Gripper = true; Count = Count + 1)");
    m_addDerived->setToolTip("A signal recomputed every scan from an expression (e.g. Ready = J1.inPosition && !EStop)");
    m_addSensor->setToolTip("A Bool that is true while something is in a beam or zone in the scene "
                            "(part present, light barrier, contact). Mount it on the selected object.");
    for (auto* b : {m_addInput, m_addInternal, m_addDerived, m_addSensor}) bar->addWidget(b);
    bar->addStretch();
    auto* hint = new QLabel("Double-click a cell to edit. Names are used in expressions.");
    hint->setObjectName("hint");
    bar->addWidget(hint);
    bar->addWidget(m_delete);
    vb->addLayout(bar);

    m_table = new QTableWidget(0, C_Count);
    m_table->setHorizontalHeaderLabels({"Name", "Kind", "Type", "Initial", "Expression / Sensor", "Description", "Value"});
    m_table->verticalHeader()->hide();
    m_table->setAlternatingRowColors(true);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed |
                             QAbstractItemView::AnyKeyPressed);
    auto* hh = m_table->horizontalHeader();
    hh->setSectionResizeMode(QHeaderView::Interactive);
    hh->setStretchLastSection(false);
    m_table->setColumnWidth(C_Name, 140);
    m_table->setColumnWidth(C_Kind, 90);
    m_table->setColumnWidth(C_Type, 70);
    m_table->setColumnWidth(C_Initial, 70);
    m_table->setColumnWidth(C_Expr, 300);
    m_table->setColumnWidth(C_Desc, 180);
    m_table->setColumnWidth(C_Value, 80);
    hh->setSectionResizeMode(C_Expr, QHeaderView::Stretch);
    m_table->verticalHeader()->setDefaultSectionSize(28);
    vb->addWidget(m_table);

    connect(m_addInput,    &QPushButton::clicked, this, [this]{ addSignal(SignalKind::Input); });
    connect(m_addInternal, &QPushButton::clicked, this, [this]{ addSignal(SignalKind::Internal); });
    connect(m_addDerived,  &QPushButton::clicked, this, [this]{ addSignal(SignalKind::Derived); });
    connect(m_addSensor,   &QPushButton::clicked, this, [this]{ addSensorOn(m_scene->selectedObjectId()); });
    connect(m_delete,      &QPushButton::clicked, this, &SignalsPage::deleteSelected);
    connect(m_table, &QTableWidget::cellChanged, this, &SignalsPage::onItemChanged);

    refresh();
}

int SignalsPage::signalIdAt(int row) const {
    auto* it = m_table->item(row, C_Name);
    return it ? it->data(Qt::UserRole).toInt() : -1;
}

void SignalsPage::refresh() {
    int keepRow = m_table->currentRow();
    m_building = true;
    m_table->setRowCount(0);
    int row = 0;
    for (auto& [id, sg] : m_scene->allSignals()) {
        m_table->insertRow(row);
        const int sid = id;

        auto* name = new QTableWidgetItem(sg.name);
        name->setData(Qt::UserRole, id);
        QFont f = name->font(); f.setBold(true); name->setFont(f);
        m_table->setItem(row, C_Name, name);

        auto* kind = new QComboBox;
        kind->addItems({"Input", "Internal", "Derived", "Sensor"});
        kind->setCurrentIndex(int(sg.kind));
        kind->setEnabled(!m_locked);
        connect(kind, &QComboBox::currentIndexChanged, this, [this, sid](int k){
            Signal* s = m_scene->getSignal(sid);
            if (!s) return;
            edit([&]{
                s->kind = SignalKind(k);
                if (s->kind == SignalKind::Sensor) s->type = SignalType::Bool;
                m_scene->notifyAutomationChanged();
            });
            scheduleRefresh();
        });
        m_table->setCellWidget(row, C_Kind, kind);

        auto* type = new QComboBox;
        type->addItems({"Bool", "Int", "Real"});
        type->setCurrentIndex(int(sg.type));
        type->setEnabled(!m_locked && sg.kind != SignalKind::Sensor);
        connect(type, &QComboBox::currentIndexChanged, this, [this, sid](int t){
            Signal* s = m_scene->getSignal(sid);
            if (!s) return;
            edit([&]{
                s->type = SignalType(t);
                s->initialValue = coerceSignal(s->type, s->initialValue);
                m_scene->notifyAutomationChanged();
            });
            scheduleRefresh();
        });
        m_table->setCellWidget(row, C_Type, type);

        const bool computed = sg.kind == SignalKind::Derived || sg.kind == SignalKind::Sensor;
        auto* init = new QTableWidgetItem(computed ? QString("—")
                                          : autoui::formatValue(sg.initialValue, false, sg.type));
        if (computed) init->setFlags(init->flags() & ~Qt::ItemIsEditable);
        init->setToolTip("Value at the start of the simulation (and after Reset)");
        m_table->setItem(row, C_Initial, init);

        if (sg.kind == SignalKind::Derived) {
            auto* ex = autoui::makeExprEdit(m_scene, autoui::ExprKind::Value, "e.g. J1.inPosition && !EStop");
            ex->setExpression(sg.expression);
            ex->setEnabled(!m_locked);
            ex->setFrame(false);
            connect(ex, &ExpressionEdit::committed, this, [this, sid](const QString& t){
                Signal* s = m_scene->getSignal(sid);
                if (!s) return;
                edit([&]{ s->expression = t; m_scene->notifyAutomationChanged(); });
            });
            m_table->setCellWidget(row, C_Expr, ex);
        } else if (sg.kind == SignalKind::Sensor) {
            auto* cell = new QWidget;
            auto* hb = new QHBoxLayout(cell);
            hb->setContentsMargins(4, 0, 2, 0);
            const QString problem = sensors::problem(*m_scene, sg.sensor);
            auto* text = new QLabel(problem.isEmpty() ? sensors::summary(*m_scene, sg.sensor) : "⚠ " + problem);
            text->setProperty("error", !problem.isEmpty());
            text->setToolTip(text->text());
            auto* cfg = new QPushButton(Theme::icon("sliders-horizontal"), "Configure…");
            cfg->setEnabled(!m_locked);
            connect(cfg, &QPushButton::clicked, this, [this, sid]{ configureSensor(sid); });
            hb->addWidget(text, 1);
            hb->addWidget(cfg);
            m_table->setCellWidget(row, C_Expr, cell);
        } else {
            auto* na = new QTableWidgetItem(sg.kind == SignalKind::Input ? "set from the Runtime tab"
                                                                         : "set by operation actions");
            na->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
            na->setForeground(Theme::colors().muted);
            m_table->setItem(row, C_Expr, na);
        }

        m_table->setItem(row, C_Desc, new QTableWidgetItem(sg.description));

        auto* val = new QTableWidgetItem("—");
        val->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
        val->setFont(QFont("monospace"));
        m_table->setItem(row, C_Value, val);

        if (m_locked)
            for (int c : {C_Name, C_Initial, C_Desc})
                if (auto* it = m_table->item(row, c)) it->setFlags(it->flags() & ~Qt::ItemIsEditable);
        ++row;
    }
    m_building = false;
    if (keepRow >= 0 && keepRow < m_table->rowCount()) m_table->selectRow(keepRow);
    refreshRuntime();
}

void SignalsPage::refreshRuntime() {
    AutomationEngine* ae = m_sim->automation();
    for (int r = 0; r < m_table->rowCount(); ++r) {
        auto* it = m_table->item(r, C_Value);
        auto* nm = m_table->item(r, C_Name);
        if (!it || !nm) continue;
        if (!ae->isActive()) { it->setText("—"); it->setBackground(QBrush()); continue; }
        Signal* s = m_scene->getSignal(nm->data(Qt::UserRole).toInt());
        if (!s) continue;
        double v = m_sim->displayValue(s->name);
        it->setText(autoui::formatValue(v, false, s->type));
        if (s->type == SignalType::Bool)
            it->setBackground(autoui::boolBrush(v != 0.0));
    }
}

void SignalsPage::setLocked(bool locked) {
    Page::setLocked(locked);
    for (auto* b : {m_addInput, m_addInternal, m_addDerived, m_addSensor, m_delete}) b->setEnabled(!locked);
    refresh();
}

void SignalsPage::addSignal(SignalKind kind) {
    Signal s;
    s.kind = kind;
    switch (kind) {
        case SignalKind::Input:    s.name = "Start";   s.type = SignalType::Bool; break;
        case SignalKind::Internal: s.name = "Flag";    s.type = SignalType::Bool; break;
        case SignalKind::Derived:  s.name = "Ready";   s.type = SignalType::Bool; s.expression = "true"; break;
        case SignalKind::Sensor:   s.name = "PartPresent"; s.type = SignalType::Bool; break;
    }
    edit([&]{ m_scene->addSignal(s); });
    refresh();
    m_table->selectRow(m_table->rowCount() - 1);
    m_table->editItem(m_table->item(m_table->rowCount() - 1, C_Name));
}

void SignalsPage::addSensorOn(int objectId) {
    if (m_locked) return;
    Signal s;
    s.kind = SignalKind::Sensor;
    s.type = SignalType::Bool;
    s.name = "PartPresent";
    s.sensor.objectId = m_scene->getObject(objectId) ? objectId : -1;
    s.sensor.target   = SensorTarget::RigidBodies;
    int id = -1;
    edit([&]{ id = m_scene->addSignal(s); });
    refresh();
    m_table->selectRow(m_table->rowCount() - 1);
    configureSensor(id);
}

void SignalsPage::configureSensor(int signalId) {
    Signal* s = m_scene->getSignal(signalId);
    if (!s || m_locked) return;
    SensorDialog dlg(m_scene, s->name, s->sensor, this);
    if (dlg.exec() != QDialog::Accepted) return;
    s = m_scene->getSignal(signalId);
    if (!s) return;
    edit([&]{ s->sensor = dlg.config(); m_scene->notifyAutomationChanged(); });
    scheduleRefresh();
}

void SignalsPage::deleteSelected() {
    int row = m_table->currentRow();
    int id = signalIdAt(row);
    if (id < 0) return;
    edit([&]{ m_scene->removeSignal(id); });
    refresh();
    emit statusMessage("Signal deleted — expressions that used it will show errors", true);
}

void SignalsPage::onItemChanged(int row, int col) {
    if (m_building) return;
    Signal* s = m_scene->getSignal(signalIdAt(row));
    if (!s) return;
    auto* item = m_table->item(row, col);
    QString text = item->text().trimmed();

    if (col == C_Name) {
        if (text == s->name) return;
        if (!expr::isValidName(text)) {
            emit statusMessage("'" + text + "' is not a valid name — use letters, digits and _ (no spaces, not starting with a digit)", false);
            scheduleRefresh();
            return;
        }
        if (m_scene->isSymbolNameTaken(text, s->id)) {
            emit statusMessage("The name '" + text + "' is already used", false);
            scheduleRefresh();
            return;
        }
        QString old = s->name;
        edit([&]{
            s->name = text;
            m_scene->renameSymbolReferences(old, text);
        });
        emit statusMessage(QString("Renamed %1 → %2 (expressions updated)").arg(old, text), true);
        scheduleRefresh();
    } else if (col == C_Initial) {
        double v;
        if (!parseValue(text, v)) {
            emit statusMessage("Initial value must be a number, true or false", false);
            scheduleRefresh();
            return;
        }
        edit([&]{ s->initialValue = coerceSignal(s->type, v); m_scene->notifyAutomationChanged(); });
        scheduleRefresh();
    } else if (col == C_Desc) {
        edit([&]{ s->description = item->text(); m_scene->notifyAutomationChanged(); });
    }
}
