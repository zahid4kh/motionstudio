#include "PartFlowPage.h"
#include "automation/Expression.h"
#include "automation/Sensors.h"
#include "physics/SimulationEngine.h"
#include "ui/Theme.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QVBoxLayout>
#include <algorithm>

namespace {
enum Col { C_Name, C_Kind, C_Settings, C_Count, C_Count_ };
enum Kind { K_Source, K_Sink };

QString nameOf(const SceneGraph& scene, int id) {
    const SceneObject* o = scene.getObject(id);
    return o ? o->name : QString("(deleted)");
}

QDoubleSpinBox* metres(double lo, double hi, double v) {
    auto* sp = new QDoubleSpinBox;
    sp->setRange(lo, hi);
    sp->setDecimals(3);
    sp->setSingleStep(0.01);
    sp->setSuffix(" m");
    sp->setValue(v);
    return sp;
}

// Whether the part a source copies is a free rigid body (falls, rides belts)
bool hasRigidBody(const SceneGraph& scene, int id) {
    return scene.allRigidBodies().count(id) > 0;
}

QVBoxLayout* dialogFrame(QDialog* d, const QString& title, const QString& sub) {
    d->setWindowTitle(title);
    d->setMinimumWidth(480);
    auto* vb = new QVBoxLayout(d);
    vb->setSpacing(10);
    vb->setContentsMargins(18, 16, 18, 16);
    auto* t = new QLabel(title);
    t->setObjectName("dialogTitle");
    vb->addWidget(t);
    auto* s = new QLabel(sub);
    s->setObjectName("hint");
    s->setWordWrap(true);
    vb->addWidget(s);
    return vb;
}

void addButtons(QDialog* d, QVBoxLayout* vb) {
    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, d);
    btns->button(QDialogButtonBox::Ok)->setDefault(true);
    vb->addWidget(btns);
    QObject::connect(btns, &QDialogButtonBox::accepted, d, &QDialog::accept);
    QObject::connect(btns, &QDialogButtonBox::rejected, d, &QDialog::reject);
}
} // namespace

namespace autoui {

QString sourceSummary(const SceneGraph& scene, const PartSource& src) {
    QString when = src.mode == SourceMode::Interval
                       ? QString("every %1 s").arg(src.interval.trimmed().isEmpty() ? "?" : src.interval.trimmed())
                       : QString("when %1").arg(src.trigger.trimmed().isEmpty() ? "?" : src.trigger.trimmed());
    QString out = QString("Copies %1 %2").arg(src.templateId == -1 ? QString("(nothing)") : nameOf(scene, src.templateId), when);
    if (src.maxCount > 0) out += QString(", up to %1").arg(src.maxCount);
    if (src.whenClear) out += ", when the spot is clear";
    if (!src.enable.trimmed().isEmpty()) out += ", while " + src.enable.trimmed();
    return out;
}

QString sinkSummary(const SceneGraph& scene, const PartSink& sink) {
    const SensorConfig& z = sink.zone;
    QString where = z.objectId == -1 ? QString("world") : nameOf(scene, z.objectId);
    QString zone = z.objectBounds && z.objectId != -1
                       ? QString("Touching %1").arg(where)
                       : QString("Zone on %1, %2×%3×%4 m").arg(where)
                             .arg(double(z.size.x()), 0, 'g', 3).arg(double(z.size.y()), 0, 'g', 3)
                             .arg(double(z.size.z()), 0, 'g', 3);
    const PartSource* src = scene.getSource(sink.sourceId);
    QString what = sink.sourceId == -1 ? QString("any part") : src ? "copies from " + src->name : QString("(deleted source)");
    QString out = zone + " → takes " + what;
    if (!sink.enable.trimmed().isEmpty()) out += ", while " + sink.enable.trimmed();
    return out;
}

} // namespace autoui

// ── Page ─────────────────────────────────────────────────────────────────────

PartFlowPage::PartFlowPage(SceneGraph* scene, SimulationEngine* sim, QWidget* parent)
    : Page(scene, sim, parent)
{
    auto* vb = new QVBoxLayout(this);
    vb->setContentsMargins(6, 6, 6, 6);
    vb->setSpacing(4);

    auto* bar = new QHBoxLayout;
    m_addSource = new QPushButton(Theme::icon("boxes"),   "New Source");
    m_addSink   = new QPushButton(Theme::icon("log-out"), "New Sink");
    m_delete    = new QPushButton(Theme::icon("trash-2", Theme::Tint::Danger), "Delete");
    m_delete->setProperty("danger", true);
    m_addSource->setToolTip("Make copies of the selected part while simulating: every few seconds, or when a "
                            "condition becomes true. The part itself is the template and stays hidden.");
    m_addSink->setToolTip("Take parts away when they reach a zone (the selected object's bounds), e.g. the end "
                          "of a conveyor or a bin. They come back on Reset.");
    bar->addWidget(m_addSource);
    bar->addWidget(m_addSink);
    bar->addStretch();
    auto* hint = new QLabel("Double-click a name to rename it. Use <name>.count in expressions.");
    hint->setObjectName("hint");
    bar->addWidget(hint);
    bar->addWidget(m_delete);
    vb->addLayout(bar);

    m_table = new QTableWidget(0, C_Count_);
    m_table->setHorizontalHeaderLabels({"Name", "Kind", "Settings", "Count"});
    m_table->verticalHeader()->hide();
    m_table->setAlternatingRowColors(true);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed);
    auto* hh = m_table->horizontalHeader();
    hh->setSectionResizeMode(QHeaderView::Interactive);
    m_table->setColumnWidth(C_Name, 150);
    m_table->setColumnWidth(C_Kind, 90);
    m_table->setColumnWidth(C_Count, 150);
    hh->setSectionResizeMode(C_Settings, QHeaderView::Stretch);
    m_table->verticalHeader()->setDefaultSectionSize(28);
    vb->addWidget(m_table);

    connect(m_addSource, &QPushButton::clicked, this, [this]{ addSourceFor(m_scene->selectedObjectId()); });
    connect(m_addSink,   &QPushButton::clicked, this, [this]{ addSinkOn(m_scene->selectedObjectId()); });
    connect(m_delete,    &QPushButton::clicked, this, &PartFlowPage::deleteSelected);
    connect(m_table, &QTableWidget::cellChanged, this, &PartFlowPage::onItemChanged);
    // The "no rigid body" warning follows Properties edits
    connect(scene, &SceneGraph::rigidBodyAdded,   this, [this](int){ scheduleRefresh(); });
    connect(scene, &SceneGraph::rigidBodyRemoved, this, [this](int){ scheduleRefresh(); });
    connect(m_table, &QTableWidget::cellDoubleClicked, this, [this](int row, int col){
        if (col != C_Name) configure(row);
    });
    refresh();
}

void PartFlowPage::refresh() {
    const int keepRow = m_table->currentRow();
    m_building = true;
    m_table->setRowCount(0);
    int row = 0;
    auto addRow = [&](int id, Kind kind, const QString& name, const QString& settings, const QString& problem) {
        m_table->insertRow(row);
        auto* n = new QTableWidgetItem(name);
        n->setData(Qt::UserRole, id);
        n->setData(Qt::UserRole + 1, int(kind));
        QFont f = n->font(); f.setBold(true); n->setFont(f);
        if (m_locked) n->setFlags(n->flags() & ~Qt::ItemIsEditable);
        m_table->setItem(row, C_Name, n);
        auto* k = new QTableWidgetItem(Theme::icon(kind == K_Source ? "boxes" : "log-out"),
                                       kind == K_Source ? "Source" : "Sink");
        k->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
        m_table->setItem(row, C_Kind, k);

        auto* cell = new QWidget;
        auto* hb = new QHBoxLayout(cell);
        hb->setContentsMargins(4, 0, 2, 0);
        auto* text = new QLabel(problem.isEmpty() ? settings : "⚠ " + problem);
        text->setProperty("error", !problem.isEmpty());
        text->setToolTip(text->text());
        auto* cfg = new QPushButton(Theme::icon("sliders-horizontal"), "Configure…");
        cfg->setEnabled(!m_locked);
        const int r = row;
        connect(cfg, &QPushButton::clicked, this, [this, r]{ configure(r); });
        hb->addWidget(text, 1);
        hb->addWidget(cfg);
        m_table->setCellWidget(row, C_Settings, cell);

        auto* count = new QTableWidgetItem("—");
        count->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
        count->setFont(QFont("monospace"));
        m_table->setItem(row, C_Count, count);
        ++row;
    };
    for (auto& [id, src] : m_scene->allSources()) {
        QString problem;
        if (!m_scene->getObject(src.templateId)) problem = src.templateId == -1 ? "nothing to copy selected" : "the object it copies no longer exists";
        else if (!hasRigidBody(*m_scene, src.templateId))
            problem = QString("%1 has no rigid body: copies won't fall or ride belts").arg(nameOf(*m_scene, src.templateId));
        addRow(id, K_Source, src.name, autoui::sourceSummary(*m_scene, src), problem);
    }
    for (auto& [id, sink] : m_scene->allSinks())
        addRow(id, K_Sink, sink.name, autoui::sinkSummary(*m_scene, sink), sensors::problem(*m_scene, sink.zone));
    m_building = false;
    if (keepRow >= 0 && keepRow < m_table->rowCount()) m_table->selectRow(keepRow);
    m_delete->setEnabled(!m_locked && m_table->rowCount() > 0);
    refreshRuntime();
}

void PartFlowPage::refreshRuntime() {
    AutomationEngine* ae = m_sim->automation();
    for (int r = 0; r < m_table->rowCount(); ++r) {
        auto* n = m_table->item(r, C_Name);
        auto* it = m_table->item(r, C_Count);
        if (!n || !it) continue;
        if (!ae->isActive()) { it->setText("—"); continue; }
        const QString name = n->text();
        const int count = int(m_sim->displayValue(name + ".count"));
        if (n->data(Qt::UserRole + 1).toInt() == K_Source) {
            const bool blocked = m_sim->displayValue(name + ".blocked") != 0.0;
            it->setText(QString("%1 made%2").arg(count).arg(blocked ? " · waiting for room" : ""));
            it->setBackground(blocked ? QBrush(Theme::soft(Theme::colors().warning, 0.22)) : QBrush());
        } else {
            it->setText(QString("%1 taken").arg(count));
        }
    }
}

void PartFlowPage::setLocked(bool locked) {
    Page::setLocked(locked);
    m_addSource->setEnabled(!locked);
    m_addSink->setEnabled(!locked);
    refresh();
}

void PartFlowPage::addSourceFor(int objectId) {
    if (m_locked) return;
    PartSource src;
    const SceneObject* o = m_scene->getObject(objectId);
    src.name = m_scene->makeSymbolName(o ? o->name + "Source" : QString("Source"));
    src.templateId = o ? objectId : -1;
    int id = -1;
    edit([&]{ id = m_scene->addSource(src); });
    refresh();
    for (int r = 0; r < m_table->rowCount(); ++r)
        if (m_table->item(r, C_Name)->data(Qt::UserRole).toInt() == id) { m_table->selectRow(r); configure(r); break; }
}

void PartFlowPage::addSinkOn(int objectId) {
    if (m_locked) return;
    PartSink sink;
    const SceneObject* o = m_scene->getObject(objectId);
    sink.name = m_scene->makeSymbolName(o ? o->name + "Sink" : QString("Sink"));
    sink.zone.objectId = o ? objectId : -1;
    sink.zone.objectBounds = o != nullptr;
    sink.zone.size = {0.3f, 0.3f, 0.3f};
    int id = -1;
    edit([&]{ id = m_scene->addSink(sink); });
    refresh();
    for (int r = 0; r < m_table->rowCount(); ++r)
        if (m_table->item(r, C_Name)->data(Qt::UserRole).toInt() == id) { m_table->selectRow(r); configure(r); break; }
}

void PartFlowPage::configure(int row) {
    auto* n = m_table->item(row, C_Name);
    if (!n || m_locked) return;
    const int id = n->data(Qt::UserRole).toInt();
    if (n->data(Qt::UserRole + 1).toInt() == K_Source) {
        PartSource* src = m_scene->getSource(id);
        if (!src) return;
        SourceDialog dlg(m_scene, *src, this);
        if (dlg.exec() != QDialog::Accepted || !(src = m_scene->getSource(id))) return;
        edit([&]{ *src = dlg.config(); m_scene->notifyAutomationChanged(); });
    } else {
        PartSink* sink = m_scene->getSink(id);
        if (!sink) return;
        SinkDialog dlg(m_scene, *sink, this);
        if (dlg.exec() != QDialog::Accepted || !(sink = m_scene->getSink(id))) return;
        edit([&]{ *sink = dlg.config(); m_scene->notifyAutomationChanged(); });
    }
    scheduleRefresh();
}

void PartFlowPage::deleteSelected() {
    auto* n = m_table->item(m_table->currentRow(), C_Name);
    if (!n || m_locked) return;
    const int id = n->data(Qt::UserRole).toInt();
    const bool source = n->data(Qt::UserRole + 1).toInt() == K_Source;
    edit([&]{ source ? m_scene->removeSource(id) : m_scene->removeSink(id); });
    refresh();
    emit statusMessage(QString("%1 deleted — expressions that used it will show errors").arg(source ? "Source" : "Sink"), true);
}

void PartFlowPage::onItemChanged(int row, int col) {
    if (m_building || col != C_Name) return;
    auto* n = m_table->item(row, C_Name);
    const int id = n->data(Qt::UserRole).toInt();
    const bool source = n->data(Qt::UserRole + 1).toInt() == K_Source;
    QString* name = nullptr;
    if (source) { if (PartSource* s = m_scene->getSource(id)) name = &s->name; }
    else if (PartSink* s = m_scene->getSink(id)) name = &s->name;
    if (!name) return;
    const QString text = n->text().trimmed();
    if (text == *name) return;
    if (!expr::isValidName(text)) {
        emit statusMessage("'" + text + "' is not a valid name — use letters, digits and _ (no spaces, not starting with a digit)", false);
        scheduleRefresh();
        return;
    }
    if (m_scene->isSymbolNameTaken(text, id)) {
        emit statusMessage("The name '" + text + "' is already used", false);
        scheduleRefresh();
        return;
    }
    const QString old = *name;
    edit([&]{
        *name = text;
        m_scene->renameSymbolReferences(old, text);
    });
    emit statusMessage(QString("Renamed %1 → %2 (expressions updated)").arg(old, text), true);
    scheduleRefresh();
}

// ── Source dialog ────────────────────────────────────────────────────────────

SourceDialog::SourceDialog(SceneGraph* scene, const PartSource& src, QWidget* parent)
    : QDialog(parent), m_scene(scene), m_src(src)
{
    auto* vb = dialogFrame(this, "Source " + src.name,
        "Makes copies of a part while simulating. The part you pick is the template: it is hidden during "
        "the run and each copy appears where it is, then falls, rides belts and can be sensed and gripped "
        "like any part. Copies are removed on Reset.");
    m_form = new QFormLayout;
    m_form->setLabelAlignment(Qt::AlignRight);
    vb->addLayout(m_form);

    m_template = new QComboBox;
    autoui::fillObjectCombo(m_template, *scene, src.templateId, "(choose)");
    m_template->setToolTip("The part to copy, with everything under it");
    m_form->addRow("Copies", m_template);

    m_mode = new QComboBox;
    m_mode->addItem("Every few seconds", int(SourceMode::Interval));
    m_mode->addItem("Each time a condition becomes true", int(SourceMode::Trigger));
    m_mode->setCurrentIndex(src.mode == SourceMode::Trigger ? 1 : 0);
    m_form->addRow("When", m_mode);

    m_interval = autoui::makeExprEdit(scene, autoui::ExprKind::Value, "seconds, e.g. 2");
    m_interval->setExpression(src.interval);
    m_interval->setHelpText("Seconds between copies; the first is made at the start. An expression, read as "
                            "each copy is made, so it can depend on signals.");
    m_form->addRow("Interval", m_interval);
    m_trigger = autoui::makeExprEdit(scene, autoui::ExprKind::Value, "e.g. Feed");
    m_trigger->setExpression(src.trigger);
    m_trigger->setHelpText("A copy is made each time this becomes true (rising edge)");
    m_form->addRow("Trigger", m_trigger);
    m_enable = autoui::makeExprEdit(scene, autoui::ExprKind::Value, "always");
    m_enable->setExpression(src.enable);
    m_enable->setHelpText("No copies while this is false");
    m_form->addRow("Enabled", m_enable);

    m_max = new QSpinBox;
    m_max->setRange(0, 1000000);
    m_max->setSpecialValueText("No limit");
    m_max->setValue(src.maxCount);
    m_form->addRow("At most", m_max);
    m_clear = new QCheckBox("Wait until the spot is clear");
    m_clear->setChecked(src.whenClear);
    m_clear->setToolTip("Don't make a copy while another part still overlaps the template's place, "
                        "so parts never appear inside each other");
    m_form->addRow("", m_clear);

    m_hint = new QLabel;
    m_hint->setObjectName("hint");
    m_hint->setWordWrap(true);
    vb->addWidget(m_hint);
    addButtons(this, vb);

    connect(m_template, &QComboBox::currentIndexChanged, this, &SourceDialog::updateForm);
    connect(m_mode, &QComboBox::currentIndexChanged, this, &SourceDialog::updateForm);
    updateForm();
}

PartSource SourceDialog::config() const {
    PartSource s = m_src;
    s.templateId = m_template->currentData().toInt();
    s.mode       = SourceMode(m_mode->currentData().toInt());
    s.interval   = m_interval->text().trimmed();
    s.trigger    = m_trigger->text().trimmed();
    s.enable     = m_enable->text().trimmed();
    s.maxCount   = m_max->value();
    s.whenClear  = m_clear->isChecked();
    return s;
}

void SourceDialog::updateForm() {
    const bool interval = m_mode->currentData().toInt() == int(SourceMode::Interval);
    m_form->setRowVisible(m_interval, interval);
    m_form->setRowVisible(m_trigger, !interval);
    const int t = m_template->currentData().toInt();
    QString text;
    if (!m_scene->getObject(t))
        text = "Pick the part to copy.";
    else if (!hasRigidBody(*m_scene, t))
        text = QString("⚠ %1 has no rigid body, so its copies stay where they appear (until gripped). "
                       "Give it one in Properties to let them fall and ride conveyors.").arg(nameOf(*m_scene, t));
    else
        text = QString("Copies of %1 are named “%1 #1”, “#2”… In expressions: %2.count (copies made) and "
                       "%2.blocked (one is due but the spot is taken).").arg(nameOf(*m_scene, t), m_src.name);
    m_hint->setText(text);
    adjustSize();
}

// ── Sink dialog ──────────────────────────────────────────────────────────────

SinkDialog::SinkDialog(SceneGraph* scene, const PartSink& sink, QWidget* parent)
    : QDialog(parent), m_scene(scene), m_sink(sink)
{
    auto* vb = dialogFrame(this, "Sink " + sink.name,
        "Takes parts away while simulating when they touch its zone: copies made by sources, and free parts "
        "(rigid bodies at the top of the tree). Parts held by a gripper are left alone. "
        "Everything comes back on Reset.");
    m_form = new QFormLayout;
    m_form->setLabelAlignment(Qt::AlignRight);
    vb->addLayout(m_form);

    m_mount = new QComboBox;
    autoui::fillObjectCombo(m_mount, *scene, sink.zone.objectId, "World (fixed)");
    m_mount->setToolTip("The zone moves with this object; the object itself is never taken");
    m_form->addRow("Mounted on", m_mount);
    m_bounds = new QCheckBox("Use the mounted object's bounding box");
    m_bounds->setChecked(sink.zone.objectBounds);
    m_form->addRow("", m_bounds);
    auto* offRow = new QHBoxLayout;
    for (int i = 0; i < 3; ++i) {
        m_offset[i] = metres(-100, 100, sink.zone.offset[i]);
        m_offset[i]->setPrefix(QString("XYZ"[i]) + "  ");
        offRow->addWidget(m_offset[i]);
    }
    m_form->addRow("Offset", offRow);
    auto* sizeRow = new QHBoxLayout;
    for (int i = 0; i < 3; ++i) {
        m_size[i] = metres(0.001, 100, sink.zone.size[i]);
        m_size[i]->setPrefix(QString("XYZ"[i]) + "  ");
        sizeRow->addWidget(m_size[i]);
    }
    m_form->addRow("Size", sizeRow);
    m_sizeRow = sizeRow;

    m_source = new QComboBox;
    m_source->addItem("Any part", -1);
    for (auto& [id, src] : scene->allSources()) m_source->addItem(Theme::icon("boxes"), "Copies from " + src.name, id);
    int idx = m_source->findData(sink.sourceId);
    if (idx < 0) { m_source->addItem("(deleted source)", sink.sourceId); idx = m_source->count() - 1; }
    m_source->setCurrentIndex(idx);
    m_form->addRow("Takes", m_source);
    m_enable = autoui::makeExprEdit(scene, autoui::ExprKind::Value, "always");
    m_enable->setExpression(sink.enable);
    m_enable->setHelpText("Takes nothing while this is false");
    m_form->addRow("Enabled", m_enable);

    m_hint = new QLabel;
    m_hint->setObjectName("hint");
    m_hint->setWordWrap(true);
    vb->addWidget(m_hint);
    addButtons(this, vb);

    connect(m_mount, &QComboBox::currentIndexChanged, this, &SinkDialog::updateForm);
    connect(m_bounds, &QCheckBox::toggled, this, &SinkDialog::updateForm);
    for (auto* sp : {m_offset[0], m_offset[1], m_offset[2], m_size[0], m_size[1], m_size[2]})
        connect(sp, &QDoubleSpinBox::valueChanged, this, &SinkDialog::updateForm);
    updateForm();
}

PartSink SinkDialog::config() const {
    PartSink s = m_sink;
    s.zone.type         = SensorType::Zone;
    s.zone.objectId     = m_mount->currentData().toInt();
    s.zone.objectBounds = m_bounds->isChecked() && s.zone.objectId != -1;
    s.zone.offset       = {float(m_offset[0]->value()), float(m_offset[1]->value()), float(m_offset[2]->value())};
    s.zone.size         = {float(m_size[0]->value()), float(m_size[1]->value()), float(m_size[2]->value())};
    s.sourceId          = m_source->currentData().toInt();
    s.enable            = m_enable->text().trimmed();
    return s;
}

void SinkDialog::updateForm() {
    const PartSink s = config();
    m_bounds->setEnabled(s.zone.objectId != -1);
    const bool sized = !s.zone.objectBounds;
    m_form->setRowVisible(m_sizeRow, sized);
    for (auto* sp : m_offset) sp->setEnabled(sized);
    const QString problem = sensors::problem(*m_scene, s.zone);
    m_hint->setText(problem.isEmpty()
        ? QString("In expressions: %1.count, the parts taken so far. The zone is drawn in the viewport.").arg(m_sink.name)
        : "⚠ " + problem);
    adjustSize();
}
