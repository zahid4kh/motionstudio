#include "RobotBuilderDialog.h"
#include "RobotAnalysisPanel.h"
#include "RobotSketch.h"
#include "robot/RobotAnalysis.h"
#include "scene/SceneGraph.h"
#include "ui/Theme.h"
#include <QButtonGroup>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QTabWidget>
#include <QTableWidget>
#include <QVBoxLayout>
#include <cmath>

namespace {

enum AxisCol { A_Name, A_Role, A_Min, A_Max, A_Home, A_Speed, A_Accel, A_Count };

QLabel* subheading(const QString& text) {
    auto* l = new QLabel(text);
    l->setObjectName("subheading");
    return l;
}

QLabel* hint(const QString& text) {
    auto* l = new QLabel(text);
    l->setObjectName("hint");
    l->setWordWrap(true);
    return l;
}

QIcon swatch(const robot::ColorScheme& s) {
    QPixmap pm(30, 14);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(Qt::NoPen);
    p.setBrush(s.primary);
    p.drawRoundedRect(QRectF(0, 0, 20, 14), 3, 3);
    p.setBrush(s.secondary);
    p.drawRoundedRect(QRectF(16, 0, 14, 14), 3, 3);
    return QIcon(pm);
}

QDoubleSpinBox* spin(double min, double max, int decimals, const QString& suffix, double step = 1) {
    auto* s = new QDoubleSpinBox;
    s->setRange(min, max);
    s->setDecimals(decimals);
    s->setSuffix(suffix);
    s->setSingleStep(step);
    s->setKeyboardTracking(false);
    s->setAlignment(Qt::AlignRight);
    return s;
}

} // namespace

RobotBuilderDialog::~RobotBuilderDialog() = default;

RobotBuilderDialog::RobotBuilderDialog(QWidget* parent) : QDialog(parent), m_preview(std::make_unique<SceneGraph>()) {
    setWindowTitle("Robot Builder");
    setMinimumSize(1100, 720);
    m_timer.setSingleShot(true);
    m_timer.setInterval(120);
    connect(&m_timer, &QTimer::timeout, this, &RobotBuilderDialog::refresh);

    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(18, 16, 18, 14);
    outer->setSpacing(10);
    auto* title = new QLabel("Robot Builder");
    title->setObjectName("dialogTitle");
    outer->addWidget(title);
    outer->addWidget(hint("Pick a robot type and a preset, then set its dimensions, axis ranges and end effector. The "
                          "drawing shows where each dimension is measured and how every axis turns; Analysis shows "
                          "the workspace it reaches. Create Robot adds it to the scene with its actuators, tool signals "
                          "and a Home pose, ready to jog and to use in sequences."));

    auto* body = new QHBoxLayout;
    body->setSpacing(14);
    outer->addLayout(body, 1);

    // ── Left: the configuration ──
    auto* form = new QWidget;
    auto* fl = new QVBoxLayout(form);
    fl->setContentsMargins(0, 0, 8, 0);
    fl->setSpacing(6);

    fl->addWidget(subheading("ROBOT TYPE"));
    auto* kinds = new QHBoxLayout;
    kinds->setSpacing(6);
    m_kind = new QButtonGroup(this);
    const struct { robot::Kind kind; const char* icon; const char* text; const char* tip; } kindButtons[] = {
        {robot::Kind::Scara, "disc-3", "SCARA\n4-axis", "Two arms turning about vertical axes, a vertical Z stroke and tool rotation: fast pick & place, assembly"},
        {robot::Kind::Palletizer, "boxes", "Palletizer\n4-axis", "Base, shoulder and arm with parallel links that keep the flange facing down, plus tool rotation: stacking"},
        {robot::Kind::Articulated, "bot", "Articulated\n6-axis", "Base, shoulder, elbow and a spherical wrist: reaches any point at any angle"},
    };
    for (const auto& k : kindButtons) {
        auto* b = new QPushButton(Theme::icon(k.icon), k.text);
        b->setObjectName("segment");
        b->setCheckable(true);
        b->setToolTip(k.tip);
        b->setIconSize(QSize(18, 18));
        Theme::segmentIcon(b, k.icon);
        m_kind->addButton(b, int(k.kind));
        kinds->addWidget(b);
    }
    fl->addLayout(kinds);

    fl->addWidget(subheading("PRESET"));
    m_preset = new QComboBox;
    m_preset->setMaxVisibleItems(20);
    fl->addWidget(m_preset);
    m_presetDesc = hint("");
    fl->addWidget(m_presetDesc);

    auto* names = new QGridLayout;
    names->setHorizontalSpacing(8);
    m_name = new QLineEdit;
    m_name->setToolTip("The robot's top object in the hierarchy");
    m_prefix = new QLineEdit;
    m_prefix->setMaximumWidth(70);
    m_prefix->setToolTip("Actuators are named <prefix>1 … <prefix>N (A1 … A6); a name already used gets a suffix (A1_2)");
    auto* nl = new QLabel("Name");
    nl->setObjectName("fieldLabel");
    auto* pl = new QLabel("Axes");
    pl->setObjectName("fieldLabel");
    names->addWidget(nl, 0, 0);
    names->addWidget(m_name, 0, 1);
    names->addWidget(pl, 0, 2);
    names->addWidget(m_prefix, 0, 3);
    fl->addLayout(names);

    fl->addWidget(subheading("DIMENSIONS"));
    fl->addWidget(hint("Point at a value to find it on the drawing; click a value on the drawing to edit it."));
    m_dimGrid = new QGridLayout;
    m_dimGrid->setHorizontalSpacing(8);
    m_dimGrid->setVerticalSpacing(4);
    fl->addLayout(m_dimGrid);

    fl->addWidget(subheading("END EFFECTOR"));
    m_tool = new QComboBox;
    m_tool->setMaxVisibleItems(20);
    for (const robot::ToolDef& t : robot::toolDefs()) m_tool->addItem(Theme::icon(t.icon), t.name, int(t.tool));
    fl->addWidget(m_tool);
    m_toolDesc = hint("");
    fl->addWidget(m_toolDesc);
    auto* toolRow = new QHBoxLayout;
    auto* tsl = new QLabel("Tool size");
    tsl->setObjectName("fieldLabel");
    m_toolScale = spin(0.3, 4, 2, " ×", 0.05);
    m_toolScale->setToolTip("The tool is sized to suit the robot; scale it further here");
    toolRow->addWidget(tsl);
    toolRow->addWidget(m_toolScale);
    toolRow->addStretch();
    fl->addLayout(toolRow);

    fl->addWidget(subheading("APPEARANCE"));
    auto* look = new QGridLayout;
    look->setHorizontalSpacing(8);
    m_style = new QComboBox;
    m_style->addItems(robot::styleNames());
    m_style->setToolTip("Industrial: cast boxy links · Cobot: rounded tubes with joint rings · Slim: thin rods · "
                        "Heavy-duty: thick links, counterbalance and a base plate");
    m_scheme = new QComboBox;
    for (const robot::ColorScheme& s : robot::colorSchemes()) m_scheme->addItem(swatch(s), s.name);
    m_scheme->setIconSize(QSize(30, 14));
    m_linkScale = spin(0.4, 2.5, 2, " ×", 0.05);
    m_linkScale->setToolTip("Link thickness (the kinematics stay the same)");
    int r = 0;
    for (auto [label, w] : {std::pair<const char*, QWidget*>{"Style", m_style}, {"Colours", m_scheme}, {"Link thickness", m_linkScale}}) {
        auto* l = new QLabel(label);
        l->setObjectName("fieldLabel");
        look->addWidget(l, r, 0);
        look->addWidget(w, r++, 1);
    }
    fl->addLayout(look);
    m_problems = new QLabel;
    m_problems->setObjectName("warningText");
    m_problems->setWordWrap(true);
    fl->addWidget(m_problems);
    fl->addStretch();

    auto* scroll = new QScrollArea;
    scroll->setWidget(form);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setFixedWidth(400);
    body->addWidget(scroll);

    // ── Right: drawing, axes, analysis ──
    m_tabs = new QTabWidget;
    auto* drawTab = new QWidget;
    auto* dl = new QVBoxLayout(drawTab);
    dl->setContentsMargins(0, 8, 0, 0);
    auto* views = new QHBoxLayout;
    auto* side = new QPushButton(Theme::icon("ruler"), "Side");
    auto* top = new QPushButton(Theme::icon("radar"), "Top");
    auto* vg = new QButtonGroup(this);
    for (auto [b, icon] : {std::pair{side, "ruler"}, {top, "radar"}}) {
        b->setObjectName("segment");
        b->setCheckable(true);
        Theme::segmentIcon(b, icon);
        vg->addButton(b);
        views->addWidget(b);
    }
    side->setChecked(true);
    views->addStretch();
    auto* legend = hint("Blue: dimensions · the line through the joints is the kinematic chain: ● turns in this view, "
                        "▭ turns about the dashed axis, ■ slides, ○ follows another joint");
    legend->setMaximumWidth(520);
    views->addWidget(legend);
    dl->addLayout(views);
    m_drawing = new RobotSketch;
    dl->addWidget(m_drawing, 1);
    m_tabs->addTab(drawTab, Theme::icon("drafting-compass"), "Drawing");

    auto* axesTab = new QWidget;
    auto* al = new QVBoxLayout(axesTab);
    al->setContentsMargins(0, 8, 0, 0);
    al->addWidget(hint("Each axis's range, the value the robot is built at (its Home pose) and its actuator's speed "
                       "and acceleration. Positive directions: see the role; the drawing shows each axis."));
    m_axes = new QTableWidget(0, A_Count);
    m_axes->setHorizontalHeaderLabels({"Axis", "Role", "Min", "Max", "Home", "Speed", "Acceleration"});
    m_axes->verticalHeader()->hide();
    m_axes->setSelectionMode(QAbstractItemView::NoSelection);
    m_axes->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_axes->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_axes->horizontalHeader()->setSectionResizeMode(A_Name, QHeaderView::ResizeToContents);
    m_axes->horizontalHeader()->setSectionResizeMode(A_Role, QHeaderView::ResizeToContents);
    m_axes->verticalHeader()->setDefaultSectionSize(30);
    al->addWidget(m_axes, 1);
    m_tabs->addTab(axesTab, Theme::icon("sliders-horizontal"), "Axes");

    auto* anTab = new QWidget;
    auto* anl = new QVBoxLayout(anTab);
    anl->setContentsMargins(0, 8, 0, 0);
    m_analysis = new RobotAnalysisPanel;
    anl->addWidget(m_analysis);
    m_tabs->addTab(anTab, Theme::icon("gauge"), "Analysis");
    body->addWidget(m_tabs, 1);

    auto* bottom = new QHBoxLayout;
    m_summary = hint("");
    bottom->addWidget(m_summary, 1);
    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QPushButton* ok = btns->button(QDialogButtonBox::Ok);
    ok->setText("Create Robot");
    ok->setIcon(Theme::icon("bot", Theme::Tint::OnAccent));
    ok->setDefault(true);
    bottom->addWidget(btns);
    outer->addLayout(bottom);

    // ── Wiring ──
    connect(btns, &QDialogButtonBox::accepted, this, [this] {
        readUi();
        QSettings("MotionStudio", "MotionStudio").setValue(lastSpecKey(),
            QString::fromUtf8(QJsonDocument(robot::toJson(spec())).toJson(QJsonDocument::Compact)));
        accept();
    });
    connect(btns, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(m_kind, &QButtonGroup::idClicked, this, [this](int id) {
        if (robot::Kind(id) != m_spec.kind) setKind(robot::Kind(id));
    });
    connect(m_preset, &QComboBox::activated, this, [this](int i) {
        const int idx = m_preset->itemData(i).toInt();
        if (idx < 0) return;
        robot::Spec s = robot::presets()[idx].spec;
        s.name = m_name->text();
        s.prefix = m_prefix->text();
        loadSpec(s);
        m_preset->setCurrentIndex(i);
        m_presetDesc->setText(robot::presets()[idx].description);
    });
    connect(m_name, &QLineEdit::editingFinished, this, &RobotBuilderDialog::edited);
    connect(m_prefix, &QLineEdit::editingFinished, this, &RobotBuilderDialog::edited);
    connect(m_tool, &QComboBox::currentIndexChanged, this, &RobotBuilderDialog::edited);
    connect(m_style, &QComboBox::currentIndexChanged, this, &RobotBuilderDialog::edited);
    connect(m_scheme, &QComboBox::currentIndexChanged, this, &RobotBuilderDialog::edited);
    connect(m_toolScale, &QDoubleSpinBox::valueChanged, this, &RobotBuilderDialog::edited);
    connect(m_linkScale, &QDoubleSpinBox::valueChanged, this, &RobotBuilderDialog::edited);
    connect(side, &QPushButton::toggled, this, [this](bool on) { if (on) m_drawing->setView(RobotSketch::View::Side); });
    connect(top, &QPushButton::toggled, this, [this](bool on) { if (on) m_drawing->setView(RobotSketch::View::Top); });
    connect(m_drawing, &RobotSketch::dimensionClicked, this, [this](const QString& sym) {
        for (auto& [key, sp] : m_dims)
            if (key == sym) { sp->setFocus(); sp->selectAll(); }
        m_drawing->setHighlight(sym);
    });

    // Start from the last robot designed, else the first 6-axis preset
    const QString saved = QSettings("MotionStudio", "MotionStudio").value(lastSpecKey()).toString();
    robot::Spec start;
    if (!saved.isEmpty()) {
        start = robot::fromJson(QJsonDocument::fromJson(saved.toUtf8()).object());
    } else {
        for (const robot::Preset& p : robot::presets())
            if (p.spec.kind == robot::Kind::Articulated) { start = p.spec; break; }
    }
    loadSpec(start);
}

robot::Spec RobotBuilderDialog::spec() const {
    robot::Spec s = m_spec;
    robot::validate(s);
    return s;
}

void RobotBuilderDialog::setKind(robot::Kind k) {
    robot::Spec s = robot::defaultSpec(k);
    for (const robot::Preset& p : robot::presets())
        if (p.spec.kind == k) { s = p.spec; break; }
    s.name = m_name->text().isEmpty() ? QString("Robot") : m_name->text();
    loadSpec(s);
}

void RobotBuilderDialog::fillPresets() {
    m_preset->clear();
    m_preset->addItem(Theme::icon("pencil"), "Custom", -1);
    const std::vector<robot::Preset>& ps = robot::presets();
    int match = 0;
    for (int i = 0; i < int(ps.size()); ++i) {
        if (ps[i].spec.kind != m_spec.kind) continue;
        m_preset->addItem(Theme::icon(robot::toolDef(ps[i].spec.tool).icon), ps[i].name, i);
        robot::Spec a = ps[i].spec, b = m_spec;
        a.name = b.name;
        a.prefix = b.prefix;
        if (robot::toJson(a) == robot::toJson(b)) match = m_preset->count() - 1;
    }
    m_preset->setCurrentIndex(match);
    const int idx = m_preset->currentData().toInt();
    m_presetDesc->setText(idx >= 0 ? ps[idx].description : QString("Your own design: change anything, the drawing follows."));
}

void RobotBuilderDialog::loadSpec(const robot::Spec& in) {
    m_loading = true;
    m_spec = in;
    robot::validate(m_spec);
    m_kind->button(int(m_spec.kind))->setChecked(true);
    m_name->setText(m_spec.name);
    m_prefix->setText(m_spec.prefix);
    fillPresets();
    rebuildDimensions();
    m_tool->setCurrentIndex(m_tool->findData(int(m_spec.tool)));
    m_toolDesc->setText(robot::toolDef(m_spec.tool).description);
    m_toolScale->setValue(m_spec.toolScale);
    m_style->setCurrentIndex(int(m_spec.style));
    m_scheme->setCurrentIndex(m_spec.scheme);
    m_linkScale->setValue(m_spec.linkScale);
    rebuildAxes();
    m_problems->clear();
    m_loading = false;
    refresh();
}

void RobotBuilderDialog::rebuildDimensions() {
    while (QLayoutItem* it = m_dimGrid->takeAt(0)) {
        delete it->widget();
        delete it;
    }
    m_dims.clear();
    int r = 0;
    for (const robot::DimensionDef& d : robot::dimensionDefs(m_spec.kind)) {
        auto* sym = new QLabel(d.symbol);
        sym->setObjectName("dimSymbol");
        sym->setMinimumWidth(26);
        auto* label = new QLabel(d.label);
        label->setObjectName("fieldLabel");
        label->setToolTip(d.description);
        auto* sp = spin(d.min * 1000, d.max * 1000, 0, " mm", 5);
        sp->setValue(m_spec.dim(d.key) * 1000);
        sp->setToolTip(QString("%1 — %2").arg(d.symbol, d.description));
        sp->setProperty("dimKey", d.key);
        sp->installEventFilter(this);
        connect(sp, &QDoubleSpinBox::valueChanged, this, &RobotBuilderDialog::edited);
        m_dimGrid->addWidget(sym, r, 0);
        m_dimGrid->addWidget(label, r, 1);
        m_dimGrid->addWidget(sp, r, 2);
        m_dims.push_back({d.key, sp});
        ++r;
    }
}

void RobotBuilderDialog::rebuildAxes() {
    const std::vector<robot::AxisDef> defs = robot::axisDefs(m_spec.kind);
    m_axes->setRowCount(int(defs.size()));
    for (int r = 0; r < int(defs.size()); ++r) {
        const robot::AxisDef& d = defs[r];
        const robot::AxisSpec& a = m_spec.axes[r];
        auto* name = new QTableWidgetItem(QString("%1%2").arg(m_spec.prefix).arg(r + 1));
        QFont f = name->font();
        f.setBold(true);
        name->setFont(f);
        m_axes->setItem(r, A_Name, name);
        m_axes->setItem(r, A_Role, new QTableWidgetItem(d.role));
        const double k = d.prismatic ? 1000 : 1;
        const QString unit = d.prismatic ? " mm" : "°";
        auto* mn = spin(d.prismatic ? -5000 : -720, d.prismatic ? 5000 : 720, 1, unit, d.prismatic ? 5 : 5);
        auto* mx = spin(d.prismatic ? -5000 : -720, d.prismatic ? 5000 : 720, 1, unit, d.prismatic ? 5 : 5);
        auto* home = spin(d.prismatic ? -5000 : -720, d.prismatic ? 5000 : 720, 1, unit, d.prismatic ? 5 : 5);
        auto* sp = spin(d.prismatic ? 1 : 1, d.prismatic ? 10000 : 10000, 0, d.prismatic ? " mm/s" : " °/s", 10);
        auto* ac = spin(0, 200000, 0, d.prismatic ? " mm/s²" : " °/s²", 50);
        mn->setValue(a.min * k);
        mx->setValue(a.max * k);
        home->setValue(a.home * k);
        sp->setValue(a.speed * k);
        ac->setValue(a.accel * k);
        if (m_spec.kind == robot::Kind::Scara && r == 2) {
            mn->setEnabled(false);
            mx->setEnabled(false);
            mx->setToolTip("J3's range is the Z stroke S (set under Dimensions)");
        }
        ac->setToolTip("0 = instant speed changes");
        int c = A_Min;
        for (QDoubleSpinBox* s : {mn, mx, home, sp, ac}) {
            m_axes->setCellWidget(r, c++, s);
            connect(s, &QDoubleSpinBox::valueChanged, this, &RobotBuilderDialog::edited);
        }
    }
}

void RobotBuilderDialog::readUi() {
    m_spec.name = m_name->text().trimmed();
    m_spec.prefix = m_prefix->text().trimmed();
    for (auto& [key, sp] : m_dims) m_spec.dims[key] = sp->value() / 1000.0;
    m_spec.tool = robot::Tool(m_tool->currentData().toInt());
    m_spec.toolScale = m_toolScale->value();
    m_spec.style = robot::Style(m_style->currentIndex());
    m_spec.scheme = m_scheme->currentIndex();
    m_spec.linkScale = m_linkScale->value();
    const std::vector<robot::AxisDef> defs = robot::axisDefs(m_spec.kind);
    for (int r = 0; r < int(defs.size()) && r < m_axes->rowCount(); ++r) {
        const double k = defs[r].prismatic ? 1000 : 1;
        auto val = [&](int c) { return static_cast<QDoubleSpinBox*>(m_axes->cellWidget(r, c))->value() / k; };
        robot::AxisSpec& a = m_spec.axes[r];
        a.min = val(A_Min);
        a.max = val(A_Max);
        a.home = val(A_Home);
        a.speed = val(A_Speed);
        a.accel = val(A_Accel);
    }
}

void RobotBuilderDialog::edited() {
    if (m_loading) return;
    readUi();
    robot::Spec checked = m_spec;
    const QStringList problems = robot::validate(checked);
    m_problems->setText(problems.isEmpty() ? QString() : "⚠ " + problems.join("\n⚠ "));
    m_toolDesc->setText(robot::toolDef(m_spec.tool).description);
    // Keep the axes table in step with what validation fixes (SCARA stroke → J3)
    m_loading = true;
    if (m_spec.kind == robot::Kind::Scara && m_axes->rowCount() > 2) {
        static_cast<QDoubleSpinBox*>(m_axes->cellWidget(2, A_Min))->setValue(checked.axes[2].min * 1000);
        static_cast<QDoubleSpinBox*>(m_axes->cellWidget(2, A_Max))->setValue(checked.axes[2].max * 1000);
        m_spec.axes[2] = checked.axes[2];
    }
    for (int r = 0; r < m_axes->rowCount(); ++r)
        m_axes->item(r, A_Name)->setText(QString("%1%2").arg(checked.prefix).arg(r + 1));
    if (m_preset->currentIndex() != 0) {
        m_preset->setCurrentIndex(0);
        m_presetDesc->setText("Your own design: change anything, the drawing follows.");
    }
    m_loading = false;
    m_timer.start();
}

void RobotBuilderDialog::refresh() {
    const robot::Spec s = spec();
    m_preview = std::make_unique<SceneGraph>();
    const robot::Built b = robot::build(*m_preview, s);
    m_drawing->capture(*m_preview, b.root, b.tcp);
    m_drawing->setDimensions(robot::dimensionLines(s));
    const robot::Analysis a = robot::analyze(*m_preview, b.tcp, 20000);
    m_analysis->setAnalysis(a, *m_preview);
    int parts = 0;
    for (auto& [id, o] : m_preview->allObjects()) if (o.type != PrimitiveType::ImportedMesh) ++parts;
    QStringList extras;
    for (auto& [id, sg] : m_preview->allSignals()) extras << sg.name;
    m_summary->setText(QString("%1 · %2 axes %3%4–%3%5 · reach %6 mm · %7 · %8 parts%9")
                           .arg(robot::kindName(s.kind)).arg(b.joints.size()).arg(s.prefix).arg(1).arg(b.joints.size())
                           .arg(std::round(a.maxReach * 1000)).arg(robot::toolDef(s.tool).name).arg(parts)
                           .arg(extras.isEmpty() ? QString() : " · signals " + extras.join(", ")));
}

bool RobotBuilderDialog::eventFilter(QObject* o, QEvent* e) {
    const QString key = o->property("dimKey").toString();
    if (!key.isEmpty()) {
        if (e->type() == QEvent::Enter || e->type() == QEvent::FocusIn) {
            m_drawing->setHighlight(key);
            if (m_tabs->currentIndex() != 0 && e->type() == QEvent::FocusIn) m_tabs->setCurrentIndex(0);
        } else if (e->type() == QEvent::Leave || e->type() == QEvent::FocusOut) {
            QString focused;
            for (auto& [k, sp] : m_dims) if (sp->hasFocus()) focused = k;
            m_drawing->setHighlight(focused);
        }
    }
    return QDialog::eventFilter(o, e);
}
