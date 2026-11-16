#include "HumanoidBuilderDialog.h"
#include "RobotSketch.h"
#include "scene/SceneGraph.h"
#include "ui/Theme.h"
#include <QButtonGroup>
#include <QCheckBox>
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
#include <optional>

namespace {

enum JointCol { J_Name, J_Positive, J_Min, J_Max, J_Home, J_Speed, J_Accel, J_Count };

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

QLabel* fieldLabel(const QString& text) {
    auto* l = new QLabel(text);
    l->setObjectName("fieldLabel");
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

QString handDescription(humanoid::Hand h) {
    switch (h) {
    case humanoid::Hand::Mitten:  return "A solid hand with a thumb: no actuators.";
    case humanoid::Hand::Gripper: return "Two parallel jaws per hand, closed by the actuators LGrip and RGrip (0 = open).";
    case humanoid::Hand::Fingers: return "A thumb and four two-part fingers per hand, all curled together by the actuators "
                                         "LHand and RHand (0 = open, 90 = fist).";
    }
    return {};
}

const char* kCustom = "Your own design: change anything, the drawing follows.";

} // namespace

HumanoidBuilderDialog::~HumanoidBuilderDialog() = default;

HumanoidBuilderDialog::HumanoidBuilderDialog(QWidget* parent)
    : QDialog(parent), m_preview(std::make_unique<SceneGraph>()) {
    setWindowTitle("Humanoid Builder");
    setMinimumSize(1100, 740);
    m_timer.setSingleShot(true);
    m_timer.setInterval(120);
    connect(&m_timer, &QTimer::timeout, this, &HumanoidBuilderDialog::refresh);

    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(18, 16, 18, 14);
    outer->setSpacing(10);
    auto* title = new QLabel("Humanoid Builder");
    title->setObjectName("dialogTitle");
    outer->addWidget(title);
    outer->addWidget(hint("Start from a preset, then set the body's dimensions and choose its joints in the Joints tab: "
                          "each one you tick gets an actuator, left and right mirrored so the same value moves both "
                          "sides alike. Create Humanoid adds it to the scene with a Home pose (and demo poses and a "
                          "looping sequence if you want them), ready to jog and to use in sequences."));

    auto* body = new QHBoxLayout;
    body->setSpacing(14);
    outer->addLayout(body, 1);

    // ── Left: the body ──
    auto* form = new QWidget;
    auto* fl = new QVBoxLayout(form);
    fl->setContentsMargins(0, 0, 8, 0);
    fl->setSpacing(6);

    fl->addWidget(subheading("PRESET"));
    m_preset = new QComboBox;
    m_preset->setMaxVisibleItems(20);
    fl->addWidget(m_preset);
    m_presetDesc = hint("");
    fl->addWidget(m_presetDesc);

    auto* nameRow = new QHBoxLayout;
    m_name = new QLineEdit;
    m_name->setToolTip("The humanoid's top object in the hierarchy; its poses are named after it");
    nameRow->addWidget(fieldLabel("Name"));
    nameRow->addWidget(m_name, 1);
    fl->addLayout(nameRow);

    fl->addWidget(subheading("BODY"));
    auto* bases = new QHBoxLayout;
    bases->setSpacing(6);
    m_base = new QButtonGroup(this);
    const struct { humanoid::Base base; const char* icon; const char* text; const char* tip; } baseButtons[] = {
        {humanoid::Base::Legs, "person-standing", "On its legs", "A full humanoid standing on the floor; its pelvis stays in place"},
        {humanoid::Base::Pedestal, "cylinder", "On a pedestal", "Only the upper body, on a column"},
    };
    for (const auto& b : baseButtons) {
        auto* btn = new QPushButton(Theme::icon(b.icon), b.text);
        btn->setObjectName("segment");
        btn->setCheckable(true);
        btn->setToolTip(b.tip);
        Theme::segmentIcon(btn, b.icon);
        m_base->addButton(btn, int(b.base));
        bases->addWidget(btn);
    }
    fl->addLayout(bases);
    auto* heightRow = new QHBoxLayout;
    m_height = spin(150, 5000, 0, " mm", 10);
    m_height->setToolTip("Floor to the top of the head. Changing it scales every dimension below.");
    heightRow->addWidget(fieldLabel("Overall height"));
    heightRow->addWidget(m_height);
    heightRow->addStretch();
    fl->addLayout(heightRow);

    fl->addWidget(subheading("DIMENSIONS"));
    fl->addWidget(hint("Point at a value to find it on the drawing; click a value on the drawing to edit it."));
    m_dimGrid = new QGridLayout;
    m_dimGrid->setHorizontalSpacing(8);
    m_dimGrid->setVerticalSpacing(4);
    fl->addLayout(m_dimGrid);

    fl->addWidget(subheading("HANDS"));
    m_hand = new QComboBox;
    m_hand->addItems(humanoid::handNames());
    fl->addWidget(m_hand);
    m_handDesc = hint("");
    fl->addWidget(m_handDesc);

    fl->addWidget(subheading("APPEARANCE"));
    auto* look = new QGridLayout;
    look->setHorizontalSpacing(8);
    m_style = new QComboBox;
    m_style->addItems(humanoid::styleNames());
    m_style->setToolTip("Android: smooth shells and ball joints · Mechanical: boxy links and joint drums · "
                        "Slim: a frame of rods that shows every joint");
    m_scheme = new QComboBox;
    for (const robot::ColorScheme& s : robot::colorSchemes()) m_scheme->addItem(swatch(s), s.name);
    m_scheme->setIconSize(QSize(30, 14));
    m_bulk = spin(0.4, 2.5, 2, " ×", 0.05);
    m_bulk->setToolTip("Limb and torso thickness (the joint positions stay the same)");
    int r = 0;
    for (auto [label, w] : {std::pair<const char*, QWidget*>{"Style", m_style}, {"Colours", m_scheme}, {"Thickness", m_bulk}}) {
        look->addWidget(fieldLabel(label), r, 0);
        look->addWidget(w, r++, 1);
    }
    fl->addLayout(look);

    fl->addWidget(subheading("MOTION"));
    m_demo = new QCheckBox("Demo poses and a looping sequence");
    m_demo->setToolTip("Adds T-Pose, Wave, Step and Bow poses and a sequence that runs through them on Play; on "
                       "legs it also walks out, turns round and walks back. Without it you get the Home pose only.\n"
                       "Programs can walk it too (Tools › Robot API).");
    fl->addWidget(m_demo);
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

    // ── Right: drawing and joints ──
    m_tabs = new QTabWidget;
    auto* drawTab = new QWidget;
    auto* dl = new QVBoxLayout(drawTab);
    dl->setContentsMargins(0, 8, 0, 0);
    auto* views = new QHBoxLayout;
    m_views = new QButtonGroup(this);
    const struct { RobotSketch::View view; const char* icon; const char* text; } viewButtons[] = {
        {RobotSketch::View::Front, "person-standing", "Front"},
        {RobotSketch::View::Side, "ruler", "Side"},
        {RobotSketch::View::Top, "radar", "Top"},
    };
    for (const auto& v : viewButtons) {
        auto* b = new QPushButton(Theme::icon(v.icon), v.text);
        b->setObjectName("segment");
        b->setCheckable(true);
        b->setChecked(v.view == RobotSketch::View::Front);
        Theme::segmentIcon(b, v.icon);
        m_views->addButton(b, int(v.view));
        views->addWidget(b);
    }
    auto* names = new QCheckBox("Joint names");
    views->addSpacing(8);
    views->addWidget(names);
    views->addStretch();
    auto* legend = hint("Blue: dimensions · ● turns in this view, ▭ turns about the dashed axis, ○ follows another joint");
    legend->setMaximumWidth(460);
    views->addWidget(legend);
    dl->addLayout(views);
    m_drawing = new RobotSketch;
    m_drawing->setView(RobotSketch::View::Front);
    m_drawing->setJointLabels(false);
    dl->addWidget(m_drawing, 1);
    m_tabs->addTab(drawTab, Theme::icon("drafting-compass"), "Drawing");

    auto* jointsTab = new QWidget;
    auto* jl = new QVBoxLayout(jointsTab);
    jl->setContentsMargins(0, 8, 0, 0);
    auto* jointsTop = new QHBoxLayout;
    jointsTop->addWidget(hint("Tick the joints the humanoid has. Arm and leg joints are built on both sides (L… and "
                              "R…) with mirrored axes. Home is the value it is built at; a joint you leave out is "
                              "rigid."), 1);
    QPushButton* allBtn = new QPushButton("All");
    QPushButton* typicalBtn = new QPushButton("Typical");
    QPushButton* noneBtn = new QPushButton("None");
    allBtn->setToolTip("Every joint in the catalogue");
    typicalBtn->setToolTip("The usual set: 2 waist, 2 neck, 6 per arm, 6 per leg");
    noneBtn->setToolTip("A statue: tick the joints you want");
    for (QPushButton* b : {allBtn, typicalBtn, noneBtn}) jointsTop->addWidget(b);
    jl->addLayout(jointsTop);
    m_joints = new QTableWidget(0, J_Count);
    m_joints->setHorizontalHeaderLabels({"Joint", "Positive value", "Min", "Max", "Home", "Speed", "Acceleration"});
    m_joints->verticalHeader()->hide();
    m_joints->setSelectionMode(QAbstractItemView::NoSelection);
    m_joints->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_joints->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_joints->horizontalHeader()->setSectionResizeMode(J_Name, QHeaderView::ResizeToContents);
    m_joints->horizontalHeader()->setSectionResizeMode(J_Positive, QHeaderView::ResizeToContents);
    m_joints->verticalHeader()->setDefaultSectionSize(30);
    jl->addWidget(m_joints, 1);
    m_tabs->addTab(jointsTab, Theme::icon("sliders-horizontal"), "Joints");
    body->addWidget(m_tabs, 1);

    auto* bottom = new QHBoxLayout;
    m_summary = hint("");
    bottom->addWidget(m_summary, 1);
    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QPushButton* ok = btns->button(QDialogButtonBox::Ok);
    ok->setText("Create Humanoid");
    ok->setIcon(Theme::icon("person-standing", Theme::Tint::OnAccent));
    ok->setDefault(true);
    bottom->addWidget(btns);
    outer->addLayout(bottom);

    // ── Wiring ──
    connect(btns, &QDialogButtonBox::accepted, this, [this] {
        readUi();
        QSettings("MotionStudio", "MotionStudio").setValue(lastSpecKey(),
            QString::fromUtf8(QJsonDocument(humanoid::toJson(spec())).toJson(QJsonDocument::Compact)));
        accept();
    });
    connect(btns, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(m_preset, &QComboBox::activated, this, [this](int i) {
        const int idx = m_preset->itemData(i).toInt();
        if (idx < 0) return;
        loadSpec(humanoid::presets()[idx].spec);
    });
    connect(m_base, &QButtonGroup::idClicked, this, [this](int id) {
        if (humanoid::Base(id) == m_spec.base) return;
        readUi();
        m_spec.base = humanoid::Base(id);
        m_loading = true;
        rebuildDimensions();
        rebuildJoints();
        m_loading = false;
        edited();
    });
    connect(m_height, &QDoubleSpinBox::valueChanged, this, [this](double mm) {
        if (m_loading) return;
        readUi();
        humanoid::scaleToHeight(m_spec, mm / 1000.0);
        m_loading = true;
        for (auto& [key, sp] : m_dims) sp->setValue(m_spec.dim(key) * 1000);
        m_loading = false;
        edited();
    });
    connect(m_name, &QLineEdit::editingFinished, this, &HumanoidBuilderDialog::edited);
    connect(m_hand, &QComboBox::currentIndexChanged, this, &HumanoidBuilderDialog::edited);
    connect(m_style, &QComboBox::currentIndexChanged, this, &HumanoidBuilderDialog::edited);
    connect(m_scheme, &QComboBox::currentIndexChanged, this, &HumanoidBuilderDialog::edited);
    connect(m_bulk, &QDoubleSpinBox::valueChanged, this, &HumanoidBuilderDialog::edited);
    connect(m_demo, &QCheckBox::toggled, this, &HumanoidBuilderDialog::edited);
    connect(m_views, &QButtonGroup::idClicked, this, [this](int id) { m_drawing->setView(RobotSketch::View(id)); });
    connect(names, &QCheckBox::toggled, m_drawing, &RobotSketch::setJointLabels);
    connect(m_drawing, &RobotSketch::dimensionClicked, this, [this](const QString& sym) {
        for (auto& [key, sp] : m_dims)
            if (key == sym) { sp->setFocus(); sp->selectAll(); }
        showDimension(sym);
    });
    auto setAll = [this](int mode) {   // 0 none, 1 typical, 2 all
        readUi();
        for (const humanoid::JointDef& d : humanoid::jointDefs())
            m_spec.joints[d.key].enabled = mode == 2 || (mode == 1 && d.defaults.enabled);
        m_loading = true;
        rebuildJoints();
        m_loading = false;
        edited();
    };
    connect(noneBtn, &QPushButton::clicked, this, [setAll] { setAll(0); });
    connect(typicalBtn, &QPushButton::clicked, this, [setAll] { setAll(1); });
    connect(allBtn, &QPushButton::clicked, this, [setAll] { setAll(2); });

    // Start from the last humanoid designed, else the first preset
    const QString saved = QSettings("MotionStudio", "MotionStudio").value(lastSpecKey()).toString();
    loadSpec(saved.isEmpty() ? humanoid::presets().front().spec
                             : humanoid::fromJson(QJsonDocument::fromJson(saved.toUtf8()).object()));
}

humanoid::Spec HumanoidBuilderDialog::spec() const {
    humanoid::Spec s = m_spec;
    humanoid::validate(s);
    return s;
}

void HumanoidBuilderDialog::fillPresets() {
    m_preset->clear();
    m_preset->addItem(Theme::icon("pencil"), "Custom", -1);
    const std::vector<humanoid::Preset>& ps = humanoid::presets();
    int match = 0;
    for (int i = 0; i < int(ps.size()); ++i) {
        m_preset->addItem(Theme::icon(ps[i].spec.base == humanoid::Base::Legs ? "person-standing" : "cylinder"), ps[i].name, i);
        if (humanoid::toJson(ps[i].spec) == humanoid::toJson(m_spec)) match = m_preset->count() - 1;
    }
    m_preset->setCurrentIndex(match);
    const int idx = m_preset->currentData().toInt();
    m_presetDesc->setText(idx >= 0 ? ps[idx].description : QString(kCustom));
}

void HumanoidBuilderDialog::loadSpec(const humanoid::Spec& in) {
    m_loading = true;
    m_spec = in;
    humanoid::validate(m_spec);
    m_name->setText(m_spec.name);
    m_base->button(int(m_spec.base))->setChecked(true);
    fillPresets();
    rebuildDimensions();
    m_height->setValue(humanoid::height(m_spec) * 1000);
    m_hand->setCurrentIndex(int(m_spec.hand));
    m_handDesc->setText(handDescription(m_spec.hand));
    m_style->setCurrentIndex(int(m_spec.style));
    m_scheme->setCurrentIndex(m_spec.scheme);
    m_bulk->setValue(m_spec.bulk);
    m_demo->setChecked(m_spec.demo);
    rebuildJoints();
    m_problems->clear();
    m_loading = false;
    refresh();
}

void HumanoidBuilderDialog::rebuildDimensions() {
    while (QLayoutItem* it = m_dimGrid->takeAt(0)) {
        delete it->widget();
        delete it;
    }
    m_dims.clear();
    int r = 0;
    for (const robot::DimensionDef& d : humanoid::dimensionDefs(m_spec.base)) {
        auto* sym = new QLabel(d.symbol);
        sym->setObjectName("dimSymbol");
        sym->setMinimumWidth(30);
        auto* label = fieldLabel(d.label);
        label->setToolTip(d.description);
        auto* sp = spin(d.min * 1000, d.max * 1000, 0, " mm", 5);
        sp->setValue(m_spec.dim(d.key) * 1000);
        sp->setToolTip(QString("%1 — %2").arg(d.symbol, d.description));
        sp->setProperty("dimKey", d.key);
        sp->installEventFilter(this);
        connect(sp, &QDoubleSpinBox::valueChanged, this, &HumanoidBuilderDialog::edited);
        m_dimGrid->addWidget(sym, r, 0);
        m_dimGrid->addWidget(label, r, 1);
        m_dimGrid->addWidget(sp, r, 2);
        m_dims.push_back({d.key, sp});
        ++r;
    }
}

void HumanoidBuilderDialog::rebuildJoints() {
    m_joints->clearSpans();
    m_joints->setRowCount(0);
    m_jointRows.clear();
    QString group;
    for (const humanoid::JointDef& d : humanoid::jointDefs()) {
        const bool hidden = d.group == "Legs" && m_spec.base != humanoid::Base::Legs;
        if (d.group != group) {
            group = d.group;
            const int r = m_joints->rowCount();
            m_joints->insertRow(r);
            auto* head = new QTableWidgetItem(d.sided ? QString("%1  (left L…, right R…)").arg(group.toUpper()) : group.toUpper());
            QFont f = head->font();
            f.setBold(true);
            head->setFont(f);
            m_joints->setItem(r, 0, head);
            m_joints->setSpan(r, 0, 1, J_Count);
            m_joints->setRowHidden(r, hidden);
        }
        const int r = m_joints->rowCount();
        m_joints->insertRow(r);
        m_jointRows.push_back({d.key, r});
        const humanoid::JointSpec js = m_spec.joint(d.key);
        auto* on = new QCheckBox(d.sided ? QString("L/R %1").arg(d.key) : d.key);
        on->setChecked(js.enabled);
        on->setToolTip(d.label);
        m_joints->setCellWidget(r, J_Name, on);
        m_joints->setItem(r, J_Positive, new QTableWidgetItem(d.positive));
        auto* mn = spin(-360, 360, 1, "°", 5);
        auto* mx = spin(-360, 360, 1, "°", 5);
        auto* home = spin(-360, 360, 1, "°", 5);
        auto* sp = spin(1, 10000, 0, " °/s", 10);
        auto* ac = spin(0, 200000, 0, " °/s²", 50);
        mn->setValue(js.min);
        mx->setValue(js.max);
        home->setValue(js.home);
        sp->setValue(js.speed);
        ac->setValue(js.accel);
        ac->setToolTip("0 = instant speed changes");
        int c = J_Min;
        for (QDoubleSpinBox* s : {mn, mx, home, sp, ac}) {
            s->setEnabled(js.enabled);
            m_joints->setCellWidget(r, c++, s);
            connect(s, &QDoubleSpinBox::valueChanged, this, &HumanoidBuilderDialog::edited);
        }
        connect(on, &QCheckBox::toggled, this, [this, mn, mx, home, sp, ac](bool en) {
            for (QDoubleSpinBox* s : {mn, mx, home, sp, ac}) s->setEnabled(en);
            edited();
        });
        m_joints->setRowHidden(r, hidden);
    }
}

void HumanoidBuilderDialog::readUi() {
    m_spec.name = m_name->text().trimmed();
    for (auto& [key, sp] : m_dims) m_spec.dims[key] = sp->value() / 1000.0;
    m_spec.hand = humanoid::Hand(m_hand->currentIndex());
    m_spec.style = humanoid::Style(m_style->currentIndex());
    m_spec.scheme = m_scheme->currentIndex();
    m_spec.bulk = m_bulk->value();
    m_spec.demo = m_demo->isChecked();
    for (auto& [key, r] : m_jointRows) {
        auto val = [&](int c) { return static_cast<QDoubleSpinBox*>(m_joints->cellWidget(r, c))->value(); };
        humanoid::JointSpec& j = m_spec.joints[key];
        j.enabled = static_cast<QCheckBox*>(m_joints->cellWidget(r, J_Name))->isChecked();
        j.min = val(J_Min);
        j.max = val(J_Max);
        j.home = val(J_Home);
        j.speed = val(J_Speed);
        j.accel = val(J_Accel);
    }
}

void HumanoidBuilderDialog::markCustom() {
    if (m_preset->currentIndex() != 0) {
        m_preset->setCurrentIndex(0);
        m_presetDesc->setText(kCustom);
    }
}

void HumanoidBuilderDialog::edited() {
    if (m_loading) return;
    readUi();
    humanoid::Spec checked = m_spec;
    const QStringList problems = humanoid::validate(checked);
    m_problems->setText(problems.isEmpty() ? QString() : "⚠ " + problems.join("\n⚠ "));
    m_handDesc->setText(handDescription(m_spec.hand));
    m_loading = true;
    m_height->setValue(humanoid::height(checked) * 1000);
    markCustom();
    m_loading = false;
    m_timer.start();
}

void HumanoidBuilderDialog::refresh() {
    const humanoid::Spec s = spec();
    m_preview = std::make_unique<SceneGraph>();
    const humanoid::Built b = humanoid::build(*m_preview, s);
    std::vector<RobotSketch::Tip> tips = {{b.leftHand, "TCP"}, {b.rightHand, "TCP"}, {b.head, QString()}};
    for (int foot : {b.leftFoot, b.rightFoot})
        if (foot >= 0) tips.push_back({foot, QString()});
    m_drawing->capture(*m_preview, b.root, tips);
    m_drawing->setDimensions(humanoid::dimensionLines(s));
    const int body = humanoid::jointCount(s);
    const int hands = int(b.motors.size()) - body;
    m_summary->setText(QString("%1 mm tall · %2 body joints%3 · %4 actuators · %5 poses%6")
                           .arg(std::round(humanoid::height(s) * 1000)).arg(body)
                           .arg(hands > 0 ? QString(" + %1 hand").arg(hands) : QString())
                           .arg(b.motors.size()).arg(b.poses.size())
                           .arg(b.sequence >= 0 ? " · sequence " + m_preview->getSequence(b.sequence)->name : QString()));
}

void HumanoidBuilderDialog::showDimension(const QString& key) {
    m_drawing->setHighlight(key);
    bool here = false;
    std::optional<RobotSketch::View> other;
    for (const robot::DimensionLine& d : humanoid::dimensionLines(m_spec)) {
        if (d.symbol != key) continue;
        const RobotSketch::View v = d.front ? RobotSketch::View::Front : d.top ? RobotSketch::View::Top : RobotSketch::View::Side;
        if (v == m_drawing->view()) here = true;
        else other = v;
    }
    if (!here && other) {
        m_views->button(int(*other))->setChecked(true);
        m_drawing->setView(*other);
    }
}

bool HumanoidBuilderDialog::eventFilter(QObject* o, QEvent* e) {
    const QString key = o->property("dimKey").toString();
    if (!key.isEmpty()) {
        if (e->type() == QEvent::Enter || e->type() == QEvent::FocusIn) {
            if (e->type() == QEvent::FocusIn) showDimension(key);
            else m_drawing->setHighlight(key);
            if (m_tabs->currentIndex() != 0 && e->type() == QEvent::FocusIn) m_tabs->setCurrentIndex(0);
        } else if (e->type() == QEvent::Leave || e->type() == QEvent::FocusOut) {
            QString focused;
            for (auto& [k, sp] : m_dims) if (sp->hasFocus()) focused = k;
            m_drawing->setHighlight(focused);
        }
    }
    return QDialog::eventFilter(o, e);
}
