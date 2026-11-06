#include "PropertiesPanel.h"
#include "scene/Assembly.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QScrollArea>
#include <QInputDialog>
#include <QColorDialog>
#include <QPainter>
#include <QToolButton>
#include <QSlider>
#include <algorithm>
#include <functional>
#include <cmath>
#include "Theme.h"
#include "automation/AutomationCommon.h"
#include "PrimitiveIcons.h"

// ── Helpers ──────────────────────────────────────────────────────────────────

QDoubleSpinBox* PropertiesPanel::makeSpin(float mn, float mx, float step, const QString& suffix) {
    auto* s = new QDoubleSpinBox;
    s->setRange(mn, mx);
    s->setSingleStep(step);
    s->setDecimals(3);
    s->setAccelerated(true);
    // Shares the row in XYZ rows; labelRow() gives single fields a fixed width
    s->setMinimumWidth(72);
    s->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    if (!suffix.isEmpty()) s->setSuffix(suffix);
    return s;
}

// Length-quantity spinbox: value is always kept in metres internally, but
// its range/step/suffix are rescaled for the active LengthUnit (see onUnitChanged).
QDoubleSpinBox* PropertiesPanel::makeLengthSpin(float mn, float mx, float step, const QString& baseSuffix) {
    auto* s = makeSpin(mn, mx, step, baseSuffix);
    m_lengthFields.push_back({s, mn, mx, step, baseSuffix});
    return s;
}

static QPushButton* iconButton(const QString& icon, const QString& text) {
    return new QPushButton(Theme::icon(icon), text);
}

static QPushButton* dangerButton(const QString& text) {
    auto* b = new QPushButton(Theme::icon("trash-2", Theme::Tint::Danger), text);
    b->setProperty("danger", true);
    return b;
}

// Only " m" and " m/s" suffixes appear on length fields today.
static QString scaledLengthSuffix(const QString& baseSuffix, bool millimeters) {
    if (!millimeters) return baseSuffix;
    if (baseSuffix == " m")   return " mm";
    if (baseSuffix == " m/s") return " mm/s";
    return baseSuffix;
}

// Labeled XYZ row with color-coded X/Y/Z chips.
// Label column is 66px to accommodate "Dimensions".
static QHBoxLayout* xyzRow(const QString& label,
                            QDoubleSpinBox* x, QDoubleSpinBox* y, QDoubleSpinBox* z) {
    auto* row = new QHBoxLayout;
    row->setSpacing(3);
    row->setContentsMargins(0, 0, 0, 0);
    auto chip = [&](const QString& t) {
        auto* l = new QLabel(t);
        l->setObjectName("axis" + t);
        l->setAlignment(Qt::AlignCenter);
        l->setFixedWidth(12);
        return l;
    };
    // Compact fields: no step arrows (wheel, arrow keys and typing still work)
    for (QDoubleSpinBox* s : {x, y, z}) s->setButtonSymbols(QAbstractSpinBox::NoButtons);
    if (!label.isEmpty()) {
        auto* ll = new QLabel(label);
        ll->setFixedWidth(66);
        ll->setObjectName("fieldLabel");
        row->addWidget(ll);
    }
    row->addWidget(chip("X"));  row->addWidget(x);
    row->addWidget(chip("Y"));  row->addWidget(y);
    row->addWidget(chip("Z"));  row->addWidget(z);
    return row;
}

// Single label + spinbox row, used throughout sections.
static void labelRow(QVBoxLayout* vb, const QString& lbl, QWidget* w, int labelWidth = 70) {
    auto* row = new QHBoxLayout;
    auto* l = new QLabel(lbl);
    l->setFixedWidth(labelWidth);
    l->setObjectName("fieldLabel");
    row->addWidget(l);
    if (qobject_cast<QAbstractSpinBox*>(w)) w->setFixedWidth(130);
    row->addWidget(w);
    row->addStretch();
    vb->addLayout(row);
}

// ── Constructor ───────────────────────────────────────────────────────────────

PropertiesPanel::PropertiesPanel(SceneGraph* scene, QWidget* parent)
    : QWidget(parent), m_scene(scene)
{
    setMinimumWidth(260);

    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->setSpacing(0);

    // Shown in the dock's header (see unitCombo())
    m_unitCombo = new QComboBox(this);
    m_unitCombo->addItems({"m", "mm"});
    m_unitCombo->setToolTip("Length display unit (values are still stored in metres)");
    m_unitCombo->setFixedWidth(78);
    connect(m_unitCombo, &QComboBox::currentIndexChanged, this, &PropertiesPanel::onUnitChanged);

    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    outer->addWidget(scroll);

    auto* inner = new QWidget;
    auto* vbox = new QVBoxLayout(inner);
    vbox->setContentsMargins(8, 2, 8, 8);
    vbox->setSpacing(4);

    buildNameSection();
    buildPointSection();
    buildTransformSection();
    buildConnectionsSection();
    buildJointSection();
    buildMotorSection();
    buildPhysicsSection();
    buildConveyorSection();

    // Shown instead of the object's sections while nothing is selected
    m_emptyHint = new QLabel("Nothing selected.\nClick a part in the viewport or the Hierarchy "
                             "to see and edit its properties.");
    m_emptyHint->setObjectName("hint");
    m_emptyHint->setWordWrap(true);
    m_emptyHint->setAlignment(Qt::AlignCenter);
    m_emptyHint->setContentsMargins(8, 24, 8, 8);
    vbox->addWidget(m_emptyHint);

    vbox->addWidget(m_nameLabel->parentWidget());
    vbox->addWidget(m_pointGroup);
    vbox->addWidget(m_transformGroup);
    vbox->addWidget(m_connectionsGroup);
    vbox->addWidget(m_jointGroup);
    vbox->addWidget(m_motorGroup);
    vbox->addWidget(m_physicsGroup);
    vbox->addWidget(m_conveyorGroup);
    vbox->addStretch();
    scroll->setWidget(inner);

    // Initial hidden state
    m_nameLabel->parentWidget()->hide();
    m_pointGroup->hide();
    m_transformGroup->hide();
    showConnectionsGroup(false);
    showJointGroup(false);
    showMotorGroup(false);
    m_physicsGroup->hide();
    m_conveyorGroup->hide();

    connect(scene, &SceneGraph::selectionChanged, this, &PropertiesPanel::onSelectionChanged);
    // Clearing the scene drops the selection without a signal (undo then selects again)
    connect(scene, &SceneGraph::sceneCleared,     this, [this]{ onSelectionChanged(-1); });
    connect(scene, &SceneGraph::objectChanged,    this, [this](int id){
        if (id == m_currentId) loadObject(id);
    });
    connect(scene, &SceneGraph::jointAdded,  this, [this](int){ if (m_currentId != -1) loadObject(m_currentId); });
    connect(scene, &SceneGraph::motorAdded,  this, [this](int){ if (m_currentId != -1) loadObject(m_currentId); });
    connect(scene, &SceneGraph::automationChanged, this, [this]{ if (m_currentId != -1) loadObject(m_currentId); });
    connect(scene, &SceneGraph::jointsMoved, this, [this]{
        Joint* j = m_scene->jointForChild(m_currentId);
        if (!j || m_jPosition->hasFocus()) return;
        const double f = (j->type == JointType::Revolute) ? 1.0 : unitFactor();
        QSignalBlocker b(m_jPosition);
        m_jPosition->setValue(j->currentValue * f);
    });
    connect(scene, &SceneGraph::rigidBodyAdded,   this, [this](int id){
        if (id == m_currentId) { m_loading = true; updatePhysicsUI(id); m_loading = false; }
    });
    connect(scene, &SceneGraph::rigidBodyRemoved, this, [this](int id){
        if (id == m_currentId) { m_loading = true; updatePhysicsUI(id); m_loading = false; }
    });
    connect(scene, &SceneGraph::rigidBodyChanged, this, [this](int id){
        if (id == m_currentId) { m_loading = true; updatePhysicsUI(id); m_loading = false; }
    });
    connect(scene, &SceneGraph::conveyorBeltAdded,  this, [this](int id){
        if (id == m_currentId) { m_loading = true; updateConveyorUI(id); m_loading = false; }
    });
    connect(scene, &SceneGraph::conveyorBeltRemoved,this, [this](int id){
        if (id == m_currentId) { m_loading = true; updateConveyorUI(id); m_loading = false; }
    });
    connect(scene, &SceneGraph::conveyorBeltChanged,this, [this](int id){
        if (id == m_currentId) { m_loading = true; updateConveyorUI(id); m_loading = false; }
    });
}

// ── Section builders ──────────────────────────────────────────────────────────

void PropertiesPanel::buildNameSection() {
    auto* card = new QWidget;
    card->setObjectName("card");
    card->setAttribute(Qt::WA_StyledBackground);
    auto* hb = new QHBoxLayout(card);
    hb->setContentsMargins(12, 10, 12, 10);
    hb->setSpacing(10);
    m_typeIcon = new QLabel;
    m_typeIcon->setFixedSize(26, 26);
    m_typeIcon->setAlignment(Qt::AlignCenter);
    hb->addWidget(m_typeIcon);
    auto* vb = new QVBoxLayout;
    vb->setSpacing(1);
    m_nameLabel = new QLabel("—");
    m_nameLabel->setObjectName("nameTitle");
    m_typeLabel = new QLabel;
    m_typeLabel->setObjectName("hint");
    vb->addWidget(m_nameLabel);
    vb->addWidget(m_typeLabel);
    hb->addLayout(vb, 1);
}

void PropertiesPanel::buildPointSection() {
    m_pointGroup = new QGroupBox("Selected Point");
    auto* vb = new QVBoxLayout(m_pointGroup);
    vb->setSpacing(3);

    m_pointNameLabel = new QLabel("—");
    m_pointNameLabel->setObjectName("mono");
    vb->addWidget(m_pointNameLabel);

    auto mkCoord = [&](const QString& axis) -> QLabel* {
        auto* l = new QLabel(QString("%1: —").arg(axis));
        l->setObjectName("monoHint");
        return l;
    };
    m_ptX = mkCoord("X"); m_ptY = mkCoord("Y"); m_ptZ = mkCoord("Z");
    vb->addWidget(m_ptX); vb->addWidget(m_ptY); vb->addWidget(m_ptZ);
}

void PropertiesPanel::showPointCoords(const QString& name, const QVector3D& pos) {
    m_lastPointName = name;
    m_lastPointPos  = pos;

    m_pointGroup->show();
    m_pointNameLabel->setText(name);
    const double f    = unitFactor();
    const QString unit = m_lengthUnit == LengthUnit::Millimeters ? "mm" : "m";
    const int decimals  = m_lengthUnit == LengthUnit::Millimeters ? 1 : 4;
    m_ptX->setText(QString("X: %1 %2").arg(double(pos.x()) * f, 0, 'f', decimals).arg(unit));
    m_ptY->setText(QString("Y: %1 %2").arg(double(pos.y()) * f, 0, 'f', decimals).arg(unit));
    m_ptZ->setText(QString("Z: %1 %2").arg(double(pos.z()) * f, 0, 'f', decimals).arg(unit));
}

void PropertiesPanel::buildTransformSection() {
    m_transformGroup = new QGroupBox("Transform");
    auto* vb = new QVBoxLayout(m_transformGroup);
    vb->setSpacing(5);

    // Position — stored in metres, displayed in the active unit
    m_px = makeLengthSpin(-1e4, 1e4, 0.01f);
    m_py = makeLengthSpin(-1e4, 1e4, 0.01f);
    m_pz = makeLengthSpin(-1e4, 1e4, 0.01f);

    // Rotation in degrees
    m_rx = makeSpin(-360, 360, 1.0f, "°");
    m_ry = makeSpin(-360, 360, 1.0f, "°");
    m_rz = makeSpin(-360, 360, 1.0f, "°");

    // Dimensions — stored in metres (scale × unit mesh = actual size)
    m_sx = makeLengthSpin(0.001f, 1000, 0.01f);
    m_sy = makeLengthSpin(0.001f, 1000, 0.01f);
    m_sz = makeLengthSpin(0.001f, 1000, 0.01f);
    m_sx->setValue(1); m_sy->setValue(1); m_sz->setValue(1);

    vb->addLayout(xyzRow("Position",   m_px, m_py, m_pz));
    vb->addLayout(xyzRow("Rotation",   m_rx, m_ry, m_rz));
    vb->addLayout(xyzRow("Dimensions", m_sx, m_sy, m_sz));

    m_snapToFloorBtn = iconButton("arrow-down-to-line", "Snap Bottom to Floor");
    m_snapToFloorBtn->setToolTip("Move Y so the object's bottom face rests at Y = 0");
    vb->addWidget(m_snapToFloorBtn);

    // Colour: own, or inherited from the nearest coloured parent; presets
    // are common machine colours
    {
        auto* row = new QHBoxLayout;
        row->setSpacing(3);
        auto* l = new QLabel("Colour");
        l->setObjectName("fieldLabel");
        l->setFixedWidth(70);
        row->addWidget(l);
        m_colorBtn = new QToolButton;
        m_colorBtn->setIconSize(QSize(34, 16));
        m_colorBtn->setToolTip("Choose a colour. Parts without their own colour use their parent's, "
                               "so colouring an assembly colours its parts.");
        row->addWidget(m_colorBtn);
        m_colorHint = new QLabel;
        m_colorHint->setObjectName("hint");
        row->addWidget(m_colorHint, 1);
        m_colorReset = new QToolButton;
        m_colorReset->setIcon(Theme::icon("x", Theme::Tint::Muted));
        m_colorReset->setAutoRaise(true);
        m_colorReset->setToolTip("Remove this part's own colour (use its parent's, or the default)");
        row->addWidget(m_colorReset);
        vb->addLayout(row);

        auto* presets = new QHBoxLayout;
        presets->setSpacing(1);
        presets->addSpacing(70 + presets->spacing());
        static const struct { const char* hex; const char* name; } kPresets[] = {
            {"#9ea8b8", "Steel"}, {"#d0d4da", "Aluminium"}, {"#3a3f47", "Anthracite"}, {"#ff7a1a", "Safety orange"},
            {"#f2c230", "Signal yellow"}, {"#2f6fd6", "Blue"}, {"#2da55a", "Green"}, {"#d63a3a", "Red"},
        };
        for (const auto& p : kPresets) {
            auto* b = new QToolButton;
            QPixmap px(12, 12);
            px.fill(QColor(p.hex));
            b->setIcon(QIcon(px));
            b->setIconSize(QSize(12, 12));
            b->setAutoRaise(true);
            b->setToolTip(p.name);
            const QColor c(p.hex);
            connect(b, &QToolButton::clicked, this, [this, c]{ setObjectColor(c); });
            presets->addWidget(b);
        }
        presets->addStretch();
        vb->addLayout(presets);
        connect(m_colorBtn, &QToolButton::clicked, this, [this]{
            const QColor start = m_scene->colorOf(m_currentId);
            const QColor c = QColorDialog::getColor(start.isValid() ? start : QColor("#9ea8b8"), this, "Part Colour");
            if (c.isValid()) setObjectColor(c);
        });
        connect(m_colorReset, &QToolButton::clicked, this, [this]{ setObjectColor(QColor()); });
    }

    // Opacity: slider for dragging, box for an exact 0–1 value. Multiplies
    // down onto the children, like colour.
    {
        auto* row = new QHBoxLayout;
        row->setSpacing(6);
        auto* l = new QLabel("Opacity");
        l->setObjectName("fieldLabel");
        l->setFixedWidth(70);
        row->addWidget(l);
        m_opacitySlider = new QSlider(Qt::Horizontal);
        m_opacitySlider->setRange(0, 100);
        m_opacitySlider->setSingleStep(1);
        m_opacitySlider->setPageStep(10);
        m_opacitySlider->setValue(100);
        m_opacitySlider->setToolTip("How solid the part looks: 1 is solid, 0 is invisible. An assembly's "
                                    "opacity applies to all its parts. Parts below 0.5 are clicked through.");
        row->addWidget(m_opacitySlider, 1);
        m_opacitySpin = new QDoubleSpinBox;
        m_opacitySpin->setRange(0.0, 1.0);
        m_opacitySpin->setDecimals(2);
        m_opacitySpin->setSingleStep(0.05);
        m_opacitySpin->setValue(1.0);
        m_opacitySpin->setFixedWidth(m_opacitySpin->fontMetrics().horizontalAdvance("0.00") + 44);
        // Typed values apply on Enter, so "0." on the way to 0.5 doesn't hide the part
        m_opacitySpin->setKeyboardTracking(false);
        m_opacitySpin->setToolTip(m_opacitySlider->toolTip());
        row->addWidget(m_opacitySpin);
        vb->addLayout(row);

        m_opacityHint = new QLabel;
        m_opacityHint->setObjectName("hint");
        m_opacityHint->setContentsMargins(70 + row->spacing(), 0, 0, 0);
        m_opacityHint->hide();
        vb->addWidget(m_opacityHint);

        connect(m_opacitySlider, &QSlider::valueChanged, this, [this](int v){ setObjectOpacity(v / 100.0); });
        connect(m_opacitySpin, &QDoubleSpinBox::valueChanged, this, [this](double v){ setObjectOpacity(v); });
    }

    auto wire = [this](QDoubleSpinBox* s){
        connect(s, &QDoubleSpinBox::valueChanged, this, &PropertiesPanel::onTransformEdited);
    };
    for (auto* s : {m_px, m_py, m_pz, m_rx, m_ry, m_rz, m_sx, m_sy, m_sz}) wire(s);
    connect(m_snapToFloorBtn, &QPushButton::clicked, this, &PropertiesPanel::snapToFloor);
}

void PropertiesPanel::setObjectColor(const QColor& c) {
    SceneObject* obj = m_scene->getObject(m_currentId);
    if (!obj || m_loading) return;
    obj->color = c;
    emit m_scene->objectChanged(m_currentId);
    // Children inherit: repaint and reload them too
    std::function<void(int)> touch = [&](int id) {
        if (const SceneObject* o = m_scene->getObject(id))
            for (int child : o->childIds) { emit m_scene->objectChanged(child); touch(child); }
    };
    touch(m_currentId);
    updateColorRow(m_currentId);
}

void PropertiesPanel::updateColorRow(int id) {
    const SceneObject* obj = m_scene->getObject(id);
    if (!obj) return;
    const QColor c = m_scene->colorOf(id);
    QPixmap px(34, 16);
    px.fill(c.isValid() ? c : QColor(0, 0, 0, 0));
    if (!c.isValid()) {
        QPainter p(&px);
        p.setPen(QPen(Theme::colors().muted, 1, Qt::DashLine));
        p.drawRect(px.rect().adjusted(0, 0, -1, -1));
    }
    m_colorBtn->setIcon(QIcon(px));
    m_colorReset->setEnabled(obj->color.isValid());
    const int source = m_scene->colorSource(id);
    if (obj->color.isValid()) m_colorHint->setText(obj->color.name().toUpper());
    else if (source != -1) m_colorHint->setText("Inherited from " + m_scene->getObject(source)->name);
    else m_colorHint->setText("Default colour");
}

void PropertiesPanel::setObjectOpacity(double opacity) {
    SceneObject* obj = m_scene->getObject(m_currentId);
    if (!obj || m_loading) return;
    const float v = float(std::clamp(opacity, 0.0, 1.0));
    if (std::abs(obj->opacity - v) < 1e-4f) return;
    obj->opacity = v;
    updateOpacityRow(m_currentId);           // keep the slider and the box together
    emit m_scene->objectChanged(m_currentId);   // repaints, and one undo step per drag
}

void PropertiesPanel::updateOpacityRow(int id) {
    const SceneObject* obj = m_scene->getObject(id);
    if (!obj) return;
    {
        QSignalBlocker a(m_opacitySlider), b(m_opacitySpin);
        m_opacitySlider->setValue(int(std::lround(obj->opacity * 100.0f)));
        if (!m_opacitySpin->hasFocus() || std::abs(m_opacitySpin->value() - obj->opacity) > 0.005)
            m_opacitySpin->setValue(obj->opacity);
    }
    // A faded parent fades this part too: say so, or the slider looks wrong
    // Hidden with Alt+V (this part or a parent) says that instead
    if (const int by = m_scene->hiddenBy(id); by != -1) {
        m_opacityHint->setVisible(true);
        m_opacityHint->setText(by == id ? QString("Hidden  —  Alt+V shows it")
                                        : QString("Hidden with \"%1\"").arg(m_scene->getObject(by)->name));
        return;
    }
    const float shown = m_scene->opacityOf(id);
    m_opacityHint->setVisible(std::abs(shown - obj->opacity) > 1e-3f);
    m_opacityHint->setText(QString("Shown at %1 (a parent is faded too)").arg(shown, 0, 'f', 2));
}

void PropertiesPanel::buildConnectionsSection() {
    m_connectionsGroup = new QGroupBox("Connections");
    auto* vb = new QVBoxLayout(m_connectionsGroup);
    vb->setSpacing(4);

    auto* info = new QLabel("This object has a parent but no joint.\nAdd a joint to enable motion.");
    info->setWordWrap(true);
    info->setObjectName("hint");
    vb->addWidget(info);

    m_addJointBtn = iconButton("rotate-3d", "Add Joint to Parent");
    m_addJointBtn->setToolTip("Create a Revolute joint connecting this object to its parent");
    vb->addWidget(m_addJointBtn);

    connect(m_addJointBtn, &QPushButton::clicked, this, &PropertiesPanel::addJoint);
}

void PropertiesPanel::buildJointSection() {
    m_jointGroup = new QGroupBox("Joint");
    auto* vb = new QVBoxLayout(m_jointGroup);
    vb->setSpacing(5);

    m_jType = new QComboBox;
    m_jType->addItems({"Revolute  (rotation)", "Prismatic  (slide)"});
    {
        auto* row = new QHBoxLayout;
        auto* l = new QLabel("Type"); l->setFixedWidth(70);
        l->setObjectName("fieldLabel");
        row->addWidget(l); row->addWidget(m_jType, 1);
        vb->addLayout(row);
    }

    m_jAxisX = makeSpin(-1, 1, 0.1f);
    m_jAxisY = makeSpin(-1, 1, 0.1f);
    m_jAxisZ = makeSpin(-1, 1, 0.1f);
    for (auto* s : {m_jAxisX, m_jAxisY, m_jAxisZ}) {
        s->setDecimals(4);   // picked axes are rarely round numbers
        s->setToolTip("Joint axis in the parent's frame");
    }
    m_jAxisY->setValue(1);
    vb->addLayout(xyzRow("Axis", m_jAxisX, m_jAxisY, m_jAxisZ));

    m_jPickAxisBtn = iconButton("crosshair", "Pick Axis on Model…");
    m_jPickAxisBtn->setToolTip("Click a round edge or cylindrical face (its centre line), a straight edge, or a "
                               "flat face (its normal) on any part. Sets the axis and the rotation origin.\n"
                               "Clicking one of the gizmo's arrows picks that world axis instead.");
    m_jFlipAxisBtn = iconButton("arrow-left-right", "Flip");
    m_jFlipAxisBtn->setToolTip("Reverse the axis, so positive values turn (or slide) the other way");
    {
        auto* row = new QHBoxLayout;
        row->addWidget(m_jPickAxisBtn, 1);
        row->addWidget(m_jFlipAxisBtn);
        vb->addLayout(row);
    }

    // Suffix "°" for revolute, " m" for prismatic — set in updateJointUnits()
    m_jLimitMin  = makeSpin(-1e4, 1e4, 1.0f, "°"); m_jLimitMin->setValue(-180);
    m_jLimitMax  = makeSpin(-1e4, 1e4, 1.0f, "°"); m_jLimitMax->setValue(180);
    m_jInitAngle = makeSpin(-1e4, 1e4, 1.0f, "°");

    m_jInitLabel = new QLabel("Init angle");
    m_jInitLabel->setFixedWidth(70);
    m_jInitLabel->setObjectName("fieldLabel");

    m_jContinuous = new QCheckBox("Continuous (no limits)");
    m_jContinuous->setToolTip("Revolute only: allow unlimited rotation (wheels, turntables)");
    vb->addWidget(m_jContinuous);
    labelRow(vb, "Limit min",  m_jLimitMin);
    labelRow(vb, "Limit max",  m_jLimitMax);
    {
        auto* row = new QHBoxLayout;
        m_jInitAngle->setFixedWidth(130);
        row->addWidget(m_jInitLabel);
        row->addWidget(m_jInitAngle);
        row->addStretch();
        vb->addLayout(row);
    }

    // Rotation origin (revolute only) — direct numeric entry
    m_jPivotRow = new QWidget;
    {
        auto* pvb = new QVBoxLayout(m_jPivotRow);
        pvb->setContentsMargins(0, 0, 0, 0);
        pvb->setSpacing(5);
        m_jPivotX = makeLengthSpin(-1e4, 1e4, 0.01f);
        m_jPivotY = makeLengthSpin(-1e4, 1e4, 0.01f);
        m_jPivotZ = makeLengthSpin(-1e4, 1e4, 0.01f);
        pvb->addLayout(xyzRow("Rotation Origin", m_jPivotX, m_jPivotY, m_jPivotZ));
    }
    vb->addWidget(m_jPivotRow);

    // Live joint value — jogging here poses the mechanism (not saved; see Init angle)
    m_jPosition = makeSpin(-1e4, 1e4, 1.0f, "°");
    m_jPosition->setKeyboardTracking(true);
    m_jPosition->setToolTip("Current joint value. Jog to pose the mechanism; the simulation starts from here.");
    m_jPosLabel = new QLabel("Position");
    m_jPosLabel->setFixedWidth(70);
    m_jPosLabel->setObjectName("fieldLabel");
    {
        auto* row = new QHBoxLayout;
        m_jPosition->setFixedWidth(130);
        row->addWidget(m_jPosLabel);
        row->addWidget(m_jPosition);
        row->addStretch();
        vb->addLayout(row);
    }

    // Coupling: follow another joint (gears, belts, rack and pinion)
    m_jCouple = new QComboBox;
    m_jCouple->setToolTip("Make this joint follow another one: value = ratio × that joint + offset. "
                          "For gears, belts, rack and pinion, or two fingers of a gripper. "
                          "A follower needs no actuator.");
    {
        auto* row = new QHBoxLayout;
        auto* l = new QLabel("Follows"); l->setFixedWidth(70);
        l->setObjectName("fieldLabel");
        row->addWidget(l); row->addWidget(m_jCouple, 1);
        vb->addLayout(row);
    }
    m_jCoupleRow = new QWidget;
    {
        auto* row = new QHBoxLayout(m_jCoupleRow);
        row->setContentsMargins(0, 0, 0, 0);
        auto* l = new QLabel("Ratio"); l->setFixedWidth(70);
        l->setObjectName("fieldLabel");
        m_jRatio = makeSpin(-1e4, 1e4, 0.1f);
        m_jRatio->setDecimals(5);
        m_jRatio->setToolTip("Follower value per unit of the master: −0.5 is a 2:1 gear turning the other "
                             "way; a rack on a pinion of radius r moves 2πr/360 m per degree");
        m_jOffset = makeSpin(-1e4, 1e4, 1.0f);
        m_jOffset->setDecimals(4);
        m_jOffset->setToolTip("Added to the follower's value, in its own units");
        auto* lo = new QLabel("Offset");
        lo->setObjectName("fieldLabel");
        row->addWidget(l); row->addWidget(m_jRatio, 1);
        row->addWidget(lo); row->addWidget(m_jOffset, 1);
    }
    vb->addWidget(m_jCoupleRow);
    connect(m_jCouple, &QComboBox::currentIndexChanged, this, &PropertiesPanel::onCouplingEdited);
    connect(m_jRatio,  &QDoubleSpinBox::valueChanged,   this, &PropertiesPanel::onCouplingEdited);
    connect(m_jOffset, &QDoubleSpinBox::valueChanged,   this, &PropertiesPanel::onCouplingEdited);

    m_addMotorBtn = iconButton("cog", "Add Actuator to this Joint");
    m_addMotorBtn->setToolTip("Attach an actuator so automation logic can drive this joint");
    vb->addWidget(m_addMotorBtn);

    auto* btnRow = new QHBoxLayout;
    auto* rmBtn = dangerButton("Remove Joint");
    btnRow->addStretch(); btnRow->addWidget(rmBtn);
    vb->addLayout(btnRow);

    connect(m_jType,      &QComboBox::currentIndexChanged,  this, &PropertiesPanel::onJointEdited);
    connect(m_jAxisX,     &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jAxisY,     &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jAxisZ,     &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jPickAxisBtn, &QPushButton::clicked, this, &PropertiesPanel::pickAxisRequested);
    connect(m_jFlipAxisBtn, &QPushButton::clicked, this, [this]{
        Joint* j = m_scene->jointForChild(m_currentId);
        if (!j) return;
        j->axis = -j->axis;
        emit m_scene->objectChanged(m_currentId);
    });
    connect(m_jLimitMin,  &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jLimitMax,  &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jInitAngle, &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jPivotX,    &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jPivotY,    &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jPivotZ,    &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jContinuous, &QCheckBox::toggled,             this, &PropertiesPanel::onJointEdited);
    connect(m_jPosition,  &QDoubleSpinBox::valueChanged,    this, [this](double v){
        if (m_loading) return;
        Joint* j = m_scene->jointForChild(m_currentId);
        if (!j || m_simActive) return;
        const double f = (j->type == JointType::Revolute) ? 1.0 : unitFactor();
        j->currentValue = float(v / f);
        m_scene->notifyJointsMoved();
    });
    connect(m_addMotorBtn, &QPushButton::clicked,           this, &PropertiesPanel::addMotor);
    connect(rmBtn,        &QPushButton::clicked,            this, &PropertiesPanel::removeJoint);
}

void PropertiesPanel::buildMotorSection() {
    m_motorGroup = new QGroupBox("Actuator");
    auto* vb = new QVBoxLayout(m_motorGroup);
    vb->setSpacing(5);

    m_mName = new QLabel;
    m_mName->setObjectName("mono");
    m_mName->setToolTip("Name used in expressions (rename it in the Automation panel)");
    labelRow(vb, "Name", m_mName);

    m_mMode = new QComboBox;
    m_mMode->addItems({"Position", "Speed"});
    m_mMode->setToolTip("Position: holds still until logic commands a target.\n"
                        "Speed: runs continuously at the speed below (when active).");
    labelRow(vb, "Mode", m_mMode);

    // Suffix "°/s" or " m/s" — updated in updateJointUnits()
    m_mVel   = makeSpin(-9999, 9999, 5.0f, "°/s");
    m_mAccel = makeSpin(0, 1e6, 10.0f, "°/s²");
    m_mAccel->setSpecialValueText("instant");
    m_mEnabled = new QCheckBox("Active at start");
    m_mEnabled->setChecked(true);
    m_mVel->setValue(45);

    labelRow(vb, "Speed", m_mVel);
    labelRow(vb, "Accel", m_mAccel);
    vb->addWidget(m_mEnabled);

    m_mBindings = new QLabel;
    m_mBindings->setWordWrap(true);
    m_mBindings->setObjectName("monoHint");
    vb->addWidget(m_mBindings);

    auto* openBtn = iconButton("workflow", "Edit Logic in Automation Panel…");
    vb->addWidget(openBtn);

    auto* btnRow = new QHBoxLayout;
    auto* rmBtn = dangerButton("Remove Actuator");
    btnRow->addStretch(); btnRow->addWidget(rmBtn);
    vb->addLayout(btnRow);

    connect(m_mMode,   &QComboBox::currentIndexChanged, this, &PropertiesPanel::onMotorEdited);
    connect(m_mVel,    &QDoubleSpinBox::valueChanged, this, &PropertiesPanel::onMotorEdited);
    connect(m_mAccel,  &QDoubleSpinBox::valueChanged, this, &PropertiesPanel::onMotorEdited);
    connect(m_mEnabled, &QCheckBox::toggled,          this, &PropertiesPanel::onMotorEdited);
    connect(rmBtn,     &QPushButton::clicked,         this, &PropertiesPanel::removeMotor);
    connect(openBtn,   &QPushButton::clicked,         this, [this]{
        if (Joint* j = m_scene->jointForChild(m_currentId)) emit openActuatorRequested(j->id);
    });
}

void PropertiesPanel::buildPhysicsSection() {
    m_physicsGroup = new QGroupBox("Physics");
    auto* vb = new QVBoxLayout(m_physicsGroup);
    vb->setSpacing(5);

    // ── No rigid body: just an add button ──────────────────────────────────
    m_rbAddBtn = iconButton("weight", "Add Rigid Body");
    m_rbAddBtn->setToolTip("Make this object a physics actor (mass, gravity, collision)");
    vb->addWidget(m_rbAddBtn);

    // ── Rigid body properties (visible when rigid body exists) ────────────
    m_rbPropsWidget = new QWidget;
    auto* rbvb = new QVBoxLayout(m_rbPropsWidget);
    rbvb->setContentsMargins(0, 0, 0, 0);
    rbvb->setSpacing(5);

    m_rbMass = makeSpin(0.001f, 1e6f, 0.1f, " kg");
    m_rbMass->setValue(1.0f);
    labelRow(rbvb, "Mass", m_rbMass);

    m_rbGravity = new QCheckBox("Gravity enabled");
    m_rbGravity->setChecked(true);
    rbvb->addWidget(m_rbGravity);

    // Add collision button (shown when no collision yet)
    m_colAddBtn = iconButton("square-dashed", "Add Collision Body");
    m_colAddBtn->setToolTip("Add a collision shape matching this primitive's dimensions");
    rbvb->addWidget(m_colAddBtn);

    // ── Collision body sub-section ────────────────────────────────────────
    m_colWidget = new QWidget;
    auto* colvb = new QVBoxLayout(m_colWidget);
    colvb->setContentsMargins(0, 0, 0, 0);
    colvb->setSpacing(4);

    auto* colHdr = new QLabel("COLLISION BODY");
    colHdr->setObjectName("subheading");
    colvb->addWidget(colHdr);

    m_colType = new QComboBox;
    m_colType->addItems({"Box", "Cylinder", "Sphere", "Capsule"});
    labelRow(colvb, "Type", m_colType);

    // Box row: half-extents W / H / D
    m_colBoxRow = new QWidget;
    {
        auto* bvb = new QVBoxLayout(m_colBoxRow);
        bvb->setContentsMargins(0, 0, 0, 0);
        bvb->setSpacing(3);
        m_cHW = makeLengthSpin(0.001f, 1e4f, 0.01f); m_cHW->setValue(0.5f);
        m_cHH = makeLengthSpin(0.001f, 1e4f, 0.01f); m_cHH->setValue(0.5f);
        m_cHD = makeLengthSpin(0.001f, 1e4f, 0.01f); m_cHD->setValue(0.5f);
        labelRow(bvb, "Half W", m_cHW);
        labelRow(bvb, "Half H", m_cHH);
        labelRow(bvb, "Half D", m_cHD);
    }
    colvb->addWidget(m_colBoxRow);

    // Radius row (Cylinder / Sphere / Capsule)
    m_colRadiusRow = new QWidget;
    {
        auto* rvb = new QVBoxLayout(m_colRadiusRow);
        rvb->setContentsMargins(0, 0, 0, 0);
        m_cRadius = makeLengthSpin(0.001f, 1e4f, 0.01f); m_cRadius->setValue(0.5f);
        labelRow(rvb, "Radius", m_cRadius);
    }
    colvb->addWidget(m_colRadiusRow);

    // Height row (Cylinder / Capsule)
    m_colHeightRow = new QWidget;
    {
        auto* hvb = new QVBoxLayout(m_colHeightRow);
        hvb->setContentsMargins(0, 0, 0, 0);
        m_cHeight = makeLengthSpin(0.001f, 1e4f, 0.01f); m_cHeight->setValue(1.0f);
        labelRow(hvb, "Height", m_cHeight);
    }
    colvb->addWidget(m_colHeightRow);

    // Offset XYZ
    m_cOffX = makeLengthSpin(-1e4f, 1e4f, 0.01f);
    m_cOffY = makeLengthSpin(-1e4f, 1e4f, 0.01f);
    m_cOffZ = makeLengthSpin(-1e4f, 1e4f, 0.01f);
    colvb->addLayout(xyzRow("Offset", m_cOffX, m_cOffY, m_cOffZ));

    m_colRemBtn = dangerButton("Remove Collision Body");
    colvb->addWidget(m_colRemBtn);

    rbvb->addWidget(m_colWidget);

    // Remove rigid body (always at bottom of rb section)
    {
        auto* row = new QHBoxLayout;
        m_rbRemBtn = dangerButton("Remove Rigid Body");
        row->addStretch();
        row->addWidget(m_rbRemBtn);
        rbvb->addLayout(row);
    }

    vb->addWidget(m_rbPropsWidget);

    // Connections
    connect(m_rbAddBtn,  &QPushButton::clicked, this, &PropertiesPanel::addRigidBody);
    connect(m_rbRemBtn,  &QPushButton::clicked, this, &PropertiesPanel::removeRigidBody);
    connect(m_colAddBtn, &QPushButton::clicked, this, &PropertiesPanel::addCollision);
    connect(m_colRemBtn, &QPushButton::clicked, this, &PropertiesPanel::removeCollision);
    connect(m_rbMass,    &QDoubleSpinBox::valueChanged, this, &PropertiesPanel::onRigidBodyEdited);
    connect(m_rbGravity, &QCheckBox::toggled,           this, &PropertiesPanel::onRigidBodyEdited);
    connect(m_colType,   &QComboBox::currentIndexChanged, this, &PropertiesPanel::onCollisionTypeChanged);
    for (auto* s : {m_cHW, m_cHH, m_cHD, m_cRadius, m_cHeight, m_cOffX, m_cOffY, m_cOffZ})
        connect(s, &QDoubleSpinBox::valueChanged, this, &PropertiesPanel::onCollisionEdited);
}

void PropertiesPanel::buildConveyorSection() {
    m_conveyorGroup = new QGroupBox("Conveyor Belt");
    auto* vb = new QVBoxLayout(m_conveyorGroup);
    vb->setSpacing(5);

    m_cvAddBtn = iconButton("chevrons-right", "Add Conveyor Belt");
    m_cvAddBtn->setToolTip("Make this surface push objects placed on top of it");
    vb->addWidget(m_cvAddBtn);

    m_cvPropsWidget = new QWidget;
    auto* pvb = new QVBoxLayout(m_cvPropsWidget);
    pvb->setContentsMargins(0, 0, 0, 0);
    pvb->setSpacing(5);

    m_cvVelocity = makeLengthSpin(-100.f, 100.f, 0.1f, " m/s");
    m_cvVelocity->setValue(1.0f);
    labelRow(pvb, "Velocity", m_cvVelocity);

    m_cvDir = new QComboBox;
    m_cvDir->addItems({"+X  (→)", "-X  (←)", "+Z  (into)", "-Z  (out)"});
    {
        auto* row = new QHBoxLayout;
        auto* l = new QLabel("Direction"); l->setFixedWidth(70);
        l->setObjectName("fieldLabel");
        row->addWidget(l); row->addWidget(m_cvDir, 1);
        pvb->addLayout(row);
    }

    m_cvEnabled = new QCheckBox("Belt enabled");
    m_cvEnabled->setChecked(true);
    pvb->addWidget(m_cvEnabled);

    m_cvSpeedExpr = autoui::makeExprEdit(m_scene, autoui::ExprKind::Value, "fixed velocity");
    m_cvSpeedExpr->setHelpText("Optional: speed in m/s computed every scan while simulating, overriding "
                               "Velocity. E.g. PartAtEnd ? 0 : 0.3 stops the belt while a sensor sees a part.");
    labelRow(pvb, "Speed expr", m_cvSpeedExpr);
    connect(m_cvSpeedExpr, &ExpressionEdit::committed, this, [this](const QString& text){
        if (m_loading) return;
        ConveyorBelt* belt = m_scene->getConveyorBelt(m_currentId);
        if (!belt || belt->speedExpr == text) return;
        belt->speedExpr = text;
        emit m_scene->conveyorBeltChanged(m_currentId);
        m_scene->notifyAutomationChanged();
    });

    auto* btnRow = new QHBoxLayout;
    m_cvRemBtn = dangerButton("Remove Conveyor Belt");
    btnRow->addStretch(); btnRow->addWidget(m_cvRemBtn);
    pvb->addLayout(btnRow);

    vb->addWidget(m_cvPropsWidget);

    connect(m_cvAddBtn,  &QPushButton::clicked,               this, &PropertiesPanel::addConveyorBelt);
    connect(m_cvRemBtn,  &QPushButton::clicked,               this, &PropertiesPanel::removeConveyorBelt);
    connect(m_cvVelocity,&QDoubleSpinBox::valueChanged,       this, &PropertiesPanel::onConveyorEdited);
    connect(m_cvDir,     &QComboBox::currentIndexChanged,     this, &PropertiesPanel::onConveyorEdited);
    connect(m_cvEnabled, &QCheckBox::toggled,                 this, &PropertiesPanel::onConveyorEdited);
}

// ── Visibility helpers ────────────────────────────────────────────────────────

void PropertiesPanel::showConnectionsGroup(bool v) { m_connectionsGroup->setVisible(v); }
void PropertiesPanel::showJointGroup(bool v)        { m_jointGroup->setVisible(v); }
void PropertiesPanel::showMotorGroup(bool v)        { m_motorGroup->setVisible(v); }

void PropertiesPanel::updateCollisionRows(int idx) {
    // 0=Box 1=Cylinder 2=Sphere 3=Capsule
    m_colBoxRow->setVisible(idx == 0);
    m_colRadiusRow->setVisible(idx != 0);
    m_colHeightRow->setVisible(idx == 1 || idx == 3);
}

void PropertiesPanel::updateJointUnits(bool isRevolute) {
    const bool mm = !isRevolute && m_lengthUnit == LengthUnit::Millimeters;
    const double f = isRevolute ? 1.0 : unitFactor();

    QString limSfx = isRevolute ? "°"   : scaledLengthSuffix(" m", mm);
    QString velSfx = isRevolute ? "°/s" : scaledLengthSuffix(" m/s", mm);
    QString tgtSfx = isRevolute ? "°"   : scaledLengthSuffix(" m", mm);
    m_jLimitMin->setSuffix(limSfx);
    m_jLimitMax->setSuffix(limSfx);
    m_jInitAngle->setSuffix(limSfx);
    m_jInitLabel->setText(isRevolute ? "Init angle" : "Init pos");
    m_mVel->setSuffix(velSfx);
    m_mAccel->setSuffix(isRevolute ? "°/s²" : scaledLengthSuffix(" m", mm) + "/s²");
    m_jPosition->setSuffix(tgtSfx);
    m_jContinuous->setVisible(isRevolute);

    // Prismatic limits/velocity are length quantities — rescale range for the active unit.
    // Revolute (degree) ranges are unaffected since f == 1.0 in that case.
    m_jLimitMin->setRange(-1e4 * f, 1e4 * f);
    m_jLimitMax->setRange(-1e4 * f, 1e4 * f);
    m_jInitAngle->setRange(-1e4 * f, 1e4 * f);
    m_mVel->setRange(-9999 * f, 9999 * f);
    m_mAccel->setRange(0, 1e6 * f);
    m_jPosition->setRange(-1e4 * f, 1e4 * f);

    m_jLimitMin->setSingleStep(1.0 * f);
    m_jLimitMax->setSingleStep(1.0 * f);
    m_jInitAngle->setSingleStep(1.0 * f);
    m_mVel->setSingleStep(5.0 * f);
    m_mAccel->setSingleStep(10.0 * f);
    m_jPosition->setSingleStep(1.0 * f);
}

void PropertiesPanel::updatePhysicsUI(int objectId) {
    const SceneObject* obj = m_scene->getObject(objectId);
    bool isBasket = obj && obj->type == PrimitiveType::Basket;

    RigidBody* rb = m_scene->getRigidBody(objectId);
    // Name card icon: the primitive, tinted when it is a physics actor
    if (obj)
        Theme::setLabelIcon(m_typeIcon, primitiveIconName(obj->type, obj->mesh != nullptr), 22,
                            rb ? Theme::Tint::Physics : Theme::Tint::Text);
    if (!rb) {
        m_rbAddBtn->show();
        m_rbPropsWidget->hide();
    } else {
        m_rbAddBtn->hide();
        m_rbPropsWidget->show();
        m_rbMass->setValue(rb->mass);
        m_rbGravity->setChecked(rb->gravityEnabled);
        // Basket always uses auto-generated 5-wall collision — hide the custom collision section
        if (isBasket) {
            m_colAddBtn->hide();
            m_colWidget->hide();
        } else if (rb->hasCollision) {
            m_colAddBtn->hide();
            m_colWidget->show();
            m_colType->setCurrentIndex(int(rb->collision.type));
            const double f = unitFactor();
            m_cHW->setValue(rb->collision.halfW * f);
            m_cHH->setValue(rb->collision.halfH * f);
            m_cHD->setValue(rb->collision.halfD * f);
            m_cRadius->setValue(rb->collision.radius * f);
            m_cHeight->setValue(rb->collision.height * f);
            m_cOffX->setValue(rb->collision.offset.x() * f);
            m_cOffY->setValue(rb->collision.offset.y() * f);
            m_cOffZ->setValue(rb->collision.offset.z() * f);
            updateCollisionRows(int(rb->collision.type));
        } else {
            m_colAddBtn->show();
            m_colWidget->hide();
        }
    }
}

void PropertiesPanel::updateConveyorUI(int objectId) {
    ConveyorBelt* belt = m_scene->getConveyorBelt(objectId);
    if (!belt) {
        m_cvAddBtn->show();
        m_cvPropsWidget->hide();
    } else {
        m_cvAddBtn->hide();
        m_cvPropsWidget->show();
        m_cvVelocity->setValue(belt->velocity * unitFactor());
        // Map direction to combo index
        QVector3D d = belt->direction;
        int idx = 0;
        if      (d.x() > 0.5f)  idx = 0;
        else if (d.x() < -0.5f) idx = 1;
        else if (d.z() > 0.5f)  idx = 2;
        else                     idx = 3;
        m_cvDir->setCurrentIndex(idx);
        m_cvEnabled->setChecked(belt->enabled);
        m_cvSpeedExpr->setExpression(belt->speedExpr);
    }
}

// ── Load / selection ──────────────────────────────────────────────────────────

void PropertiesPanel::onSelectionChanged(int id) {
    if (!m_scene->getObject(id)) id = -1;
    m_currentId = id;
    m_emptyHint->setVisible(id == -1);
    if (id == -1) {
        m_nameLabel->parentWidget()->hide();
        m_transformGroup->hide();
        showConnectionsGroup(false);
        showJointGroup(false);
        showMotorGroup(false);
        m_physicsGroup->hide();
        m_conveyorGroup->hide();
        return;
    }
    m_nameLabel->parentWidget()->show();
    m_transformGroup->show();
    m_physicsGroup->show();
    m_conveyorGroup->show();
    loadObject(id);
}

void PropertiesPanel::loadObject(int id) {
    const SceneObject* obj = m_scene->getObject(id);
    if (!obj) return;
    m_loading = true;

    // Name card
    m_nameLabel->setText(obj->name);
    static const char* typeNames[] = {"Cube", "Cylinder", "Sphere", "Capsule", "Basket", "Imported Mesh"};
    QString parentInfo = obj->parentId == -1 ? "Root object" :
        "Child of: " + (m_scene->getObject(obj->parentId) ? m_scene->getObject(obj->parentId)->name : "?");
    m_typeLabel->setText(QString("%1  ·  %2").arg(typeNames[int(obj->type)], parentInfo));

    // Transform
    const double lf = unitFactor();
    m_px->setValue(obj->transform.position.x() * lf);
    m_py->setValue(obj->transform.position.y() * lf);
    m_pz->setValue(obj->transform.position.z() * lf);
    m_rx->setValue(obj->transform.rotation.x());
    m_ry->setValue(obj->transform.rotation.y());
    m_rz->setValue(obj->transform.rotation.z());
    m_sx->setValue(obj->transform.scale.x() * lf);
    m_sy->setValue(obj->transform.scale.y() * lf);
    m_sz->setValue(obj->transform.scale.z() * lf);
    updateColorRow(id);
    updateOpacityRow(id);

    // Joint / Motor
    Joint* joint = m_scene->jointForChild(id);
    bool hasParent = obj->parentId != -1;

    showConnectionsGroup(hasParent && !joint);
    if (joint) {
        bool isRevolute = (joint->type == JointType::Revolute);
        updateJointUnits(isRevolute);

        const double jf = isRevolute ? 1.0 : unitFactor();
        m_jType->setCurrentIndex(isRevolute ? 0 : 1);
        m_jAxisX->setValue(joint->axis.x());
        m_jAxisY->setValue(joint->axis.y());
        m_jAxisZ->setValue(joint->axis.z());
        m_jLimitMin->setValue(joint->limitMin * jf);
        m_jLimitMax->setValue(joint->limitMax * jf);
        m_jInitAngle->setValue(joint->initialAngle * jf);
        m_jPivotX->setValue(joint->pivot.x() * lf);
        m_jPivotY->setValue(joint->pivot.y() * lf);
        m_jPivotZ->setValue(joint->pivot.z() * lf);
        m_jPivotRow->setVisible(isRevolute);
        m_jContinuous->setChecked(joint->continuous);
        bool limited = !(isRevolute && joint->continuous);
        m_jLimitMin->setEnabled(limited);
        m_jLimitMax->setEnabled(limited);
        m_jPosition->setValue(joint->currentValue * jf);
        fillCouplingCombo(*joint);
        m_jRatio->setValue(joint->ratio);
        m_jOffset->setValue(joint->offset * jf);
        m_jOffset->setSuffix(isRevolute ? "°" : " " + m_jPosition->suffix().trimmed());
        m_jCoupleRow->setVisible(joint->coupledTo != -1);
        m_jPosition->setEnabled(joint->coupledTo == -1 && !m_simActive);
        showJointGroup(true);

        Motor* motor = m_scene->motorForJoint(joint->id);
        m_addMotorBtn->setVisible(!motor);
        if (motor) {
            m_mName->setText(motor->name);
            m_mMode->setCurrentIndex(motor->mode == ActuatorMode::Speed ? 1 : 0);
            m_mVel->setValue(motor->velocity * jf);
            m_mAccel->setValue(motor->acceleration * jf);
            m_mEnabled->setChecked(motor->enabled);
            QStringList b;
            if (!motor->targetExpr.trimmed().isEmpty())   b << "target = " + motor->targetExpr.trimmed();
            if (!motor->velocityExpr.trimmed().isEmpty()) b << "speed = " + motor->velocityExpr.trimmed();
            if (!motor->activeExpr.trimmed().isEmpty())   b << "active = " + motor->activeExpr.trimmed();
            m_mBindings->setText(b.isEmpty() ? QString("No bindings — driven by sequence operations.")
                                             : "Bindings:\n" + b.join("\n"));
            showMotorGroup(true);
        } else {
            showMotorGroup(false);
        }
    } else {
        showJointGroup(false);
        showMotorGroup(false);
    }

    // Physics
    updatePhysicsUI(id);
    updateConveyorUI(id);

    m_loading = false;
}

// ── Write-back ────────────────────────────────────────────────────────────────

void PropertiesPanel::onTransformEdited() {
    if (m_loading) return;
    SceneObject* obj = m_scene->getObject(m_currentId);
    if (!obj) return;
    const double f = unitFactor();
    obj->transform.position = { float(m_px->value() / f), float(m_py->value() / f), float(m_pz->value() / f) };
    obj->transform.rotation = { float(m_rx->value()), float(m_ry->value()), float(m_rz->value()) };
    obj->transform.scale    = { float(m_sx->value() / f), float(m_sy->value() / f), float(m_sz->value() / f) };
    emit m_scene->objectChanged(m_currentId);
    // Mates hold: the part keeps only the motion they leave free, and parts
    // mated to it follow
    if (!m_simActive && !m_scene->allMates().empty()) assembly::solve(*m_scene);
}

void PropertiesPanel::onJointEdited() {
    if (m_loading) return;
    Joint* j = m_scene->jointForChild(m_currentId);
    if (!j) return;
    j->type = m_jType->currentIndex() == 0 ? JointType::Revolute : JointType::Prismatic;
    const double f = (j->type == JointType::Revolute) ? 1.0 : unitFactor();
    // A field still showing the stored value rounded keeps the exact stored
    // value, so editing one field doesn't round away a picked axis or origin
    auto read = [](float stored, const QDoubleSpinBox* s, double factor) {
        const double shown = std::round(stored * factor * std::pow(10.0, s->decimals())) / std::pow(10.0, s->decimals());
        return std::abs(shown - s->value()) < 1e-9 ? stored : float(s->value() / factor);
    };
    j->axis         = { read(j->axis.x(), m_jAxisX, 1.0), read(j->axis.y(), m_jAxisY, 1.0), read(j->axis.z(), m_jAxisZ, 1.0) };
    j->limitMin     = float(m_jLimitMin->value() / f);
    j->limitMax     = float(m_jLimitMax->value() / f);
    j->initialAngle = float(m_jInitAngle->value() / f);
    const double lf = unitFactor();  // pivot is always a length quantity, regardless of joint type
    j->pivot        = { read(j->pivot.x(), m_jPivotX, lf), read(j->pivot.y(), m_jPivotY, lf), read(j->pivot.z(), m_jPivotZ, lf) };
    j->continuous   = m_jContinuous->isChecked();
    bool isRevolute = (j->type == JointType::Revolute);
    m_jLimitMin->setEnabled(!(isRevolute && j->continuous));
    m_jLimitMax->setEnabled(!(isRevolute && j->continuous));
    m_jPivotRow->setVisible(isRevolute);
    updateJointUnits(isRevolute);
    emit m_scene->objectChanged(m_currentId);
}

void PropertiesPanel::fillCouplingCombo(const Joint& joint) {
    const QSignalBlocker block(m_jCouple);
    m_jCouple->clear();
    m_jCouple->addItem("Nothing (moves on its own)", -1);
    std::vector<int> ids;
    for (auto& [jid, j] : m_scene->allJoints()) if (jid != joint.id) ids.push_back(jid);
    std::sort(ids.begin(), ids.end());
    for (int jid : ids) {
        const Joint* j = m_scene->getJoint(jid);
        const Motor* m = m_scene->motorForJoint(jid);
        const SceneObject* child = m_scene->getObject(j->childBodyId);
        m_jCouple->addItem(QString("%1  (%2)").arg(m ? m->name : (j->name.isEmpty() ? QString("Joint %1").arg(jid) : j->name),
                                                   child ? child->name : QString("?")), jid);
    }
    m_jCouple->setCurrentIndex(std::max(0, m_jCouple->findData(joint.coupledTo)));
}

void PropertiesPanel::onCouplingEdited() {
    if (m_loading) return;
    Joint* j = m_scene->jointForChild(m_currentId);
    if (!j) return;
    const int master = m_jCouple->currentData().toInt();
    const bool wasFollower = j->coupledTo != -1;
    if (master != -1 && !wasFollower && m_jRatio->value() == 0.0) {
        const QSignalBlocker block(m_jRatio);
        m_jRatio->setValue(1.0);
    }
    j->coupledTo = master;
    j->ratio     = float(m_jRatio->value());
    const double f = (j->type == JointType::Revolute) ? 1.0 : unitFactor();
    j->offset    = float(m_jOffset->value() / f);
    m_jCoupleRow->setVisible(master != -1);
    m_jPosition->setEnabled(master == -1 && !m_simActive);
    m_scene->notifyJointsMoved();
    m_scene->notifyAutomationChanged();
    emit m_scene->objectChanged(m_currentId);
}

void PropertiesPanel::onMotorEdited() {
    if (m_loading) return;
    Joint* j = m_scene->jointForChild(m_currentId);
    if (!j) return;
    Motor* motor = m_scene->motorForJoint(j->id);
    if (!motor) return;
    const double f = (j->type == JointType::Revolute) ? 1.0 : unitFactor();
    motor->mode         = m_mMode->currentIndex() == 1 ? ActuatorMode::Speed : ActuatorMode::Position;
    motor->velocity     = float(m_mVel->value() / f);
    motor->acceleration = float(m_mAccel->value() / f);
    motor->enabled      = m_mEnabled->isChecked();
    m_scene->notifyAutomationChanged();
}

void PropertiesPanel::onRigidBodyEdited() {
    if (m_loading) return;
    RigidBody* rb = m_scene->getRigidBody(m_currentId);
    if (!rb) return;
    rb->mass           = float(m_rbMass->value());
    rb->gravityEnabled = m_rbGravity->isChecked();
    emit m_scene->rigidBodyChanged(m_currentId);
}

void PropertiesPanel::onCollisionTypeChanged(int idx) {
    updateCollisionRows(idx);
    onCollisionEdited();
}

void PropertiesPanel::onCollisionEdited() {
    if (m_loading) return;
    RigidBody* rb = m_scene->getRigidBody(m_currentId);
    if (!rb || !rb->hasCollision) return;
    const double f = unitFactor();
    auto& c  = rb->collision;
    c.type   = CollisionType(m_colType->currentIndex());
    c.halfW  = float(m_cHW->value() / f);
    c.halfH  = float(m_cHH->value() / f);
    c.halfD  = float(m_cHD->value() / f);
    c.radius = float(m_cRadius->value() / f);
    c.height = float(m_cHeight->value() / f);
    c.offset = { float(m_cOffX->value() / f), float(m_cOffY->value() / f), float(m_cOffZ->value() / f) };
}

// ── Joint / Motor CRUD ────────────────────────────────────────────────────────

void PropertiesPanel::addJoint() {
    SceneObject* obj = m_scene->getObject(m_currentId);
    if (!obj || obj->parentId == -1 || m_scene->jointForChild(m_currentId)) return;
    Joint j;
    j.name         = obj->name + "_joint";
    j.parentBodyId = obj->parentId;
    j.childBodyId  = m_currentId;
    m_scene->addJoint(j);
}

void PropertiesPanel::removeJoint() {
    Joint* j = m_scene->jointForChild(m_currentId);
    if (!j) return;
    m_scene->removeJoint(j->id);
    showJointGroup(false); showMotorGroup(false); showConnectionsGroup(false);
    loadObject(m_currentId);
}

void PropertiesPanel::addMotor() {
    Joint* j = m_scene->jointForChild(m_currentId);
    if (!j) return;
    m_scene->addActuatorForJoint(j->id);
}

void PropertiesPanel::removeMotor() {
    Joint* j = m_scene->jointForChild(m_currentId);
    if (!j) return;
    if (Motor* m = m_scene->motorForJoint(j->id)) m_scene->removeMotor(m->id);
    showMotorGroup(false);
    m_addMotorBtn->setVisible(true);
}

// ── Rigid body / Collision CRUD ───────────────────────────────────────────────

void PropertiesPanel::addRigidBody() {
    m_scene->addRigidBody(m_currentId);
}

void PropertiesPanel::removeRigidBody() {
    m_scene->removeRigidBody(m_currentId);
}

void PropertiesPanel::addCollision() {
    RigidBody* rb = m_scene->getRigidBody(m_currentId);
    if (!rb) return;
    // Initialize collision from current primitive type and dimensions
    const SceneObject* obj = m_scene->getObject(m_currentId);
    if (obj) {
        auto& c = rb->collision;
        switch (obj->type) {
            case PrimitiveType::Cube:
                c.type  = CollisionType::Box;
                c.halfW = obj->transform.scale.x() / 2.0f;
                c.halfH = obj->transform.scale.y() / 2.0f;
                c.halfD = obj->transform.scale.z() / 2.0f;
                break;
            case PrimitiveType::Sphere:
                c.type   = CollisionType::Sphere;
                c.radius = obj->transform.scale.x() / 2.0f;
                break;
            case PrimitiveType::Cylinder:
                c.type   = CollisionType::Cylinder;
                c.radius = obj->transform.scale.x() / 2.0f;
                c.height = obj->transform.scale.y();
                break;
            case PrimitiveType::Capsule:
                c.type   = CollisionType::Capsule;
                c.radius = obj->transform.scale.x() / 2.0f;
                c.height = obj->transform.scale.y();
                break;
            case PrimitiveType::Basket:
                c.type  = CollisionType::Box;
                c.halfW = obj->transform.scale.x() / 2.0f;
                c.halfH = obj->transform.scale.y() / 2.0f;
                c.halfD = obj->transform.scale.z() / 2.0f;
                break;
            case PrimitiveType::ImportedMesh: {
                // Box around the mesh AABB (world half extents, offset to AABB centre)
                QVector3D he = (obj->aabbMax() - obj->aabbMin()) * 0.5f * obj->transform.scale;
                c.type   = CollisionType::Box;
                c.halfW  = he.x();
                c.halfH  = he.y();
                c.halfD  = he.z();
                c.offset = (obj->aabbMax() + obj->aabbMin()) * 0.5f * obj->transform.scale;
                break;
            }
        }
    }
    rb->hasCollision = true;
    emit m_scene->rigidBodyChanged(m_currentId);
}

void PropertiesPanel::removeCollision() {
    RigidBody* rb = m_scene->getRigidBody(m_currentId);
    if (!rb) return;
    rb->hasCollision = false;
    emit m_scene->rigidBodyChanged(m_currentId);
}

void PropertiesPanel::addConveyorBelt() {
    if (m_currentId == -1) return;
    m_scene->addConveyorBelt(m_currentId);
}

void PropertiesPanel::removeConveyorBelt() {
    if (m_currentId == -1) return;
    m_scene->removeConveyorBelt(m_currentId);
}

void PropertiesPanel::onConveyorEdited() {
    if (m_loading) return;
    ConveyorBelt* belt = m_scene->getConveyorBelt(m_currentId);
    if (!belt) return;
    belt->velocity = float(m_cvVelocity->value() / unitFactor());
    belt->enabled  = m_cvEnabled->isChecked();
    static const QVector3D kDirs[] = {{1,0,0},{-1,0,0},{0,0,1},{0,0,-1}};
    belt->direction = kDirs[m_cvDir->currentIndex()];
    emit m_scene->conveyorBeltChanged(m_currentId);
}

void PropertiesPanel::snapToFloor() {
    if (m_currentId == -1) return;
    SceneObject* obj = m_scene->getObject(m_currentId);
    if (!obj) return;
    // Primitives span ±0.5 in local Y before scale; imported meshes use their real AABB
    float newY = -obj->aabbMin().y() * obj->transform.scale.y();
    m_loading = true;
    obj->transform.position.setY(newY);
    m_py->setValue(newY * unitFactor());
    m_loading = false;
    emit m_scene->objectChanged(m_currentId);
}

// ── Units ─────────────────────────────────────────────────────────────────────

void PropertiesPanel::onUnitChanged(int idx) {
    m_lengthUnit = (idx == 1) ? LengthUnit::Millimeters : LengthUnit::Meters;
    const bool mm = m_lengthUnit == LengthUnit::Millimeters;
    const double f = unitFactor();

    m_loading = true;
    for (auto& lf : m_lengthFields) {
        lf.spin->setRange(lf.baseMin * f, lf.baseMax * f);
        lf.spin->setSingleStep(lf.baseStep * f);
        lf.spin->setSuffix(scaledLengthSuffix(lf.baseSuffix, mm));
    }
    m_loading = false;

    // Refresh currently displayed values (transform, joint/motor, physics, conveyor)
    // into the newly selected unit.
    if (m_currentId != -1) loadObject(m_currentId);

    // Re-render the last picked point, if any, in the new unit.
    if (m_pointGroup->isVisible()) showPointCoords(m_lastPointName, m_lastPointPos);
}

// ── Simulation lock ───────────────────────────────────────────────────────────

void PropertiesPanel::setSimulationActive(bool active) {
    m_simActive = active;
    m_jointGroup->setEnabled(!active);
    m_motorGroup->setEnabled(!active);
    m_connectionsGroup->setEnabled(!active);
}
