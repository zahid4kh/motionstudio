#include "PropertiesPanel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QScrollArea>
#include <QInputDialog>

// ── Style ─────────────────────────────────────────────────────────────────────

static const char* kPanelStyle =
    "QWidget        { background:#ffffff; }"
    "QGroupBox      { color:#2255aa; border:1px solid #d4dae8; border-radius:5px; "
    "                 margin-top:10px; font-size:11px; font-weight:bold; background:#ffffff; }"
    "QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 4px; }"
    "QLabel         { color:#333; font-size:11px; background:transparent; }"
    "QDoubleSpinBox { background:#f5f7fc; color:#111; border:1px solid #c0c8da; "
    "                 border-radius:4px; padding:3px 5px; min-height:24px; font-size:11px; }"
    "QDoubleSpinBox:focus { border:1px solid #4480cc; background:#eef4ff; }"
    "QDoubleSpinBox:hover { border:1px solid #88aadd; }"
    "QComboBox      { background:#f5f7fc; color:#111; border:1px solid #c0c8da; "
    "                 border-radius:4px; padding:3px 5px; min-height:24px; font-size:11px; }"
    "QComboBox:focus { border:1px solid #4480cc; }"
    "QCheckBox      { color:#333; font-size:11px; }"
    "QPushButton    { background:#f0f4fc; color:#333; border:1px solid #b8c4da; "
    "                 border-radius:4px; padding:5px 12px; font-size:11px; }"
    "QPushButton:hover   { background:#dce8f8; border-color:#7aaae0; color:#1040a0; }"
    "QPushButton:pressed { background:#c8d8f0; }"
    "QPushButton[danger='true'] { background:#fff0f0; color:#aa2020; border-color:#ddb0b0; }"
    "QPushButton[danger='true']:hover { background:#ffe0e0; border-color:#cc6060; }"
    "QScrollArea    { border:none; background:#ffffff; }";

// ── Helpers ──────────────────────────────────────────────────────────────────

QDoubleSpinBox* PropertiesPanel::makeSpin(float mn, float mx, float step, const QString& suffix) {
    auto* s = new QDoubleSpinBox;
    s->setRange(mn, mx);
    s->setSingleStep(step);
    s->setDecimals(3);
    s->setAccelerated(true);
    s->setFixedWidth(92);
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
    auto chip = [&](const QString& t, QColor c) {
        auto* l = new QLabel(t);
        l->setAlignment(Qt::AlignCenter);
        l->setFixedWidth(14);
        l->setStyleSheet(QString("color:%1; font-weight:bold; font-size:10px; background:transparent;").arg(c.name()));
        return l;
    };
    if (!label.isEmpty()) {
        auto* ll = new QLabel(label);
        ll->setFixedWidth(66);
        ll->setStyleSheet("color:#555; font-size:11px; background:transparent;");
        row->addWidget(ll);
    }
    row->addWidget(chip("X", {200, 60, 60}));  row->addWidget(x);
    row->addWidget(chip("Y", {40, 160, 60}));  row->addWidget(y);
    row->addWidget(chip("Z", {40, 80, 220}));  row->addWidget(z);
    return row;
}

// Single label + spinbox row, used throughout sections.
static void labelRow(QVBoxLayout* vb, const QString& lbl, QWidget* w, int labelWidth = 70) {
    auto* row = new QHBoxLayout;
    auto* l = new QLabel(lbl);
    l->setFixedWidth(labelWidth);
    l->setStyleSheet("color:#555; font-size:11px; background:transparent;");
    row->addWidget(l);
    row->addWidget(w);
    row->addStretch();
    vb->addLayout(row);
}

// ── Constructor ───────────────────────────────────────────────────────────────

PropertiesPanel::PropertiesPanel(SceneGraph* scene, QWidget* parent)
    : QWidget(parent), m_scene(scene)
{
    setStyleSheet(kPanelStyle);
    setMinimumWidth(260);

    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->setSpacing(0);

    auto* hdrRow = new QWidget(this);
    hdrRow->setFixedHeight(28);
    hdrRow->setStyleSheet("background:#e8ecf2; border-bottom:1px solid #d0d0d0;");
    auto* hdrLayout = new QHBoxLayout(hdrRow);
    hdrLayout->setContentsMargins(6, 0, 6, 0);

    auto* hdr = new QLabel("Properties", hdrRow);
    hdr->setStyleSheet("color:#333; font-size:11px; font-weight:bold; background:transparent; border:none;");
    hdrLayout->addWidget(hdr);
    hdrLayout->addStretch();

    m_unitCombo = new QComboBox(hdrRow);
    m_unitCombo->addItems({"m", "mm"});
    m_unitCombo->setToolTip("Length display unit (values are still stored in metres)");
    m_unitCombo->setFixedWidth(56);
    hdrLayout->addWidget(m_unitCombo);
    connect(m_unitCombo, &QComboBox::currentIndexChanged, this, &PropertiesPanel::onUnitChanged);

    outer->addWidget(hdrRow);

    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    outer->addWidget(scroll);

    auto* inner = new QWidget;
    inner->setStyleSheet("background:#ffffff;");
    auto* vbox = new QVBoxLayout(inner);
    vbox->setContentsMargins(10, 6, 10, 10);
    vbox->setSpacing(6);

    buildNameSection();
    buildPointSection();
    buildTransformSection();
    buildConnectionsSection();
    buildJointSection();
    buildMotorSection();
    buildPhysicsSection();
    buildConveyorSection();

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
    m_transformGroup->setEnabled(false);
    showConnectionsGroup(false);
    showJointGroup(false);
    showMotorGroup(false);
    m_physicsGroup->hide();
    m_conveyorGroup->hide();

    connect(scene, &SceneGraph::selectionChanged, this, &PropertiesPanel::onSelectionChanged);
    connect(scene, &SceneGraph::objectChanged,    this, [this](int id){
        if (id == m_currentId) loadObject(id);
    });
    connect(scene, &SceneGraph::jointAdded,  this, [this](int){ if (m_currentId != -1) loadObject(m_currentId); });
    connect(scene, &SceneGraph::motorAdded,  this, [this](int){ if (m_currentId != -1) loadObject(m_currentId); });
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
    card->setStyleSheet("QWidget { background:#f0f4fc; border:1px solid #d4dae8; border-radius:5px; }");
    auto* vb = new QVBoxLayout(card);
    vb->setContentsMargins(10, 6, 10, 6);
    vb->setSpacing(1);
    m_nameLabel = new QLabel("—");
    m_nameLabel->setStyleSheet("font-size:13px; font-weight:bold; color:#1a3a7a; background:transparent;");
    m_typeLabel = new QLabel;
    m_typeLabel->setStyleSheet("font-size:10px; color:#667799; background:transparent;");
    vb->addWidget(m_nameLabel);
    vb->addWidget(m_typeLabel);
}

void PropertiesPanel::buildPointSection() {
    m_pointGroup = new QGroupBox("Selected Point");
    auto* vb = new QVBoxLayout(m_pointGroup);
    vb->setSpacing(3);

    m_pointNameLabel = new QLabel("—");
    m_pointNameLabel->setStyleSheet("font-size:12px; font-weight:bold; color:#1a3a7a; background:transparent;");
    vb->addWidget(m_pointNameLabel);

    auto mkCoord = [&](const QString& axis) -> QLabel* {
        auto* l = new QLabel(QString("%1: —").arg(axis));
        l->setStyleSheet("font-family:monospace; font-size:11px; color:#333; background:transparent;");
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

    m_snapToFloorBtn = new QPushButton("Snap Bottom to Floor");
    m_snapToFloorBtn->setToolTip("Move Y so the object's bottom face rests at Y = 0");
    vb->addWidget(m_snapToFloorBtn);

    auto wire = [this](QDoubleSpinBox* s){
        connect(s, &QDoubleSpinBox::valueChanged, this, &PropertiesPanel::onTransformEdited);
    };
    for (auto* s : {m_px, m_py, m_pz, m_rx, m_ry, m_rz, m_sx, m_sy, m_sz}) wire(s);
    connect(m_snapToFloorBtn, &QPushButton::clicked, this, &PropertiesPanel::snapToFloor);
}

void PropertiesPanel::buildConnectionsSection() {
    m_connectionsGroup = new QGroupBox("Connections");
    auto* vb = new QVBoxLayout(m_connectionsGroup);
    vb->setSpacing(4);

    auto* info = new QLabel("This object has a parent but no joint.\nAdd a joint to enable motion.");
    info->setWordWrap(true);
    info->setStyleSheet("color:#556; font-size:10px; background:transparent;");
    vb->addWidget(info);

    m_addJointBtn = new QPushButton("Add Joint to Parent");
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
        l->setStyleSheet("color:#555; font-size:11px; background:transparent;");
        row->addWidget(l); row->addWidget(m_jType, 1);
        vb->addLayout(row);
    }

    m_jAxisX = makeSpin(-1, 1, 0.1f);
    m_jAxisY = makeSpin(-1, 1, 0.1f);
    m_jAxisZ = makeSpin(-1, 1, 0.1f);
    m_jAxisY->setValue(1);
    vb->addLayout(xyzRow("Axis", m_jAxisX, m_jAxisY, m_jAxisZ));

    m_jPickAxisBtn = new QPushButton("Pick Axis in 3D…");
    m_jPickAxisBtn->setToolTip("Click one of the gizmo's arrows on the selected part to set this axis");
    vb->addWidget(m_jPickAxisBtn);

    // Suffix "°" for revolute, " m" for prismatic — set in updateJointUnits()
    m_jLimitMin  = makeSpin(-1e4, 1e4, 1.0f, "°"); m_jLimitMin->setValue(-180);
    m_jLimitMax  = makeSpin(-1e4, 1e4, 1.0f, "°"); m_jLimitMax->setValue(180);
    m_jInitAngle = makeSpin(-1e4, 1e4, 1.0f, "°");

    m_jInitLabel = new QLabel("Init angle");
    m_jInitLabel->setFixedWidth(70);
    m_jInitLabel->setStyleSheet("color:#555; font-size:11px; background:transparent;");

    labelRow(vb, "Limit min",  m_jLimitMin);
    labelRow(vb, "Limit max",  m_jLimitMax);
    {
        auto* row = new QHBoxLayout;
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

    m_addMotorBtn = new QPushButton("Add Motor to this Joint");
    m_addMotorBtn->setToolTip("Attach a motor so this joint can be driven during simulation");
    vb->addWidget(m_addMotorBtn);

    auto* btnRow = new QHBoxLayout;
    auto* rmBtn = new QPushButton("Remove Joint");
    rmBtn->setProperty("danger", true);
    btnRow->addStretch(); btnRow->addWidget(rmBtn);
    vb->addLayout(btnRow);

    connect(m_jType,      &QComboBox::currentIndexChanged,  this, &PropertiesPanel::onJointEdited);
    connect(m_jAxisX,     &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jAxisY,     &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jAxisZ,     &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jPickAxisBtn, &QPushButton::clicked, this, &PropertiesPanel::pickAxisRequested);
    connect(m_jLimitMin,  &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jLimitMax,  &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jInitAngle, &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jPivotX,    &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jPivotY,    &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jPivotZ,    &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_addMotorBtn, &QPushButton::clicked,           this, &PropertiesPanel::addMotor);
    connect(rmBtn,        &QPushButton::clicked,            this, &PropertiesPanel::removeJoint);
}

void PropertiesPanel::buildMotorSection() {
    m_motorGroup = new QGroupBox("Motor");
    auto* vb = new QVBoxLayout(m_motorGroup);
    vb->setSpacing(5);

    // Suffix "°/s" or " m/s" — updated in updateJointUnits()
    m_mVel    = makeSpin(-9999, 9999, 5.0f, "°/s");
    m_mTarget = makeSpin(-9999, 9999, 1.0f, "°");
    m_mEnabled = new QCheckBox("Motor enabled");
    m_mEnabled->setChecked(true);
    m_mVel->setValue(45);

    labelRow(vb, "Velocity", m_mVel);
    labelRow(vb, "Target",   m_mTarget);
    vb->addWidget(m_mEnabled);

    auto* btnRow = new QHBoxLayout;
    auto* rmBtn = new QPushButton("Remove Motor");
    rmBtn->setProperty("danger", true);
    btnRow->addStretch(); btnRow->addWidget(rmBtn);
    vb->addLayout(btnRow);

    connect(m_mVel,    &QDoubleSpinBox::valueChanged, this, &PropertiesPanel::onMotorEdited);
    connect(m_mTarget, &QDoubleSpinBox::valueChanged, this, &PropertiesPanel::onMotorEdited);
    connect(m_mEnabled, &QCheckBox::toggled,          this, &PropertiesPanel::onMotorEdited);
    connect(rmBtn,     &QPushButton::clicked,         this, &PropertiesPanel::removeMotor);
}

void PropertiesPanel::buildPhysicsSection() {
    m_physicsGroup = new QGroupBox("Physics");
    auto* vb = new QVBoxLayout(m_physicsGroup);
    vb->setSpacing(5);

    // ── No rigid body: just an add button ──────────────────────────────────
    m_rbAddBtn = new QPushButton("Add Rigid Body");
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
    m_colAddBtn = new QPushButton("Add Collision Body");
    m_colAddBtn->setToolTip("Add a collision shape matching this primitive's dimensions");
    rbvb->addWidget(m_colAddBtn);

    // ── Collision body sub-section ────────────────────────────────────────
    m_colWidget = new QWidget;
    auto* colvb = new QVBoxLayout(m_colWidget);
    colvb->setContentsMargins(0, 0, 0, 0);
    colvb->setSpacing(4);

    auto* colHdr = new QLabel("— Collision Body —");
    colHdr->setAlignment(Qt::AlignCenter);
    colHdr->setStyleSheet("color:#2255aa; font-size:10px; font-weight:bold; "
                          "padding:2px 0; background:transparent;");
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

    m_colRemBtn = new QPushButton("Remove Collision Body");
    m_colRemBtn->setProperty("danger", true);
    colvb->addWidget(m_colRemBtn);

    rbvb->addWidget(m_colWidget);

    // Remove rigid body (always at bottom of rb section)
    {
        auto* row = new QHBoxLayout;
        m_rbRemBtn = new QPushButton("Remove Rigid Body");
        m_rbRemBtn->setProperty("danger", true);
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

    m_cvAddBtn = new QPushButton("Add Conveyor Belt");
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
        l->setStyleSheet("color:#555; font-size:11px; background:transparent;");
        row->addWidget(l); row->addWidget(m_cvDir, 1);
        pvb->addLayout(row);
    }

    m_cvEnabled = new QCheckBox("Belt enabled");
    m_cvEnabled->setChecked(true);
    pvb->addWidget(m_cvEnabled);

    auto* btnRow = new QHBoxLayout;
    m_cvRemBtn = new QPushButton("Remove Conveyor Belt");
    m_cvRemBtn->setProperty("danger", true);
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
    m_mTarget->setSuffix(tgtSfx);

    // Prismatic limits/velocity are length quantities — rescale range for the active unit.
    // Revolute (degree) ranges are unaffected since f == 1.0 in that case.
    m_jLimitMin->setRange(-1e4 * f, 1e4 * f);
    m_jLimitMax->setRange(-1e4 * f, 1e4 * f);
    m_jInitAngle->setRange(-1e4 * f, 1e4 * f);
    m_mVel->setRange(-9999 * f, 9999 * f);
    m_mTarget->setRange(-9999 * f, 9999 * f);

    m_jLimitMin->setSingleStep(1.0 * f);
    m_jLimitMax->setSingleStep(1.0 * f);
    m_jInitAngle->setSingleStep(1.0 * f);
    m_mVel->setSingleStep(5.0 * f);
    m_mTarget->setSingleStep(1.0 * f);
}

void PropertiesPanel::updatePhysicsUI(int objectId) {
    const SceneObject* obj = m_scene->getObject(objectId);
    bool isBasket = obj && obj->type == PrimitiveType::Basket;

    RigidBody* rb = m_scene->getRigidBody(objectId);
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
    }
}

// ── Load / selection ──────────────────────────────────────────────────────────

void PropertiesPanel::onSelectionChanged(int id) {
    m_currentId = id;
    if (id == -1) {
        m_nameLabel->parentWidget()->hide();
        m_transformGroup->setEnabled(false);
        showConnectionsGroup(false);
        showJointGroup(false);
        showMotorGroup(false);
        m_physicsGroup->hide();
        m_conveyorGroup->hide();
        return;
    }
    m_nameLabel->parentWidget()->show();
    m_transformGroup->setEnabled(true);
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
    m_typeLabel->setText(QString("%1  —  %2").arg(typeNames[int(obj->type)], parentInfo));

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
        showJointGroup(true);

        Motor* motor = nullptr;
        for (auto& [mid, m] : m_scene->allMotors()) {
            if (m.jointId == joint->id) { motor = &const_cast<Motor&>(m); break; }
        }
        m_addMotorBtn->setVisible(!motor);
        if (motor) {
            m_mVel->setValue(motor->velocity * jf);
            m_mTarget->setValue(motor->targetPosition * jf);
            m_mEnabled->setChecked(motor->enabled);
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
}

void PropertiesPanel::onJointEdited() {
    if (m_loading) return;
    Joint* j = m_scene->jointForChild(m_currentId);
    if (!j) return;
    j->type = m_jType->currentIndex() == 0 ? JointType::Revolute : JointType::Prismatic;
    const double f = (j->type == JointType::Revolute) ? 1.0 : unitFactor();
    j->axis         = { float(m_jAxisX->value()), float(m_jAxisY->value()), float(m_jAxisZ->value()) };
    j->limitMin     = float(m_jLimitMin->value() / f);
    j->limitMax     = float(m_jLimitMax->value() / f);
    j->initialAngle = float(m_jInitAngle->value() / f);
    const double lf = unitFactor();  // pivot is always a length quantity, regardless of joint type
    j->pivot        = { float(m_jPivotX->value() / lf), float(m_jPivotY->value() / lf), float(m_jPivotZ->value() / lf) };
    bool isRevolute = (j->type == JointType::Revolute);
    m_jPivotRow->setVisible(isRevolute);
    updateJointUnits(isRevolute);
    emit m_scene->objectChanged(m_currentId);
}

void PropertiesPanel::onMotorEdited() {
    if (m_loading) return;
    Joint* j = m_scene->jointForChild(m_currentId);
    if (!j) return;
    const double f = (j->type == JointType::Revolute) ? 1.0 : unitFactor();
    for (auto& [mid, motor] : m_scene->allMotors()) {
        if (motor.jointId == j->id) {
            const_cast<Motor&>(motor).velocity       = float(m_mVel->value() / f);
            const_cast<Motor&>(motor).targetPosition = float(m_mTarget->value() / f);
            const_cast<Motor&>(motor).enabled        = m_mEnabled->isChecked();
            emit m_scene->objectChanged(m_currentId);
            return;
        }
    }
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
    Motor m;
    m.name    = "Motor_" + j->name;
    m.jointId = j->id;
    m_scene->addMotor(m);
}

void PropertiesPanel::removeMotor() {
    Joint* j = m_scene->jointForChild(m_currentId);
    if (!j) return;
    for (auto& [mid, motor] : m_scene->allMotors()) {
        if (motor.jointId == j->id) { m_scene->removeMotor(mid); break; }
    }
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
