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
    s->setFixedWidth(86);
    if(!suffix.isEmpty()) s->setSuffix(suffix);
    return s;
}

static QHBoxLayout* xyzRow(const QString& label,
                            QDoubleSpinBox* x, QDoubleSpinBox* y, QDoubleSpinBox* z) {
    auto* row = new QHBoxLayout;
    row->setSpacing(3);
    row->setContentsMargins(0,0,0,0);
    auto lbl = [&](const QString& t, QColor c) {
        auto* l = new QLabel(t);
        l->setAlignment(Qt::AlignCenter);
        l->setFixedWidth(14);
        l->setStyleSheet(QString("color:%1; font-weight:bold; font-size:10px; background:transparent;").arg(c.name()));
        return l;
    };
    if(!label.isEmpty()){
        auto* ll = new QLabel(label);
        ll->setFixedWidth(52);
        ll->setStyleSheet("color:#555; font-size:11px; background:transparent;");
        row->addWidget(ll);
    }
    row->addWidget(lbl("X", {200,60,60}));   row->addWidget(x);
    row->addWidget(lbl("Y", {40,160,60}));   row->addWidget(y);
    row->addWidget(lbl("Z", {40,80,220}));   row->addWidget(z);
    return row;
}

// ── Constructor ───────────────────────────────────────────────────────────────

PropertiesPanel::PropertiesPanel(SceneGraph* scene, QWidget* parent)
    : QWidget(parent), m_scene(scene)
{
    setStyleSheet(kPanelStyle);
    setMinimumWidth(250);

    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0,0,0,0);
    outer->setSpacing(0);

    // Header bar
    auto* hdr = new QLabel("  Properties", this);
    hdr->setFixedHeight(28);
    hdr->setStyleSheet("background:#e8ecf2; color:#333; font-size:11px; font-weight:bold;"
                       "border-bottom:1px solid #d0d0d0;");
    outer->addWidget(hdr);

    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    outer->addWidget(scroll);

    auto* inner = new QWidget;
    inner->setStyleSheet("background:#ffffff;");
    auto* vbox = new QVBoxLayout(inner);
    vbox->setContentsMargins(10, 6, 10, 10);
    vbox->setSpacing(6);

    buildNameSection();
    buildTransformSection();
    buildConnectionsSection();
    buildJointSection();
    buildMotorSection();

    vbox->addWidget(m_nameLabel->parentWidget()); // name card
    vbox->addWidget(m_transformGroup);
    vbox->addWidget(m_connectionsGroup);
    vbox->addWidget(m_jointGroup);
    vbox->addWidget(m_motorGroup);
    vbox->addStretch();
    scroll->setWidget(inner);

    m_nameLabel->parentWidget()->hide();
    m_transformGroup->setEnabled(false);
    showConnectionsGroup(false);
    showJointGroup(false);
    showMotorGroup(false);

    connect(scene, &SceneGraph::selectionChanged, this, &PropertiesPanel::onSelectionChanged);
    connect(scene, &SceneGraph::objectChanged,    this, [this](int id){
        if(id == m_currentId) loadObject(id);
    });
    connect(scene, &SceneGraph::jointAdded,  this, [this](int){ if(m_currentId!=-1) loadObject(m_currentId); });
    connect(scene, &SceneGraph::motorAdded,  this, [this](int){ if(m_currentId!=-1) loadObject(m_currentId); });
}

// ── Section builders ──────────────────────────────────────────────────────────

void PropertiesPanel::buildNameSection() {
    auto* card = new QWidget;
    card->setStyleSheet("QWidget { background:#f0f4fc; border:1px solid #d4dae8; border-radius:5px; }");
    auto* vb = new QVBoxLayout(card);
    vb->setContentsMargins(10,6,10,6);
    vb->setSpacing(1);
    m_nameLabel = new QLabel("—");
    m_nameLabel->setStyleSheet("font-size:13px; font-weight:bold; color:#1a3a7a; background:transparent;");
    m_typeLabel = new QLabel;
    m_typeLabel->setStyleSheet("font-size:10px; color:#667799; background:transparent;");
    vb->addWidget(m_nameLabel);
    vb->addWidget(m_typeLabel);
}

void PropertiesPanel::buildTransformSection() {
    m_transformGroup = new QGroupBox("Transform");
    auto* vb = new QVBoxLayout(m_transformGroup);
    vb->setSpacing(5);

    m_px = makeSpin(-1e4,1e4,0.1f); m_py = makeSpin(-1e4,1e4,0.1f); m_pz = makeSpin(-1e4,1e4,0.1f);
    m_rx = makeSpin(-360,360,1.0f,"°"); m_ry = makeSpin(-360,360,1.0f,"°"); m_rz = makeSpin(-360,360,1.0f,"°");
    m_sx = makeSpin(0.001f,100,0.05f); m_sy = makeSpin(0.001f,100,0.05f); m_sz = makeSpin(0.001f,100,0.05f);
    m_sx->setValue(1); m_sy->setValue(1); m_sz->setValue(1);

    vb->addLayout(xyzRow("Position", m_px, m_py, m_pz));
    vb->addLayout(xyzRow("Rotation", m_rx, m_ry, m_rz));
    vb->addLayout(xyzRow("Scale",    m_sx, m_sy, m_sz));

    auto wire = [this](QDoubleSpinBox* s){
        connect(s, &QDoubleSpinBox::valueChanged, this, &PropertiesPanel::onTransformEdited);
    };
    for(auto* s : {m_px,m_py,m_pz,m_rx,m_ry,m_rz,m_sx,m_sy,m_sz}) wire(s);
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
        row->addWidget(new QLabel("Type"));
        row->addWidget(m_jType, 1);
        vb->addLayout(row);
    }

    m_jAxisX = makeSpin(-1,1,0.1f); m_jAxisY = makeSpin(-1,1,0.1f); m_jAxisZ = makeSpin(-1,1,0.1f);
    m_jAxisY->setValue(1);
    vb->addLayout(xyzRow("Axis", m_jAxisX, m_jAxisY, m_jAxisZ));

    m_jLimitMin  = makeSpin(-1e4,1e4,1.0f,"°"); m_jLimitMin->setValue(-180);
    m_jLimitMax  = makeSpin(-1e4,1e4,1.0f,"°"); m_jLimitMax->setValue( 180);
    m_jInitAngle = makeSpin(-360, 360,1.0f,"°");

    auto addRow = [&](const QString& lbl, QWidget* w){
        auto* row = new QHBoxLayout;
        auto* l = new QLabel(lbl); l->setFixedWidth(70);
        l->setStyleSheet("color:#555; font-size:11px; background:transparent;");
        row->addWidget(l); row->addWidget(w); row->addStretch();
        vb->addLayout(row);
    };
    addRow("Limit min",   m_jLimitMin);
    addRow("Limit max",   m_jLimitMax);
    addRow("Init angle",  m_jInitAngle);

    // Add motor button (shown when joint has no motor)
    m_addMotorBtn = new QPushButton("Add Motor to this Joint");
    m_addMotorBtn->setToolTip("Attach a motor so this joint can be driven during simulation");
    vb->addWidget(m_addMotorBtn);

    // Remove joint
    auto* btnRow = new QHBoxLayout;
    auto* rmBtn = new QPushButton("Remove Joint");
    rmBtn->setProperty("danger", true);
    btnRow->addStretch(); btnRow->addWidget(rmBtn);
    vb->addLayout(btnRow);

    connect(m_jType,     &QComboBox::currentIndexChanged,  this, &PropertiesPanel::onJointEdited);
    connect(m_jAxisX,    &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jAxisY,    &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jAxisZ,    &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jLimitMin, &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jLimitMax, &QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_jInitAngle,&QDoubleSpinBox::valueChanged,    this, &PropertiesPanel::onJointEdited);
    connect(m_addMotorBtn, &QPushButton::clicked,          this, &PropertiesPanel::addMotor);
    connect(rmBtn,       &QPushButton::clicked,            this, &PropertiesPanel::removeJoint);
}

void PropertiesPanel::buildMotorSection() {
    m_motorGroup = new QGroupBox("Motor");
    auto* vb = new QVBoxLayout(m_motorGroup);
    vb->setSpacing(5);

    m_mVel    = makeSpin(-9999,9999,5.0f,"°/s");
    m_mTarget = makeSpin(-9999,9999,1.0f,"°");
    m_mEnabled = new QCheckBox("Motor enabled");
    m_mEnabled->setChecked(true);
    m_mVel->setValue(45);

    auto addRow = [&](const QString& lbl, QWidget* w){
        auto* row = new QHBoxLayout;
        auto* l = new QLabel(lbl); l->setFixedWidth(70);
        l->setStyleSheet("color:#555; font-size:11px; background:transparent;");
        row->addWidget(l); row->addWidget(w); row->addStretch();
        vb->addLayout(row);
    };
    addRow("Velocity",  m_mVel);
    addRow("Target",    m_mTarget);
    vb->addWidget(m_mEnabled);

    auto* btnRow = new QHBoxLayout;
    auto* rmBtn = new QPushButton("Remove Motor");
    rmBtn->setProperty("danger", true);
    btnRow->addStretch(); btnRow->addWidget(rmBtn);
    vb->addLayout(btnRow);

    connect(m_mVel,    &QDoubleSpinBox::valueChanged, this, &PropertiesPanel::onMotorEdited);
    connect(m_mTarget, &QDoubleSpinBox::valueChanged, this, &PropertiesPanel::onMotorEdited);
    connect(m_mEnabled,&QCheckBox::toggled,           this, &PropertiesPanel::onMotorEdited);
    connect(rmBtn,     &QPushButton::clicked,         this, &PropertiesPanel::removeMotor);
}

// ── Visibility helpers ────────────────────────────────────────────────────────

void PropertiesPanel::showConnectionsGroup(bool v) { m_connectionsGroup->setVisible(v); }
void PropertiesPanel::showJointGroup(bool v)        { m_jointGroup->setVisible(v); }
void PropertiesPanel::showMotorGroup(bool v)        { m_motorGroup->setVisible(v); }

// ── Load / selection ──────────────────────────────────────────────────────────

void PropertiesPanel::onSelectionChanged(int id) {
    m_currentId = id;
    if(id == -1){
        m_nameLabel->parentWidget()->hide();
        m_transformGroup->setEnabled(false);
        showConnectionsGroup(false);
        showJointGroup(false);
        showMotorGroup(false);
        return;
    }
    m_nameLabel->parentWidget()->show();
    m_transformGroup->setEnabled(true);
    loadObject(id);
}

void PropertiesPanel::loadObject(int id) {
    const SceneObject* obj = m_scene->getObject(id);
    if(!obj) return;
    m_loading = true;

    // Name + type
    m_nameLabel->setText(obj->name);
    static const char* typeNames[] = {"Cube","Cylinder","Sphere","Capsule"};
    QString parentInfo = obj->parentId == -1 ? "Root object" :
        "Child of: " + (m_scene->getObject(obj->parentId) ? m_scene->getObject(obj->parentId)->name : "?");
    m_typeLabel->setText(QString("%1  —  %2").arg(typeNames[int(obj->type)], parentInfo));

    // Transform
    m_px->setValue(obj->transform.position.x());
    m_py->setValue(obj->transform.position.y());
    m_pz->setValue(obj->transform.position.z());
    m_rx->setValue(obj->transform.rotation.x());
    m_ry->setValue(obj->transform.rotation.y());
    m_rz->setValue(obj->transform.rotation.z());
    m_sx->setValue(obj->transform.scale.x());
    m_sy->setValue(obj->transform.scale.y());
    m_sz->setValue(obj->transform.scale.z());

    // Joint
    Joint* joint = m_scene->jointForChild(id);
    bool hasParent = obj->parentId != -1;

    showConnectionsGroup(hasParent && !joint);
    if(joint){
        m_jType->setCurrentIndex(joint->type == JointType::Revolute ? 0 : 1);
        m_jAxisX->setValue(joint->axis.x());
        m_jAxisY->setValue(joint->axis.y());
        m_jAxisZ->setValue(joint->axis.z());
        m_jLimitMin->setValue(joint->limitMin);
        m_jLimitMax->setValue(joint->limitMax);
        m_jInitAngle->setValue(joint->initialAngle);
        showJointGroup(true);

        Motor* motor = nullptr;
        for(auto& [mid,m] : m_scene->allMotors()){
            if(m.jointId == joint->id){ motor = &const_cast<Motor&>(m); break; }
        }
        m_addMotorBtn->setVisible(!motor);
        if(motor){
            m_mVel->setValue(motor->velocity);
            m_mTarget->setValue(motor->targetPosition);
            m_mEnabled->setChecked(motor->enabled);
            showMotorGroup(true);
        } else {
            showMotorGroup(false);
        }
    } else {
        showJointGroup(false);
        showMotorGroup(false);
    }

    m_loading = false;
}

// ── Write-back ────────────────────────────────────────────────────────────────

void PropertiesPanel::onTransformEdited() {
    if(m_loading) return;
    SceneObject* obj = m_scene->getObject(m_currentId);
    if(!obj) return;
    obj->transform.position = { float(m_px->value()), float(m_py->value()), float(m_pz->value()) };
    obj->transform.rotation = { float(m_rx->value()), float(m_ry->value()), float(m_rz->value()) };
    obj->transform.scale    = { float(m_sx->value()), float(m_sy->value()), float(m_sz->value()) };
    emit m_scene->objectChanged(m_currentId);
}

void PropertiesPanel::onJointEdited() {
    if(m_loading) return;
    Joint* j = m_scene->jointForChild(m_currentId);
    if(!j) return;
    j->type         = m_jType->currentIndex() == 0 ? JointType::Revolute : JointType::Prismatic;
    j->axis         = { float(m_jAxisX->value()), float(m_jAxisY->value()), float(m_jAxisZ->value()) };
    j->limitMin     = float(m_jLimitMin->value());
    j->limitMax     = float(m_jLimitMax->value());
    j->initialAngle = float(m_jInitAngle->value());
}

void PropertiesPanel::onMotorEdited() {
    if(m_loading) return;
    Joint* j = m_scene->jointForChild(m_currentId);
    if(!j) return;
    for(auto& [mid,motor] : m_scene->allMotors()){
        if(motor.jointId == j->id){
            const_cast<Motor&>(motor).velocity       = float(m_mVel->value());
            const_cast<Motor&>(motor).targetPosition = float(m_mTarget->value());
            const_cast<Motor&>(motor).enabled        = m_mEnabled->isChecked();
            return;
        }
    }
}

void PropertiesPanel::addJoint() {
    SceneObject* obj = m_scene->getObject(m_currentId);
    if(!obj || obj->parentId == -1 || m_scene->jointForChild(m_currentId)) return;
    Joint j;
    j.name         = obj->name + "_joint";
    j.parentBodyId = obj->parentId;
    j.childBodyId  = m_currentId;
    m_scene->addJoint(j);
}

void PropertiesPanel::removeJoint() {
    Joint* j = m_scene->jointForChild(m_currentId);
    if(!j) return;
    m_scene->removeJoint(j->id);
    showJointGroup(false); showMotorGroup(false); showConnectionsGroup(false);
    loadObject(m_currentId); // re-evaluate connections section
}

void PropertiesPanel::addMotor() {
    Joint* j = m_scene->jointForChild(m_currentId);
    if(!j) return;
    Motor m;
    m.name    = "Motor_" + j->name;
    m.jointId = j->id;
    m_scene->addMotor(m);
}

void PropertiesPanel::removeMotor() {
    Joint* j = m_scene->jointForChild(m_currentId);
    if(!j) return;
    for(auto& [mid,motor] : m_scene->allMotors()){
        if(motor.jointId == j->id){ m_scene->removeMotor(mid); break; }
    }
    showMotorGroup(false);
    m_addMotorBtn->setVisible(true);
}
