#include "PropertiesPanel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>

// ── helpers ──────────────────────────────────────────────────────────────────

QDoubleSpinBox* PropertiesPanel::spinXYZ(QWidget* parent, float mn, float mx, float step) {
    auto* s = new QDoubleSpinBox(parent);
    s->setRange(mn, mx);
    s->setSingleStep(step);
    s->setDecimals(3);
    s->setFixedWidth(72);
    return s;
}

static QHBoxLayout* xyzRow(QDoubleSpinBox* x, QDoubleSpinBox* y, QDoubleSpinBox* z) {
    auto* row = new QHBoxLayout;
    row->setSpacing(2);
    row->addWidget(new QLabel("X"));  row->addWidget(x);
    row->addWidget(new QLabel("Y"));  row->addWidget(y);
    row->addWidget(new QLabel("Z"));  row->addWidget(z);
    return row;
}

// ── constructor ──────────────────────────────────────────────────────────────

PropertiesPanel::PropertiesPanel(SceneGraph* scene, QWidget* parent)
    : QWidget(parent), m_scene(scene)
{
    setStyleSheet(
        "QGroupBox { color:#aaaaaa; border:1px solid #333; margin-top:6px; font-size:11px; }"
        "QGroupBox::title { subcontrol-origin:margin; left:8px; }"
        "QLabel { color:#c0c0c0; font-size:11px; }"
        "QDoubleSpinBox { background:#252830; color:#d0d0d0; border:1px solid #444; }"
        "QComboBox { background:#252830; color:#d0d0d0; border:1px solid #444; }"
        "QCheckBox { color:#c0c0c0; }"
        "QPushButton { background:#2a3040; color:#aaaaaa; border:1px solid #444; padding:3px 8px; }"
        "QPushButton:hover { background:#3a4a6a; }"
    );
    setMinimumWidth(240);

    auto* outerLayout = new QVBoxLayout(this);
    outerLayout->setContentsMargins(0, 0, 0, 0);
    outerLayout->setSpacing(0);

    auto* header = new QLabel("  Properties", this);
    header->setFixedHeight(28);
    header->setStyleSheet("background:#1e2128; color:#aaaaaa; font-size:11px; font-weight:bold;");
    outerLayout->addWidget(header);

    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setStyleSheet("QScrollArea { background:#1a1c22; border:none; }");
    outerLayout->addWidget(scroll);

    auto* inner = new QWidget;
    inner->setStyleSheet("background:#1a1c22;");
    auto* vbox = new QVBoxLayout(inner);
    vbox->setContentsMargins(8, 8, 8, 8);
    vbox->setSpacing(4);

    buildTransformSection();
    buildJointSection();
    buildMotorSection();

    vbox->addWidget(m_transformGroup);
    vbox->addWidget(m_jointGroup);
    vbox->addWidget(m_motorGroup);
    vbox->addStretch();
    scroll->setWidget(inner);

    m_transformGroup->setEnabled(false);
    showJointGroup(false);
    showMotorGroup(false);

    connect(scene, &SceneGraph::selectionChanged, this, &PropertiesPanel::onSelectionChanged);
    connect(scene, &SceneGraph::objectChanged,    this, [this](int id){
        if(id == m_currentId) loadObject(id);
    });
}

// ── section builders ──────────────────────────────────────────────────────────

void PropertiesPanel::buildTransformSection() {
    m_transformGroup = new QGroupBox("Transform");
    auto* vb = new QVBoxLayout(m_transformGroup);
    vb->setSpacing(4);
    auto* form = new QFormLayout;
    form->setSpacing(4);

    m_px = spinXYZ(this, -1e4, 1e4, 0.1f);
    m_py = spinXYZ(this, -1e4, 1e4, 0.1f);
    m_pz = spinXYZ(this, -1e4, 1e4, 0.1f);
    m_rx = spinXYZ(this, -360, 360, 1.0f);
    m_ry = spinXYZ(this, -360, 360, 1.0f);
    m_rz = spinXYZ(this, -360, 360, 1.0f);
    m_sx = spinXYZ(this, 0.001f, 100, 0.05f);
    m_sy = spinXYZ(this, 0.001f, 100, 0.05f);
    m_sz = spinXYZ(this, 0.001f, 100, 0.05f);
    m_sx->setValue(1); m_sy->setValue(1); m_sz->setValue(1);

    form->addRow("Position", xyzRow(m_px, m_py, m_pz));
    form->addRow("Rotation", xyzRow(m_rx, m_ry, m_rz));
    form->addRow("Scale",    xyzRow(m_sx, m_sy, m_sz));
    vb->addLayout(form);

    auto wire = [this](QDoubleSpinBox* s){
        connect(s, &QDoubleSpinBox::valueChanged, this, &PropertiesPanel::onTransformEdited);
    };
    wire(m_px); wire(m_py); wire(m_pz);
    wire(m_rx); wire(m_ry); wire(m_rz);
    wire(m_sx); wire(m_sy); wire(m_sz);
}

void PropertiesPanel::buildJointSection() {
    m_jointGroup = new QGroupBox("Joint");
    auto* vb = new QVBoxLayout(m_jointGroup);
    vb->setSpacing(4);
    auto* form = new QFormLayout;
    form->setSpacing(4);

    m_jType = new QComboBox;
    m_jType->addItems({"Revolute", "Prismatic"});
    form->addRow("Type", m_jType);

    m_jAxisX = spinXYZ(this, -1, 1, 0.1f);
    m_jAxisY = spinXYZ(this, -1, 1, 0.1f);
    m_jAxisZ = spinXYZ(this, -1, 1, 0.1f);
    m_jAxisY->setValue(1);
    form->addRow("Axis", xyzRow(m_jAxisX, m_jAxisY, m_jAxisZ));

    m_jLimitMin  = spinXYZ(this, -1e4, 1e4, 1.0f);
    m_jLimitMax  = spinXYZ(this, -1e4, 1e4, 1.0f);
    m_jInitAngle = spinXYZ(this, -360, 360, 1.0f);
    m_jLimitMin->setValue(-180);
    m_jLimitMax->setValue( 180);

    form->addRow("Limit Min",  m_jLimitMin);
    form->addRow("Limit Max",  m_jLimitMax);
    form->addRow("Init Angle", m_jInitAngle);
    vb->addLayout(form);

    auto* btnRow = new QHBoxLayout;
    auto* btnRemove = new QPushButton("Remove Joint");
    btnRow->addStretch();
    btnRow->addWidget(btnRemove);
    vb->addLayout(btnRow);

    connect(m_jType,     &QComboBox::currentIndexChanged, this, &PropertiesPanel::onJointEdited);
    connect(m_jAxisX,    &QDoubleSpinBox::valueChanged,   this, &PropertiesPanel::onJointEdited);
    connect(m_jAxisY,    &QDoubleSpinBox::valueChanged,   this, &PropertiesPanel::onJointEdited);
    connect(m_jAxisZ,    &QDoubleSpinBox::valueChanged,   this, &PropertiesPanel::onJointEdited);
    connect(m_jLimitMin, &QDoubleSpinBox::valueChanged,   this, &PropertiesPanel::onJointEdited);
    connect(m_jLimitMax, &QDoubleSpinBox::valueChanged,   this, &PropertiesPanel::onJointEdited);
    connect(m_jInitAngle,&QDoubleSpinBox::valueChanged,   this, &PropertiesPanel::onJointEdited);
    connect(btnRemove,   &QPushButton::clicked,           this, &PropertiesPanel::removeJoint);
}

void PropertiesPanel::buildMotorSection() {
    m_motorGroup = new QGroupBox("Motor");
    auto* vb = new QVBoxLayout(m_motorGroup);
    vb->setSpacing(4);
    auto* form = new QFormLayout;
    form->setSpacing(4);

    m_mVel    = spinXYZ(this, -1e4, 1e4, 5.0f);
    m_mTarget = spinXYZ(this, -1e4, 1e4, 1.0f);
    m_mEnabled = new QCheckBox("Enabled");
    m_mEnabled->setChecked(true);
    m_mVel->setValue(45);

    form->addRow("Velocity", m_mVel);
    form->addRow("Target",   m_mTarget);
    form->addRow("",         m_mEnabled);
    vb->addLayout(form);

    auto* btnRow = new QHBoxLayout;
    auto* btnRemove = new QPushButton("Remove Motor");
    btnRow->addStretch();
    btnRow->addWidget(btnRemove);
    vb->addLayout(btnRow);

    connect(m_mVel,    &QDoubleSpinBox::valueChanged, this, &PropertiesPanel::onMotorEdited);
    connect(m_mTarget, &QDoubleSpinBox::valueChanged, this, &PropertiesPanel::onMotorEdited);
    connect(m_mEnabled,&QCheckBox::toggled,           this, &PropertiesPanel::onMotorEdited);
    connect(btnRemove, &QPushButton::clicked,         this, &PropertiesPanel::removeMotor);
}

// ── show / hide ───────────────────────────────────────────────────────────────

void PropertiesPanel::showJointGroup(bool show) { m_jointGroup->setVisible(show); }
void PropertiesPanel::showMotorGroup(bool show) { m_motorGroup->setVisible(show); }

// ── load from scene ───────────────────────────────────────────────────────────

void PropertiesPanel::onSelectionChanged(int id) {
    m_currentId = id;
    if(id == -1){
        m_transformGroup->setEnabled(false);
        showJointGroup(false);
        showMotorGroup(false);
        return;
    }
    m_transformGroup->setEnabled(true);
    loadObject(id);
}

void PropertiesPanel::loadObject(int id) {
    const SceneObject* obj = m_scene->getObject(id);
    if(!obj) return;

    m_loading = true;

    m_px->setValue(obj->transform.position.x());
    m_py->setValue(obj->transform.position.y());
    m_pz->setValue(obj->transform.position.z());
    m_rx->setValue(obj->transform.rotation.x());
    m_ry->setValue(obj->transform.rotation.y());
    m_rz->setValue(obj->transform.rotation.z());
    m_sx->setValue(obj->transform.scale.x());
    m_sy->setValue(obj->transform.scale.y());
    m_sz->setValue(obj->transform.scale.z());

    Joint* joint = m_scene->jointForChild(id);
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
        for(auto& [mid, m] : m_scene->allMotors()){
            if(m.jointId == joint->id){ motor = &const_cast<Motor&>(m); break; }
        }
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

// ── write-back ────────────────────────────────────────────────────────────────

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
    Joint* joint = m_scene->jointForChild(m_currentId);
    if(!joint) return;
    joint->type         = m_jType->currentIndex() == 0 ? JointType::Revolute : JointType::Prismatic;
    joint->axis         = { float(m_jAxisX->value()), float(m_jAxisY->value()), float(m_jAxisZ->value()) };
    joint->limitMin     = float(m_jLimitMin->value());
    joint->limitMax     = float(m_jLimitMax->value());
    joint->initialAngle = float(m_jInitAngle->value());
}

void PropertiesPanel::onMotorEdited() {
    if(m_loading) return;
    Joint* joint = m_scene->jointForChild(m_currentId);
    if(!joint) return;
    for(auto& [mid, motor] : m_scene->allMotors()){
        if(motor.jointId == joint->id){
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
    loadObject(m_currentId);
}

void PropertiesPanel::removeJoint() {
    Joint* joint = m_scene->jointForChild(m_currentId);
    if(!joint) return;
    m_scene->removeJoint(joint->id);
    showJointGroup(false);
    showMotorGroup(false);
}

void PropertiesPanel::addMotor() {
    Joint* joint = m_scene->jointForChild(m_currentId);
    if(!joint) return;
    Motor m;
    m.name    = "Motor_" + joint->name;
    m.jointId = joint->id;
    m_scene->addMotor(m);
    loadObject(m_currentId);
}

void PropertiesPanel::removeMotor() {
    Joint* joint = m_scene->jointForChild(m_currentId);
    if(!joint) return;
    for(auto& [mid, motor] : m_scene->allMotors()){
        if(motor.jointId == joint->id){
            m_scene->removeMotor(mid);
            break;
        }
    }
    showMotorGroup(false);
}
