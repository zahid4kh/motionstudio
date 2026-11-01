#include "SensorDialog.h"
#include "AutomationCommon.h"
#include "automation/Sensors.h"
#include "scene/SceneGraph.h"
#include "ui/Theme.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace {
const QVector3D kAxes[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
const char* kAxisNames[6] = {"+X", "−X", "+Y", "−Y", "+Z", "−Z"};

QDoubleSpinBox* metres(double lo, double hi, double v) {
    auto* sp = new QDoubleSpinBox;
    sp->setRange(lo, hi);
    sp->setDecimals(3);
    sp->setSingleStep(0.01);
    sp->setSuffix(" m");
    sp->setValue(v);
    return sp;
}
} // namespace

SensorDialog::SensorDialog(const SceneGraph* scene, const QString& name, const SensorConfig& cfg, QWidget* parent)
    : QDialog(parent), m_scene(scene), m_customDirection(cfg.direction)
{
    setWindowTitle("Sensor " + name);
    setMinimumWidth(460);
    auto* vb = new QVBoxLayout(this);
    vb->setSpacing(10);
    vb->setContentsMargins(18, 16, 18, 16);

    auto* title = new QLabel("Sensor " + name);
    title->setObjectName("dialogTitle");
    vb->addWidget(title);
    auto* sub = new QLabel("A sensor is a Bool signal that is true while something is in its beam or zone. "
                           "It moves with the object it is mounted on.");
    sub->setObjectName("hint");
    sub->setWordWrap(true);
    vb->addWidget(sub);

    auto* form = new QFormLayout;
    form->setLabelAlignment(Qt::AlignRight);
    vb->addLayout(form);

    m_type = new QComboBox;
    m_type->addItem(Theme::icon("scan-line"), "Beam: light barrier / part present", int(SensorType::Beam));
    m_type->addItem(Theme::icon("box"),       "Zone: presence / contact",          int(SensorType::Zone));
    m_type->setCurrentIndex(cfg.type == SensorType::Zone ? 1 : 0);
    form->addRow("Type", m_type);

    m_mount = new QComboBox;
    autoui::fillObjectCombo(m_mount, *scene, cfg.objectId, "World (fixed)");
    m_mount->setToolTip("The sensor moves with this object. It never detects this object, its children "
                        "or the objects it hangs from.");
    form->addRow("Mounted on", m_mount);

    auto* offRow = new QHBoxLayout;
    for (int i = 0; i < 3; ++i) {
        m_offset[i] = metres(-100, 100, cfg.offset[i]);
        m_offset[i]->setPrefix(QString("XYZ"[i]) + "  ");
        offRow->addWidget(m_offset[i]);
    }
    form->addRow("Offset", offRow);

    // Beam rows
    m_direction = new QComboBox;
    int dirIdx = -1;
    for (int i = 0; i < 6; ++i) {
        m_direction->addItem(QString("%1 of the mount").arg(kAxisNames[i]));
        if (cfg.direction.normalized() == kAxes[i]) dirIdx = i;
    }
    if (dirIdx < 0) {
        m_direction->addItem(QString("Custom (%1, %2, %3)").arg(double(cfg.direction.x()), 0, 'g', 3)
                                 .arg(double(cfg.direction.y()), 0, 'g', 3).arg(double(cfg.direction.z()), 0, 'g', 3));
        dirIdx = 6;
    }
    m_direction->setCurrentIndex(dirIdx);
    form->addRow("Direction", m_direction);
    m_range = metres(0.001, 100, cfg.range);
    form->addRow("Range", m_range);

    // Zone rows
    m_bounds = new QCheckBox("Use the mounted object's bounding box (contact sensor)");
    m_bounds->setChecked(cfg.objectBounds);
    form->addRow("", m_bounds);
    auto* sizeRow = new QHBoxLayout;
    for (int i = 0; i < 3; ++i) {
        m_size[i] = metres(0.001, 100, cfg.size[i]);
        m_size[i]->setPrefix(QString("XYZ"[i]) + "  ");
        sizeRow->addWidget(m_size[i]);
    }
    form->addRow("Size", sizeRow);
    m_form = form;
    m_sizeRow = sizeRow;

    m_target = new QComboBox;
    m_target->addItem("Anything", int(SensorTarget::Any));
    m_target->addItem("Rigid bodies (parts)", int(SensorTarget::RigidBodies));
    m_target->addItem("A specific object", int(SensorTarget::Object));
    m_target->setCurrentIndex(int(cfg.target));
    m_targetObject = new QComboBox;
    autoui::fillObjectCombo(m_targetObject, *scene, cfg.targetId);
    auto* tRow = new QHBoxLayout;
    tRow->addWidget(m_target);
    tRow->addWidget(m_targetObject, 1);
    form->addRow("Detects", tRow);

    m_reading = new QLabel;
    m_reading->setObjectName("hint");
    m_reading->setWordWrap(true);
    vb->addWidget(m_reading);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    btns->button(QDialogButtonBox::Ok)->setDefault(true);
    vb->addWidget(btns);
    connect(btns, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, this, &QDialog::reject);

    for (auto* c : {m_type, m_mount, m_direction, m_target, m_targetObject})
        connect(c, &QComboBox::currentIndexChanged, this, &SensorDialog::updateForm);
    for (auto* sp : {m_offset[0], m_offset[1], m_offset[2], m_range, m_size[0], m_size[1], m_size[2]})
        connect(sp, &QDoubleSpinBox::valueChanged, this, &SensorDialog::updateForm);
    connect(m_bounds, &QCheckBox::toggled, this, &SensorDialog::updateForm);
    updateForm();
}

SensorConfig SensorDialog::config() const {
    SensorConfig c;
    c.type      = SensorType(m_type->currentData().toInt());
    c.objectId  = m_mount->currentData().toInt();
    c.offset    = {float(m_offset[0]->value()), float(m_offset[1]->value()), float(m_offset[2]->value())};
    const int d = m_direction->currentIndex();
    c.direction = d >= 0 && d < 6 ? kAxes[d] : m_customDirection;
    c.range     = float(m_range->value());
    c.size      = {float(m_size[0]->value()), float(m_size[1]->value()), float(m_size[2]->value())};
    c.objectBounds = m_bounds->isChecked();
    c.target    = SensorTarget(m_target->currentData().toInt());
    c.targetId  = c.target == SensorTarget::Object ? m_targetObject->currentData().toInt() : -1;
    return c;
}

void SensorDialog::updateForm() {
    const SensorConfig c = config();
    const bool beam = c.type == SensorType::Beam;
    m_form->setRowVisible(m_direction, beam);
    m_form->setRowVisible(m_range, beam);
    m_form->setRowVisible(m_bounds, !beam);
    m_form->setRowVisible(m_sizeRow, !beam);
    m_bounds->setEnabled(c.objectId != -1);
    const bool sized = !(c.objectBounds && c.objectId != -1);
    for (auto* sp : m_size) sp->setEnabled(sized);
    m_targetObject->setVisible(c.target == SensorTarget::Object);

    QString text;
    const QString problem = sensors::problem(*m_scene, c);
    if (!problem.isEmpty()) {
        text = "⚠ " + problem;
    } else {
        const sensors::Reading r = sensors::read(*m_scene, c);
        const SceneObject* o = m_scene->getObject(r.objectId);
        if (!r.active)
            text = "In the current pose: nothing detected (false).";
        else if (beam)
            text = QString("In the current pose: detects %1 at %2 m (true).")
                       .arg(o ? o->name : QString("something")).arg(double(r.distance), 0, 'f', 3);
        else
            text = QString("In the current pose: detects %1 (true).").arg(o ? o->name : QString("something"));
    }
    m_reading->setText(text);
    adjustSize();
}
