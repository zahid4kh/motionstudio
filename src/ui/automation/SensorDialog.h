#pragma once
#include <QDialog>
#include "automation/Automation.h"

class SceneGraph;
class QComboBox;
class QDoubleSpinBox;
class QCheckBox;
class QLabel;

// Edits a sensor signal's configuration: beam or zone, what it is mounted
// on, where it points and what it detects. Shows what the sensor sees in the
// scene's current pose as you edit.
class SensorDialog : public QDialog {
    Q_OBJECT
public:
    SensorDialog(const SceneGraph* scene, const QString& name, const SensorConfig& cfg, QWidget* parent = nullptr);
    SensorConfig config() const;

private:
    void updateForm();

    const SceneGraph* m_scene;
    QComboBox*      m_type = nullptr;
    QComboBox*      m_mount = nullptr;
    QDoubleSpinBox* m_offset[3] = {};
    QComboBox*      m_direction = nullptr;
    QDoubleSpinBox* m_range = nullptr;
    QCheckBox*      m_bounds = nullptr;
    QDoubleSpinBox* m_size[3] = {};
    QComboBox*      m_target = nullptr;
    QComboBox*      m_targetObject = nullptr;
    class QFormLayout* m_form = nullptr;
    class QLayout*  m_sizeRow = nullptr;
    QLabel*         m_reading = nullptr;
    QVector3D       m_customDirection;   // a direction not on a local axis, kept as is
};
