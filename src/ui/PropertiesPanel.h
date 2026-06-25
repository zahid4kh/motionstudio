#pragma once
#include <QWidget>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QLabel>
#include "../scene/SceneGraph.h"

class PropertiesPanel : public QWidget {
    Q_OBJECT
public:
    explicit PropertiesPanel(SceneGraph* scene, QWidget* parent = nullptr);

private slots:
    void onSelectionChanged(int id);
    void onTransformEdited();
    void onJointEdited();
    void onMotorEdited();
    void addJoint();
    void removeJoint();
    void addMotor();
    void removeMotor();

private:
    void buildTransformSection();
    void buildJointSection();
    void buildMotorSection();

    void loadObject(int id);
    void showJointGroup(bool show);
    void showMotorGroup(bool show);

    QDoubleSpinBox* spinXYZ(QWidget* parent, float min, float max, float step);

    SceneGraph* m_scene;
    int         m_currentId = -1;
    bool        m_loading   = false;

    // Transform
    QGroupBox*      m_transformGroup = nullptr;
    QDoubleSpinBox* m_px = nullptr, *m_py = nullptr, *m_pz = nullptr;
    QDoubleSpinBox* m_rx = nullptr, *m_ry = nullptr, *m_rz = nullptr;
    QDoubleSpinBox* m_sx = nullptr, *m_sy = nullptr, *m_sz = nullptr;

    // Joint
    QGroupBox*      m_jointGroup  = nullptr;
    QComboBox*      m_jType       = nullptr;
    QDoubleSpinBox* m_jAxisX = nullptr, *m_jAxisY = nullptr, *m_jAxisZ = nullptr;
    QDoubleSpinBox* m_jLimitMin   = nullptr;
    QDoubleSpinBox* m_jLimitMax   = nullptr;
    QDoubleSpinBox* m_jInitAngle  = nullptr;

    // Motor
    QGroupBox*      m_motorGroup  = nullptr;
    QDoubleSpinBox* m_mVel        = nullptr;
    QDoubleSpinBox* m_mTarget     = nullptr;
    QCheckBox*      m_mEnabled    = nullptr;
};
