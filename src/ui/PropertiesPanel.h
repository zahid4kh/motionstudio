#pragma once
#include <QWidget>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QLabel>
#include <QPushButton>
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
    void buildNameSection();
    void buildTransformSection();
    void buildConnectionsSection();   // "Add Joint to Parent" when no joint
    void buildJointSection();
    void buildMotorSection();

    void loadObject(int id);
    void showJointGroup(bool show);
    void showMotorGroup(bool show);
    void showConnectionsGroup(bool show);

    QDoubleSpinBox* makeSpin(float mn, float mx, float step, const QString& suffix = {});

    SceneGraph* m_scene;
    int         m_currentId = -1;
    bool        m_loading   = false;

    // Name / header
    QLabel*         m_nameLabel  = nullptr;
    QLabel*         m_typeLabel  = nullptr;

    // Transform
    QGroupBox*      m_transformGroup = nullptr;
    QDoubleSpinBox* m_px = nullptr, *m_py = nullptr, *m_pz = nullptr;
    QDoubleSpinBox* m_rx = nullptr, *m_ry = nullptr, *m_rz = nullptr;
    QDoubleSpinBox* m_sx = nullptr, *m_sy = nullptr, *m_sz = nullptr;

    // Connections (shown when parent exists but no joint)
    QGroupBox*      m_connectionsGroup = nullptr;
    QPushButton*    m_addJointBtn      = nullptr;

    // Joint
    QGroupBox*      m_jointGroup  = nullptr;
    QComboBox*      m_jType       = nullptr;
    QDoubleSpinBox* m_jAxisX = nullptr, *m_jAxisY = nullptr, *m_jAxisZ = nullptr;
    QDoubleSpinBox* m_jLimitMin   = nullptr;
    QDoubleSpinBox* m_jLimitMax   = nullptr;
    QDoubleSpinBox* m_jInitAngle  = nullptr;
    QPushButton*    m_addMotorBtn = nullptr;   // shown when joint has no motor

    // Motor
    QGroupBox*      m_motorGroup  = nullptr;
    QDoubleSpinBox* m_mVel        = nullptr;
    QDoubleSpinBox* m_mTarget     = nullptr;
    QCheckBox*      m_mEnabled    = nullptr;
};
