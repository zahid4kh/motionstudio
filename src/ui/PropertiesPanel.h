#pragma once
#include <QWidget>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QLabel>
#include <QPushButton>
#include <vector>
#include "../scene/SceneGraph.h"

enum class LengthUnit { Meters, Millimeters };

class PropertiesPanel : public QWidget {
    Q_OBJECT
public:
    explicit PropertiesPanel(SceneGraph* scene, QWidget* parent = nullptr);

public slots:
    void showPointCoords(const QString& name, const QVector3D& pos);

private slots:
    void onSelectionChanged(int id);
    void onTransformEdited();
    void onJointEdited();
    void onMotorEdited();
    void addJoint();
    void removeJoint();
    void addMotor();
    void removeMotor();
    void onRigidBodyEdited();
    void addRigidBody();
    void removeRigidBody();
    void addCollision();
    void removeCollision();
    void onCollisionEdited();
    void onCollisionTypeChanged(int idx);
    void snapToFloor();
    void addConveyorBelt();
    void removeConveyorBelt();
    void onConveyorEdited();
    void onUnitChanged(int idx);

private:
    void buildNameSection();
    void buildPointSection();
    void buildTransformSection();
    void buildConnectionsSection();
    void buildJointSection();
    void buildMotorSection();
    void buildPhysicsSection();
    void buildConveyorSection();

    void loadObject(int id);
    void showJointGroup(bool show);
    void showMotorGroup(bool show);
    void showConnectionsGroup(bool show);
    void updatePhysicsUI(int objectId);
    void updateConveyorUI(int objectId);
    void updateCollisionRows(int colTypeIdx);
    void updateJointUnits(bool isRevolute);

    QDoubleSpinBox* makeSpin(float mn, float mx, float step, const QString& suffix = {});
    QDoubleSpinBox* makeLengthSpin(float mn, float mx, float step, const QString& baseSuffix = " m");
    double unitFactor() const { return m_lengthUnit == LengthUnit::Millimeters ? 1000.0 : 1.0; }

    SceneGraph* m_scene;
    int         m_currentId = -1;
    bool        m_loading   = false;

    // Unit system
    LengthUnit  m_lengthUnit = LengthUnit::Meters;
    QComboBox*  m_unitCombo  = nullptr;
    struct LengthField {
        QDoubleSpinBox* spin;
        double baseMin, baseMax, baseStep;
        QString baseSuffix;
    };
    std::vector<LengthField> m_lengthFields;

    // Last picked point, kept so its display can be re-rendered when units change
    QString    m_lastPointName;
    QVector3D  m_lastPointPos;

    // Name card
    QLabel*         m_nameLabel  = nullptr;
    QLabel*         m_typeLabel  = nullptr;

    // Selected point coordinates
    QGroupBox*      m_pointGroup     = nullptr;
    QLabel*         m_pointNameLabel = nullptr;
    QLabel*         m_ptX = nullptr, *m_ptY = nullptr, *m_ptZ = nullptr;

    // Transform
    QGroupBox*      m_transformGroup  = nullptr;
    QDoubleSpinBox* m_px = nullptr, *m_py = nullptr, *m_pz = nullptr;
    QDoubleSpinBox* m_rx = nullptr, *m_ry = nullptr, *m_rz = nullptr;
    QDoubleSpinBox* m_sx = nullptr, *m_sy = nullptr, *m_sz = nullptr;
    QPushButton*    m_snapToFloorBtn  = nullptr;

    // Connections
    QGroupBox*      m_connectionsGroup = nullptr;
    QPushButton*    m_addJointBtn      = nullptr;

    // Joint
    QGroupBox*      m_jointGroup  = nullptr;
    QComboBox*      m_jType       = nullptr;
    QDoubleSpinBox* m_jAxisX = nullptr, *m_jAxisY = nullptr, *m_jAxisZ = nullptr;
    QDoubleSpinBox* m_jLimitMin   = nullptr;
    QDoubleSpinBox* m_jLimitMax   = nullptr;
    QDoubleSpinBox* m_jInitAngle  = nullptr;
    QLabel*         m_jInitLabel  = nullptr;
    QPushButton*    m_addMotorBtn = nullptr;

    // Motor
    QGroupBox*      m_motorGroup  = nullptr;
    QDoubleSpinBox* m_mVel        = nullptr;
    QDoubleSpinBox* m_mTarget     = nullptr;
    QCheckBox*      m_mEnabled    = nullptr;

    // Physics — rigid body
    QGroupBox*      m_physicsGroup  = nullptr;
    QPushButton*    m_rbAddBtn      = nullptr;
    QWidget*        m_rbPropsWidget = nullptr;
    QDoubleSpinBox* m_rbMass        = nullptr;
    QCheckBox*      m_rbGravity     = nullptr;
    QPushButton*    m_colAddBtn     = nullptr;
    QPushButton*    m_rbRemBtn      = nullptr;

    // Physics — collision body
    QWidget*        m_colWidget     = nullptr;
    QComboBox*      m_colType       = nullptr;
    QWidget*        m_colBoxRow     = nullptr;
    QDoubleSpinBox* m_cHW = nullptr, *m_cHH = nullptr, *m_cHD = nullptr;
    QWidget*        m_colRadiusRow  = nullptr;
    QDoubleSpinBox* m_cRadius       = nullptr;
    QWidget*        m_colHeightRow  = nullptr;
    QDoubleSpinBox* m_cHeight       = nullptr;
    QDoubleSpinBox* m_cOffX = nullptr, *m_cOffY = nullptr, *m_cOffZ = nullptr;
    QPushButton*    m_colRemBtn     = nullptr;

    // Conveyor belt
    QGroupBox*      m_conveyorGroup   = nullptr;
    QPushButton*    m_cvAddBtn        = nullptr;
    QWidget*        m_cvPropsWidget   = nullptr;
    QDoubleSpinBox* m_cvVelocity      = nullptr;
    QComboBox*      m_cvDir           = nullptr;
    QCheckBox*      m_cvEnabled       = nullptr;
    QPushButton*    m_cvRemBtn        = nullptr;
};
