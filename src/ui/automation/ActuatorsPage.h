#pragma once
#include "AutomationCommon.h"
#include <vector>

class QTableWidget;
class QListWidget;
class QPushButton;
class QToolButton;
class QComboBox;
class QDoubleSpinBox;
class QCheckBox;
class QLineEdit;
class QSlider;
class QLabel;
class QGroupBox;
class QVBoxLayout;

// Actuators tab: list of actuators (motors) with a detail form for the
// selected one (mode, speeds, expression bindings), joint jog sliders for
// posing the mechanism at edit time, and named poses (teach points).
class ActuatorsPage : public autoui::Page {
    Q_OBJECT
public:
    ActuatorsPage(SceneGraph* scene, SimulationEngine* sim, QWidget* parent = nullptr);
    void refresh() override;
    void refreshRuntime() override;
    void setLocked(bool locked) override;

    void selectActuatorForJoint(int jointId);

private:
    void rebuildTable();
    void rebuildJog();
    void rebuildPoses();
    // The "Next:" line: the first thing the project still needs
    void updateGuidance();
    void loadForm();
    int  currentMotorId() const;
    void commitForm();
    void addActuatorMenu();
    void deleteActuator();
    void renameActuator();

    void capturePose();
    void updatePose();
    void goToPose();
    void deletePose();
    int  currentPoseId() const;

    // Actuator list + form
    QTableWidget*   m_table = nullptr;
    QToolButton*    m_addBtn = nullptr;
    QPushButton*    m_delBtn = nullptr;
    QGroupBox*      m_form = nullptr;
    QLineEdit*      m_name = nullptr;
    QLabel*         m_jointLabel = nullptr;
    QComboBox*      m_mode = nullptr;
    QDoubleSpinBox* m_velocity = nullptr;
    QDoubleSpinBox* m_accel = nullptr;
    QCheckBox*      m_active = nullptr;
    ExpressionEdit* m_bindTarget = nullptr;
    ExpressionEdit* m_bindVelocity = nullptr;
    ExpressionEdit* m_bindActive = nullptr;
    QLabel*         m_velLabel = nullptr;
    QLabel*         m_modeHint = nullptr;   // what the selected actuator does on Play
    QLabel*         m_nextStep = nullptr;

    // Jog
    struct JogRow { int jointId; QSlider* slider; QDoubleSpinBox* spin; double scale; };
    std::vector<JogRow> m_jogRows;
    QWidget*        m_jogHost = nullptr;
    QVBoxLayout*    m_jogLayout = nullptr;
    QPushButton*    m_toInitBtn = nullptr;
    QPushButton*    m_setInitBtn = nullptr;

    // Poses
    QListWidget*    m_poses = nullptr;
    QPushButton*    m_poseCapture = nullptr, *m_poseUpdate = nullptr, *m_poseGo = nullptr, *m_poseDelete = nullptr;

    bool            m_loading = false;
    int             m_selectedMotor = -1;
};
