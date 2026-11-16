#pragma once
#include "AutomationCommon.h"

class QAction;
class QComboBox;
class QFormLayout;
class QLabel;
class QLineEdit;
class QPushButton;
class QTableWidget;
class QToolButton;

// Motions tab: the motions kept in the project (takes recorded with Record
// Values or imported from CSV) and the models trained on them (Train). Play
// one on its robot right away, rename it, give it to another robot, export
// it; Play Motion operations use them.
class MotionsPage : public autoui::Page {
    Q_OBJECT
public:
    MotionsPage(SceneGraph* scene, SimulationEngine* sim, QWidget* parent = nullptr);
    void refresh() override;
    void refreshRuntime() override;
    void setLocked(bool locked) override;

    // The Record Values action, shown as this tab's Record button
    void setRecordAction(QAction* record);
    void selectMotion(int id);

private:
    int  currentId() const;
    void loadDetails();
    void rename();
    void setRobot();
    void importCsv();
    void exportCsv();
    void remove();
    void play();
    void train();
    // The selected entry's name and robot, a motion's or a learned model's
    QString* nameRef(int id);
    int*     robotRef(int id);
    // A Play Motion operation for the selection (a learned model: the chosen clip)
    Operation previewOperation() const;
    // What the motion holds and how it fits its robot
    QString describe(const MotionClip& m) const;
    QString describe(const LearnedMotion& m) const;
    // How the joint columns fit the robot: "<verb> 5 of Arm's 6 actuators<more>." and what's left out
    QString fit(int robot, const QStringList& joints, const QString& verb, const QString& more) const;

    QTableWidget* m_list = nullptr;
    QToolButton*  m_record = nullptr;
    QPushButton*  m_import = nullptr, *m_export = nullptr, *m_delete = nullptr, *m_play = nullptr, *m_train = nullptr;
    QFormLayout*  m_form = nullptr;
    QComboBox*    m_clip = nullptr;     // a learned model's clip to play or export
    QLineEdit*    m_name = nullptr;
    QComboBox*    m_robot = nullptr;
    QLabel*       m_info = nullptr;
    QLabel*       m_empty = nullptr;
    int           m_playing = 0;    // the motion id of a preview under way
    int           m_selected = -1;
};
