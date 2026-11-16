#pragma once
#include "AutomationCommon.h"

class QAction;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTableWidget;
class QToolButton;

// Motions tab: the motions kept in the project (takes recorded with Record
// Values or imported from CSV). Play one on its robot right away, rename it,
// give it to another robot, export it; Play Motion operations use them.
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
    // What the motion holds and how it fits its robot
    QString describe(const MotionClip& m) const;

    QTableWidget* m_list = nullptr;
    QToolButton*  m_record = nullptr;
    QPushButton*  m_import = nullptr, *m_export = nullptr, *m_delete = nullptr, *m_play = nullptr;
    QLineEdit*    m_name = nullptr;
    QComboBox*    m_robot = nullptr;
    QLabel*       m_info = nullptr;
    QLabel*       m_empty = nullptr;
    int           m_playing = 0;    // the motion id of a preview under way
    int           m_selected = -1;
};
