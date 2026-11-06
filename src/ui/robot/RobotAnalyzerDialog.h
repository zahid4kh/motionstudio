#pragma once
#include <QDialog>
#include <QTimer>
#include <vector>
#include "robot/RobotAnalysis.h"

class QComboBox;
class QLabel;
class RobotAnalysisPanel;
class SceneGraph;

// Tools › Analyze Robot: the workspace, DH parameters, dexterity and
// singularities of any robot in the scene (built, imported or modelled by
// hand), following its pose live while it is jogged or simulated.
class RobotAnalyzerDialog : public QDialog {
    Q_OBJECT
public:
    RobotAnalyzerDialog(SceneGraph* scene, QWidget* parent = nullptr);

    // Analyses the robot `objectId` belongs to; false when there is none
    bool analyzeObject(int objectId);

private:
    void fillTools();
    void recompute();
    void poll();

    SceneGraph* m_scene;
    int m_tool = -1;
    robot::Analysis m_analysis;
    std::vector<float> m_lastPose;
    QTimer m_poll;
    QComboBox* m_tools;
    QLabel* m_status;
    RobotAnalysisPanel* m_panel;
};
