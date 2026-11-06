#pragma once
#include <QWidget>
#include "robot/RobotAnalysis.h"

class QLabel;
class QProgressBar;
class QPushButton;
class QTableWidget;
class RobotSketch;
class SceneGraph;

// Shows a robot::Analysis: the workspace drawn over the robot (side or top),
// reach, height range and volume, the current pose's dexterity and
// singularities, and the axes with their DH parameters.
class RobotAnalysisPanel : public QWidget {
    Q_OBJECT
public:
    explicit RobotAnalysisPanel(QWidget* parent = nullptr);

    // Everything, for the robot in `g` (the parts under its first joint's parent)
    void setAnalysis(const robot::Analysis& a, const SceneGraph& g);
    // Only what depends on the pose (after robot::analyzePose)
    void setPose(const robot::Analysis& a, const SceneGraph& g);
    void setError(const QString& text);

private:
    void fillTable(const robot::Analysis& a);
    void capture(const robot::Analysis& a, const SceneGraph& g);

    RobotSketch*  m_sketch;
    QPushButton*  m_side;
    QPushButton*  m_top;
    QLabel*       m_reach;
    QLabel*       m_minReach;
    QLabel*       m_height;
    QLabel*       m_volume;
    QLabel*       m_sweep;
    QLabel*       m_tcp;
    QProgressBar* m_dex;
    QLabel*       m_dexLabel;
    QLabel*       m_warnings;
    QTableWidget* m_table;
};
