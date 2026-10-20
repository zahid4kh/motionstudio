#pragma once
#include <QWidget>
#include <QTimer>

class SceneGraph;
class SimulationEngine;
class QTabWidget;
class QLabel;
class QToolButton;
class SignalsPage;
class ActuatorsPage;
class SequencesPage;
class RuntimePage;

// Dockable automation environment: Signals / Actuators / Sequences / Runtime
// tabs, with a live problem indicator that compiles all definitions.
class AutomationPanel : public QWidget {
    Q_OBJECT
public:
    AutomationPanel(SceneGraph* scene, SimulationEngine* sim, QWidget* parent = nullptr);

    void showRuntime();
    void showProblems(const QStringList& problems);
    void showActuatorForJoint(int jointId);

signals:
    void statusMessage(const QString& msg, bool ok);

private:
    void setLocked(bool locked);
    void revalidate();

    SceneGraph*       m_scene;
    SimulationEngine* m_sim;
    QTabWidget*       m_tabs = nullptr;
    QLabel*           m_lockLabel = nullptr;
    QToolButton*      m_problems = nullptr;
    SignalsPage*      m_signals = nullptr;
    ActuatorsPage*    m_actuators = nullptr;
    SequencesPage*    m_sequences = nullptr;
    RuntimePage*      m_runtime = nullptr;
    QTimer            m_runtimeTimer;
    QTimer            m_validateTimer;
    QStringList       m_problemList;
};
