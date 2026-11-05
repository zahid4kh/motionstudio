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
class PartFlowPage;

// Dockable automation environment: Signals / Actuators / Sequences / Runtime
// tabs, with a live problem indicator that compiles all definitions.
class AutomationPanel : public QWidget {
    Q_OBJECT
public:
    AutomationPanel(SceneGraph* scene, SimulationEngine* sim, QWidget* parent = nullptr);

    void showRuntime();
    void showProblems(const QStringList& problems);
    void showActuatorForJoint(int jointId);
    // Signals tab: a new sensor mounted on `objectId`, with its settings open
    void addSensorOn(int objectId);
    // Part Flow tab: a new source copying `objectId` / sink on it, settings open
    void addSourceFor(int objectId);
    void addSinkOn(int objectId);

signals:
    void statusMessage(const QString& msg, bool ok);
    void externalIoRequested();

private:
    void setLocked(bool locked);
    void revalidate();

    SceneGraph*       m_scene;
    SimulationEngine* m_sim;
    QTabWidget*       m_tabs = nullptr;
    QLabel*           m_lockLabel = nullptr;
    QLabel*           m_lockIcon  = nullptr;
    QToolButton*      m_problems = nullptr;
    SignalsPage*      m_signals = nullptr;
    ActuatorsPage*    m_actuators = nullptr;
    SequencesPage*    m_sequences = nullptr;
    RuntimePage*      m_runtime = nullptr;
    PartFlowPage*     m_flow = nullptr;
    QTimer            m_runtimeTimer;
    QTimer            m_validateTimer;
    QStringList       m_problemList;
};
