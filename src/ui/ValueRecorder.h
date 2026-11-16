#pragma once
#include <QObject>
#include <QString>
#include <memory>
#include <vector>

class QAction;
class QWidget;
class SceneGraph;
class SimulationEngine;
namespace take { class Recorder; }

// Record Values (Simulation toolbar, Tools menu): records one robot's joint
// values and where it stands, every scan or at a rate, while the simulation
// runs, and saves them as a take (CSV, see take::Take) when you stop it,
// when the robot stands still again, or on Reset. Pressing it starts Play.
class ValueRecorder : public QObject {
    Q_OBJECT
public:
    ValueRecorder(SceneGraph* scene, SimulationEngine* sim, QWidget* window);
    ~ValueRecorder() override;

    QAction* action() const { return m_action; }
    bool     recording() const { return bool(m_rec); }

signals:
    void statusMessage(const QString& msg, bool ok);

private:
    void toggle();
    bool begin();                    // asks what to record; false when cancelled
    void onScan();
    void finish(const QString& why);
    void sync();

    SceneGraph*       m_scene;
    SimulationEngine* m_sim;
    QWidget*          m_window;
    QAction*          m_action = nullptr;
    std::unique_ptr<take::Recorder> m_rec;
    QString           m_path;
    bool              m_stopWhenStill = true;
    bool              m_moved = false;
    double            m_stillSince = -1;
    std::vector<int>  m_compare;     // columns that say it moved
};
