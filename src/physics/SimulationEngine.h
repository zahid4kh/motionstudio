#pragma once
#include <QObject>
#include <QTimer>
#include <unordered_map>
#include "../scene/SceneGraph.h"

class SimulationEngine : public QObject {
    Q_OBJECT
public:
    explicit SimulationEngine(SceneGraph* scene, QObject* parent = nullptr);

    bool   isRunning()    const { return m_running; }
    double elapsedTime()  const { return m_elapsed; }

public slots:
    void play();
    void pause();
    void reset();

signals:
    void stateChanged(bool running);
    void timeUpdated(double seconds);
    void stepped();   // emitted each tick so Viewport can repaint

private slots:
    void tick();

private:
    void applyMotors(double dt);
    void resetJoints();

    SceneGraph*                    m_scene;
    QTimer                         m_timer;
    bool                           m_running = false;
    double                         m_elapsed = 0.0;
    std::unordered_map<int,float>  m_motorDirs; // per-motor direction (+1/-1)

    static constexpr int    kTickMs = 16;
    static constexpr double kDt     = kTickMs / 1000.0;
};
