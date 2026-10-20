#pragma once
#include <QObject>
#include <QTimer>
#include <unordered_map>
#include <QStringList>
#include "../scene/SceneGraph.h"
#include "../automation/AutomationEngine.h"

#ifdef MUJOCO_ENABLED
#include <mujoco/mujoco.h>
#endif

class SimulationEngine : public QObject {
    Q_OBJECT
public:
    explicit SimulationEngine(SceneGraph* scene, QObject* parent = nullptr);
    ~SimulationEngine();

    bool   isRunning()    const { return m_running; }
    // True from the first Play/Step until Reset: the scene is in "simulation
    // state" and automation definitions are locked.
    bool   isActive()     const { return m_active; }
    double elapsedTime()  const { return m_elapsed; }
    double speedFactor()  const { return m_speed; }
    AutomationEngine* automation() const { return m_auto; }

    static constexpr int    kTickMs = 16;
    static constexpr double kDt     = kTickMs / 1000.0;

public slots:
    void play();
    void pause();
    void step();            // exactly one scan, then paused
    void reset();
    void setSpeedFactor(double f);

signals:
    void stateChanged(bool running);
    void activeChanged(bool active);
    void timeUpdated(double seconds);
    void stepped();
    void startFailed(const QStringList& errors);

private slots:
    void tick();

private:
    bool activate();        // compile automation, snapshot scene, build physics
    void runScan();

    // ── MuJoCo backend ───────────────────────────────────────────────────────
#ifdef MUJOCO_ENABLED
    bool    buildMuJoCoModel();
    void    teardownMuJoCo();
    void    syncFromMuJoCo();
    void    prescribeActuatedJoints(double alpha);
    QString buildMJCF() const;
    void    writeBodyXML(QString& xml, int objectId, int indent) const;
    void    writeGeomXML(QString& xml, const SceneObject* obj,
                         const RigidBody* rb, const QString& pad) const;
    void    buildIdMappings();

    mjModel* m_mjModel = nullptr;
    mjData*  m_mjData  = nullptr;

    // Our IDs → MuJoCo IDs
    std::unordered_map<int, int> m_jointQposAddr;     // scene joint id → qpos address
    std::unordered_map<int, int> m_bodyMjId;          // scene object id → mujoco body id
    std::unordered_map<int, int> m_jointDofAddr;      // scene joint id → dof address
    // Actuated joint positions before/after the current automation scan, for
    // interpolating prescribed motion across MuJoCo sub-steps
    std::unordered_map<int, std::pair<double, double>> m_prescribed;
    std::unordered_map<int, int> m_freeJointQposAddr; // scene object id → freejoint qpos address
    std::unordered_map<int, int> m_freeJointDofAddr;  // scene object id → freejoint dof address

    void applyConveyorBelts();
#else
    void teardownMuJoCo() {}
#endif

    SceneGraph*                        m_scene;
    AutomationEngine*                  m_auto;
    QTimer                             m_timer;
    bool                               m_running = false;
    bool                               m_active  = false;
    double                             m_elapsed = 0.0;
    double                             m_speed   = 1.0;
    double                             m_accum   = 0.0;
    std::unordered_map<int, Transform> m_savedTransforms;
    std::unordered_map<int, float>     m_savedJointAngles;
};
