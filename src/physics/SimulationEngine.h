#pragma once
#include <QObject>
#include <QTimer>
#include <unordered_map>
#include "../scene/SceneGraph.h"

#ifdef MUJOCO_ENABLED
#include <mujoco/mujoco.h>
#endif

class SimulationEngine : public QObject {
    Q_OBJECT
public:
    explicit SimulationEngine(SceneGraph* scene, QObject* parent = nullptr);
    ~SimulationEngine();

    bool   isRunning()    const { return m_running; }
    double elapsedTime()  const { return m_elapsed; }

public slots:
    void play();
    void pause();
    void reset();

signals:
    void stateChanged(bool running);
    void timeUpdated(double seconds);
    void stepped();

private slots:
    void tick();

private:
    // ── Kinematic (joint/motor only) ─────────────────────────────────────────
    void applyMotors(double dt);
    void resetJoints();

    // ── MuJoCo backend ───────────────────────────────────────────────────────
#ifdef MUJOCO_ENABLED
    bool    buildMuJoCoModel();
    void    teardownMuJoCo();
    void    syncFromMuJoCo();
    void    setMuJoCoActuators();
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
    std::unordered_map<int, int> m_motorActuatorId;   // scene motor id → mujoco actuator id
    std::unordered_map<int, int> m_freeJointQposAddr; // scene object id → freejoint qpos address
    std::unordered_map<int, int> m_freeJointDofAddr;  // scene object id → freejoint dof address

    void applyConveyorBelts();
#else
    void teardownMuJoCo() {}
#endif

    SceneGraph*                        m_scene;
    QTimer                             m_timer;
    bool                               m_running = false;
    double                             m_elapsed = 0.0;
    std::unordered_map<int, float>     m_motorDirs;
    std::unordered_map<int, Transform> m_savedTransforms;
    std::unordered_map<int, float>     m_savedJointAngles;

    static constexpr int    kTickMs = 16;
    static constexpr double kDt     = kTickMs / 1000.0;
};
