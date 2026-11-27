#pragma once
#include <QObject>
#include <QTimer>
#include <unordered_map>
#include <QStringList>
#include "../scene/SceneGraph.h"
#include "../automation/AutomationEngine.h"
#include "Recording.h"
#include "../scene/Interference.h"

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

    // ── Recording / timeline ────────────────────────────────────────────────
    // Every scan since the first Play/Step is recorded until Reset.
    const Recording& recording() const { return m_rec; }
    // While paused, shows the recorded state of `frame` in the scene (the
    // simulation itself stays at its latest scan). Play or Step return to
    // the latest scan first.
    void   seek(int frame);
    void   seekLive();
    int    viewFrame() const;                 // frame on display (the last one when live)
    bool   isScrubbing() const { return m_viewFrame >= 0; }
    // Interference check between moving parts, run every other scan while
    // enabled (takes effect at the next Play/Step from reset)
    void   setInterferenceCheck(bool on) { m_checkInterference = on; }
    bool   interferenceCheck() const { return m_checkInterference; }
    const InterferenceChecker& interference() const { return m_interference; }

    // A slot's value on display: the recorded one while scrubbing
    double displayValue(int slot) const;
    double displayValue(const QString& name) const { return displayValue(m_auto->slotOf(name)); }

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
    void scrubbed(int frame);                 // a recorded frame (or live, -1) is now on display
    void stepped();
    void startFailed(const QStringList& errors);

private slots:
    void tick();

private:
    bool activate();        // compile automation, snapshot scene, build physics
    void runScan();
    void capture() { m_rec.capture(*m_scene, m_elapsed, m_auto->values()); }

    // ── MuJoCo backend ───────────────────────────────────────────────────────
#ifdef MUJOCO_ENABLED
    bool    buildMuJoCoModel();
    // Rebuilds the model after parts were added or taken out, keeping every
    // body's and joint's position and velocity
    void    rebuildMuJoCoModel();
    void    teardownMuJoCo();
    void    syncFromMuJoCo();
    void    prescribeActuatedJoints(double alpha);
    void    prescribeHeldParts();
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
    // Each belt's world transform at the start of the scan: a belt that moves
    // (on a turntable, a lift) carries what lies on it
    std::unordered_map<int, QMatrix4x4> m_beltStart;

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
    Recording                          m_rec;
    InterferenceChecker                m_interference;
    bool                               m_checkInterference = false;
    long                               m_scans = 0;
    int                                m_viewFrame = -1;   // -1 = live
    // The live state, kept while a recorded frame is on display
    std::unordered_map<int, Transform> m_liveTransforms;
    std::unordered_map<int, float>     m_liveJoints;
    std::unordered_map<int, bool>      m_liveParked;
    std::unordered_map<int, Transform> m_savedTransforms;
    std::unordered_map<int, float>     m_savedJointAngles;
};
