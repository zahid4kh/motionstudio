#include "SimulationEngine.h"
#include <algorithm>

SimulationEngine::SimulationEngine(SceneGraph* scene, QObject* parent)
    : QObject(parent), m_scene(scene)
{
    m_timer.setInterval(kTickMs);
    connect(&m_timer, &QTimer::timeout, this, &SimulationEngine::tick);
}

void SimulationEngine::play() {
    if(m_running) return;
    m_running = true;
    m_timer.start();
    emit stateChanged(true);
}

void SimulationEngine::pause() {
    if(!m_running) return;
    m_running = false;
    m_timer.stop();
    emit stateChanged(false);
}

void SimulationEngine::reset() {
    pause();
    m_elapsed = 0.0;
    m_motorDirs.clear();
    resetJoints();
    emit timeUpdated(0.0);
}

void SimulationEngine::tick() {
    applyMotors(kDt);
    m_elapsed += kDt;
    emit timeUpdated(m_elapsed);
    emit stepped();
}

void SimulationEngine::applyMotors(double dt) {
    for(auto& [mid, motor] : m_scene->allMotors()){
        if(!motor.enabled) continue;
        Joint* joint = m_scene->getJoint(motor.jointId);
        if(!joint) continue;

        // Per-motor direction multiplier (default +1)
        if(m_motorDirs.find(mid) == m_motorDirs.end())
            m_motorDirs[mid] = 1.0f;

        float step = motor.velocity * m_motorDirs[mid] * float(dt);
        joint->currentValue += step;
        joint->currentValue  = std::clamp(joint->currentValue,
                                           joint->limitMin,
                                           joint->limitMax);

        // Bounce at limits
        if(joint->currentValue >= joint->limitMax ||
           joint->currentValue <= joint->limitMin){
            m_motorDirs[mid] *= -1.0f;
        }
    }
}

void SimulationEngine::resetJoints() {
    for(auto& [jid, _] : m_scene->allJoints()){
        if(Joint* j = m_scene->getJoint(jid))
            j->currentValue = j->initialAngle;
    }
}
