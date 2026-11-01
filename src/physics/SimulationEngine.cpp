#include "SimulationEngine.h"
#include <algorithm>
#include <cmath>
#include <QtMath>
#include <QQuaternion>
#include "../scene/SceneQuery.h"

// ── Constructor / Destructor ─────────────────────────────────────────────────

SimulationEngine::SimulationEngine(SceneGraph* scene, QObject* parent)
    : QObject(parent), m_scene(scene), m_auto(new AutomationEngine(scene, this))
{
    m_timer.setInterval(kTickMs);
    connect(&m_timer, &QTimer::timeout, this, &SimulationEngine::tick);
}

SimulationEngine::~SimulationEngine() {
    teardownMuJoCo();
}

// ── Public control ────────────────────────────────────────────────────────────

bool SimulationEngine::activate() {
    if (m_active) return true;

    QStringList errors;
    if (!m_auto->start(&errors)) {
        emit startFailed(errors);
        return false;
    }

    // Snapshot the edit-time state so Reset can return to it
    m_savedTransforms.clear();
    m_savedJointAngles.clear();
    for (auto& [id, obj] : m_scene->allObjects())
        m_savedTransforms[id] = obj.transform;
    for (auto& [jid, j] : m_scene->allJoints())
        m_savedJointAngles[jid] = j.currentValue;

#ifdef MUJOCO_ENABLED
    // Physics only matters when something is a free/dynamic rigid body
    if (!m_scene->allRigidBodies().empty()) {
        if (!buildMuJoCoModel()) {
            m_auto->stop();
            emit startFailed({"MuJoCo could not build a physics model for this scene (see console output)."});
            return false;
        }
    }
#endif

    m_elapsed = 0.0;
    m_accum   = 0.0;
    m_active  = true;
    m_rec.clear();
    m_viewFrame = -1;
    capture();
    emit activeChanged(true);
    return true;
}

void SimulationEngine::play() {
    if (m_running) return;
    seekLive();
    if (!activate()) return;
    m_running = true;
    m_timer.start();
    emit stateChanged(true);
}

void SimulationEngine::pause() {
    if (!m_running) return;
    m_running = false;
    m_timer.stop();
    emit stateChanged(false);
}

void SimulationEngine::step() {
    pause();
    seekLive();
    if (!activate()) return;
    runScan();
    emit timeUpdated(m_elapsed);
    emit stepped();
}

void SimulationEngine::reset() {
    pause();
    teardownMuJoCo();
    m_auto->stop();
    m_elapsed = 0.0;
    m_accum   = 0.0;

    for (auto& [id, t] : m_savedTransforms) {
        if (SceneObject* obj = m_scene->getObject(id))
            obj->transform = t;
    }
    for (auto& [jid, val] : m_savedJointAngles) {
        if (Joint* j = m_scene->getJoint(jid))
            j->currentValue = val;
    }
    m_savedTransforms.clear();
    m_savedJointAngles.clear();
    m_rec.clear();
    m_viewFrame = -1;
    m_liveTransforms.clear();
    m_liveJoints.clear();

    bool wasActive = m_active;
    m_active = false;
    if (wasActive) emit activeChanged(false);
    emit timeUpdated(0.0);
    emit stepped();
}

void SimulationEngine::setSpeedFactor(double f) {
    m_speed = std::clamp(f, 0.05, 20.0);
}

// ── Simulation tick ───────────────────────────────────────────────────────────

void SimulationEngine::tick() {
    // Scans always advance by the fixed kDt so results don't depend on the
    // playback speed; the speed factor only changes how many run per frame.
    m_accum += m_speed;
    int n = 0;
    while (m_accum >= 1.0 - 1e-9 && n < 64) {
        m_accum -= 1.0;
        runScan();
        ++n;
    }
    if (n > 0) {
        emit timeUpdated(m_elapsed);
        emit stepped();
    }
}

void SimulationEngine::runScan() {
#ifdef MUJOCO_ENABLED
    if (m_mjModel && m_mjData) {
        m_prescribed.clear();
        for (auto& [jid, _] : m_jointQposAddr) {
            double pos, vel;
            if (m_auto->actuatedJointState(jid, pos, vel)) m_prescribed[jid].first = pos;
        }
    }
#endif

    m_auto->scan(kDt);

#ifdef MUJOCO_ENABLED
    if (m_mjModel && m_mjData) {
        for (auto& [jid, fromTo] : m_prescribed) {
            double pos, vel;
            if (m_auto->actuatedJointState(jid, pos, vel)) fromTo.second = pos;
        }
        // Run multiple sub-steps for stability (16ms frame / 2ms MuJoCo dt = 8 steps)
        int nSteps = qMax(1, int(kDt / m_mjModel->opt.timestep + 0.5));
        for (int i = 0; i < nSteps; i++) {
            prescribeActuatedJoints(double(i + 1) / nSteps);
            prescribeHeldParts();
            // Every sub-step: set once per scan, the static belt's friction
            // would tip parts over during the remaining sub-steps
            applyConveyorBelts();
            mj_step(m_mjModel, m_mjData);
        }
        prescribeActuatedJoints(1.0);
        prescribeHeldParts();
        mj_forward(m_mjModel, m_mjData);
        syncFromMuJoCo();
    }
#endif
    m_elapsed += kDt;
    capture();
}

// ── Timeline ─────────────────────────────────────────────────────────────────

void SimulationEngine::seek(int frame) {
    if (!m_active || m_rec.frames() == 0) return;
    pause();
    frame = std::clamp(frame, 0, m_rec.frames() - 1);
    if (m_viewFrame < 0) {
        // Leaving the live state: keep it to come back to (a full recording
        // no longer ends at it)
        m_liveTransforms.clear();
        m_liveJoints.clear();
        for (auto& [id, o] : m_scene->allObjects()) m_liveTransforms[id] = o.transform;
        for (auto& [id, j] : m_scene->allJoints()) m_liveJoints[id] = j.currentValue;
    }
    m_rec.apply(*m_scene, frame);
    m_viewFrame = frame;
    emit timeUpdated(m_rec.time(frame));
    emit scrubbed(m_viewFrame);
    emit stepped();
}

void SimulationEngine::seekLive() {
    if (m_viewFrame < 0) return;
    for (auto& [id, t] : m_liveTransforms)
        if (SceneObject* o = m_scene->getObject(id)) o->transform = t;
    for (auto& [id, v] : m_liveJoints)
        if (Joint* j = m_scene->getJoint(id)) j->currentValue = v;
    m_liveTransforms.clear();
    m_liveJoints.clear();
    m_viewFrame = -1;
    emit timeUpdated(m_elapsed);
    emit scrubbed(-1);
    emit stepped();
}

int SimulationEngine::viewFrame() const {
    return m_viewFrame >= 0 ? m_viewFrame : m_rec.frames() - 1;
}

double SimulationEngine::displayValue(int slot) const {
    return isScrubbing() ? m_rec.value(slot, m_viewFrame) : m_auto->value(slot);
}

// ── MuJoCo backend ────────────────────────────────────────────────────────────

#ifdef MUJOCO_ENABLED

static QVector3D quatToEulerXYZ(float w, float x, float y, float z) {
    // Rotation matrix from quaternion
    float r00 = 1-2*(y*y+z*z), r01 = 2*(x*y-w*z), r02 = 2*(x*z+w*y);
    float r11 = 1-2*(x*x+z*z), r12 = 2*(y*z-w*x);
    float r21 = 2*(y*z+w*x),   r22 = 1-2*(x*x+y*y);

    // Extract XYZ Euler angles: R = Rx(a)*Ry(b)*Rz(c)
    float b = asinf(std::clamp(r02, -1.0f, 1.0f));
    float a, c;
    if (fabsf(cosf(b)) > 1e-6f) {
        a = atan2f(-r12, r22);
        c = atan2f(-r01, r00);
    } else {
        a = atan2f(r21, r11);
        c = 0.0f;
    }
    const float kR2D = 180.0f / float(M_PI);
    return { a*kR2D, b*kR2D, c*kR2D };
}

static QString flt(float v) { return QString::number(double(v), 'f', 6); }

void SimulationEngine::writeGeomXML(QString& xml, const SceneObject* obj,
                                     const RigidBody* rb, const QString& pad) const {
    bool hasColl = rb && rb->hasCollision;
    QString massAttr = rb ? QString(" mass=\"%1\"").arg(flt(rb->mass))
                          : QString(" mass=\"0.001\"");
    // Collision is opt-in: a body only participates in contact physics if it has a
    // RigidBody (dynamic — needs to fall/rest/interact) or a ConveyorBelt (static
    // platform other objects rest on). Everything else — plain structural links,
    // most STEP-imported parts — is excluded from collision entirely. Without this,
    // every body defaults to collidable, and CAD assemblies routinely have parts
    // that touch/overlap at mating faces by design, which reads as interpenetration
    // to MuJoCo and blows up the solver the moment a joint gives them relative motion.
    bool isStaticCollider = !rb && m_scene->getConveyorBelt(obj->id) != nullptr;
    QString noCol = (rb || isStaticCollider)
        ? QString()
        : QString(" contype=\"0\" conaffinity=\"0\"");

    auto pos3 = [](QVector3D v) {
        return QString(" pos=\"%1 %2 %3\"").arg(flt(v.x())).arg(flt(v.y())).arg(flt(v.z()));
    };

    if (hasColl) {
        const auto& c = rb->collision;
        QString posAttr = c.offset.isNull() ? QString() : pos3(c.offset);
        switch (c.type) {
            case CollisionType::Box:
                xml += pad + QString("<geom type=\"box\" size=\"%1 %2 %3\"%4%5/>\n")
                    .arg(flt(c.halfW)).arg(flt(c.halfH)).arg(flt(c.halfD))
                    .arg(posAttr).arg(massAttr);
                break;
            case CollisionType::Cylinder:
                xml += pad + QString("<geom type=\"cylinder\" size=\"%1 %2\" euler=\"-90 0 0\"%3%4/>\n")
                    .arg(flt(c.radius)).arg(flt(c.height * 0.5f))
                    .arg(posAttr).arg(massAttr);
                break;
            case CollisionType::Sphere:
                xml += pad + QString("<geom type=\"sphere\" size=\"%1\"%2%3/>\n")
                    .arg(flt(c.radius)).arg(posAttr).arg(massAttr);
                break;
            case CollisionType::Capsule:
                xml += pad + QString("<geom type=\"capsule\" size=\"%1 %2\" euler=\"-90 0 0\"%3%4/>\n")
                    .arg(flt(c.radius)).arg(flt(c.height * 0.5f))
                    .arg(posAttr).arg(massAttr);
                break;
        }
    } else {
        // Use visual mesh dimensions (no explicit collision shape)
        const auto& s = obj->transform.scale;
        switch (obj->type) {
            case PrimitiveType::Cube:
                xml += pad + QString("<geom type=\"box\" size=\"%1 %2 %3\"%4%5/>\n")
                    .arg(flt(s.x()*0.5f)).arg(flt(s.y()*0.5f)).arg(flt(s.z()*0.5f))
                    .arg(massAttr).arg(noCol);
                break;
            case PrimitiveType::Cylinder:
                // mesh: r=0.5*scale.x, half-h=0.5*scale.y
                xml += pad + QString("<geom type=\"cylinder\" size=\"%1 %2\" euler=\"-90 0 0\"%3%4/>\n")
                    .arg(flt(s.x()*0.5f)).arg(flt(s.y()*0.5f))
                    .arg(massAttr).arg(noCol);
                break;
            case PrimitiveType::Sphere:
                xml += pad + QString("<geom type=\"sphere\" size=\"%1\"%2%3/>\n")
                    .arg(flt(s.x()*0.5f)).arg(massAttr).arg(noCol);
                break;
            case PrimitiveType::Capsule:
                xml += pad + QString("<geom type=\"capsule\" size=\"%1 %2\" euler=\"-90 0 0\"%3%4/>\n")
                    .arg(flt(s.x()*0.25f)).arg(flt(s.y()*0.25f))
                    .arg(massAttr).arg(noCol);
                break;
            case PrimitiveType::Basket: {
                // 5-wall basket: floor + 4 walls, each with 1/5 of the mass
                float m5 = rb ? rb->mass / 5.0f : 0.001f / 5.0f;
                QString ma5 = QString(" mass=\"%1\"").arg(flt(m5));
                float sx = s.x(), sy = s.y(), sz = s.z();
                // Floor
                xml += pad + QString("<geom type=\"box\" size=\"%1 %2 %3\" pos=\"0 %4 0\"%5%6/>\n")
                    .arg(flt(0.5f*sx)).arg(flt(0.025f*sy)).arg(flt(0.5f*sz))
                    .arg(flt(-0.475f*sy)).arg(ma5).arg(noCol);
                // Left wall
                xml += pad + QString("<geom type=\"box\" size=\"%1 %2 %3\" pos=\"%4 %5 0\"%6%7/>\n")
                    .arg(flt(0.025f*sx)).arg(flt(0.475f*sy)).arg(flt(0.5f*sz))
                    .arg(flt(-0.475f*sx)).arg(flt(0.025f*sy)).arg(ma5).arg(noCol);
                // Right wall
                xml += pad + QString("<geom type=\"box\" size=\"%1 %2 %3\" pos=\"%4 %5 0\"%6%7/>\n")
                    .arg(flt(0.025f*sx)).arg(flt(0.475f*sy)).arg(flt(0.5f*sz))
                    .arg(flt(0.475f*sx)).arg(flt(0.025f*sy)).arg(ma5).arg(noCol);
                // Front wall
                xml += pad + QString("<geom type=\"box\" size=\"%1 %2 %3\" pos=\"0 %4 %5\"%6%7/>\n")
                    .arg(flt(0.45f*sx)).arg(flt(0.475f*sy)).arg(flt(0.025f*sz))
                    .arg(flt(0.025f*sy)).arg(flt(0.475f*sz)).arg(ma5).arg(noCol);
                // Back wall
                xml += pad + QString("<geom type=\"box\" size=\"%1 %2 %3\" pos=\"0 %4 %5\"%6%7/>\n")
                    .arg(flt(0.45f*sx)).arg(flt(0.475f*sy)).arg(flt(0.025f*sz))
                    .arg(flt(0.025f*sy)).arg(flt(-0.475f*sz)).arg(ma5).arg(noCol);
                break;
            }
            case PrimitiveType::ImportedMesh: {
                // No convex-hull collision yet — approximate with an AABB box
                QVector3D he = (obj->aabbMax() - obj->aabbMin()) * 0.5f * s;
                QVector3D c2 = (obj->aabbMax() + obj->aabbMin()) * 0.5f * s;
                QString posAttr = c2.isNull() ? QString() : pos3(c2);
                xml += pad + QString("<geom type=\"box\" size=\"%1 %2 %3\"%4%5%6/>\n")
                    .arg(flt(he.x())).arg(flt(he.y())).arg(flt(he.z()))
                    .arg(posAttr).arg(massAttr).arg(noCol);
                break;
            }
        }
    }
}

void SimulationEngine::writeBodyXML(QString& xml, int oid, int indent) const {
    const SceneObject* obj = m_scene->getObject(oid);
    if (!obj) return;

    const RigidBody* rb   = m_scene->getRigidBody(oid);
    const Joint*     jnt  = m_scene->jointForChild(oid);
    const auto&      t    = obj->transform;
    QString          pad(indent * 2, ' ');

    // Gravity compensation for bodies without a gravity-enabled RigidBody
    QString gravComp = (!rb || !rb->gravityEnabled) ? " gravcomp=\"1\"" : "";

    xml += pad + QString("<body name=\"body_%1\" pos=\"%2 %3 %4\"%5>\n")
        .arg(oid).arg(flt(t.position.x())).arg(flt(t.position.y())).arg(flt(t.position.z()))
        .arg(gravComp);

    // Initial body euler rotation (only if non-zero)
    if (t.rotation.x() != 0 || t.rotation.y() != 0 || t.rotation.z() != 0) {
        // Re-open line — instead insert euler into the body tag
        // Replace the last >\n with euler...>\n
        QString eulTag = QString(" euler=\"%1 %2 %3\"")
            .arg(flt(t.rotation.x())).arg(flt(t.rotation.y())).arg(flt(t.rotation.z()));
        xml.replace(xml.lastIndexOf(">\n"), 2, eulTag + ">\n");
    }

    // Joint connecting this body to its parent (lives inside the child body in MJCF)
    if (jnt) {
        QString jType = (jnt->type == JointType::Revolute) ? "hinge" : "slide";

        // Joint::pivot is stored in the parent's local frame; MJCF's <joint
        // pos> is defined in this (child) body's own local frame, so convert.
        QString posAttr;
        if (jnt->type == JointType::Revolute && !jnt->pivot.isNull()) {
            QVector3D pivotLocal = m_scene->localTransform(oid).inverted().map(jnt->pivot);
            posAttr = QString(" pos=\"%1 %2 %3\"")
                .arg(flt(pivotLocal.x())).arg(flt(pivotLocal.y())).arg(flt(pivotLocal.z()));
        }

        // Joint::axis is in the parent's frame too; MJCF's is in the child's
        const QVector3D axis = m_scene->localTransform(oid).inverted().mapVector(jnt->axis).normalized();

        // Continuous revolute joints get no range (autolimits → unlimited)
        QString rangeAttr = (jnt->type == JointType::Revolute && jnt->continuous)
            ? QString()
            : QString(" range=\"%1 %2\"").arg(flt(jnt->limitMin)).arg(flt(jnt->limitMax));
        xml += pad + QString("  <joint name=\"joint_%1\" type=\"%2\" axis=\"%3 %4 %5\"%6%7"
                             " damping=\"1.0\"/>\n")
            .arg(jnt->id).arg(jType)
            .arg(flt(axis.x())).arg(flt(axis.y())).arg(flt(axis.z()))
            .arg(posAttr, rangeAttr);
    } else if (obj->parentId == -1 && rb) {
        // Root body with RigidBody: free joint so it can move dynamically
        xml += pad + QString("  <freejoint name=\"free_%1\"/>\n").arg(oid);
    }

    writeGeomXML(xml, obj, rb, pad + "  ");

    for (int childId : obj->childIds)
        writeBodyXML(xml, childId, indent + 1);

    xml += pad + "</body>\n";
}

QString SimulationEngine::buildMJCF() const {
    QString xml;
    xml += "<mujoco model=\"MotionStudio\">\n";
    xml += "  <compiler autolimits=\"true\"/>\n";
    // Y-up coordinate system: gravity in -Y.
    // implicitfast: handles velocity-dependent forces (joint damping, our
    // velocity actuators) implicitly. Small imported parts get a near-zero
    // placeholder mass/inertia when they have no RigidBody, which makes the
    // damping+actuator system numerically stiff -- explicit Euler diverges to
    // NaN within a handful of steps; implicitfast stays stable without
    // requiring per-part mass/gain tuning.
    xml += "  <option gravity=\"0 -9.81 0\" timestep=\"0.002\" integrator=\"implicitfast\"/>\n";
    xml += "  <worldbody>\n";

    // Floor plane at Y=0 (euler rotates MuJoCo's default Z-normal to +Y)
    if (m_scene->floorEnabled())
        xml += "    <geom name=\"floor\" type=\"plane\" size=\"20 20 0.1\""
               " pos=\"0 0 0\" euler=\"-90 0 0\"/>\n";

    for (int rootId : m_scene->rootObjects())
        writeBodyXML(xml, rootId, 2);

    // Actuated joints are not driven by MuJoCo actuators: the automation
    // engine prescribes their position every sub-step (kinematic drive), so
    // MuJoCo only simulates free rigid bodies and unactuated joints.
    xml += "  </worldbody>\n";
    xml += "</mujoco>\n";
    return xml;
}

bool SimulationEngine::buildMuJoCoModel() {
    teardownMuJoCo();

    QString xml = buildMJCF();
    QByteArray xmlBytes = xml.toUtf8();

    char errBuf[2000] = {};
    mjSpec* spec = mj_parseXMLString(xmlBytes.constData(), nullptr, errBuf, sizeof(errBuf));
    if (!spec) {
        qWarning() << "MuJoCo XML parse error:" << errBuf;
        qWarning() << "MJCF was:\n" << xml;
        return false;
    }

    m_mjModel = mj_compile(spec, nullptr);
    mj_deleteSpec(spec);

    if (!m_mjModel) {
        qWarning() << "MuJoCo compile error";
        return false;
    }

    m_mjData = mj_makeData(m_mjModel);
    buildIdMappings();

    // Start from the joints' current (edit-time) values (MuJoCo starts at ref=0 radians)
    for (auto& [jid, qposadr] : m_jointQposAddr) {
        const Joint* j = m_scene->getJoint(jid);
        if (!j) continue;
        double initVal = (j->type == JointType::Revolute)
            ? j->currentValue * M_PI / 180.0
            : j->currentValue;
        m_mjData->qpos[qposadr] = initVal;
    }

    mj_forward(m_mjModel, m_mjData);
    return true;
}

void SimulationEngine::buildIdMappings() {
    m_jointQposAddr.clear();
    m_bodyMjId.clear();
    m_jointDofAddr.clear();
    m_freeJointQposAddr.clear();
    m_freeJointDofAddr.clear();

    for (auto& [jid, _] : m_scene->allJoints()) {
        QString name = QString("joint_%1").arg(jid);
        int mjId = mj_name2id(m_mjModel, mjOBJ_JOINT, name.toStdString().c_str());
        if (mjId >= 0) {
            m_jointQposAddr[jid] = m_mjModel->jnt_qposadr[mjId];
            m_jointDofAddr[jid]  = m_mjModel->jnt_dofadr[mjId];
        }
    }

    for (auto& [oid, _] : m_scene->allObjects()) {
        QString name = QString("body_%1").arg(oid);
        int mjId = mj_name2id(m_mjModel, mjOBJ_BODY, name.toStdString().c_str());
        if (mjId >= 0)
            m_bodyMjId[oid] = mjId;

        // Free joint for root dynamic bodies
        QString fname = QString("free_%1").arg(oid);
        int fjId = mj_name2id(m_mjModel, mjOBJ_JOINT, fname.toStdString().c_str());
        if (fjId >= 0) {
            m_freeJointQposAddr[oid] = m_mjModel->jnt_qposadr[fjId];
            m_freeJointDofAddr[oid]  = m_mjModel->jnt_dofadr[fjId];
        }
    }
}

void SimulationEngine::applyConveyorBelts() {
    for (auto& [beltId, belt] : m_scene->allConveyorBelts()) {
        if (!belt.enabled) continue;
        const SceneObject* beltObj = m_scene->getObject(beltId);
        if (!beltObj) continue;
        const double speed = m_auto->beltSpeed(beltId, belt.velocity);

        // Belt top surface in world space
        QMatrix4x4 bw = m_scene->worldTransform(beltId);
        QVector3D beltPos = QVector3D(bw.column(3));
        float beltTopY   = beltPos.y() + beltObj->transform.scale.y() * 0.5f;
        float beltHalfX  = beltObj->transform.scale.x() * 0.5f + 0.1f;
        float beltHalfZ  = beltObj->transform.scale.z() * 0.5f + 0.1f;

        // World-space push direction
        QVector3D worldDir = (bw * QVector4D(belt.direction, 0.0f)).toVector3D().normalized();

        for (auto& [oid, dofAdr] : m_freeJointDofAddr) {
            const SceneObject* obj = m_scene->getObject(oid);
            if (!obj || m_auto->holderOf(oid) != -1) continue;

            float bodyBottomY = obj->transform.position.y() - obj->transform.scale.y() * 0.5f;
            float dy = bodyBottomY - beltTopY;

            // Body must be resting on (or just above) the belt surface
            if (dy > -0.08f && dy < 0.25f) {
                float dx = fabsf(obj->transform.position.x() - beltPos.x());
                float dz = fabsf(obj->transform.position.z() - beltPos.z());
                if (dx < beltHalfX && dz < beltHalfZ) {
                    // Override linear velocity along belt direction
                    m_mjData->qvel[dofAdr + 0] = double(worldDir.x()) * speed;
                    m_mjData->qvel[dofAdr + 2] = double(worldDir.z()) * speed;
                    // Zero angular velocity so the object slides, not rolls
                    m_mjData->qvel[dofAdr + 3] = 0.0;
                    m_mjData->qvel[dofAdr + 4] = 0.0;
                    m_mjData->qvel[dofAdr + 5] = 0.0;
                }
            }
        }
    }
}

void SimulationEngine::prescribeActuatedJoints(double alpha) {
    // Kinematic drive: actuated joints follow the automation engine's motion
    // profile exactly, interpolated across the sub-steps of one scan.
    const double inv = 1.0 / kDt;
    for (auto& [jid, fromTo] : m_prescribed) {
        const Joint* j = m_scene->getJoint(jid);
        auto qa = m_jointQposAddr.find(jid);
        auto da = m_jointDofAddr.find(jid);
        if (!j || qa == m_jointQposAddr.end() || da == m_jointDofAddr.end()) continue;
        double k   = (j->type == JointType::Revolute) ? M_PI / 180.0 : 1.0;
        double pos = fromTo.first + (fromTo.second - fromTo.first) * alpha;
        m_mjData->qpos[qa->second] = pos * k;
        m_mjData->qvel[da->second] = (fromTo.second - fromTo.first) * inv * k;
    }
}

void SimulationEngine::prescribeHeldParts() {
    // A gripped free body goes where the automation engine put it (with its
    // gripper) and carries no velocity, so it drops straight when released
    for (int oid : m_auto->heldObjects()) {
        auto qa = m_freeJointQposAddr.find(oid);
        auto da = m_freeJointDofAddr.find(oid);
        const SceneObject* obj = m_scene->getObject(oid);
        if (!obj || qa == m_freeJointQposAddr.end() || da == m_freeJointDofAddr.end()) continue;
        const QMatrix4x4 w = m_scene->worldTransform(oid);
        const QQuaternion q = QQuaternion::fromRotationMatrix(
            scenequery::stripScale(w).toGenericMatrix<3, 3>()).normalized();
        mjtNum* qpos = m_mjData->qpos + qa->second;
        qpos[0] = w(0, 3); qpos[1] = w(1, 3); qpos[2] = w(2, 3);
        qpos[3] = q.scalar(); qpos[4] = q.x(); qpos[5] = q.y(); qpos[6] = q.z();
        for (int k = 0; k < 6; ++k) m_mjData->qvel[da->second + k] = 0;
    }
}

void SimulationEngine::syncFromMuJoCo() {
    // Sync regular joint angles from qpos
    for (auto& [jid, qposadr] : m_jointQposAddr) {
        Joint* j = m_scene->getJoint(jid);
        if (!j || m_prescribed.count(jid)) continue;   // actuated: already set by automation
        double raw = m_mjData->qpos[qposadr];
        j->currentValue = (j->type == JointType::Revolute)
            ? float(raw * 180.0 / M_PI)
            : float(raw);
    }

    // Sync free-body world positions and orientations
    for (auto& [oid, mjBId] : m_bodyMjId) {
        SceneObject* obj = m_scene->getObject(oid);
        if (!obj) continue;
        if (obj->parentId != -1) continue;          // only root objects
        if (!m_freeJointQposAddr.count(oid)) continue; // only freejoint bodies

        const mjtNum* xp = m_mjData->xpos  + mjBId * 3;
        const mjtNum* xq = m_mjData->xquat + mjBId * 4;

        obj->transform.position = { float(xp[0]), float(xp[1]), float(xp[2]) };
        obj->transform.rotation = quatToEulerXYZ(
            float(xq[0]), float(xq[1]), float(xq[2]), float(xq[3]));
    }
}

void SimulationEngine::teardownMuJoCo() {
    if (m_mjData)  { mj_deleteData(m_mjData);   m_mjData  = nullptr; }
    if (m_mjModel) { mj_deleteModel(m_mjModel); m_mjModel = nullptr; }
    m_jointQposAddr.clear();
    m_bodyMjId.clear();
    m_jointDofAddr.clear();
    m_prescribed.clear();
    m_freeJointQposAddr.clear();
    m_freeJointDofAddr.clear();
}

#endif // MUJOCO_ENABLED
