#include "AutomationEngine.h"
#include "../scene/SceneGraph.h"
#include "Sensors.h"
#include "Kinematics.h"
#include "../io/ModbusServer.h"
#include "../scene/PrimitiveMesh.h"
#include <QSet>
#include <QtMath>
#include <algorithm>
#include <cmath>
#include <functional>

namespace {

struct FieldInfo { const char* suffix; bool isBool; const char* help; };

const FieldInfo kActuatorFields[] = {
    {"pos",        false, "current position (deg or m)"},
    {"vel",        false, "current velocity (deg/s or m/s)"},
    {"target",     false, "commanded target position"},
    {"inPosition", true,  "at its target and stopped (Position mode)"},
    {"moving",     true,  "velocity is non-zero"},
    {"atMin",      true,  "at the lower joint limit"},
    {"atMax",      true,  "at the upper joint limit"},
    {"active",     true,  "actuator is active"},
};
constexpr int kActFieldCount = int(sizeof(kActuatorFields) / sizeof(kActuatorFields[0]));
enum ActField { AF_Pos, AF_Vel, AF_Target, AF_InPos, AF_Moving, AF_AtMin, AF_AtMax, AF_Active };

const FieldInfo kSequenceFields[] = {
    {"running", true,  "a cycle is in progress"},
    {"done",    true,  "the last cycle has completed and no new one has started"},
    {"cycles",  false, "number of completed cycles"},
};
constexpr int kSeqFieldCount = int(sizeof(kSequenceFields) / sizeof(kSequenceFields[0]));
enum SeqField { SF_Running, SF_Done, SF_Cycles };

const FieldInfo kSourceFields[] = {
    {"count",   false, "copies made so far"},
    {"blocked", true,  "a copy is due but a part still occupies the spot"},
};
enum SourceField { SrcF_Count, SrcF_Blocked };

const char* const kPositionHelp[6] = {
    "detected part's X in the sensor's frame (m)", "detected part's Y in the sensor's frame (m)",
    "detected part's Z in the sensor's frame (m)", "detected part's rotation about X (deg)",
    "detected part's rotation about Y (deg)",      "detected part's rotation about Z (deg)",
};

constexpr double kPosTol = 1e-4;

QString opLabel(const Operation& op, int index) {
    return op.name.isEmpty() ? QString("#%1").arg(index + 1) : op.name;
}

} // namespace

// ── Symbols ──────────────────────────────────────────────────────────────────

std::vector<AutomationEngine::Symbol> AutomationEngine::buildSymbols(const SceneGraph& scene) {
    std::vector<Symbol> out;
    for (auto& [id, s] : scene.allSignals()) {
        Symbol sym;
        sym.name     = s.name;
        sym.kind     = SymbolKind::Signal;
        sym.ownerId  = id;
        sym.writable = s.kind == SignalKind::Input || s.kind == SignalKind::Internal;
        sym.type     = s.type;
        sym.isBool   = s.type == SignalType::Bool;
        sym.help     = signalKindName(s.kind) + " " + signalTypeName(s.type) +
                       (s.description.isEmpty() ? QString() : " — " + s.description);
        out.push_back(sym);
        // A position sensor's coordinates follow its signal, in this order
        if (s.kind == SignalKind::Sensor && s.sensor.type == SensorType::Position) {
            for (int i = 0; i < sensors::kPositionFieldCount; ++i) {
                Symbol f;
                f.name    = s.name + "." + sensors::kPositionFields[i];
                f.kind    = SymbolKind::Signal;
                f.ownerId = id;
                f.type    = SignalType::Real;
                f.help    = QString("Sensor %1: %2").arg(s.name, kPositionHelp[i]);
                out.push_back(f);
            }
        }
    }
    for (int mid : scene.motorIdsSorted()) {
        const Motor* m = scene.getMotor(mid);
        for (const auto& f : kActuatorFields) {
            Symbol sym;
            sym.name    = m->name + "." + f.suffix;
            sym.kind    = SymbolKind::Actuator;
            sym.ownerId = mid;
            sym.isBool  = f.isBool;
            sym.type    = f.isBool ? SignalType::Bool : SignalType::Real;
            sym.help    = QString("Actuator %1: %2").arg(m->name, f.help);
            out.push_back(sym);
        }
    }
    for (auto& [id, q] : scene.allSequences()) {
        for (const auto& f : kSequenceFields) {
            Symbol sym;
            sym.name    = q.name + "." + f.suffix;
            sym.kind    = SymbolKind::Sequence;
            sym.ownerId = id;
            sym.isBool  = f.isBool;
            sym.type    = f.isBool ? SignalType::Bool : SignalType::Int;
            sym.help    = QString("Sequence %1: %2").arg(q.name, f.help);
            out.push_back(sym);
        }
    }
    for (auto& [id, src] : scene.allSources()) {
        for (const auto& f : kSourceFields) {
            Symbol sym;
            sym.name    = src.name + "." + f.suffix;
            sym.kind    = SymbolKind::Flow;
            sym.ownerId = id;
            sym.isBool  = f.isBool;
            sym.type    = f.isBool ? SignalType::Bool : SignalType::Int;
            sym.help    = QString("Source %1: %2").arg(src.name, f.help);
            out.push_back(sym);
        }
    }
    for (auto& [id, sink] : scene.allSinks()) {
        Symbol sym;
        sym.name    = sink.name + ".count";
        sym.kind    = SymbolKind::Flow;
        sym.ownerId = id;
        sym.type    = SignalType::Int;
        sym.help    = QString("Sink %1: parts taken so far").arg(sink.name);
        out.push_back(sym);
    }
    return out;
}

QStringList AutomationEngine::symbolNames(const SceneGraph& scene) {
    QStringList out;
    for (const auto& s : buildSymbols(scene)) out << s.name;
    return out;
}

namespace {
struct Resolvers {
    QHash<QString, int> read, write;
    expr::Resolver r() const { return [this](const QString& n) { return read.value(n, -1); }; }
    expr::Resolver w() const { return [this](const QString& n) { return write.value(n, -1); }; }
};
Resolvers makeResolvers(const std::vector<AutomationEngine::Symbol>& syms) {
    Resolvers res;
    for (int i = 0; i < int(syms.size()); ++i) {
        res.read.insert(syms[size_t(i)].name, i);
        if (syms[size_t(i)].writable) res.write.insert(syms[size_t(i)].name, i);
    }
    return res;
}
} // namespace

QString AutomationEngine::validateExpression(const SceneGraph& scene, const QString& source) {
    auto res = makeResolvers(buildSymbols(scene));
    expr::Expression e;
    QString err;
    if (!e.compile(source, res.r(), &err)) return err;
    return {};
}

QString AutomationEngine::validateAssignments(const SceneGraph& scene, const QString& source) {
    auto res = makeResolvers(buildSymbols(scene));
    std::vector<expr::Assignment> a;
    QString err;
    if (!expr::compileAssignments(source, res.r(), res.w(), a, &err)) return err;
    return {};
}

QStringList AutomationEngine::validateAll(const SceneGraph& scene) {
    QStringList errs;
    auto syms = buildSymbols(scene);
    auto res  = makeResolvers(syms);

    auto chk = [&](const QString& where, const QString& src) {
        expr::Expression e;
        QString err;
        if (!e.compile(src, res.r(), &err)) errs << where + ": " + err;
    };
    auto chkA = [&](const QString& where, const QString& src) {
        std::vector<expr::Assignment> a;
        QString err;
        if (!expr::compileAssignments(src, res.r(), res.w(), a, &err)) errs << where + ": " + err;
    };

    for (auto& [id, s] : scene.allSignals()) {
        if (s.kind == SignalKind::Derived) {
            if (s.expression.trimmed().isEmpty()) errs << "Signal " + s.name + ": derived signal has no expression";
            else chk("Signal " + s.name, s.expression);
        }
        if (s.kind == SignalKind::Sensor) {
            QString p = sensors::problem(scene, s.sensor);
            if (!p.isEmpty()) errs << "Sensor " + s.name + ": " + p;
        }
    }
    for (int mid : scene.motorIdsSorted()) {
        const Motor* m = scene.getMotor(mid);
        if (!scene.getJoint(m->jointId)) errs << "Actuator " + m->name + ": joint no longer exists";
        chk("Actuator " + m->name + " target",   m->targetExpr);
        chk("Actuator " + m->name + " velocity", m->velocityExpr);
        chk("Actuator " + m->name + " active",   m->activeExpr);
    }
    for (auto& [qid, q] : scene.allSequences()) {
        chk("Sequence " + q.name + " enable", q.enableCondition);
        chk("Sequence " + q.name + " start",  q.startCondition);
        QSet<int> opIds;
        for (auto& op : q.operations) opIds.insert(op.id);
        for (int i = 0; i < int(q.operations.size()); ++i) {
            const Operation& op = q.operations[size_t(i)];
            QString where = QString("%1 › %2").arg(q.name, opLabel(op, i));
            switch (op.type) {
                case OperationType::Move:
                    if (!scene.getMotor(op.actuatorId)) errs << where + ": no actuator selected";
                    if (op.target.trimmed().isEmpty()) errs << where + ": no target";
                    chk(where + " target", op.target);
                    chk(where + " velocity", op.velocity);
                    break;
                case OperationType::Run:
                    if (!scene.getMotor(op.actuatorId)) errs << where + ": no actuator selected";
                    if (op.velocity.trimmed().isEmpty()) errs << where + ": no speed";
                    chk(where + " speed", op.velocity);
                    break;
                case OperationType::Stop:
                    if (!scene.getMotor(op.actuatorId)) errs << where + ": no actuator selected";
                    break;
                case OperationType::MoveToPose:
                    if (!scene.getPose(op.poseId)) errs << where + ": no pose selected";
                    chk(where + " speed %", op.velocity);
                    break;
                case OperationType::Wait:
                    if (op.duration.trimmed().isEmpty()) errs << where + ": no duration";
                    chk(where + " duration", op.duration);
                    break;
                case OperationType::WaitFor:
                    if (op.condition.trimmed().isEmpty()) errs << where + ": no condition";
                    chk(where + " condition", op.condition);
                    break;
                case OperationType::Set:
                    break;
                case OperationType::Grip: {
                    const SceneObject* gripper = scene.getObject(op.gripperId);
                    if (!gripper) errs << where + (op.gripperId == -1 ? ": no gripper selected" : ": the gripper no longer exists");
                    if (op.sensorId != -1) {
                        auto it = scene.allSignals().find(op.sensorId);
                        if (it == scene.allSignals().end() || it->second.kind != SignalKind::Sensor)
                            errs << where + ": the sensor no longer exists";
                    } else if (!scene.getObject(op.partId)) {
                        errs << where + (op.partId == -1 ? ": no part selected" : ": the part no longer exists");
                    } else if (gripper && scenequery::inSubtree(scene, op.gripperId, op.partId)) {
                        errs << where + ": the part is the gripper or holds it";
                    }
                    break;
                }
                case OperationType::Release:
                    if (!scene.getObject(op.gripperId))
                        errs << where + (op.gripperId == -1 ? ": no gripper selected" : ": the gripper no longer exists");
                    break;
                case OperationType::MoveL: {
                    const Pose* pose = scene.getPose(op.poseId);
                    // To a pose taught on a part, the tool may be left to the pose
                    const bool poseTool = op.toolId == -1 && pose && pose->isRelative();
                    if (!poseTool) {
                        if (!scene.getObject(op.toolId))
                            errs << where + (op.toolId == -1 ? ": no tool selected" : ": the tool no longer exists");
                        else if (kin::chainTo(scene, op.toolId).empty())
                            errs << where + ": no actuated joint moves the tool";
                        else if (pose && pose->isRelative() && op.toolId != pose->toolId) {
                            const SceneObject* t = scene.getObject(pose->toolId);
                            errs << where + QString(": pose %1 was taught with the tool %2; use that tool")
                                                .arg(pose->name, t ? t->name : QString("(deleted)"));
                        }
                    }
                    if (!pose) errs << where + ": no pose selected";
                    chk(where + " speed", op.velocity);
                    break;
                }
                case OperationType::Walk: {
                    QString why;
                    gait::Legs legs;
                    // findLegs straightens the legs for a moment and puts them back
                    if (!scene.getObject(op.walkerId))
                        errs << where + (op.walkerId == -1 ? ": no robot selected" : ": the robot no longer exists");
                    else if (!gait::findLegs(const_cast<SceneGraph&>(scene), op.walkerId, &legs, &why))
                        errs << where + ": " + why;
                    if (op.walkToId != -1 && !scene.getObject(op.walkToId))
                        errs << where + ": the place to walk to no longer exists";
                    else if (op.walkToId == -1 && op.forward.trimmed().isEmpty() && op.sideways.trimmed().isEmpty() &&
                             op.turn.trimmed().isEmpty())
                        errs << where + ": nothing to walk: give a distance, a turn or a place";
                    chk(where + " forward", op.forward);
                    chk(where + " sideways", op.sideways);
                    chk(where + " turn", op.turn);
                    chk(where + " speed", op.velocity);
                    break;
                }
            }
            // The part a pose taught on a part is applied to
            if (const Pose* pose = scene.getPose(op.poseId);
                pose && pose->isRelative() &&
                (op.type == OperationType::MoveL || op.type == OperationType::MoveToPose)) {
                if (op.sensorId != -1) {
                    auto it = scene.allSignals().find(op.sensorId);
                    if (it == scene.allSignals().end() || it->second.kind != SignalKind::Sensor)
                        errs << where + ": the sensor that finds the part no longer exists";
                } else if (op.partId != -1 && !scene.getObject(op.partId)) {
                    errs << where + ": the part no longer exists";
                }
            }
            chk(where + " start condition", op.startCondition);
            chkA(where + " on start", op.onStart);
            chkA(where + " on finish", op.onFinish);
            for (int p : op.predecessors)
                if (!opIds.contains(p)) errs << where + ": predecessor no longer exists";
        }
    }

    // Poses taught on a part
    for (auto& [pid, p] : scene.allPoses()) {
        if (!p.isRelative()) continue;
        if (!scene.getObject(p.frameId)) errs << "Pose " + p.name + ": the part it was taught on no longer exists";
        if (!scene.getObject(p.toolId)) errs << "Pose " + p.name + ": its tool no longer exists";
        else if (kin::chainTo(scene, p.toolId).empty()) errs << "Pose " + p.name + ": no actuated joint moves its tool";
    }

    // Part flow
    for (auto& [id, src] : scene.allSources()) {
        const QString where = "Source " + src.name;
        const SceneObject* t = scene.getObject(src.templateId);
        if (!t) errs << where + (src.templateId == -1 ? ": no object to copy selected" : ": the object it copies no longer exists");
        else if (t->type == PrimitiveType::ImportedMesh && !t->mesh && t->childIds.empty())
            errs << where + ": the object it copies has no geometry";
        if (src.mode == SourceMode::Interval) {
            if (src.interval.trimmed().isEmpty()) errs << where + ": no interval";
            chk(where + " interval", src.interval);
        } else {
            if (src.trigger.trimmed().isEmpty()) errs << where + ": no trigger condition";
            chk(where + " trigger", src.trigger);
        }
        chk(where + " enable", src.enable);
    }
    for (auto& [id, sink] : scene.allSinks()) {
        const QString where = "Sink " + sink.name;
        const QString p = sensors::problem(scene, sink.zone);
        if (!p.isEmpty()) errs << where + ": " + p;
        if (sink.sourceId != -1 && !scene.getSource(sink.sourceId)) errs << where + ": the source it takes from no longer exists";
        chk(where + " enable", sink.enable);
    }

    errs << ModbusServer::validate(scene);

    // Couplings: the master exists, the follower has no actuator, no loops
    for (auto& [jid, j] : scene.allJoints()) {
        if (j.coupledTo == -1) continue;
        const QString name = j.name.isEmpty() ? QString("Joint %1").arg(jid) : j.name;
        if (!scene.getJoint(j.coupledTo)) { errs << "Joint " + name + ": the joint it follows no longer exists"; continue; }
        if (const Motor* m = scene.motorForJoint(jid))
            errs << "Joint " + name + ": it follows another joint, so its actuator " + m->name + " can't drive it";
        int steps = 0;
        for (int at = j.coupledTo; at != -1 && steps <= int(scene.allJoints().size()); ++steps) {
            if (at == jid) { errs << "Joint " + name + ": couplings form a loop"; break; }
            const Joint* next = scene.getJoint(at);
            at = next ? next->coupledTo : -1;
        }
    }

    for (auto& [oid, belt] : scene.allConveyorBelts()) {
        const SceneObject* o = scene.getObject(oid);
        chk("Conveyor " + (o ? o->name : QString::number(oid)) + " speed", belt.speedExpr);
    }

    // Cycles among derived signals
    std::map<int, std::vector<int>> deps;   // slot → derived slots it depends on
    QSet<int> derivedSlots;
    for (auto& [id, s] : scene.allSignals())
        if (s.kind == SignalKind::Derived) derivedSlots.insert(res.read.value(s.name, -1));
    for (auto& [id, s] : scene.allSignals()) {
        if (s.kind != SignalKind::Derived) continue;
        expr::Expression e;
        if (!e.compile(s.expression, res.r())) continue;
        int slot = res.read.value(s.name);
        for (int d : e.dependencies())
            if (derivedSlots.contains(d)) deps[slot].push_back(d);
    }
    std::map<int, int> mark;   // 0 = unvisited, 1 = visiting, 2 = done
    std::function<bool(int)> visit = [&](int n) -> bool {
        if (mark[n] == 1) return false;
        if (mark[n] == 2) return true;
        mark[n] = 1;
        for (int d : deps[n]) if (!visit(d)) return false;
        mark[n] = 2;
        return true;
    };
    for (int s : derivedSlots) {
        if (!visit(s)) {
            errs << "Signal " + syms[size_t(s)].name + ": derived signals depend on each other in a loop";
            break;
        }
    }
    return errs;
}

// ── Lifecycle ────────────────────────────────────────────────────────────────

AutomationEngine::AutomationEngine(SceneGraph* scene, QObject* parent)
    : QObject(parent), m_scene(scene) {}

bool AutomationEngine::start(QStringList* errors) {
    stop();
    QStringList errs = validateAll(*m_scene);
    if (!errs.isEmpty()) {
        if (errors) *errors = errs;
        return false;
    }

    m_symbols = buildSymbols(*m_scene);
    auto res  = makeResolvers(m_symbols);
    m_slotIndex = res.read;
    m_cur.assign(m_symbols.size(), 0.0);
    m_slotType.resize(m_symbols.size());
    for (size_t i = 0; i < m_symbols.size(); ++i) m_slotType[i] = m_symbols[i].type;

    // Signals
    for (auto& [id, s] : m_scene->allSignals()) {
        int slot = res.read.value(s.name);
        m_cur[size_t(slot)] = coerceSignal(s.type, s.initialValue);
        if (s.kind == SignalKind::Sensor)
            m_sensors.push_back({id, slot, s.sensor, -1});
        if (s.kind == SignalKind::Derived) {
            DerivedRt d;
            d.slot = slot;
            d.type = s.type;
            d.ex.compile(s.expression, res.r());
            m_derived.push_back(std::move(d));
        }
    }
    // Order derived signals so each is evaluated after the derived signals it reads
    {
        std::vector<DerivedRt> ordered;
        std::vector<bool> placed(m_derived.size(), false);
        QSet<int> done;
        QSet<int> derivedSlots;
        for (auto& d : m_derived) derivedSlots.insert(d.slot);
        while (ordered.size() < m_derived.size()) {
            bool progress = false;
            for (size_t i = 0; i < m_derived.size(); ++i) {
                if (placed[i]) continue;
                bool ready = true;
                for (int dep : m_derived[i].ex.dependencies())
                    if (derivedSlots.contains(dep) && !done.contains(dep) && dep != m_derived[i].slot) ready = false;
                if (!ready) continue;
                placed[i] = true;
                done.insert(m_derived[i].slot);
                ordered.push_back(std::move(m_derived[i]));
                progress = true;
            }
            if (!progress) break;   // validated acyclic above; defensive
        }
        m_derived = std::move(ordered);
    }

    // Actuators
    for (int mid : m_scene->motorIdsSorted()) {
        const Motor* m = m_scene->getMotor(mid);
        const Joint* j = m_scene->getJoint(m->jointId);
        if (!j) continue;
        ActuatorRt a;
        a.motorId     = mid;
        a.jointId     = j->id;
        a.name        = m->name;
        a.limited     = !(j->type == JointType::Revolute && j->continuous);
        a.limMin      = std::min(j->limitMin, j->limitMax);
        a.limMax      = std::max(j->limitMin, j->limitMax);
        a.defVelocity = m->velocity;
        a.defAccel    = std::max(0.0f, m->acceleration);
        a.mode        = m->mode;
        a.velocity    = m->velocity;
        a.accel       = a.defAccel;
        a.active      = m->enabled;
        a.st.pos      = j->currentValue;
        a.st.vel      = 0;
        a.target      = clampToLimits(a, j->currentValue);
        a.bindTarget.compile(m->targetExpr, res.r());
        a.bindVelocity.compile(m->velocityExpr, res.r());
        a.bindActive.compile(m->activeExpr, res.r());
        a.slotBase    = res.read.value(m->name + "." + kActuatorFields[0].suffix);
        m_actIndexByMotor[mid] = int(m_actuators.size());
        m_actuators.push_back(std::move(a));
    }

    // Conveyor belt speed bindings
    for (auto& [oid, belt] : m_scene->allConveyorBelts()) {
        if (belt.speedExpr.trimmed().isEmpty()) continue;
        BeltRt b;
        b.objectId = oid;
        b.speed.compile(belt.speedExpr, res.r());
        b.value = belt.velocity;
        m_belts.push_back(std::move(b));
    }

    // Sequences
    for (auto& [qid, q] : m_scene->allSequences()) {
        SeqRt s;
        s.id   = qid;
        s.name = q.name;
        s.loop = q.loop;
        s.enable.compile(q.enableCondition, res.r());
        s.startCond.compile(q.startCondition, res.r());
        s.slotBase = res.read.value(q.name + "." + kSequenceFields[0].suffix);
        std::map<int, int> idxById;
        for (int i = 0; i < int(q.operations.size()); ++i) idxById[q.operations[size_t(i)].id] = i;
        for (auto& op : q.operations) {
            OpRt o;
            o.def = op;
            o.target.compile(op.target, res.r());
            o.velocity.compile(op.velocity, res.r());
            o.duration.compile(op.duration, res.r());
            o.condition.compile(op.condition, res.r());
            o.startCond.compile(op.startCondition, res.r());
            o.forward.compile(op.forward, res.r());
            o.sideways.compile(op.sideways, res.r());
            o.turn.compile(op.turn, res.r());
            expr::compileAssignments(op.onStart, res.r(), res.w(), o.onStart);
            expr::compileAssignments(op.onFinish, res.r(), res.w(), o.onFinish);
            for (int p : op.predecessors) {
                auto it = idxById.find(p);
                if (it != idxById.end()) o.predIdx.push_back(it->second);
            }
            auto ai = m_actIndexByMotor.find(op.actuatorId);
            o.actIdx = ai != m_actIndexByMotor.end() ? ai->second : -1;
            s.ops.push_back(std::move(o));
        }
        m_sequences.push_back(std::move(s));
    }

    // Part flow. A source's template is out of the run: its copies appear in its place.
    for (auto& [id, src] : m_scene->allSources()) {
        SourceRt r;
        r.def = src;
        r.interval.compile(src.interval, res.r());
        r.trigger.compile(src.trigger, res.r());
        r.enable.compile(src.enable, res.r());
        r.slotBase = res.read.value(src.name + "." + kSourceFields[0].suffix);
        m_sources.push_back(std::move(r));
        m_scene->setParked(src.templateId, true);
    }
    for (auto& [id, sink] : m_scene->allSinks()) {
        SinkRt r;
        r.def = sink;
        r.enable.compile(sink.enable, res.r());
        r.slot = res.read.value(sink.name + ".count");
        m_sinks.push_back(std::move(r));
    }

    m_time = 0;
    m_scanCount = 0;
    m_warnings.clear();
    m_structureChanged = false;
    m_scene->applyCouplings();
    publishStatus();
    m_prev = m_cur;
    m_active = true;
    return true;
}

void AutomationEngine::stop() {
    cancelMotions({}, "the simulation was reset", "reset");
    m_active = false;
    m_symbols.clear();
    m_slotIndex.clear();
    m_cur.clear();
    m_prev.clear();
    m_slotType.clear();
    m_derived.clear();
    m_sensors.clear();
    m_holds.clear();
    m_belts.clear();
    m_actuators.clear();
    m_sequences.clear();
    m_actIndexByMotor.clear();
    m_sources.clear();
    m_sinks.clear();
    m_structureChanged = false;
    // Copies go, parked templates and parts come back
    m_scene->clearRuntimeObjects();
    m_pendingWrites.clear();
    m_inputQueue.clear();
    m_time = 0;
    m_scanCount = 0;
}

void AutomationEngine::setInput(const QString& name, double v) {
    int slot = slotOf(name);
    if (slot < 0 || !m_symbols[size_t(slot)].writable) return;
    m_inputQueue[slot].push_back(coerceSignal(m_slotType[size_t(slot)], v));
}

double AutomationEngine::displayValue(const QString& name) const {
    int slot = slotOf(name);
    auto it = m_inputQueue.find(slot);
    if (it != m_inputQueue.end() && !it->second.empty()) return it->second.back();
    return value(slot);
}

// ── Scan ─────────────────────────────────────────────────────────────────────

expr::EvalContext AutomationEngine::ctx() const {
    expr::EvalContext c;
    c.cur  = m_cur.data();
    c.prev = m_prev.data();
    c.time = m_time;
    c.dt   = m_dt;
    return c;
}

void AutomationEngine::scan(double dt) {
    if (!m_active) return;
    m_dt = dt;
    m_prev = m_cur;

    // 0. Action writes from the previous scan, then one queued input per signal
    for (auto& [slot, v] : m_pendingWrites)
        m_cur[size_t(slot)] = coerceSignal(m_slotType[size_t(slot)], v);
    m_pendingWrites.clear();
    for (auto it = m_inputQueue.begin(); it != m_inputQueue.end();) {
        m_cur[size_t(it->first)] = it->second.front();
        it->second.pop_front();
        it = it->second.empty() ? m_inputQueue.erase(it) : std::next(it);
    }

    // 1. Status signals
    publishStatus();

    // 2. Derived signals
    for (auto& d : m_derived)
        m_cur[size_t(d.slot)] = coerceSignal(d.type, d.ex.eval(ctx()));

    // 3. Sequences, then motions commanded from outside
    for (auto& s : m_sequences) advanceSequence(s);
    advanceMotions();

    // 4. Bindings
    applyBindings();

    // 5. Motion, then gripped parts follow their grippers
    integrate(dt);
    applyHolds();

    // 6. Part flow
    updateFlow();

    m_time += dt;
    ++m_scanCount;
    emit scanned();
}

void AutomationEngine::publishStatus() {
    for (auto& sn : m_sensors) {
        const sensors::Reading r = sensors::read(*m_scene, sn.cfg);
        m_cur[size_t(sn.slot)] = r.value(sn.cfg);
        sn.detected = r.objectId;
        if (sn.cfg.type == SensorType::Position)
            sensors::partPose(*m_scene, sn.cfg, r.objectId, m_cur.data() + sn.slot + 1);
    }
    for (auto& src : m_sources) {
        double* v = m_cur.data() + src.slotBase;
        v[SrcF_Count]   = src.count;
        v[SrcF_Blocked] = src.blocked ? 1 : 0;
    }
    for (auto& sink : m_sinks) m_cur[size_t(sink.slot)] = sink.count;
    for (auto& a : m_actuators) {
        double* v = m_cur.data() + a.slotBase;
        v[AF_Pos]    = a.st.pos;
        v[AF_Vel]    = a.st.vel;
        v[AF_Target] = a.mode == ActuatorMode::Position ? a.target : a.st.pos;
        v[AF_InPos]  = (a.mode == ActuatorMode::Position && inPosition(a, a.target)) ? 1 : 0;
        v[AF_Moving] = std::fabs(a.st.vel) > 1e-9 ? 1 : 0;
        v[AF_AtMin]  = (a.limited && a.st.pos <= a.limMin + kPosTol) ? 1 : 0;
        v[AF_AtMax]  = (a.limited && a.st.pos >= a.limMax - kPosTol) ? 1 : 0;
        v[AF_Active] = a.active ? 1 : 0;
    }
    for (auto& s : m_sequences) {
        double* v = m_cur.data() + s.slotBase;
        v[SF_Running] = s.status.state == SeqState::Running ? 1 : 0;
        v[SF_Done]    = s.status.state != SeqState::Running && s.status.cycles > 0 ? 1 : 0;
        v[SF_Cycles]  = s.status.cycles;
    }
}

void AutomationEngine::queueWrite(const std::vector<expr::Assignment>& actions) {
    auto c = ctx();
    for (const auto& a : actions)
        m_pendingWrites.emplace_back(a.slot, a.value.eval(c));
}

double AutomationEngine::clampToLimits(const ActuatorRt& a, double v) const {
    return a.limited ? std::clamp(v, a.limMin, a.limMax) : v;
}

bool AutomationEngine::inPosition(const ActuatorRt& a, double target) const {
    return std::fabs(a.st.pos - target) <= kPosTol && std::fabs(a.st.vel) <= 1e-9;
}

void AutomationEngine::advanceSequence(SeqRt& seq) {
    auto c = ctx();
    auto& st = seq.status;
    st.enabled = seq.enable.isEmpty() || seq.enable.test(c);

    if (st.state == SeqState::Finished) return;

    if (st.state == SeqState::Idle) {
        if (!st.enabled) { st.detail = "disabled (enable condition is false)"; return; }
        if (!seq.startCond.isEmpty() && !seq.startCond.test(c)) {
            st.detail = "waiting for start condition";
            return;
        }
        // Begin a cycle
        for (auto& op : seq.ops) {
            const OpStatus old = op.status;
            op.status = OpStatus{};
            op.status.prevStart = old.startTime >= 0 ? old.startTime - st.cycleStart : old.prevStart;
            op.status.prevEnd   = old.endTime   >= 0 ? old.endTime   - st.cycleStart : old.prevEnd;
        }
        st.cycleStart = m_time;
        st.state = SeqState::Running;
    }

    // Iterate so chains of instantaneous operations (Set, Run, satisfied
    // WaitFor…) complete within a single scan.
    for (int pass = 0; pass < 64; ++pass) {
        bool changed = false;

        for (auto& op : seq.ops) {
            if (op.status.state != OpState::Running) continue;
            if (operationFinished(op)) {
                finishOperation(op);
                changed = true;
            }
        }

        for (int i = 0; i < int(seq.ops.size()); ++i) {
            OpRt& op = seq.ops[size_t(i)];
            if (op.status.state != OpState::Waiting) continue;
            QStringList waitingOn;
            for (int p : op.predIdx)
                if (seq.ops[size_t(p)].status.state != OpState::Done)
                    waitingOn << opLabel(seq.ops[size_t(p)].def, p);
            if (!waitingOn.isEmpty()) { op.status.detail = "after " + waitingOn.join(", "); continue; }
            if (!st.enabled) { op.status.detail = "sequence disabled"; continue; }
            if (!op.startCond.isEmpty() && !op.startCond.test(ctx())) {
                op.status.detail = "start condition: " + op.def.startCondition.trimmed();
                continue;
            }
            startOperation(seq, op);
            changed = true;
            if (operationFinished(op)) finishOperation(op);
        }
        if (!changed) break;
    }

    bool allDone = std::all_of(seq.ops.begin(), seq.ops.end(),
                               [](const OpRt& o) { return o.status.state == OpState::Done; });
    if (allDone) {
        st.cycles++;
        st.lastCycleTime = m_time + m_dt - st.cycleStart;
        // Critical path: back from the operation that finished last, always
        // through the predecessor that finished last
        st.criticalPath.clear();
        st.criticalBusy = 0;
        int cur = -1;
        for (int i = 0; i < int(seq.ops.size()); ++i)
            if (cur < 0 || seq.ops[size_t(i)].status.endTime > seq.ops[size_t(cur)].status.endTime) cur = i;
        while (cur >= 0) {
            const OpRt& o = seq.ops[size_t(cur)];
            st.criticalPath.push_back(o.def.id);
            st.criticalBusy += std::max(0.0, o.status.endTime - o.status.startTime);
            int next = -1;
            for (int p : o.predIdx)
                if (next < 0 || seq.ops[size_t(p)].status.endTime > seq.ops[size_t(next)].status.endTime) next = p;
            cur = next;
        }
        std::reverse(st.criticalPath.begin(), st.criticalPath.end());
        st.state  = seq.loop ? SeqState::Idle : SeqState::Finished;
        st.detail = seq.loop ? "cycle complete — restarting" : "finished";
    } else {
        QStringList running;
        for (int i = 0; i < int(seq.ops.size()); ++i)
            if (seq.ops[size_t(i)].status.state == OpState::Running)
                running << opLabel(seq.ops[size_t(i)].def, i);
        st.detail = running.isEmpty() ? "waiting" : "running: " + running.join(", ");
    }
}

void AutomationEngine::startOperation(SeqRt& seq, OpRt& op) {
    Q_UNUSED(seq);
    auto c = ctx();
    op.status.state     = OpState::Running;
    op.status.startTime = m_time;
    op.status.detail.clear();
    op.status.progress  = 0;
    queueWrite(op.onStart);

    ActuatorRt* a = op.actIdx >= 0 ? &m_actuators[size_t(op.actIdx)] : nullptr;
    switch (op.def.type) {
        case OperationType::Move: {
            if (!a) break;
            double raw = op.target.eval(c);
            a->mode     = ActuatorMode::Position;
            a->target   = clampToLimits(*a, raw);
            a->velocity = op.velocity.isEmpty() ? a->defVelocity : std::fabs(op.velocity.eval(c));
            a->accel    = a->defAccel;
            a->active   = true;
            op.moveTarget = a->target;
            op.moveFrom   = a->st.pos;
            if (a->target != raw)
                m_warnings << QString("%1: target %2 clamped to joint limit %3")
                                  .arg(op.def.name.isEmpty() ? a->name : op.def.name)
                                  .arg(raw).arg(a->target);
            op.status.detail = QString("%1 → %2").arg(a->name).arg(a->target, 0, 'f', 2);
            break;
        }
        case OperationType::MoveToPose: {
            op.poseTargets.clear();
            op.poseFrom.clear();
            op.planned = false;
            op.part    = -1;
            const Pose* pose = m_scene->getPose(op.def.poseId);
            if (!pose) break;
            op.status.detail = QString("→ %1").arg(pose->name);
            planPoseMove(op);
            break;
        }
        case OperationType::Run:
            if (!a) break;
            a->mode     = ActuatorMode::Speed;
            a->velocity = op.velocity.eval(c);
            a->accel    = a->defAccel;
            a->active   = true;
            break;
        case OperationType::Stop:
            if (!a) break;
            a->mode     = ActuatorMode::Speed;
            a->velocity = 0;
            a->accel    = a->defAccel;
            op.status.detail = "stopping " + a->name;
            break;
        case OperationType::Wait:
            op.waitUntil = m_time + std::max(0.0, op.duration.eval(c));
            break;
        case OperationType::WaitFor:
            op.status.detail = "until " + op.def.condition.trimmed();
            break;
        case OperationType::Set:
            break;
        case OperationType::Grip:
            op.status.detail = "waiting for a part";
            break;
        case OperationType::MoveL: {
            op.planned = false;
            op.part    = -1;
            const Pose* pose = m_scene->getPose(op.def.poseId);
            op.path.tool  = op.def.toolId != -1 ? op.def.toolId : pose ? pose->toolId : -1;
            op.path.chain = kin::chainTo(*m_scene, op.path.tool);
            if (!pose || op.path.chain.empty()) break;
            op.status.detail = QString("→ %1 (linear)").arg(pose->name);
            planLinearMove(op);
            break;
        }
        case OperationType::Walk: {
            WalkRequest w;
            w.root = op.def.walkerId;
            if (const SceneObject* to = m_scene->getObject(op.def.walkToId)) {
                const gait::Place place = gait::placeOf(kin::toolFrame(*m_scene, to->id));
                w.path = true;
                w.points = {QPointF(place.x, place.z)};
                w.heading = qRadiansToDegrees(place.heading);
                op.status.detail = "walking to " + to->name;
            } else {
                w.forward = op.forward.isEmpty() ? 0.0 : op.forward.eval(c);
                w.left = op.sideways.isEmpty() ? 0.0 : op.sideways.eval(c);
                w.turn = op.turn.isEmpty() ? 0.0 : op.turn.eval(c);
                op.status.detail = "walking";
            }
            if (!op.velocity.isEmpty()) w.params.speed = std::max(0.0, op.velocity.eval(c));
            QString code, err;
            op.walker = makeWalker(w, &code, &err);
            op.walkStart = m_time;
            op.pathWarned = false;
            if (op.walker) op.walker->begin(*m_scene);
            else m_warnings << QString("%1: %2").arg(op.def.name.isEmpty() ? QString("Walk") : op.def.name, err);
            break;
        }
        case OperationType::Release:
            m_holds.erase(std::remove_if(m_holds.begin(), m_holds.end(),
                                         [&](const Hold& h) { return h.gripperId == op.def.gripperId; }),
                          m_holds.end());
            break;
    }
}

bool AutomationEngine::operationFinished(OpRt& op) {
    ActuatorRt* a = op.actIdx >= 0 ? &m_actuators[size_t(op.actIdx)] : nullptr;
    switch (op.def.type) {
        case OperationType::Move: {
            if (!a) return true;
            double span = std::fabs(op.moveTarget - op.moveFrom);
            op.status.progress = span > 0 ? 1.0 - std::fabs(op.moveTarget - a->st.pos) / span : 1.0;
            if (!a->active) op.status.detail = a->name + " is inactive";
            return inPosition(*a, op.moveTarget);
        }
        case OperationType::MoveToPose: {
            if (!op.planned && !planPoseMove(op)) return false;
            bool all = true;
            op.status.progress = jointProgress(op.poseTargets, op.poseFrom, &all);
            return all;
        }
        case OperationType::Run:
        case OperationType::Set:
            return true;
        case OperationType::Stop:
            return !a || std::fabs(a->st.vel) <= 1e-9;
        case OperationType::Wait: {
            double total = op.waitUntil - op.status.startTime;
            op.status.progress = total > 0 ? std::clamp((m_time - op.status.startTime) / total, 0.0, 1.0) : 1.0;
            op.status.detail = QString("%1 s left").arg(std::max(0.0, op.waitUntil - m_time), 0, 'f', 2);
            return m_time + 1e-9 >= op.waitUntil;
        }
        case OperationType::WaitFor:
            return op.condition.test(ctx());
        case OperationType::Grip: {
            const int gripper = op.def.gripperId;
            int part = op.def.partId;
            if (op.def.sensorId != -1) {
                part = sensorDetection(op.def.sensorId);
                if (part == -1) {
                    const Signal* sg = m_scene->getSignal(op.def.sensorId);
                    op.status.detail = QString("waiting for %1 to detect a part").arg(sg ? sg->name : QString("the sensor"));
                    return false;
                }
            }
            if (!m_scene->getObject(part) || !m_scene->getObject(gripper)) return true;
            if (scenequery::inSubtree(*m_scene, gripper, part)) {
                op.status.detail = "can't grip the gripper itself or what it hangs from";
                return false;
            }
            attach(gripper, part);
            return true;
        }
        case OperationType::Release:
            return true;
        case OperationType::MoveL:
            return stepLinearMove(op);
        case OperationType::Walk: {
            if (!op.walker) return true;
            double error = 0, progress = 0;
            if (!stepWalk(*op.walker, op.walkStart, &error, &progress) && !op.pathWarned) {
                op.pathWarned = true;
                m_warnings << QString("%1: a foot can't reach its spot (%2 mm off)")
                                  .arg(op.def.name.isEmpty() ? QString("Walk") : op.def.name)
                                  .arg(error * 1000, 0, 'f', 1);
            }
            op.status.progress = progress;
            const int done = op.walker->stepsDone(m_time + m_dt - op.walkStart), total = op.walker->stepCount();
            if (total > 0) op.status.detail = QString("step %1 of %2").arg(std::min(done + 1, total)).arg(total);
            if (progress < 1.0) return false;
            op.walker.reset();
            return true;
        }
    }
    return true;
}

int AutomationEngine::partFor(OpRt& op, const Pose& pose) {
    if (op.part != -1) return op.part;   // found once per run of the operation
    if (op.def.sensorId != -1) {
        const int seen = sensorDetection(op.def.sensorId);
        if (seen == -1) {
            const Signal* sg = m_scene->getSignal(op.def.sensorId);
            op.status.detail = QString("waiting for %1 to find a part").arg(sg ? sg->name : QString("the sensor"));
            return -1;
        }
        // The pose was taught on a part's frame: use the object (or its
        // ancestor) that is that part or a copy of it, not a piece of it
        op.part = seen;
        for (const SceneObject* o = m_scene->getObject(seen); o; o = m_scene->getObject(o->parentId))
            if (o->id == pose.frameId || o->copyOf == pose.frameId) { op.part = o->id; break; }
    } else {
        op.part = op.def.partId != -1 ? op.def.partId : pose.frameId;
    }
    return op.part;
}

void AutomationEngine::commandJoints(const std::map<int, double>& values, double pct,
                                     std::vector<std::pair<int, double>>& targets, std::vector<double>& from) {
    // Synchronized point-to-point: every axis uses the slowest axis's
    // profile scaled by its own distance, so all arrive together.
    double leaderV = 0, leaderA = 0, leaderD = 0, leaderT = 0;
    const size_t first = targets.size();
    for (auto& [jid, val] : values) {
        const int idx = actIndexForJoint(jid);
        if (idx < 0) continue;
        ActuatorRt& ax = m_actuators[size_t(idx)];
        double tgt = clampToLimits(ax, val);
        double d   = std::fabs(tgt - ax.st.pos);
        double v   = std::fabs(ax.defVelocity) * pct / 100.0;
        double acc = ax.defAccel * pct / 100.0;
        double T   = motion::moveTime(d, v, acc);
        if (T > leaderT) { leaderT = T; leaderV = v; leaderA = acc; leaderD = d; }
        targets.emplace_back(idx, tgt);
        from.push_back(ax.st.pos);
    }
    for (size_t i = first; i < targets.size(); ++i) {
        auto& [idx, tgt] = targets[i];
        ActuatorRt& ax = m_actuators[size_t(idx)];
        double d = std::fabs(tgt - ax.st.pos);
        double k = leaderD > 0 ? d / leaderD : 0.0;
        ax.mode     = ActuatorMode::Position;
        ax.target   = tgt;
        ax.velocity = std::max(1e-6, leaderV * k);
        ax.accel    = leaderA * k;
        ax.active   = true;
    }
}

double AutomationEngine::jointProgress(const std::vector<std::pair<int, double>>& targets,
                                       const std::vector<double>& from, bool* arrived) const {
    double worst = 1.0;
    bool all = true;
    for (size_t i = 0; i < targets.size(); ++i) {
        const ActuatorRt& ax = m_actuators[size_t(targets[i].first)];
        const double tgt  = targets[i].second;
        const double span = std::fabs(tgt - from[i]);
        if (span > 0) worst = std::min(worst, 1.0 - std::fabs(tgt - ax.st.pos) / span);
        if (!inPosition(ax, tgt)) all = false;
    }
    if (arrived) *arrived = all;
    return std::clamp(worst, 0.0, 1.0);
}

int AutomationEngine::actIndexForJoint(int jointId) const {
    const Motor* m = m_scene->motorForJoint(jointId);
    if (!m) return -1;
    auto it = m_actIndexByMotor.find(m->id);
    return it != m_actIndexByMotor.end() ? it->second : -1;
}

bool AutomationEngine::planPoseMove(OpRt& op) {
    const Pose* pose = m_scene->getPose(op.def.poseId);
    if (!pose) { op.planned = true; return true; }
    std::map<int, float> solved;
    std::map<int, double> values;
    if (pose->isRelative()) {
        const int part = partFor(op, *pose);
        if (part == -1) return false;
        const kin::Result r = kin::solveFor(*m_scene, *pose, pose->toolId,
                                            kin::poseTarget(*m_scene, *pose, part), &solved);
        values.insert(solved.begin(), solved.end());
        if (!r.reached && r.positionError > 1e-3f)
            m_warnings << QString("%1: %2 is out of reach on %3 (%4 mm off)")
                              .arg(op.def.name.isEmpty() ? QString("MoveToPose") : op.def.name, pose->name)
                              .arg(m_scene->getObject(part) ? m_scene->getObject(part)->name : QString("the part"))
                              .arg(double(r.positionError) * 1000, 0, 'f', 1);
        const SceneObject* p = m_scene->getObject(part);
        op.status.detail = QString("→ %1 on %2").arg(pose->name, p ? p->name : QString("?"));
    } else {
        values.insert(pose->jointValues.begin(), pose->jointValues.end());
    }
    const double pct = op.velocity.isEmpty() ? 100.0 : std::max(0.1, op.velocity.eval(ctx()));
    commandJoints(values, pct, op.poseTargets, op.poseFrom);
    op.planned = true;
    return true;
}

bool AutomationEngine::planLinearMove(OpRt& op) {
    const Pose* pose = m_scene->getPose(op.def.poseId);
    if (!pose || op.path.chain.empty()) { op.planned = true; return true; }
    if (pose->isRelative()) {
        const int part = partFor(op, *pose);
        if (part == -1) return false;
        op.path.to = kin::poseTarget(*m_scene, *pose, part);
        const SceneObject* p = m_scene->getObject(part);
        op.status.detail = QString("→ %1 on %2 (linear)").arg(pose->name, p ? p->name : QString("?"));
    } else {
        std::map<int, float> values(pose->jointValues.begin(), pose->jointValues.end());
        op.path.to = kin::toolFrameAt(*m_scene, op.path.tool, values);
    }
    const double speed = op.velocity.isEmpty() ? 0.25 : std::max(1e-4, std::fabs(op.velocity.eval(ctx())));
    beginPath(op.path, speed);
    op.pathWarned = false;
    op.planned = true;
    return true;
}

void AutomationEngine::beginPath(LinearPath& p, double speed) {
    p.from = kin::toolFrame(*m_scene, p.tool);
    p.orient = kin::canOrient(p.chain);
    const double dist = (p.to.column(3) - p.from.column(3)).toVector3D().length();
    double angle = 0;
    if (p.orient) {
        const QQuaternion qa = kin::targetFrom(p.from).orientation, qb = kin::targetFrom(p.to).orientation;
        angle = 2.0 * std::acos(std::min(1.0, std::fabs(double(QQuaternion::dotProduct(qa, qb)))));
    }
    // A radian of tool rotation takes as long as 0.3 m of travel
    p.duration = std::max(dist, 0.3 * angle) / speed;
    p.start = m_time;
}

// One scan of a straight-line tool move: the next point on the path (eased
// in and out), inverse kinematics from the current joint values, and the
// chain's actuators set straight to the solution.
double AutomationEngine::stepPath(LinearPath& p, kin::Result* result) {
    const double T = p.duration;
    const double s = T > 1e-9 ? std::clamp((m_time + m_dt - p.start) / T, 0.0, 1.0) : 1.0;
    const double k = s * s * (3 - 2 * s);

    kin::Target t;
    const kin::Target a = kin::targetFrom(p.from), b = kin::targetFrom(p.to);
    t.position = a.position + (b.position - a.position) * float(k);
    t.orientation = QQuaternion::slerp(a.orientation, b.orientation, float(k));
    t.useOrientation = p.orient;
    const kin::Result r = kin::solve(*m_scene, p.tool, p.chain, t, 30);
    if (result) *result = r;
    for (int jid : p.chain) {
        const int idx = actIndexForJoint(jid);
        const Joint* j = m_scene->getJoint(jid);
        if (idx < 0 || !j) continue;
        ActuatorRt& ax = m_actuators[size_t(idx)];
        const double v = clampToLimits(ax, j->currentValue);
        ax.st.vel  = m_dt > 0 ? (v - ax.st.pos) / m_dt : 0.0;
        ax.st.pos  = v;
        ax.target  = v;
        ax.mode    = ActuatorMode::Position;
        ax.active  = true;
        ax.direct  = true;
    }
    if (s >= 1.0)
        for (int jid : p.chain)
            if (const int idx = actIndexForJoint(jid); idx >= 0) m_actuators[size_t(idx)].st.vel = 0;
    return s;
}

// To a pose on a part, the end of the path follows the part, so the tool
// meets it where it is.
bool AutomationEngine::stepLinearMove(OpRt& op) {
    if (op.path.chain.empty()) return true;
    if (!op.planned && !planLinearMove(op)) return false;
    if (const Pose* pose = m_scene->getPose(op.def.poseId);
        pose && pose->isRelative() && m_scene->getObject(op.part) && !m_scene->isParked(op.part))
        op.path.to = kin::poseTarget(*m_scene, *pose, op.part);
    kin::Result r;
    const double s = stepPath(op.path, &r);
    op.status.progress = s;
    if (!r.reached && !op.pathWarned && r.positionError > 1e-3f) {
        op.pathWarned = true;
        m_warnings << QString("%1: tool path out of reach (%2 mm off)")
                          .arg(op.def.name.isEmpty() ? QString("MoveL") : op.def.name)
                          .arg(double(r.positionError) * 1000, 0, 'f', 1);
    }
    return s >= 1.0;
}

void AutomationEngine::attach(int gripperId, int partId) {
    m_holds.erase(std::remove_if(m_holds.begin(), m_holds.end(),
                                 [&](const Hold& h) { return h.partId == partId; }),
                  m_holds.end());
    Hold h;
    h.gripperId = gripperId;
    h.partId    = partId;
    h.relative  = scenequery::stripScale(m_scene->worldTransform(gripperId)).inverted() *
                  m_scene->worldTransform(partId);
    m_holds.push_back(h);
}

void AutomationEngine::applyHolds() {
    for (const Hold& h : m_holds) {
        if (!m_scene->getObject(h.gripperId) || !m_scene->getObject(h.partId)) continue;
        m_scene->setWorldTransform(h.partId,
                                   scenequery::stripScale(m_scene->worldTransform(h.gripperId)) * h.relative);
    }
}

// ── Part flow ────────────────────────────────────────────────────────────────

// Whether any surface in the part's subtree overlaps `box`
bool AutomationEngine::partTouches(int rootId, const scenequery::Obb& box) const {
    const SceneObject* o = m_scene->getObject(rootId);
    if (!o) return false;
    if (surfaceOf(*o) && scenequery::overlap(box, scenequery::worldBox(*m_scene, rootId))) return true;
    for (int c : o->childIds)
        if (partTouches(c, box)) return true;
    return false;
}

// A source's spot is taken while any part (a copy, or a free rigid body)
// overlaps the template's surfaces. They are shrunk a little, so a part that
// only touches the spot (e.g. the previous copy, one length further on)
// doesn't block it.
bool AutomationEngine::spotTaken(const SourceRt& src) const {
    std::vector<scenequery::Obb> spot;
    std::function<void(int)> collect = [&](int id) {
        const SceneObject* o = m_scene->getObject(id);
        if (!o) return;
        if (surfaceOf(*o)) {
            scenequery::Obb b = scenequery::worldBox(*m_scene, id);
            for (int i = 0; i < 3; ++i) b.half[i] = std::max(0.0f, b.half[i] - 1e-4f);
            spot.push_back(b);
        }
        for (int c : o->childIds) collect(c);
    };
    collect(src.def.templateId);
    for (int r : m_scene->rootObjects()) {
        const SceneObject* o = m_scene->getObject(r);
        if (!o || o->parked) continue;
        if (o->copyOf == -1 && !m_scene->allRigidBodies().count(r)) continue;
        for (const auto& b : spot)
            if (partTouches(r, b)) return true;
    }
    return false;
}

void AutomationEngine::updateFlow() {
    if (m_sources.empty() && m_sinks.empty()) return;
    const auto c = ctx();

    // Sinks first, so a part that has just left a source's spot into a sink
    // frees the spot this scan
    for (auto& sink : m_sinks) {
        if (!sink.enable.isEmpty() && !sink.enable.test(c)) continue;
        if (!sensors::problem(*m_scene, sink.def.zone).isEmpty()) continue;
        const scenequery::Obb box = sensors::zone(*m_scene, sink.def.zone);
        const int mount = sink.def.zone.objectId;
        for (int r : std::vector<int>(m_scene->rootObjects())) {
            const SceneObject* o = m_scene->getObject(r);
            if (!o || o->parked) continue;
            const bool copy = o->copyOf != -1;
            if (!copy && !m_scene->allRigidBodies().count(r)) continue;   // only free parts
            if (sink.def.sourceId != -1 && o->sourceId != sink.def.sourceId) continue;
            if (mount != -1 && (scenequery::inSubtree(*m_scene, mount, r) || scenequery::inSubtree(*m_scene, r, mount)))
                continue;
            if (std::any_of(m_holds.begin(), m_holds.end(),
                            [&](const Hold& h) { return scenequery::inSubtree(*m_scene, h.partId, r); }))
                continue;   // still being handled
            if (!partTouches(r, box)) continue;
            m_scene->setParked(r, true);
            ++sink.count;
            m_structureChanged = true;
        }
    }

    for (auto& src : m_sources) {
        const bool enabled = src.enable.isEmpty() || src.enable.test(c);
        if (src.def.mode == SourceMode::Trigger) {
            const bool t = src.trigger.test(c);
            if (t && !src.prevTrigger && enabled) src.pending = std::min(src.pending + 1, 1000);
            src.prevTrigger = t;
        }
        src.blocked = false;
        const bool room = src.def.maxCount <= 0 || src.count < src.def.maxCount;
        const bool due = src.def.mode == SourceMode::Interval ? m_time + 1e-9 >= src.nextAt : src.pending > 0;
        if (!enabled || !room || !due) continue;
        const SceneObject* t = m_scene->getObject(src.def.templateId);
        if (!t) continue;
        if (src.def.whenClear && spotTaken(src)) { src.blocked = true; continue; }
        m_scene->spawnCopy(src.def.templateId, src.def.id, QString("%1 #%2").arg(t->name).arg(src.count + 1));
        ++src.count;
        if (src.def.mode == SourceMode::Interval) src.nextAt = m_time + std::max(0.0, src.interval.eval(c));
        else --src.pending;
        m_structureChanged = true;
    }
}

int AutomationEngine::sourceCount(int sourceId) const {
    for (const auto& s : m_sources) if (s.def.id == sourceId) return s.count;
    return 0;
}

int AutomationEngine::sinkCount(int sinkId) const {
    for (const auto& s : m_sinks) if (s.def.id == sinkId) return s.count;
    return 0;
}

void AutomationEngine::finishOperation(OpRt& op) {
    op.status.state    = OpState::Done;
    op.status.endTime  = m_time;
    op.status.progress = 1.0;
    op.status.detail.clear();
    queueWrite(op.onFinish);
}

void AutomationEngine::applyBindings() {
    auto c = ctx();
    for (auto& b : m_belts) b.value = b.speed.eval(c);
    for (auto& a : m_actuators) {
        if (!a.bindTarget.isEmpty()) {
            a.mode   = ActuatorMode::Position;
            a.target = clampToLimits(a, a.bindTarget.eval(c));
        }
        if (!a.bindVelocity.isEmpty()) {
            double v = a.bindVelocity.eval(c);
            a.velocity = a.mode == ActuatorMode::Position ? std::fabs(v) : v;
        }
        if (!a.bindActive.isEmpty())
            a.active = a.bindActive.test(c);
    }
}

void AutomationEngine::integrate(double dt) {
    for (auto& a : m_actuators) {
        if (a.direct) {
            // Placed by a MoveL this scan: no profile
            a.direct = false;
            if (Joint* j = m_scene->getJoint(a.jointId)) j->currentValue = float(a.st.pos);
            continue;
        }
        if (!a.active)
            motion::stepSpeed(a.st, 0.0, a.accel, dt);
        else if (a.mode == ActuatorMode::Position)
            motion::stepPosition(a.st, a.target, a.velocity, a.accel, dt);
        else
            motion::stepSpeed(a.st, a.velocity, a.accel, dt);

        if (a.limited) {
            if (a.st.pos < a.limMin) { a.st.pos = a.limMin; a.st.vel = 0; }
            if (a.st.pos > a.limMax) { a.st.pos = a.limMax; a.st.vel = 0; }
        }
        if (Joint* j = m_scene->getJoint(a.jointId))
            j->currentValue = float(a.st.pos);
    }
    m_scene->applyCouplings();
}


// ── Motions commanded from outside ───────────────────────────────────────────

namespace {
constexpr double kReachTol  = 1e-3;   // m: a tool goal or path point this far off is out of reach
constexpr double kAngleTol  = 0.5;    // degrees
constexpr size_t kKeepFinished = 2000;

QString jointLabel(const SceneGraph& g, int jid) {
    if (const Motor* m = g.motorForJoint(jid)) return m->name;
    const Joint* j = g.getJoint(jid);
    return j ? j->name : QString("joint %1").arg(jid);
}
QString unitOf(const Joint& j) { return j.type == JointType::Prismatic ? " mm" : "°"; }
double shown(const Joint& j, double v) { return j.type == JointType::Prismatic ? v * 1000 : v; }

QMatrix4x4 rotationXyz(double rx, double ry, double rz) {
    QMatrix4x4 m;
    m.rotate(float(rx), 1, 0, 0);
    m.rotate(float(ry), 0, 1, 0);
    m.rotate(float(rz), 0, 0, 1);
    return m;
}
QMatrix4x4 rotationOnly(QMatrix4x4 m) {
    m.setColumn(3, {0, 0, 0, 1});
    return m;
}
} // namespace

QString AutomationEngine::motionStateName(MotionState s) {
    switch (s) {
        case MotionState::Queued:    return "queued";
        case MotionState::Running:   return "running";
        case MotionState::Done:      return "done";
        case MotionState::Failed:    return "failed";
        case MotionState::Cancelled: return "cancelled";
    }
    return {};
}

int AutomationEngine::submitMotion(const MotionRequest& r, QString* code, QString* error) {
    auto reject = [&](const QString& c, const QString& e) {
        if (code) *code = c;
        if (error) *error = e;
        return 0;
    };
    if (!m_active) return reject("not_running", "the simulation is not running");
    MotionRt m;
    m.req = r;
    if (r.kind == MotionKind::Joints) {
        if (r.joints.empty()) return reject("bad_request", "no joints to move");
        if (r.speed < 0 || r.speed > 100) return reject("bad_request", "speed must be between 0 and 100 %");
        for (auto& [jid, v] : r.joints) {
            const Joint* j = m_scene->getJoint(jid);
            if (!j) return reject("not_found", QString("joint %1 does not exist").arg(jid));
            const int idx = actIndexForJoint(jid);
            if (idx < 0) return reject("not_actuated", QString("%1 has no actuator").arg(jointLabel(*m_scene, jid)));
            const ActuatorRt& a = m_actuators[size_t(idx)];
            if (!std::isfinite(v)) return reject("bad_request", QString("%1: the value is not a number").arg(a.name));
            if (!r.relativeJoints && a.limited && (v < a.limMin - kPosTol || v > a.limMax + kPosTol))
                return reject("out_of_limits", QString("%1 = %2%3 is outside its range %4 … %5%3")
                                  .arg(a.name).arg(shown(*j, v)).arg(unitOf(*j))
                                  .arg(shown(*j, a.limMin)).arg(shown(*j, a.limMax)));
            m.joints.push_back(jid);
        }
    } else if (r.kind == MotionKind::Walk) {
        QString c, e;
        const std::shared_ptr<gait::Walker> w = makeWalker(r.walk, &c, &e);
        if (!w) return reject(c, e);
        m.joints = w->joints();
        for (int jid : m.joints)
            if (actIndexForJoint(jid) < 0)
                return reject("not_actuated", QString("%1 has no actuator").arg(jointLabel(*m_scene, jid)));
    } else {
        if (!m_scene->getObject(r.tool)) return reject("not_found", "the tool does not exist");
        if (m.req.chain.empty()) m.req.chain = kin::chainTo(*m_scene, r.tool);
        if (m.req.chain.empty())
            return reject("not_actuated", QString("no actuated joint moves %1").arg(m_scene->getObject(r.tool)->name));
        for (int jid : m.req.chain)
            if (actIndexForJoint(jid) < 0)
                return reject("not_actuated", QString("%1 has no actuator").arg(jointLabel(*m_scene, jid)));
        if (r.goal.hasRotation() && !kin::canOrient(m.req.chain))
            return reject("bad_request", QString("%1 joints can't hold an orientation (it takes 6): give only "
                                                 "x, y and z").arg(m.req.chain.size()));
        if (r.goal.frameId != -1 && !m_scene->getObject(r.goal.frameId))
            return reject("not_found", "the reference frame does not exist");
        if (r.kind == MotionKind::Linear ? r.speed < 0 : (r.speed < 0 || r.speed > 100))
            return reject("bad_request", r.kind == MotionKind::Linear ? QString("speed must be positive")
                                                                      : QString("speed must be between 0 and 100 %"));
        m.joints = m.req.chain;
    }
    if (m.req.label.isEmpty())
        m.req.label = r.kind == MotionKind::Linear ? "moveL" : r.kind == MotionKind::JointsToPose ? "moveJ"
                    : r.kind == MotionKind::Walk ? "walk" : "moveJoints";
    m.id = m_nextMotionId++;
    m_motions.push_back(std::move(m));
    return m_motions.back().id;
}

bool AutomationEngine::hasMotion(int id) const {
    if (m_finishedMotions.count(id)) return true;
    return std::any_of(m_motions.begin(), m_motions.end(), [&](const MotionRt& m) { return m.id == id; });
}

AutomationEngine::MotionStatus AutomationEngine::motionStatus(int id) const {
    for (const MotionRt& m : m_motions)
        if (m.id == id) return m.status;
    auto it = m_finishedMotions.find(id);
    if (it != m_finishedMotions.end()) return it->second;
    MotionStatus st;
    st.state = MotionState::Failed;
    st.code  = "not_found";
    st.error = QString("there is no motion %1").arg(id);
    return st;
}

void AutomationEngine::holdJoints(const std::vector<int>& joints) {
    for (int jid : joints) {
        const int idx = actIndexForJoint(jid);
        if (idx < 0) continue;
        ActuatorRt& a = m_actuators[size_t(idx)];
        // Brake at the actuator's own deceleration, then hold
        a.mode     = ActuatorMode::Speed;
        a.velocity = 0;
        a.accel    = a.defAccel;
        a.direct   = false;
    }
}

void AutomationEngine::finishMotion(MotionRt& m, MotionState state, const QString& code, const QString& error) {
    const bool wasRunning = m.status.state == MotionState::Running;
    m.status.state = state;
    m.status.code  = code;
    m.status.error = error;
    if (state == MotionState::Done) m.status.progress = 1;
    else if (wasRunning) holdJoints(m.joints);
    m_finishedMotions[m.id] = m.status;
    while (m_finishedMotions.size() > kKeepFinished) m_finishedMotions.erase(m_finishedMotions.begin());
}

void AutomationEngine::cancelMotion(int id, const QString& why) {
    for (size_t i = 0; i < m_motions.size(); ++i) {
        if (m_motions[i].id != id) continue;
        finishMotion(m_motions[i], MotionState::Cancelled, "cancelled", why.isEmpty() ? QString("cancelled") : why);
        m_motions.erase(m_motions.begin() + long(i));
        return;
    }
}

int AutomationEngine::cancelMotions(const std::set<int>& joints, const QString& why, const QString& code) {
    int n = 0;
    for (size_t i = 0; i < m_motions.size();) {
        MotionRt& m = m_motions[i];
        const bool hit = joints.empty() || std::any_of(m.joints.begin(), m.joints.end(),
                                                       [&](int j) { return joints.count(j) > 0; });
        if (!hit) { ++i; continue; }
        finishMotion(m, MotionState::Cancelled, code, why.isEmpty() ? QString("stopped") : why);
        m_motions.erase(m_motions.begin() + long(i));
        ++n;
    }
    return n;
}

bool AutomationEngine::runActuator(int jointId, double speed, QString* error) {
    const int idx = m_active ? actIndexForJoint(jointId) : -1;
    if (idx < 0) {
        if (error) *error = m_active ? QString("%1 has no actuator").arg(jointLabel(*m_scene, jointId))
                                     : QString("the simulation is not running");
        return false;
    }
    cancelMotions({jointId}, "the actuator was set to run at a speed");
    ActuatorRt& a = m_actuators[size_t(idx)];
    a.mode     = ActuatorMode::Speed;
    a.velocity = speed;
    a.accel    = a.defAccel;
    a.active   = true;
    return true;
}

QMatrix4x4 AutomationEngine::goalFrame(int tool, const CartesianGoal& g) const {
    const QMatrix4x4 cur = kin::toolFrame(*m_scene, tool);
    const QMatrix4x4 ref = g.frameId != -1 ? kin::toolFrame(*m_scene, g.frameId) : QMatrix4x4();
    if (!g.relative) {
        const QMatrix4x4 local = ref.inverted() * cur;
        const Transform t = SceneGraph::toTransform(local);
        QMatrix4x4 out;
        out.translate(float(g.x.value_or(t.position.x())), float(g.y.value_or(t.position.y())),
                      float(g.z.value_or(t.position.z())));
        out *= g.hasRotation() ? rotationXyz(g.rx.value_or(t.rotation.x()), g.ry.value_or(t.rotation.y()),
                                             g.rz.value_or(t.rotation.z()))
                               : rotationOnly(local);
        return ref * out;
    }
    const QVector3D d(float(g.x.value_or(0)), float(g.y.value_or(0)), float(g.z.value_or(0)));
    const QMatrix4x4 rot = rotationXyz(g.rx.value_or(0), g.ry.value_or(0), g.rz.value_or(0));
    if (g.toolAxes) {
        QMatrix4x4 off;
        off.translate(d);
        return cur * off * rot;
    }
    // Along the reference frame's axes, turning about the tool's own point
    const QMatrix4x4 refRot = rotationOnly(ref);
    QMatrix4x4 out;
    out.translate(cur.column(3).toVector3D() + refRot.map(d));
    out *= refRot * rot * refRot.inverted() * rotationOnly(cur);
    return out;
}

bool AutomationEngine::compileExpression(const QString& source, expr::Expression& out, QString* error) const {
    if (!m_active) {
        if (error) *error = "the simulation is not running";
        return false;
    }
    return out.compile(source, [this](const QString& n) { return m_slotIndex.value(n, -1); }, error);
}

bool AutomationEngine::startMotion(MotionRt& m) {
    auto fail = [&](const QString& code, const QString& err) {
        finishMotion(m, MotionState::Failed, code, err);
        return false;
    };
    const MotionRequest& r = m.req;
    if (r.kind == MotionKind::Walk) {
        // Planned from where the robot stands now (after the walks before it)
        QString code, err;
        m.walker = makeWalker(r.walk, &code, &err);
        if (!m.walker) return fail(code, err);
        m.walker->begin(*m_scene);
        m.walkStart = m_time;
        m.status.state = MotionState::Running;
        return true;
    }
    if (r.kind == MotionKind::Joints) {
        std::map<int, double> values;
        for (auto& [jid, v] : r.joints) {
            const ActuatorRt& a = m_actuators[size_t(actIndexForJoint(jid))];
            const double t = r.relativeJoints ? a.st.pos + v : v;
            if (a.limited && (t < a.limMin - kPosTol || t > a.limMax + kPosTol)) {
                const Joint& j = *m_scene->getJoint(jid);
                return fail("out_of_limits", QString("%1 would go to %2%3, outside its range %4 … %5%3")
                                                 .arg(a.name).arg(shown(j, t)).arg(unitOf(j))
                                                 .arg(shown(j, a.limMin)).arg(shown(j, a.limMax)));
            }
            values[jid] = t;
        }
        commandJoints(values, r.speed > 0 ? r.speed : 100.0, m.targets, m.from);
    } else {
        const QMatrix4x4 goal = goalFrame(r.tool, r.goal);
        const bool orient = kin::canOrient(r.chain);
        // Solve the end first, so a goal out of reach fails now instead of
        // half-way. A joint move may end in any configuration, so when the
        // solution from where the joints are misses (a stretched arm is
        // singular), it tries other starting points too.
        std::map<int, float> saved;
        for (auto& [jid, j] : m_scene->allJoints()) saved[jid] = j.currentValue;
        kin::Target t = kin::targetFrom(goal);
        t.useOrientation = orient;
        auto missed = [&](const kin::Result& x) {
            return x.positionError > kReachTol || (orient && x.angleError > kAngleTol);
        };
        kin::Result res = kin::solve(*m_scene, r.tool, r.chain, t, 200);
        std::map<int, double> solved;
        for (int jid : r.chain) solved[jid] = m_scene->getJoint(jid)->currentValue;
        if (missed(res) && r.kind == MotionKind::JointsToPose) {
            quint32 seed = 12345;
            auto rnd = [&] { seed = seed * 1664525u + 1013904223u; return double(seed >> 8) / double(1u << 24); };
            double bestTravel = 1e30;
            for (int attempt = 0; attempt < 24; ++attempt) {
                // Mostly near the current pose, more spread out later
                const double spread = 0.15 + 0.85 * attempt / 23.0;
                for (int jid : r.chain) {
                    Joint* j = m_scene->getJoint(jid);
                    const ActuatorRt& a = m_actuators[size_t(actIndexForJoint(jid))];
                    const double lo = a.limited ? a.limMin : saved[jid] - 180, hi = a.limited ? a.limMax : saved[jid] + 180;
                    const double v = saved[jid] + (lo + rnd() * (hi - lo) - saved[jid]) * spread;
                    j->currentValue = float(std::clamp(v, lo, hi));
                }
                m_scene->applyCouplings();
                const kin::Result x = kin::solve(*m_scene, r.tool, r.chain, t, 200);
                if (missed(x) && !missed(res)) continue;
                double travel = 0;
                for (int jid : r.chain) travel += std::fabs(m_scene->getJoint(jid)->currentValue - saved[jid]);
                const bool better = missed(res) ? x.positionError + x.angleError * 1e-3 < res.positionError + res.angleError * 1e-3
                                                : travel < bestTravel;
                if (!better) continue;
                res = x;
                if (!missed(x)) bestTravel = travel;
                for (int jid : r.chain) solved[jid] = m_scene->getJoint(jid)->currentValue;
            }
        }
        for (auto& [jid, v] : saved) m_scene->getJoint(jid)->currentValue = v;
        m_scene->applyCouplings();
        if (missed(res))
            return fail("out_of_reach", orient && res.positionError <= kReachTol
                                            ? QString("the target orientation is out of reach (%1° off)").arg(double(res.angleError), 0, 'f', 1)
                                            : QString("the target is out of reach (%1 mm off)").arg(double(res.positionError) * 1000, 0, 'f', 1));
        if (r.kind == MotionKind::JointsToPose) {
            commandJoints(solved, r.speed > 0 ? r.speed : 100.0, m.targets, m.from);
        } else {
            m.path.tool  = r.tool;
            m.path.chain = r.chain;
            m.path.to    = goal;
            beginPath(m.path, r.speed > 0 ? r.speed : 0.25);
        }
    }
    m.status.state = MotionState::Running;
    return true;
}

void AutomationEngine::advanceMotions() {
    std::set<int> busy;   // joints of earlier unfinished motions: later ones that share one wait
    for (size_t i = 0; i < m_motions.size();) {
        MotionRt& m = m_motions[i];
        const bool blocked = std::any_of(m.joints.begin(), m.joints.end(), [&](int j) { return busy.count(j) > 0; });
        busy.insert(m.joints.begin(), m.joints.end());
        bool finished = false;
        if (m.status.state == MotionState::Queued && !blocked && !startMotion(m)) {
            finished = true;
            // Later motions that share a joint with it were planned from where it would end
            const std::set<int> joints(m.joints.begin(), m.joints.end());
            const QString why = QString("an earlier %1 failed: %2").arg(m.req.label, m.status.error);
            for (size_t k = i + 1; k < m_motions.size();) {
                MotionRt& o = m_motions[k];
                if (std::any_of(o.joints.begin(), o.joints.end(), [&](int j) { return joints.count(j) > 0; })) {
                    finishMotion(o, MotionState::Cancelled, "cancelled", why);
                    m_motions.erase(m_motions.begin() + long(k));
                } else {
                    ++k;
                }
            }
        } else if (m.status.state == MotionState::Running) {
            if (m.req.kind == MotionKind::Walk) {
                double error = 0, progress = 0;
                if (!stepWalk(*m.walker, m.walkStart, &error, &progress)) {
                    const int step = m.walker->stepsDone(m_time + m_dt - m.walkStart) + 1;
                    finishMotion(m, MotionState::Failed, "out_of_reach",
                                 QString("a foot can't reach its spot at step %1 (%2 mm off): take shorter steps")
                                     .arg(step).arg(error * 1000, 0, 'f', 1));
                    finished = true;
                } else {
                    m.status.progress = progress;
                    if (progress >= 1.0) {
                        finishMotion(m, MotionState::Done);
                        finished = true;
                    }
                }
            } else if (m.req.kind == MotionKind::Linear) {
                kin::Result res;
                const double s = stepPath(m.path, &res);
                m.status.progress = s;
                if (res.positionError > kReachTol || (m.path.orient && res.angleError > kAngleTol)) {
                    finishMotion(m, MotionState::Failed, "out_of_reach",
                                 QString("the straight line leaves the reach of the robot at %1 % (%2 mm off)")
                                     .arg(int(s * 100)).arg(double(res.positionError) * 1000, 0, 'f', 1));
                    finished = true;
                } else if (s >= 1.0) {
                    finishMotion(m, MotionState::Done);
                    finished = true;
                }
            } else {
                bool arrived = false;
                m.status.progress = jointProgress(m.targets, m.from, &arrived);
                if (arrived) {
                    finishMotion(m, MotionState::Done);
                    finished = true;
                }
            }
        }
        if (finished) m_motions.erase(m_motions.begin() + long(i));
        else ++i;
    }
}

// ── Walking ──────────────────────────────────────────────────────────────────

std::shared_ptr<gait::Walker> AutomationEngine::makeWalker(const WalkRequest& w, QString* code, QString* error) const {
    auto reject = [&](const QString& c, const QString& e) {
        if (code) *code = c;
        if (error) *error = e;
        return std::shared_ptr<gait::Walker>();
    };
    if (!m_scene->getObject(w.root)) return reject("not_found", "the robot does not exist");
    gait::Legs legs;
    QString why;
    if (!gait::findLegs(*m_scene, w.root, &legs, &why, w.leftFoot, w.rightFoot)) return reject("bad_request", why);
    gait::Params p = w.params;
    why = gait::resolve(p, legs);
    if (!why.isEmpty()) return reject("bad_request", why);
    auto finite = [](double v) { return std::isfinite(v); };
    if (!finite(w.forward) || !finite(w.left) || !finite(w.turn) || (w.heading && !finite(*w.heading)))
        return reject("bad_request", "the distances and angles must be numbers");
    for (const QPointF& q : w.points)
        if (!finite(q.x()) || !finite(q.y())) return reject("bad_request", "the points must be numbers");
    if (w.path && w.points.empty() && !w.heading) return reject("bad_request", "a path needs at least one point");

    const gait::Place start = gait::placeOf(kin::toolFrame(*m_scene, w.root));
    // Points in a frame: on the floor below that frame's spot, turned with it
    if (w.frameId != -1 && !m_scene->getObject(w.frameId)) return reject("not_found", "the reference frame does not exist");
    const gait::Place ref = w.fromStart ? start : w.frameId != -1 ? gait::placeOf(kin::toolFrame(*m_scene, w.frameId))
                                                                   : gait::Place{};
    const QMatrix4x4 refFrame = gait::frameOf(ref);
    std::vector<QPointF> points;
    for (const QPointF& q : w.points) {
        const QVector3D v = refFrame.map(QVector3D(float(q.x()), 0, float(q.y())));
        points.push_back(QPointF(v.x(), v.z()));
    }
    std::optional<double> heading;
    if (w.heading) heading = ref.heading + qDegreesToRadians(*w.heading);
    const std::vector<gait::Place> places = w.path
        ? gait::pathPlaces(start, points, heading, p)
        : gait::relativePlaces(start, w.forward, w.left, qDegreesToRadians(w.turn), p);
    if (places.size() > 5000) return reject("bad_request", "the walk is too long (over 5000 steps)");
    return std::make_shared<gait::Walker>(std::move(legs), gait::makePlan(start, places, p));
}

bool AutomationEngine::stepWalk(gait::Walker& w, double start, double* error, double* progress) {
    const double t = m_time + m_dt - start;
    std::map<int, double> values;
    const bool ok = w.apply(*m_scene, t, &values, error);
    for (auto& [jid, v] : values) {
        const int idx = actIndexForJoint(jid);
        if (idx < 0) continue;
        ActuatorRt& ax = m_actuators[size_t(idx)];
        const double c = clampToLimits(ax, v);
        ax.st.vel = m_dt > 0 ? (c - ax.st.pos) / m_dt : 0.0;
        ax.st.pos = c;
        ax.target = c;
        ax.mode   = ActuatorMode::Position;
        ax.active = true;
        ax.direct = true;
    }
    const double T = w.duration();
    *progress = T > 1e-9 ? std::clamp(t / T, 0.0, 1.0) : 1.0;
    if (*progress >= 1.0)
        for (auto& [jid, v] : values)
            if (const int idx = actIndexForJoint(jid); idx >= 0) m_actuators[size_t(idx)].st.vel = 0;
    return ok;
}

int AutomationEngine::stopWalking(int root) {
    int n = 0;
    for (size_t i = 0; i < m_motions.size();) {
        MotionRt& m = m_motions[i];
        if (m.req.kind != MotionKind::Walk || m.req.walk.root != root) { ++i; continue; }
        ++n;
        if (m.status.state == MotionState::Running && m.walker) {
            m.walker->stopAfter(m_time - m.walkStart);
            ++i;
        } else {
            finishMotion(m, MotionState::Cancelled, "cancelled", "the walk was stopped");
            m_motions.erase(m_motions.begin() + long(i));
        }
    }
    for (auto& s : m_sequences)
        for (auto& op : s.ops)
            if (op.walker && op.walker->legs().root == root) {
                op.walker->stopAfter(m_time - op.walkStart);
                ++n;
            }
    return n;
}

std::vector<AutomationEngine::WalkView> AutomationEngine::walks() const {
    std::vector<WalkView> out;
    auto view = [&](const gait::Walker& w, double start, int motion) {
        WalkView v;
        v.root = w.legs().root;
        v.motion = motion;
        v.elapsed = std::max(0.0, m_time - start);
        v.duration = w.duration();
        v.steps = w.stepCount();
        v.stepsDone = w.stepsDone(v.elapsed);
        v.prints = w.prints(v.elapsed);
        v.footLength = w.legs().footLength;
        v.footWidth = w.legs().footWidth;
        out.push_back(std::move(v));
    };
    for (const MotionRt& m : m_motions)
        if (m.walker && m.status.state == MotionState::Running) view(*m.walker, m.walkStart, m.id);
    for (const auto& s : m_sequences)
        for (const auto& op : s.ops)
            if (op.walker && op.status.state == OpState::Running) view(*op.walker, op.walkStart, 0);
    return out;
}

// ── Runtime queries ──────────────────────────────────────────────────────────

AutomationEngine::OpStatus AutomationEngine::opStatus(int sequenceId, int operationId) const {
    for (const auto& s : m_sequences) {
        if (s.id != sequenceId) continue;
        for (const auto& o : s.ops) {
            if (o.def.id != operationId) continue;
            OpStatus st = o.status;
            // Report times relative to the cycle start
            if (st.startTime >= 0) st.startTime -= s.status.cycleStart;
            if (st.endTime >= 0)   st.endTime   -= s.status.cycleStart;
            return st;
        }
    }
    return {};
}

std::vector<int> AutomationEngine::heldObjects() const {
    std::vector<int> out;
    for (const Hold& h : m_holds) out.push_back(h.partId);
    return out;
}

int AutomationEngine::holderOf(int objectId) const {
    for (const Hold& h : m_holds)
        if (h.partId == objectId) return h.gripperId;
    return -1;
}

double AutomationEngine::beltSpeed(int objectId, double fallback) const {
    for (const BeltRt& b : m_belts)
        if (b.objectId == objectId) return b.value;
    return fallback;
}

int AutomationEngine::sensorDetection(int signalId) const {
    for (const auto& sn : m_sensors)
        if (sn.signalId == signalId) return sn.detected;
    return -1;
}

AutomationEngine::SeqStatus AutomationEngine::seqStatus(int sequenceId) const {
    for (const auto& s : m_sequences)
        if (s.id == sequenceId) return s.status;
    return {};
}

std::vector<int> AutomationEngine::movingMotors() const {
    std::vector<int> out;
    for (const auto& a : m_actuators)
        if (std::fabs(a.st.vel) > 1e-9) out.push_back(a.motorId);
    return out;
}

bool AutomationEngine::actuatedJointState(int jointId, double& pos, double& vel) const {
    for (const auto& a : m_actuators) {
        if (a.jointId != jointId) continue;
        pos = a.st.pos;
        vel = a.st.vel;
        return true;
    }
    return false;
}

QStringList AutomationEngine::activeOperationLabels() const {
    QStringList out;
    for (const auto& s : m_sequences) {
        QStringList ops;
        for (int i = 0; i < int(s.ops.size()); ++i)
            if (s.ops[size_t(i)].status.state == OpState::Running)
                ops << opLabel(s.ops[size_t(i)].def, i);
        if (!ops.isEmpty()) out << s.name + ": " + ops.join(", ");
    }
    return out;
}
