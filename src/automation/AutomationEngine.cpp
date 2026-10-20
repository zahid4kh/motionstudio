#include "AutomationEngine.h"
#include "../scene/SceneGraph.h"
#include <QSet>
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
        sym.writable = s.kind != SignalKind::Derived;
        sym.type     = s.type;
        sym.isBool   = s.type == SignalType::Bool;
        sym.help     = signalKindName(s.kind) + " " + signalTypeName(s.type) +
                       (s.description.isEmpty() ? QString() : " — " + s.description);
        out.push_back(sym);
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
            }
            chk(where + " start condition", op.startCondition);
            chkA(where + " on start", op.onStart);
            chkA(where + " on finish", op.onFinish);
            for (int p : op.predecessors)
                if (!opIds.contains(p)) errs << where + ": predecessor no longer exists";
        }
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

    m_time = 0;
    m_scanCount = 0;
    m_warnings.clear();
    publishStatus();
    m_prev = m_cur;
    m_active = true;
    return true;
}

void AutomationEngine::stop() {
    m_active = false;
    m_symbols.clear();
    m_slotIndex.clear();
    m_cur.clear();
    m_prev.clear();
    m_slotType.clear();
    m_derived.clear();
    m_actuators.clear();
    m_sequences.clear();
    m_actIndexByMotor.clear();
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

    // 3. Sequences
    for (auto& s : m_sequences) advanceSequence(s);

    // 4. Bindings
    applyBindings();

    // 5. Motion
    integrate(dt);

    m_time += dt;
    ++m_scanCount;
    emit scanned();
}

void AutomationEngine::publishStatus() {
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
            const Pose* pose = m_scene->getPose(op.def.poseId);
            if (!pose) break;
            double pct = op.velocity.isEmpty() ? 100.0 : std::max(0.1, op.velocity.eval(c));
            // Synchronized point-to-point: every axis uses the slowest axis's
            // profile scaled by its own distance, so all arrive together.
            double leaderT = 0, leaderV = 0, leaderA = 0, leaderD = 0;
            for (auto& [jid, val] : pose->jointValues) {
                const Motor* m = m_scene->motorForJoint(jid);
                if (!m) continue;
                auto it = m_actIndexByMotor.find(m->id);
                if (it == m_actIndexByMotor.end()) continue;
                ActuatorRt& ax = m_actuators[size_t(it->second)];
                double tgt = clampToLimits(ax, val);
                double d   = std::fabs(tgt - ax.st.pos);
                double v   = std::fabs(ax.defVelocity) * pct / 100.0;
                double acc = ax.defAccel * pct / 100.0;
                double T   = motion::moveTime(d, v, acc);
                if (T > leaderT) { leaderT = T; leaderV = v; leaderA = acc; leaderD = d; }
                op.poseTargets.emplace_back(it->second, tgt);
                op.poseFrom.push_back(ax.st.pos);
            }
            for (auto& [idx, tgt] : op.poseTargets) {
                ActuatorRt& ax = m_actuators[size_t(idx)];
                double d = std::fabs(tgt - ax.st.pos);
                double k = leaderD > 0 ? d / leaderD : 0.0;
                ax.mode     = ActuatorMode::Position;
                ax.target   = tgt;
                ax.velocity = std::max(1e-6, leaderV * k);
                ax.accel    = leaderA * k;
                ax.active   = true;
            }
            op.status.detail = QString("→ %1").arg(pose->name);
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
            double worst = 1.0;
            bool all = true;
            for (size_t i = 0; i < op.poseTargets.size(); ++i) {
                const ActuatorRt& ax = m_actuators[size_t(op.poseTargets[i].first)];
                double tgt  = op.poseTargets[i].second;
                double span = std::fabs(tgt - op.poseFrom[i]);
                if (span > 0) worst = std::min(worst, 1.0 - std::fabs(tgt - ax.st.pos) / span);
                if (!inPosition(ax, tgt)) all = false;
            }
            op.status.progress = worst;
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
    }
    return true;
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
