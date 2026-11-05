#include "AutomationCommon.h"
#include "physics/SimulationEngine.h"
#include "ui/Theme.h"
#include <QComboBox>
#include <QTimer>
#include <functional>
#include <cmath>

namespace autoui {

ExpressionEdit* makeExprEdit(SceneGraph* scene, ExprKind kind, const QString& placeholder, QWidget* parent) {
    auto* e = new ExpressionEdit(parent);
    e->setPlaceholderText(placeholder);
    if (kind == ExprKind::Value)
        e->setValidator([scene](const QString& s) { return AutomationEngine::validateExpression(*scene, s); });
    else
        e->setValidator([scene](const QString& s) { return AutomationEngine::validateAssignments(*scene, s); });
    e->setWordSource([scene] { return AutomationEngine::symbolNames(*scene); });
    return e;
}

QString formatValue(double v, bool isBool, SignalType type) {
    if (isBool || type == SignalType::Bool) return v != 0.0 ? "true" : "false";
    if (type == SignalType::Int) return QString::number(qint64(std::llround(v)));
    double a = std::fabs(v);
    int dec = a >= 1000 ? 1 : (a >= 10 ? 2 : 3);
    return QString::number(v, 'f', dec);
}

QBrush boolBrush(bool on) {
    const Theme::Colors& c = Theme::colors();
    return on ? Theme::soft(c.success, 0.26) : Theme::soft(c.muted, 0.10);
}

QBrush opStateBrush(AutomationEngine::OpState state) {
    const Theme::Colors& c = Theme::colors();
    switch (state) {
        case AutomationEngine::OpState::Running: return Theme::soft(c.info, 0.24);
        case AutomationEngine::OpState::Done:    return Theme::soft(c.success, 0.22);
        case AutomationEngine::OpState::Waiting: break;
    }
    return Theme::soft(c.muted, 0.10);
}

QString posUnit(const Joint* j) { return (j && j->type == JointType::Prismatic) ? " m" : "°"; }
QString velUnit(const Joint* j) { return (j && j->type == JointType::Prismatic) ? " m/s" : "°/s"; }

void fillObjectCombo(QComboBox* combo, const SceneGraph& scene, int currentId, const QString& noneLabel) {
    const QSignalBlocker block(combo);
    combo->clear();
    if (!noneLabel.isEmpty()) combo->addItem(noneLabel, -1);
    std::function<void(int, int)> add = [&](int id, int depth) {
        const SceneObject* o = scene.getObject(id);
        if (!o || o->copyOf != -1) return;   // copies made while simulating
        combo->addItem(QString(depth * 3, QChar(' ')) + o->name, id);
        for (int c : o->childIds) add(c, depth + 1);
    };
    for (int r : scene.rootObjects()) add(r, 0);
    int idx = combo->findData(currentId);
    if (idx < 0 && currentId != -1) {
        combo->addItem("(deleted object)", currentId);
        idx = combo->count() - 1;
    }
    combo->setCurrentIndex(std::max(0, idx));
}

Page::Page(SceneGraph* scene, SimulationEngine* sim, QWidget* parent)
    : QWidget(parent), m_scene(scene), m_sim(sim)
{
    auto sched = [this] { if (!m_selfEdit) scheduleRefresh(); };
    connect(scene, &SceneGraph::automationChanged, this, sched);
    connect(scene, &SceneGraph::sceneCleared,      this, sched);
    connect(scene, &SceneGraph::jointAdded,        this, sched);
    connect(scene, &SceneGraph::objectRemoved,     this, sched);
    connect(scene, &SceneGraph::objectChanged,     this, sched);
    // Item backgrounds and rich text bake theme colours in: rebuild on a switch
    connect(Theme::notifier(), &Theme::Notifier::changed, this, [this] { scheduleRefresh(); });
}

void Page::scheduleRefresh() {
    if (m_pending) return;
    m_pending = true;
    QTimer::singleShot(0, this, [this] {
        m_pending = false;
        refresh();
    });
}

} // namespace autoui
