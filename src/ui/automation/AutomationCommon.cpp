#include "AutomationCommon.h"
#include "physics/SimulationEngine.h"
#include <QTimer>
#include <cmath>

namespace autoui {

const char* kPageStyle =
    "QWidget        { background:#ffffff; color:#222; font-size:11px; }"
    "QGroupBox      { color:#2255aa; border:1px solid #d4dae8; border-radius:5px; "
    "                 margin-top:10px; font-weight:bold; }"
    "QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 4px; }"
    "QLabel         { background:transparent; }"
    "QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox {"
    "                 background:#f5f7fc; border:1px solid #c0c8da; border-radius:4px; "
    "                 padding:2px 5px; min-height:22px; }"
    "QLineEdit:focus, QDoubleSpinBox:focus, QComboBox:focus { border:1px solid #4480cc; background:#eef4ff; }"
    "QLineEdit:disabled, QDoubleSpinBox:disabled, QComboBox:disabled { color:#888; background:#f2f2f2; }"
    "QPushButton, QToolButton { background:#f0f4fc; color:#333; border:1px solid #b8c4da; "
    "                 border-radius:4px; padding:4px 10px; }"
    "QPushButton:hover, QToolButton:hover { background:#dce8f8; border-color:#7aaae0; color:#1040a0; }"
    "QPushButton:pressed, QToolButton:pressed { background:#c8d8f0; }"
    "QPushButton:checked { background:#1a6fda; color:#fff; border-color:#1050b0; }"
    "QPushButton:disabled, QToolButton:disabled { color:#aaa; background:#f4f4f4; border-color:#ddd; }"
    "QPushButton[danger='true'] { background:#fff0f0; color:#aa2020; border-color:#ddb0b0; }"
    "QPushButton[danger='true']:disabled { background:#f7f4f4; color:#c8b4b4; border-color:#e6dada; }"
    "QTableWidget, QTreeWidget, QListWidget { border:1px solid #d4dae8; gridline-color:#e6e9f0; "
    "                 alternate-background-color:#f7f9fd; selection-background-color:#d0e4f8; "
    "                 selection-color:#111; }"
    "QHeaderView::section { background:#eef1f6; color:#445; border:none; border-right:1px solid #d8dce6; "
    "                 border-bottom:1px solid #d0d0d0; padding:3px 6px; font-weight:bold; }"
    "QTabWidget::pane { border:none; }"
    "QTabBar::tab   { background:#e8ecf2; color:#445; padding:5px 14px; border:1px solid #d0d6e0; "
    "                 border-bottom:none; border-top-left-radius:4px; border-top-right-radius:4px; margin-right:2px; }"
    "QTabBar::tab:selected { background:#ffffff; color:#1a3a7a; font-weight:bold; }"
    "QSplitter::handle { background:#e4e8ee; }";

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

QString posUnit(const Joint* j) { return (j && j->type == JointType::Prismatic) ? " m" : "°"; }
QString velUnit(const Joint* j) { return (j && j->type == JointType::Prismatic) ? " m/s" : "°/s"; }

Page::Page(SceneGraph* scene, SimulationEngine* sim, QWidget* parent)
    : QWidget(parent), m_scene(scene), m_sim(sim)
{
    auto sched = [this] { if (!m_selfEdit) scheduleRefresh(); };
    connect(scene, &SceneGraph::automationChanged, this, sched);
    connect(scene, &SceneGraph::sceneCleared,      this, sched);
    connect(scene, &SceneGraph::jointAdded,        this, sched);
    connect(scene, &SceneGraph::objectRemoved,     this, sched);
    connect(scene, &SceneGraph::objectChanged,     this, sched);
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
