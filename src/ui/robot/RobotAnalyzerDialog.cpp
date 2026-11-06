#include "RobotAnalyzerDialog.h"
#include "RobotAnalysisPanel.h"
#include "automation/Kinematics.h"
#include "scene/SceneGraph.h"
#include "ui/Theme.h"
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

RobotAnalyzerDialog::RobotAnalyzerDialog(SceneGraph* scene, QWidget* parent) : QDialog(parent), m_scene(scene) {
    setWindowTitle("Analyze Robot");
    setMinimumSize(900, 680);
    auto* vb = new QVBoxLayout(this);
    vb->setContentsMargins(18, 16, 18, 14);
    vb->setSpacing(10);
    auto* title = new QLabel("Analyze Robot");
    title->setObjectName("dialogTitle");
    vb->addWidget(title);
    auto* sub = new QLabel("Measures the robot through its own joints: every actuated joint above the tool (TCP). "
                           "The pose, dexterity and singularity checks follow the robot while you jog it or the "
                           "simulation runs; Recompute samples the workspace again after you change the robot.");
    sub->setObjectName("hint");
    sub->setWordWrap(true);
    vb->addWidget(sub);

    auto* row = new QHBoxLayout;
    auto* tl = new QLabel("Tool");
    tl->setObjectName("fieldLabel");
    m_tools = new QComboBox;
    m_tools->setMinimumWidth(320);
    m_tools->setToolTip("The object whose frame is the tool centre point. Robots built with the Robot Builder "
                        "have one called TCP.");
    auto* again = new QPushButton(Theme::icon("refresh-cw"), "Recompute");
    m_status = new QLabel;
    m_status->setObjectName("hint");
    row->addWidget(tl);
    row->addWidget(m_tools);
    row->addWidget(again);
    row->addWidget(m_status, 1);
    vb->addLayout(row);

    m_panel = new RobotAnalysisPanel;
    vb->addWidget(m_panel, 1);

    connect(again, &QPushButton::clicked, this, [this] { fillTools(); recompute(); });
    connect(m_tools, &QComboBox::activated, this, [this](int i) {
        m_tool = m_tools->itemData(i).toInt();
        recompute();
    });
    m_poll.setInterval(100);
    connect(&m_poll, &QTimer::timeout, this, &RobotAnalyzerDialog::poll);
    connect(m_scene, &SceneGraph::sceneCleared, this, [this] { m_tool = -1; fillTools(); recompute(); });
    connect(m_scene, &SceneGraph::objectRemoved, this, [this](int) {
        if (!m_scene->getObject(m_tool)) { m_tool = -1; fillTools(); recompute(); }
    });
}

bool RobotAnalyzerDialog::analyzeObject(int objectId) {
    int tool = robot::findTool(*m_scene, objectId);
    if (tool == -1) {
        // Nothing selected (or not part of a robot): the first robot in the scene
        for (int r : m_scene->rootObjects())
            if ((tool = robot::findTool(*m_scene, r)) != -1) break;
    }
    if (tool == -1) return false;
    m_tool = tool;
    fillTools();
    recompute();
    return true;
}

void RobotAnalyzerDialog::fillTools() {
    m_tools->clear();
    auto label = [this](int id) {
        const std::vector<int> chain = kin::chainTo(*m_scene, id);
        QString robotName;
        if (!chain.empty())
            if (const Joint* j = m_scene->getJoint(chain.front()))
                if (const SceneObject* base = m_scene->getObject(j->parentBodyId)) robotName = base->name;
        return QString("%1 › %2  (%3 axes)").arg(robotName, m_scene->getObject(id)->name).arg(chain.size());
    };
    std::vector<int> ids;
    for (auto& [id, o] : m_scene->allObjects())
        if (!o.parked && o.copyOf == -1 && o.name.startsWith("TCP", Qt::CaseInsensitive) && !kin::chainTo(*m_scene, id).empty())
            ids.push_back(id);
    if (m_tool != -1 && std::find(ids.begin(), ids.end(), m_tool) == ids.end()) ids.push_back(m_tool);
    std::sort(ids.begin(), ids.end());
    for (int id : ids) m_tools->addItem(Theme::icon("crosshair"), label(id), id);
    m_tools->setCurrentIndex(m_tools->findData(m_tool));
}

void RobotAnalyzerDialog::recompute() {
    if (m_tool == -1 || !m_scene->getObject(m_tool)) {
        m_poll.stop();
        m_panel->setError("No robot here: select a robot (or its TCP) in the scene and open Analyze Robot again, "
                          "or build one with Tools › Robot Builder.");
        m_status->clear();
        return;
    }
    m_analysis = robot::analyze(*m_scene, m_tool);
    m_panel->setAnalysis(m_analysis, *m_scene);
    m_status->setText(m_analysis.error.isEmpty()
                          ? QString("%1 samples · %2 axes").arg(m_analysis.samples).arg(m_analysis.axes.size())
                          : QString());
    m_lastPose.clear();
    m_poll.start();
}

void RobotAnalyzerDialog::poll() {
    if (!isVisible() || !m_analysis.error.isEmpty() || !m_scene->getObject(m_tool)) return;
    std::vector<float> pose;
    for (const robot::AxisInfo& ax : m_analysis.axes)
        if (const Joint* j = m_scene->getJoint(ax.jointId)) pose.push_back(j->currentValue);
    if (pose.size() != m_analysis.axes.size()) { recompute(); return; }
    // The robot's base may move too (dragged with the gizmo)
    const QMatrix4x4 tool = m_scene->worldTransform(m_tool);
    for (int k = 0; k < 16; ++k) pose.push_back(tool.constData()[k]);
    if (pose == m_lastPose) return;
    m_lastPose = pose;
    robot::analyzePose(*m_scene, m_analysis);
    m_panel->setPose(m_analysis, *m_scene);
}
