#include "AutomationPanel.h"
#include "ActuatorsPage.h"
#include "RuntimePage.h"
#include "SequencesPage.h"
#include "SignalsPage.h"
#include "physics/SimulationEngine.h"
#include "ui/HelpDialog.h"
#include "ui/Theme.h"
#include <QHBoxLayout>
#include <QLabel>
#include <QMenu>
#include <QTabBar>
#include <QTabWidget>
#include <QToolButton>
#include <QVBoxLayout>

AutomationPanel::AutomationPanel(SceneGraph* scene, SimulationEngine* sim, QWidget* parent)
    : QWidget(parent), m_scene(scene), m_sim(sim)
{
    auto* vb = new QVBoxLayout(this);
    vb->setContentsMargins(0, 0, 0, 0);
    vb->setSpacing(0);

    m_tabs = new QTabWidget;
    m_tabs->setDocumentMode(true);
    m_tabs->tabBar()->setDrawBase(false);
    m_signals   = new SignalsPage(scene, sim);
    m_actuators = new ActuatorsPage(scene, sim);
    m_sequences = new SequencesPage(scene, sim);
    m_runtime   = new RuntimePage(scene, sim);
    m_tabs->addTab(m_signals,   Theme::icon("toggle-right"), "Signals");
    m_tabs->addTab(m_actuators, Theme::icon("cog"),          "Actuators && Poses");
    m_tabs->addTab(m_sequences, Theme::icon("list-ordered"), "Sequences");
    m_tabs->addTab(m_runtime,   Theme::icon("activity"),     "Runtime");
    m_tabs->setIconSize(QSize(15, 15));

    // Corner: lock indicator + problems button
    auto* corner = new QWidget;
    auto* ch = new QHBoxLayout(corner);
    ch->setContentsMargins(0, 2, 8, 2);
    ch->setSpacing(6);
    auto* lockIcon = new QLabel;
    Theme::setLabelIcon(lockIcon, "lock", 13, Theme::Tint::Warning);
    m_lockLabel = new QLabel("Definitions locked while simulating — Reset to edit");
    m_lockLabel->setObjectName("warningText");
    m_lockLabel->hide();
    lockIcon->hide();
    m_lockIcon = lockIcon;
    m_problems = new QToolButton;
    m_problems->setObjectName("problems");
    m_problems->setPopupMode(QToolButton::InstantPopup);
    m_problems->setAutoRaise(true);
    m_problems->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    m_problems->setIconSize(QSize(14, 14));
    ch->addWidget(lockIcon);
    ch->addWidget(m_lockLabel);
    ch->addSpacing(6);
    auto* guide = new QToolButton;
    guide->setObjectName("problems");
    guide->setText("Guide");
    guide->setIcon(Theme::icon("circle-help"));
    guide->setIconSize(QSize(14, 14));
    guide->setAutoRaise(true);
    guide->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    guide->setToolTip("How to use this tab, with examples to copy  (F1 for the full guide)");
    connect(guide, &QToolButton::clicked, this, [this]{
        QWidget* tab = m_tabs->currentWidget();
        HelpDialog::showTopic(tab == m_signals   ? "signals"
                            : tab == m_sequences ? "sequences"
                            : tab == m_runtime   ? "runtime"
                                                 : "actuators", this);
    });
    ch->addWidget(guide);
    ch->addWidget(m_problems);
    m_tabs->setCornerWidget(corner, Qt::TopRightCorner);
    vb->addWidget(m_tabs);

    for (autoui::Page* p : std::initializer_list<autoui::Page*>{m_signals, m_actuators, m_sequences, m_runtime})
        connect(p, &autoui::Page::statusMessage, this, &AutomationPanel::statusMessage);

    // Live values at ~20 Hz while active
    m_runtimeTimer.setInterval(50);
    connect(&m_runtimeTimer, &QTimer::timeout, this, [this]{
        if (!m_sim->automation()->isActive()) return;
        if (auto* p = qobject_cast<autoui::Page*>(m_tabs->currentWidget())) p->refreshRuntime();
    });
    m_runtimeTimer.start();
    // Step (and Reset) should update immediately even when paused
    connect(m_sim, &SimulationEngine::stepped, this, [this]{
        if (m_sim->isRunning()) return;
        if (auto* p = qobject_cast<autoui::Page*>(m_tabs->currentWidget())) p->refreshRuntime();
    });
    connect(m_tabs, &QTabWidget::currentChanged, this, [this]{
        if (auto* p = qobject_cast<autoui::Page*>(m_tabs->currentWidget())) p->refreshRuntime();
    });
    connect(scene, &SceneGraph::jointsMoved, m_actuators, &ActuatorsPage::refreshRuntime);
    connect(m_actuators, &ActuatorsPage::sequenceCreated, this, [this](int id){
        m_sequences->selectSequence(id);
        m_tabs->setCurrentWidget(m_sequences);
    });
    connect(sim, &SimulationEngine::activeChanged, this, &AutomationPanel::setLocked);

    // Problems: re-validate shortly after definitions change
    m_validateTimer.setSingleShot(true);
    m_validateTimer.setInterval(250);
    connect(&m_validateTimer, &QTimer::timeout, this, &AutomationPanel::revalidate);
    auto kick = [this]{ m_validateTimer.start(); };
    connect(scene, &SceneGraph::automationChanged, this, kick);
    connect(scene, &SceneGraph::sceneCleared,      this, kick);
    connect(scene, &SceneGraph::objectRemoved,     this, kick);
    revalidate();
}

void AutomationPanel::setLocked(bool locked) {
    m_lockLabel->setVisible(locked);
    m_lockIcon->setVisible(locked);
    for (autoui::Page* p : std::initializer_list<autoui::Page*>{m_signals, m_actuators, m_sequences, m_runtime})
        p->setLocked(locked);
}

void AutomationPanel::revalidate() {
    m_problemList = AutomationEngine::validateAll(*m_scene);
    auto* menu = new QMenu(m_problems);
    if (m_problemList.isEmpty()) {
        m_problems->setText("No problems");
        m_problems->setIcon(Theme::icon("circle-check", Theme::Tint::Success));
        m_problems->setProperty("tone", "ok");
        menu->addAction("All expressions and references are valid")->setEnabled(false);
    } else {
        m_problems->setText(QString("%1 problem%2").arg(m_problemList.size()).arg(m_problemList.size() == 1 ? "" : "s"));
        m_problems->setIcon(Theme::icon("triangle-alert", Theme::Tint::Danger));
        m_problems->setProperty("tone", "bad");
        for (const QString& p : m_problemList) menu->addAction(p)->setEnabled(false);
    }
    Theme::repolish(m_problems);
    if (QMenu* old = m_problems->menu()) old->deleteLater();
    m_problems->setMenu(menu);
}

void AutomationPanel::showRuntime() { m_tabs->setCurrentWidget(m_runtime); }

void AutomationPanel::showProblems(const QStringList&) {
    revalidate();
    m_problems->showMenu();
}

void AutomationPanel::showActuatorForJoint(int jointId) {
    m_tabs->setCurrentWidget(m_actuators);
    m_actuators->selectActuatorForJoint(jointId);
}
