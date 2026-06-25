#include "MainWindow.h"
#include "serialization/ProjectSerializer.h"
#include <QApplication>
#include <QMenuBar>
#include <QToolBar>
#include <QDockWidget>
#include <QStatusBar>
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>
#include <QKeySequence>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("MotionStudio — New Project");
    resize(1280, 800);
    setStyleSheet(
        "QMainWindow { background:#12141a; }"
        "QMenuBar { background:#1a1c22; color:#c8c8c8; }"
        "QMenuBar::item:selected { background:#2a3040; }"
        "QMenu { background:#1e2128; color:#c8c8c8; border:1px solid #444; }"
        "QMenu::item:selected { background:#2a3040; }"
        "QToolBar { background:#1a1c22; border:none; spacing:4px; padding:2px; }"
        "QToolButton { color:#c8c8c8; background:#252830; border:1px solid #3a3a3a; "
        "              padding:4px 8px; border-radius:3px; }"
        "QToolButton:hover { background:#3a4a6a; }"
        "QToolButton:pressed { background:#2a3860; }"
        "QToolButton:checked { background:#1e5080; border:1px solid #4090c0; }"
        "QDockWidget { color:#c8c8c8; }"
        "QDockWidget::title { background:#1e2128; padding:4px; }"
        "QStatusBar { background:#1a1c22; color:#888; }"
    );

    m_scene    = new SceneGraph(this);
    m_sim      = new SimulationEngine(m_scene, this);
    m_viewport = new Viewport(m_scene, this);

    setCentralWidget(m_viewport);

    buildMenu();
    buildToolbar();
    buildDocks();
    buildStatusBar();

    connect(m_sim, &SimulationEngine::stateChanged, this, &MainWindow::onSimStateChanged);
    connect(m_sim, &SimulationEngine::timeUpdated,  this, &MainWindow::onTimeUpdated);
    connect(m_sim, &SimulationEngine::stepped,      m_viewport, QOverload<>::of(&QWidget::update));
}

// ── Menu ─────────────────────────────────────────────────────────────────────

void MainWindow::buildMenu() {
    auto* file = menuBar()->addMenu("&File");
    auto mkAction = [&](QMenu* m, const QString& text, auto slot, QKeySequence::StandardKey key = QKeySequence::UnknownKey) {
        auto* a = new QAction(text, this);
        if(key != QKeySequence::UnknownKey) a->setShortcut(key);
        connect(a, &QAction::triggered, this, slot);
        m->addAction(a);
        return a;
    };
    mkAction(file, "&New Project",   &MainWindow::newProject,   QKeySequence::New);
    mkAction(file, "&Open Project…", &MainWindow::openProject,  QKeySequence::Open);
    file->addSeparator();
    mkAction(file, "&Save",          &MainWindow::saveProject,  QKeySequence::Save);
    mkAction(file, "Save &As…",      &MainWindow::saveProjectAs);
    file->addSeparator();
    {
        auto* quit = new QAction("&Quit", this);
        quit->setShortcut(QKeySequence::Quit);
        connect(quit, &QAction::triggered, qApp, &QApplication::quit);
        file->addAction(quit);
    }
}

// ── Toolbar ───────────────────────────────────────────────────────────────────

void MainWindow::buildToolbar() {
    // File toolbar
    auto* tbFile = addToolBar("File");
    tbFile->setObjectName("tbFile");
    tbFile->addAction("New",  this, &MainWindow::newProject);
    tbFile->addAction("Open", this, &MainWindow::openProject);
    tbFile->addAction("Save", this, &MainWindow::saveProject);
    tbFile->addSeparator();

    // Add objects
    tbFile->addAction("+ Cube",     this, [this]{ addObject(PrimitiveType::Cube); });
    tbFile->addAction("+ Cylinder", this, [this]{ addObject(PrimitiveType::Cylinder); });
    tbFile->addAction("+ Sphere",   this, [this]{ addObject(PrimitiveType::Sphere); });
    tbFile->addAction("+ Capsule",  this, [this]{ addObject(PrimitiveType::Capsule); });
    tbFile->addSeparator();

    // Joint / motor helpers
    tbFile->addAction("Add Joint",  this, &MainWindow::addJointToSelected);
    tbFile->addAction("Add Motor",  this, &MainWindow::addMotorToSelected);
    tbFile->addAction("Delete",     this, &MainWindow::deleteSelected);
    tbFile->addSeparator();

    // Simulation
    auto* playAction  = tbFile->addAction("▶ Play",  m_sim, &SimulationEngine::play);
    auto* pauseAction = tbFile->addAction("⏸ Pause", m_sim, &SimulationEngine::pause);
    auto* resetAction = tbFile->addAction("↺ Reset", m_sim, &SimulationEngine::reset);

    connect(m_sim, &SimulationEngine::stateChanged, [playAction, pauseAction](bool running){
        playAction->setEnabled(!running);
        pauseAction->setEnabled(running);
    });
    pauseAction->setEnabled(false);
    Q_UNUSED(resetAction);
}

// ── Docks ─────────────────────────────────────────────────────────────────────

void MainWindow::buildDocks() {
    m_hierarchy  = new HierarchyPanel(m_scene, this);
    m_properties = new PropertiesPanel(m_scene, this);

    auto* leftDock = new QDockWidget("Hierarchy", this);
    leftDock->setObjectName("leftDock");
    leftDock->setWidget(m_hierarchy);
    leftDock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
    addDockWidget(Qt::LeftDockWidgetArea, leftDock);

    auto* rightDock = new QDockWidget("Properties", this);
    rightDock->setObjectName("rightDock");
    rightDock->setWidget(m_properties);
    rightDock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
    addDockWidget(Qt::RightDockWidgetArea, rightDock);
}

// ── Status bar ────────────────────────────────────────────────────────────────

void MainWindow::buildStatusBar() {
    m_timeLabel = new QLabel("t = 0.000 s", this);
    statusBar()->addPermanentWidget(m_timeLabel);
    statusBar()->showMessage("Ready");
}

// ── File actions ──────────────────────────────────────────────────────────────

void MainWindow::newProject() {
    if(QMessageBox::question(this, "New Project",
        "Discard current project and start fresh?",
        QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) return;
    m_sim->reset();
    m_scene->clear();
    m_currentFile.clear();
    setWindowTitle("MotionStudio — New Project");
    statusBar()->showMessage("New project created");
}

void MainWindow::openProject() {
    QString path = QFileDialog::getOpenFileName(this, "Open Project", {},
        "MotionStudio Project (*.mstudio);;JSON Files (*.json)");
    if(path.isEmpty()) return;
    m_sim->reset();
    if(ProjectSerializer::load(*m_scene, path)){
        m_currentFile = path;
        setWindowTitle("MotionStudio — " + path);
        statusBar()->showMessage("Loaded: " + path);
    } else {
        QMessageBox::warning(this, "Error", "Failed to load project.");
    }
}

void MainWindow::saveProject() {
    if(m_currentFile.isEmpty()){ saveProjectAs(); return; }
    if(ProjectSerializer::save(*m_scene, m_currentFile))
        statusBar()->showMessage("Saved: " + m_currentFile);
    else
        QMessageBox::warning(this, "Error", "Failed to save project.");
}

void MainWindow::saveProjectAs() {
    QString path = QFileDialog::getSaveFileName(this, "Save Project", "project.mstudio",
        "MotionStudio Project (*.mstudio);;JSON Files (*.json)");
    if(path.isEmpty()) return;
    m_currentFile = path;
    setWindowTitle("MotionStudio — " + path);
    saveProject();
}

// ── Object actions ────────────────────────────────────────────────────────────

void MainWindow::addObject(PrimitiveType type) {
    static const char* names[] = {"Cube","Cylinder","Sphere","Capsule"};
    int parentId = m_scene->selectedObjectId();
    int id = m_scene->addObject(names[int(type)], type, parentId == -1 ? -1 : parentId);
    // Position slightly above grid if root
    if(parentId == -1){
        if(auto* obj = m_scene->getObject(id))
            obj->transform.position = {0, 0.5f, 0};
    }
    m_scene->selectObject(id);
    statusBar()->showMessage(QString("Added %1").arg(names[int(type)]));
}

void MainWindow::addJointToSelected() {
    int id = m_scene->selectedObjectId();
    if(id == -1){
        statusBar()->showMessage("Select a child object first");
        return;
    }
    SceneObject* obj = m_scene->getObject(id);
    if(!obj || obj->parentId == -1){
        statusBar()->showMessage("Object must have a parent to add a joint");
        return;
    }
    if(m_scene->jointForChild(id)){
        statusBar()->showMessage("Object already has a joint");
        return;
    }
    Joint j;
    j.name         = obj->name + "_joint";
    j.parentBodyId = obj->parentId;
    j.childBodyId  = id;
    m_scene->addJoint(j);
    statusBar()->showMessage("Joint added to " + obj->name);
}

void MainWindow::addMotorToSelected() {
    int id = m_scene->selectedObjectId();
    if(id == -1){ statusBar()->showMessage("Select an object first"); return; }
    Joint* joint = m_scene->jointForChild(id);
    if(!joint){ statusBar()->showMessage("Add a joint first"); return; }
    Motor m;
    m.name    = "Motor_" + joint->name;
    m.jointId = joint->id;
    m_scene->addMotor(m);
    statusBar()->showMessage("Motor added");
}

void MainWindow::deleteSelected() {
    int id = m_scene->selectedObjectId();
    if(id == -1){ statusBar()->showMessage("Nothing selected"); return; }
    m_scene->removeObject(id);
    statusBar()->showMessage("Deleted");
}

// ── Sim slots ─────────────────────────────────────────────────────────────────

void MainWindow::onSimStateChanged(bool running) {
    statusBar()->showMessage(running ? "Simulation running" : "Simulation paused");
}

void MainWindow::onTimeUpdated(double seconds) {
    m_timeLabel->setText(QString("t = %1 s").arg(seconds, 0, 'f', 3));
}
