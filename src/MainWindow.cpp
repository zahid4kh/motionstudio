#include "MainWindow.h"
#include "serialization/ProjectSerializer.h"
#include "import/StepImporter.h"
#include "ui/TemplateDialog.h"
#include <QApplication>
#include <QMenuBar>
#include <QToolBar>
#include <QDockWidget>
#include <QStatusBar>
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>
#include <QKeySequence>
#include <QFileInfo>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QGuiApplication>
#include <QCursor>
#include <QUrl>
#include <QtConcurrent/QtConcurrentRun>
#include <functional>

// ── Style constants ───────────────────────────────────────────────────────────

static const char* kWinStyle =
    "QMainWindow   { background:#f0f2f5; }"
    "QMenuBar       { background:#ffffff; color:#222; border-bottom:1px solid #d0d0d0; font-size:12px; }"
    "QMenuBar::item:selected { background:#e0eaf8; }"
    "QMenu          { background:#ffffff; color:#222; border:1px solid #c8c8c8; font-size:12px; }"
    "QMenu::item:selected    { background:#d0e4f8; }"
    "QDockWidget    { color:#222; font-size:12px; }"
    "QDockWidget::title { background:#e8ecf2; padding:5px 8px; color:#333; font-weight:bold; border-bottom:1px solid #d0d0d0; }"
    "QStatusBar     { background:#f8f9fc; color:#555; border-top:1px solid #d8d8d8; font-size:11px; padding:2px 6px; }";

static const char* kToolbarStyle =
    "QToolBar { background:#ffffff; border-bottom:1px solid #d8d8d8; spacing:3px; padding:4px 6px; }"
    "QToolButton { color:#333; background:#f4f6fb; border:1px solid #c8cdd8; "
    "              padding:5px 11px; border-radius:5px; font-size:12px; min-width:32px; }"
    "QToolButton:hover   { background:#ddeafc; border-color:#7aaae8; color:#1040a0; }"
    "QToolButton:pressed { background:#c4d8f8; border-color:#5588cc; }"
    "QToolButton:checked { background:#1a6fda; color:#fff; border-color:#1050b0; }"
    "QToolButton:disabled { color:#aaa; background:#f0f0f0; border-color:#ddd; }";

// ── Constructor ───────────────────────────────────────────────────────────────

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle("MotionStudio — New Project");
    resize(1280, 800);
    setStyleSheet(kWinStyle);
    setAcceptDrops(true);

    m_scene    = new SceneGraph(this);
    m_undo     = new UndoManager(m_scene, this);
    m_sim      = new SimulationEngine(m_scene, this);
    m_viewport = new Viewport(m_scene, this);

    setCentralWidget(m_viewport);

    buildMenu();
    buildToolbar();
    buildDocks();
    buildStatusBar();

    // Dirty-tracking
    connect(m_scene, &SceneGraph::objectAdded,   this, &MainWindow::onSceneDirtied);
    connect(m_scene, &SceneGraph::objectRemoved, this, &MainWindow::onSceneDirtied);
    connect(m_scene, &SceneGraph::objectChanged, this, &MainWindow::onSceneDirtied);
    connect(m_scene, &SceneGraph::jointAdded,    this, &MainWindow::onSceneDirtied);
    connect(m_scene, &SceneGraph::motorAdded,    this, &MainWindow::onSceneDirtied);

    connect(m_undo, &UndoManager::historyChanged, this, [this]{
        m_undoAction->setEnabled(m_undo->canUndo());
        m_redoAction->setEnabled(m_undo->canRedo());
    });

    connect(m_sim, &SimulationEngine::stateChanged, this, &MainWindow::onSimStateChanged);
    connect(m_sim, &SimulationEngine::timeUpdated,  this, &MainWindow::onTimeUpdated);
    connect(m_sim, &SimulationEngine::stepped,      m_viewport, QOverload<>::of(&QWidget::update));

    m_statusTimer.setSingleShot(true);
    connect(&m_statusTimer, &QTimer::timeout, this, [this](){
        statusBar()->setStyleSheet(
            "QStatusBar { background:#f8f9fc; color:#555; border-top:1px solid #d8d8d8; font-size:11px; padding:2px 6px; }");
        statusBar()->clearMessage();
    });
}

// ── Status helper ─────────────────────────────────────────────────────────────

void MainWindow::showStatus(const QString& msg, bool success) {
    QString bg   = success ? "#f0fff4" : "#fff2f2";
    QString fg   = success ? "#1a7a30" : "#b01020";
    QString bdr  = success ? "#b0ddb8" : "#f0b0b0";
    statusBar()->setStyleSheet(QString(
        "QStatusBar { background:%1; color:%2; border-top:1px solid %3; font-size:11px; padding:2px 6px; }"
    ).arg(bg, fg, bdr));
    statusBar()->showMessage(msg);
    m_statusTimer.start(4000);
}

void MainWindow::updateTitle() {
    QString base = "MotionStudio";
    if(!m_currentFile.isEmpty())
        base += " — " + QFileInfo(m_currentFile).fileName();
    else
        base += " — New Project";
    if(m_dirty) base += "  •";
    setWindowTitle(base);
}

void MainWindow::onSceneDirtied() {
    if(!m_dirty) { m_dirty = true; updateTitle(); }
}

// ── Menu ──────────────────────────────────────────────────────────────────────

void MainWindow::buildMenu() {
    auto* file = menuBar()->addMenu("&File");
    auto mkA = [&](QMenu* m, const QString& txt, auto slot, QKeySequence::StandardKey k = QKeySequence::UnknownKey){
        auto* a = new QAction(txt, this);
        if(k != QKeySequence::UnknownKey) a->setShortcut(k);
        connect(a, &QAction::triggered, this, slot);
        m->addAction(a);
        return a;
    };
    mkA(file, "&New Project",    &MainWindow::newProject,   QKeySequence::New);
    mkA(file, "&Open Project…", &MainWindow::openProject,  QKeySequence::Open);
    mkA(file, "Open &Template…",&MainWindow::openTemplate);
    file->addSeparator();
    mkA(file, "&Import STEP…",  &MainWindow::importStep);
    file->addSeparator();
    mkA(file, "&Save",          &MainWindow::saveProject,  QKeySequence::Save);
    mkA(file, "Save &As…",      &MainWindow::saveProjectAs);
    file->addSeparator();
    {
        auto* q = new QAction("&Quit", this);
        q->setShortcut(QKeySequence::Quit);
        connect(q, &QAction::triggered, qApp, &QApplication::quit);
        file->addAction(q);
    }

    auto* edit = menuBar()->addMenu("&Edit");
    m_undoAction = new QAction("&Undo", this);
    m_undoAction->setShortcut(QKeySequence::Undo);
    m_undoAction->setEnabled(false);
    connect(m_undoAction, &QAction::triggered, m_undo, &UndoManager::undo);
    edit->addAction(m_undoAction);

    m_redoAction = new QAction("&Redo", this);
    m_redoAction->setShortcut(QKeySequence::Redo);
    m_redoAction->setEnabled(false);
    connect(m_redoAction, &QAction::triggered, m_undo, &UndoManager::redo);
    edit->addAction(m_redoAction);
}

// ── Toolbar ───────────────────────────────────────────────────────────────────

static QAction* mkBtn(QToolBar* tb, const QString& text, const QString& tip) {
    auto* a = tb->addAction(text);
    a->setToolTip(tip);
    return a;
}

void MainWindow::buildToolbar() {
    auto* tb = addToolBar("Main");
    tb->setObjectName("mainToolbar");
    tb->setMovable(false);
    tb->setStyleSheet(kToolbarStyle);

    // ── File ──
    connect(mkBtn(tb,"New", "New project  (Ctrl+N)"),  &QAction::triggered, this, &MainWindow::newProject);
    connect(mkBtn(tb,"Open","Open project  (Ctrl+O)"), &QAction::triggered, this, &MainWindow::openProject);
    connect(mkBtn(tb,"Save","Save project  (Ctrl+S)"), &QAction::triggered, this, &MainWindow::saveProject);
    tb->addSeparator();

    // ── Add primitives ──
    connect(mkBtn(tb,"+ Cube",    "Add Cube\nAdded as child of selection"),     &QAction::triggered, this, [this]{ addObject(PrimitiveType::Cube);     });
    connect(mkBtn(tb,"+ Cylinder","Add Cylinder\nAdded as child of selection"), &QAction::triggered, this, [this]{ addObject(PrimitiveType::Cylinder); });
    connect(mkBtn(tb,"+ Sphere",  "Add Sphere\nAdded as child of selection"),   &QAction::triggered, this, [this]{ addObject(PrimitiveType::Sphere);   });
    connect(mkBtn(tb,"+ Capsule", "Add Capsule\nAdded as child of selection"),  &QAction::triggered, this, [this]{ addObject(PrimitiveType::Capsule);  });
    connect(mkBtn(tb,"+ Basket",  "Add Basket\nOpen-top container — add as root to drop objects into"), &QAction::triggered, this, [this]{ addObject(PrimitiveType::Basket);  });
    tb->addSeparator();

    // ── Assembly ──
    connect(mkBtn(tb,"Add Joint","Add Revolute joint\nObject must have a parent"), &QAction::triggered, this, &MainWindow::addJointToSelected);
    connect(mkBtn(tb,"Add Motor","Add Motor to joint\nObject must have a joint"),  &QAction::triggered, this, &MainWindow::addMotorToSelected);
    connect(mkBtn(tb,"Delete",   "Delete selected  (Del)"),                        &QAction::triggered, this, &MainWindow::deleteSelected);
    tb->addSeparator();

    // ── Simulation ──
    auto* playA  = mkBtn(tb, "▶  Play",  "Start simulation  (Space)");
    auto* pauseA = mkBtn(tb, "⏸  Pause", "Pause simulation  (Space)");
    auto* resetA = mkBtn(tb, "↺  Reset", "Stop and reset to t=0");
    connect(playA,  &QAction::triggered, m_sim, &SimulationEngine::play);
    connect(pauseA, &QAction::triggered, m_sim, &SimulationEngine::pause);
    connect(resetA, &QAction::triggered, m_sim, &SimulationEngine::reset);
    Q_UNUSED(resetA);

    connect(m_sim, &SimulationEngine::stateChanged, [playA, pauseA](bool running){
        playA->setEnabled(!running);
        pauseA->setEnabled(running);
    });
    pauseA->setEnabled(false);

    // Spacebar shortcut for play/pause toggle
    auto* spaceShortcut = new QAction(this);
    spaceShortcut->setShortcut(Qt::Key_Space);
    connect(spaceShortcut, &QAction::triggered, this, [this](){
        m_sim->isRunning() ? m_sim->pause() : m_sim->play();
    });
    addAction(spaceShortcut);

    // Delete key shortcut
    auto* delShortcut = new QAction(this);
    delShortcut->setShortcut(Qt::Key_Delete);
    connect(delShortcut, &QAction::triggered, this, &MainWindow::deleteSelected);
    addAction(delShortcut);
}

// ── Docks ─────────────────────────────────────────────────────────────────────

void MainWindow::buildDocks() {
    m_hierarchy  = new HierarchyPanel(m_scene, this);
    m_properties = new PropertiesPanel(m_scene, this);

    auto mkDock = [&](const QString& title, QWidget* w, Qt::DockWidgetArea area){
        auto* d = new QDockWidget(title, this);
        d->setWidget(w);
        d->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
        d->setMinimumWidth(200);
        addDockWidget(area, d);
        return d;
    };
    mkDock("Hierarchy",  m_hierarchy,  Qt::LeftDockWidgetArea);
    mkDock("Properties", m_properties, Qt::RightDockWidgetArea);

    connect(m_viewport, &Viewport::keyPointSelected,
            m_properties, &PropertiesPanel::showPointCoords);

    connect(m_properties, &PropertiesPanel::pickAxisRequested, this, [this]{
        int id = m_scene->selectedObjectId();
        if (id != -1) m_viewport->beginAxisPick(id);
    });
    connect(m_viewport, &Viewport::axisPicked, this, [this](int objectId, const QVector3D& localAxis){
        Joint* j = m_scene->jointForChild(objectId);
        if (!j) return;
        j->axis = localAxis;
        emit m_scene->objectChanged(objectId);
    });
}

// ── Status bar ────────────────────────────────────────────────────────────────

void MainWindow::buildStatusBar() {
    m_timeLabel = new QLabel("t = 0.000 s", this);
    m_timeLabel->setStyleSheet("color:#666; font-size:11px; padding-right:8px;");
    statusBar()->addPermanentWidget(m_timeLabel);
    statusBar()->showMessage("Ready — open a project or add objects to get started");
}

// ── File actions ──────────────────────────────────────────────────────────────

void MainWindow::newProject() {
    if(m_dirty && QMessageBox::question(this, "New Project",
        "You have unsaved changes. Discard and start fresh?",
        QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) return;
    m_sim->reset();
    m_scene->clear();
    m_currentFile.clear();
    m_dirty = false;
    updateTitle();
    m_undo->reset();
    showStatus("New project created");
}

void MainWindow::openProject() {
    if(m_dirty && QMessageBox::question(this, "Open Project",
        "You have unsaved changes. Discard and open another project?",
        QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) return;

    QString path = QFileDialog::getOpenFileName(this, "Open Project", {},
        "MotionStudio Project (*.mstudio);;JSON Files (*.json);;All Files (*)");
    if(path.isEmpty()) return;
    m_sim->reset();
    if(ProjectSerializer::load(*m_scene, path)){
        m_currentFile = path;
        m_dirty = false;
        updateTitle();
        m_undo->reset();
        showStatus("Opened: " + QFileInfo(path).fileName());
    } else {
        QMessageBox::critical(this, "Load Failed",
            "Could not load project from:\n" + path +
            "\n\nThe file may be corrupt or incompatible.");
        showStatus("Failed to open project", false);
    }
}

void MainWindow::openTemplate() {
    if(m_dirty && QMessageBox::question(this, "Open Template",
        "You have unsaved changes. Discard and load a template?",
        QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) return;

    TemplateDialog dlg(this);
    if(dlg.exec() != QDialog::Accepted) return;

    QString path = dlg.selectedPath();
    if(path.isEmpty()) return;

    m_sim->reset();
    if(ProjectSerializer::load(*m_scene, path)){
        m_currentFile.clear();
        m_dirty = false;
        updateTitle();
        m_undo->reset();
        showStatus("Template loaded — press Play to run the simulation");
    } else {
        showStatus("Failed to load template", false);
    }
}

void MainWindow::saveProject() {
    if(m_currentFile.isEmpty()){ saveProjectAs(); return; }
    if(ProjectSerializer::save(*m_scene, m_currentFile)){
        m_dirty = false;
        updateTitle();
        showStatus("Saved: " + QFileInfo(m_currentFile).fileName());
    } else {
        QMessageBox::critical(this, "Save Failed",
            "Could not save to:\n" + m_currentFile);
        showStatus("Save failed — check file permissions", false);
    }
}

void MainWindow::saveProjectAs() {
    QString path = QFileDialog::getSaveFileName(this, "Save Project As", "project.mstudio",
        "MotionStudio Project (*.mstudio);;JSON Files (*.json)");
    if(path.isEmpty()) return;
    m_currentFile = path;
    saveProject();
}

// ── STEP import ───────────────────────────────────────────────────────────────

static bool isStepPath(const QString& path) {
    QString s = QFileInfo(path).suffix().toLower();
    return s == "step" || s == "stp";
}

void MainWindow::importStep() {
    QString path = QFileDialog::getOpenFileName(this, "Import STEP", {},
        "STEP Files (*.step *.stp *.STEP *.STP);;All Files (*)");
    if(path.isEmpty()) return;
    importStepFile(path);
}

int MainWindow::addImportedPart(const StepImporter::Part& part, int parentId) {
    QString name = part.name.isEmpty() ? "Part" : part.name;
    int id = m_scene->addObject(name, PrimitiveType::ImportedMesh, parentId);
    if(SceneObject* obj = m_scene->getObject(id)){
        obj->transform.position = part.position;
        obj->transform.rotation = part.rotationDeg;
        obj->transform.scale    = {1, 1, 1};
        obj->mesh               = part.mesh;
    }
    for(const auto& child : part.children)
        addImportedPart(child, id);
    return id;
}

void MainWindow::importStepFile(const QString& path) {
    if(!StepImporter::available()){
        QMessageBox::warning(this, "STEP Import Unavailable",
            "MotionStudio was built without OpenCASCADE.\n"
            "Install the libocct-*-dev packages and rebuild to enable STEP import.");
        return;
    }
    if(m_stepWatcher){
        showStatus("A STEP import is already in progress", false);
        return;
    }

    m_stepImportName = QFileInfo(path).fileName();

    // Large assemblies can take a minute-plus to triangulate (OCCT is CPU-bound
    // and single-threaded per shape) — run it off the GUI thread so the window
    // doesn't appear to hang, with a modal progress dialog since there's nothing
    // useful to do until it finishes.
    m_stepProgress = new QProgressDialog("Importing " + m_stepImportName + "…", QString(), 0, 0, this);
    m_stepProgress->setWindowTitle("STEP Import");
    m_stepProgress->setWindowModality(Qt::WindowModal);
    m_stepProgress->setMinimumDuration(0);
    m_stepProgress->setCancelButton(nullptr);  // OCCT translation can't be safely interrupted mid-flight
    m_stepProgress->setValue(0);
    m_stepProgress->show();

    m_stepWatcher = new QFutureWatcher<StepImportResult>(this);
    connect(m_stepWatcher, &QFutureWatcher<StepImportResult>::finished,
            this, &MainWindow::onStepImportFinished);

    QFuture<StepImportResult> future = QtConcurrent::run([path]() -> StepImportResult {
        StepImportResult r;
        r.ok = StepImporter::importFile(path, r.root, r.error);
        return r;
    });
    m_stepWatcher->setFuture(future);
}

void MainWindow::onStepImportFinished() {
    StepImportResult result = m_stepWatcher->result();

    m_stepProgress->close();
    m_stepProgress->deleteLater();
    m_stepProgress = nullptr;
    m_stepWatcher->deleteLater();
    m_stepWatcher = nullptr;

    if(!result.ok){
        QMessageBox::critical(this, "STEP Import Failed",
            "Could not import:\n" + m_stepImportName + "\n\n" + result.error);
        showStatus("STEP import failed", false);
        return;
    }

    int rootId = addImportedPart(result.root, -1);
    m_scene->selectObject(rootId);

    // Count leaf parts for the status message
    int partCount = 0;
    std::function<void(const StepImporter::Part&)> count = [&](const StepImporter::Part& p){
        if(p.mesh) partCount++;
        for(const auto& c : p.children) count(c);
    };
    count(result.root);
    showStatus(QString("Imported %1  —  %2 part%3")
        .arg(m_stepImportName).arg(partCount).arg(partCount == 1 ? "" : "s"));
}

// ── Drag & drop ───────────────────────────────────────────────────────────────

void MainWindow::dragEnterEvent(QDragEnterEvent* e) {
    if(!e->mimeData()->hasUrls()) return;
    for(const QUrl& url : e->mimeData()->urls()){
        if(url.isLocalFile() && isStepPath(url.toLocalFile())){
            e->acceptProposedAction();
            return;
        }
    }
}

void MainWindow::dropEvent(QDropEvent* e) {
    for(const QUrl& url : e->mimeData()->urls()){
        if(url.isLocalFile() && isStepPath(url.toLocalFile()))
            importStepFile(url.toLocalFile());
    }
    e->acceptProposedAction();
}

// ── Object actions ────────────────────────────────────────────────────────────

void MainWindow::addObject(PrimitiveType type) {
    static const char* names[] = {"Cube","Cylinder","Sphere","Capsule","Basket"};
    int parentId = m_scene->selectedObjectId();
    int id = m_scene->addObject(names[int(type)], type, parentId == -1 ? -1 : parentId);
    if(parentId == -1){
        if(auto* obj = m_scene->getObject(id))
            obj->transform.position = {0, 0.5f, 0};
    }
    m_scene->selectObject(id);
    showStatus(QString("Added %1").arg(names[int(type)]));
}

void MainWindow::addJointToSelected() {
    int id = m_scene->selectedObjectId();
    if(id == -1){
        showStatus("Select a child object first, then click Add Joint", false);
        return;
    }
    SceneObject* obj = m_scene->getObject(id);
    if(!obj || obj->parentId == -1){
        showStatus("Object must have a parent to add a joint — select a child object", false);
        return;
    }
    if(m_scene->jointForChild(id)){
        showStatus("This object already has a joint — edit it in the Properties panel", false);
        return;
    }
    Joint j;
    j.name         = obj->name + "_joint";
    j.parentBodyId = obj->parentId;
    j.childBodyId  = id;
    m_scene->addJoint(j);
    showStatus("Joint added to " + obj->name + "  — configure it in Properties");
}

void MainWindow::addMotorToSelected() {
    int id = m_scene->selectedObjectId();
    if(id == -1){ showStatus("Select an object first", false); return; }
    Joint* joint = m_scene->jointForChild(id);
    if(!joint){ showStatus("Add a joint to this object first", false); return; }
    Motor m;
    m.name    = "Motor_" + joint->name;
    m.jointId = joint->id;
    m_scene->addMotor(m);
    showStatus("Motor added — set velocity in Properties, then press Play");
}

void MainWindow::deleteSelected() {
    int id = m_scene->selectedObjectId();
    if(id == -1){ showStatus("Nothing selected", false); return; }
    QString name = m_scene->getObject(id) ? m_scene->getObject(id)->name : "object";
    m_scene->removeObject(id);
    showStatus("Deleted " + name);
}

// ── Sim slots ─────────────────────────────────────────────────────────────────

void MainWindow::onSimStateChanged(bool running) {
    showStatus(running ? "Simulation running  —  Space to pause" : "Simulation paused");
}

void MainWindow::onTimeUpdated(double seconds) {
    m_timeLabel->setText(QString("t = %1 s").arg(seconds, 0, 'f', 3));
}
