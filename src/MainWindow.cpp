#include "MainWindow.h"
#include "serialization/ProjectSerializer.h"
#include "import/StepImporter.h"
#include "ui/TemplateDialog.h"
#include "ui/Island.h"
#include "ui/Theme.h"
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
#include <QComboBox>
#include <QActionGroup>
#include <QHBoxLayout>
#include <QToolButton>
#include <QVBoxLayout>
#include <QSet>
#include <functional>

// ── Constructor ───────────────────────────────────────────────────────────────

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle("MotionStudio — New Project");
    resize(1440, 900);
    setAcceptDrops(true);
    setDockOptions(QMainWindow::AnimatedDocks | QMainWindow::AllowTabbedDocks);
    // The frame around the islands (docks + viewport); the top edge is the menu widget's
    setContentsMargins(8, 0, 8, 0);

    m_scene    = new SceneGraph(this);
    m_undo     = new UndoManager(m_scene, this);
    m_sim      = new SimulationEngine(m_scene, this);
    m_viewport = new Viewport(m_scene);

    setCentralWidget(new Island(m_viewport, this));

    buildMenu();
    buildToolbar();
    buildDocks();
    buildViewMenu();
    buildStatusBar();

    // Dirty-tracking
    connect(m_scene, &SceneGraph::objectAdded,   this, &MainWindow::onSceneDirtied);
    connect(m_scene, &SceneGraph::objectRemoved, this, &MainWindow::onSceneDirtied);
    connect(m_scene, &SceneGraph::objectChanged, this, &MainWindow::onSceneDirtied);
    connect(m_scene, &SceneGraph::jointAdded,    this, &MainWindow::onSceneDirtied);
    connect(m_scene, &SceneGraph::motorAdded,    this, &MainWindow::onSceneDirtied);
    connect(m_scene, &SceneGraph::automationChanged, this, &MainWindow::onSceneDirtied);

    connect(m_undo, &UndoManager::historyChanged, this, &MainWindow::updateUndoActions);

    connect(m_sim, &SimulationEngine::stateChanged,  this, &MainWindow::onSimStateChanged);
    connect(m_sim, &SimulationEngine::activeChanged, this, &MainWindow::onSimActiveChanged);
    connect(m_sim, &SimulationEngine::startFailed,   this, &MainWindow::onSimStartFailed);
    connect(m_sim, &SimulationEngine::timeUpdated,   this, &MainWindow::onTimeUpdated);
    connect(m_sim, &SimulationEngine::stepped,       m_viewport, QOverload<>::of(&QWidget::update));
    connect(m_sim, &SimulationEngine::stepped,       this, &MainWindow::updateSimulationOverlay);

    m_statusTimer.setSingleShot(true);
    connect(&m_statusTimer, &QTimer::timeout, this, [this](){
        statusBar()->setProperty("tone", QString());
        Theme::repolish(statusBar());
        statusBar()->clearMessage();
    });
}

// ── Status helper ─────────────────────────────────────────────────────────────

void MainWindow::showStatus(const QString& msg, bool success) {
    statusBar()->setProperty("tone", success ? "success" : "error");
    Theme::repolish(statusBar());
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
    m_menuBar = new QMenuBar;
    auto* file = m_menuBar->addMenu("&File");
    auto mkA = [&](QMenu* m, const QString& icon, const QString& txt, auto slot,
                   QKeySequence::StandardKey k = QKeySequence::UnknownKey){
        auto* a = new QAction(icon.isEmpty() ? QIcon() : Theme::icon(icon), txt, this);
        if(k != QKeySequence::UnknownKey) a->setShortcut(k);
        connect(a, &QAction::triggered, this, slot);
        m->addAction(a);
        return a;
    };
    mkA(file, "file-plus",       "&New Project",    &MainWindow::newProject,   QKeySequence::New);
    mkA(file, "folder-open",     "&Open Project…", &MainWindow::openProject,  QKeySequence::Open);
    mkA(file, "layout-template", "Open &Template…",&MainWindow::openTemplate);
    file->addSeparator();
    mkA(file, "file-input",      "&Import STEP…",  &MainWindow::importStep);
    file->addSeparator();
    mkA(file, "save",            "&Save",          &MainWindow::saveProject,  QKeySequence::Save);
    mkA(file, "save-all",        "Save &As…",      &MainWindow::saveProjectAs);
    file->addSeparator();
    {
        auto* q = new QAction(Theme::icon("log-out"), "&Quit", this);
        q->setShortcut(QKeySequence::Quit);
        connect(q, &QAction::triggered, qApp, &QApplication::quit);
        file->addAction(q);
    }

    auto* edit = m_menuBar->addMenu("&Edit");
    m_undoAction = new QAction(Theme::icon("undo-2"), "&Undo", this);
    m_undoAction->setShortcut(QKeySequence::Undo);
    m_undoAction->setToolTip("Undo  (Ctrl+Z)");
    m_undoAction->setEnabled(false);
    connect(m_undoAction, &QAction::triggered, m_undo, &UndoManager::undo);
    edit->addAction(m_undoAction);

    m_redoAction = new QAction(Theme::icon("redo-2"), "&Redo", this);
    m_redoAction->setShortcut(QKeySequence::Redo);
    m_redoAction->setToolTip("Redo  (Ctrl+Shift+Z)");
    m_redoAction->setEnabled(false);
    connect(m_redoAction, &QAction::triggered, m_undo, &UndoManager::redo);
    edit->addAction(m_redoAction);

    // Menu bar on the frame, the toolbar island below it (filled by buildToolbar)
    auto* top = new QWidget;
    top->setObjectName("frame");
    auto* tv = new QVBoxLayout(top);
    tv->setContentsMargins(0, 2, 0, 8);
    tv->setSpacing(4);
    tv->addWidget(m_menuBar);
    m_toolHost = new QWidget;
    auto* th = new QHBoxLayout(m_toolHost);
    th->setContentsMargins(8, 6, 8, 6);
    th->setSpacing(6);
    tv->addWidget(new Island(m_toolHost));
    setMenuWidget(top);
}

void MainWindow::buildViewMenu() {
    auto* view = m_menuBar->addMenu("&View");
    auto* themes = view->addMenu(Theme::icon("sun-moon"), "&Theme");
    auto* group = new QActionGroup(themes);
    const struct { const char* name; Theme::Mode mode; } modes[] = {
        {"&Dark",  Theme::Mode::Dark},
        {"&Light", Theme::Mode::Light},
    };
    for (const auto& m : modes) {
        QAction* a = themes->addAction(m.name, this, [mode = m.mode]{ Theme::setMode(mode); });
        a->setCheckable(true);
        a->setChecked(Theme::mode() == m.mode);
        group->addAction(a);
        connect(Theme::notifier(), &Theme::Notifier::changed, a, [a, mode = m.mode]{
            a->setChecked(Theme::mode() == mode);
        });
    }
    view->addSeparator();
    const struct { QDockWidget* dock; const char* icon; } panels[] = {
        {m_hierarchyDock,  "list-tree"},
        {m_propertiesDock, "sliders-horizontal"},
        {m_automationDock, "workflow"},
    };
    for (const auto& p : panels) {
        QAction* a = p.dock->toggleViewAction();
        a->setIcon(Theme::icon(p.icon));
        view->addAction(a);
    }
}

// ── Toolbar ───────────────────────────────────────────────────────────────────

static QAction* mkBtn(QToolBar* tb, const QString& icon, const QString& text, const QString& tip) {
    auto* a = tb->addAction(Theme::icon(icon), text);
    a->setToolTip(tip);
    return a;
}

void MainWindow::buildToolbar() {
    auto mkBar = [this](const QString& name) {
        auto* tb = new QToolBar(name, m_toolHost);
        tb->setMovable(false);
        tb->setIconSize(QSize(18, 18));
        tb->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        m_toolHost->layout()->addWidget(tb);
        return tb;
    };
    auto* tb = mkBar("Main");
    tb->setObjectName("mainToolbar");

    // ── File ──
    connect(mkBtn(tb, "file-plus",   "New",  "New project  (Ctrl+N)"),  &QAction::triggered, this, &MainWindow::newProject);
    connect(mkBtn(tb, "folder-open", "Open", "Open project  (Ctrl+O)"), &QAction::triggered, this, &MainWindow::openProject);
    connect(mkBtn(tb, "save",        "Save", "Save project  (Ctrl+S)"), &QAction::triggered, this, &MainWindow::saveProject);
    tb->addSeparator();
    tb->addAction(m_undoAction);
    tb->addAction(m_redoAction);
    tb->addSeparator();

    // ── Add primitives ──
    connect(mkBtn(tb, "box",             "Cube",     "Add Cube\nAdded as child of selection"),     &QAction::triggered, this, [this]{ addObject(PrimitiveType::Cube);     });
    connect(mkBtn(tb, "cylinder",        "Cylinder", "Add Cylinder\nAdded as child of selection"), &QAction::triggered, this, [this]{ addObject(PrimitiveType::Cylinder); });
    connect(mkBtn(tb, "globe",           "Sphere",   "Add Sphere\nAdded as child of selection"),   &QAction::triggered, this, [this]{ addObject(PrimitiveType::Sphere);   });
    connect(mkBtn(tb, "pill",            "Capsule",  "Add Capsule\nAdded as child of selection"),  &QAction::triggered, this, [this]{ addObject(PrimitiveType::Capsule);  });
    connect(mkBtn(tb, "shopping-basket", "Basket",   "Add Basket\nOpen-top container — add as root to drop objects into"), &QAction::triggered, this, [this]{ addObject(PrimitiveType::Basket);  });
    tb->addSeparator();

    // ── Assembly ──
    connect(mkBtn(tb, "rotate-3d", "Joint",    "Add Revolute joint\nObject must have a parent"), &QAction::triggered, this, &MainWindow::addJointToSelected);
    connect(mkBtn(tb, "cog",       "Actuator", "Add an actuator to the selected part's joint\nso automation logic can drive it"),  &QAction::triggered, this, &MainWindow::addMotorToSelected);
    connect(mkBtn(tb, "trash-2",   "Delete",   "Delete selected  (Del)"),                        &QAction::triggered, this, &MainWindow::deleteSelected);

    static_cast<QHBoxLayout*>(m_toolHost->layout())->addStretch(1);

    // ── Simulation ──
    auto* sim = mkBar("Simulation");
    auto* playA  = mkBtn(sim, "play",          "Play",  "Run the automation logic  (Space)");
    auto* pauseA = mkBtn(sim, "pause",         "Pause", "Pause  (Space)");
    auto* stepA  = mkBtn(sim, "step-forward",  "Step",  "Run exactly one scan (16 ms), then pause  (F10)");
    auto* resetA = mkBtn(sim, "rotate-ccw",    "Reset", "Stop and restore the scene to how it was before Play");
    playA->setIcon(Theme::icon("play", Theme::Tint::OnAccent));
    if (QWidget* w = sim->widgetForAction(playA)) w->setObjectName("primaryTool");
    stepA->setShortcut(Qt::Key_F10);
    connect(playA,  &QAction::triggered, m_sim, &SimulationEngine::play);
    connect(pauseA, &QAction::triggered, m_sim, &SimulationEngine::pause);
    connect(stepA,  &QAction::triggered, m_sim, &SimulationEngine::step);
    connect(resetA, &QAction::triggered, m_sim, &SimulationEngine::reset);

    // Speed and time, captioned like the text-under-icon buttons
    auto captioned = [](QWidget* w, const QString& caption) {
        auto* box = new QWidget;
        auto* v = new QVBoxLayout(box);
        v->setContentsMargins(4, 0, 4, 0);
        v->setSpacing(1);
        v->addWidget(w, 0, Qt::AlignHCenter);
        auto* l = new QLabel(caption);
        l->setObjectName("hint");
        v->addWidget(l, 0, Qt::AlignHCenter);
        return box;
    };
    auto* speed = new QComboBox;
    speed->addItems({"0.25×", "0.5×", "1×", "2×", "4×"});
    speed->setCurrentIndex(2);
    speed->setToolTip("Playback speed (scan results are identical at any speed)");
    connect(speed, &QComboBox::currentIndexChanged, this, [this](int i){
        static const double f[] = {0.25, 0.5, 1.0, 2.0, 4.0};
        m_sim->setSpeedFactor(f[i]);
    });
    sim->addSeparator();
    sim->addWidget(captioned(speed, "Speed"));

    m_timeLabel = new QLabel("t = 0.000 s");
    m_timeLabel->setObjectName("timeChip");
    m_timeLabel->setToolTip("Simulation time");
    sim->addWidget(captioned(m_timeLabel, "Time"));
    sim->addSeparator();

    auto* themeA = mkBtn(sim, "sun-moon", "Theme", "Switch between the dark and light theme");
    connect(themeA, &QAction::triggered, this, []{ Theme::toggle(); });

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

// A dock drawn as one island: an icon + title header (still the drag handle, so
// the dock stays movable; double-click or the button floats it) above the content.
QDockWidget* MainWindow::makeDock(const QString& title, const QString& icon, QWidget* content,
                                  Qt::DockWidgetArea area, QWidget* headerTools) {
    auto* d = new QDockWidget(title, this);
    d->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
    d->setMinimumWidth(200);

    auto* header = new QWidget;
    header->setObjectName("islandHeader");
    header->setAttribute(Qt::WA_StyledBackground);
    auto* h = new QHBoxLayout(header);
    h->setContentsMargins(12, 8, 8, 4);
    h->setSpacing(8);
    auto* ic = new QLabel;
    Theme::setLabelIcon(ic, icon, 15, Theme::Tint::Muted);
    auto* tl = new QLabel(title.toUpper());
    tl->setObjectName("islandTitle");
    h->addWidget(ic);
    h->addWidget(tl);
    h->addStretch();
    if (headerTools) h->addWidget(headerTools);
    auto* floatBtn = new QToolButton;
    floatBtn->setObjectName("islandButton");
    floatBtn->setIcon(Theme::icon("picture-in-picture-2", Theme::Tint::Muted));
    floatBtn->setIconSize(QSize(14, 14));
    floatBtn->setToolTip("Float / dock this panel");
    connect(floatBtn, &QToolButton::clicked, d, [d]{ d->setFloating(!d->isFloating()); });
    h->addWidget(floatBtn);
    d->setTitleBarWidget(header);

    auto* body = new QWidget;
    body->setObjectName("island");
    body->setAttribute(Qt::WA_StyledBackground);
    auto* bl = new QVBoxLayout(body);
    bl->setContentsMargins(0, 0, 0, 4);
    bl->addWidget(content);
    d->setWidget(body);

    Island::decorate(d);
    addDockWidget(area, d);
    return d;
}

void MainWindow::buildDocks() {
    m_hierarchy  = new HierarchyPanel(m_scene, this);
    m_properties = new PropertiesPanel(m_scene, this);

    m_hierarchyDock  = makeDock("Hierarchy",  "list-tree", m_hierarchy, Qt::LeftDockWidgetArea);
    m_hierarchyDock->setObjectName("hierarchyDock");
    m_propertiesDock = makeDock("Properties", "sliders-horizontal", m_properties, Qt::RightDockWidgetArea,
                                m_properties->unitCombo());
    m_propertiesDock->setObjectName("propertiesDock");
    // Initial panel sizes, once the window has its real size
    QTimer::singleShot(0, this, [this]{
        resizeDocks({m_hierarchyDock, m_propertiesDock}, {250, 390}, Qt::Horizontal);
        resizeDocks({m_automationDock}, {320}, Qt::Vertical);
    });

    m_automation = new AutomationPanel(m_scene, m_sim, this);
    m_automationDock = makeDock("Automation", "workflow", m_automation, Qt::BottomDockWidgetArea);
    m_automationDock->setObjectName("automationDock");
    m_automationDock->setMinimumHeight(200);
    connect(m_automation, &AutomationPanel::statusMessage, this, &MainWindow::showStatus);
    connect(m_properties, &PropertiesPanel::openActuatorRequested, this, [this](int jointId){
        m_automationDock->show();
        m_automationDock->raise();
        m_automation->showActuatorForJoint(jointId);
    });

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
    statusBar()->setSizeGripEnabled(false);
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
    openFile(path);
}

bool MainWindow::openFile(const QString& path) {
    m_sim->reset();
    if(ProjectSerializer::load(*m_scene, path)){
        m_currentFile = path;
        m_dirty = false;
        updateTitle();
        m_undo->reset();
        showStatus("Opened: " + QFileInfo(path).fileName());
        return true;
    }
    QMessageBox::critical(this, "Load Failed",
        "Could not load project from:\n" + path +
        "\n\nThe file may be corrupt or incompatible.");
    showStatus("Failed to open project", false);
    return false;
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
        showStatus("Template loaded — see the Automation panel, then press Play");
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
    if(m_scene->motorForJoint(joint->id)){ showStatus("This joint already has an actuator", false); return; }
    int mid = m_scene->addActuatorForJoint(joint->id);
    const Motor* m = m_scene->getMotor(mid);
    showStatus(QString("Actuator %1 added — drive it from a sequence in the Automation panel").arg(m ? m->name : ""));
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

void MainWindow::onSimActiveChanged(bool active) {
    m_properties->setSimulationActive(active);
    updateUndoActions();
    if (!active) {
        m_viewport->setSimulationOverlay({}, {});
        showStatus("Reset — scene restored, definitions editable");
    }
}

void MainWindow::onSimStartFailed(const QStringList& errors) {
    m_automationDock->show();
    m_automationDock->raise();
    QString list = errors.mid(0, 12).join("\n• ");
    if (errors.size() > 12) list += QString("\n… and %1 more").arg(errors.size() - 12);
    QMessageBox::warning(this, "Cannot Start Simulation",
        "The automation definitions have problems:\n\n• " + list +
        "\n\nFix them in the Automation panel (fields with errors are outlined in red).");
    showStatus(QString("Cannot start — %1 problem%2 in the automation logic")
                   .arg(errors.size()).arg(errors.size() == 1 ? "" : "s"), false);
}

void MainWindow::updateSimulationOverlay() {
    AutomationEngine* ae = m_sim->automation();
    if (!ae->isActive()) { m_viewport->setSimulationOverlay({}, {}); return; }

    // Tint each moving actuator's driven link plus everything rigidly attached
    // to it (descendants reached without crossing another joint).
    QSet<int> moving;
    std::function<void(int)> addRigid = [&](int oid) {
        moving.insert(oid);
        if (const SceneObject* o = m_scene->getObject(oid))
            for (int c : o->childIds)
                if (!m_scene->jointForChild(c)) addRigid(c);
    };
    for (int mid : ae->movingMotors()) {
        const Motor* m = m_scene->getMotor(mid);
        const Joint* j = m ? m_scene->getJoint(m->jointId) : nullptr;
        if (j) addRigid(j->childBodyId);
    }
    m_viewport->setSimulationOverlay(moving, ae->activeOperationLabels());
}

void MainWindow::updateUndoActions() {
    // Undo/redo rebuild the scene from a snapshot, which would pull the rug
    // out from under a running simulation — only allow them while stopped.
    bool idle = !m_sim->isActive();
    m_undoAction->setEnabled(idle && m_undo->canUndo());
    m_redoAction->setEnabled(idle && m_undo->canRedo());
}

void MainWindow::onTimeUpdated(double seconds) {
    m_timeLabel->setText(QString("t = %1 s").arg(seconds, 0, 'f', 3));
}
