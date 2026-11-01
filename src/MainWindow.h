#pragma once
#include <QMainWindow>
#include <QLabel>
#include <QTimer>
#include <QProgressDialog>
#include <QFutureWatcher>
#include "scene/SceneGraph.h"
#include "scene/UndoManager.h"
#include "viewport/Viewport.h"
#include "physics/SimulationEngine.h"
#include "ui/HierarchyPanel.h"
#include "ui/PropertiesPanel.h"
#include "ui/automation/AutomationPanel.h"
#include "import/StepImporter.h"

class QAction;
class QDockWidget;
class QMenuBar;

// Result of a background StepImporter::importFile() call (see importStepFile)
struct StepImportResult {
    bool               ok = false;
    StepImporter::Part root;
    QString            error;
};

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

    // Opens a .mstudio project (e.g. passed on the command line)
    bool openFile(const QString& path);

protected:
    void dragEnterEvent(QDragEnterEvent* e) override;
    void dropEvent(QDropEvent* e) override;

private slots:
    void newProject();
    void openProject();
    void openTemplate();
    void insertComponent();
    void saveProject();
    void saveProjectAs();
    void importStep();
    void importRobot();
    void exportRobot(bool mjcf);

    void addObject(PrimitiveType type);
    void addJointToSelected();
    void addMotorToSelected();
    void deleteSelected();
    void duplicateObject(int id);

    void onSimStateChanged(bool running);
    void onSimActiveChanged(bool active);
    void onSimStartFailed(const QStringList& errors);
    void updateSimulationOverlay();
    void onTimeUpdated(double seconds);
    void onSceneDirtied();
    void onStepImportFinished();

private:
    void buildMenu();
    void buildToolbar();
    void buildDocks();
    void buildViewMenu();
    void buildHelpMenu();
    void buildToolsMenu();
    void openExternalIo();
    void updateModbusLabel();
    void buildStatusBar();
    QDockWidget* makeDock(const QString& title, const QString& icon, QWidget* content,
                          Qt::DockWidgetArea area, QWidget* headerTools = nullptr);
    void updateTitle();
    void updateUndoActions();
    // File › Open Recent, most recent first (QSettings "file/recent")
    void rememberRecentFile(const QString& path);
    void fillRecentMenu(QMenu* menu);

    void importStepFile(const QString& path);
    int  addImportedPart(const StepImporter::Part& part, int parentId);

    // "success" = green feedback, default = error (red)
    void showStatus(const QString& msg, bool success = true);

    SceneGraph*       m_scene      = nullptr;
    UndoManager*      m_undo       = nullptr;
    Viewport*         m_viewport   = nullptr;
    SimulationEngine* m_sim        = nullptr;
    HierarchyPanel*   m_hierarchy  = nullptr;
    PropertiesPanel*  m_properties = nullptr;
    AutomationPanel*  m_automation = nullptr;
    class TimelineBar* m_timeline  = nullptr;
    class ModbusServer* m_modbus   = nullptr;
    QLabel*  m_modbusLabel = nullptr;   // status bar: server state
    QDockWidget*      m_hierarchyDock  = nullptr;
    QDockWidget*      m_propertiesDock = nullptr;
    QDockWidget*      m_automationDock = nullptr;

    // The menu bar lives in a custom menu widget (with the toolbar island),
    // so QMainWindow::menuBar() must not be used — it would replace that widget.
    QMenuBar*         m_menuBar   = nullptr;
    QWidget*          m_toolHost  = nullptr;

    QAction* m_undoAction = nullptr;
    QAction* m_redoAction = nullptr;

    QLabel*  m_timeLabel   = nullptr;
    QTimer   m_statusTimer;
    QString  m_currentFile;
    bool     m_dirty = false;

    // STEP import runs on a worker thread (large assemblies can take a
    // minute-plus to triangulate); a modal progress dialog blocks further
    // interaction until it completes.
    QFutureWatcher<StepImportResult>* m_stepWatcher  = nullptr;
    QProgressDialog*                  m_stepProgress = nullptr;
    QString                           m_stepImportName;
};
