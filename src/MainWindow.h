#pragma once
#include <QMainWindow>
#include <QLabel>
#include "scene/SceneGraph.h"
#include "viewport/Viewport.h"
#include "physics/SimulationEngine.h"
#include "ui/HierarchyPanel.h"
#include "ui/PropertiesPanel.h"

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void newProject();
    void openProject();
    void saveProject();
    void saveProjectAs();

    void addObject(PrimitiveType type);
    void addJointToSelected();
    void addMotorToSelected();
    void deleteSelected();

    void onSimStateChanged(bool running);
    void onTimeUpdated(double seconds);

private:
    void buildMenu();
    void buildToolbar();
    void buildDocks();
    void buildStatusBar();

    SceneGraph*       m_scene    = nullptr;
    Viewport*         m_viewport = nullptr;
    SimulationEngine* m_sim      = nullptr;
    HierarchyPanel*   m_hierarchy = nullptr;
    PropertiesPanel*  m_properties = nullptr;

    QLabel*  m_timeLabel  = nullptr;
    QString  m_currentFile;
};
