#pragma once
#include <QDialog>

class SceneGraph;
class ApiServer;
class QCheckBox;
class QSpinBox;
class QComboBox;
class QLabel;
class QTableWidget;
class QTabWidget;
class QPlainTextEdit;

// Tools › Robot API: runs the server that the Python, C++ and Kotlin
// libraries connect to, lists the robots a program can address and shows a
// quick start for this scene. Server settings are kept per machine
// (QSettings api/…).
class ApiServerDialog : public QDialog {
    Q_OBJECT
public:
    ApiServerDialog(SceneGraph* scene, ApiServer* server, QWidget* parent = nullptr);

    // Starts the server unless it was switched off (it is on by default)
    static void restore(ApiServer* server);

private:
    void applyServer();
    void rebuild();
    void updateStatus();

    SceneGraph*     m_scene;
    ApiServer*      m_server;
    QCheckBox*      m_run = nullptr;
    QSpinBox*       m_port = nullptr;
    QComboBox*      m_bind = nullptr;
    QLabel*         m_status = nullptr;
    QTableWidget*   m_robots = nullptr;
    QTabWidget*     m_langs = nullptr;
    std::vector<QPlainTextEdit*> m_code;
};
