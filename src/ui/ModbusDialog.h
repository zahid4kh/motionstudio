#pragma once
#include <QDialog>

class SceneGraph;
class ModbusServer;
class QCheckBox;
class QSpinBox;
class QComboBox;
class QLabel;
class QTableWidget;

// External I/O: runs the Modbus TCP server and edits each signal's address.
// Server settings are kept per machine (QSettings), addresses in the project.
class ModbusDialog : public QDialog {
    Q_OBJECT
public:
    ModbusDialog(SceneGraph* scene, ModbusServer* server, QWidget* parent = nullptr);

    // Starts the server if it was left running last time (QSettings modbus/…)
    static void restore(ModbusServer* server);

private:
    void applyServer();
    void rebuildTable();
    void updateStatus();

    SceneGraph*   m_scene;
    ModbusServer* m_server;
    QCheckBox*    m_run = nullptr;
    QSpinBox*     m_port = nullptr;
    QComboBox*    m_bind = nullptr;
    QLabel*       m_status = nullptr;
    QTableWidget* m_table = nullptr;
    bool          m_building = false;
};
