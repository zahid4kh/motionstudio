#pragma once
#include "AutomationCommon.h"
#include <QSet>
#include <vector>

class QTableWidget;
class QTreeWidget;
class QLineEdit;
class QLabel;
class QVBoxLayout;
class QComboBox;
class QPushButton;
class QDoubleSpinBox;
class SignalTracePlot;

// Runtime tab: HMI-style input controls, live watch list, sequence/operation
// status with "why is it waiting" details, and a signal trace plot.
class RuntimePage : public autoui::Page {
    Q_OBJECT
public:
    RuntimePage(SceneGraph* scene, SimulationEngine* sim, QWidget* parent = nullptr);
    void refresh() override;
    void refreshRuntime() override;
    void setLocked(bool locked) override;

private:
    void rebuildInputs();
    void rebuildWatch();
    void rebuildStatusTree();
    void updateTraces();
    void onScanned();

    struct InputRow {
        QString         name;
        SignalType      type;
        QPushButton*    toggle = nullptr;
        QPushButton*    pulse  = nullptr;
        QDoubleSpinBox* spin   = nullptr;
    };
    std::vector<InputRow> m_inputs;
    QWidget*        m_inputHost = nullptr;
    QVBoxLayout*    m_inputLayout = nullptr;

    QLineEdit*      m_filter = nullptr;
    QTableWidget*   m_watch = nullptr;
    QSet<QString>   m_plotted;

    QLabel*         m_header = nullptr;
    QTreeWidget*    m_status = nullptr;
    QLabel*         m_warnings = nullptr;
    class QCheckBox* m_checkInterference = nullptr;
    QLabel*         m_interference = nullptr;

    SignalTracePlot* m_plot = nullptr;
    QComboBox*       m_window = nullptr;
    std::vector<int> m_traceSlots;
    long             m_lastScan = -1;
    bool             m_building = false;
};
