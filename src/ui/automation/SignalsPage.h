#pragma once
#include "AutomationCommon.h"

class QTableWidget;
class QPushButton;

// Signals tab: table of user signals (Input / Internal / Derived / Sensor)
// with inline editing of name, kind, type, initial value, expression or
// sensor settings, description.
class SignalsPage : public autoui::Page {
    Q_OBJECT
public:
    SignalsPage(SceneGraph* scene, SimulationEngine* sim, QWidget* parent = nullptr);
    void refresh() override;
    void refreshRuntime() override;
    void setLocked(bool locked) override;
    // Adds a sensor mounted on `objectId` (-1: the world) and opens its settings
    void addSensorOn(int objectId);

private:
    void addSignal(SignalKind kind);
    void configureSensor(int signalId);
    void deleteSelected();
    void onItemChanged(int row, int col);
    int  signalIdAt(int row) const;

    QTableWidget* m_table = nullptr;
    QPushButton*  m_addInput = nullptr, *m_addInternal = nullptr, *m_addDerived = nullptr, *m_addSensor = nullptr, *m_delete = nullptr;
    bool          m_building = false;
};
