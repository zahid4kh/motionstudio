#pragma once
#include "AutomationCommon.h"

class QTableWidget;
class QPushButton;

// Signals tab: table of user signals (Input / Internal / Derived) with
// inline editing of name, kind, type, initial value, expression, description.
class SignalsPage : public autoui::Page {
    Q_OBJECT
public:
    SignalsPage(SceneGraph* scene, SimulationEngine* sim, QWidget* parent = nullptr);
    void refresh() override;
    void refreshRuntime() override;
    void setLocked(bool locked) override;

private:
    void addSignal(SignalKind kind);
    void deleteSelected();
    void onItemChanged(int row, int col);
    int  signalIdAt(int row) const;

    QTableWidget* m_table = nullptr;
    QPushButton*  m_addInput = nullptr, *m_addInternal = nullptr, *m_addDerived = nullptr, *m_delete = nullptr;
    bool          m_building = false;
};
