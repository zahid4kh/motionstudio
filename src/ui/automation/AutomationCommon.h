#pragma once
#include <QWidget>
#include <QString>
#include "ExpressionEdit.h"
#include "scene/SceneGraph.h"
#include "automation/AutomationEngine.h"

class SimulationEngine;

namespace autoui {

// Shared look for the automation pages (matches PropertiesPanel's light theme)
extern const char* kPageStyle;

enum class ExprKind { Value, Assignments };

// ExpressionEdit wired to validate against the scene's current symbols and
// to autocomplete their names.
ExpressionEdit* makeExprEdit(SceneGraph* scene, ExprKind kind, const QString& placeholder,
                             QWidget* parent = nullptr);

// Formats a runtime value for display, using the symbol's type
QString formatValue(double v, bool isBool, SignalType type);

// Unit suffix for a joint's position ("°" or " m") and velocity
QString posUnit(const Joint* j);
QString velUnit(const Joint* j);

// Base for the panel pages: coalesced refresh on scene changes (skipping
// changes the page made itself), and an edit lock while the simulation is active.
class Page : public QWidget {
    Q_OBJECT
public:
    Page(SceneGraph* scene, SimulationEngine* sim, QWidget* parent = nullptr);

    virtual void refresh() = 0;            // rebuild from scene definitions
    virtual void refreshRuntime() {}       // update live values (called ~20 Hz)
    virtual void setLocked(bool locked) { m_locked = locked; }
    bool locked() const { return m_locked; }

signals:
    void statusMessage(const QString& msg, bool ok);

protected:
    // Run a scene edit without triggering this page's own refresh
    template <typename F> void edit(F&& f) {
        m_selfEdit = true;
        f();
        m_selfEdit = false;
    }
    void scheduleRefresh();

    SceneGraph*       m_scene;
    SimulationEngine* m_sim;
    bool              m_locked   = false;
    bool              m_selfEdit = false;
    bool              m_pending  = false;
};

} // namespace autoui
