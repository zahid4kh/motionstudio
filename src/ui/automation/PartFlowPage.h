#pragma once
#include <QDialog>
#include "AutomationCommon.h"

class QTableWidget;
class QPushButton;
class QComboBox;
class QCheckBox;
class QDoubleSpinBox;
class QSpinBox;
class QLabel;
class QFormLayout;

// Part Flow tab: object sources (copies of a template part, made while
// simulating) and sinks (zones that take parts away), with their counts.
class PartFlowPage : public autoui::Page {
    Q_OBJECT
public:
    PartFlowPage(SceneGraph* scene, SimulationEngine* sim, QWidget* parent = nullptr);
    void refresh() override;
    void refreshRuntime() override;
    void setLocked(bool locked) override;
    // A source copying `objectId` / a sink on `objectId`'s bounds, settings open
    void addSourceFor(int objectId);
    void addSinkOn(int objectId);

private:
    void configure(int row);
    void deleteSelected();
    void onItemChanged(int row, int col);

    QTableWidget* m_table = nullptr;
    QPushButton*  m_addSource = nullptr, *m_addSink = nullptr, *m_delete = nullptr;
    bool          m_building = false;
};

// Edits an object source: what it copies, when, how many
class SourceDialog : public QDialog {
    Q_OBJECT
public:
    SourceDialog(SceneGraph* scene, const PartSource& src, QWidget* parent = nullptr);
    PartSource config() const;

private:
    void updateForm();

    SceneGraph*     m_scene;
    PartSource      m_src;
    QComboBox*      m_template = nullptr;
    QComboBox*      m_mode = nullptr;
    ExpressionEdit* m_interval = nullptr;
    ExpressionEdit* m_trigger = nullptr;
    ExpressionEdit* m_enable = nullptr;
    QSpinBox*       m_max = nullptr;
    QCheckBox*      m_clear = nullptr;
    QFormLayout*    m_form = nullptr;
    QLabel*         m_hint = nullptr;
};

// Edits an object sink: its zone, which parts it takes
class SinkDialog : public QDialog {
    Q_OBJECT
public:
    SinkDialog(SceneGraph* scene, const PartSink& sink, QWidget* parent = nullptr);
    PartSink config() const;

private:
    void updateForm();

    SceneGraph*     m_scene;
    PartSink        m_sink;
    QComboBox*      m_mount = nullptr;
    QCheckBox*      m_bounds = nullptr;
    QDoubleSpinBox* m_offset[3] = {};
    QDoubleSpinBox* m_size[3] = {};
    QComboBox*      m_source = nullptr;
    ExpressionEdit* m_enable = nullptr;
    QFormLayout*    m_form = nullptr;
    class QLayout*  m_sizeRow = nullptr;
    QLabel*         m_hint = nullptr;
};

namespace autoui {
// One-line descriptions for tables
QString sourceSummary(const SceneGraph& scene, const PartSource& src);
QString sinkSummary(const SceneGraph& scene, const PartSink& sink);
}
