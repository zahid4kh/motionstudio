#pragma once
#include <QDialog>
#include <QTimer>
#include <memory>
#include <vector>
#include "robot/RobotBuilder.h"

class QButtonGroup;
class QComboBox;
class QDoubleSpinBox;
class QGridLayout;
class QLabel;
class QLineEdit;
class QTableWidget;
class QTabWidget;
class RobotAnalysisPanel;
class RobotSketch;
class SceneGraph;

// Tools › Robot Builder: designs a SCARA, palletizer or 6-axis robot from a
// preset, its dimensions, axis ranges, end effector and look, with a live
// dimensioned drawing and analysis of the robot it will build.
class RobotBuilderDialog : public QDialog {
    Q_OBJECT
public:
    explicit RobotBuilderDialog(QWidget* parent = nullptr);
    ~RobotBuilderDialog() override;

    robot::Spec spec() const;   // validated

    static QString lastSpecKey() { return "robotBuilder/spec"; }

protected:
    bool eventFilter(QObject* o, QEvent* e) override;

private:
    void setKind(robot::Kind k);
    void loadSpec(const robot::Spec& s);
    void rebuildDimensions();
    void rebuildAxes();
    void fillPresets();
    void readUi();
    void edited();
    void refresh();

    robot::Spec m_spec;
    bool m_loading = false;
    QTimer m_timer;
    std::unique_ptr<SceneGraph> m_preview;

    QButtonGroup* m_kind;
    QComboBox* m_preset;
    QLabel* m_presetDesc;
    QLineEdit* m_name;
    QLineEdit* m_prefix;
    QGridLayout* m_dimGrid;
    std::vector<std::pair<QString, QDoubleSpinBox*>> m_dims;
    QComboBox* m_tool;
    QLabel* m_toolDesc;
    QDoubleSpinBox* m_toolScale;
    QComboBox* m_style;
    QComboBox* m_scheme;
    QDoubleSpinBox* m_linkScale;
    QTableWidget* m_axes;
    QLabel* m_problems;
    QLabel* m_summary;
    QTabWidget* m_tabs;
    RobotSketch* m_drawing;
    RobotAnalysisPanel* m_analysis;
};
