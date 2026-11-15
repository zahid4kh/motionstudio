#pragma once
#include <QDialog>
#include <QTimer>
#include <memory>
#include <vector>
#include "robot/HumanoidBuilder.h"

class QButtonGroup;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGridLayout;
class QLabel;
class QLineEdit;
class QTableWidget;
class QTabWidget;
class RobotSketch;
class SceneGraph;

// Tools › Humanoid Builder: designs a humanoid from a preset, its body
// dimensions, the joints it has (with their ranges and speeds), its hands and
// look, with a live dimensioned drawing in front, side and top views.
class HumanoidBuilderDialog : public QDialog {
    Q_OBJECT
public:
    explicit HumanoidBuilderDialog(QWidget* parent = nullptr);
    ~HumanoidBuilderDialog() override;

    humanoid::Spec spec() const;   // validated

    static QString lastSpecKey() { return "humanoidBuilder/spec"; }

protected:
    bool eventFilter(QObject* o, QEvent* e) override;

private:
    void loadSpec(const humanoid::Spec& s);
    void rebuildDimensions();
    void rebuildJoints();
    void fillPresets();
    void readUi();
    void edited();
    void markCustom();
    void showDimension(const QString& key);   // highlights it, in a view that shows it
    void refresh();

    humanoid::Spec m_spec;
    bool m_loading = false;
    QTimer m_timer;
    std::unique_ptr<SceneGraph> m_preview;

    QComboBox* m_preset;
    QLabel* m_presetDesc;
    QLineEdit* m_name;
    QButtonGroup* m_base;
    QDoubleSpinBox* m_height;
    QGridLayout* m_dimGrid;
    std::vector<std::pair<QString, QDoubleSpinBox*>> m_dims;
    QComboBox* m_hand;
    QLabel* m_handDesc;
    QComboBox* m_style;
    QComboBox* m_scheme;
    QDoubleSpinBox* m_bulk;
    QCheckBox* m_demo;
    QTableWidget* m_joints;
    std::vector<std::pair<QString, int>> m_jointRows;   // joint key → table row
    QLabel* m_problems;
    QLabel* m_summary;
    QTabWidget* m_tabs;
    QButtonGroup* m_views;
    RobotSketch* m_drawing;
};
