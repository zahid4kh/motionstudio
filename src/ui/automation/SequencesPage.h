#pragma once
#include "AutomationCommon.h"

class QListWidget;
class QTableWidget;
class QPushButton;
class QToolButton;
class QComboBox;
class QCheckBox;
class QLineEdit;
class QLabel;
class QGroupBox;
class QFormLayout;
class GanttView;

// Sequences tab: list of sequences with their conditions, the selected
// sequence's operations (table + Gantt timing), and an editor form for the
// selected operation.
class SequencesPage : public autoui::Page {
    Q_OBJECT
public:
    SequencesPage(SceneGraph* scene, SimulationEngine* sim, QWidget* parent = nullptr);
    void refresh() override;
    void refreshRuntime() override;
    void setLocked(bool locked) override;
    void selectSequence(int sequenceId);

private:
    Sequence*  currentSequence();
    Operation* currentOperation();
    int        currentOpIndex() const;

    void rebuildSequenceList();
    void loadSequenceForm();
    void rebuildOpTable();
    void loadOpForm();
    void updateOpFormVisibility(OperationType t);
    void rebuildPredecessorMenu();

    void addSequence();
    void deleteSequence();
    void commitSequenceForm();
    void renameSequence();

    void addOperation(OperationType t);
    void duplicateOperation();
    void deleteOperation();
    void moveOperation(int delta);
    void commitOpForm();

    QString describe(const Operation& op) const;
    // " on <part>" for a move to a pose taught on a part, else empty
    QString partText(const Operation& op) const;
    QString predecessorText(const Sequence& seq, const Operation& op) const;

    // Sequence list + props
    QListWidget*    m_seqList = nullptr;
    QPushButton*    m_seqAdd = nullptr, *m_seqDel = nullptr;
    QGroupBox*      m_seqForm = nullptr;
    QLineEdit*      m_seqName = nullptr;
    ExpressionEdit* m_seqEnable = nullptr;
    ExpressionEdit* m_seqStart = nullptr;
    QCheckBox*      m_seqLoop = nullptr;
    QLabel*         m_seqState = nullptr;

    // Operation table
    QTableWidget*   m_ops = nullptr;
    QToolButton*    m_opAdd = nullptr;
    QPushButton*    m_opDup = nullptr, *m_opDel = nullptr, *m_opUp = nullptr, *m_opDown = nullptr;
    GanttView*      m_gantt = nullptr;

    // Operation editor
    QGroupBox*      m_opForm = nullptr;
    QFormLayout*    m_opLayout = nullptr;
    QLineEdit*      m_opName = nullptr;
    QComboBox*      m_opType = nullptr;
    QComboBox*      m_opActuator = nullptr;
    QComboBox*      m_opPose = nullptr;
    QComboBox*      m_opGripper = nullptr;
    QComboBox*      m_opPart = nullptr;
    QComboBox*      m_opOnPart = nullptr;   // MoveL / MoveToPose to a pose taught on a part
    ExpressionEdit* m_opTarget = nullptr;
    ExpressionEdit* m_opVelocity = nullptr;
    ExpressionEdit* m_opDuration = nullptr;
    ExpressionEdit* m_opCondition = nullptr;
    QComboBox*      m_opWalker = nullptr;     // Walk: the robot with legs
    QComboBox*      m_opWalkTo = nullptr;     // Walk: an object's place, or by distance
    ExpressionEdit* m_opForward = nullptr;
    ExpressionEdit* m_opSideways = nullptr;
    ExpressionEdit* m_opTurn = nullptr;
    QComboBox*      m_opMotion = nullptr;     // PlayMotion: the motion
    QComboBox*      m_opRobot = nullptr;      // PlayMotion: the robot (first: the one it was recorded on)
    QCheckBox*      m_opRelative = nullptr;   // PlayMotion: start where the robot stands
    QComboBox*      m_opClip = nullptr;       // PlayMotion of a learned model: its clip
    QComboBox*      m_opBlendClip = nullptr;  // ... blended with another
    ExpressionEdit* m_opBlend = nullptr;      // ... by this much
    QToolButton*    m_opAfter = nullptr;
    ExpressionEdit* m_opStartCond = nullptr;
    ExpressionEdit* m_opOnStart = nullptr;
    ExpressionEdit* m_opOnFinish = nullptr;
    QLabel*         m_opVelLabel = nullptr;
    QLabel*         m_opStatus = nullptr;

    int  m_seqId = -1;
    int  m_opId  = -1;
    bool m_loading = false;
};
