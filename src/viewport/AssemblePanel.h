#pragma once
#include <QFrame>
#include <unordered_map>
#include "../scene/Mate.h"
#include "../scene/SceneObject.h"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QListWidget;
class QPushButton;
class QWidget;
class SceneGraph;

// The island shown in the viewport while Assemble mode is on: what to click
// next, the mate just made (type, flip, offset) and every mate in the scene.
class AssemblePanel : public QFrame {
    Q_OBJECT
public:
    AssemblePanel(SceneGraph* scene, QWidget* parent);

    // The viewport's pick state: 0 = pick the part to move, 1 = pick where it goes
    void setStage(int stage, const QString& firstPick = {});
    // Makes a mate from two picked features, solves it and opens it for editing.
    // Returns false (and says why) when it can't move anything.
    bool createMate(const MateFeature& moving, const MateFeature& fixed);
    void editMate(int id);
    // Closes the mate being edited, keeping it
    void finishEdit();
    int  editedMate() const { return m_editId; }
    int  selectedMate() const;

signals:
    void changed();          // redraw: the edited / selected mate changed
    void closeRequested();

private:
    void rebuildList();
    void syncEditor();
    // Puts every part back where it was before the edited mate was made or
    // opened, then solves again
    void resolveFromSnapshot();
    void takeSnapshot();
    void showStatus(const QString& text, bool error);

    SceneGraph*     m_scene;
    int             m_editId = -1;
    bool            m_syncing = false;
    std::unordered_map<int, Transform> m_snapshot;

    QLabel*         m_step = nullptr;
    QWidget*        m_editor = nullptr;
    QLabel*         m_mateTitle = nullptr;
    QComboBox*      m_type = nullptr;
    QCheckBox*      m_flip = nullptr;
    QLabel*         m_offsetLabel = nullptr;
    QDoubleSpinBox* m_offset = nullptr;
    QLabel*         m_status = nullptr;
    QLabel*         m_listTitle = nullptr;
    QLabel*         m_hint = nullptr;
    QListWidget*    m_list = nullptr;
    QPushButton*    m_delete = nullptr;
};
