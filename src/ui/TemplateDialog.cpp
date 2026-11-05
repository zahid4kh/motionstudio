#include "TemplateDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDialogButtonBox>
#include <QPushButton>
#include "Theme.h"

const QList<TemplateDialog::TemplateInfo> TemplateDialog::s_templates = {
    {
        "Robotic Arm — Pick & Place",
        "3-axis arm running a looping pick & place sequence built from poses,\n"
        "parallel moves, waits and signal actions. Press Play, then switch Start ON\n"
        "in the Automation panel's Runtime tab. Stop holds after the current step.",
        ":/templates/robotic_arm.mstudio",
        "bot"
    },
    {
        "Sensor & Gripper Cell",
        "A belt carries parts until a beam sensor sees one (its speed binding stops it);\n"
        "a gantry lowers, grips whatever its gripper zone detects, and drops it in a bin.\n"
        "Needs MuJoCo for the parts to ride the belt and fall. Press Play.",
        ":/templates/sensor_cell.mstudio",
        "hand-grab"
    },
    {
        "Part Flow Cell — Source, Sink & Vision Pick",
        "A source feeds a box onto the belt every 4 s; a laser measures each one's height.\n"
        "A position sensor finds where the box stopped and the gantry picks it there\n"
        "(MoveL to a pose taught on the part), then drops it in a bin whose sink takes it.\n"
        "Needs MuJoCo for the parts to ride the belt and fall. Press Play.",
        ":/templates/part_flow_cell.mstudio",
        "boxes"
    },
    {
        "Indexing Table",
        "A turntable indexes 90° (target = Table.target + 90), then a press stamps\n"
        "the part and retracts. A derived Safe signal interlocks the table; a counter\n"
        "tracks parts. Press Play, then switch Start ON.",
        ":/templates/indexing_table.mstudio",
        "disc-3"
    },
    {
        "Simple Pendulum",
        "A looping two-operation sequence swings the bob between ±Amplitude.\n"
        "Change Amplitude live from the Runtime tab.",
        ":/templates/simple_pendulum.mstudio",
        "timer"
    },
    {
        "Solar System",
        "Continuous speed-mode actuators with expression bindings:\n"
        "speed = 20 · TimeWarp, active = !Hold. Try them in the Runtime tab.",
        ":/templates/solar_system.mstudio",
        "orbit"
    },
    {
        "Spinning Top",
        "A disc driven in speed mode with acceleration ramps; the Fast input\n"
        "switches its bound speed between 180 and 720 °/s.",
        ":/templates/spinning_top.mstudio",
        "rotate-cw"
    },
    {
        "Crank Arm",
        "A crank turning at 60 °/s while the Run input is on, ramping up and down.",
        ":/templates/crank_arm.mstudio",
        "cog"
    },
};

const QList<TemplateDialog::TemplateInfo> TemplateDialog::s_components = {
    {
        "Parallel Gripper",
        "Two fingers closed by one actuator (Fingers, 0 to 0.035 m); the right finger\n"
        "follows the left through a coupling. A zone sensor InGripper sees a part between\n"
        "them. Parent it under a robot's flange, then Grip with Part \"Detected by InGripper\".",
        ":/library/parallel_gripper.mstudio",
        "hand-grab"
    },
    {
        "Conveyor with End Sensor",
        "A 2 m belt at 0.3 m/s with a beam sensor PartAtEnd at its end; its speed\n"
        "expression stops the belt while a part is there.",
        ":/library/conveyor_end_sensor.mstudio",
        "chevrons-right"
    },
    {
        "Gantry (X/Z)",
        "A two-axis gantry: actuators X (0 to 1.1 m) and Z (lift, −0.5 to 0 m), a\n"
        "gripper pad and a zone sensor AtGripper under it.",
        ":/library/gantry.mstudio",
        "move-diagonal-2"
    },
    {
        "Turntable",
        "An indexing plate on a continuous revolute joint with actuator Table\n"
        "(90 °/s, ramped). Move it with Table.target + 90.",
        ":/library/turntable.mstudio",
        "disc-3"
    },
    {
        "Light Barrier",
        "Emitter and reflector posts 0.6 m apart with a beam sensor Beam between them\n"
        "that sees rigid bodies (parts).",
        ":/library/light_barrier.mstudio",
        "scan-line"
    },
    {
        "Part (box)",
        "A 0.12 m box with a rigid body (0.5 kg): it falls, rides conveyors and can be gripped.",
        ":/library/part_box.mstudio",
        "box"
    },
    {
        "Bin",
        "An open bin with a rigid body (5 kg) to drop parts into.",
        ":/library/bin.mstudio",
        "shopping-basket"
    },
};

TemplateDialog::TemplateDialog(QWidget* parent, Kind kind) : QDialog(parent) {
    const bool components = kind == Kind::Components;
    m_items = components ? &s_components : &s_templates;
    setWindowTitle(components ? "Insert Component" : "Open Template");
    setMinimumSize(540, 420);

    auto* vb = new QVBoxLayout(this);
    vb->setSpacing(10);
    vb->setContentsMargins(18, 16, 18, 16);

    auto* header = new QLabel(components ? "Insert a component into the scene" : "Choose a template to explore");
    header->setObjectName("dialogTitle");
    vb->addWidget(header);
    auto* sub = new QLabel(components
        ? "It is added at the centre of the view with its actuators and sensors. Names that are already "
          "used get a suffix (J1 → J1_2), and its expressions follow."
        : "Each one is a working automation example — load it, then press Play.");
    sub->setWordWrap(true);
    sub->setObjectName("hint");
    vb->addWidget(sub);

    m_list = new QListWidget(this);
    m_list->setObjectName("templateList");
    m_list->setIconSize(QSize(18, 18));
    m_list->setMinimumHeight(190);
    vb->addWidget(m_list);

    m_desc = new QLabel(this);
    m_desc->setObjectName("templateDesc");
    m_desc->setWordWrap(true);
    m_desc->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    m_desc->setText("Select a template above to see its description.");
    vb->addWidget(m_desc);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    btns->button(QDialogButtonBox::Ok)->setText(components ? "Insert" : "Load Template");
    btns->button(QDialogButtonBox::Ok)->setIcon(Theme::icon(components ? "package" : "layout-template", Theme::Tint::OnAccent));
    btns->button(QDialogButtonBox::Ok)->setDefault(true);
    btns->button(QDialogButtonBox::Ok)->setEnabled(false);
    vb->addWidget(btns);

    populate();

    connect(m_list, &QListWidget::currentItemChanged, this,
        [this, btns](QListWidgetItem* item, QListWidgetItem*){
            onItemChanged(item);
            btns->button(QDialogButtonBox::Ok)->setEnabled(item != nullptr);
        });
    connect(m_list, &QListWidget::itemDoubleClicked, this, &TemplateDialog::accept);
    connect(btns, &QDialogButtonBox::accepted, this, &TemplateDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, this, &TemplateDialog::reject);
}

void TemplateDialog::populate() {
    for(int i = 0; i < m_items->size(); ++i){
        auto* item = new QListWidgetItem(Theme::icon(m_items->at(i).icon), m_items->at(i).name, m_list);
        item->setData(Qt::UserRole, i);
    }
}

void TemplateDialog::onItemChanged(QListWidgetItem* item) {
    if(!item){ m_desc->setText("Select a template above to see its description."); return; }
    int idx = item->data(Qt::UserRole).toInt();
    if(idx < 0 || idx >= m_items->size()) return;
    m_desc->setText(m_items->at(idx).description);
}

QString TemplateDialog::selectedPath() const {
    auto* item = m_list->currentItem();
    if(!item) return {};
    int idx = item->data(Qt::UserRole).toInt();
    if(idx < 0 || idx >= m_items->size()) return {};
    return m_items->at(idx).path;
}

QString TemplateDialog::selectedName() const {
    auto* item = m_list->currentItem();
    return item ? item->text() : QString();
}
