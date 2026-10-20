#include "TemplateDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDialogButtonBox>
#include <QPushButton>

static const char* kDialogStyle =
    "QDialog { background:#f4f6fb; }"
    "QListWidget { background:#ffffff; border:1px solid #c8cdd8; border-radius:4px; "
    "              font-size:13px; color:#222; outline:0; }"
    "QListWidget::item { padding:9px 12px; border-bottom:1px solid #eef0f5; }"
    "QListWidget::item:selected { background:#ddeafc; color:#1040a0; border-color:#b8d0f0; }"
    "QListWidget::item:hover    { background:#eef4ff; }"
    "QLabel#desc { background:#ffffff; border:1px solid #c8cdd8; border-radius:4px; "
    "              padding:10px 12px; font-size:12px; color:#445; min-height:60px; }"
    "QDialogButtonBox QPushButton { padding:5px 18px; border-radius:4px; font-size:12px; "
    "                               border:1px solid #c8cdd8; background:#f4f6fb; color:#333; }"
    "QDialogButtonBox QPushButton:hover   { background:#ddeafc; border-color:#7aaae8; color:#1040a0; }"
    "QDialogButtonBox QPushButton:default { background:#1a6fda; color:#fff; border-color:#1050b0; }"
    "QDialogButtonBox QPushButton:default:hover { background:#1558b8; }";

const QList<TemplateDialog::TemplateInfo> TemplateDialog::s_templates = {
    {
        "Robotic Arm — Pick & Place",
        "3-axis arm running a looping pick & place sequence built from poses,\n"
        "parallel moves, waits and signal actions. Press Play, then switch Start ON\n"
        "in the Automation panel's Runtime tab. Stop holds after the current step.",
        ":/templates/robotic_arm.mstudio"
    },
    {
        "Indexing Table",
        "A turntable indexes 90° (target = Table.target + 90), then a press stamps\n"
        "the part and retracts. A derived Safe signal interlocks the table; a counter\n"
        "tracks parts. Press Play, then switch Start ON.",
        ":/templates/indexing_table.mstudio"
    },
    {
        "Simple Pendulum",
        "A looping two-operation sequence swings the bob between ±Amplitude.\n"
        "Change Amplitude live from the Runtime tab.",
        ":/templates/simple_pendulum.mstudio"
    },
    {
        "Solar System",
        "Continuous speed-mode actuators with expression bindings:\n"
        "speed = 20 · TimeWarp, active = !Hold. Try them in the Runtime tab.",
        ":/templates/solar_system.mstudio"
    },
    {
        "Spinning Top",
        "A disc driven in speed mode with acceleration ramps; the Fast input\n"
        "switches its bound speed between 180 and 720 °/s.",
        ":/templates/spinning_top.mstudio"
    },
    {
        "Crank Arm",
        "A crank turning at 60 °/s while the Run input is on, ramping up and down.",
        ":/templates/crank_arm.mstudio"
    },
};

TemplateDialog::TemplateDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle("Open Template");
    setMinimumSize(500, 340);
    setStyleSheet(kDialogStyle);

    auto* vb = new QVBoxLayout(this);
    vb->setSpacing(10);
    vb->setContentsMargins(14, 14, 14, 14);

    auto* header = new QLabel("Choose a template to explore:");
    header->setStyleSheet("font-size:13px; font-weight:bold; color:#222; background:transparent;");
    vb->addWidget(header);

    m_list = new QListWidget(this);
    m_list->setMinimumHeight(160);
    vb->addWidget(m_list);

    m_desc = new QLabel(this);
    m_desc->setObjectName("desc");
    m_desc->setWordWrap(true);
    m_desc->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    m_desc->setText("Select a template above to see its description.");
    vb->addWidget(m_desc);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    btns->button(QDialogButtonBox::Ok)->setText("Load Template");
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
    for(int i = 0; i < s_templates.size(); ++i){
        auto* item = new QListWidgetItem(s_templates[i].name, m_list);
        item->setData(Qt::UserRole, i);
    }
}

void TemplateDialog::onItemChanged(QListWidgetItem* item) {
    if(!item){ m_desc->setText("Select a template above to see its description."); return; }
    int idx = item->data(Qt::UserRole).toInt();
    if(idx < 0 || idx >= s_templates.size()) return;
    m_desc->setText(s_templates[idx].description);
}

QString TemplateDialog::selectedPath() const {
    auto* item = m_list->currentItem();
    if(!item) return {};
    int idx = item->data(Qt::UserRole).toInt();
    if(idx < 0 || idx >= s_templates.size()) return {};
    return s_templates[idx].path;
}
