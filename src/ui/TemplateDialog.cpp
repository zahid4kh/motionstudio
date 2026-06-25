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
        "Simple Pendulum",
        "A pivot cylinder with a sphere bob attached by a revolute joint.\n"
        "Motor drives 30 °/s — watch the pendulum sweep back and forth.",
        ":/templates/simple_pendulum.mstudio"
    },
    {
        "Robotic Arm",
        "3-segment arm: Base → Arm1 → Arm2 → Gripper.\n"
        "Three revolute joints with independent motors at 25, 40, and 90 °/s.",
        ":/templates/robotic_arm.mstudio"
    },
    {
        "Solar System",
        "Sun sphere with an orbiting Planet and a Moon that orbits the Planet.\n"
        "Nested revolute joints — planet orbits at 20 °/s, moon at 75 °/s.",
        ":/templates/solar_system.mstudio"
    },
    {
        "Spinning Top",
        "A flat disc on a stand with a spike — driven at 180 °/s.\n"
        "Good for testing fast rotation and child-body carry-along.",
        ":/templates/spinning_top.mstudio"
    },
    {
        "Crank Arm",
        "A crank cylinder attached to a body, with a swinging arm and sphere handle.\n"
        "Motor drives 60 °/s — watch the handle trace a full circle.",
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
