#include "RobotAnalysisPanel.h"
#include "RobotSketch.h"
#include "scene/SceneGraph.h"
#include "ui/Theme.h"
#include <QButtonGroup>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>
#include <cmath>

namespace {

QString mm(double m) { return QString::number(std::round(m * 1000)); }

QLabel* subheading(const QString& text) {
    auto* l = new QLabel(text);
    l->setObjectName("subheading");
    return l;
}

enum Col { C_Axis, C_Type, C_Range, C_A, C_Alpha, C_D, C_Now, C_Speed, C_Time, C_Count };

} // namespace

RobotAnalysisPanel::RobotAnalysisPanel(QWidget* parent) : QWidget(parent) {
    auto* vb = new QVBoxLayout(this);
    vb->setContentsMargins(0, 0, 0, 0);
    vb->setSpacing(10);

    auto* top = new QHBoxLayout;
    top->setSpacing(12);
    auto* left = new QVBoxLayout;
    left->setSpacing(6);
    auto* views = new QHBoxLayout;
    m_side = new QPushButton(Theme::icon("ruler"), "Side");
    m_top = new QPushButton(Theme::icon("radar"), "Top");
    auto* group = new QButtonGroup(this);
    for (auto [b, icon] : {std::pair{m_side, "ruler"}, {m_top, "radar"}}) {
        b->setObjectName("segment");
        b->setCheckable(true);
        Theme::segmentIcon(b, icon);
        group->addButton(b);
        views->addWidget(b);
    }
    m_side->setChecked(true);
    views->addStretch();
    auto* legend = new QLabel("Shaded: everywhere the TCP reaches with every axis in its range");
    legend->setObjectName("hint");
    views->addWidget(legend);
    left->addLayout(views);
    m_sketch = new RobotSketch;
    m_sketch->setMinimumHeight(260);
    left->addWidget(m_sketch, 1);
    top->addLayout(left, 1);

    auto* right = new QVBoxLayout;
    right->setSpacing(4);
    right->addWidget(subheading("WORKSPACE"));
    auto* stats = new QGridLayout;
    stats->setHorizontalSpacing(10);
    stats->setVerticalSpacing(2);
    int row = 0;
    auto stat = [&](const QString& name, const QString& tip) {
        auto* l = new QLabel(name);
        l->setObjectName("fieldLabel");
        l->setToolTip(tip);
        auto* v = new QLabel("—");
        v->setObjectName("statValue");
        v->setToolTip(tip);
        v->setTextInteractionFlags(Qt::TextSelectableByMouse);
        stats->addWidget(l, row, 0);
        stats->addWidget(v, row++, 1);
        return v;
    };
    m_reach = stat("Max reach", "Farthest the TCP gets from the base axis (J1)");
    m_minReach = stat("Min reach", "Closest the TCP gets to the base axis");
    m_height = stat("Height", "Lowest and highest TCP position above the floor");
    m_volume = stat("Volume", "Workspace volume the TCP sweeps (the side profile turned by J1)");
    m_sweep = stat("Base sweep", "How far J1 turns");
    right->addLayout(stats);
    right->addSpacing(6);
    right->addWidget(subheading("CURRENT POSE"));
    m_tcp = new QLabel("—");
    m_tcp->setObjectName("mono");
    m_tcp->setTextInteractionFlags(Qt::TextSelectableByMouse);
    right->addWidget(m_tcp);
    m_dexLabel = new QLabel;
    m_dexLabel->setObjectName("fieldLabel");
    m_dexLabel->setToolTip("How evenly the TCP can move in every direction (smallest ÷ largest singular value of the "
                           "Jacobian). Near 0 the robot is at a singularity: some direction needs huge joint speeds.");
    right->addWidget(m_dexLabel);
    m_dex = new QProgressBar;
    m_dex->setRange(0, 100);
    m_dex->setTextVisible(false);
    right->addWidget(m_dex);
    m_warnings = new QLabel;
    m_warnings->setWordWrap(true);
    m_warnings->setObjectName("warningText");
    right->addWidget(m_warnings);
    right->addStretch();
    auto* rightBox = new QWidget;
    rightBox->setLayout(right);
    rightBox->setFixedWidth(250);
    top->addWidget(rightBox);
    vb->addLayout(top, 3);

    vb->addWidget(subheading("AXES  ·  DENAVIT–HARTENBERG PARAMETERS (MEASURED FROM THE MODEL)"));
    m_table = new QTableWidget(0, C_Count);
    m_table->setHorizontalHeaderLabels({"Axis", "Type", "Range", "a (mm)", "α (°)", "d (mm)", "Now", "Speed", "90° / 100 mm"});
    m_table->horizontalHeaderItem(C_A)->setToolTip("Length of the common normal to the next axis (to the TCP for the last)");
    m_table->horizontalHeaderItem(C_Alpha)->setToolTip("Twist from this axis to the next, about the common normal");
    m_table->horizontalHeaderItem(C_D)->setToolTip("Offset along this axis from the previous common normal to the next");
    m_table->horizontalHeaderItem(C_Time)->setToolTip("Time for a 90° (or 100 mm) move at the actuator's speed and acceleration");
    m_table->verticalHeader()->hide();
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->setSelectionMode(QAbstractItemView::NoSelection);
    m_table->setAlternatingRowColors(true);
    m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    for (int c : {C_A, C_Alpha, C_D, C_Now}) m_table->horizontalHeader()->setSectionResizeMode(c, QHeaderView::Stretch);
    m_table->verticalHeader()->setDefaultSectionSize(22);
    m_table->setMinimumHeight(170);
    vb->addWidget(m_table, 2);

    connect(m_side, &QPushButton::toggled, this, [this](bool on) { if (on) m_sketch->setView(RobotSketch::View::Side); });
    connect(m_top, &QPushButton::toggled, this, [this](bool on) { if (on) m_sketch->setView(RobotSketch::View::Top); });
}

void RobotAnalysisPanel::capture(const robot::Analysis& a, const SceneGraph& g) {
    const Joint* j1 = a.axes.empty() ? nullptr : g.getJoint(a.axes.front().jointId);
    if (!j1) return;
    m_sketch->capture(g, j1->parentBodyId, a.toolId);
}

void RobotAnalysisPanel::setAnalysis(const robot::Analysis& a, const SceneGraph& g) {
    if (!a.error.isEmpty()) { setError(a.error); return; }
    capture(a, g);
    m_sketch->setEnvelope(&a, &g);
    m_reach->setText(QString("%1 mm").arg(mm(a.maxReach)));
    m_minReach->setText(QString("%1 mm").arg(mm(a.minReach)));
    m_height->setText(QString("%1 … %2 mm").arg(mm(a.minHeight), mm(a.maxHeight)));
    m_volume->setText(QString("%1 m³").arg(a.volume, 0, 'f', a.volume < 1 ? 3 : 2));
    m_sweep->setText(a.verticalBase ? QString("%1°").arg(std::round(a.sweep)) : QString("— (J1 not vertical)"));
    setPose(a, g);
}

void RobotAnalysisPanel::setPose(const robot::Analysis& a, const SceneGraph& g) {
    if (!a.error.isEmpty()) { setError(a.error); return; }
    capture(a, g);
    m_tcp->setText(QString("TCP  x %1  y %2  z %3 mm\nreach now %4 mm")
                       .arg(mm(a.tcp.x()), mm(a.tcp.y()), mm(a.tcp.z()), mm(a.reachNow)));
    const int pct = int(std::round(a.dexterity * 100));
    m_dex->setValue(std::clamp(pct, a.dexterity > 0 ? 1 : 0, 100));
    m_dexLabel->setText(QString("Dexterity %1%  ·  %2").arg(pct)
                            .arg(a.dexterity < 0.02 ? "singular" : a.dexterity < 0.1 ? "poorly conditioned" : "well conditioned"));
    if (a.warnings.isEmpty()) {
        m_warnings->setObjectName("hint");
        m_warnings->setText("No singularity; every axis is inside its range.");
    } else {
        m_warnings->setObjectName("warningText");
        m_warnings->setText("⚠ " + a.warnings.join("\n⚠ "));
    }
    Theme::repolish(m_warnings);
    fillTable(a);
}

void RobotAnalysisPanel::setError(const QString& text) {
    m_sketch->clear();
    m_sketch->setMessage(text);
    for (QLabel* l : {m_reach, m_minReach, m_height, m_volume, m_sweep}) l->setText("—");
    m_tcp->setText("—");
    m_dex->setValue(0);
    m_dexLabel->clear();
    m_warnings->setText(text);
    m_table->setRowCount(0);
}

void RobotAnalysisPanel::fillTable(const robot::Analysis& a) {
    m_table->setRowCount(int(a.axes.size()));
    auto set = [&](int r, int c, const QString& t, const QString& tip = {}) {
        auto* it = m_table->item(r, c);
        if (!it) { it = new QTableWidgetItem; m_table->setItem(r, c, it); }
        it->setText(t);
        it->setToolTip(tip);
        it->setTextAlignment(c <= C_Type ? Qt::AlignLeft | Qt::AlignVCenter : Qt::AlignRight | Qt::AlignVCenter);
    };
    for (int r = 0; r < int(a.axes.size()); ++r) {
        const robot::AxisInfo& x = a.axes[r];
        const bool last = r == int(a.axes.size()) - 1;
        set(r, C_Axis, x.name);
        set(r, C_Type, x.prismatic ? "Linear" : x.continuous ? "Rotary ∞" : "Rotary");
        set(r, C_Range, x.prismatic ? QString("%1 … %2 mm").arg(mm(x.min), mm(x.max))
                                    : x.continuous ? QString("endless")
                                                   : QString("%1 … %2°").arg(std::round(x.min)).arg(std::round(x.max)));
        set(r, C_A, mm(x.a), last ? "Distance from this axis to the TCP" : QString());
        set(r, C_Alpha, last ? QString("—") : QString::number(std::round(x.alpha * 10) / 10));
        set(r, C_D, mm(x.d), last ? "Along this axis to the TCP" : QString());
        set(r, C_Now, x.prismatic ? QString("%1 mm").arg(mm(x.value)) : QString("%1°").arg(x.value, 0, 'f', 1));
        set(r, C_Speed, x.prismatic ? QString("%1 mm/s").arg(mm(x.speed)) : QString("%1 °/s").arg(std::round(x.speed)));
        set(r, C_Time, x.moveTime > 0 ? QString("%1 s").arg(x.moveTime, 0, 'f', 2) : QString("—"));
    }
}
