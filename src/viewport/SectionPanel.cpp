#include "SectionPanel.h"
#include "ui/Theme.h"
#include <QCheckBox>
#include <QComboBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QVBoxLayout>
#include <QVector4D>
#include <algorithm>

QVector4D SectionPlane::equation() const {
    QVector3D n(axis == 0, axis == 1, axis == 2);
    // Keep the positive side (beyond `position`) unless flipped
    return flip ? QVector4D(-n, position) : QVector4D(n, -position);
}

SectionPanel::SectionPanel(QWidget* parent) : QFrame(parent, Qt::Popup) {
    setObjectName("lightingPanel");
    setFixedWidth(280);
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(14, 12, 14, 12);
    root->setSpacing(10);

    auto* header = new QHBoxLayout;
    header->setSpacing(8);
    auto* icon = new QLabel;
    Theme::setLabelIcon(icon, "scissors", 16);
    auto* title = new QLabel("SECTION VIEW");
    title->setObjectName("islandTitle");
    header->addWidget(icon);
    header->addWidget(title, 1);
    root->addLayout(header);

    m_on = new QCheckBox("Cut the model");
    m_on->setToolTip("Hide everything on one side of a plane to see inside assemblies");
    root->addWidget(m_on);

    auto* grid = new QGridLayout;
    grid->setHorizontalSpacing(10);
    grid->setVerticalSpacing(8);
    grid->setColumnStretch(1, 1);
    auto* axisLabel = new QLabel("Plane");
    axisLabel->setObjectName("fieldLabel");
    m_axis = new QComboBox;
    m_axis->addItems({"X (side to side)", "Y (height)", "Z (front to back)"});
    grid->addWidget(axisLabel, 0, 0);
    grid->addWidget(m_axis, 0, 1, 1, 2);
    auto* posLabel = new QLabel("Position");
    posLabel->setObjectName("fieldLabel");
    m_slider = new QSlider(Qt::Horizontal);
    m_slider->setRange(0, 1000);
    m_value = new QLabel;
    m_value->setObjectName("mono");
    m_value->setMinimumWidth(56);
    m_value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    grid->addWidget(posLabel, 1, 0);
    grid->addWidget(m_slider, 1, 1);
    grid->addWidget(m_value, 1, 2);
    root->addLayout(grid);
    m_flip = new QCheckBox("Keep the other side");
    root->addWidget(m_flip);
    auto* hint = new QLabel("Cut faces show in a flat colour. Moving parts are cut as they move.");
    hint->setObjectName("hint");
    hint->setWordWrap(true);
    root->addWidget(hint);

    connect(m_on, &QCheckBox::toggled, this, &SectionPanel::emitChange);
    connect(m_flip, &QCheckBox::toggled, this, &SectionPanel::emitChange);
    connect(m_axis, &QComboBox::currentIndexChanged, this, [this]{
        if (m_syncing) return;
        // A new axis starts in the middle of the scene
        const int a = m_axis->currentIndex();
        m_plane.position = (m_min[a] + m_max[a]) * 0.5f;
        m_plane.axis = a;
        syncControls();
        emit planeChanged(m_plane);
    });
    connect(m_slider, &QSlider::valueChanged, this, &SectionPanel::emitChange);
    syncControls();
}

void SectionPanel::setBounds(const QVector3D& min, const QVector3D& max) {
    m_min = min;
    m_max = max;
    for (int a = 0; a < 3; ++a)
        if (m_max[a] - m_min[a] < 1e-3f) { m_min[a] -= 0.5f; m_max[a] += 0.5f; }
    syncControls();
}

void SectionPanel::setPlane(const SectionPlane& plane) {
    m_plane = plane;
    syncControls();
}

void SectionPanel::syncControls() {
    m_syncing = true;
    const int a = m_plane.axis;
    m_on->setChecked(m_plane.enabled);
    m_axis->setCurrentIndex(a);
    m_flip->setChecked(m_plane.flip);
    const float span = m_max[a] - m_min[a];
    m_slider->setValue(int(std::clamp((m_plane.position - m_min[a]) / span, 0.0f, 1.0f) * 1000));
    m_value->setText(QString("%1 m").arg(double(m_plane.position), 0, 'f', 3));
    for (QWidget* w : {static_cast<QWidget*>(m_axis), static_cast<QWidget*>(m_slider), static_cast<QWidget*>(m_flip)})
        w->setEnabled(m_plane.enabled);
    m_syncing = false;
}

void SectionPanel::emitChange() {
    if (m_syncing) return;
    const int a = m_axis->currentIndex();
    m_plane.enabled = m_on->isChecked();
    m_plane.axis = a;
    m_plane.flip = m_flip->isChecked();
    m_plane.position = m_min[a] + (m_max[a] - m_min[a]) * float(m_slider->value()) / 1000.0f;
    syncControls();
    emit planeChanged(m_plane);
}
