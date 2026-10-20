#include "LightingPanel.h"
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSettings>
#include <QSlider>
#include <QVBoxLayout>
#include <QtMath>

namespace {

QString modeKey(Theme::Mode mode) {
    return mode == Theme::Mode::Dark ? "dark" : "light";
}

QVector3D fromAngles(float azimuthDeg, float elevationDeg) {
    const float a = qDegreesToRadians(azimuthDeg), e = qDegreesToRadians(elevationDeg);
    return QVector3D(std::cos(e) * std::cos(a), std::sin(e), std::cos(e) * std::sin(a)).normalized();
}

} // namespace

QVector3D ViewportLighting::keyDirection() const {
    return fromAngles(azimuth, elevation);
}

QVector3D ViewportLighting::fillDirection() const {
    return fromAngles(azimuth + 180.0f, 20.0f);
}

ViewportLighting ViewportLighting::load(Theme::Mode mode) {
    QSettings s("MotionStudio", "MotionStudio");
    s.beginGroup("viewport/lighting/" + modeKey(mode));
    ViewportLighting d, l;
    l.ambient   = s.value("ambient",   d.ambient).toFloat();
    l.key       = s.value("key",       d.key).toFloat();
    l.fill      = s.value("fill",      d.fill).toFloat();
    l.specular  = s.value("specular",  d.specular).toFloat();
    l.azimuth   = s.value("azimuth",   d.azimuth).toFloat();
    l.elevation = s.value("elevation", d.elevation).toFloat();
    return l;
}

void ViewportLighting::save(Theme::Mode mode) const {
    QSettings s("MotionStudio", "MotionStudio");
    s.beginGroup("viewport/lighting/" + modeKey(mode));
    s.setValue("ambient",   ambient);
    s.setValue("key",       key);
    s.setValue("fill",      fill);
    s.setValue("specular",  specular);
    s.setValue("azimuth",   azimuth);
    s.setValue("elevation", elevation);
}

LightingPanel::LightingPanel(QWidget* parent) : QFrame(parent, Qt::Popup) {
    setObjectName("lightingPanel");
    setFixedWidth(280);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(14, 12, 14, 12);
    root->setSpacing(10);

    auto* header = new QHBoxLayout;
    header->setSpacing(8);
    auto* icon = new QLabel;
    Theme::setLabelIcon(icon, "lightbulb", 16);
    m_title = new QLabel;
    m_title->setObjectName("islandTitle");
    header->addWidget(icon);
    header->addWidget(m_title, 1);
    root->addLayout(header);

    auto* grid = new QGridLayout;
    grid->setHorizontalSpacing(10);
    grid->setVerticalSpacing(8);
    grid->setColumnStretch(1, 1);
    addRow(grid, "Ambient",   &ViewportLighting::ambient,   0, 100, 0.01f, "%");
    addRow(grid, "Key light", &ViewportLighting::key,       0, 150, 0.01f, "%");
    addRow(grid, "Fill light",&ViewportLighting::fill,      0, 100, 0.01f, "%");
    addRow(grid, "Specular",  &ViewportLighting::specular,  0, 100, 0.01f, "%");
    addRow(grid, "Direction", &ViewportLighting::azimuth,   0, 359, 1.0f,  "°");
    addRow(grid, "Height",    &ViewportLighting::elevation, 5,  90, 1.0f,  "°");
    root->addLayout(grid);

    auto* reset = new QPushButton(Theme::icon("rotate-ccw"), "Reset to default");
    connect(reset, &QPushButton::clicked, this, [this]{
        setLighting(ViewportLighting{});
        emit lightingChanged(m_light);
    });
    root->addWidget(reset);

    connect(Theme::notifier(), &Theme::Notifier::changed, this, [this]{ syncControls(); });
    syncControls();
}

void LightingPanel::addRow(QGridLayout* grid, const QString& label, float ViewportLighting::* field,
                           int min, int max, float scale, const QString& unit) {
    const int r = int(m_rows.size());
    auto* name = new QLabel(label);
    name->setObjectName("fieldLabel");
    auto* slider = new QSlider(Qt::Horizontal);
    slider->setRange(min, max);
    auto* value = new QLabel;
    value->setObjectName("mono");
    value->setMinimumWidth(40);
    value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    grid->addWidget(name,   r, 0);
    grid->addWidget(slider, r, 1);
    grid->addWidget(value,  r, 2);

    m_rows.append({slider, value, field, scale, unit});
    connect(slider, &QSlider::valueChanged, this, [this, slider, value, field, scale, unit](int v){
        value->setText(QString::number(v) + unit);
        if (m_syncing) return;
        m_light.*field = v * scale;
        emit lightingChanged(m_light);
    });
}

void LightingPanel::setLighting(const ViewportLighting& l) {
    m_light = l;
    syncControls();
}

void LightingPanel::syncControls() {
    m_title->setText(Theme::mode() == Theme::Mode::Dark ? "LIGHTING · DARK THEME" : "LIGHTING · LIGHT THEME");
    m_syncing = true;
    for (const Row& row : m_rows) {
        const int v = qRound(m_light.*(row.field) / row.scale);
        row.slider->setValue(v);
        row.value->setText(QString::number(row.slider->value()) + row.unit);
    }
    m_syncing = false;
}
