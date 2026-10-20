#pragma once
#include <QFrame>
#include <QVector3D>
#include "ui/Theme.h"

class QSlider;
class QLabel;

// How the viewport shades objects. Each theme keeps its own settings, so a
// brighter dark-theme setup leaves the light theme alone.
struct ViewportLighting {
    float ambient   = 0.18f;   // light everywhere, also on faces turned away
    float key       = 0.72f;   // main directional light
    float fill      = 0.00f;   // softer light from the opposite side
    float specular  = 0.40f;   // highlight strength
    float azimuth   = 55.0f;   // key light direction around Y, degrees
    float elevation = 49.0f;   // key light height above the floor, degrees

    // Unit vectors pointing from the surface towards each light
    QVector3D keyDirection() const;
    QVector3D fillDirection() const;

    static ViewportLighting load(Theme::Mode mode);
    void save(Theme::Mode mode) const;
};

// Popup with a slider per lighting setting, opened from the viewport's
// Lighting button. Edits apply live and are saved for the active theme.
class LightingPanel : public QFrame {
    Q_OBJECT
public:
    explicit LightingPanel(QWidget* parent = nullptr);

    void setLighting(const ViewportLighting& l);
    const ViewportLighting& lighting() const { return m_light; }

signals:
    void lightingChanged(const ViewportLighting& l);

private:
    struct Row { QSlider* slider; QLabel* value; float ViewportLighting::* field; float scale; QString unit; };

    void addRow(class QGridLayout* grid, const QString& label, float ViewportLighting::* field,
                int min, int max, float scale, const QString& unit);
    void syncControls();

    ViewportLighting m_light;
    QLabel*          m_title = nullptr;
    QList<Row>       m_rows;
    bool             m_syncing = false;
};
