#pragma once
#include <QFrame>
#include <QVector3D>

class QCheckBox;
class QComboBox;
class QSlider;
class QLabel;

// A section view: everything on one side of a plane square to a world axis
// is cut away, and the inside of cut parts is shown in a flat colour
struct SectionPlane {
    bool  enabled  = false;
    int   axis     = 0;        // 0 = X, 1 = Y, 2 = Z
    float position = 0.0f;     // world coordinate along the axis, m
    bool  flip     = false;    // keep the other side
    // Plane (n, d) with dot(n, p) + d >= 0 on the side that is kept
    QVector4D equation() const;
};

// Popup behind the viewport's Section button
class SectionPanel : public QFrame {
    Q_OBJECT
public:
    explicit SectionPanel(QWidget* parent = nullptr);
    // The scene's extent along each axis, for the position slider
    void setBounds(const QVector3D& min, const QVector3D& max);
    void setPlane(const SectionPlane& plane);
    const SectionPlane& plane() const { return m_plane; }

signals:
    void planeChanged(const SectionPlane& plane);

private:
    void syncControls();
    void emitChange();

    SectionPlane m_plane;
    QVector3D    m_min{-1, -1, -1}, m_max{1, 1, 1};
    QCheckBox*   m_on = nullptr;
    QComboBox*   m_axis = nullptr;
    QSlider*     m_slider = nullptr;
    QLabel*      m_value = nullptr;
    QCheckBox*   m_flip = nullptr;
    bool         m_syncing = false;
};
