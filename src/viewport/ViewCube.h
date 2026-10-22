#pragma once

#include <QQuaternion>
#include <QVector3D>
#include <QWidget>

// The view cube in a corner of the 3D view (Fusion 360 style): a cube turned like the model, with labelled
// faces. Clicking a face, edge or corner looks at the model from that side (corners give isometric views);
// dragging the cube orbits. When looking straight at a face, triangles next to the cube turn to the
// neighbouring face and two curved arrows roll the view by 90 degrees. The house goes back to the home view.
// The widget only asks for camera rotations; the viewport animates to them.
// Ported from wowcad, with the faces laid out for MotionStudio's Y-up world
// (front looks along -Z).
class ViewCube : public QWidget
{
    Q_OBJECT
public:
    explicit ViewCube(QWidget *parent = nullptr);

    // The camera's rotation (camera right / up / back as X / Y / Z, see Camera).
    void setCameraRotation(const QQuaternion &rotation);

signals:
    // `fit`: frame the model too (standard views), not just turn (arrows).
    void rotationRequested(const QQuaternion &rotation, bool fit);
    void homeRequested();
    void orbitRequested(float dxPixels, float dyPixels);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void leaveEvent(QEvent *event) override;
    bool event(QEvent *event) override;

private:
    enum class Part { None, Cube, Up, Down, Left, Right, RollClockwise, RollCounterClockwise, Home };
    struct Hit
    {
        Part part = Part::None;
        QVector3D direction; // Cube: the face / edge / corner, a sum of face normals
        bool operator==(const Hit &o) const { return part == o.part && direction == o.direction; }
        bool operator!=(const Hit &o) const { return !(*this == o); }
    };

    Hit hitAt(const QPointF &pos) const;
    // Looking straight at a face (with an axis pointing up)?
    bool faceView() const;
    QPointF centre() const;
    QPointF toScreen(const QVector3D &v) const; // offset from the centre of a cube-space vector
    QPolygonF arrow(Part part) const;
    QRectF iconRect(Part part) const;
    void activate(const Hit &hit);
    void setHover(const Hit &hit);

    QVector3D m_right{1, 0, 0}, m_up{0, 1, 0}, m_back{0, 0, 1};
    Hit m_hover;
    Hit m_pressed;
    QPoint m_pressPos, m_lastPos;
    bool m_dragging = false;
};
