#pragma once
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector3D>

// Orbit camera for the Y-up world. The eye sits on a sphere around the target;
// its orientation is a full rotation (camera right / up / back as the X / Y / Z
// axes), so the standard views are exact and can be rolled.
// Perspective or orthographic: an orthographic view shows the size the
// perspective one has at the target, so switching keeps the model in place.
class Camera {
public:
    enum class Projection { Perspective, Orthographic };

    Camera();

    // Mouse deltas are in pixels. Orbiting turns around world Y and the
    // camera's right axis; an upright camera stops at straight above / below.
    void orbit(float dxPixels, float dyPixels);
    void pan(float dxPixels, float dyPixels, float viewportHeight);
    // Zooms so the point under the cursor stays put (ndc in [-1, 1], y up).
    void zoomAt(float steps, float ndcX, float ndcY, float aspect);
    void reset();
    // Centers the target on `center` and picks a distance so a sphere of the
    // given radius fits within the view. The rotation is kept.
    void frame(const QVector3D& center, float radius);

    QMatrix4x4 viewMatrix()                   const;
    QMatrix4x4 projectionMatrix(float aspect) const;
    // The perspective eye (orthographic views have none; this is still where
    // the orbit centre is viewed from, which is what the shading uses).
    QVector3D  position()                     const;

    // Ray through a screen point (ndc in [-1, 1], y up); `dir` is normalised.
    // Orthographic rays are parallel and start far behind the target, so
    // everything in front of the view lies ahead of them.
    void ray(float ndcX, float ndcY, float aspect, QVector3D* origin, QVector3D* dir) const;

    Projection projection() const { return m_projection; }
    void setProjection(Projection p) { m_projection = p; }
    QQuaternion rotation() const { return m_rotation; }
    void setRotation(const QQuaternion& r) { m_rotation = r.normalized(); }
    QVector3D target() const { return m_target; }
    void setTarget(const QVector3D& t) { m_target = t; }
    float distance() const { return m_distance; }
    void setDistance(float d);

    QVector3D forward() const;   // the viewing direction
    QVector3D right()   const;
    QVector3D up()      const;

    // The rotation that looks along -back with `up` (made perpendicular to it)
    // pointing up on screen.
    static QQuaternion lookRotation(const QVector3D& back, const QVector3D& up);
    // The home view: isometric from the front-right-top corner.
    static QQuaternion homeRotation();

private:
    static constexpr float kFov = 45.0f;

    QVector3D   m_target{0.0f, 0.0f, 0.0f};
    float       m_distance = 10.0f;
    QQuaternion m_rotation;
    Projection  m_projection = Projection::Orthographic;
};
