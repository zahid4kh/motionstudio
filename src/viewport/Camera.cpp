#include "Camera.h"
#include <QtMath>
#include <algorithm>
#include <cmath>

namespace {
constexpr float kOrbitDegreesPerPixel = 0.5f;
constexpr float kMinDistance = 0.05f;
constexpr float kMaxDistance = 500.0f;
constexpr float kZoomFactorPerStep = 0.9f;
// How far in front of and behind the target an orthographic view sees
constexpr float kOrthoDepth = 1000.0f;
} // namespace

Camera::Camera() { reset(); }

QQuaternion Camera::lookRotation(const QVector3D& back, const QVector3D& up) {
    const QVector3D b = back.normalized();
    QVector3D u = up - b * QVector3D::dotProduct(up, b);
    if (u.lengthSquared() < 1e-12f)   // up along back: any perpendicular will do
        u = std::abs(b.y()) < 0.9f ? QVector3D(0, 1, 0) - b * b.y() : QVector3D(0, 0, -1) + b * b.z();
    u.normalize();
    return QQuaternion::fromAxes(QVector3D::crossProduct(u, b), u, b);
}

QQuaternion Camera::homeRotation() {
    return lookRotation(QVector3D(1, 1, 1), QVector3D(0, 1, 0));
}

void Camera::reset() {
    m_target   = {0, 0, 0};
    m_distance = 10.0f;
    m_rotation = homeRotation();
}

QVector3D Camera::forward() const { return -m_rotation.rotatedVector(QVector3D(0, 0, 1)); }
QVector3D Camera::right()   const { return  m_rotation.rotatedVector(QVector3D(1, 0, 0)); }
QVector3D Camera::up()      const { return  m_rotation.rotatedVector(QVector3D(0, 1, 0)); }

QVector3D Camera::position() const {
    return m_target - forward() * m_distance;
}

void Camera::setDistance(float d) {
    m_distance = std::clamp(d, kMinDistance, kMaxDistance);
}

void Camera::orbit(float dx, float dy) {
    m_rotation = (QQuaternion::fromAxisAndAngle(QVector3D(0, 1, 0), -dx * kOrbitDegreesPerPixel) * m_rotation)
                     .normalized();
    float pitch = dy * kOrbitDegreesPerPixel;   // positive raises the eye
    const QVector3D r = right(), u = up(), back = -forward();
    // An upright camera (not rolled, not upside down) stops at straight
    // above / below instead of tipping over.
    if (std::abs(r.y()) < 1e-3f && u.y() > -1e-3f) {
        const float elevation = qRadiansToDegrees(std::asin(std::clamp(back.y(), -1.0f, 1.0f)));
        pitch = std::clamp(pitch, -90.0f - elevation, 90.0f - elevation);
    }
    m_rotation = (QQuaternion::fromAxisAndAngle(r, -pitch) * m_rotation).normalized();
}

void Camera::pan(float dx, float dy, float viewportHeight) {
    if (viewportHeight <= 0.0f) return;
    // World units per pixel on the plane through the target
    const float wpp = 2.0f * m_distance * std::tan(qDegreesToRadians(kFov) * 0.5f) / viewportHeight;
    m_target += (-right() * dx + up() * dy) * wpp;
}

void Camera::zoomAt(float steps, float ndcX, float ndcY, float aspect) {
    const float halfHeight = std::tan(qDegreesToRadians(kFov) * 0.5f);
    // Offset of the cursor's point on the plane through the target, per unit
    // of distance (the same for both projections)
    const QVector3D offsetPerUnit = right() * (ndcX * halfHeight * aspect) + up() * (ndcY * halfHeight);
    const float newDistance = std::clamp(m_distance * std::pow(kZoomFactorPerStep, steps),
                                         kMinDistance, kMaxDistance);
    m_target  += offsetPerUnit * (m_distance - newDistance);
    m_distance = newDistance;
}

void Camera::frame(const QVector3D& center, float radius) {
    m_target = center;
    radius   = std::max(radius, 0.05f);
    // Distance so the bounding sphere fits comfortably within the vertical FOV.
    setDistance(radius / std::sin(qDegreesToRadians(kFov * 0.5f)) * 1.2f);
}

QMatrix4x4 Camera::viewMatrix() const {
    QMatrix4x4 v;
    v.lookAt(position(), m_target, up());
    return v;
}

QMatrix4x4 Camera::projectionMatrix(float aspect) const {
    QMatrix4x4 p;
    if (m_projection == Projection::Orthographic) {
        const float h = m_distance * std::tan(qDegreesToRadians(kFov) * 0.5f);
        // Depth is measured from the eye, which sits m_distance in front of the target
        p.ortho(-h * aspect, h * aspect, -h, h, m_distance - kOrthoDepth, m_distance + kOrthoDepth);
    } else {
        p.perspective(kFov, aspect, std::clamp(m_distance * 0.01f, 0.005f, 0.1f), 1000.0f);
    }
    return p;
}

void Camera::ray(float ndcX, float ndcY, float aspect, QVector3D* origin, QVector3D* dir) const {
    const float halfHeight = std::tan(qDegreesToRadians(kFov) * 0.5f);
    const QVector3D offset = right() * (ndcX * halfHeight * aspect) + up() * (ndcY * halfHeight);
    if (m_projection == Projection::Orthographic) {
        *dir    = forward();
        *origin = m_target + offset * m_distance - forward() * kOrthoDepth;
    } else {
        *origin = position();
        *dir    = (forward() + offset).normalized();
    }
}

float Camera::worldPerPixel(const QVector3D& point, float viewportHeight) const {
    const float depth = m_projection == Projection::Orthographic
                            ? m_distance
                            : std::max(QVector3D::dotProduct(point - position(), forward()), 1e-3f);
    return 2.0f * depth * std::tan(qDegreesToRadians(kFov) * 0.5f) / std::max(viewportHeight, 1.0f);
}
