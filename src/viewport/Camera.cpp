#include "Camera.h"
#include <QtMath>
#include <algorithm>

Camera::Camera() { reset(); }

void Camera::orbit(float dx, float dy) {
    m_azimuth   += dx * 0.5f;
    m_elevation += dy * 0.5f;
    m_elevation  = std::clamp(m_elevation, -89.0f, 89.0f);
}

void Camera::pan(float dx, float dy) {
    QMatrix4x4 v = viewMatrix();
    QVector3D right(v.row(0).x(), v.row(0).y(), v.row(0).z());
    QVector3D up   (v.row(1).x(), v.row(1).y(), v.row(1).z());
    float speed = m_distance * 0.002f;
    m_target -= right * dx * speed;
    m_target += up    * dy * speed;
}

void Camera::zoom(float delta) {
    m_distance -= delta * m_distance * 0.1f;
    m_distance  = std::max(0.5f, m_distance);
}

void Camera::reset() {
    m_target    = {0, 0, 0};
    m_distance  = 10.0f;
    m_azimuth   = 45.0f;
    m_elevation = 30.0f;
}

QVector3D Camera::position() const {
    float el = qDegreesToRadians(m_elevation);
    float az = qDegreesToRadians(m_azimuth);
    return m_target + QVector3D(
        m_distance * qCos(el) * qSin(az),
        m_distance * qSin(el),
        m_distance * qCos(el) * qCos(az)
    );
}

QMatrix4x4 Camera::viewMatrix() const {
    QMatrix4x4 v;
    v.lookAt(position(), m_target, {0, 1, 0});
    return v;
}

QMatrix4x4 Camera::projectionMatrix(float aspect) const {
    QMatrix4x4 p;
    p.perspective(m_fov, aspect, m_near, m_far);
    return p;
}

QVector3D Camera::rayFromNDC(float ndcX, float ndcY, float aspect) const {
    QMatrix4x4 invVP = (projectionMatrix(aspect) * viewMatrix()).inverted();
    QVector4D nearPt = invVP * QVector4D(ndcX, ndcY, -1.0f, 1.0f);
    QVector4D farPt  = invVP * QVector4D(ndcX, ndcY,  1.0f, 1.0f);
    return (farPt.toVector3DAffine() - nearPt.toVector3DAffine()).normalized();
}
