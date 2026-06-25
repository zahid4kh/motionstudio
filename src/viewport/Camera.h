#pragma once
#include <QMatrix4x4>
#include <QVector3D>

class Camera {
public:
    Camera();

    void orbit(float deltaX, float deltaY);
    void pan(float deltaX, float deltaY);
    void zoom(float delta);
    void reset();

    QMatrix4x4 viewMatrix()                   const;
    QMatrix4x4 projectionMatrix(float aspect) const;
    QVector3D  position()                     const;

    // Returns world-space ray direction from a screen NDC point
    QVector3D rayFromNDC(float ndcX, float ndcY, float aspect) const;

private:
    QVector3D m_target{0.0f, 0.0f, 0.0f};
    float m_distance  = 10.0f;
    float m_azimuth   = 45.0f;   // degrees, horizontal
    float m_elevation = 30.0f;   // degrees, vertical
    float m_fov       = 45.0f;
    float m_near      = 0.1f;
    float m_far       = 1000.0f;
};
