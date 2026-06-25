#pragma once
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QPainter>
#include <QPoint>
#include "Camera.h"
#include "../scene/SceneGraph.h"

struct MeshData {
    GLuint vao = 0;
    GLuint vbo = 0;
    int    count = 0;
};

class Viewport : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core {
    Q_OBJECT
public:
    explicit Viewport(SceneGraph* scene, QWidget* parent = nullptr);
    ~Viewport() override;

    Camera& camera() { return m_camera; }

protected:
    void initializeGL()          override;
    void resizeGL(int w, int h)  override;
    void paintGL()               override;
    void mousePressEvent  (QMouseEvent*)  override;
    void mouseMoveEvent   (QMouseEvent*)  override;
    void mouseReleaseEvent(QMouseEvent*)  override;
    void wheelEvent       (QWheelEvent*)  override;

private:
    void buildShaders();
    void buildMeshes();
    void buildGrid();
    void buildAxisGizmo();

    MeshData makeCube();
    MeshData makeCylinder(int segs = 20);
    MeshData makeSphere  (int rings = 16, int sectors = 24);
    MeshData makeCapsule (int rings = 8,  int sectors = 20);

    void uploadMesh(MeshData& m, const std::vector<float>& v);
    void drawMesh (const MeshData& m);
    void drawScene();
    void drawGrid();
    void drawAxisGizmo();
    void drawAxisLabels(QPainter& p);
    void drawKeyHints  (QPainter& p);

    void tryPickObject(int x, int y);
    void updateHover  (int x, int y);
    bool rayAABB(const QVector3D& orig, const QVector3D& dir,
                 const QVector3D& mn,  const QVector3D& mx,  float& t);

    SceneGraph*            m_scene;
    Camera                 m_camera;
    QOpenGLShaderProgram*  m_objShader  = nullptr;
    QOpenGLShaderProgram*  m_gridShader = nullptr;

    MeshData m_cube, m_cylinder, m_sphere, m_capsule;
    GLuint   m_gridVao = 0, m_gridVbo = 0;
    GLuint   m_axisVao = 0, m_axisVbo = 0;
    int      m_gridLines = 0;

    // Gizmo: set in drawAxisGizmo(), read in drawAxisLabels()
    QMatrix4x4 m_gizmoView, m_gizmoProj;
    static constexpr int kGizmoSize   = 80;
    static constexpr int kGizmoMargin = 14;

    int              m_hoveredId = -1;
    QPoint           m_lastMouse;
    Qt::MouseButton  m_dragBtn = Qt::NoButton;
    int              m_vpW = 1, m_vpH = 1;
};
