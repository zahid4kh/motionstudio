#pragma once
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QPainter>
#include <QPushButton>
#include <QPoint>
#include <vector>
#include <QSet>
#include <QStringList>
#include <unordered_map>
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

    // Axis-pick mode: click one of the translate gizmo's arrows (drawn along
    // world X/Y/Z at the object's origin) to set a joint's rotation axis
    // instead of dragging. No isolation needed — the gizmo is already shown
    // for the selected object.
    void beginAxisPick(int objectId);
    void cancelAxisPick();

    // Simulation feedback: tint parts driven by a moving actuator and list
    // the running automation operations in the top-left corner.
    void setSimulationOverlay(const QSet<int>& movingObjects, const QStringList& lines);

signals:
    void keyPointSelected(const QString& name, const QVector3D& worldPos);
    void axisPicked(int objectId, const QVector3D& localAxis);

protected:
    void initializeGL()          override;
    void resizeGL(int w, int h)  override;
    void paintGL()               override;
    void mousePressEvent  (QMouseEvent*)  override;
    void mouseMoveEvent   (QMouseEvent*)  override;
    void mouseReleaseEvent(QMouseEvent*)  override;
    void wheelEvent       (QWheelEvent*)  override;
    void keyPressEvent    (QKeyEvent*)    override;

private:
    struct KeyPoint { QString name; QVector3D localPos; };
    static std::vector<KeyPoint> keyPointsFor(const SceneObject& obj);

    void buildShaders();
    void buildMeshes();
    void buildGrid();
    void buildAxisGizmo();
    void buildWireResources();

    MeshData makeCube();
    MeshData makeCylinder(int segs = 20);
    MeshData makeSphere  (int rings = 16, int sectors = 24);
    MeshData makeCapsule (int rings = 8,  int sectors = 20);
    MeshData makeBasket  ();

    void uploadMesh(MeshData& m, const std::vector<float>& v);
    void drawMesh (const MeshData& m);
    void drawScene();
    void drawGrid();
    void drawAxisGizmo();
    void drawAxisLabels(QPainter& p);
    void drawKeyHints  (QPainter& p);
    void drawPhysicsOverlays();
    void drawKeyPoints (QPainter& p);
    void drawAxisPickBanner(QPainter& p);
    void drawWireLines(const std::vector<float>& verts, const QVector3D& color,
                       const QMatrix4x4& model, float alpha = 1.0f);
    void repositionOverlays();

    void tryPickObject(int x, int y);
    void updateHover  (int x, int y);
    bool rayAABB(const QVector3D& orig, const QVector3D& dir,
                 const QVector3D& mn,  const QVector3D& mx,  float& t);

    SceneGraph*            m_scene;
    Camera                 m_camera;
    QOpenGLShaderProgram*  m_objShader  = nullptr;
    QOpenGLShaderProgram*  m_gridShader = nullptr;
    QOpenGLShaderProgram*  m_wireShader = nullptr;

    MeshData m_cube, m_cylinder, m_sphere, m_capsule, m_basket;

    // GPU meshes for ImportedMesh objects, keyed by MeshGeometry pointer so
    // assembly instances sharing one geometry also share one VBO.
    // Uploaded lazily in drawScene (GL context current), freed on scene clear.
    std::unordered_map<const MeshGeometry*, MeshData> m_importedMeshes;
    const MeshData* meshFor(const SceneObject& obj);
    void releaseImportedMeshes();
    GLuint   m_gridVao = 0, m_gridVbo = 0;
    GLuint   m_axisVao = 0, m_axisVbo = 0;
    GLuint   m_wireVao = 0, m_wireVbo = 0;
    GLuint   m_floorVao = 0, m_floorVbo = 0;
    int      m_gridLines = 0;

    // Overlay view-snap + floor buttons
    QPushButton* m_btnFront  = nullptr;
    QPushButton* m_btnTop    = nullptr;
    QPushButton* m_btnSide   = nullptr;
    QPushButton* m_btnFloor  = nullptr;

    // Axis-pick mode (see beginAxisPick/cancelAxisPick)
    int          m_axisPickId     = -1;

    // Gizmo: set in drawAxisGizmo(), read in drawAxisLabels()
    QMatrix4x4 m_gizmoView, m_gizmoProj;
    static constexpr int kGizmoSize   = 80;
    static constexpr int kGizmoMargin = 14;

    int              m_hoveredId = -1;
    QSet<int>        m_movingObjects;
    QStringList      m_overlayLines;
    class QLabel*    m_simOverlay = nullptr;   // a widget, not QPainter text (renders reliably over GL)
    int              m_nearKeyPointIdx = -1;  // index into keyPointsFor(hovered obj type)
    QPoint           m_lastMouse;
    Qt::MouseButton  m_dragBtn = Qt::NoButton;
    int              m_vpW = 1, m_vpH = 1;

    // Translate gizmo
    int       m_gizmoHoveredAxis  = -1;   // 0=X, 1=Y, 2=Z
    int       m_gizmoDragAxis     = -1;
    float     m_gizmoDragStartT   = 0.f;
    QVector3D m_gizmoDragStartPos;        // local position at drag start
    QVector3D m_gizmoDragWorldCenter;     // world center at drag start (fixed reference)

    void  drawTranslateGizmo();
    int   hitTestGizmoAxis(int mx, int my) const;
    float projectOntoAxis(int axis, const QVector3D& center, int mx, int my) const;
    float gizmoArrowLength(int objectId) const;
};
