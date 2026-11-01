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
#include <memory>
#include "Camera.h"
#include "MeshFeatures.h"
#include "LightingPanel.h"
#include "../scene/SceneGraph.h"

struct MeshData {
    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint ebo = 0;     // set for indexed meshes: count is then the index count
    int    count = 0;
};

class Viewport : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core {
    Q_OBJECT
public:
    explicit Viewport(SceneGraph* scene, QWidget* parent = nullptr);
    ~Viewport() override;

    Camera& camera() { return m_camera; }

    // Orthographic (no foreshortening: Front / Top / Side are flat 2D views)
    // or perspective
    void setProjection(Camera::Projection projection);
    Camera::Projection projection() const { return m_camera.projection(); }
    // Frames every object, keeping the view direction
    void fitAll();

    // Axis-pick mode for the joint of `objectId`: hovering any part shows the
    // axis its geometry defines (round edge or cylindrical face → centre line,
    // straight edge → the edge, flat face → its normal) and a click sets the
    // joint's axis and rotation origin. Clicking one of the translate gizmo's
    // arrows still picks that world axis, keeping the origin.
    void beginAxisPick(int objectId);
    void cancelAxisPick();

    // Simulation feedback: tint parts driven by a moving actuator and list
    // the running automation operations in the top-left corner.
    void setSimulationOverlay(const QSet<int>& movingObjects, const QStringList& lines);

signals:
    void keyPointSelected(const QString& name, const QVector3D& worldPos);
    // Axis (and, when `setPivot`, rotation origin) in the parent's local frame
    void axisPicked(int objectId, const QVector3D& localAxis, const QVector3D& localPivot, bool setPivot);

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
    void drawPill(QPainter& p, const QPointF& topLeft, const QString& text, const QColor& border = QColor());
    void drawPhysicsOverlays();
    void drawKeyPoints (QPainter& p);
    void drawAxisPickBanner(QPainter& p);
    // The selected object's joint axis and, in axis-pick mode, the hovered candidate
    void drawJointAxes(QPainter& p);
    void drawAxisArrow(QPainter& p, const QVector3D& origin, const QVector3D& axis, bool revolute,
                       const QColor& color);
    bool toScreen(const QVector3D& world, QPointF* out) const;
    void drawWireLines(const std::vector<float>& verts, const QVector3D& color,
                       const QMatrix4x4& model, float alpha = 1.0f);
    void repositionOverlays();
    // After every camera move: the view cube turns and the view repaints
    void cameraChanged();
    // Frames every object with the given camera (its rotation is kept)
    void frameScene(Camera* camera) const;
    // Animates the camera to `rotation`, framing everything too when `fit`
    void animateTo(const QQuaternion& rotation, bool fit);

    void tryPickObject(int x, int y);
    std::pair<int,float> pickNearest(const QVector3D& orig, const QVector3D& dir) const;
    void updateHover  (int x, int y);
    bool rayAABB(const QVector3D& orig, const QVector3D& dir,
                 const QVector3D& mn,  const QVector3D& mx,  float& t);

    SceneGraph*            m_scene;
    Camera                 m_camera;
    class ViewCube*        m_viewCube = nullptr;
    class QVariantAnimation* m_viewAnimation = nullptr;
    Camera                 m_animationFrom, m_animationTo;
    QOpenGLShaderProgram*  m_objShader  = nullptr;
    QOpenGLShaderProgram*  m_gridShader = nullptr;
    QOpenGLShaderProgram*  m_wireShader = nullptr;

    MeshData m_cube, m_cylinder, m_sphere, m_capsule, m_basket;

    // GPU meshes for ImportedMesh objects, keyed by MeshGeometry pointer so
    // assembly instances sharing one geometry also share one VBO. Uploaded
    // lazily (welded and indexed) in drawScene with the GL context current.
    // Buffers live as long as their geometry does, so undo/redo, which swaps
    // the scene but keeps the same geometry, uploads nothing; the weak
    // pointer notices a freed geometry (and an address that was reused).
    struct ImportedMesh {
        std::weak_ptr<MeshGeometry> geometry;
        MeshData                    gpu;
    };
    std::unordered_map<const MeshGeometry*, ImportedMesh> m_importedMeshes;
    bool m_sweepMeshes = false;   // geometry may have been freed: sweep on next paint
    const MeshData* meshFor(const SceneObject& obj);
    void uploadIndexedMesh(MeshData& m, const std::vector<float>& soup);
    void deleteMesh(MeshData& m);
    // Frees GPU buffers and feature caches of geometry no longer alive
    // (GL context must be current)
    void sweepFreedMeshes();
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
    QPushButton* m_btnLight  = nullptr;

    // Shading settings for the active theme, edited in m_lightPanel
    ViewportLighting m_light;
    LightingPanel*   m_lightPanel = nullptr;

    // Axis-pick mode (see beginAxisPick/cancelAxisPick)
    int          m_axisPickId     = -1;
    // The axis under the cursor, in world space (objectId -1: nothing hovered)
    struct AxisCandidate {
        int objectId = -1;
        AxisFeature::Kind kind = AxisFeature::Kind::None;
        QVector3D origin, axis;
        float radius = 0.0f;
        std::vector<QVector3D> outline;
    };
    AxisCandidate m_axisCandidate;
    void updateAxisCandidate(int x, int y);
    void commitAxisCandidate();

    // Faces and edges recognised on each part's mesh, built on first use.
    // The weak pointer notices a freed geometry whose address was reused.
    struct FeatureCache {
        std::weak_ptr<MeshGeometry>   geometry;
        std::unique_ptr<MeshFeatures> features;
    };
    std::unordered_map<const MeshGeometry*, FeatureCache> m_meshFeatures;
    std::vector<float>            m_primitiveVertices[6];   // by PrimitiveType, kept by make*()
    std::unique_ptr<MeshFeatures> m_primitiveFeatures[6];
    const MeshFeatures* featuresFor(const SceneObject& obj);

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
