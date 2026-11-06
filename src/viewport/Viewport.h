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
#include "SectionPanel.h"
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

    // Renders the scene offscreen at `size` pixels (any size, independent of
    // the widget's), multisampled, for video export: the current camera
    // widened so everything the viewport shows fits the new shape (or the
    // whole scene with `fitScene`). No selection, hover, gizmo or edit-mode
    // overlays; sensor labels and the axis triad are kept.
    QImage renderImage(const QSize& size, bool fitScene = false);
    // The camera renderImage uses for that size
    Camera exportCamera(const QSize& size, bool fitScene) const;

    // Axis-pick mode for the joint of `objectId`: hovering any part shows the
    // axis its geometry defines (round edge or cylindrical face → centre line,
    // straight edge → the edge, flat face → its normal) and a click sets the
    // joint's axis and rotation origin. Clicking one of the translate gizmo's
    // arrows still picks that world axis, keeping the origin.
    void beginAxisPick(int objectId);
    void cancelAxisPick();

    // Simulation feedback: tint parts driven by a moving actuator and list
    // the running automation operations in the top-left corner.
    void setSimulationOverlay(const QSet<int>& movingObjects, const QStringList& lines,
                              const QSet<int>& interfering = {});

    // IK drag: with the IK button on, dragging the gizmo of an object that
    // actuated joints move (a robot's tool) moves the object by inverse
    // kinematics instead of editing its transform. Off while simulating.
    void setIkAvailable(bool available);

    // Measure mode: click two points on parts for their distance (and the
    // angle between the surfaces); snaps to hole centres and corners
    void setMeasureMode(bool on);
    bool measureMode() const { return m_measureMode; }

    // Assemble mode: click a face, edge or hole on the part to move, then one
    // on the part it goes against; a mate (flush, concentric, insert …) is
    // made and solved, and edited in a panel. Off while simulating.
    void setAssembleMode(bool on);
    bool assembleMode() const { return m_assembleMode; }
    // While false (simulation running) assemble mode is unavailable and
    // dragging a part doesn't re-solve its mates
    void setAssemblyAvailable(bool available);

signals:
    void keyPointSelected(const QString& name, const QVector3D& worldPos);
    // Axis (and, when `setPivot`, rotation origin) in the parent's local frame
    void axisPicked(int objectId, const QVector3D& localAxis, const QVector3D& localPivot, bool setPivot);
    void measureModeChanged(bool on);
    void assembleModeChanged(bool on);

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

    void uploadMesh(MeshData& m, const std::vector<float>& v);
    void drawMesh (const MeshData& m);
    void drawScene();
    void drawGrid();
    void drawAxisGizmo();
    void drawAxisLabels(QPainter& p);
    void drawKeyHints  (QPainter& p);
    void drawPill(QPainter& p, const QPointF& topLeft, const QString& text, const QColor& border = QColor());
    void drawPhysicsOverlays();
    void drawSensors();
    void drawSensorLabels(QPainter& p);
    void drawKeyPoints (QPainter& p);
    void drawAxisPickBanner(QPainter& p);
    void drawMeasure(QPainter& p);
    void drawAssemble(QPainter& p);
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
    // Below 50% opacity a part is clicked through to the solid part behind it
    bool seeThrough(int id) const;
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
    QPushButton* m_btnIk     = nullptr;
    QPushButton* m_btnSection = nullptr;
    SectionPanel* m_sectionPanel = nullptr;
    SectionPlane m_section;
    void drawSectionOutline();
    // World-space bounds of everything with geometry
    void sceneBounds(QVector3D* min, QVector3D* max) const;
    bool         m_ikAvailable = true;
    bool ikDragActive() const;

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
    // Nearest triangle (of MeshFeatures) a world ray meets, its object and
    // the ray parameter; -1 when it meets nothing
    int  featureTriangleAt(const QVector3D& orig, const QVector3D& dir, int* objectId, float* worldT);
    int  featureTriangleAt(const QVector3D& orig, const QVector3D& dir, int* objectId, float* worldT,
                           bool fadedOnly);
    void commitAxisCandidate();

    struct MeasurePoint {
        bool      valid = false;
        QVector3D pos, normal;            // normal: unit, or null when the point has none
        QString   what;                   // "centre of Ø 12.0 mm", "corner", …
        std::vector<QVector3D> outline;   // the snapped feature, world segments
    };
    bool         m_measureMode = false;
    MeasurePoint m_measureA, m_measureB, m_measureHover;
    MeasurePoint measurePointAt(int x, int y);

    // Assemble mode: the feature under the cursor and the first pick
    struct MatePick {
        bool      valid = false;
        int       objectId = -1;
        MateFeature::Kind kind = MateFeature::Kind::Point;
        QVector3D point, direction;      // world
        float     radius = 0.0f;         // world
        QString   what;                  // "flat face", "round edge Ø 12.0 mm", …
        std::vector<QVector3D> outline;  // object-local segments (follow the part when it moves)
    };
    bool           m_assembleMode = false;
    bool           m_assemblyAvailable = true;
    MatePick       m_mateFirst, m_mateHover;
    class AssemblePanel* m_assemblePanel = nullptr;
    MatePick matePickAt(int x, int y);
    void     handleMateClick(int x, int y);
    // A mate feature drawn as a glyph: a square on a plane, a line along an
    // axis, a ring for a round edge, a dot for a point
    void     drawMateGlyph(QPainter& p, const MateFeature& f, const QColor& color);

    // Faces and edges recognised on each part's mesh, built on first use.
    // The weak pointer notices a freed geometry whose address was reused.
    struct FeatureCache {
        std::weak_ptr<MeshGeometry>   geometry;
        std::unique_ptr<MeshFeatures> features;
    };
    std::unordered_map<const MeshGeometry*, FeatureCache> m_meshFeatures;
    std::unique_ptr<MeshFeatures> m_primitiveFeatures[6];
    const MeshFeatures* featuresFor(const SceneObject& obj);

    // Gizmo: set in drawAxisGizmo(), read in drawAxisLabels()
    QMatrix4x4 m_gizmoView, m_gizmoProj;
    static constexpr int kGizmoSize   = 80;
    static constexpr int kGizmoMargin = 14;

    int              m_hoveredId = -1;
    QSet<int>        m_movingObjects;
    QSet<int>        m_interfering;      // tinted red: passing through another part
    QStringList      m_overlayLines;
    class QLabel*    m_simOverlay = nullptr;   // a widget, not QPainter text (renders reliably over GL)
    int              m_nearKeyPointIdx = -1;  // index into keyPointsFor(hovered obj type)
    QPoint           m_lastMouse;
    Qt::MouseButton  m_dragBtn = Qt::NoButton;
    int              m_vpW = 1, m_vpH = 1;
    bool             m_exporting = false;   // renderImage: no selection / hover tint

    // Translate gizmo
    int       m_gizmoHoveredAxis  = -1;   // 0=X, 1=Y, 2=Z
    int       m_gizmoDragAxis     = -1;
    float     m_gizmoDragStartT   = 0.f;
    QVector3D m_gizmoDragStartPos;        // local position at drag start
    QVector3D m_gizmoDragWorldCenter;     // world center at drag start (fixed reference)
    QMatrix4x4 m_ikDragStartFrame;        // IK drag: tool frame at drag start
    std::vector<int> m_ikDragChain;

    void  drawTranslateGizmo();
    int   hitTestGizmoAxis(int mx, int my) const;
    float projectOntoAxis(int axis, const QVector3D& center, int mx, int my) const;
    float gizmoArrowLength(int objectId) const;
};
