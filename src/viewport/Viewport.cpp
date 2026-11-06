#include "Viewport.h"
#include "ui/Theme.h"
#include "ViewCube.h"
#include "scene/MeshWeld.h"
#include "scene/PrimitiveMesh.h"
#include "scene/SceneQuery.h"
#include "scene/Assembly.h"
#include "AssemblePanel.h"
#include <QVector4D>
#include "automation/Sensors.h"
#include "automation/Kinematics.h"
#include <QVariantAnimation>
#include <QLabel>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QFontMetrics>
#include <QStyle>
#include <QtMath>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>

// ── Shader sources ──────────────────────────────────────────────────────────

static const char* kObjVert = R"(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNorm;
uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;
uniform mat3 normMat;
out vec3 fragPos;
out vec3 normal;
void main(){
    vec4 wp = model * vec4(aPos,1.0);
    fragPos  = wp.xyz;
    normal   = normalize(normMat * aNorm);
    gl_Position = proj * view * wp;
}
)";

static const char* kObjFrag = R"(
#version 330 core
in vec3 fragPos;
in vec3 normal;
uniform vec3  objColor;
uniform vec3  keyDir;
uniform vec3  fillDir;
uniform float ambient;
uniform float keyLight;
uniform float fillLight;
uniform float specular;
uniform vec3  viewPos;
uniform bool  isSelected;
uniform bool  isHovered;
uniform bool  isActive;
uniform bool  isAlert;
uniform bool  clipOn;
uniform vec4  clipPlane;
uniform vec3  sectionColor;
uniform float opacity;
out vec4 fragColor;
void main(){
    if(clipOn){
        if(dot(vec4(fragPos, 1.0), clipPlane) < 0.0) discard;
        // Through the cut we see the inside (back faces): fill it flat
        if(!gl_FrontFacing){ fragColor = vec4(sectionColor, opacity); return; }
    }
    vec3 n = normalize(normal);
    vec3 l = normalize(keyDir);
    float diff = max(dot(n,l), 0.0);
    float fillDiff = max(dot(n,normalize(fillDir)), 0.0);
    vec3 v = normalize(viewPos - fragPos);
    vec3 h = normalize(l + v);
    float spec = pow(max(dot(n,h), 0.0), 48.0);
    vec3 col = objColor * (ambient + keyLight*diff + fillLight*fillDiff) + vec3(specular)*spec;
    if(isActive)   col = mix(col, vec3(0.15, 0.80, 0.45), 0.30);
    if(isAlert)    col = mix(col, vec3(1.0, 0.15, 0.12), 0.55);
    if(isHovered)  col = mix(col, vec3(0.45, 0.72, 1.0), 0.28);
    if(isSelected) col = mix(col, vec3(1.0, 0.75, 0.1),  0.40);
    fragColor = vec4(col, opacity);
}
)";

static const char* kGridVert = R"(
#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 view;
uniform mat4 proj;
void main(){ gl_Position = proj * view * vec4(aPos,1.0); }
)";

static const char* kGridFrag = R"(
#version 330 core
uniform vec3 lineColor;
out vec4 fragColor;
void main(){ fragColor = vec4(lineColor,1.0); }
)";

static const char* kWireVert = R"(
#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;
void main(){ gl_Position = proj * view * model * vec4(aPos,1.0); }
)";

static const char* kWireFrag = R"(
#version 330 core
uniform vec3  lineColor;
uniform float alpha;
out vec4 fragColor;
void main(){ fragColor = vec4(lineColor, alpha); }
)";

// ── Wire geometry helpers ────────────────────────────────────────────────────

static std::vector<float> wireBox(float hw, float hh, float hd) {
    std::vector<float> v;
    auto line = [&](float x1,float y1,float z1, float x2,float y2,float z2){
        v.insert(v.end(), {x1,y1,z1, x2,y2,z2});
    };
    // Bottom face
    line(-hw,-hh,-hd,  hw,-hh,-hd);
    line( hw,-hh,-hd,  hw,-hh, hd);
    line( hw,-hh, hd, -hw,-hh, hd);
    line(-hw,-hh, hd, -hw,-hh,-hd);
    // Top face
    line(-hw, hh,-hd,  hw, hh,-hd);
    line( hw, hh,-hd,  hw, hh, hd);
    line( hw, hh, hd, -hw, hh, hd);
    line(-hw, hh, hd, -hw, hh,-hd);
    // Verticals
    line(-hw,-hh,-hd, -hw, hh,-hd);
    line( hw,-hh,-hd,  hw, hh,-hd);
    line( hw,-hh, hd,  hw, hh, hd);
    line(-hw,-hh, hd, -hw, hh, hd);
    return v;
}

static std::vector<float> wireCylinder(float r, float hh, int segs = 24) {
    std::vector<float> v;
    auto line = [&](float x1,float y1,float z1, float x2,float y2,float z2){
        v.insert(v.end(), {x1,y1,z1, x2,y2,z2});
    };
    for (int i = 0; i < segs; i++) {
        float a0 = float(i)/segs * 2*M_PI, a1 = float(i+1)/segs * 2*M_PI;
        line(r*cosf(a0),-hh,r*sinf(a0), r*cosf(a1),-hh,r*sinf(a1));
        line(r*cosf(a0), hh,r*sinf(a0), r*cosf(a1), hh,r*sinf(a1));
    }
    for (int i = 0; i < 4; i++) {
        float a = float(i)/4.0f * 2*M_PI;
        line(r*cosf(a),-hh,r*sinf(a), r*cosf(a),hh,r*sinf(a));
    }
    return v;
}

static std::vector<float> wireSphere(float r, int segs = 24) {
    std::vector<float> v;
    auto line = [&](float x1,float y1,float z1, float x2,float y2,float z2){
        v.insert(v.end(), {x1,y1,z1, x2,y2,z2});
    };
    for (int i = 0; i < segs; i++) {
        float a0 = float(i)/segs * 2*M_PI, a1 = float(i+1)/segs * 2*M_PI;
        line(r*cosf(a0), r*sinf(a0), 0,           r*cosf(a1), r*sinf(a1), 0);
        line(r*cosf(a0), 0,           r*sinf(a0), r*cosf(a1), 0,          r*sinf(a1));
        line(0, r*cosf(a0), r*sinf(a0),           0, r*cosf(a1), r*sinf(a1));
    }
    return v;
}

// h = total cylinder part height (hh = h/2)
static std::vector<float> wireCapsule(float r, float h, int segs = 24) {
    std::vector<float> v;
    float hh = h * 0.5f;
    auto line = [&](float x1,float y1,float z1, float x2,float y2,float z2){
        v.insert(v.end(), {x1,y1,z1, x2,y2,z2});
    };
    // Cylinder band
    for (int i = 0; i < segs; i++) {
        float a0 = float(i)/segs*2*M_PI, a1 = float(i+1)/segs*2*M_PI;
        line(r*cosf(a0),-hh,r*sinf(a0), r*cosf(a1),-hh,r*sinf(a1));
        line(r*cosf(a0), hh,r*sinf(a0), r*cosf(a1), hh,r*sinf(a1));
    }
    // Vertical lines
    for (int i = 0; i < 4; i++) {
        float a = float(i)/4.0f*2*M_PI;
        line(r*cosf(a),-hh,r*sinf(a), r*cosf(a),hh,r*sinf(a));
    }
    // Top hemisphere (t: 0→π, x=r*cos(t), y=hh+r*sin(t))
    int hs = segs / 2;
    for (int i = 0; i < hs; i++) {
        float t0=float(i)/hs*M_PI, t1=float(i+1)/hs*M_PI;
        line(r*cosf(t0), hh+r*sinf(t0), 0,           r*cosf(t1), hh+r*sinf(t1), 0);
        line(0,          hh+r*sinf(t0), r*cosf(t0),  0,          hh+r*sinf(t1), r*cosf(t1));
    }
    // Bottom hemisphere (t: π→2π, y=-hh+r*sin(t))
    for (int i = 0; i < hs; i++) {
        float t0=M_PI+float(i)/hs*M_PI, t1=M_PI+float(i+1)/hs*M_PI;
        line(r*cosf(t0),-hh+r*sinf(t0), 0,           r*cosf(t1),-hh+r*sinf(t1), 0);
        line(0,         -hh+r*sinf(t0), r*cosf(t0),  0,         -hh+r*sinf(t1), r*cosf(t1));
    }
    return v;
}

// Basket wireframe (bounding-box outline, no top face cross, open top visible)
static std::vector<float> wireBasket() {
    std::vector<float> v;
    auto line = [&](float x1,float y1,float z1, float x2,float y2,float z2){
        v.insert(v.end(), {x1,y1,z1, x2,y2,z2});
    };
    // Bottom
    line(-0.5f,-0.5f,-0.5f, 0.5f,-0.5f,-0.5f);
    line( 0.5f,-0.5f,-0.5f, 0.5f,-0.5f, 0.5f);
    line( 0.5f,-0.5f, 0.5f,-0.5f,-0.5f, 0.5f);
    line(-0.5f,-0.5f, 0.5f,-0.5f,-0.5f,-0.5f);
    // Top rim (open, no lid)
    line(-0.5f, 0.5f,-0.5f, 0.5f, 0.5f,-0.5f);
    line( 0.5f, 0.5f,-0.5f, 0.5f, 0.5f, 0.5f);
    line( 0.5f, 0.5f, 0.5f,-0.5f, 0.5f, 0.5f);
    line(-0.5f, 0.5f, 0.5f,-0.5f, 0.5f,-0.5f);
    // Verticals
    line(-0.5f,-0.5f,-0.5f,-0.5f, 0.5f,-0.5f);
    line( 0.5f,-0.5f,-0.5f, 0.5f, 0.5f,-0.5f);
    line( 0.5f,-0.5f, 0.5f, 0.5f, 0.5f, 0.5f);
    line(-0.5f,-0.5f, 0.5f,-0.5f, 0.5f, 0.5f);
    return v;
}

using scenequery::stripScale;

// ── Constructor / Destructor ─────────────────────────────────────────────────

Viewport::Viewport(SceneGraph* scene, QWidget* parent)
    : QOpenGLWidget(parent), m_scene(scene)
{
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);

    connect(scene, &SceneGraph::objectAdded,      this, [this](int){ update(); });
    connect(scene, &SceneGraph::objectRemoved,    this, [this](int){ m_hoveredId = -1; m_sweepMeshes = true; update(); });
    connect(scene, &SceneGraph::objectChanged,    this, [this](int){ update(); });
    connect(scene, &SceneGraph::selectionChanged, this, [this](int){
        m_gizmoHoveredAxis = -1;
        m_gizmoDragAxis    = -1;
        update();
    });
    connect(scene, &SceneGraph::sceneCleared,     this, [this]{
        m_hoveredId = -1;
        m_sweepMeshes = true;
        update();
    });
    connect(scene, &SceneGraph::rigidBodyAdded,      this, [this](int){ update(); });
    connect(scene, &SceneGraph::rigidBodyRemoved,   this, [this](int){ update(); });
    connect(scene, &SceneGraph::rigidBodyChanged,   this, [this](int){ update(); });
    connect(scene, &SceneGraph::conveyorBeltAdded,  this, [this](int){ update(); });
    connect(scene, &SceneGraph::conveyorBeltRemoved,this, [this](int){ update(); });
    connect(scene, &SceneGraph::conveyorBeltChanged,this, [this](int){ update(); });
    connect(scene, &SceneGraph::floorChanged,       this, [this](bool){ update(); });
    connect(scene, &SceneGraph::jointsMoved,        this, [this]{ update(); });
    connect(scene, &SceneGraph::automationChanged,  this, [this]{ update(); });

    // View-snap overlay buttons
    auto mkBtn = [&](const QString& label) -> QPushButton* {
        auto* b = new QPushButton(label, this);
        b->setObjectName("viewButton");
        b->setFixedSize(52, 26);
        b->setCursor(Qt::PointingHandCursor);
        b->raise();
        return b;
    };

    m_btnFront = mkBtn("Front");
    m_btnTop   = mkBtn("Top");
    m_btnSide  = mkBtn("Side");

    m_btnFloor = new QPushButton(Theme::icon("grid-3x3", Theme::Tint::OnAccent), "Floor: ON", this);
    m_btnFloor->setObjectName("viewButton");
    m_btnFloor->setToolTip("Toggle the floor collider (objects rest on Y = 0)");
    m_btnFloor->setFixedSize(96, 26);
    m_btnFloor->setCursor(Qt::PointingHandCursor);
    m_btnFloor->setProperty("active", true);
    m_btnFloor->raise();

    m_btnFront->setToolTip("Look at the front (along -Z)");
    m_btnTop->setToolTip("Look straight down from the top");
    m_btnSide->setToolTip("Look at the right side (along -X)");
    connect(m_btnFront, &QPushButton::clicked, this, [this]{
        animateTo(Camera::lookRotation({0, 0, 1}, {0, 1, 0}), true);
    });
    connect(m_btnTop, &QPushButton::clicked, this, [this]{
        animateTo(Camera::lookRotation({0, 1, 0}, {0, 0, -1}), true);
    });
    connect(m_btnSide, &QPushButton::clicked, this, [this]{
        animateTo(Camera::lookRotation({1, 0, 0}, {0, 1, 0}), true);
    });

    // View cube (top right): click a face / edge / corner to turn there,
    // drag it to orbit, the house goes back to the home view
    m_viewCube = new ViewCube(this);
    m_viewCube->setCameraRotation(m_camera.rotation());
    connect(m_viewCube, &ViewCube::rotationRequested, this, &Viewport::animateTo);
    connect(m_viewCube, &ViewCube::homeRequested, this, [this]{
        animateTo(Camera::homeRotation(), true);
    });
    connect(m_viewCube, &ViewCube::orbitRequested, this, [this](float dx, float dy){
        m_viewAnimation->stop();
        m_camera.orbit(dx, dy);
        cameraChanged();
    });
    m_viewAnimation = new QVariantAnimation(this);
    m_viewAnimation->setStartValue(0.0);
    m_viewAnimation->setEndValue(1.0);
    m_viewAnimation->setDuration(320);
    m_viewAnimation->setEasingCurve(QEasingCurve::InOutCubic);
    connect(m_viewAnimation, &QVariantAnimation::valueChanged, this, [this](const QVariant& value){
        const float t = value.toFloat();
        const Camera &a = m_animationFrom, &b = m_animationTo;
        m_camera.setRotation(QQuaternion::slerp(a.rotation(), b.rotation(), t));
        m_camera.setTarget(a.target() + (b.target() - a.target()) * t);
        // Zoom at a constant rate: interpolate the distance's logarithm
        m_camera.setDistance(a.distance() * std::pow(b.distance() / a.distance(), t));
        cameraChanged();
    });
    connect(m_btnFloor, &QPushButton::clicked, this, [this]{
        bool now = !m_scene->floorEnabled();
        m_scene->setFloorEnabled(now);
        m_btnFloor->setText(now ? "Floor: ON" : "Floor: OFF");
        m_btnFloor->setProperty("active", now);
        m_btnFloor->setIcon(Theme::icon("grid-3x3", now ? Theme::Tint::OnAccent : Theme::Tint::Text));
        Theme::repolish(m_btnFloor);
    });

    // Lighting: each theme remembers its own settings
    m_light = ViewportLighting::load(Theme::mode());
    m_lightPanel = new LightingPanel(this);
    m_lightPanel->setLighting(m_light);
    connect(m_lightPanel, &LightingPanel::lightingChanged, this, [this](const ViewportLighting& l){
        m_light = l;
        m_light.save(Theme::mode());
        update();
    });
    connect(Theme::notifier(), &Theme::Notifier::changed, this, [this]{
        m_light = ViewportLighting::load(Theme::mode());
        m_lightPanel->setLighting(m_light);
        update();
    });

    m_btnIk = new QPushButton(Theme::icon("move-diagonal-2"), "IK", this);
    m_btnIk->setObjectName("viewButton");
    m_btnIk->setToolTip("Inverse kinematics: drag the gizmo of a robot's tool (any part that actuated joints "
                        "move) and the joints follow. With 6 or more joints its orientation is kept.");
    m_btnIk->setFixedSize(58, 26);
    m_btnIk->setCursor(Qt::PointingHandCursor);
    m_btnIk->setCheckable(true);
    m_btnIk->raise();
    connect(m_btnIk, &QPushButton::toggled, this, [this](bool on){
        m_btnIk->setProperty("active", on);
        m_btnIk->setIcon(Theme::icon("move-diagonal-2", on ? Theme::Tint::OnAccent : Theme::Tint::Text));
        Theme::repolish(m_btnIk);
        update();
    });
    m_assemblePanel = new AssemblePanel(m_scene, this);
    m_assemblePanel->hide();
    connect(m_assemblePanel, &AssemblePanel::changed, this, [this]{ update(); });
    connect(m_assemblePanel, &AssemblePanel::closeRequested, this, [this]{ setAssembleMode(false); });
    m_sectionPanel = new SectionPanel(this);
    m_btnSection = new QPushButton(Theme::icon("scissors"), "Section", this);
    m_btnSection->setObjectName("viewButton");
    m_btnSection->setToolTip("Cut the model with a plane to see inside assemblies");
    m_btnSection->setFixedSize(84, 26);
    m_btnSection->setCursor(Qt::PointingHandCursor);
    m_btnSection->raise();
    connect(m_btnSection, &QPushButton::clicked, this, [this]{
        QVector3D mn, mx;
        sceneBounds(&mn, &mx);
        m_sectionPanel->setBounds(mn, mx);
        if (!m_section.enabled && m_section.position == 0.0f) {
            // First use: through the middle, on the camera's sideways axis
            m_section.axis = 0;
            m_section.position = (mn.x() + mx.x()) * 0.5f;
        }
        m_sectionPanel->setPlane(m_section);
        m_sectionPanel->adjustSize();
        m_sectionPanel->move(m_btnSection->mapToGlobal(QPoint(m_btnSection->width() - m_sectionPanel->width(),
                                                              m_btnSection->height() + 6)));
        m_sectionPanel->show();
    });
    connect(m_sectionPanel, &SectionPanel::planeChanged, this, [this](const SectionPlane& p){
        m_section = p;
        m_btnSection->setProperty("active", p.enabled);
        m_btnSection->setIcon(Theme::icon("scissors", p.enabled ? Theme::Tint::OnAccent : Theme::Tint::Text));
        Theme::repolish(m_btnSection);
        update();
    });
    m_btnLight = new QPushButton(Theme::icon("lightbulb"), "Lighting", this);
    m_btnLight->setObjectName("viewButton");
    m_btnLight->setToolTip("Adjust the viewport lighting for the current theme");
    m_btnLight->setFixedSize(90, 26);
    m_btnLight->setCursor(Qt::PointingHandCursor);
    m_btnLight->raise();
    connect(m_btnLight, &QPushButton::clicked, this, [this]{
        m_lightPanel->adjustSize();
        QPoint pos = m_btnLight->mapToGlobal(QPoint(m_btnLight->width() - m_lightPanel->width(),
                                                    m_btnLight->height() + 6));
        m_lightPanel->move(pos);
        m_lightPanel->show();
    });
}

Viewport::~Viewport() {
    makeCurrent();
    for (MeshData* m : {&m_cube, &m_cylinder, &m_sphere, &m_capsule, &m_basket}) deleteMesh(*m);
    for (auto& [geo, m] : m_importedMeshes) deleteMesh(m.gpu);
    m_importedMeshes.clear();
    if(m_gridVao)  glDeleteVertexArrays(1, &m_gridVao);
    if(m_gridVbo)  glDeleteBuffers(1, &m_gridVbo);
    if(m_axisVao)  glDeleteVertexArrays(1, &m_axisVao);
    if(m_axisVbo)  glDeleteBuffers(1, &m_axisVbo);
    if(m_wireVao)  glDeleteVertexArrays(1, &m_wireVao);
    if(m_wireVbo)  glDeleteBuffers(1, &m_wireVbo);
    if(m_floorVao) glDeleteVertexArrays(1, &m_floorVao);
    if(m_floorVbo) glDeleteBuffers(1, &m_floorVbo);
    delete m_objShader;
    delete m_gridShader;
    delete m_wireShader;
    doneCurrent();
}

// ── GL lifecycle ─────────────────────────────────────────────────────────────

void Viewport::initializeGL() {
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    buildShaders();
    buildMeshes();
    buildGrid();
    buildAxisGizmo();
    buildWireResources();
}

void Viewport::resizeGL(int w, int h) {
    m_vpW = w; m_vpH = h;
    glViewport(0, 0, w, h);
    repositionOverlays();
}

void Viewport::repositionOverlays() {
    const int mg = 10, gap = 4;
    int x = mg;
    for (auto* btn : {m_btnFront, m_btnTop, m_btnSide}) {
        btn->move(x, mg);
        x += btn->width() + gap;
    }
    m_btnFloor->move(m_vpW - m_btnFloor->width() - mg, mg);
    m_btnLight->move(m_btnFloor->x() - m_btnLight->width() - gap, mg);
    m_btnSection->move(m_btnLight->x() - m_btnSection->width() - gap, mg);
    m_btnIk->move(m_btnSection->x() - m_btnIk->width() - gap, mg);
    m_viewCube->move(m_vpW - m_viewCube->width() - 2, m_btnFloor->y() + m_btnFloor->height() + gap);
    if (m_assemblePanel) {
        m_assemblePanel->adjustSize();
        m_assemblePanel->move(mg, m_btnFront->y() + m_btnFront->height() + 10);
    }
}

// ── Camera ───────────────────────────────────────────────────────────────────

void Viewport::setProjection(Camera::Projection projection) {
    m_camera.setProjection(projection);
    m_animationFrom.setProjection(projection);
    m_animationTo.setProjection(projection);
    cameraChanged();
}

void Viewport::animateTo(const QQuaternion& rotation, bool fit) {
    m_viewAnimation->stop();
    m_animationFrom = m_camera;
    m_animationTo   = m_camera;
    m_animationTo.setRotation(rotation);
    if (fit) frameScene(&m_animationTo);
    if (!isVisible()) {
        m_camera = m_animationTo;
        cameraChanged();
        return;
    }
    m_viewAnimation->start();
}

void Viewport::cameraChanged() {
    m_viewCube->setCameraRotation(m_camera.rotation());
    update();
}

void Viewport::fitAll() {
    m_viewAnimation->stop();
    frameScene(&m_camera);
    cameraChanged();
}

void Viewport::frameScene(Camera* camera) const {
    constexpr float inf = std::numeric_limits<float>::max();
    QVector3D lo(inf, inf, inf), hi(-inf, -inf, -inf);
    for (auto& [id, obj] : m_scene->allObjects()) {
        if (obj.type == PrimitiveType::ImportedMesh && !obj.mesh) continue;   // frames only
        const QMatrix4x4 wt = m_scene->worldTransform(id);
        const QVector3D mn = obj.aabbMin(), mx = obj.aabbMax();
        for (int i = 0; i < 8; ++i) {
            const QVector3D c = wt.map(QVector3D(i & 1 ? mx.x() : mn.x(), i & 2 ? mx.y() : mn.y(),
                                                 i & 4 ? mx.z() : mn.z()));
            lo = QVector3D(std::min(lo.x(), c.x()), std::min(lo.y(), c.y()), std::min(lo.z(), c.z()));
            hi = QVector3D(std::max(hi.x(), c.x()), std::max(hi.y(), c.y()), std::max(hi.z(), c.z()));
        }
    }
    if (lo.x() > hi.x()) { lo = QVector3D(-2, 0, -2); hi = QVector3D(2, 2, 2); }   // empty scene
    camera->frame((lo + hi) / 2, (hi - lo).length() / 2);
}

void Viewport::paintGL() {
    QPainter p(this);
    p.setRenderHint(QPainter::TextAntialiasing);

    p.beginNativePainting();
    if (m_sweepMeshes) sweepFreedMeshes();

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glDisable(GL_SCISSOR_TEST);
    const QColor bg = Theme::colors().viewport;
    glClearColor(bg.redF(), bg.greenF(), bg.blueF(), 1.0f);
    glViewport(0, 0, m_vpW, m_vpH);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    drawGrid();
    drawScene();
    drawPhysicsOverlays();
    drawTranslateGizmo();
    drawAxisGizmo();

    glViewport(0, 0, m_vpW, m_vpH);
    glDisable(GL_SCISSOR_TEST);
    // QPainter doesn't reset this: left on, the 2D overlays below would be
    // depth-tested against the scene and partly vanish
    glDisable(GL_DEPTH_TEST);

    p.endNativePainting();

    drawAxisLabels(p);
    drawKeyHints(p);
    drawKeyPoints(p);
    drawSensorLabels(p);
    drawJointAxes(p);
    drawAxisPickBanner(p);
    drawMeasure(p);
    drawAssemble(p);
}

void Viewport::setIkAvailable(bool available) {
    m_ikAvailable = available;
    m_btnIk->setEnabled(available);
    m_btnIk->setToolTip(available ? m_btnIk->toolTip().section("\n(", 0, 0)
                                  : m_btnIk->toolTip().section("\n(", 0, 0) + "\n(Reset the simulation to use IK)");
}

bool Viewport::ikDragActive() const {
    return m_btnIk && m_btnIk->isChecked() && m_ikAvailable;
}

void Viewport::setSimulationOverlay(const QSet<int>& movingObjects, const QStringList& lines,
                                    const QSet<int>& interfering) {
    if (movingObjects == m_movingObjects && lines == m_overlayLines && interfering == m_interfering) return;
    m_movingObjects = movingObjects;
    m_interfering = interfering;
    if (lines != m_overlayLines) {
        m_overlayLines = lines;
        if (!m_simOverlay) {
            m_simOverlay = new QLabel(this);
            m_simOverlay->setObjectName("viewOverlay");
            m_simOverlay->setAttribute(Qt::WA_TransparentForMouseEvents);
        }
        QStringList shown;
        for (const QString& l : lines) shown << (l.startsWith("⚠") ? l : "▶ " + l);
        m_simOverlay->setText(shown.join("\n"));
        m_simOverlay->adjustSize();
        m_simOverlay->move(10, 46);
        m_simOverlay->setVisible(!lines.isEmpty());
    }
    update();
}


// ── Shader / mesh setup ──────────────────────────────────────────────────────

void Viewport::buildShaders() {
    m_objShader = new QOpenGLShaderProgram(this);
    m_objShader->addShaderFromSourceCode(QOpenGLShader::Vertex,   kObjVert);
    m_objShader->addShaderFromSourceCode(QOpenGLShader::Fragment, kObjFrag);
    m_objShader->link();

    m_gridShader = new QOpenGLShaderProgram(this);
    m_gridShader->addShaderFromSourceCode(QOpenGLShader::Vertex,   kGridVert);
    m_gridShader->addShaderFromSourceCode(QOpenGLShader::Fragment, kGridFrag);
    m_gridShader->link();

    m_wireShader = new QOpenGLShaderProgram(this);
    m_wireShader->addShaderFromSourceCode(QOpenGLShader::Vertex,   kWireVert);
    m_wireShader->addShaderFromSourceCode(QOpenGLShader::Fragment, kWireFrag);
    m_wireShader->link();
}

void Viewport::buildWireResources() {
    // Scratch VAO/VBO for dynamic wireframes
    glGenVertexArrays(1, &m_wireVao);
    glGenBuffers(1, &m_wireVbo);
    glBindVertexArray(m_wireVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_wireVbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    // Floor plane quad at Y=0 (40m×40m)
    static const float kFloor[] = {
        -20,0,-20,  20,0,-20,  20,0, 20,
        -20,0,-20,  20,0, 20, -20,0, 20,
    };
    glGenVertexArrays(1, &m_floorVao);
    glGenBuffers(1, &m_floorVbo);
    glBindVertexArray(m_floorVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_floorVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(kFloor), kFloor, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void Viewport::uploadMesh(MeshData& m, const std::vector<float>& v) {
    glGenVertexArrays(1, &m.vao);
    glGenBuffers(1, &m.vbo);
    glBindVertexArray(m.vao);
    glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
    glBufferData(GL_ARRAY_BUFFER, v.size() * sizeof(float), v.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
    m.count = int(v.size()) / 6;
}

void Viewport::buildMeshes() {
    uploadMesh(m_cube,     primitiveSoup(PrimitiveType::Cube));
    uploadMesh(m_cylinder, primitiveSoup(PrimitiveType::Cylinder));
    uploadMesh(m_sphere,   primitiveSoup(PrimitiveType::Sphere));
    uploadMesh(m_capsule,  primitiveSoup(PrimitiveType::Capsule));
    uploadMesh(m_basket,   primitiveSoup(PrimitiveType::Basket));
}

// Imported CAD meshes are triangle soup; welded, they need a third to a fifth
// of the memory (each vertex is shared by several triangles).
void Viewport::uploadIndexedMesh(MeshData& m, const std::vector<float>& soup) {
    const WeldedMesh w = weldMesh(soup);
    uploadMesh(m, w.vertices);
    glBindVertexArray(m.vao);
    glGenBuffers(1, &m.ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(w.indices.size() * sizeof(uint32_t)),
                 w.indices.data(), GL_STATIC_DRAW);
    glBindVertexArray(0);
    m.count = int(w.indices.size());
}

void Viewport::deleteMesh(MeshData& m) {
    if (m.vao) glDeleteVertexArrays(1, &m.vao);
    if (m.vbo) glDeleteBuffers(1, &m.vbo);
    if (m.ebo) glDeleteBuffers(1, &m.ebo);
    m = MeshData{};
}

// Lazy GPU upload for imported CAD meshes. Must be called with the GL
// context current (drawScene). Group nodes (null mesh) return nullptr.
const MeshData* Viewport::meshFor(const SceneObject& obj) {
    if (!obj.mesh || obj.mesh->interleaved.empty()) return nullptr;
    ImportedMesh& entry = m_importedMeshes[obj.mesh.get()];
    if (entry.gpu.vao && entry.geometry.lock() == obj.mesh) return &entry.gpu;
    deleteMesh(entry.gpu);   // a freed geometry's address, reused
    entry.geometry = obj.mesh;
    uploadIndexedMesh(entry.gpu, obj.mesh->interleaved);
    return &entry.gpu;
}

void Viewport::sweepFreedMeshes() {
    m_sweepMeshes = false;
    for (auto it = m_importedMeshes.begin(); it != m_importedMeshes.end();) {
        if (!it->second.geometry.expired()) { ++it; continue; }
        deleteMesh(it->second.gpu);
        it = m_importedMeshes.erase(it);
    }
    for (auto it = m_meshFeatures.begin(); it != m_meshFeatures.end();)
        it = it->second.geometry.expired() ? m_meshFeatures.erase(it) : std::next(it);
}

void Viewport::buildAxisGizmo() {
    static const float verts[] = {
        0,0,0,  1,0,0,
        0,0,0,  0,1,0,
        0,0,0,  0,0,1,
    };
    glGenVertexArrays(1, &m_axisVao);
    glGenBuffers(1, &m_axisVbo);
    glBindVertexArray(m_axisVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_axisVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void Viewport::buildGrid() {
    std::vector<float> v;
    const int half = 20;
    for(int i=-half;i<=half;i++){
        float f=float(i);
        v.insert(v.end(),{f,0,float(-half),f,0,float(half)});
        v.insert(v.end(),{float(-half),0,f,float(half),0,f});
    }
    glGenVertexArrays(1,&m_gridVao); glGenBuffers(1,&m_gridVbo);
    glBindVertexArray(m_gridVao); glBindBuffer(GL_ARRAY_BUFFER,m_gridVbo);
    glBufferData(GL_ARRAY_BUFFER,v.size()*sizeof(float),v.data(),GL_STATIC_DRAW);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3*sizeof(float),(void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
    m_gridLines=int(v.size())/3;
}

// ── Rendering ─────────────────────────────────────────────────────────────────

static QVector3D primitiveColor(PrimitiveType t){
    switch(t){
        case PrimitiveType::Cube:     return {0.25f,0.55f,1.00f};
        case PrimitiveType::Cylinder: return {0.15f,0.78f,0.35f};
        case PrimitiveType::Sphere:   return {1.00f,0.45f,0.15f};
        case PrimitiveType::Capsule:  return {0.72f,0.25f,1.00f};
        case PrimitiveType::Basket:   return {0.72f,0.50f,0.18f};
        case PrimitiveType::ImportedMesh: return {0.62f,0.66f,0.72f};   // machined steel
    }
    return {0.6f,0.6f,0.6f};
}

std::vector<Viewport::KeyPoint> Viewport::keyPointsFor(const SceneObject& obj) {
    const QVector3D mn = obj.aabbMin(), mx = obj.aabbMax();
    const QVector3D c  = (mn + mx) * 0.5f;

    // Cylinder/Capsule are round about the Y axis — a bounding-box corner
    // isn't even on the surface. The end-cap centers are the actual
    // mechanically meaningful points (e.g. a shaft rotating about its own
    // axis, or a pin through its end face).
    if (obj.type == PrimitiveType::Cylinder || obj.type == PrimitiveType::Capsule) {
        return {
            {"Center",      c},
            {"Top Center",  {c.x(), mx.y(), c.z()}},
            {"Bottom Center", {c.x(), mn.y(), c.z()}},
        };
    }

    // Cube/Sphere/Basket/ImportedMesh: center + 8 bounding-box corners.
    return {
        {"Center",      c},
        {"Bot-Front-L", {mn.x(), mn.y(), mn.z()}},
        {"Bot-Front-R", {mx.x(), mn.y(), mn.z()}},
        {"Bot-Back-L",  {mn.x(), mn.y(), mx.z()}},
        {"Bot-Back-R",  {mx.x(), mn.y(), mx.z()}},
        {"Top-Front-L", {mn.x(), mx.y(), mn.z()}},
        {"Top-Front-R", {mx.x(), mx.y(), mn.z()}},
        {"Top-Back-L",  {mn.x(), mx.y(), mx.z()}},
        {"Top-Back-R",  {mx.x(), mx.y(), mx.z()}},
    };
}

void Viewport::drawMesh(const MeshData& m){
    glBindVertexArray(m.vao);
    if(m.ebo) glDrawElements(GL_TRIANGLES, m.count, GL_UNSIGNED_INT, nullptr);
    else      glDrawArrays(GL_TRIANGLES, 0, m.count);
    glBindVertexArray(0);
}

void Viewport::drawGrid(){
    float aspect=float(m_vpW)/float(std::max(m_vpH,1));
    // Tint grid blue when floor is active
    auto vec = [](const QColor& c) { return QVector3D(c.redF(), c.greenF(), c.blueF()); };
    const Theme::Colors& tc = Theme::colors();
    QVector3D gridCol = vec(m_scene->floorEnabled() ? tc.gridFloor : tc.grid);
    m_gridShader->bind();
    m_gridShader->setUniformValue("view", m_camera.viewMatrix());
    m_gridShader->setUniformValue("proj", m_camera.projectionMatrix(aspect));
    m_gridShader->setUniformValue("lineColor", gridCol);
    glBindVertexArray(m_gridVao);
    glDrawArrays(GL_LINES,0,m_gridLines);
    glBindVertexArray(0);
    m_gridShader->release();

    // Semi-transparent floor quad when floor is enabled
    if (m_scene->floorEnabled() && m_wireShader && m_floorVao) {
        QMatrix4x4 ident; ident.setToIdentity();
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        m_wireShader->bind();
        m_wireShader->setUniformValue("model",     ident);
        m_wireShader->setUniformValue("view",      m_camera.viewMatrix());
        m_wireShader->setUniformValue("proj",      m_camera.projectionMatrix(aspect));
        m_wireShader->setUniformValue("lineColor", vec(tc.floorFill));
        m_wireShader->setUniformValue("alpha",     tc.dark ? 0.07f : 0.09f);
        glBindVertexArray(m_floorVao);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glBindVertexArray(0);
        m_wireShader->release();
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
}

void Viewport::drawScene(){
    float aspect=float(m_vpW)/float(std::max(m_vpH,1));
    QMatrix4x4 view=m_camera.viewMatrix(), proj=m_camera.projectionMatrix(aspect);
    QVector3D vpos=m_camera.position();

    m_objShader->bind();
    m_objShader->setUniformValue("view",     view);
    m_objShader->setUniformValue("proj",     proj);
    m_objShader->setUniformValue("keyDir",    m_light.keyDirection());
    m_objShader->setUniformValue("fillDir",   m_light.fillDirection());
    m_objShader->setUniformValue("ambient",   m_light.ambient);
    m_objShader->setUniformValue("keyLight",  m_light.key);
    m_objShader->setUniformValue("fillLight", m_light.fill);
    m_objShader->setUniformValue("specular",  m_light.specular);
    m_objShader->setUniformValue("viewPos",  vpos);
    m_objShader->setUniformValue("clipOn",   m_section.enabled);
    m_objShader->setUniformValue("clipPlane", m_section.equation());
    {
        const QColor sc = Theme::colors().warning;
        m_objShader->setUniformValue("sectionColor", QVector3D(sc.redF(), sc.greenF(), sc.blueF()) * 0.85f);
    }

    auto drawObject = [&](int id, const SceneObject& obj, const QMatrix4x4& model, float opacity){
        m_objShader->setUniformValue("model",      model);
        m_objShader->setUniformValue("normMat",    model.normalMatrix());
        const QColor own = m_scene->colorOf(id);
        m_objShader->setUniformValue("objColor",   own.isValid() ? QVector3D(own.redF(), own.greenF(), own.blueF())
                                                                 : primitiveColor(obj.type));
        m_objShader->setUniformValue("opacity",    opacity);
        m_objShader->setUniformValue("isSelected", obj.selected);
        m_objShader->setUniformValue("isHovered",  id == m_hoveredId && !obj.selected);
        m_objShader->setUniformValue("isActive",   m_movingObjects.contains(id));
        m_objShader->setUniformValue("isAlert",    m_interfering.contains(id));
        switch(obj.type){
            case PrimitiveType::Cube:     drawMesh(m_cube);     break;
            case PrimitiveType::Cylinder: drawMesh(m_cylinder); break;
            case PrimitiveType::Sphere:   drawMesh(m_sphere);   break;
            case PrimitiveType::Capsule:  drawMesh(m_capsule);  break;
            case PrimitiveType::Basket:   drawMesh(m_basket);   break;
            case PrimitiveType::ImportedMesh:
                if (const MeshData* md = meshFor(obj)) drawMesh(*md);
                break;
        }
    };

    // Solid parts first; see-through ones after, farthest first, blended
    // over them without writing depth
    struct Faded { int id; const SceneObject* obj; QMatrix4x4 model; float opacity, depth; };
    std::vector<Faded> faded;
    for(auto& [id,obj] : m_scene->allObjects()){
        const QMatrix4x4 model=m_scene->worldTransform(id);
        float opacity = m_scene->opacityOf(id);
        // A part faded to 0 still shows faintly while it is selected; one
        // hidden with Alt+V doesn't
        if(obj.selected && !m_scene->isHidden(id)) opacity = std::max(opacity, 0.12f);
        if(opacity >= 0.999f){ drawObject(id, obj, model, 1.0f); continue; }
        if(opacity <= 0.0f) continue;
        const QVector3D centre = model.map((obj.aabbMin() + obj.aabbMax()) * 0.5f);
        faded.push_back({id, &obj, model, opacity, (view.map(centre)).z()});
    }
    if(!faded.empty()){
        // View space looks down -z: the most negative z is the farthest
        std::sort(faded.begin(), faded.end(), [](const Faded& a, const Faded& b){ return a.depth < b.depth; });
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        glEnable(GL_CULL_FACE);
        for(const Faded& f : faded){
            // Its far side first, then its near side over it
            glCullFace(GL_FRONT); drawObject(f.id, *f.obj, f.model, f.opacity);
            glCullFace(GL_BACK);  drawObject(f.id, *f.obj, f.model, f.opacity);
        }
        glDisable(GL_CULL_FACE);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
    m_objShader->release();
}

// ── Physics overlays ──────────────────────────────────────────────────────────

void Viewport::drawWireLines(const std::vector<float>& verts, const QVector3D& color,
                              const QMatrix4x4& model, float alpha) {
    if (verts.empty() || !m_wireShader || !m_wireVao) return;
    float aspect = float(m_vpW) / float(std::max(m_vpH, 1));

    glBindBuffer(GL_ARRAY_BUFFER, m_wireVbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size()*sizeof(float), verts.data(), GL_DYNAMIC_DRAW);

    if (alpha < 1.0f) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    m_wireShader->bind();
    m_wireShader->setUniformValue("model",     model);
    m_wireShader->setUniformValue("view",      m_camera.viewMatrix());
    m_wireShader->setUniformValue("proj",      m_camera.projectionMatrix(aspect));
    m_wireShader->setUniformValue("lineColor", color);
    m_wireShader->setUniformValue("alpha",     alpha);

    glBindVertexArray(m_wireVao);
    glDrawArrays(GL_LINES, 0, int(verts.size())/3);
    glBindVertexArray(0);

    m_wireShader->release();

    if (alpha < 1.0f) glDisable(GL_BLEND);
}

void Viewport::drawPhysicsOverlays() {
    if (!m_wireShader) return;
    glLineWidth(1.5f);

    for (auto& [id, rb] : m_scene->allRigidBodies()) {
        const SceneObject* obj = m_scene->getObject(id);
        if (!obj) continue;

        QMatrix4x4 rbModel = m_scene->worldTransform(id);
        rbModel.scale(1.004f);

        std::vector<float> rbWire;
        switch (obj->type) {
            case PrimitiveType::Cube:     rbWire = wireBox(0.5f, 0.5f, 0.5f);      break;
            case PrimitiveType::Cylinder: rbWire = wireCylinder(0.5f, 0.5f);        break;
            case PrimitiveType::Sphere:   rbWire = wireSphere(0.5f);                break;
            case PrimitiveType::Capsule:  rbWire = wireCapsule(0.25f, 0.5f);        break;
            case PrimitiveType::Basket:   rbWire = wireBasket();                    break;
            case PrimitiveType::ImportedMesh: {
                // AABB outline; the box is not origin-centred, so shift the model
                QVector3D he = (obj->aabbMax() - obj->aabbMin()) * 0.5f;
                rbModel.translate((obj->aabbMax() + obj->aabbMin()) * 0.5f);
                rbWire = wireBox(he.x(), he.y(), he.z());
                break;
            }
        }
        drawWireLines(rbWire, {1.0f, 0.55f, 0.05f}, rbModel);

        // Red wireframe — custom collision shape (not for Basket, it uses 5-wall auto-geoms)
        if (rb.hasCollision && obj->type != PrimitiveType::Basket) {
            const auto& c = rb.collision;
            QMatrix4x4 colModel = stripScale(m_scene->worldTransform(id));
            colModel.translate(c.offset);
            colModel.scale(1.008f);

            std::vector<float> colWire;
            switch (c.type) {
                case CollisionType::Box:     colWire = wireBox(c.halfW, c.halfH, c.halfD); break;
                case CollisionType::Cylinder: colWire = wireCylinder(c.radius, c.height*0.5f); break;
                case CollisionType::Sphere:  colWire = wireSphere(c.radius);               break;
                case CollisionType::Capsule: colWire = wireCapsule(c.radius, c.height);    break;
            }
            drawWireLines(colWire, {0.95f, 0.12f, 0.08f}, colModel);
        }
    }

    // Teal direction arrows on conveyor belt surfaces
    for (auto& [id, belt] : m_scene->allConveyorBelts()) {
        if (!belt.enabled) continue;
        const SceneObject* obj = m_scene->getObject(id);
        if (!obj) continue;

        QMatrix4x4 wt = m_scene->worldTransform(id);
        QVector3D dir = belt.direction.normalized();
        QVector3D perp(dir.z(), 0.f, -dir.x());
        const float yOff = 0.52f;  // slightly above top face (local space)
        const float halfLen = 0.35f, headLen = 0.12f, headW = 0.08f;

        std::vector<float> av;
        auto addLine = [&](QVector3D a, QVector3D b) {
            av.insert(av.end(), {a.x(),a.y(),a.z(), b.x(),b.y(),b.z()});
        };
        for (int i = -1; i <= 1; i++) {
            float lat = i * 0.28f;
            QVector3D base = perp*lat - dir*halfLen; base.setY(yOff);
            QVector3D tip  = perp*lat + dir*halfLen; tip.setY(yOff);
            addLine(base, tip);
            QVector3D backTip = tip - dir*headLen;
            QVector3D lp = backTip + perp*headW; lp.setY(yOff);
            QVector3D rp = backTip - perp*headW; rp.setY(yOff);
            addLine(tip, lp);
            addLine(tip, rp);
        }
        drawWireLines(av, {0.05f, 0.82f, 0.72f}, wt);
    }

    drawSensors();
    drawSectionOutline();
    glLineWidth(1.0f);
}

void Viewport::sceneBounds(QVector3D* mn, QVector3D* mx) const {
    *mn = QVector3D(1e30f, 1e30f, 1e30f);
    *mx = -*mn;
    for (auto& [id, obj] : m_scene->allObjects()) {
        if (!surfaceOf(obj)) continue;
        const QMatrix4x4 w = m_scene->worldTransform(id);
        const QVector3D a = obj.aabbMin(), b = obj.aabbMax();
        for (int k = 0; k < 8; ++k) {
            const QVector3D p = w.map(QVector3D(k & 1 ? b.x() : a.x(), k & 2 ? b.y() : a.y(), k & 4 ? b.z() : a.z()));
            for (int i = 0; i < 3; ++i) { (*mn)[i] = std::min((*mn)[i], p[i]); (*mx)[i] = std::max((*mx)[i], p[i]); }
        }
    }
    if ((*mn).x() > (*mx).x()) { *mn = QVector3D(-1, -1, -1); *mx = QVector3D(1, 1, 1); }
}

// The cutting plane's outline, a little larger than the scene
void Viewport::drawSectionOutline() {
    if (!m_section.enabled) return;
    QVector3D mn, mx;
    sceneBounds(&mn, &mx);
    const QVector3D pad = (mx - mn) * 0.05f + QVector3D(0.02f, 0.02f, 0.02f);
    mn -= pad;
    mx += pad;
    const int a = m_section.axis, u = (a + 1) % 3, v = (a + 2) % 3;
    auto corner = [&](float cu, float cv) {
        QVector3D p;
        p[a] = m_section.position;
        p[u] = cu;
        p[v] = cv;
        return p;
    };
    const QVector3D c[4] = {corner(mn[u], mn[v]), corner(mx[u], mn[v]), corner(mx[u], mx[v]), corner(mn[u], mx[v])};
    std::vector<float> verts;
    for (int i = 0; i < 4; ++i) {
        const QVector3D& p = c[i];
        const QVector3D& q = c[(i + 1) % 4];
        verts.insert(verts.end(), {p.x(), p.y(), p.z(), q.x(), q.y(), q.z()});
    }
    const QColor sc = Theme::colors().warning;
    drawWireLines(verts, QVector3D(sc.redF(), sc.greenF(), sc.blueF()), QMatrix4x4(), 0.8f);
}

// Beams and zones of sensor signals, read live from the current pose:
// green while clear, orange while something is detected. Part flow: a
// source's template outlined where its copies appear, a sink's zone in red.
void Viewport::drawSensors() {
    const QVector3D clear(0.20f, 0.85f, 0.45f), detected(1.0f, 0.42f, 0.10f);
    QMatrix4x4 ident;
    auto obbMatrix = [](const scenequery::Obb& z) {
        QMatrix4x4 m;
        m.setColumn(0, QVector4D(z.axis[0], 0));
        m.setColumn(1, QVector4D(z.axis[1], 0));
        m.setColumn(2, QVector4D(z.axis[2], 0));
        m.setColumn(3, QVector4D(z.center, 1));
        return m;
    };
    for (auto& [id, sg] : m_scene->allSignals()) {
        if (sg.kind != SignalKind::Sensor || !sensors::problem(*m_scene, sg.sensor).isEmpty()) continue;
        const sensors::Reading r = sensors::read(*m_scene, sg.sensor);
        const QVector3D col = r.active ? detected : clear;
        if (sensorIsRay(sg.sensor.type)) {
            QVector3D from, dir;
            sensors::beam(*m_scene, sg.sensor, &from, &dir);
            const QVector3D to = from + dir * r.distance;
            // The beam, plus a small cross at the emitter
            const QVector3D u = QVector3D::crossProduct(dir, std::abs(dir.y()) < 0.9f ? QVector3D(0, 1, 0)
                                                                                     : QVector3D(1, 0, 0)).normalized();
            const QVector3D w = QVector3D::crossProduct(dir, u);
            const float k = std::max(0.01f, sg.sensor.range * 0.04f);
            std::vector<float> v;
            auto line = [&](QVector3D a, QVector3D b) { v.insert(v.end(), {a.x(), a.y(), a.z(), b.x(), b.y(), b.z()}); };
            line(from, to);
            line(from - u * k, from + u * k);
            line(from - w * k, from + w * k);
            if (r.active) { line(to - u * k, to + u * k); line(to - w * k, to + w * k); }
            glLineWidth(2.5f);
            drawWireLines(v, col, ident);
            glLineWidth(1.5f);
        } else {
            const scenequery::Obb z = sensors::zone(*m_scene, sg.sensor);
            drawWireLines(wireBox(z.half.x() * 1.01f, z.half.y() * 1.01f, z.half.z() * 1.01f), col, obbMatrix(z));
            // A position sensor marks the frame of the part it finds
            if (sg.sensor.type == SensorType::Position && r.active) {
                const QMatrix4x4 f = scenequery::stripScale(m_scene->worldTransform(r.objectId));
                const float k = std::max(0.02f, std::min({z.half.x(), z.half.y(), z.half.z()}) * 0.6f);
                const std::vector<float> axes = {0, 0, 0, k, 0, 0,  0, 0, 0, 0, k, 0,  0, 0, 0, 0, 0, k};
                glLineWidth(2.5f);
                drawWireLines(axes, col, f);
                glLineWidth(1.5f);
            }
        }
    }

    const Theme::Colors& c = Theme::colors();
    auto rgb = [](const QColor& q) { return QVector3D(q.redF(), q.greenF(), q.blueF()); };
    for (auto& [id, src] : m_scene->allSources()) {
        std::function<void(int)> outline = [&](int oid) {
            const SceneObject* o = m_scene->getObject(oid);
            if (!o) return;
            if (surfaceOf(*o)) {
                const scenequery::Obb b = scenequery::worldBox(*m_scene, oid);
                drawWireLines(wireBox(b.half.x() * 1.02f, b.half.y() * 1.02f, b.half.z() * 1.02f), rgb(c.info), obbMatrix(b), 0.9f);
            }
            for (int ch : o->childIds) outline(ch);
        };
        outline(src.templateId);
    }
    for (auto& [id, sink] : m_scene->allSinks()) {
        if (!sensors::problem(*m_scene, sink.zone).isEmpty()) continue;
        const scenequery::Obb z = sensors::zone(*m_scene, sink.zone);
        glLineWidth(2.0f);
        drawWireLines(wireBox(z.half.x() * 1.01f, z.half.y() * 1.01f, z.half.z() * 1.01f), rgb(c.danger), obbMatrix(z));
        glLineWidth(1.5f);
    }
}

void Viewport::drawSensorLabels(QPainter& p) {
    const Theme::Colors& c = Theme::colors();
    for (auto& [id, sg] : m_scene->allSignals()) {
        if (sg.kind != SignalKind::Sensor || !sensors::problem(*m_scene, sg.sensor).isEmpty()) continue;
        const QVector3D at = sensorIsRay(sg.sensor.type)
                                 ? sensors::frame(*m_scene, sg.sensor).column(3).toVector3D()
                                 : sensors::zone(*m_scene, sg.sensor).center;
        QPointF pt;
        if (!toScreen(at, &pt)) continue;
        const sensors::Reading r = sensors::read(*m_scene, sg.sensor);
        QString text = sg.name + (r.active ? "  ●" : "  ○");
        if (sg.sensor.type == SensorType::Distance)
            text = QString("%1  %2 mm").arg(sg.name).arg(double(r.distance) * 1000, 0, 'f', 1);
        drawPill(p, pt + QPointF(8, -24), text, r.active ? c.warning : QColor());
    }
    // Part flow: a source over where its copies appear, a sink over its zone
    for (auto& [id, src] : m_scene->allSources()) {
        if (!m_scene->getObject(src.templateId)) continue;
        QPointF pt;
        if (!toScreen(scenequery::worldBox(*m_scene, src.templateId).center, &pt)) continue;
        drawPill(p, pt + QPointF(8, -24), src.name, c.info);
    }
    for (auto& [id, sink] : m_scene->allSinks()) {
        if (!sensors::problem(*m_scene, sink.zone).isEmpty()) continue;
        QPointF pt;
        if (!toScreen(sensors::zone(*m_scene, sink.zone).center, &pt)) continue;
        drawPill(p, pt + QPointF(8, -24), sink.name, c.danger);
    }
}

// ── Key point overlay ─────────────────────────────────────────────────────────

// Text on a rounded, theme-coloured chip whose top-left corner is at `topLeft`
void Viewport::drawPill(QPainter& p, const QPointF& topLeft, const QString& text, const QColor& border) {
    const Theme::Colors& c = Theme::colors();
    QFontMetricsF fm(p.font());
    QRectF r(topLeft, QSizeF(fm.horizontalAdvance(text) + 18, fm.height() + 8));
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(QPen(border.isValid() ? border : c.border, 1));
    p.setBrush(Theme::soft(c.panel, 0.92));
    p.drawRoundedRect(r.adjusted(0.5, 0.5, -0.5, -0.5), r.height() / 2, r.height() / 2);
    p.setPen(c.text);
    p.drawText(r, Qt::AlignCenter, text);
    p.restore();
}

void Viewport::drawKeyPoints(QPainter& p) {
    if (m_hoveredId == -1 || m_axisPickId != -1 || m_assembleMode) { m_nearKeyPointIdx = -1; return; }
    const SceneObject* obj = m_scene->getObject(m_hoveredId);
    if (!obj) { m_nearKeyPointIdx = -1; return; }

    auto kps = keyPointsFor(*obj);
    if (kps.empty()) return;

    QMatrix4x4 wt = m_scene->worldTransform(m_hoveredId);
    float aspect = float(m_vpW) / float(std::max(m_vpH, 1));
    QMatrix4x4 vp = m_camera.projectionMatrix(aspect) * m_camera.viewMatrix();

    // Compute screen positions
    struct ScreenPt { QPointF pos; bool valid; };
    std::vector<ScreenPt> spts;
    spts.reserve(kps.size());
    for (auto& kp : kps) {
        QVector4D clip = vp * QVector4D(wt.map(kp.localPos), 1.0f);
        if (clip.w() <= 0.001f) { spts.push_back({{-9999,-9999}, false}); continue; }
        float nx = clip.x()/clip.w(), ny = clip.y()/clip.w();
        float sx = (nx+1.f)*0.5f*m_vpW, sy = (1.f-ny)*0.5f*m_vpH;
        spts.push_back({{sx, sy}, true});
    }

    // Find nearest to cursor
    m_nearKeyPointIdx = -1;
    float minDist = 14.f;
    for (int i = 0; i < (int)spts.size(); i++) {
        if (!spts[i].valid) continue;
        float dx = m_lastMouse.x() - spts[i].pos.x();
        float dy = m_lastMouse.y() - spts[i].pos.y();
        float d  = sqrtf(dx*dx + dy*dy);
        if (d < minDist) { minDist = d; m_nearKeyPointIdx = i; }
    }

    // Draw dots
    p.setRenderHint(QPainter::Antialiasing);
    for (int i = 0; i < (int)spts.size(); i++) {
        if (!spts[i].valid) continue;
        bool near = (i == m_nearKeyPointIdx);
        int r = near ? 7 : 5;
        p.setPen(QPen(Theme::colors().info, near ? 2.0 : 1.5));
        p.setBrush(near ? QColor(255, 220, 40, 235) : Theme::soft(Theme::colors().panel, 0.85));
        p.drawEllipse(spts[i].pos, r, r);
    }

    // Label for nearest point
    if (m_nearKeyPointIdx != -1 && spts[m_nearKeyPointIdx].valid) {
        QPointF sp = spts[m_nearKeyPointIdx].pos;
        QVector3D wp = wt.map(kps[m_nearKeyPointIdx].localPos);
        QString lbl = QString("%1  (%2, %3, %4)")
            .arg(kps[m_nearKeyPointIdx].name)
            .arg(double(wp.x()), 0, 'f', 3)
            .arg(double(wp.y()), 0, 'f', 3)
            .arg(double(wp.z()), 0, 'f', 3);
        QFont f = font(); f.setPointSize(8); f.setBold(true); p.setFont(f);
        drawPill(p, sp + QPointF(12, -10), lbl);
    }
}

// ── Translate gizmo ───────────────────────────────────────────────────────────

float Viewport::gizmoArrowLength(int objectId) const {
    const SceneObject* obj = m_scene->getObject(objectId);
    if (!obj) return 1.0f;
    // Extent = scale × local AABB size (unit box for primitives, real AABB for meshes)
    QVector3D ext = obj->aabbMax() - obj->aabbMin();
    float sx = std::abs(obj->transform.scale.x() * ext.x());
    float sy = std::abs(obj->transform.scale.y() * ext.y());
    float sz = std::abs(obj->transform.scale.z() * ext.z());
    return std::max({sx, sy, sz}) * 0.5f + 0.8f;
}

float Viewport::projectOntoAxis(int axis, const QVector3D& center, int mx, int my) const {
    float aspect = float(m_vpW) / float(std::max(m_vpH, 1));
    float ndcX = (2.0f * mx) / m_vpW - 1.0f;
    float ndcY = 1.0f - (2.0f * my) / m_vpH;
    QVector3D rayOrig, rayDir;
    m_camera.ray(ndcX, ndcY, aspect, &rayOrig, &rayDir);

    static const float kAxData[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    QVector3D axisDir(kAxData[axis][0], kAxData[axis][1], kAxData[axis][2]);

    QVector3D toCam = (m_camera.projection() == Camera::Projection::Orthographic
                       ? -m_camera.forward() : m_camera.position() - center).normalized();
    // Plane normal = component of toCam perpendicular to axis (most camera-facing plane)
    QVector3D planeNorm = (toCam - axisDir * QVector3D::dotProduct(toCam, axisDir)).normalized();
    if (planeNorm.length() < 1e-6f) return 0.f;

    float denom = QVector3D::dotProduct(rayDir, planeNorm);
    if (std::abs(denom) < 1e-6f) return 0.f;

    float t = QVector3D::dotProduct(center - rayOrig, planeNorm) / denom;
    QVector3D hit = rayOrig + rayDir * t;
    return QVector3D::dotProduct(hit - center, axisDir);
}

int Viewport::hitTestGizmoAxis(int mx, int my) const {
    int selId = m_scene->selectedObjectId();
    if (selId == -1) return -1;
    const SceneObject* obj = m_scene->getObject(selId);
    if (!obj) return -1;

    QMatrix4x4 wt = m_scene->worldTransform(selId);
    QVector3D origin = wt.map(QVector3D(0, 0, 0));
    float arrowLen = gizmoArrowLength(selId);

    float aspect = float(m_vpW) / float(std::max(m_vpH, 1));
    QMatrix4x4 vp = m_camera.projectionMatrix(aspect) * m_camera.viewMatrix();

    auto toScreen = [&](QVector3D wp) -> QPointF {
        QVector4D clip = vp * QVector4D(wp, 1.0f);
        if (clip.w() <= 0.001f) return {-9999, -9999};
        float nx = clip.x()/clip.w(), ny = clip.y()/clip.w();
        return {(nx+1.f)*0.5f*m_vpW, (1.f-ny)*0.5f*m_vpH};
    };

    static const float kAxData[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    QPointF cursor(mx, my);

    int   bestAxis = -1;
    float bestDist = 10.f;   // pixel threshold

    for (int i = 0; i < 3; i++) {
        QVector3D tipW = origin + QVector3D(kAxData[i][0], kAxData[i][1], kAxData[i][2]) * arrowLen;
        QPointF a = toScreen(origin);
        QPointF b = toScreen(tipW);

        QPointF ab = b - a;
        QPointF ac = cursor - a;
        float len2 = float(ab.x()*ab.x() + ab.y()*ab.y());
        float dist;
        if (len2 < 1.f) {
            dist = sqrtf(float(ac.x()*ac.x() + ac.y()*ac.y()));
        } else {
            float t = std::clamp(float((ac.x()*ab.x() + ac.y()*ab.y()) / len2), 0.f, 1.f);
            QPointF proj = a + ab * t;
            QPointF dp   = cursor - proj;
            dist = sqrtf(float(dp.x()*dp.x() + dp.y()*dp.y()));
        }
        if (dist < bestDist) { bestDist = dist; bestAxis = i; }
    }
    return bestAxis;
}

void Viewport::drawTranslateGizmo() {
    int selId = m_scene->selectedObjectId();
    if (selId == -1) return;
    const SceneObject* obj = m_scene->getObject(selId);
    if (!obj) return;

    QMatrix4x4 wt = m_scene->worldTransform(selId);
    QVector3D origin = wt.map(QVector3D(0, 0, 0));
    float arrowLen = gizmoArrowLength(selId);
    float headLen  = arrowLen * 0.18f;
    float headW    = arrowLen * 0.09f;

    static const float kAxData[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    // Base colors: X=red, Y=green, Z=blue
    static const float kBaseCol[3][3] = {
        {0.92f, 0.15f, 0.15f},
        {0.10f, 0.82f, 0.18f},
        {0.18f, 0.22f, 0.95f},
    };
    // Perpendicular pairs for arrowhead cross
    static const float kP1[3][3] = {{0,1,0},{1,0,0},{1,0,0}};
    static const float kP2[3][3] = {{0,0,1},{0,0,1},{0,1,0}};

    QMatrix4x4 ident; ident.setToIdentity();

    glDisable(GL_DEPTH_TEST);
    glLineWidth(3.0f);

    for (int i = 0; i < 3; i++) {
        QVector3D ax(kAxData[i][0], kAxData[i][1], kAxData[i][2]);
        QVector3D p1(kP1[i][0], kP1[i][1], kP1[i][2]);
        QVector3D p2(kP2[i][0], kP2[i][1], kP2[i][2]);

        QVector3D tip  = origin + ax * arrowLen;
        QVector3D back = tip    - ax * headLen;

        std::vector<float> av;
        auto addLine = [&](QVector3D a, QVector3D b) {
            av.insert(av.end(), {a.x(),a.y(),a.z(), b.x(),b.y(),b.z()});
        };

        addLine(origin, tip);            // shaft
        addLine(tip, back + p1 * headW); // head
        addLine(tip, back - p1 * headW);
        addLine(tip, back + p2 * headW);
        addLine(tip, back - p2 * headW);

        bool lit = (m_gizmoDragAxis == i) || (m_gizmoHoveredAxis == i && m_gizmoDragAxis == -1);
        QVector3D col = lit ? QVector3D(1.0f, 0.85f, 0.05f)
                            : QVector3D(kBaseCol[i][0], kBaseCol[i][1], kBaseCol[i][2]);

        drawWireLines(av, col, ident);
    }

    glEnable(GL_DEPTH_TEST);
    glLineWidth(1.0f);
}

// ── Axis gizmo ────────────────────────────────────────────────────────────────

void Viewport::drawAxisGizmo(){
    int sz=kGizmoSize, mg=kGizmoMargin;

    glEnable(GL_SCISSOR_TEST);
    glScissor(mg, mg, sz, sz);
    glViewport(mg, mg, sz, sz);
    glClear(GL_DEPTH_BUFFER_BIT);

    m_gizmoView = m_camera.viewMatrix();
    m_gizmoView.setColumn(3, {0, 0, -2.5f, 1});

    m_gizmoProj.setToIdentity();
    m_gizmoProj.ortho(-1.4f, 1.4f, -1.4f, 1.4f, 0.1f, 10.0f);

    m_gridShader->bind();
    m_gridShader->setUniformValue("view", m_gizmoView);
    m_gridShader->setUniformValue("proj", m_gizmoProj);

    glBindVertexArray(m_axisVao);
    glLineWidth(2.5f);

    m_gridShader->setUniformValue("lineColor", QVector3D(0.92f, 0.18f, 0.18f));
    glDrawArrays(GL_LINES, 0, 2);
    m_gridShader->setUniformValue("lineColor", QVector3D(0.15f, 0.78f, 0.20f));
    glDrawArrays(GL_LINES, 2, 2);
    m_gridShader->setUniformValue("lineColor", QVector3D(0.18f, 0.40f, 0.95f));
    glDrawArrays(GL_LINES, 4, 2);

    glBindVertexArray(0);
    glLineWidth(1.0f);
    m_gridShader->release();

    glDisable(GL_SCISSOR_TEST);
}

void Viewport::drawAxisLabels(QPainter& p){
    int sz=kGizmoSize, mg=kGizmoMargin;

    auto project=[&](QVector3D wp) -> QPointF {
        QVector4D clip = m_gizmoProj * m_gizmoView * QVector4D(wp, 1.0f);
        float ndcX = clip.x() / clip.w();
        float ndcY = clip.y() / clip.w();
        float glX  = mg + (ndcX + 1.0f) * 0.5f * sz;
        float glY  = mg + (ndcY + 1.0f) * 0.5f * sz;
        return { glX, float(m_vpH) - glY };
    };

    struct { QVector3D tip; QColor col; QString label; } axes[] = {
        { {1,0,0}, Theme::colors().axisX, "X" },
        { {0,1,0}, Theme::colors().axisY, "Y" },
        { {0,0,1}, Theme::colors().axisZ, "Z" },
    };

    QFont f = font();
    f.setBold(true);
    f.setPointSize(9);
    p.setFont(f);

    for(auto& ax : axes){
        QPointF pos = project(ax.tip);
        p.setPen(QColor(0,0,0,60));
        p.drawText(pos + QPointF(3,3), ax.label);
        p.setPen(ax.col);
        p.drawText(pos + QPointF(2,2), ax.label);
    }
}

void Viewport::drawAxisPickBanner(QPainter& p){
    if(m_axisPickId == -1) return;

    const SceneObject* obj = m_scene->getObject(m_axisPickId);
    QString name = obj ? obj->name : QString("object");
    QString text = QString("Joint axis for \"%1\": click a round edge, cylinder, edge or face  ·  Esc cancels").arg(name);

    QFont f = font(); f.setPointSize(9); f.setBold(true);
    p.setFont(f);
    QFontMetrics fm(f);
    const double pillW = fm.horizontalAdvance(text) + 18;
    // Centred, but kept clear of the view cube
    const double x = std::max(8.0, std::min((m_vpW - pillW) / 2.0, m_viewCube->x() - pillW - 8.0));
    drawPill(p, QPointF(x, 46), text, Theme::colors().info);
}

void Viewport::drawKeyHints(QPainter& p){
    static const QString hints[] = {
        "MMB  Orbit",
        "RMB  Pan",
        "Scroll  Zoom",
        "Click  Select",
        "F  Fit all",
    };
    QFont f = font();
    f.setPointSize(8);
    p.setFont(f);

    int x = m_vpW - 120;
    int y = m_vpH - 14 - 4*16;

    p.setPen(Theme::colors().muted);
    for(auto& h : hints){
        p.drawText(x, y, h);
        y += 16;
    }
}

// ── Hover / selection helpers ─────────────────────────────────────────────────

bool Viewport::rayAABB(const QVector3D& orig,const QVector3D& dir,
                        const QVector3D& mn,  const QVector3D& mx, float& t){
    float tMin=-std::numeric_limits<float>::max(), tMax=std::numeric_limits<float>::max();
    for(int i=0;i<3;i++){
        float d=(i==0)?dir.x():(i==1)?dir.y():dir.z();
        float o=(i==0)?orig.x():(i==1)?orig.y():orig.z();
        float lo=(i==0)?mn.x():(i==1)?mn.y():mn.z();
        float hi=(i==0)?mx.x():(i==1)?mx.y():mx.z();
        if(std::abs(d)<1e-8f){ if(o<lo||o>hi) return false; }
        else {
            float t1=(lo-o)/d, t2=(hi-o)/d;
            if(t1>t2) std::swap(t1,t2);
            tMin=std::max(tMin,t1); tMax=std::min(tMax,t2);
            if(tMin>tMax) return false;
        }
    }
    t=tMin; return tMin>=0;
}

// The object whose drawn surface the ray meets first (see scenequery::rayCast).
// Mostly see-through parts are clicked through when something solid is behind
// them; invisible ones can't be picked.
std::pair<int,float> Viewport::pickNearest(const QVector3D& orig, const QVector3D& dir) const {
    const scenequery::RayHit hit = scenequery::rayCast(*m_scene, orig, dir,
        std::numeric_limits<float>::max(), [this](int id){ return m_scene->opacityOf(id) > 0.0f; });
    if (hit.hit() && seeThrough(hit.objectId)) {
        const scenequery::RayHit behind = scenequery::rayCast(*m_scene, orig, dir,
            std::numeric_limits<float>::max(), [this](int id){ return !seeThrough(id); });
        if (behind.hit()) return {behind.objectId, behind.t};
    }
    return {hit.objectId, hit.t};
}

bool Viewport::seeThrough(int id) const { return m_scene->opacityOf(id) < 0.5f; }

void Viewport::tryPickObject(int x, int y){
    float aspect=float(m_vpW)/float(std::max(m_vpH,1));
    float ndcX=(2.0f*x)/m_vpW-1.0f, ndcY=1.0f-(2.0f*y)/m_vpH;
    QVector3D orig, dir;
    m_camera.ray(ndcX, ndcY, aspect, &orig, &dir);
    auto [id,t]=pickNearest(orig, dir);
    m_scene->selectObject(id);
}

void Viewport::updateHover(int x, int y){
    float aspect=float(m_vpW)/float(std::max(m_vpH,1));
    float ndcX=(2.0f*x)/m_vpW-1.0f, ndcY=1.0f-(2.0f*y)/m_vpH;
    QVector3D orig, dir;
    m_camera.ray(ndcX, ndcY, aspect, &orig, &dir);
    auto [id,t]=pickNearest(orig, dir);
    if(id!=m_hoveredId){
        m_hoveredId=id;
        setCursor(id!=-1 ? Qt::PointingHandCursor : Qt::ArrowCursor);
        update();
    }
}

// ── Mouse events ──────────────────────────────────────────────────────────────

void Viewport::mousePressEvent(QMouseEvent* e){
    m_lastMouse=e->pos();
    m_dragBtn=e->button();
    if(m_dragBtn!=Qt::LeftButton || (e->modifiers()&Qt::AltModifier)) m_viewAnimation->stop();   // orbiting or panning takes over

    if(m_axisPickId != -1){
        // Axis-pick mode: clicking a gizmo arrow commits its world direction
        // (converted to the parent's local frame) as the joint's axis instead
        // of starting a drag; clicking geometry commits the hovered candidate.
        // A miss just stays in pick mode.
        if(e->button()==Qt::LeftButton){
            int axisHit = m_scene->selectedObjectId() == m_axisPickId
                              ? hitTestGizmoAxis(e->pos().x(), e->pos().y()) : -1;
            if(axisHit == -1){
                updateAxisCandidate(e->pos().x(), e->pos().y());
                commitAxisCandidate();
            } else {
                static const QVector3D kWorldAx[3] = {{1,0,0},{0,1,0},{0,0,1}};
                const SceneObject* obj = m_scene->getObject(m_axisPickId);
                QVector3D localAxis = kWorldAx[axisHit];
                if(obj && obj->parentId != -1)
                    localAxis = m_scene->worldTransform(obj->parentId).inverted().mapVector(localAxis).normalized();
                int pickedId = m_axisPickId;
                cancelAxisPick();
                emit axisPicked(pickedId, localAxis, QVector3D(), false);
            }
        }
        return;
    }

    if(m_assembleMode && e->button()==Qt::LeftButton && !(e->modifiers()&Qt::AltModifier)){
        handleMateClick(e->pos().x(), e->pos().y());
        return;
    }

    if(m_measureMode && e->button()==Qt::LeftButton && !(e->modifiers()&Qt::AltModifier)){
        const MeasurePoint p = measurePointAt(e->pos().x(), e->pos().y());
        if(p.valid){
            if(!m_measureA.valid || m_measureB.valid){ m_measureA = p; m_measureB = MeasurePoint{}; }
            else m_measureB = p;
            update();
        }
        return;
    }

    if(e->button()==Qt::LeftButton && !(e->modifiers()&Qt::AltModifier)){
        // Gizmo drag takes highest priority
        int axisHit = hitTestGizmoAxis(e->pos().x(), e->pos().y());
        if (axisHit != -1) {
            int selId = m_scene->selectedObjectId();
            const SceneObject* obj = m_scene->getObject(selId);
            if (obj) {
                m_ikDragChain.clear();
                if (ikDragActive()) {
                    m_ikDragChain = kin::chainTo(*m_scene, selId);
                    m_ikDragStartFrame = kin::toolFrame(*m_scene, selId);
                }
                m_gizmoDragAxis       = axisHit;
                m_gizmoDragStartPos   = obj->transform.position;
                m_gizmoDragWorldCenter = m_scene->worldTransform(selId).map(QVector3D(0,0,0));
                m_gizmoDragStartT     = projectOntoAxis(axisHit, m_gizmoDragWorldCenter,
                                                         e->pos().x(), e->pos().y());
                setCursor(Qt::SizeAllCursor);
                update();
                return;
            }
        }

        // Key point click
        if (m_nearKeyPointIdx != -1 && m_hoveredId != -1) {
            const SceneObject* obj = m_scene->getObject(m_hoveredId);
            if (obj) {
                auto kps = keyPointsFor(*obj);
                if (m_nearKeyPointIdx < (int)kps.size()) {
                    QVector3D wp = m_scene->worldTransform(m_hoveredId).map(kps[m_nearKeyPointIdx].localPos);
                    emit keyPointSelected(kps[m_nearKeyPointIdx].name, wp);
                }
            }
        }
        makeCurrent();
        tryPickObject(e->pos().x(), e->pos().y());
        doneCurrent();
    }
}

void Viewport::mouseMoveEvent(QMouseEvent* e){
    QPoint pos   = e->pos();
    QPoint delta = pos - m_lastMouse;
    m_lastMouse  = pos;

    // Gizmo axis drag — highest priority, captures all LMB motion
    if (m_gizmoDragAxis != -1) {
        int selId = m_scene->selectedObjectId();
        SceneObject* obj = m_scene->getObject(selId);
        if (obj) {
            float newT = projectOntoAxis(m_gizmoDragAxis, m_gizmoDragWorldCenter, pos.x(), pos.y());
            float axisDelta = newT - m_gizmoDragStartT;

            static const float kAxData[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
            QVector3D ax(kAxData[m_gizmoDragAxis][0], kAxData[m_gizmoDragAxis][1], kAxData[m_gizmoDragAxis][2]);
            QVector3D worldDelta = ax * axisDelta;

            if (!m_ikDragChain.empty()) {
                // Joints follow the tool; the transform is not edited
                kin::Target t = kin::targetFrom(m_ikDragStartFrame);
                t.position += worldDelta;
                t.useOrientation = kin::canOrient(m_ikDragChain);
                kin::solve(*m_scene, selId, m_ikDragChain, t, 40);
                m_scene->notifyJointsMoved();
                update();
                return;
            }
            if (obj->parentId == -1) {
                obj->transform.position = m_gizmoDragStartPos + worldDelta;
            } else {
                QMatrix4x4 parentWorldInv = m_scene->worldTransform(obj->parentId).inverted();
                QVector3D newWorldPos = m_gizmoDragWorldCenter + worldDelta;
                obj->transform.position = parentWorldInv.map(newWorldPos);
            }
            emit m_scene->objectChanged(selId);
            // Mated parts slide only where their mates let them, and parts
            // mated to this one come along
            if(m_assemblyAvailable && !m_scene->allMates().empty()) assembly::solve(*m_scene);
        }
        update();
        return;
    }

    if(m_dragBtn==Qt::NoButton && m_assembleMode){
        m_mateHover = matePickAt(pos.x(), pos.y());
        setCursor(Qt::CrossCursor);
        update();
        return;
    }
    if(m_dragBtn==Qt::NoButton && m_measureMode){
        m_measureHover = measurePointAt(pos.x(), pos.y());
        setCursor(Qt::CrossCursor);
        update();
        return;
    }
    if(m_dragBtn==Qt::NoButton){
        updateHover(pos.x(), pos.y());
        if(m_axisPickId != -1){
            updateAxisCandidate(pos.x(), pos.y());
            setCursor(Qt::CrossCursor);
        }
        // Update gizmo hover and override cursor if needed
        int newGizmoHover = hitTestGizmoAxis(pos.x(), pos.y());
        if (newGizmoHover != m_gizmoHoveredAxis) {
            m_gizmoHoveredAxis = newGizmoHover;
            update();
        }
        if (m_gizmoHoveredAxis != -1)
            setCursor(Qt::SizeAllCursor);
        if(m_hoveredId != -1) update();
        return;
    }

    bool alt=e->modifiers()&Qt::AltModifier;
    if(m_dragBtn==Qt::MiddleButton || (m_dragBtn==Qt::LeftButton&&alt)){
        if(e->modifiers()&Qt::ShiftModifier)
            m_camera.pan(delta.x(), delta.y(), float(m_vpH));
        else
            m_camera.orbit(delta.x(), delta.y());
        cameraChanged();
    } else if(m_dragBtn==Qt::RightButton){
        m_camera.pan(delta.x(), delta.y(), float(m_vpH));
        cameraChanged();
    }
}

void Viewport::mouseReleaseEvent(QMouseEvent* e){
    if (m_gizmoDragAxis != -1) {
        m_gizmoDragAxis = -1;
        // Restore cursor based on current state
        int axisHov = hitTestGizmoAxis(e->pos().x(), e->pos().y());
        setCursor(axisHov != -1 ? Qt::SizeAllCursor :
                  m_hoveredId != -1 ? Qt::PointingHandCursor : Qt::ArrowCursor);
        update();
    }
    m_dragBtn = Qt::NoButton;
}

void Viewport::wheelEvent(QWheelEvent* e){
    const QPointF pos = e->position();
    m_viewAnimation->stop();
    m_camera.zoomAt(e->angleDelta().y()/120.0f, float(2.0*pos.x()/m_vpW - 1.0), float(1.0 - 2.0*pos.y()/m_vpH),
                    float(m_vpW)/float(std::max(m_vpH,1)));
    cameraChanged();
}

void Viewport::keyPressEvent(QKeyEvent* e){
    if(e->key() == Qt::Key_Escape){
        if(m_axisPickId != -1){ cancelAxisPick(); return; }
        if(m_measureMode){ setMeasureMode(false); return; }
        if(m_assembleMode){
            if(m_mateFirst.valid){ m_mateFirst = MatePick{}; m_assemblePanel->setStage(0); update(); }
            else if(m_assemblePanel->editedMate() >= 0) m_assemblePanel->finishEdit();
            else setAssembleMode(false);
            return;
        }
    }
    if(m_assembleMode && (e->key() == Qt::Key_Return || e->key() == Qt::Key_Enter)){
        m_assemblePanel->finishEdit();
        return;
    }
    if(e->key() == Qt::Key_F && e->modifiers() == Qt::NoModifier){ fitAll(); return; }
    QOpenGLWidget::keyPressEvent(e);
}

// ── Axis-pick mode ────────────────────────────────────────────────────────────

void Viewport::beginAxisPick(int objectId){
    if(!m_scene->getObject(objectId)) return;
    m_axisPickId = objectId;
    m_axisCandidate = {};
    setFocus();
    setCursor(Qt::CrossCursor);
    update();
}

void Viewport::cancelAxisPick(){
    if(m_axisPickId == -1) return;
    m_axisPickId = -1;
    m_axisCandidate = {};
    setCursor(Qt::ArrowCursor);
    update();
}

const MeshFeatures* Viewport::featuresFor(const SceneObject& obj){
    if(obj.type == PrimitiveType::ImportedMesh){
        if(!obj.mesh || obj.mesh->interleaved.empty()) return nullptr;
        FeatureCache& cache = m_meshFeatures[obj.mesh.get()];
        if(!cache.features || cache.geometry.expired()){
            cache.geometry = obj.mesh;
            cache.features = std::make_unique<MeshFeatures>(obj.mesh->interleaved);
        }
        return cache.features.get();
    }
    const int k = int(obj.type);
    if(!m_primitiveFeatures[k] && !primitiveSoup(obj.type).empty())
        m_primitiveFeatures[k] = std::make_unique<MeshFeatures>(primitiveSoup(obj.type));
    return m_primitiveFeatures[k].get();
}

bool Viewport::toScreen(const QVector3D& world, QPointF* out) const {
    const float aspect = float(m_vpW) / float(std::max(m_vpH, 1));
    const QVector4D clip = m_camera.projectionMatrix(aspect) * m_camera.viewMatrix() * QVector4D(world, 1.0f);
    if(clip.w() <= 1e-6f) return false;
    *out = QPointF((clip.x() / clip.w() + 1.0f) * 0.5f * m_vpW, (1.0f - clip.y() / clip.w()) * 0.5f * m_vpH);
    return true;
}


static float distanceToSegment(const QPointF& p, const QPointF& a, const QPointF& b){
    const QPointF ab = b - a;
    const double len2 = QPointF::dotProduct(ab, ab);
    const double t = len2 > 0 ? std::clamp(QPointF::dotProduct(p - a, ab) / len2, 0.0, 1.0) : 0.0;
    const QPointF d = p - (a + ab * t);
    return float(std::hypot(d.x(), d.y()));
}

int Viewport::featureTriangleAt(const QVector3D& orig, const QVector3D& dir, int* objectId, float* worldT){
    // Solid parts first, so mostly see-through ones are picked through (as in
    // pickNearest); a see-through part only when nothing solid is behind it
    for(const bool throughFaded : {false, true}){
        const int tri = featureTriangleAt(orig, dir, objectId, worldT, throughFaded);
        if(tri != -1) return tri;
    }
    return -1;
}

int Viewport::featureTriangleAt(const QVector3D& orig, const QVector3D& dir, int* objectId, float* worldT,
                                bool fadedOnly){
    int hitTri = -1;
    float bestT = std::numeric_limits<float>::max();
    *objectId = -1;
    for(auto& [id, obj] : m_scene->allObjects()){
        if(seeThrough(id) != fadedOnly || m_scene->opacityOf(id) <= 0.0f) continue;
        const MeshFeatures* features = featuresFor(obj);
        if(!features) continue;
        const QMatrix4x4 wt = m_scene->worldTransform(id);
        bool invertible = false;
        const QMatrix4x4 inv = wt.inverted(&invertible);
        if(!invertible) continue;
        const QVector3D lo = inv.map(orig), ld = inv.mapVector(dir);
        if(!scenequery::rayMeetsBox(lo, ld, obj.aabbMin(), obj.aabbMax())) continue;
        float t;
        const int tri = features->intersect(lo, ld, &t);
        if(tri == -1) continue;
        const float wt2 = QVector3D::dotProduct(wt.map(lo + ld * t) - orig, dir);
        if(wt2 < bestT){ bestT = wt2; hitTri = tri; *objectId = id; }
    }
    if(worldT) *worldT = bestT;
    return hitTri;
}

// ── Measure ──────────────────────────────────────────────────────────────────

void Viewport::setMeasureMode(bool on){
    if(on == m_measureMode) return;
    if(on){ cancelAxisPick(); setAssembleMode(false); }
    m_measureMode = on;
    m_measureA = m_measureB = m_measureHover = MeasurePoint{};
    setCursor(on ? Qt::CrossCursor : Qt::ArrowCursor);
    if(on) setFocus();
    emit measureModeChanged(on);
    update();
}

Viewport::MeasurePoint Viewport::measurePointAt(int x, int y){
    MeasurePoint mp;
    const float aspect = float(m_vpW) / float(std::max(m_vpH, 1));
    QVector3D orig, dir;
    m_camera.ray((2.0f * x) / m_vpW - 1.0f, 1.0f - (2.0f * y) / m_vpH, aspect, &orig, &dir);
    int objectId = -1;
    float t = 0;
    const int tri = featureTriangleAt(orig, dir, &objectId, &t);
    if(tri < 0) return mp;
    const SceneObject* obj = m_scene->getObject(objectId);
    const MeshFeatures* features = featuresFor(*obj);
    const std::vector<float>* soup = surfaceOf(*obj);
    const QMatrix4x4 wt = m_scene->worldTransform(objectId);
    const QMatrix4x4 normalWt = wt.inverted().transposed();
    mp.valid = true;
    mp.pos = orig + dir * t;
    if(soup && size_t(tri) * 18 + 18 <= soup->size()){
        const float* f = &(*soup)[size_t(tri) * 18];
        const QVector3D a(f[0], f[1], f[2]), b(f[6], f[7], f[8]), c(f[12], f[13], f[14]);
        mp.normal = normalWt.mapVector(QVector3D::crossProduct(b - a, c - a)).normalized();
    }
    auto fmt = [](float m){ return m < 1.0f ? QString("%1 mm").arg(double(m) * 1000, 0, 'f', 1)
                                            : QString("%1 m").arg(double(m), 0, 'f', 3); };
    const float scale = wt.mapVector(QVector3D(1, 0, 0)).length();   // for radii (uniform scale assumed)

    // Snap: a round edge (its centre) or an end of a straight edge near the cursor
    float nearest = 10.0f;   // px
    for(const AxisFeature* edge : features->edgesAround(tri)){
        if(edge->kind == AxisFeature::Kind::CircularEdge){
            for(size_t i = 0; i + 1 < edge->outline.size(); i += 2){
                QPointF pa, pb;
                if(!toScreen(wt.map(edge->outline[i]), &pa) || !toScreen(wt.map(edge->outline[i + 1]), &pb)) continue;
                const float d = distanceToSegment(QPointF(x, y), pa, pb);
                if(d >= nearest) continue;
                nearest = d;
                mp.pos = wt.map(edge->origin);
                mp.normal = normalWt.mapVector(edge->axis).normalized();
                mp.what = "centre of Ø " + fmt(2 * edge->radius * scale);
                mp.outline.clear();
                for(const QVector3D& q : edge->outline) mp.outline.push_back(wt.map(q));
            }
        } else if(edge->kind == AxisFeature::Kind::StraightEdge && edge->outline.size() >= 2){
            // Its two ends: the extreme points along the edge direction
            QVector3D lo = edge->outline.front(), hi = lo;
            for(const QVector3D& q : edge->outline){
                if(QVector3D::dotProduct(q - lo, edge->axis) < 0) lo = q;
                if(QVector3D::dotProduct(q - hi, edge->axis) > 0) hi = q;
            }
            for(const QVector3D& end : {lo, hi}){
                QPointF ps;
                if(!toScreen(wt.map(end), &ps)) continue;
                const float d = float(std::hypot(ps.x() - x, ps.y() - y));
                if(d >= nearest) continue;
                nearest = d;
                mp.pos = wt.map(end);
                mp.what = "corner";
                mp.outline.clear();
                for(const QVector3D& q : edge->outline) mp.outline.push_back(wt.map(q));
            }
        }
    }
    if(mp.what.isEmpty()){
        const AxisFeature& face = features->faceFeature(tri);
        if(face.kind == AxisFeature::Kind::CylindricalFace){
            // On a bore or shaft: the point on its centre line
            const QVector3D o = wt.map(face.origin), a = wt.mapVector(face.axis).normalized();
            mp.pos = o + a * QVector3D::dotProduct(mp.pos - o, a);
            mp.normal = QVector3D();
            mp.what = "axis of Ø " + fmt(2 * face.radius * scale);
            for(const QVector3D& q : face.outline) mp.outline.push_back(wt.map(q));
        } else if(face.kind == AxisFeature::Kind::FlatFace){
            mp.normal = normalWt.mapVector(face.axis).normalized();
            mp.what = "on " + obj->name;
        } else {
            mp.what = "on " + obj->name;
        }
    }
    return mp;
}

void Viewport::drawMeasure(QPainter& p){
    if(!m_measureMode) return;
    const Theme::Colors& c = Theme::colors();
    p.setRenderHint(QPainter::Antialiasing);
    QFont f = font(); f.setPointSize(9); f.setBold(true);
    p.setFont(f);
    QFontMetrics fm(f);
    const QString banner = !m_measureA.valid || m_measureB.valid
        ? "Measure: click a point on a part (snaps to hole centres, bore axes and corners)  ·  Esc to finish"
        : "Measure: click the second point  ·  Esc to finish";
    const double pillW = fm.horizontalAdvance(banner) + 18;
    drawPill(p, QPointF(std::max(8.0, std::min((m_vpW - pillW) / 2.0, m_viewCube->x() - pillW - 8.0)), 46),
             banner, c.info);

    auto outline = [&](const MeasurePoint& mp, const QColor& col){
        p.setPen(QPen(col, 2));
        for(size_t i = 0; i + 1 < mp.outline.size(); i += 2){
            QPointF a, b;
            if(toScreen(mp.outline[i], &a) && toScreen(mp.outline[i + 1], &b)) p.drawLine(a, b);
        }
    };
    auto dot = [&](const MeasurePoint& mp, const QColor& col, double r){
        QPointF s;
        if(!toScreen(mp.pos, &s)) return;
        p.setPen(QPen(c.viewport, 1.5));
        p.setBrush(col);
        p.drawEllipse(s, r, r);
    };
    if(m_measureHover.valid){
        outline(m_measureHover, c.info);
        dot(m_measureHover, c.info, 4);
        QPointF s;
        if(toScreen(m_measureHover.pos, &s) && !m_measureHover.what.isEmpty()){
            QFont small = font(); small.setPointSize(8);
            p.setFont(small);
            drawPill(p, s + QPointF(10, 8), m_measureHover.what);
            p.setFont(f);
        }
    }
    if(m_measureA.valid){ outline(m_measureA, c.warning); dot(m_measureA, c.warning, 5); }
    if(m_measureB.valid){ outline(m_measureB, c.warning); dot(m_measureB, c.warning, 5); }
    if(m_measureA.valid && m_measureB.valid){
        QPointF a, b;
        if(toScreen(m_measureA.pos, &a) && toScreen(m_measureB.pos, &b)){
            p.setPen(QPen(c.warning, 2, Qt::DashLine));
            p.drawLine(a, b);
            const QVector3D d = m_measureB.pos - m_measureA.pos;
            auto fmt = [](float m){ return std::abs(m) < 1.0f ? QString("%1 mm").arg(double(m) * 1000, 0, 'f', 2)
                                                              : QString("%1 m").arg(double(m), 0, 'f', 4); };
            QString text = fmt(d.length());
            if(!m_measureA.normal.isNull() && !m_measureB.normal.isNull()){
                const float cosA = std::clamp(QVector3D::dotProduct(m_measureA.normal, m_measureB.normal), -1.0f, 1.0f);
                text += QString("   ∠ %1°").arg(qRadiansToDegrees(std::acos(cosA)), 0, 'f', 1);
            }
            const QPointF mid = (a + b) / 2;
            drawPill(p, mid + QPointF(10, -26), text, c.warning);
            QFont small = font(); small.setPointSize(8);
            p.setFont(small);
            drawPill(p, mid + QPointF(10, 2), QString("ΔX %1   ΔY %2   ΔZ %3").arg(fmt(d.x()), fmt(d.y()), fmt(d.z())));
            p.setFont(f);
        }
    }
}

// ── Assemble ─────────────────────────────────────────────────────────────────

void Viewport::setAssembleMode(bool on){
    if(on && !m_assemblyAvailable) on = false;
    if(on == m_assembleMode) return;
    if(on){ cancelAxisPick(); setMeasureMode(false); }
    m_assembleMode = on;
    m_mateFirst = m_mateHover = MatePick{};
    if(on){
        m_assemblePanel->setStage(0);
        m_assemblePanel->show();
        m_assemblePanel->raise();
        repositionOverlays();
        setFocus();
    } else {
        m_assemblePanel->finishEdit();
        m_assemblePanel->hide();
    }
    setCursor(on ? Qt::CrossCursor : Qt::ArrowCursor);
    emit assembleModeChanged(on);
    update();
}

void Viewport::setAssemblyAvailable(bool available){
    m_assemblyAvailable = available;
    if(!available) setAssembleMode(false);
}

Viewport::MatePick Viewport::matePickAt(int x, int y){
    using K = MateFeature::Kind;
    MatePick mp;
    const float aspect = float(m_vpW) / float(std::max(m_vpH, 1));
    QVector3D orig, dir;
    m_camera.ray((2.0f * x) / m_vpW - 1.0f, 1.0f - (2.0f * y) / m_vpH, aspect, &orig, &dir);
    int objectId = -1;
    float t = 0;
    const int tri = featureTriangleAt(orig, dir, &objectId, &t);
    if(tri < 0) return mp;
    const SceneObject* obj = m_scene->getObject(objectId);
    const MeshFeatures* features = featuresFor(*obj);
    const QMatrix4x4 wt = m_scene->worldTransform(objectId);
    const QMatrix4x4 normalWt = wt.inverted().transposed();
    const float scale = wt.mapVector(QVector3D(1, 0, 0)).length();
    auto fmt = [](float m){ return m < 1.0f ? QString("%1 mm").arg(double(m) * 1000, 0, 'f', 1)
                                            : QString("%1 m").arg(double(m), 0, 'f', 3); };
    mp.valid = true;
    mp.objectId = objectId;
    mp.point = orig + dir * t;

    // Near the cursor: an edge end (corner), then a round or straight edge
    float nearestEnd = 9.0f, nearestEdge = 7.0f;   // px
    const AxisFeature* edgePick = nullptr;
    QVector3D corner;
    bool haveCorner = false;
    for(const AxisFeature* edge : features->edgesAround(tri)){
        if(edge->kind == AxisFeature::Kind::StraightEdge && edge->outline.size() >= 2){
            QVector3D lo = edge->outline.front(), hi = lo;
            for(const QVector3D& q : edge->outline){
                if(QVector3D::dotProduct(q - lo, edge->axis) < 0) lo = q;
                if(QVector3D::dotProduct(q - hi, edge->axis) > 0) hi = q;
            }
            for(const QVector3D& end : {lo, hi}){
                QPointF ps;
                if(!toScreen(wt.map(end), &ps)) continue;
                const float d = float(std::hypot(ps.x() - x, ps.y() - y));
                if(d < nearestEnd){ nearestEnd = d; corner = end; haveCorner = true; }
            }
        }
        if(edge->kind != AxisFeature::Kind::StraightEdge && edge->kind != AxisFeature::Kind::CircularEdge) continue;
        for(size_t i = 0; i + 1 < edge->outline.size(); i += 2){
            QPointF pa, pb;
            if(!toScreen(wt.map(edge->outline[i]), &pa) || !toScreen(wt.map(edge->outline[i + 1]), &pb)) continue;
            const float d = distanceToSegment(QPointF(x, y), pa, pb);
            if(d < nearestEdge){ nearestEdge = d; edgePick = edge; }
        }
    }
    if(haveCorner){
        mp.kind = K::Point;
        mp.point = wt.map(corner);
        mp.what = "corner";
        // A small cross marks it
        return mp;
    }
    if(edgePick){
        mp.outline = edgePick->outline;
        if(edgePick->kind == AxisFeature::Kind::CircularEdge){
            mp.kind = K::Circle;
            mp.point = wt.map(edgePick->origin);
            mp.direction = normalWt.mapVector(edgePick->axis).normalized();
            // Point it out of the part: along the clicked face's normal when
            // that face is flat (an end or a face with a hole), else away
            // from the part's middle
            const AxisFeature& onFace = features->faceFeature(tri);
            QVector3D out = mp.point - wt.map((obj->aabbMin() + obj->aabbMax()) * 0.5f);
            if(onFace.kind == AxisFeature::Kind::FlatFace){
                out = normalWt.mapVector(onFace.axis);
                if(QVector3D::dotProduct(out, dir) > 0) out = -out;
            }
            if(QVector3D::dotProduct(mp.direction, out) < 0) mp.direction = -mp.direction;
            mp.radius = edgePick->radius * scale;
            mp.what = "round edge Ø " + fmt(2 * mp.radius);
        } else {
            mp.kind = K::Axis;
            mp.point = wt.map(edgePick->origin);
            mp.direction = wt.mapVector(edgePick->axis).normalized();
            mp.what = "straight edge";
        }
        return mp;
    }
    const AxisFeature& face = features->faceFeature(tri);
    mp.outline = face.outline;
    if(face.kind == AxisFeature::Kind::CylindricalFace){
        mp.kind = K::Axis;
        mp.direction = wt.mapVector(face.axis).normalized();
        const QVector3D o = wt.map(face.origin);
        mp.point = o + mp.direction * QVector3D::dotProduct(mp.point - o, mp.direction);
        mp.radius = face.radius * scale;
        mp.what = "axis of Ø " + fmt(2 * mp.radius);
    } else if(face.kind == AxisFeature::Kind::FlatFace){
        mp.kind = K::Plane;
        mp.direction = normalWt.mapVector(face.axis).normalized();
        // The side you clicked faces you: that's the outward normal
        if(QVector3D::dotProduct(mp.direction, dir) > 0) mp.direction = -mp.direction;
        mp.what = "flat face";
    } else {
        mp.kind = K::Point;
        mp.outline.clear();
        mp.what = "point on " + obj->name;
    }
    return mp;
}

void Viewport::handleMateClick(int x, int y){
    const MatePick pick = matePickAt(x, y);
    if(!pick.valid) return;
    const SceneObject* obj = m_scene->getObject(pick.objectId);
    if(!m_mateFirst.valid){
        m_assemblePanel->finishEdit();
        m_mateFirst = pick;
        m_assemblePanel->setStage(1, QString("%1 (%2)").arg(obj->name, pick.what));
        m_assemblePanel->adjustSize();
        update();
        return;
    }
    if(pick.objectId == m_mateFirst.objectId) return;   // a second feature on the same part: ignore
    auto toFeature = [&](const MatePick& p){
        return assembly::fromWorld(*m_scene, p.objectId, p.kind, p.point, p.direction, p.radius);
    };
    const MateFeature moving = toFeature(m_mateFirst), fixed = toFeature(pick);
    m_mateFirst = MatePick{};
    m_assemblePanel->createMate(moving, fixed);
    m_assemblePanel->setStage(0);
    m_assemblePanel->adjustSize();
    m_mateHover = matePickAt(x, y);
    update();
}

void Viewport::drawMateGlyph(QPainter& p, const MateFeature& f, const QColor& color){
    using K = MateFeature::Kind;
    const SceneObject* obj = m_scene->getObject(f.objectId);
    if(!obj) return;
    const assembly::WorldFeature w = assembly::toWorld(*m_scene, f);
    const QMatrix4x4 wt = m_scene->worldTransform(f.objectId);
    const float size = std::max(0.01f, 0.25f * wt.mapVector(obj->aabbMax() - obj->aabbMin()).length());
    auto line = [&](const QVector3D& a, const QVector3D& b){
        QPointF sa, sb;
        if(toScreen(a, &sa) && toScreen(b, &sb)) p.drawLine(sa, sb);
    };
    p.setPen(QPen(color, 2.5));
    p.setBrush(Qt::NoBrush);
    if(f.kind == K::Point){
        QPointF s;
        if(!toScreen(w.point, &s)) return;
        p.setPen(QPen(Theme::colors().viewport, 1.5));
        p.setBrush(color);
        p.drawEllipse(s, 5, 5);
        return;
    }
    QVector3D u = QVector3D::crossProduct(w.direction, QVector3D(0, 1, 0));
    if(u.length() < 1e-3f) u = QVector3D::crossProduct(w.direction, QVector3D(1, 0, 0));
    u.normalize();
    const QVector3D v = QVector3D::crossProduct(w.direction, u).normalized();
    if(f.kind == K::Plane){
        const float h = size * 0.5f;
        const QVector3D c[4] = {w.point + (u + v) * h, w.point + (u - v) * h, w.point - (u + v) * h, w.point + (v - u) * h};
        for(int i = 0; i < 4; ++i) line(c[i], c[(i + 1) % 4]);
        line(w.point, w.point + w.direction * size * 0.6f);   // the normal
    } else if(f.kind == K::Axis){
        p.setPen(QPen(color, 2.5, Qt::DashDotLine));
        line(w.point - w.direction * size, w.point + w.direction * size);
    } else {
        const float r = f.radius > 0 ? f.radius : size * 0.3f;
        QVector3D prev = w.point + u * r;
        for(int i = 1; i <= 32; ++i){
            const float a = float(i) / 32 * 2 * float(M_PI);
            const QVector3D q = w.point + (u * std::cos(a) + v * std::sin(a)) * r;
            line(prev, q);
            prev = q;
        }
        p.setPen(QPen(color, 1.5, Qt::DashDotLine));
        line(w.point - w.direction * size, w.point + w.direction * size);
    }
}

void Viewport::drawAssemble(QPainter& p){
    if(!m_assembleMode) return;
    const Theme::Colors& c = Theme::colors();
    p.setRenderHint(QPainter::Antialiasing);
    QFont f = font(); f.setPointSize(9); f.setBold(true);
    p.setFont(f);

    // The mate being edited or selected in the list
    const int mateId = m_assemblePanel->editedMate() >= 0 ? m_assemblePanel->editedMate()
                                                          : m_assemblePanel->selectedMate();
    if(const Mate* m = m_scene->getMate(mateId)){
        drawMateGlyph(p, m->fixed, c.accent);
        drawMateGlyph(p, m->moving, c.warning);
    }

    auto outline = [&](const MatePick& mp, const QColor& col){
        if(!m_scene->getObject(mp.objectId)) return;
        const QMatrix4x4 wt = m_scene->worldTransform(mp.objectId);
        p.setPen(QPen(col, 2.5));
        for(size_t i = 0; i + 1 < mp.outline.size(); i += 2){
            QPointF a, b;
            if(toScreen(wt.map(mp.outline[i]), &a) && toScreen(wt.map(mp.outline[i + 1]), &b)) p.drawLine(a, b);
        }
        if(mp.kind == MateFeature::Kind::Point || mp.kind == MateFeature::Kind::Circle){
            QPointF s;
            if(toScreen(mp.point, &s)){
                p.setPen(QPen(c.viewport, 1.5));
                p.setBrush(col);
                p.drawEllipse(s, 4.5, 4.5);
                p.setBrush(Qt::NoBrush);
            }
        }
    };
    if(m_mateFirst.valid) outline(m_mateFirst, c.warning);
    if(m_mateHover.valid){
        const bool samePart = m_mateFirst.valid && m_mateHover.objectId == m_mateFirst.objectId;
        outline(m_mateHover, samePart ? c.muted : c.info);
        QPointF s;
        if(toScreen(m_mateHover.point, &s)){
            QFont small = font(); small.setPointSize(8);
            p.setFont(small);
            const SceneObject* o = m_scene->getObject(m_mateHover.objectId);
            drawPill(p, s + QPointF(12, 8), samePart ? "same part: pick another one"
                                                     : QString("%1 · %2").arg(o ? o->name : QString(), m_mateHover.what));
            p.setFont(f);
        }
    }
}

void Viewport::updateAxisCandidate(int x, int y){
    AxisCandidate found;
    const bool overGizmo = m_scene->selectedObjectId() == m_axisPickId && hitTestGizmoAxis(x, y) != -1;
    const float aspect = float(m_vpW) / float(std::max(m_vpH, 1));
    QVector3D orig, dir;
    m_camera.ray((2.0f * x) / m_vpW - 1.0f, 1.0f - (2.0f * y) / m_vpH, aspect, &orig, &dir);

    // Nearest triangle under the cursor over every part
    float bestT = 0;
    const int hitTri = overGizmo ? -1 : featureTriangleAt(orig, dir, &found.objectId, &bestT);

    if(found.objectId != -1){
        const SceneObject* obj = m_scene->getObject(found.objectId);
        const MeshFeatures* features = featuresFor(*obj);
        const QMatrix4x4 wt = m_scene->worldTransform(found.objectId);

        // An edge of the hovered face near the cursor wins over the face itself
        const AxisFeature* pick = nullptr;
        float nearest = 8.0f;   // px
        for(const AxisFeature* edge : features->edgesAround(hitTri)){
            for(size_t i = 0; i + 1 < edge->outline.size(); i += 2){
                QPointF a, b;
                if(!toScreen(wt.map(edge->outline[i]), &a) || !toScreen(wt.map(edge->outline[i + 1]), &b)) continue;
                const float d = distanceToSegment(QPointF(x, y), a, b);
                if(d < nearest){ nearest = d; pick = edge; }
            }
        }
        const AxisFeature& face = features->faceFeature(hitTri);
        if(!pick) pick = &face;   // possibly kind None: outlined, but not pickable

        found.kind = pick->kind;
        found.origin = wt.map(pick->origin);
        // Circle and face normals transform as normals, edge and cylinder
        // directions as directions (they differ under non-uniform scale)
        const bool isNormal = pick->kind == AxisFeature::Kind::CircularEdge || pick->kind == AxisFeature::Kind::FlatFace;
        found.axis = (isNormal ? wt.inverted().transposed().mapVector(pick->axis) : wt.mapVector(pick->axis)).normalized();
        const QVector3D across = QVector3D::crossProduct(pick->axis, std::abs(pick->axis.x()) < 0.9f
                                                                         ? QVector3D(1, 0, 0) : QVector3D(0, 1, 0)).normalized();
        found.radius = pick->radius * wt.mapVector(across).length();
        // The sign is arbitrary: point towards the viewer, so positive
        // rotation turns counter-clockwise on screen (right-hand rule)
        const QVector3D toViewer = m_camera.projection() == Camera::Projection::Orthographic
                                       ? -m_camera.forward() : (m_camera.position() - found.origin).normalized();
        const float facing = QVector3D::dotProduct(found.axis, toViewer);
        if(facing < -1e-3f) found.axis = -found.axis;
        else if(std::abs(facing) <= 1e-3f){
            int major = 0;
            for(int i = 1; i < 3; ++i) if(std::abs(found.axis[i]) > std::abs(found.axis[major])) major = i;
            if(found.axis[major] < 0) found.axis = -found.axis;
        }
        found.outline.reserve(pick->outline.size());
        for(const QVector3D& p : pick->outline) found.outline.push_back(wt.map(p));
    }

    m_axisCandidate = std::move(found);
    update();
}


void Viewport::commitAxisCandidate(){
    const AxisCandidate c = m_axisCandidate;
    const SceneObject* child = m_scene->getObject(m_axisPickId);
    if(!child || c.objectId == -1 || c.kind == AxisFeature::Kind::None) return;

    QVector3D origin = c.origin, axis = c.axis;
    QMatrix4x4 parentWorld;
    if(child->parentId != -1) parentWorld = m_scene->worldTransform(child->parentId);
    // Geometry on the moving side is seen in its jogged pose: take the joint's
    // own motion back out so the axis lands where that geometry is at zero
    const Joint* joint = m_scene->jointForChild(m_axisPickId);
    if(joint && scenequery::inSubtree(*m_scene, c.objectId, m_axisPickId)){
        const QMatrix4x4 rest = parentWorld * SceneGraph::jointMotion(*joint).inverted() * parentWorld.inverted();
        origin = rest.map(origin);
        axis   = rest.mapVector(axis);
    }
    const QMatrix4x4 toParent = parentWorld.inverted();
    const int pickedId = m_axisPickId;
    cancelAxisPick();
    emit axisPicked(pickedId, toParent.mapVector(axis).normalized(), toParent.map(origin), true);
}

// Filled arrowhead with its point at `tip`, pointing along `direction`
static void drawArrowHead(QPainter& p, const QPointF& tip, QPointF direction, double size){
    const double len = std::hypot(direction.x(), direction.y());
    if(len < 1e-6) return;
    direction /= len;
    const QPointF side(-direction.y(), direction.x());
    const QPointF back = tip - direction * size;
    p.drawPolygon(QPolygonF{tip, back + side * (size * 0.45), back - side * (size * 0.45)});
}

// An arrow from `origin` along `axis` over a dashed line marking the axis;
// revolute joints also get a curled arrow around it showing which way
// positive rotation turns (right-hand rule: thumb along the arrow)
void Viewport::drawAxisArrow(QPainter& p, const QVector3D& origin, const QVector3D& axis, bool revolute,
                             const QColor& color){
    const float length = 90.0f * m_camera.worldPerPixel(origin, float(m_vpH));
    QPointF o, tip, lineFrom, lineTo;
    if(!toScreen(origin, &o) || !toScreen(origin + axis * length, &tip) ||
       !toScreen(origin - axis * (length * 0.8f), &lineFrom) || !toScreen(origin + axis * (length * 1.5f), &lineTo))
        return;

    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    QColor halo = Theme::colors().viewport;
    halo.setAlpha(200);
    p.setPen(QPen(color, 1.2, Qt::DashLine));
    p.drawLine(lineFrom, lineTo);
    for(const QPen& pen : {QPen(halo, 5.0, Qt::SolidLine, Qt::RoundCap), QPen(color, 2.4, Qt::SolidLine, Qt::RoundCap)}){
        p.setPen(pen);
        p.drawLine(o, tip);
    }
    p.setPen(Qt::NoPen);
    p.setBrush(color);
    drawArrowHead(p, tip, tip - o, 12);
    p.drawEllipse(o, 3.5, 3.5);

    if(revolute){
        const QVector3D u = QVector3D::crossProduct(axis, std::abs(axis.x()) < 0.9f ? QVector3D(1, 0, 0)
                                                                                    : QVector3D(0, 1, 0)).normalized();
        const QVector3D v = QVector3D::crossProduct(axis, u);   // u → v turns positively about axis
        const QVector3D center = origin + axis * (length * 0.6f);
        const float r = length * 0.3f;
        constexpr int kSteps = 40;
        constexpr float kSweep = 300.0f * float(M_PI) / 180.0f;
        QPolygonF arc;
        for(int i = 0; i <= kSteps; ++i){
            const float a = kSweep * i / kSteps;
            QPointF s;
            if(toScreen(center + (u * std::cos(a) + v * std::sin(a)) * r, &s)) arc << s;
        }
        if(arc.size() > 2){
            p.setBrush(Qt::NoBrush);
            for(const QPen& pen : {QPen(halo, 4.5, Qt::SolidLine, Qt::RoundCap), QPen(color, 2.0, Qt::SolidLine, Qt::RoundCap)}){
                p.setPen(pen);
                p.drawPolyline(arc);
            }
            p.setPen(Qt::NoPen);
            p.setBrush(color);
            drawArrowHead(p, arc.last(), arc.last() - arc[arc.size() - 3], 10);
        }
    }
    p.restore();
}

void Viewport::drawJointAxes(QPainter& p){
    const Theme::Colors& c = Theme::colors();

    // The selected part's joint, as it is now
    const int selId = m_scene->selectedObjectId();
    const SceneObject* sel = m_scene->getObject(selId);
    if(const Joint* joint = sel ? m_scene->jointForChild(selId) : nullptr){
        QMatrix4x4 parentWorld;
        if(sel->parentId != -1) parentWorld = m_scene->worldTransform(sel->parentId);
        const QVector3D origin = joint->type == JointType::Revolute
                                     ? parentWorld.map(joint->pivot) : m_scene->worldTransform(selId).map(QVector3D());
        const QVector3D axis = parentWorld.mapVector(joint->axis).normalized();
        if(!axis.isNull()){
            QColor col = c.warning;
            if(m_axisPickId != -1) col.setAlpha(110);   // the candidate is what matters now
            drawAxisArrow(p, origin, axis, joint->type == JointType::Revolute, col);
        }
    }

    if(m_axisPickId == -1 || m_axisCandidate.objectId == -1) return;
    const AxisCandidate& cand = m_axisCandidate;
    const bool usable = cand.kind != AxisFeature::Kind::None;

    // Outline the edge or face the axis comes from
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    QColor outline = usable ? c.info : c.muted;
    p.setPen(QPen(outline, usable ? 3.0 : 1.5, Qt::SolidLine, Qt::RoundCap));
    for(size_t i = 0; i + 1 < cand.outline.size(); i += 2){
        QPointF a, b;
        if(toScreen(cand.outline[i], &a) && toScreen(cand.outline[i + 1], &b)) p.drawLine(a, b);
    }
    p.restore();

    const Joint* joint = m_scene->jointForChild(m_axisPickId);
    if(usable) drawAxisArrow(p, cand.origin, cand.axis, !joint || joint->type == JointType::Revolute, c.info);

    QString label;
    const QString diameter = QString("Ø %1 mm").arg(double(cand.radius) * 2000.0, 0, 'f', 1);
    switch(cand.kind){
        case AxisFeature::Kind::CircularEdge:    label = "Circular edge · " + diameter + " · centre line";   break;
        case AxisFeature::Kind::CylindricalFace: label = "Cylindrical face · " + diameter + " · centre line"; break;
        case AxisFeature::Kind::StraightEdge:    label = "Straight edge · along the edge";                   break;
        case AxisFeature::Kind::FlatFace:        label = "Flat face · normal through its centre";            break;
        case AxisFeature::Kind::None:            label = "No axis on this face — try a round edge or cylinder"; break;
    }
    QFont f = font(); f.setPointSize(8); f.setBold(true);
    p.setFont(f);
    drawPill(p, QPointF(m_lastMouse) + QPointF(16, 14), label, usable ? c.info : QColor());
}
