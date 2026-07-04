#include "Viewport.h"
#include <QMouseEvent>
#include <QWheelEvent>
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
uniform vec3  lightDir;
uniform vec3  viewPos;
uniform bool  isSelected;
uniform bool  isHovered;
out vec4 fragColor;
void main(){
    vec3 n = normalize(normal);
    vec3 l = normalize(-lightDir);
    float diff = max(dot(n,l), 0.0);
    vec3 v = normalize(viewPos - fragPos);
    vec3 h = normalize(l + v);
    float spec = pow(max(dot(n,h), 0.0), 48.0);
    vec3 col = objColor * (0.18 + 0.72*diff) + vec3(0.4)*spec;
    if(isHovered)  col = mix(col, vec3(0.45, 0.72, 1.0), 0.28);
    if(isSelected) col = mix(col, vec3(1.0, 0.75, 0.1),  0.40);
    fragColor = vec4(col, 1.0);
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

// Strip non-uniform scale from a world transform, preserving position + rotation
static QMatrix4x4 stripScale(const QMatrix4x4& m) {
    QMatrix4x4 r = m;
    for (int col = 0; col < 3; col++) {
        float len = sqrtf(m(0,col)*m(0,col) + m(1,col)*m(1,col) + m(2,col)*m(2,col));
        if (len > 1e-6f) { r(0,col)/=len; r(1,col)/=len; r(2,col)/=len; }
    }
    return r;
}

// ── Constructor / Destructor ─────────────────────────────────────────────────

static const char* kOverlayBtnStyle =
    "QPushButton { background:rgba(240,244,255,210); color:#334; "
    "  border:1px solid #8899cc; border-radius:3px; "
    "  padding:2px 6px; font-size:10px; font-weight:bold; }"
    "QPushButton:hover   { background:rgba(180,200,255,230); }"
    "QPushButton:pressed { background:rgba(150,175,240,255); }"
    "QPushButton[active='true'] { background:rgba(55,115,210,200); color:#fff; border-color:#2255bb; }";

Viewport::Viewport(SceneGraph* scene, QWidget* parent)
    : QOpenGLWidget(parent), m_scene(scene)
{
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);

    connect(scene, &SceneGraph::objectAdded,      this, [this](int){ update(); });
    connect(scene, &SceneGraph::objectRemoved,    this, [this](int){ m_hoveredId = -1; update(); });
    connect(scene, &SceneGraph::objectChanged,    this, [this](int){ update(); });
    connect(scene, &SceneGraph::selectionChanged, this, [this](int){
        m_gizmoHoveredAxis = -1;
        m_gizmoDragAxis    = -1;
        update();
    });
    connect(scene, &SceneGraph::sceneCleared,     this, [this]{
        m_hoveredId = -1;
        releaseImportedMeshes();
        update();
    });
    connect(scene, &SceneGraph::rigidBodyAdded,      this, [this](int){ update(); });
    connect(scene, &SceneGraph::rigidBodyRemoved,   this, [this](int){ update(); });
    connect(scene, &SceneGraph::rigidBodyChanged,   this, [this](int){ update(); });
    connect(scene, &SceneGraph::conveyorBeltAdded,  this, [this](int){ update(); });
    connect(scene, &SceneGraph::conveyorBeltRemoved,this, [this](int){ update(); });
    connect(scene, &SceneGraph::conveyorBeltChanged,this, [this](int){ update(); });
    connect(scene, &SceneGraph::floorChanged,       this, [this](bool){ update(); });

    // View-snap overlay buttons
    auto mkBtn = [&](const QString& label) -> QPushButton* {
        auto* b = new QPushButton(label, this);
        b->setStyleSheet(kOverlayBtnStyle);
        b->setFixedSize(46, 22);
        b->raise();
        return b;
    };

    m_btnFront = mkBtn("Front");
    m_btnTop   = mkBtn("Top");
    m_btnSide  = mkBtn("Side");

    m_btnFloor = new QPushButton("Floor: ON", this);
    m_btnFloor->setStyleSheet(kOverlayBtnStyle);
    m_btnFloor->setFixedSize(70, 22);
    m_btnFloor->setProperty("active", true);
    m_btnFloor->raise();

    connect(m_btnFront, &QPushButton::clicked, this, [this]{
        m_camera.setView(0.0f, 0.0f); update();
    });
    connect(m_btnTop, &QPushButton::clicked, this, [this]{
        m_camera.setView(0.0f, 89.0f); update();
    });
    connect(m_btnSide, &QPushButton::clicked, this, [this]{
        m_camera.setView(90.0f, 0.0f); update();
    });
    connect(m_btnFloor, &QPushButton::clicked, this, [this]{
        bool now = !m_scene->floorEnabled();
        m_scene->setFloorEnabled(now);
        m_btnFloor->setText(now ? "Floor: ON" : "Floor: OFF");
        m_btnFloor->setProperty("active", now);
        m_btnFloor->style()->unpolish(m_btnFloor);
        m_btnFloor->style()->polish(m_btnFloor);
    });
}

Viewport::~Viewport() {
    makeCurrent();
    auto del = [&](MeshData& m){
        if(m.vao) glDeleteVertexArrays(1, &m.vao);
        if(m.vbo) glDeleteBuffers(1, &m.vbo);
    };
    del(m_cube); del(m_cylinder); del(m_sphere); del(m_capsule); del(m_basket);
    for (auto& [geo, m] : m_importedMeshes) {
        if (m.vao) glDeleteVertexArrays(1, &m.vao);
        if (m.vbo) glDeleteBuffers(1, &m.vbo);
    }
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
    glClearColor(0.88f, 0.90f, 0.94f, 1.0f);
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
    const int mg = 6, gap = 3;
    int x = mg;
    for (auto* btn : {m_btnFront, m_btnTop, m_btnSide}) {
        btn->move(x, mg);
        x += btn->width() + gap;
    }
    m_btnFloor->move(m_vpW - m_btnFloor->width() - mg, mg);
}

void Viewport::paintGL() {
    QPainter p(this);
    p.setRenderHint(QPainter::TextAntialiasing);

    p.beginNativePainting();

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glDisable(GL_SCISSOR_TEST);
    glClearColor(0.88f, 0.90f, 0.94f, 1.0f);
    glViewport(0, 0, m_vpW, m_vpH);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    drawGrid();
    drawScene();
    drawPhysicsOverlays();
    drawTranslateGizmo();
    drawAxisGizmo();

    glViewport(0, 0, m_vpW, m_vpH);
    glDisable(GL_SCISSOR_TEST);

    p.endNativePainting();

    drawAxisLabels(p);
    drawKeyHints(p);
    drawKeyPoints(p);
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
    m_cube     = makeCube();
    m_cylinder = makeCylinder();
    m_sphere   = makeSphere();
    m_capsule  = makeCapsule();
    m_basket   = makeBasket();
}

// Lazy GPU upload for imported CAD meshes. Must be called with the GL
// context current (drawScene). Group nodes (null mesh) return nullptr.
const MeshData* Viewport::meshFor(const SceneObject& obj) {
    if (!obj.mesh || obj.mesh->interleaved.empty()) return nullptr;
    auto it = m_importedMeshes.find(obj.mesh.get());
    if (it != m_importedMeshes.end()) return &it->second;
    MeshData m;
    uploadMesh(m, obj.mesh->interleaved);
    return &(m_importedMeshes[obj.mesh.get()] = m);
}

void Viewport::releaseImportedMeshes() {
    if (m_importedMeshes.empty()) return;
    makeCurrent();
    for (auto& [geo, m] : m_importedMeshes) {
        if (m.vao) glDeleteVertexArrays(1, &m.vao);
        if (m.vbo) glDeleteBuffers(1, &m.vbo);
    }
    m_importedMeshes.clear();
    doneCurrent();
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

// ── Mesh generators ──────────────────────────────────────────────────────────

MeshData Viewport::makeCube() {
    static const float v[] = {
        -0.5f,-0.5f, 0.5f, 0,0,1,  0.5f,-0.5f, 0.5f, 0,0,1,  0.5f, 0.5f, 0.5f, 0,0,1,
        -0.5f,-0.5f, 0.5f, 0,0,1,  0.5f, 0.5f, 0.5f, 0,0,1, -0.5f, 0.5f, 0.5f, 0,0,1,
        -0.5f,-0.5f,-0.5f, 0,0,-1,  0.5f, 0.5f,-0.5f, 0,0,-1,  0.5f,-0.5f,-0.5f, 0,0,-1,
        -0.5f,-0.5f,-0.5f, 0,0,-1, -0.5f, 0.5f,-0.5f, 0,0,-1,  0.5f, 0.5f,-0.5f, 0,0,-1,
         0.5f,-0.5f,-0.5f, 1,0,0,  0.5f, 0.5f,-0.5f, 1,0,0,  0.5f, 0.5f, 0.5f, 1,0,0,
         0.5f,-0.5f,-0.5f, 1,0,0,  0.5f, 0.5f, 0.5f, 1,0,0,  0.5f,-0.5f, 0.5f, 1,0,0,
        -0.5f,-0.5f,-0.5f,-1,0,0, -0.5f, 0.5f, 0.5f,-1,0,0, -0.5f, 0.5f,-0.5f,-1,0,0,
        -0.5f,-0.5f,-0.5f,-1,0,0, -0.5f,-0.5f, 0.5f,-1,0,0, -0.5f, 0.5f, 0.5f,-1,0,0,
        -0.5f, 0.5f,-0.5f, 0,1,0, -0.5f, 0.5f, 0.5f, 0,1,0,  0.5f, 0.5f, 0.5f, 0,1,0,
        -0.5f, 0.5f,-0.5f, 0,1,0,  0.5f, 0.5f, 0.5f, 0,1,0,  0.5f, 0.5f,-0.5f, 0,1,0,
        -0.5f,-0.5f,-0.5f, 0,-1,0,  0.5f,-0.5f, 0.5f, 0,-1,0, -0.5f,-0.5f, 0.5f, 0,-1,0,
        -0.5f,-0.5f,-0.5f, 0,-1,0,  0.5f,-0.5f,-0.5f, 0,-1,0,  0.5f,-0.5f, 0.5f, 0,-1,0,
    };
    MeshData m;
    uploadMesh(m, std::vector<float>(std::begin(v), std::end(v)));
    return m;
}

MeshData Viewport::makeCylinder(int segs) {
    std::vector<float> v;
    const float r = 0.5f, h = 0.5f;
    auto push = [&](float x,float y,float z, float nx,float ny,float nz){
        v.insert(v.end(), {x,y,z,nx,ny,nz});
    };
    for(int i = 0; i < segs; i++){
        float a0 = float(i)   / segs * 2*M_PI;
        float a1 = float(i+1) / segs * 2*M_PI;
        float x0=r*cosf(a0),z0=r*sinf(a0), nx0=cosf(a0),nz0=sinf(a0);
        float x1=r*cosf(a1),z1=r*sinf(a1), nx1=cosf(a1),nz1=sinf(a1);
        push(x0,-h,z0, nx0,0,nz0); push(x1,-h,z1, nx1,0,nz1); push(x1, h,z1, nx1,0,nz1);
        push(x0,-h,z0, nx0,0,nz0); push(x1, h,z1, nx1,0,nz1); push(x0, h,z0, nx0,0,nz0);
        push(0,h,0, 0,1,0); push(r*cosf(a0),h,r*sinf(a0), 0,1,0); push(r*cosf(a1),h,r*sinf(a1), 0,1,0);
        push(0,-h,0, 0,-1,0); push(r*cosf(a1),-h,r*sinf(a1), 0,-1,0); push(r*cosf(a0),-h,r*sinf(a0), 0,-1,0);
    }
    MeshData m; uploadMesh(m, v); return m;
}

MeshData Viewport::makeSphere(int rings, int sectors) {
    std::vector<float> v;
    const float r = 0.5f;
    auto push = [&](float phi, float theta){
        float nx=sinf(phi)*cosf(theta), ny=cosf(phi), nz=sinf(phi)*sinf(theta);
        v.insert(v.end(), {r*nx,r*ny,r*nz,nx,ny,nz});
    };
    for(int i=0;i<rings;i++){
        float p0=float(i)/rings*M_PI, p1=float(i+1)/rings*M_PI;
        for(int j=0;j<sectors;j++){
            float t0=float(j)/sectors*2*M_PI, t1=float(j+1)/sectors*2*M_PI;
            push(p0,t0); push(p1,t0); push(p1,t1);
            push(p0,t0); push(p1,t1); push(p0,t1);
        }
    }
    MeshData m; uploadMesh(m, v); return m;
}

MeshData Viewport::makeCapsule(int rings, int sectors) {
    std::vector<float> v;
    const float r=0.25f, hcy=0.25f;
    auto pushS=[&](float phi,float theta,float yOff,bool flipY){
        float nx=cosf(phi)*cosf(theta), ny=flipY?-sinf(phi):sinf(phi), nz=cosf(phi)*sinf(theta);
        v.insert(v.end(), {r*nx,r*ny+yOff,r*nz,nx,ny,nz});
    };
    for(int j=0;j<sectors;j++){
        float a0=float(j)/sectors*2*M_PI, a1=float(j+1)/sectors*2*M_PI;
        float x0=r*cosf(a0),z0=r*sinf(a0),nx0=cosf(a0),nz0=sinf(a0);
        float x1=r*cosf(a1),z1=r*sinf(a1),nx1=cosf(a1),nz1=sinf(a1);
        v.insert(v.end(),{x0,-hcy,z0,nx0,0,nz0}); v.insert(v.end(),{x1,-hcy,z1,nx1,0,nz1}); v.insert(v.end(),{x1,hcy,z1,nx1,0,nz1});
        v.insert(v.end(),{x0,-hcy,z0,nx0,0,nz0}); v.insert(v.end(),{x1,hcy,z1,nx1,0,nz1}); v.insert(v.end(),{x0,hcy,z0,nx0,0,nz0});
    }
    for(int i=0;i<rings;i++){
        float p0=float(i)/rings*M_PI/2, p1=float(i+1)/rings*M_PI/2;
        for(int j=0;j<sectors;j++){
            float t0=float(j)/sectors*2*M_PI, t1=float(j+1)/sectors*2*M_PI;
            pushS(p0,t0,hcy,false); pushS(p1,t0,hcy,false); pushS(p1,t1,hcy,false);
            pushS(p0,t0,hcy,false); pushS(p1,t1,hcy,false); pushS(p0,t1,hcy,false);
            pushS(p0,t1,-hcy,true); pushS(p1,t1,-hcy,true); pushS(p1,t0,-hcy,true);
            pushS(p0,t1,-hcy,true); pushS(p1,t0,-hcy,true); pushS(p0,t0,-hcy,true);
        }
    }
    MeshData m; uploadMesh(m, v); return m;
}

// Helper: append one axis-aligned box (pos+normal) to a flat vertex array
static void appendBox(std::vector<float>& v, QVector3D c, QVector3D hs) {
    auto push = [&](float x,float y,float z, float nx,float ny,float nz){
        v.insert(v.end(), {x,y,z,nx,ny,nz});
    };
    float x0=c.x()-hs.x(), x1=c.x()+hs.x();
    float y0=c.y()-hs.y(), y1=c.y()+hs.y();
    float z0=c.z()-hs.z(), z1=c.z()+hs.z();
    push(x0,y0,z1,0,0,1); push(x1,y0,z1,0,0,1); push(x1,y1,z1,0,0,1);
    push(x0,y0,z1,0,0,1); push(x1,y1,z1,0,0,1); push(x0,y1,z1,0,0,1);
    push(x1,y0,z0,0,0,-1); push(x0,y0,z0,0,0,-1); push(x0,y1,z0,0,0,-1);
    push(x1,y0,z0,0,0,-1); push(x0,y1,z0,0,0,-1); push(x1,y1,z0,0,0,-1);
    push(x1,y0,z0,1,0,0); push(x1,y0,z1,1,0,0); push(x1,y1,z1,1,0,0);
    push(x1,y0,z0,1,0,0); push(x1,y1,z1,1,0,0); push(x1,y1,z0,1,0,0);
    push(x0,y0,z1,-1,0,0); push(x0,y0,z0,-1,0,0); push(x0,y1,z0,-1,0,0);
    push(x0,y0,z1,-1,0,0); push(x0,y1,z0,-1,0,0); push(x0,y1,z1,-1,0,0);
    push(x0,y1,z0,0,1,0); push(x0,y1,z1,0,1,0); push(x1,y1,z1,0,1,0);
    push(x0,y1,z0,0,1,0); push(x1,y1,z1,0,1,0); push(x1,y1,z0,0,1,0);
    push(x0,y0,z1,0,-1,0); push(x0,y0,z0,0,-1,0); push(x1,y0,z0,0,-1,0);
    push(x0,y0,z1,0,-1,0); push(x1,y0,z0,0,-1,0); push(x1,y0,z1,0,-1,0);
}

MeshData Viewport::makeBasket() {
    // Unit basket: 1×1×1 bounding box, 5-sided open-top container
    // t=0.05 wall/floor thickness in unit space
    std::vector<float> v;
    appendBox(v, {0.f,    -0.475f, 0.f},    {0.5f,  0.025f, 0.5f});   // floor
    appendBox(v, {-0.475f, 0.025f, 0.f},    {0.025f,0.475f, 0.5f});   // left  wall
    appendBox(v, { 0.475f, 0.025f, 0.f},    {0.025f,0.475f, 0.5f});   // right wall
    appendBox(v, {0.f,     0.025f, 0.475f}, {0.45f, 0.475f, 0.025f}); // front wall
    appendBox(v, {0.f,     0.025f,-0.475f}, {0.45f, 0.475f, 0.025f}); // back  wall
    MeshData m; uploadMesh(m, v); return m;
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
    // 9 points: center + 8 bounding-box corners.
    // Primitives span ±0.5; imported meshes use their real AABB.
    const QVector3D mn = obj.aabbMin(), mx = obj.aabbMax();
    const QVector3D c  = (mn + mx) * 0.5f;
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
    glDrawArrays(GL_TRIANGLES,0,m.count);
    glBindVertexArray(0);
}

void Viewport::drawGrid(){
    float aspect=float(m_vpW)/float(std::max(m_vpH,1));
    // Tint grid blue when floor is active
    QVector3D gridCol = m_scene->floorEnabled()
        ? QVector3D(0.52f, 0.60f, 0.80f)
        : QVector3D(0.65f, 0.68f, 0.74f);
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
        m_wireShader->setUniformValue("lineColor", QVector3D(0.30f, 0.50f, 0.88f));
        m_wireShader->setUniformValue("alpha",     0.09f);
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
    QVector3D lightDir=QVector3D(-0.5f,-1.0f,-0.7f).normalized();

    m_objShader->bind();
    m_objShader->setUniformValue("view",     view);
    m_objShader->setUniformValue("proj",     proj);
    m_objShader->setUniformValue("lightDir", lightDir);
    m_objShader->setUniformValue("viewPos",  vpos);

    for(auto& [id,obj] : m_scene->allObjects()){
        QMatrix4x4 model=m_scene->worldTransform(id);
        m_objShader->setUniformValue("model",      model);
        m_objShader->setUniformValue("normMat",    model.normalMatrix());
        m_objShader->setUniformValue("objColor",   primitiveColor(obj.type));
        m_objShader->setUniformValue("isSelected", obj.selected);
        m_objShader->setUniformValue("isHovered",  id == m_hoveredId && !obj.selected);
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

    glLineWidth(1.0f);
}

// ── Key point overlay ─────────────────────────────────────────────────────────

void Viewport::drawKeyPoints(QPainter& p) {
    if (m_hoveredId == -1) { m_nearKeyPointIdx = -1; return; }
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
        p.setPen(QPen(QColor(40, 60, 200, 220), near ? 2.0 : 1.5));
        p.setBrush(near ? QColor(255, 240, 30, 230) : QColor(255, 255, 255, 190));
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
        p.setPen(QColor(20, 20, 80, 200));
        QFont f; f.setPointSize(8); f.setBold(true); p.setFont(f);
        p.drawText(sp + QPointF(10, -4), lbl);
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
    QVector3D rayOrig = m_camera.position();
    QVector3D rayDir  = m_camera.rayFromNDC(ndcX, ndcY, aspect).normalized();

    static const float kAxData[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    QVector3D axisDir(kAxData[axis][0], kAxData[axis][1], kAxData[axis][2]);

    QVector3D toCam = (rayOrig - center).normalized();
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
        { {1,0,0}, {210,40,40},  "X" },
        { {0,1,0}, {20,170,55},  "Y" },
        { {0,0,1}, {35,90,235},  "Z" },
    };

    QFont f = p.font();
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

void Viewport::drawKeyHints(QPainter& p){
    static const QString hints[] = {
        "MMB  Orbit",
        "RMB  Pan",
        "Scroll  Zoom",
        "Click  Select",
    };
    QFont f;
    f.setPointSize(8);
    p.setFont(f);

    int x = m_vpW - 120;
    int y = m_vpH - 12 - 4*16;

    for(auto& h : hints){
        p.setPen(QColor(0,0,0,55));
        p.drawText(x+1, y+1, h);
        p.setPen(QColor(40,50,80,130));
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

static std::pair<int,float> pickNearest(SceneGraph* scene, const QVector3D& orig, const QVector3D& dir){
    int bestId=-1; float bestT=std::numeric_limits<float>::max();
    for(auto& [id,obj] : scene->allObjects()){
        // Assembly group nodes have no geometry — selectable via hierarchy only
        if(obj.type==PrimitiveType::ImportedMesh && !obj.mesh) continue;
        QMatrix4x4 inv=scene->worldTransform(id).inverted();
        QVector3D lo=inv.map(orig), ld(inv*QVector4D(dir,0.0f));
        float t; QVector3D bmn=obj.aabbMin(), bmx=obj.aabbMax();
        if(true){
            float tMn=-1e9f,tMx=1e9f;
            for(int i=0;i<3;i++){
                float d=(i==0)?ld.x():(i==1)?ld.y():ld.z();
                float o2=(i==0)?lo.x():(i==1)?lo.y():lo.z();
                float mn2=(i==0)?bmn.x():(i==1)?bmn.y():bmn.z();
                float mx2=(i==0)?bmx.x():(i==1)?bmx.y():bmx.z();
                if(std::abs(d)<1e-8f){ if(o2<mn2||o2>mx2){ tMn=1e9f; break; } }
                else { float t1=(mn2-o2)/d,t2=(mx2-o2)/d; if(t1>t2) std::swap(t1,t2); tMn=std::max(tMn,t1); tMx=std::min(tMx,t2); if(tMn>tMx){ tMn=1e9f; break; } }
            }
            t=tMn;
        }
        if(t<bestT&&t>=0){ bestT=t; bestId=id; }
    }
    return {bestId,bestT};
}

void Viewport::tryPickObject(int x, int y){
    float aspect=float(m_vpW)/float(std::max(m_vpH,1));
    float ndcX=(2.0f*x)/m_vpW-1.0f, ndcY=1.0f-(2.0f*y)/m_vpH;
    auto [id,t]=pickNearest(m_scene, m_camera.position(), m_camera.rayFromNDC(ndcX,ndcY,aspect));
    m_scene->selectObject(id);
}

void Viewport::updateHover(int x, int y){
    float aspect=float(m_vpW)/float(std::max(m_vpH,1));
    float ndcX=(2.0f*x)/m_vpW-1.0f, ndcY=1.0f-(2.0f*y)/m_vpH;
    auto [id,t]=pickNearest(m_scene, m_camera.position(), m_camera.rayFromNDC(ndcX,ndcY,aspect));
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
    if(e->button()==Qt::LeftButton && !(e->modifiers()&Qt::AltModifier)){
        // Gizmo drag takes highest priority
        int axisHit = hitTestGizmoAxis(e->pos().x(), e->pos().y());
        if (axisHit != -1) {
            int selId = m_scene->selectedObjectId();
            const SceneObject* obj = m_scene->getObject(selId);
            if (obj) {
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

            if (obj->parentId == -1) {
                obj->transform.position = m_gizmoDragStartPos + worldDelta;
            } else {
                QMatrix4x4 parentWorldInv = m_scene->worldTransform(obj->parentId).inverted();
                QVector3D newWorldPos = m_gizmoDragWorldCenter + worldDelta;
                obj->transform.position = parentWorldInv.map(newWorldPos);
            }
            emit m_scene->objectChanged(selId);
        }
        update();
        return;
    }

    if(m_dragBtn==Qt::NoButton){
        updateHover(pos.x(), pos.y());
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
            m_camera.pan(delta.x(), delta.y());
        else
            m_camera.orbit(delta.x(), delta.y());
        update();
    } else if(m_dragBtn==Qt::RightButton){
        m_camera.pan(delta.x(), delta.y());
        update();
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
    m_camera.zoom(e->angleDelta().y()/120.0f);
    update();
}
