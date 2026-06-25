#include "Viewport.h"
#include <QMouseEvent>
#include <QWheelEvent>
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
out vec4 fragColor;
void main(){
    vec3 n = normalize(normal);
    vec3 l = normalize(-lightDir);
    float diff = max(dot(n,l), 0.0);
    vec3 v = normalize(viewPos - fragPos);
    vec3 h = normalize(l + v);
    float spec = pow(max(dot(n,h), 0.0), 48.0);
    vec3 col = objColor * (0.18 + 0.72*diff) + vec3(0.4)*spec;
    if(isSelected) col = mix(col, vec3(1.0,0.75,0.1), 0.35);
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

// ── Constructor / Destructor ─────────────────────────────────────────────────

Viewport::Viewport(SceneGraph* scene, QWidget* parent)
    : QOpenGLWidget(parent), m_scene(scene)
{
    setFocusPolicy(Qt::StrongFocus);
    connect(scene, &SceneGraph::objectAdded,   this, [this](int){ update(); });
    connect(scene, &SceneGraph::objectRemoved, this, [this](int){ update(); });
    connect(scene, &SceneGraph::objectChanged, this, [this](int){ update(); });
    connect(scene, &SceneGraph::selectionChanged, this, [this](int){ update(); });
    connect(scene, &SceneGraph::sceneCleared,  this, [this]{ update(); });
}

Viewport::~Viewport() {
    makeCurrent();
    auto del = [&](MeshData& m){
        if(m.vao) glDeleteVertexArrays(1, &m.vao);
        if(m.vbo) glDeleteBuffers(1, &m.vbo);
    };
    del(m_cube); del(m_cylinder); del(m_sphere); del(m_capsule);
    if(m_gridVao) glDeleteVertexArrays(1, &m_gridVao);
    if(m_gridVbo) glDeleteBuffers(1, &m_gridVbo);
    delete m_objShader;
    delete m_gridShader;
    doneCurrent();
}

// ── GL lifecycle ─────────────────────────────────────────────────────────────

void Viewport::initializeGL() {
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glClearColor(0.12f, 0.14f, 0.18f, 1.0f);
    buildShaders();
    buildMeshes();
    buildGrid();
}

void Viewport::resizeGL(int w, int h) {
    m_vpW = w; m_vpH = h;
    glViewport(0, 0, w, h);
}

void Viewport::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    float aspect = float(m_vpW) / float(std::max(m_vpH, 1));
    QMatrix4x4 view = m_camera.viewMatrix();
    QMatrix4x4 proj = m_camera.projectionMatrix(aspect);

    // Grid
    m_gridShader->bind();
    m_gridShader->setUniformValue("view", view);
    m_gridShader->setUniformValue("proj", proj);
    m_gridShader->setUniformValue("lineColor", QVector3D(0.28f, 0.30f, 0.35f));
    glBindVertexArray(m_gridVao);
    glDrawArrays(GL_LINES, 0, m_gridLines);
    glBindVertexArray(0);
    m_gridShader->release();

    // Objects
    drawScene();
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
}

void Viewport::uploadMesh(MeshData& m, const std::vector<float>& v) {
    glGenVertexArrays(1, &m.vao);
    glGenBuffers(1, &m.vbo);
    glBindVertexArray(m.vao);
    glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
    glBufferData(GL_ARRAY_BUFFER, v.size() * sizeof(float), v.data(), GL_STATIC_DRAW);
    // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // normal
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
}

// ── Mesh generators ──────────────────────────────────────────────────────────

MeshData Viewport::makeCube() {
    // 6 faces × 2 triangles × 3 vertices × 6 floats
    static const float v[] = {
        // pos             // normal
        // +Z
        -0.5f,-0.5f, 0.5f, 0,0,1,  0.5f,-0.5f, 0.5f, 0,0,1,  0.5f, 0.5f, 0.5f, 0,0,1,
        -0.5f,-0.5f, 0.5f, 0,0,1,  0.5f, 0.5f, 0.5f, 0,0,1, -0.5f, 0.5f, 0.5f, 0,0,1,
        // -Z
        -0.5f,-0.5f,-0.5f, 0,0,-1,  0.5f, 0.5f,-0.5f, 0,0,-1,  0.5f,-0.5f,-0.5f, 0,0,-1,
        -0.5f,-0.5f,-0.5f, 0,0,-1, -0.5f, 0.5f,-0.5f, 0,0,-1,  0.5f, 0.5f,-0.5f, 0,0,-1,
        // +X
         0.5f,-0.5f,-0.5f, 1,0,0,  0.5f, 0.5f,-0.5f, 1,0,0,  0.5f, 0.5f, 0.5f, 1,0,0,
         0.5f,-0.5f,-0.5f, 1,0,0,  0.5f, 0.5f, 0.5f, 1,0,0,  0.5f,-0.5f, 0.5f, 1,0,0,
        // -X
        -0.5f,-0.5f,-0.5f,-1,0,0, -0.5f, 0.5f, 0.5f,-1,0,0, -0.5f, 0.5f,-0.5f,-1,0,0,
        -0.5f,-0.5f,-0.5f,-1,0,0, -0.5f,-0.5f, 0.5f,-1,0,0, -0.5f, 0.5f, 0.5f,-1,0,0,
        // +Y
        -0.5f, 0.5f,-0.5f, 0,1,0, -0.5f, 0.5f, 0.5f, 0,1,0,  0.5f, 0.5f, 0.5f, 0,1,0,
        -0.5f, 0.5f,-0.5f, 0,1,0,  0.5f, 0.5f, 0.5f, 0,1,0,  0.5f, 0.5f,-0.5f, 0,1,0,
        // -Y
        -0.5f,-0.5f,-0.5f, 0,-1,0,  0.5f,-0.5f, 0.5f, 0,-1,0, -0.5f,-0.5f, 0.5f, 0,-1,0,
        -0.5f,-0.5f,-0.5f, 0,-1,0,  0.5f,-0.5f,-0.5f, 0,-1,0,  0.5f,-0.5f, 0.5f, 0,-1,0,
    };
    MeshData m;
    uploadMesh(m, std::vector<float>(std::begin(v), std::end(v)));
    return m;
}

MeshData Viewport::makeCylinder(int segs) {
    std::vector<float> v;
    v.reserve(segs * 12 * 6);
    const float r = 0.5f, h = 0.5f;

    auto push = [&](float x,float y,float z, float nx,float ny,float nz){
        v.insert(v.end(), {x,y,z,nx,ny,nz});
    };

    for(int i = 0; i < segs; i++){
        float a0 = float(i)   / segs * 2*M_PI;
        float a1 = float(i+1) / segs * 2*M_PI;
        float x0=r*cosf(a0), z0=r*sinf(a0);
        float x1=r*cosf(a1), z1=r*sinf(a1);
        float nx0=cosf(a0), nz0=sinf(a0);
        float nx1=cosf(a1), nz1=sinf(a1);
        // Side quad
        push(x0,-h,z0, nx0,0,nz0); push(x1,-h,z1, nx1,0,nz1); push(x1, h,z1, nx1,0,nz1);
        push(x0,-h,z0, nx0,0,nz0); push(x1, h,z1, nx1,0,nz1); push(x0, h,z0, nx0,0,nz0);
        // Top cap
        push(0,h,0, 0,1,0); push(r*cosf(a0),h,r*sinf(a0), 0,1,0); push(r*cosf(a1),h,r*sinf(a1), 0,1,0);
        // Bottom cap
        push(0,-h,0, 0,-1,0); push(r*cosf(a1),-h,r*sinf(a1), 0,-1,0); push(r*cosf(a0),-h,r*sinf(a0), 0,-1,0);
    }

    MeshData m;
    uploadMesh(m, v);
    return m;
}

MeshData Viewport::makeSphere(int rings, int sectors) {
    std::vector<float> v;
    const float r = 0.5f;

    auto push = [&](float phi, float theta){
        float nx = sinf(phi)*cosf(theta);
        float ny = cosf(phi);
        float nz = sinf(phi)*sinf(theta);
        v.insert(v.end(), {r*nx, r*ny, r*nz, nx, ny, nz});
    };

    for(int i = 0; i < rings; i++){
        float p0 = float(i)   / rings * M_PI;
        float p1 = float(i+1) / rings * M_PI;
        for(int j = 0; j < sectors; j++){
            float t0 = float(j)   / sectors * 2*M_PI;
            float t1 = float(j+1) / sectors * 2*M_PI;
            push(p0,t0); push(p1,t0); push(p1,t1);
            push(p0,t0); push(p1,t1); push(p0,t1);
        }
    }
    MeshData m;
    uploadMesh(m, v);
    return m;
}

MeshData Viewport::makeCapsule(int rings, int sectors) {
    std::vector<float> v;
    const float r = 0.25f, halfCyl = 0.25f;

    auto pushSide = [&](float phi, float theta, float yOff, bool flipY){
        float nx = cosf(phi)*cosf(theta);
        float ny = flipY ? -sinf(phi) : sinf(phi);
        float nz = cosf(phi)*sinf(theta);
        v.insert(v.end(), {r*nx, r*ny + yOff, r*nz, nx, ny, nz});
    };

    // Cylinder section
    for(int j = 0; j < sectors; j++){
        float a0 = float(j)   / sectors * 2*M_PI;
        float a1 = float(j+1) / sectors * 2*M_PI;
        float x0=r*cosf(a0),z0=r*sinf(a0), nx0=cosf(a0),nz0=sinf(a0);
        float x1=r*cosf(a1),z1=r*sinf(a1), nx1=cosf(a1),nz1=sinf(a1);
        v.insert(v.end(), {x0,-halfCyl,z0, nx0,0,nz0});
        v.insert(v.end(), {x1,-halfCyl,z1, nx1,0,nz1});
        v.insert(v.end(), {x1, halfCyl,z1, nx1,0,nz1});
        v.insert(v.end(), {x0,-halfCyl,z0, nx0,0,nz0});
        v.insert(v.end(), {x1, halfCyl,z1, nx1,0,nz1});
        v.insert(v.end(), {x0, halfCyl,z0, nx0,0,nz0});
    }

    // Top hemisphere
    for(int i = 0; i < rings; i++){
        float p0 = float(i)   / rings * M_PI/2;
        float p1 = float(i+1) / rings * M_PI/2;
        for(int j = 0; j < sectors; j++){
            float t0 = float(j)   / sectors * 2*M_PI;
            float t1 = float(j+1) / sectors * 2*M_PI;
            pushSide(p0,t0, halfCyl,false); pushSide(p1,t0, halfCyl,false); pushSide(p1,t1, halfCyl,false);
            pushSide(p0,t0, halfCyl,false); pushSide(p1,t1, halfCyl,false); pushSide(p0,t1, halfCyl,false);
        }
    }

    // Bottom hemisphere (flipped)
    for(int i = 0; i < rings; i++){
        float p0 = float(i)   / rings * M_PI/2;
        float p1 = float(i+1) / rings * M_PI/2;
        for(int j = 0; j < sectors; j++){
            float t0 = float(j)   / sectors * 2*M_PI;
            float t1 = float(j+1) / sectors * 2*M_PI;
            pushSide(p0,t1,-halfCyl,true); pushSide(p1,t1,-halfCyl,true); pushSide(p1,t0,-halfCyl,true);
            pushSide(p0,t1,-halfCyl,true); pushSide(p1,t0,-halfCyl,true); pushSide(p0,t0,-halfCyl,true);
        }
    }

    MeshData m;
    uploadMesh(m, v);
    return m;
}

void Viewport::buildGrid() {
    std::vector<float> v;
    const int half = 20;
    for(int i = -half; i <= half; i++){
        float f = float(i);
        // Along Z
        v.insert(v.end(), {f, 0, float(-half),  f, 0, float(half)});
        // Along X
        v.insert(v.end(), {float(-half), 0, f,  float(half), 0, f});
    }
    glGenVertexArrays(1, &m_gridVao);
    glGenBuffers(1, &m_gridVbo);
    glBindVertexArray(m_gridVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_gridVbo);
    glBufferData(GL_ARRAY_BUFFER, v.size() * sizeof(float), v.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
    m_gridLines = int(v.size()) / 3;
}

// ── Rendering ────────────────────────────────────────────────────────────────

static QVector3D primitiveColor(PrimitiveType t) {
    switch(t){
        case PrimitiveType::Cube:     return {0.40f, 0.60f, 1.00f};
        case PrimitiveType::Cylinder: return {0.40f, 1.00f, 0.45f};
        case PrimitiveType::Sphere:   return {1.00f, 0.60f, 0.30f};
        case PrimitiveType::Capsule:  return {0.80f, 0.45f, 1.00f};
    }
    return {0.8f, 0.8f, 0.8f};
}

void Viewport::drawMesh(const MeshData& m) {
    glBindVertexArray(m.vao);
    glDrawArrays(GL_TRIANGLES, 0, m.count);
    glBindVertexArray(0);
}

void Viewport::drawScene() {
    float aspect = float(m_vpW) / float(std::max(m_vpH, 1));
    QMatrix4x4 view = m_camera.viewMatrix();
    QMatrix4x4 proj = m_camera.projectionMatrix(aspect);
    QVector3D  vpos = m_camera.position();

    QVector3D lightDir = QVector3D(-0.5f, -1.0f, -0.7f).normalized();

    m_objShader->bind();
    m_objShader->setUniformValue("view",     view);
    m_objShader->setUniformValue("proj",     proj);
    m_objShader->setUniformValue("lightDir", lightDir);
    m_objShader->setUniformValue("viewPos",  vpos);

    for(auto& [id, obj] : m_scene->allObjects()){
        QMatrix4x4 model = m_scene->worldTransform(id);
        QMatrix3x3 nm    = model.normalMatrix();

        m_objShader->setUniformValue("model",      model);
        m_objShader->setUniformValue("normMat",    nm);
        m_objShader->setUniformValue("objColor",   primitiveColor(obj.type));
        m_objShader->setUniformValue("isSelected", obj.selected);

        switch(obj.type){
            case PrimitiveType::Cube:     drawMesh(m_cube);     break;
            case PrimitiveType::Cylinder: drawMesh(m_cylinder); break;
            case PrimitiveType::Sphere:   drawMesh(m_sphere);   break;
            case PrimitiveType::Capsule:  drawMesh(m_capsule);  break;
        }
    }
    m_objShader->release();
}

// ── Selection via ray-AABB ───────────────────────────────────────────────────

bool Viewport::rayAABB(const QVector3D& orig, const QVector3D& dir,
                        const QVector3D& mn,  const QVector3D& mx, float& t) {
    float tMin = -std::numeric_limits<float>::max();
    float tMax =  std::numeric_limits<float>::max();
    for(int i = 0; i < 3; i++){
        float d  = (i==0)?dir.x():(i==1)?dir.y():dir.z();
        float o  = (i==0)?orig.x():(i==1)?orig.y():orig.z();
        float lo = (i==0)?mn.x():(i==1)?mn.y():mn.z();
        float hi = (i==0)?mx.x():(i==1)?mx.y():mx.z();
        if(std::abs(d) < 1e-8f){
            if(o < lo || o > hi) return false;
        } else {
            float t1 = (lo - o) / d;
            float t2 = (hi - o) / d;
            if(t1 > t2) std::swap(t1, t2);
            tMin = std::max(tMin, t1);
            tMax = std::min(tMax, t2);
            if(tMin > tMax) return false;
        }
    }
    t = tMin;
    return tMin >= 0;
}

void Viewport::tryPickObject(int x, int y) {
    float aspect = float(m_vpW) / float(std::max(m_vpH, 1));
    float ndcX   = (2.0f * x) / m_vpW - 1.0f;
    float ndcY   = 1.0f - (2.0f * y) / m_vpH;

    QVector3D orig = m_camera.position();
    QVector3D dir  = m_camera.rayFromNDC(ndcX, ndcY, aspect);

    int   bestId = -1;
    float bestT  = std::numeric_limits<float>::max();

    for(auto& [id, obj] : m_scene->allObjects()){
        QMatrix4x4 world = m_scene->worldTransform(id);
        // Half-extents in local space (approximate bounding box)
        QVector3D he(0.5f, 0.5f, 0.5f);
        // Transform ray to object local space
        QMatrix4x4 inv = world.inverted();
        QVector3D lo = inv.map(orig);
        // Direction transform (no translation)
        QVector3D ld = QVector3D(inv * QVector4D(dir, 0.0f));
        float t;
        if(rayAABB(lo, ld, -he, he, t) && t < bestT){
            bestT  = t;
            bestId = id;
        }
    }

    m_scene->selectObject(bestId);
}

// ── Mouse events ─────────────────────────────────────────────────────────────

void Viewport::mousePressEvent(QMouseEvent* e) {
    m_lastMouse = e->pos();
    m_dragBtn   = e->button();

    if(e->button() == Qt::LeftButton && !(e->modifiers() & Qt::AltModifier)){
        makeCurrent();
        tryPickObject(e->pos().x(), e->pos().y());
        doneCurrent();
    }
}

void Viewport::mouseMoveEvent(QMouseEvent* e) {
    QPoint delta = e->pos() - m_lastMouse;
    m_lastMouse  = e->pos();

    bool alt = e->modifiers() & Qt::AltModifier;

    if((m_dragBtn == Qt::LeftButton && alt) || m_dragBtn == Qt::MiddleButton){
        if(e->modifiers() & Qt::ShiftModifier)
            m_camera.pan(delta.x(), delta.y());
        else
            m_camera.orbit(delta.x(), delta.y());
        update();
    } else if(m_dragBtn == Qt::RightButton){
        m_camera.pan(delta.x(), delta.y());
        update();
    } else if(m_dragBtn == Qt::LeftButton && alt){
        m_camera.orbit(delta.x(), delta.y());
        update();
    }
}

void Viewport::mouseReleaseEvent(QMouseEvent*) {
    m_dragBtn = Qt::NoButton;
}

void Viewport::wheelEvent(QWheelEvent* e) {
    m_camera.zoom(e->angleDelta().y() / 120.0f);
    update();
}
