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

// ── Constructor / Destructor ─────────────────────────────────────────────────

Viewport::Viewport(SceneGraph* scene, QWidget* parent)
    : QOpenGLWidget(parent), m_scene(scene)
{
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true); // needed for hover without button press
    connect(scene, &SceneGraph::objectAdded,      this, [this](int){ update(); });
    connect(scene, &SceneGraph::objectRemoved,    this, [this](int){ m_hoveredId = -1; update(); });
    connect(scene, &SceneGraph::objectChanged,    this, [this](int){ update(); });
    connect(scene, &SceneGraph::selectionChanged, this, [this](int){ update(); });
    connect(scene, &SceneGraph::sceneCleared,     this, [this]{ m_hoveredId = -1; update(); });
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
    if(m_axisVao) glDeleteVertexArrays(1, &m_axisVao);
    if(m_axisVbo) glDeleteBuffers(1, &m_axisVbo);
    delete m_objShader;
    delete m_gridShader;
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
}

void Viewport::resizeGL(int w, int h) {
    m_vpW = w; m_vpH = h;
    glViewport(0, 0, w, h);
}

void Viewport::paintGL() {
    QPainter p(this);
    p.setRenderHint(QPainter::TextAntialiasing);

    p.beginNativePainting();

    // Explicitly reset state — QPainter leaves depth writes disabled
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
    drawAxisGizmo();

    glViewport(0, 0, m_vpW, m_vpH);
    glDisable(GL_SCISSOR_TEST);

    p.endNativePainting();

    drawAxisLabels(p);
    drawKeyHints(p);
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
}

void Viewport::buildAxisGizmo() {
    // 3 axes × 2 vertices × 3 floats (position only, colour via uniform)
    static const float verts[] = {
        0,0,0,  1,0,0,   // X
        0,0,0,  0,1,0,   // Y
        0,0,0,  0,0,1,   // Z
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
    }
    return {0.6f,0.6f,0.6f};
}

void Viewport::drawMesh(const MeshData& m){
    glBindVertexArray(m.vao);
    glDrawArrays(GL_TRIANGLES,0,m.count);
    glBindVertexArray(0);
}

void Viewport::drawGrid(){
    float aspect=float(m_vpW)/float(std::max(m_vpH,1));
    m_gridShader->bind();
    m_gridShader->setUniformValue("view", m_camera.viewMatrix());
    m_gridShader->setUniformValue("proj", m_camera.projectionMatrix(aspect));
    m_gridShader->setUniformValue("lineColor", QVector3D(0.65f,0.68f,0.74f));
    glBindVertexArray(m_gridVao);
    glDrawArrays(GL_LINES,0,m_gridLines);
    glBindVertexArray(0);
    m_gridShader->release();
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
        }
    }
    m_objShader->release();
}

// ── Axis gizmo ────────────────────────────────────────────────────────────────

void Viewport::drawAxisGizmo(){
    int sz=kGizmoSize, mg=kGizmoMargin;

    // Scissor-limited depth clear so the main scene depth is preserved
    glEnable(GL_SCISSOR_TEST);
    glScissor(mg, mg, sz, sz);
    glViewport(mg, mg, sz, sz);
    glClear(GL_DEPTH_BUFFER_BIT);

    // Rotation-only view (no translation, just extract camera rotation)
    m_gizmoView = m_camera.viewMatrix();
    m_gizmoView.setColumn(3, {0, 0, -2.5f, 1});

    m_gizmoProj.setToIdentity();
    m_gizmoProj.ortho(-1.4f, 1.4f, -1.4f, 1.4f, 0.1f, 10.0f);

    m_gridShader->bind();
    m_gridShader->setUniformValue("view", m_gizmoView);
    m_gridShader->setUniformValue("proj", m_gizmoProj);

    glBindVertexArray(m_axisVao);
    glLineWidth(2.5f);

    m_gridShader->setUniformValue("lineColor", QVector3D(0.92f, 0.18f, 0.18f)); // X red
    glDrawArrays(GL_LINES, 0, 2);
    m_gridShader->setUniformValue("lineColor", QVector3D(0.15f, 0.78f, 0.20f)); // Y green
    glDrawArrays(GL_LINES, 2, 2);
    m_gridShader->setUniformValue("lineColor", QVector3D(0.18f, 0.40f, 0.95f)); // Z blue
    glDrawArrays(GL_LINES, 4, 2);

    glBindVertexArray(0);
    glLineWidth(1.0f);
    m_gridShader->release();

    glDisable(GL_SCISSOR_TEST);
}

void Viewport::drawAxisLabels(QPainter& p){
    int sz=kGizmoSize, mg=kGizmoMargin;

    // Project a world-space point through the gizmo VP to Qt screen coords
    auto project=[&](QVector3D wp) -> QPointF {
        QVector4D clip = m_gizmoProj * m_gizmoView * QVector4D(wp, 1.0f);
        float ndcX = clip.x() / clip.w();
        float ndcY = clip.y() / clip.w();
        float glX  = mg + (ndcX + 1.0f) * 0.5f * sz;
        float glY  = mg + (ndcY + 1.0f) * 0.5f * sz;  // y-up in GL
        return { glX, float(m_vpH) - glY };             // flip to Qt y-down
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
        // shadow
        p.setPen(QColor(0,0,0,60));
        p.drawText(pos + QPointF(3,3), ax.label);
        // label
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
        QMatrix4x4 inv=scene->worldTransform(id).inverted();
        QVector3D lo=inv.map(orig), ld(inv*QVector4D(dir,0.0f));
        float t; QVector3D he(0.5f,0.5f,0.5f);
        if(true){ // inline rayAABB
            float tMn=-1e9f,tMx=1e9f;
            for(int i=0;i<3;i++){
                float d=(i==0)?ld.x():(i==1)?ld.y():ld.z();
                float o2=(i==0)?lo.x():(i==1)?lo.y():lo.z();
                float mn2=(i==0)?-he.x():(i==1)?-he.y():-he.z();
                float mx2=(i==0)?he.x():(i==1)?he.y():he.z();
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
        makeCurrent();
        tryPickObject(e->pos().x(), e->pos().y());
        doneCurrent();
    }
}

void Viewport::mouseMoveEvent(QMouseEvent* e){
    QPoint delta=e->pos()-m_lastMouse;
    m_lastMouse=e->pos();

    if(m_dragBtn==Qt::NoButton){
        updateHover(e->pos().x(), e->pos().y());
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

void Viewport::mouseReleaseEvent(QMouseEvent*){
    m_dragBtn=Qt::NoButton;
}

void Viewport::wheelEvent(QWheelEvent* e){
    m_camera.zoom(e->angleDelta().y()/120.0f);
    update();
}
