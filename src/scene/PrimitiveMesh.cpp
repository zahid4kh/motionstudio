#include "PrimitiveMesh.h"
#include <QVector3D>
#include <cmath>
#include <iterator>

namespace {

std::vector<float> makeCube() {
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
    return {std::begin(v), std::end(v)};
}

std::vector<float> makeCylinder(int segs = 20) {
    std::vector<float> v;
    const float r = 0.5f, h = 0.5f;
    auto push = [&](float x,float y,float z, float nx,float ny,float nz){
        v.insert(v.end(), {x,y,z,nx,ny,nz});
    };
    for(int i = 0; i < segs; i++){
        float a0 = float(i)   / segs * 2*float(M_PI);
        float a1 = float(i+1) / segs * 2*float(M_PI);
        float x0=r*cosf(a0),z0=r*sinf(a0), nx0=cosf(a0),nz0=sinf(a0);
        float x1=r*cosf(a1),z1=r*sinf(a1), nx1=cosf(a1),nz1=sinf(a1);
        push(x0,-h,z0, nx0,0,nz0); push(x1,-h,z1, nx1,0,nz1); push(x1, h,z1, nx1,0,nz1);
        push(x0,-h,z0, nx0,0,nz0); push(x1, h,z1, nx1,0,nz1); push(x0, h,z0, nx0,0,nz0);
        push(0,h,0, 0,1,0); push(r*cosf(a0),h,r*sinf(a0), 0,1,0); push(r*cosf(a1),h,r*sinf(a1), 0,1,0);
        push(0,-h,0, 0,-1,0); push(r*cosf(a1),-h,r*sinf(a1), 0,-1,0); push(r*cosf(a0),-h,r*sinf(a0), 0,-1,0);
    }
    return v;
}

std::vector<float> makeSphere(int rings = 16, int sectors = 24) {
    std::vector<float> v;
    const float r = 0.5f;
    auto push = [&](float phi, float theta){
        float nx=sinf(phi)*cosf(theta), ny=cosf(phi), nz=sinf(phi)*sinf(theta);
        v.insert(v.end(), {r*nx,r*ny,r*nz,nx,ny,nz});
    };
    for(int i=0;i<rings;i++){
        float p0=float(i)/rings*float(M_PI), p1=float(i+1)/rings*float(M_PI);
        for(int j=0;j<sectors;j++){
            float t0=float(j)/sectors*2*float(M_PI), t1=float(j+1)/sectors*2*float(M_PI);
            push(p0,t0); push(p1,t0); push(p1,t1);
            push(p0,t0); push(p1,t1); push(p0,t1);
        }
    }
    return v;
}

std::vector<float> makeCapsule(int rings = 8, int sectors = 20) {
    std::vector<float> v;
    const float r=0.25f, hcy=0.25f;
    auto pushS=[&](float phi,float theta,float yOff,bool flipY){
        float nx=cosf(phi)*cosf(theta), ny=flipY?-sinf(phi):sinf(phi), nz=cosf(phi)*sinf(theta);
        v.insert(v.end(), {r*nx,r*ny+yOff,r*nz,nx,ny,nz});
    };
    for(int j=0;j<sectors;j++){
        float a0=float(j)/sectors*2*float(M_PI), a1=float(j+1)/sectors*2*float(M_PI);
        float x0=r*cosf(a0),z0=r*sinf(a0),nx0=cosf(a0),nz0=sinf(a0);
        float x1=r*cosf(a1),z1=r*sinf(a1),nx1=cosf(a1),nz1=sinf(a1);
        v.insert(v.end(),{x0,-hcy,z0,nx0,0,nz0}); v.insert(v.end(),{x1,-hcy,z1,nx1,0,nz1}); v.insert(v.end(),{x1,hcy,z1,nx1,0,nz1});
        v.insert(v.end(),{x0,-hcy,z0,nx0,0,nz0}); v.insert(v.end(),{x1,hcy,z1,nx1,0,nz1}); v.insert(v.end(),{x0,hcy,z0,nx0,0,nz0});
    }
    for(int i=0;i<rings;i++){
        float p0=float(i)/rings*float(M_PI)/2, p1=float(i+1)/rings*float(M_PI)/2;
        for(int j=0;j<sectors;j++){
            float t0=float(j)/sectors*2*float(M_PI), t1=float(j+1)/sectors*2*float(M_PI);
            pushS(p0,t0,hcy,false); pushS(p1,t0,hcy,false); pushS(p1,t1,hcy,false);
            pushS(p0,t0,hcy,false); pushS(p1,t1,hcy,false); pushS(p0,t1,hcy,false);
            pushS(p0,t1,-hcy,true); pushS(p1,t1,-hcy,true); pushS(p1,t0,-hcy,true);
            pushS(p0,t1,-hcy,true); pushS(p1,t0,-hcy,true); pushS(p0,t0,-hcy,true);
        }
    }
    return v;
}

// One axis-aligned box (pos+normal) appended to a flat vertex array
void appendBox(std::vector<float>& v, QVector3D c, QVector3D hs) {
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

// Unit basket: 1×1×1 bounding box, 5-sided open-top container with
// 0.05 wall/floor thickness in unit space
std::vector<float> makeBasket() {
    std::vector<float> v;
    appendBox(v, {0.f,    -0.475f, 0.f},    {0.5f,  0.025f, 0.5f});   // floor
    appendBox(v, {-0.475f, 0.025f, 0.f},    {0.025f,0.475f, 0.5f});   // left  wall
    appendBox(v, { 0.475f, 0.025f, 0.f},    {0.025f,0.475f, 0.5f});   // right wall
    appendBox(v, {0.f,     0.025f, 0.475f}, {0.45f, 0.475f, 0.025f}); // front wall
    appendBox(v, {0.f,     0.025f,-0.475f}, {0.45f, 0.475f, 0.025f}); // back  wall
    return v;
}

} // namespace

const std::vector<float>& primitiveSoup(PrimitiveType type) {
    static const std::vector<float> soups[6] = {
        makeCube(), makeCylinder(), makeSphere(), makeCapsule(), makeBasket(), {},
    };
    return soups[int(type)];
}

const std::vector<float>* surfaceOf(const SceneObject& obj) {
    const std::vector<float>* soup = obj.type == PrimitiveType::ImportedMesh
                                         ? (obj.mesh ? &obj.mesh->interleaved : nullptr)
                                         : &primitiveSoup(obj.type);
    return soup && !soup->empty() ? soup : nullptr;
}
