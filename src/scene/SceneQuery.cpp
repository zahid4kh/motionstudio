#include "SceneQuery.h"
#include "SceneGraph.h"
#include "PrimitiveMesh.h"
#include <algorithm>
#include <cmath>
#include <functional>

namespace scenequery {

bool rayMeetsBox(const QVector3D& o, const QVector3D& d, const QVector3D& mn, const QVector3D& mx,
                 float* tEnter) {
    float tMin = 0.0f, tMax = std::numeric_limits<float>::max();
    for (int i = 0; i < 3; ++i) {
        if (std::abs(d[i]) < 1e-12f) { if (o[i] < mn[i] || o[i] > mx[i]) return false; continue; }
        float t1 = (mn[i] - o[i]) / d[i], t2 = (mx[i] - o[i]) / d[i];
        if (t1 > t2) std::swap(t1, t2);
        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
        if (tMin > tMax) return false;
    }
    if (tEnter) *tEnter = tMin;
    return true;
}

bool rayHitsSoup(const std::vector<float>& v, const QVector3D& o, const QVector3D& d,
                 float maxT, float* tOut) {
    bool hit = false;
    const size_t triCount = v.size() / 18;
    for (size_t i = 0; i < triCount; ++i) {
        const float* f = &v[i * 18];
        const QVector3D a(f[0], f[1], f[2]);
        const QVector3D e1 = QVector3D(f[6], f[7], f[8]) - a, e2 = QVector3D(f[12], f[13], f[14]) - a;
        const QVector3D p = QVector3D::crossProduct(d, e2);
        const float det = QVector3D::dotProduct(e1, p);
        if (std::abs(det) < 1e-20f) continue;
        const float inv = 1.0f / det;
        const QVector3D s = o - a;
        const float u = QVector3D::dotProduct(s, p) * inv;
        if (u < 0.0f || u > 1.0f) continue;
        const QVector3D q = QVector3D::crossProduct(s, e1);
        const float w = QVector3D::dotProduct(d, q) * inv;
        if (w < 0.0f || u + w > 1.0f) continue;
        const float t = QVector3D::dotProduct(e2, q) * inv;
        if (t > 0.0f && t < maxT) { maxT = t; hit = true; }
    }
    if (hit) *tOut = maxT;
    return hit;
}

namespace {
// Smallest root of a t² + b t + c in (lo, hi), else hi
double firstRoot(double a, double b, double c, double lo, double hi, const std::function<bool(double)>& valid) {
    if (std::abs(a) < 1e-30) return hi;
    const double disc = b * b - 4 * a * c;
    if (disc < 0) return hi;
    const double s = std::sqrt(disc);
    // Numerically stable pair of roots
    const double q = -0.5 * (b + (b >= 0 ? s : -s));
    double r[2] = {q / a, std::abs(q) > 1e-300 ? c / q : q / a};
    if (r[0] > r[1]) std::swap(r[0], r[1]);
    for (double t : r)
        if (t > lo && t < hi && valid(t)) return t;
    return hi;
}
} // namespace

bool rayHitsPrimitive(PrimitiveType type, const QVector3D& of, const QVector3D& df, float maxT, float* tOut) {
    const double ox = of.x(), oy = of.y(), oz = of.z(), dx = df.x(), dy = df.y(), dz = df.z();
    const double hi = maxT;
    double best = hi;
    auto y = [&](double t) { return oy + t * dy; };
    // Sphere of radius r centred on (0, cy, 0), hits accepted by `valid`
    auto sphere = [&](double cy, double r, const std::function<bool(double)>& valid) {
        const double py = oy - cy;
        best = std::min(best, firstRoot(dx * dx + dy * dy + dz * dz, 2 * (ox * dx + py * dy + oz * dz),
                                        ox * ox + py * py + oz * oz - r * r, 0, best, valid));
    };
    // Side of the infinite cylinder of radius r about Y, between y0 and y1
    auto side = [&](double r, double y0, double y1) {
        best = std::min(best, firstRoot(dx * dx + dz * dz, 2 * (ox * dx + oz * dz), ox * ox + oz * oz - r * r, 0, best,
                                        [&](double t) { return y(t) >= y0 && y(t) <= y1; }));
    };
    switch (type) {
        case PrimitiveType::Sphere:
            sphere(0, 0.5, [](double) { return true; });
            break;
        case PrimitiveType::Cylinder:
            side(0.5, -0.5, 0.5);
            if (std::abs(dy) > 1e-30)
                for (double cap : {-0.5, 0.5}) {
                    const double t = (cap - oy) / dy;
                    const double x = ox + t * dx, z = oz + t * dz;
                    if (t > 0 && t < best && x * x + z * z <= 0.25) best = t;
                }
            break;
        case PrimitiveType::Capsule:
            side(0.25, -0.25, 0.25);
            sphere(0.25, 0.25, [&](double t) { return y(t) >= 0.25; });
            sphere(-0.25, 0.25, [&](double t) { return y(t) <= -0.25; });
            break;
        default:
            return false;
    }
    if (best >= hi) return false;
    *tOut = float(best);
    return true;
}

// Box entry and triangle hits share the ray parameter (the local ray is the
// world ray mapped affinely), so a box entered beyond the best hit so far is
// skipped without testing its triangles.
RayHit rayCast(const SceneGraph& scene, const QVector3D& orig, const QVector3D& dir, float maxT,
               const std::function<bool(int)>& accept) {
    struct Candidate { int id; float tBox; QVector3D lo, ld; const std::vector<float>* soup; PrimitiveType type; };
    std::vector<Candidate> candidates;
    for (auto& [id, obj] : scene.allObjects()) {
        const std::vector<float>* soup = surfaceOf(obj);
        if (!soup || obj.parked || (accept && !accept(id))) continue;
        bool invertible = false;
        const QMatrix4x4 inv = scene.worldTransform(id).inverted(&invertible);
        if (!invertible) continue;
        const QVector3D lo = inv.map(orig), ld = inv.mapVector(dir);
        float tBox;
        if (rayMeetsBox(lo, ld, obj.aabbMin(), obj.aabbMax(), &tBox) && tBox < maxT)
            candidates.push_back({id, tBox, lo, ld, soup, obj.type});
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.tBox < b.tBox; });

    RayHit best;
    best.t = maxT;
    for (const Candidate& c : candidates) {
        if (c.tBox >= best.t) break;
        float t;
        const bool curved = c.type == PrimitiveType::Sphere || c.type == PrimitiveType::Cylinder ||
                            c.type == PrimitiveType::Capsule;
        const bool hit = curved ? rayHitsPrimitive(c.type, c.lo, c.ld, best.t, &t)
                                : rayHitsSoup(*c.soup, c.lo, c.ld, best.t, &t);
        if (hit) { best.t = t; best.objectId = c.id; }
    }
    if (!best.hit()) best.t = std::numeric_limits<float>::max();
    return best;
}

Obb worldBox(const SceneGraph& scene, int objectId) {
    Obb b;
    const SceneObject* obj = scene.getObject(objectId);
    if (!obj) return b;
    const QMatrix4x4 w = scene.worldTransform(objectId);
    const QVector3D mn = obj->aabbMin(), mx = obj->aabbMax();
    b.center = w.map((mn + mx) * 0.5f);
    const QVector3D he = (mx - mn) * 0.5f;
    for (int i = 0; i < 3; ++i) {
        QVector3D col = w.column(i).toVector3D();
        float len = col.length();
        b.axis[i] = len > 1e-9f ? col / len : QVector3D(i == 0, i == 1, i == 2);
        b.half[i] = he[i] * len;
    }
    return b;
}

Obb boxInFrame(const QMatrix4x4& frame, const QVector3D& half) {
    Obb b;
    b.center = frame.column(3).toVector3D();
    for (int i = 0; i < 3; ++i) b.axis[i] = frame.column(i).toVector3D().normalized();
    b.half = half;
    return b;
}

bool overlap(const Obb& a, const Obb& b) {
    const QVector3D t = b.center - a.center;
    auto separated = [&](const QVector3D& L) {
        if (L.lengthSquared() < 1e-12f) return false;   // parallel edges: no new axis
        float ra = 0, rb = 0;
        for (int i = 0; i < 3; ++i) {
            ra += a.half[i] * std::abs(QVector3D::dotProduct(a.axis[i], L));
            rb += b.half[i] * std::abs(QVector3D::dotProduct(b.axis[i], L));
        }
        return std::abs(QVector3D::dotProduct(t, L)) > ra + rb;
    };
    for (int i = 0; i < 3; ++i) {
        if (separated(a.axis[i]) || separated(b.axis[i])) return false;
    }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (separated(QVector3D::crossProduct(a.axis[i], b.axis[j]))) return false;
    return true;
}

QMatrix4x4 stripScale(const QMatrix4x4& m) {
    QMatrix4x4 r = m;
    for (int col = 0; col < 3; col++) {
        float len = sqrtf(m(0,col)*m(0,col) + m(1,col)*m(1,col) + m(2,col)*m(2,col));
        if (len > 1e-6f) { r(0,col)/=len; r(1,col)/=len; r(2,col)/=len; }
    }
    return r;
}

bool inSubtree(const SceneGraph& scene, int id, int root) {
    for (const SceneObject* o = scene.getObject(id); o; o = scene.getObject(o->parentId))
        if (o->id == root) return true;
    return false;
}

} // namespace scenequery
