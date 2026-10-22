#include "MeshFeatures.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace {
// Adjacent triangles bending more than this meet at an edge. BRepMesh keeps
// neighbouring normals on one curved face within 0.5 rad (≈ 29°).
const float kCreaseCos = std::cos(40.0f * float(M_PI) / 180.0f);
// A chain of edges turning more than this at a vertex is split there (box corners)
const float kCornerCos = std::cos(45.0f * float(M_PI) / 180.0f);
const float kFlatCos   = std::cos(2.0f * float(M_PI) / 180.0f);
const float kLineCos   = std::cos(1.5f * float(M_PI) / 180.0f);
// Cylinder normals must stay within this of perpendicular to the axis
const float kCylinderSin = std::sin(6.0f * float(M_PI) / 180.0f);
// Points may stray this fraction of the radius from a fitted circle
constexpr float kCircleTolerance = 0.02f;

struct PointKey {
    long long x, y, z;
    bool operator==(const PointKey& o) const { return x == o.x && y == o.y && z == o.z; }
};
struct PointKeyHash {
    size_t operator()(const PointKey& k) const {
        return size_t(k.x * 73856093LL) ^ size_t(k.y * 19349663LL) ^ size_t(k.z * 83492791LL);
    }
};

// Eigen-decomposition of a symmetric 3×3 matrix (Jacobi), sorted ascending
void eigenSymmetric(double a[3][3], double values[3], QVector3D vectors[3]) {
    double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int sweep = 0; sweep < 50; ++sweep) {
        const double off = std::abs(a[0][1]) + std::abs(a[0][2]) + std::abs(a[1][2]);
        if (off < 1e-15) break;
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                if (std::abs(a[p][q]) < 1e-18) continue;
                const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                const double t = (theta >= 0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0), s = t * c;
                for (int k = 0; k < 3; ++k) {   // a = Jᵀ a J
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; ++k) {
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 3; ++k) {
                    const double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }
    int order[3] = {0, 1, 2};
    std::sort(order, order + 3, [&](int i, int j) { return a[i][i] < a[j][j]; });
    for (int i = 0; i < 3; ++i) {
        const int k = order[i];
        values[i]  = a[k][k];
        vectors[i] = QVector3D(float(v[0][k]), float(v[1][k]), float(v[2][k])).normalized();
    }
}

// Two unit vectors completing `n` to an orthonormal basis
void basisAround(const QVector3D& n, QVector3D* u, QVector3D* v) {
    const QVector3D helper = std::abs(n.x()) < 0.9f ? QVector3D(1, 0, 0) : QVector3D(0, 1, 0);
    *u = QVector3D::crossProduct(n, helper).normalized();
    *v = QVector3D::crossProduct(n, *u);
}

// Least-squares circle through points projected onto the plane ⊥ `normal`
// (Kåsa fit). Returns false if the points don't lie on a circle.
bool fitCircle(const std::vector<QVector3D>& pts, const QVector3D& normal, QVector3D* center, float* radius) {
    if (pts.size() < 3) return false;
    QVector3D u, v;
    basisAround(normal, &u, &v);
    QVector3D mean;
    for (const QVector3D& p : pts) mean += p;
    mean /= float(pts.size());

    double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, sxz = 0, syz = 0, sz = 0;
    for (const QVector3D& p : pts) {
        const double x = QVector3D::dotProduct(p - mean, u), y = QVector3D::dotProduct(p - mean, v);
        const double z = x * x + y * y;
        sxx += x * x; sxy += x * y; syy += y * y; sx += x; sy += y;
        sxz += x * z; syz += y * z; sz += z;
    }
    const double n = double(pts.size());
    // [sxx sxy sx; sxy syy sy; sx sy n] · [2a 2b c] = [sxz syz sz]
    const double m[3][3] = {{sxx, sxy, sx}, {sxy, syy, sy}, {sx, sy, n}};
    const double r[3] = {sxz, syz, sz};
    auto det3 = [](const double k[3][3]) {
        return k[0][0] * (k[1][1] * k[2][2] - k[1][2] * k[2][1]) - k[0][1] * (k[1][0] * k[2][2] - k[1][2] * k[2][0]) +
               k[0][2] * (k[1][0] * k[2][1] - k[1][1] * k[2][0]);
    };
    const double d = det3(m);
    if (std::abs(d) < 1e-30) return false;
    double sol[3];
    for (int col = 0; col < 3; ++col) {
        double k[3][3];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) k[i][j] = (j == col) ? r[i] : m[i][j];
        sol[col] = det3(k) / d;
    }
    const double a = sol[0] / 2, b = sol[1] / 2;
    const double rr = sol[2] + a * a + b * b;
    if (rr <= 0) return false;
    const float rad = float(std::sqrt(rr));
    const QVector3D c = mean + u * float(a) + v * float(b);
    for (const QVector3D& p : pts) {
        QVector3D d3 = p - c;
        d3 -= normal * QVector3D::dotProduct(d3, normal);
        if (std::abs(d3.length() - rad) > kCircleTolerance * rad) return false;
    }
    *center = c;
    *radius = rad;
    return true;
}
} // namespace

MeshFeatures::MeshFeatures(const std::vector<float>& interleaved) {
    const int vertexCount = int(interleaved.size()) / 6;
    QVector3D mn(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    QVector3D mx = -mn;
    for (int i = 0; i < vertexCount; ++i) {
        const QVector3D p(interleaved[i * 6], interleaved[i * 6 + 1], interleaved[i * 6 + 2]);
        mn = QVector3D(std::min(mn.x(), p.x()), std::min(mn.y(), p.y()), std::min(mn.z(), p.z()));
        mx = QVector3D(std::max(mx.x(), p.x()), std::max(mx.y(), p.y()), std::max(mx.z(), p.z()));
    }
    const double cell = std::max(double((mx - mn).length()) * 1e-6, 1e-9);

    // Weld coincident vertices so neighbouring triangles share them
    std::unordered_map<PointKey, int, PointKeyHash> weld;
    auto pointIndex = [&](const QVector3D& p) {
        const PointKey key{std::llround(p.x() / cell), std::llround(p.y() / cell), std::llround(p.z() / cell)};
        auto [it, inserted] = weld.try_emplace(key, int(m_points.size()));
        if (inserted) m_points.push_back(p);
        return it->second;
    };
    for (int t = 0; t + 2 < vertexCount; t += 3) {
        Triangle tri;
        QVector3D p[3], stored;
        for (int k = 0; k < 3; ++k) {
            const float* f = &interleaved[size_t(t + k) * 6];
            p[k] = QVector3D(f[0], f[1], f[2]);
            stored += QVector3D(f[3], f[4], f[5]);
            tri.v[k] = pointIndex(p[k]);
        }
        if (tri.v[0] == tri.v[1] || tri.v[1] == tri.v[2] || tri.v[0] == tri.v[2]) continue;
        QVector3D n = QVector3D::crossProduct(p[1] - p[0], p[2] - p[0]);
        tri.area = n.length() * 0.5f;
        if (tri.area <= 0.0f) continue;
        n.normalize();
        // Trust the stored normals for which side is outside
        if (QVector3D::dotProduct(n, stored) < 0.0f) n = -n;
        tri.normal = n;
        m_tris.push_back(tri);
    }

    // Edges and the triangles that share them
    std::unordered_map<unsigned long long, int> edgeIndex;
    std::vector<std::pair<int, int>> edges;
    std::vector<std::vector<int>> edgeTris;
    for (int t = 0; t < int(m_tris.size()); ++t) {
        for (int k = 0; k < 3; ++k) {
            int a = m_tris[t].v[k], b = m_tris[t].v[(k + 1) % 3];
            if (a > b) std::swap(a, b);
            const unsigned long long key = (unsigned long long)(a) << 32 | unsigned(b);
            auto [it, inserted] = edgeIndex.try_emplace(key, int(edges.size()));
            if (inserted) { edges.push_back({a, b}); edgeTris.emplace_back(); }
            edgeTris[it->second].push_back(t);
        }
    }
    std::vector<bool> crease(edges.size());
    for (size_t e = 0; e < edges.size(); ++e) {
        const auto& ts = edgeTris[e];
        crease[e] = ts.size() != 2 ||
                    QVector3D::dotProduct(m_tris[ts[0]].normal, m_tris[ts[1]].normal) < kCreaseCos;
    }

    buildRegions(edgeTris, edges, crease);
    buildChains(edges, edgeTris, crease);
}

void MeshFeatures::buildRegions(const std::vector<std::vector<int>>& edgeTris,
                                const std::vector<std::pair<int, int>>& edges, const std::vector<bool>& crease) {
    // Triangle → its three edges, to walk across the smooth ones
    std::unordered_map<unsigned long long, int> edgeOf;
    for (size_t e = 0; e < edges.size(); ++e)
        edgeOf[(unsigned long long)(edges[e].first) << 32 | unsigned(edges[e].second)] = int(e);
    auto edgeBetween = [&](int a, int b) {
        if (a > b) std::swap(a, b);
        return edgeOf.at((unsigned long long)(a) << 32 | unsigned(b));
    };

    m_regionOf.assign(m_tris.size(), -1);
    std::vector<std::vector<int>> regions;
    for (int seed = 0; seed < int(m_tris.size()); ++seed) {
        if (m_regionOf[seed] != -1) continue;
        const int region = int(regions.size());
        regions.emplace_back();
        std::queue<int> open;
        open.push(seed);
        m_regionOf[seed] = region;
        while (!open.empty()) {
            const int t = open.front();
            open.pop();
            regions[region].push_back(t);
            for (int k = 0; k < 3; ++k) {
                const int e = edgeBetween(m_tris[t].v[k], m_tris[t].v[(k + 1) % 3]);
                if (crease[e]) continue;
                for (int other : edgeTris[e]) {
                    if (m_regionOf[other] != -1) continue;
                    m_regionOf[other] = region;
                    open.push(other);
                }
            }
        }
    }

    m_faces.reserve(regions.size());
    for (const auto& tris : regions) m_faces.push_back(classifyRegion(tris));
    m_faceEdges.assign(regions.size(), {});

    // Each face is outlined by the creases around it
    for (size_t e = 0; e < edges.size(); ++e) {
        if (!crease[e]) continue;
        int last = -1;
        for (int t : edgeTris[e]) {
            const int r = m_regionOf[t];
            if (r == last) continue;
            last = r;
            m_faces[r].outline.push_back(m_points[edges[e].first]);
            m_faces[r].outline.push_back(m_points[edges[e].second]);
        }
    }
}

void MeshFeatures::buildChains(const std::vector<std::pair<int, int>>& edges,
                               const std::vector<std::vector<int>>& edgeTris, const std::vector<bool>& crease) {
    std::vector<std::vector<int>> incident(m_points.size());
    for (size_t e = 0; e < edges.size(); ++e) {
        if (!crease[e]) continue;
        incident[edges[e].first].push_back(int(e));
        incident[edges[e].second].push_back(int(e));
    }
    auto otherEnd = [&](int e, int v) { return edges[e].first == v ? edges[e].second : edges[e].first; };

    std::vector<bool> used(edges.size(), false);
    // Walks crease edges from `start` along `first` until a vertex where the
    // chain branches or ends (or back at `start` for a loop)
    auto walk = [&](int start, int first, std::vector<int>* pts, std::vector<int>* chainEdges) {
        pts->assign(1, start);
        chainEdges->clear();
        int v = start, e = first;
        while (e != -1 && !used[e]) {
            used[e] = true;
            chainEdges->push_back(e);
            v = otherEnd(e, v);
            pts->push_back(v);
            if (incident[v].size() != 2 || v == start) break;
            e = incident[v][0] == e ? incident[v][1] : incident[v][0];
        }
    };

    auto addPiece = [&](const std::vector<int>& pts, const std::vector<int>& chainEdges, bool closed) {
        AxisFeature f = classifyChain(pts, closed);
        if (f.kind == AxisFeature::Kind::None) return;
        const int index = int(m_edges.size());
        m_edges.push_back(std::move(f));
        for (int e : chainEdges)
            for (int t : edgeTris[e]) {
                auto& list = m_faceEdges[m_regionOf[t]];
                if (std::find(list.begin(), list.end(), index) == list.end()) list.push_back(index);
            }
    };

    auto turnsSharply = [&](int prev, int at, int next) {
        const QVector3D a = (m_points[at] - m_points[prev]).normalized();
        const QVector3D b = (m_points[next] - m_points[at]).normalized();
        return QVector3D::dotProduct(a, b) < kCornerCos;
    };

    // Splits a chain at its corners; a loop without corners stays whole
    auto splitAndAdd = [&](std::vector<int> pts, std::vector<int> chainEdges) {
        const bool closed = pts.size() > 2 && pts.front() == pts.back();
        if (closed) {
            const int n = int(pts.size()) - 1;   // distinct points
            int corner = -1;
            for (int i = 0; i < n && corner == -1; ++i)
                if (turnsSharply(pts[(i + n - 1) % n], pts[i], pts[(i + 1) % n])) corner = i;
            if (corner == -1) {
                pts.pop_back();
                addPiece(pts, chainEdges, true);
                return;
            }
            // Start the loop at a corner and treat it as an open chain
            std::vector<int> p2, e2;
            for (int i = 0; i <= n; ++i) p2.push_back(pts[(corner + i) % n]);
            for (int i = 0; i < n; ++i) e2.push_back(chainEdges[(corner + i) % n]);
            pts = std::move(p2);
            chainEdges = std::move(e2);
        }
        size_t begin = 0;
        for (size_t i = 1; i + 1 < pts.size(); ++i) {
            if (!turnsSharply(pts[i - 1], pts[i], pts[i + 1])) continue;
            addPiece({pts.begin() + begin, pts.begin() + i + 1}, {chainEdges.begin() + begin, chainEdges.begin() + i}, false);
            begin = i;
        }
        addPiece({pts.begin() + begin, pts.end()}, {chainEdges.begin() + begin, chainEdges.end()}, false);
    };

    std::vector<int> pts, chainEdges;
    for (int v = 0; v < int(m_points.size()); ++v) {
        if (incident[v].empty() || incident[v].size() == 2) continue;
        for (int e : incident[v]) {
            if (used[e]) continue;
            walk(v, e, &pts, &chainEdges);
            splitAndAdd(pts, chainEdges);
        }
    }
    // What is left are loops through vertices with exactly two creases
    for (size_t e = 0; e < edges.size(); ++e) {
        if (!crease[e] || used[e]) continue;
        walk(edges[e].first, int(e), &pts, &chainEdges);
        splitAndAdd(pts, chainEdges);
    }
}

AxisFeature MeshFeatures::classifyRegion(const std::vector<int>& tris) const {
    AxisFeature f;
    float area = 0.0f;
    QVector3D meanNormal, centroid;
    for (int t : tris) {
        const Triangle& tri = m_tris[t];
        area += tri.area;
        meanNormal += tri.normal * tri.area;
        centroid += (m_points[tri.v[0]] + m_points[tri.v[1]] + m_points[tri.v[2]]) * (tri.area / 3.0f);
    }
    if (area <= 0.0f) return f;
    centroid /= area;
    const QVector3D n = meanNormal.normalized();

    if (std::all_of(tris.begin(), tris.end(),
                    [&](int t) { return QVector3D::dotProduct(m_tris[t].normal, n) > kFlatCos; })) {
        f.kind   = AxisFeature::Kind::FlatFace;
        f.origin = centroid;
        f.axis   = n;
        return f;
    }

    // Cylinder: every normal is perpendicular to one direction, the axis,
    // which is the least-spread direction of the normals.
    double m[3][3] = {};
    for (int t : tris) {
        const QVector3D& tn = m_tris[t].normal;
        const float w = m_tris[t].area / area;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) m[i][j] += double(w * tn[i] * tn[j]);
    }
    double values[3];
    QVector3D vectors[3];
    eigenSymmetric(m, values, vectors);
    const QVector3D axis = vectors[0];
    if (values[1] < 0.02) return f;   // normals barely spread: no curvature to fit
    for (int t : tris)
        if (std::abs(QVector3D::dotProduct(m_tris[t].normal, axis)) > kCylinderSin) return f;

    std::vector<int> ids;
    for (int t : tris) ids.insert(ids.end(), m_tris[t].v, m_tris[t].v + 3);
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    std::vector<QVector3D> pts;
    pts.reserve(ids.size());
    for (int id : ids) pts.push_back(m_points[id]);

    QVector3D center;
    float radius;
    if (!fitCircle(pts, axis, &center, &radius)) return f;
    float lo = std::numeric_limits<float>::max(), hi = -lo;
    for (const QVector3D& p : pts) {
        const float s = QVector3D::dotProduct(p - center, axis);
        lo = std::min(lo, s);
        hi = std::max(hi, s);
    }
    f.kind   = AxisFeature::Kind::CylindricalFace;
    f.origin = center + axis * ((lo + hi) * 0.5f);
    f.axis   = axis;
    f.radius = radius;
    return f;
}

AxisFeature MeshFeatures::classifyChain(const std::vector<int>& ids, bool closed) const {
    AxisFeature f;
    const int segments = closed ? int(ids.size()) : int(ids.size()) - 1;
    if (segments < 1) return f;
    for (int i = 0; i < segments; ++i) {
        f.outline.push_back(m_points[ids[i]]);
        f.outline.push_back(m_points[ids[(i + 1) % ids.size()]]);
    }

    if (!closed) {
        const QVector3D a = m_points[ids.front()], b = m_points[ids.back()];
        const QVector3D dir = (b - a).normalized();
        bool straight = !dir.isNull();
        for (int i = 0; straight && i < segments; ++i)
            straight = QVector3D::dotProduct((m_points[ids[i + 1]] - m_points[ids[i]]).normalized(), dir) > kLineCos;
        if (straight) {
            f.kind   = AxisFeature::Kind::StraightEdge;
            f.origin = (a + b) * 0.5f;
            f.axis   = dir;
            return f;
        }
    }

    // Circle or arc: planar, and every point at one distance from a centre
    if (segments < 3) { f.outline.clear(); return f; }
    std::vector<QVector3D> pts;
    QVector3D mean;
    for (int id : ids) { pts.push_back(m_points[id]); mean += m_points[id]; }
    mean /= float(pts.size());
    double cov[3][3] = {};
    float extent = 0.0f;
    for (const QVector3D& p : pts) {
        const QVector3D d = p - mean;
        extent = std::max(extent, d.length());
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) cov[i][j] += double(d[i] * d[j]);
    }
    double values[3];
    QVector3D vectors[3];
    eigenSymmetric(cov, values, vectors);
    const QVector3D normal = vectors[0];
    for (const QVector3D& p : pts)
        if (std::abs(QVector3D::dotProduct(p - mean, normal)) > 0.02f * extent) { f.outline.clear(); return f; }

    QVector3D center;
    float radius;
    if (!fitCircle(pts, normal, &center, &radius) || radius > 20.0f * extent) { f.outline.clear(); return f; }
    f.kind   = AxisFeature::Kind::CircularEdge;
    f.origin = center;
    f.axis   = normal;
    f.radius = radius;
    return f;
}

int MeshFeatures::intersect(const QVector3D& origin, const QVector3D& dir, float* tOut) const {
    int best = -1;
    float bestT = std::numeric_limits<float>::max();
    for (int i = 0; i < int(m_tris.size()); ++i) {   // Möller–Trumbore, both sides
        const QVector3D& a = m_points[m_tris[i].v[0]];
        const QVector3D e1 = m_points[m_tris[i].v[1]] - a, e2 = m_points[m_tris[i].v[2]] - a;
        const QVector3D p = QVector3D::crossProduct(dir, e2);
        const float det = QVector3D::dotProduct(e1, p);
        if (std::abs(det) < 1e-20f) continue;
        const float inv = 1.0f / det;
        const QVector3D s = origin - a;
        const float u = QVector3D::dotProduct(s, p) * inv;
        if (u < 0.0f || u > 1.0f) continue;
        const QVector3D q = QVector3D::crossProduct(s, e1);
        const float v = QVector3D::dotProduct(dir, q) * inv;
        if (v < 0.0f || u + v > 1.0f) continue;
        const float t = QVector3D::dotProduct(e2, q) * inv;
        if (t > 0.0f && t < bestT) { bestT = t; best = i; }
    }
    if (best != -1 && tOut) *tOut = bestT;
    return best;
}

const AxisFeature& MeshFeatures::faceFeature(int triangle) const {
    return m_faces[m_regionOf[triangle]];
}

std::vector<const AxisFeature*> MeshFeatures::edgesAround(int triangle) const {
    std::vector<const AxisFeature*> out;
    for (int i : m_faceEdges[m_regionOf[triangle]]) out.push_back(&m_edges[i]);
    return out;
}
