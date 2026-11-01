#include "Interference.h"
#include "SceneGraph.h"
#include "SceneQuery.h"
#include "PrimitiveMesh.h"
#include <algorithm>
#include <cmath>

// ── Bounding-volume tree over a triangle soup ────────────────────────────────

struct InterferenceChecker::Bvh {
    struct Node {
        QVector3D mn, mx;
        int left = -1, right = -1;   // children, or -1 for a leaf
        int first = 0, count = 0;    // leaf: range in `tris`
    };
    std::vector<Node>      nodes;
    std::vector<int>       tris;
    std::vector<QVector3D> verts;    // 3 per triangle, the soup's positions

    explicit Bvh(const std::vector<float>& soup) {
        const int n = int(soup.size() / 18);
        verts.reserve(size_t(n) * 3);
        for (int t = 0; t < n; ++t)
            for (int k = 0; k < 3; ++k) {
                const float* v = &soup[size_t(t) * 18 + size_t(k) * 6];
                verts.emplace_back(v[0], v[1], v[2]);
            }
        tris.resize(size_t(n));
        for (int i = 0; i < n; ++i) tris[size_t(i)] = i;
        if (n > 0) build(0, n);
    }

    QVector3D centroid(int t) const { return (verts[size_t(t) * 3] + verts[size_t(t) * 3 + 1] + verts[size_t(t) * 3 + 2]) / 3.0f; }

    int build(int first, int count) {
        Node node;
        node.mn = QVector3D(1e30f, 1e30f, 1e30f);
        node.mx = -node.mn;
        for (int i = first; i < first + count; ++i)
            for (int k = 0; k < 3; ++k) {
                const QVector3D& v = verts[size_t(tris[size_t(i)]) * 3 + size_t(k)];
                for (int a = 0; a < 3; ++a) { node.mn[a] = std::min(node.mn[a], v[a]); node.mx[a] = std::max(node.mx[a], v[a]); }
            }
        const int index = int(nodes.size());
        nodes.push_back(node);
        if (count <= 8) {
            nodes[size_t(index)].first = first;
            nodes[size_t(index)].count = count;
            return index;
        }
        const QVector3D ext = node.mx - node.mn;
        const int axis = ext.x() > ext.y() ? (ext.x() > ext.z() ? 0 : 2) : (ext.y() > ext.z() ? 1 : 2);
        const int mid = first + count / 2;
        std::nth_element(tris.begin() + first, tris.begin() + mid, tris.begin() + first + count,
                         [&](int a, int b) { return centroid(a)[axis] < centroid(b)[axis]; });
        const int l = build(first, mid - first);
        const int r = build(mid, first + count - mid);
        nodes[size_t(index)].left = l;
        nodes[size_t(index)].right = r;
        return index;
    }
};

namespace {

// Separating-axis test for two triangles
bool trianglesIntersect(const QVector3D a[3], const QVector3D b[3]) {
    auto separated = [&](const QVector3D& axis) {
        if (axis.lengthSquared() < 1e-20f) return false;
        float amin = 1e30f, amax = -1e30f, bmin = 1e30f, bmax = -1e30f;
        for (int i = 0; i < 3; ++i) {
            const float pa = QVector3D::dotProduct(a[i], axis), pb = QVector3D::dotProduct(b[i], axis);
            amin = std::min(amin, pa); amax = std::max(amax, pa);
            bmin = std::min(bmin, pb); bmax = std::max(bmax, pb);
        }
        // A sliver of overlap (touching faces) does not count
        const float eps = 1e-6f * axis.length();
        return amax <= bmin + eps || bmax <= amin + eps;
    };
    const QVector3D ea[3] = {a[1] - a[0], a[2] - a[1], a[0] - a[2]};
    const QVector3D eb[3] = {b[1] - b[0], b[2] - b[1], b[0] - b[2]};
    const QVector3D na = QVector3D::crossProduct(ea[0], ea[1]);
    const QVector3D nb = QVector3D::crossProduct(eb[0], eb[1]);
    if (separated(na) || separated(nb)) return false;
    for (const QVector3D& x : ea)
        for (const QVector3D& y : eb)
            if (separated(QVector3D::crossProduct(x, y))) return false;
    // Coplanar triangles: the edge normals within the plane
    if (QVector3D::crossProduct(na, nb).lengthSquared() < 1e-12f * na.lengthSquared() * nb.lengthSquared()) {
        for (const QVector3D& x : ea) if (separated(QVector3D::crossProduct(na, x))) return false;
        for (const QVector3D& y : eb) if (separated(QVector3D::crossProduct(na, y))) return false;
    }
    return true;
}

scenequery::Obb nodeBox(const QVector3D& mn, const QVector3D& mx, const QMatrix4x4& m) {
    scenequery::Obb o;
    o.center = m.map((mn + mx) * 0.5f);
    const QVector3D he = (mx - mn) * 0.5f;
    for (int i = 0; i < 3; ++i) {
        const QVector3D col = m.column(i).toVector3D();
        const float len = col.length();
        o.axis[i] = len > 1e-12f ? col / len : QVector3D(i == 0, i == 1, i == 2);
        o.half[i] = he[i] * len;
    }
    return o;
}

// The object (itself or an ancestor) whose joint moves it, or -1 if static
int motionGroup(const SceneGraph& scene, int id) {
    for (const SceneObject* o = scene.getObject(id); o; o = scene.getObject(o->parentId))
        for (auto& [jid, j] : scene.allJoints())
            if (j.childBodyId == o->id) return o->id;
    return -1;
}

bool underRigidBody(const SceneGraph& scene, int id) {
    for (const SceneObject* o = scene.getObject(id); o; o = scene.getObject(o->parentId))
        if (scene.allRigidBodies().count(o->id)) return true;
    return false;
}

} // namespace

InterferenceChecker::InterferenceChecker() = default;
InterferenceChecker::~InterferenceChecker() = default;

const InterferenceChecker::Bvh* InterferenceChecker::bvhFor(const std::vector<float>* soup) {
    auto& slot = m_bvhs[soup];
    if (!slot) slot = std::make_unique<Bvh>(*soup);
    return slot.get();
}

bool InterferenceChecker::intersect(const SceneGraph& scene, int ida, int idb) {
    const SceneObject* oa = scene.getObject(ida);
    const SceneObject* ob = scene.getObject(idb);
    const std::vector<float>* sa = oa ? surfaceOf(*oa) : nullptr;
    const std::vector<float>* sb = ob ? surfaceOf(*ob) : nullptr;
    if (!sa || !sb) return false;
    if (!scenequery::overlap(scenequery::worldBox(scene, ida), scenequery::worldBox(scene, idb))) return false;

    const Bvh* A = bvhFor(sa);
    const Bvh* B = bvhFor(sb);
    if (A->nodes.empty() || B->nodes.empty()) return false;
    bool invertible = false;
    const QMatrix4x4 m = scene.worldTransform(ida).inverted(&invertible) * scene.worldTransform(idb);   // B → A local
    if (!invertible) return false;

    std::vector<std::pair<int, int>> stack{{0, 0}};
    while (!stack.empty()) {
        auto [ia, ib] = stack.back();
        stack.pop_back();
        const Bvh::Node& na = A->nodes[size_t(ia)];
        const Bvh::Node& nb = B->nodes[size_t(ib)];
        if (!scenequery::overlap(nodeBox(na.mn, na.mx, QMatrix4x4()), nodeBox(nb.mn, nb.mx, m))) continue;
        const bool leafA = na.left < 0, leafB = nb.left < 0;
        if (leafA && leafB) {
            for (int i = na.first; i < na.first + na.count; ++i) {
                const int ta = A->tris[size_t(i)];
                const QVector3D va[3] = {A->verts[size_t(ta) * 3], A->verts[size_t(ta) * 3 + 1], A->verts[size_t(ta) * 3 + 2]};
                for (int j = nb.first; j < nb.first + nb.count; ++j) {
                    const int tb = B->tris[size_t(j)];
                    const QVector3D vb[3] = {m.map(B->verts[size_t(tb) * 3]), m.map(B->verts[size_t(tb) * 3 + 1]),
                                             m.map(B->verts[size_t(tb) * 3 + 2])};
                    if (trianglesIntersect(va, vb)) return true;
                }
            }
            continue;
        }
        // Descend into the larger node (or the one that isn't a leaf)
        const float volA = (na.mx - na.mn).lengthSquared(), volB = (nb.mx - nb.mn).lengthSquared();
        if (leafB || (!leafA && volA >= volB)) {
            stack.push_back({na.left, ib});
            stack.push_back({na.right, ib});
        } else {
            stack.push_back({ia, nb.left});
            stack.push_back({ia, nb.right});
        }
    }
    return false;
}

void InterferenceChecker::begin(const SceneGraph& scene) {
    clear();
    m_active = true;
    std::vector<std::pair<int, int>> parts;   // (object, motion group)
    for (auto& [id, o] : scene.allObjects())
        if (surfaceOf(o) && !underRigidBody(scene, id)) parts.push_back({id, motionGroup(scene, id)});
    std::sort(parts.begin(), parts.end());
    for (size_t i = 0; i < parts.size(); ++i)
        for (size_t j = i + 1; j < parts.size(); ++j) {
            const int ga = parts[i].second, gb = parts[j].second;
            if (ga == gb) continue;   // move together (or are both static)
            const int a = parts[i].first, b = parts[j].first;
            if (intersect(scene, a, b)) continue;   // assembled touching: not a collision
            m_pairs.push_back({a, b});
        }
}

void InterferenceChecker::check(const SceneGraph& scene, double time) {
    if (!m_active) return;
    m_now.clear();
    std::set<std::pair<int, int>> hit;
    for (const auto& pr : m_pairs)
        if (intersect(scene, pr.first, pr.second)) {
            hit.insert(pr);
            m_now.insert(pr.first);
            m_now.insert(pr.second);
        }
    for (auto it = m_open.begin(); it != m_open.end();) {
        Event& e = m_events[it->second];
        if (hit.count(it->first)) { e.end = time; ++it; continue; }
        e.ongoing = false;
        it = m_open.erase(it);
    }
    for (const auto& pr : hit) {
        if (m_open.count(pr)) continue;
        m_open[pr] = m_events.size();
        m_events.push_back({pr.first, pr.second, time, time, true});
    }
}

void InterferenceChecker::clear() {
    m_active = false;
    m_pairs.clear();
    m_open.clear();
    m_events.clear();
    m_now.clear();
    m_bvhs.clear();
}
