#include "Assembly.h"
#include "SceneGraph.h"
#include <QMatrix4x4>
#include <QtMath>
#include <algorithm>
#include <cmath>
#include <set>

using Kind = MateFeature::Kind;

const char* mateTypeName(MateType t) {
    switch (t) {
    case MateType::Flush:         return "Flush";
    case MateType::Concentric:    return "Concentric";
    case MateType::Insert:        return "Insert";
    case MateType::Coincident:    return "Coincident";
    case MateType::Parallel:      return "Parallel";
    case MateType::Perpendicular: return "Perpendicular";
    }
    return "Flush";
}

MateType mateTypeFromName(const QString& name) {
    for (MateType t : {MateType::Flush, MateType::Concentric, MateType::Insert, MateType::Coincident,
                       MateType::Parallel, MateType::Perpendicular})
        if (name == mateTypeName(t)) return t;
    return MateType::Flush;
}

const char* mateFeatureKindName(Kind k) {
    switch (k) {
    case Kind::Plane:  return "Plane";
    case Kind::Axis:   return "Axis";
    case Kind::Circle: return "Circle";
    case Kind::Point:  return "Point";
    }
    return "Point";
}

Kind mateFeatureKindFromName(const QString& name) {
    for (Kind k : {Kind::Plane, Kind::Axis, Kind::Circle, Kind::Point})
        if (name == mateFeatureKindName(k)) return k;
    return Kind::Point;
}

namespace assembly {
namespace {

constexpr float kApplyDistance = 2e-6f;   // corrections smaller than this are float noise
constexpr float kApplyAngle    = 2e-5f;

bool directional(Kind k) { return k != Kind::Point; }
// Planes and circles carry a normal (transforms as a normal), axes a direction
bool normalLike(Kind k) { return k == Kind::Plane || k == Kind::Circle; }

float angleBetween(const QVector3D& a, const QVector3D& b) {
    return std::atan2(QVector3D::crossProduct(a, b).length(), QVector3D::dotProduct(a, b));
}

// Minimal rotation taking unit `from` onto unit `to` (any axis at 180°)
QQuaternion rotationBetween(const QVector3D& from, const QVector3D& to) {
    const float angle = angleBetween(from, to);
    if (angle < 1e-7f) return {};
    QVector3D axis = QVector3D::crossProduct(from, to);
    if (axis.length() < 1e-6f) {
        axis = QVector3D::crossProduct(from, QVector3D(1, 0, 0));
        if (axis.length() < 1e-3f) axis = QVector3D::crossProduct(from, QVector3D(0, 1, 0));
    }
    return QQuaternion::fromAxisAndAngle(axis.normalized(), qRadiansToDegrees(angle));
}

QVector3D anyPerpendicular(const QVector3D& d) {
    QVector3D p = QVector3D::crossProduct(d, QVector3D(1, 0, 0));
    if (p.length() < 1e-3f) p = QVector3D::crossProduct(d, QVector3D(0, 1, 0));
    return p.normalized();
}

// The smallest move that satisfies the mate: rotate about `pivot`, then translate
struct Correction {
    bool        valid = false;
    QQuaternion rotation;
    QVector3D   pivot, translation;
    float       angle = 0.0f;
};

Correction correction(const SceneGraph& scene, const Mate& m) {
    Correction c;
    const WorldFeature a = toWorld(scene, m.moving), b = toWorld(scene, m.fixed);
    const bool da = directional(a.kind), db = directional(b.kind);
    const float s = m.aligned ? 1.0f : -1.0f;
    c.pivot = a.point;
    auto turnTo = [&](const QVector3D& target) {
        c.angle = angleBetween(a.direction, target);
        c.rotation = rotationBetween(a.direction, target);
    };
    switch (m.type) {
    case MateType::Flush:
        if (da && db) {
            turnTo(s * b.direction);
            c.translation = b.direction * (m.offset - QVector3D::dotProduct(a.point - b.point, b.direction));
        } else if (db) {
            c.translation = b.direction * (m.offset - QVector3D::dotProduct(a.point - b.point, b.direction));
        } else if (da) {
            c.translation = a.direction * (QVector3D::dotProduct(b.point - a.point, a.direction) - m.offset);
        } else {
            return c;
        }
        break;
    case MateType::Concentric: {
        const QVector3D v = b.point - a.point;
        if (da && db) {
            turnTo(s * b.direction);
            c.translation = v - b.direction * QVector3D::dotProduct(v, b.direction);
        } else if (db) {
            c.translation = v - b.direction * QVector3D::dotProduct(v, b.direction);
        } else if (da) {
            c.translation = v - a.direction * QVector3D::dotProduct(v, a.direction);
        } else {
            return c;
        }
        break;
    }
    case MateType::Insert:
        if (!da || !db) return c;
        turnTo(s * b.direction);
        c.translation = b.point + b.direction * m.offset - a.point;
        break;
    case MateType::Coincident:
        c.translation = b.point - a.point;
        break;
    case MateType::Parallel:
        if (!da || !db) return c;
        turnTo(s * b.direction);
        break;
    case MateType::Perpendicular: {
        if (!da || !db) return c;
        QVector3D target = a.direction - b.direction * QVector3D::dotProduct(a.direction, b.direction);
        target = target.length() < 1e-6f ? anyPerpendicular(b.direction) : target.normalized();
        turnTo(target);
        break;
    }
    }
    c.valid = true;
    return c;
}

std::vector<int> pathToRoot(const SceneGraph& scene, int id) {
    std::vector<int> path;
    for (const SceneObject* o = scene.getObject(id); o; o = scene.getObject(o->parentId)) path.push_back(o->id);
    return path;
}

} // namespace

WorldFeature toWorld(const SceneGraph& scene, const MateFeature& f) {
    WorldFeature w;
    w.kind = f.kind;
    const QMatrix4x4 wt = scene.worldTransform(f.objectId);
    w.point = wt.map(f.point);
    if (directional(f.kind)) {
        w.direction = (normalLike(f.kind) ? wt.inverted().transposed().mapVector(f.direction)
                                          : wt.mapVector(f.direction)).normalized();
    }
    return w;
}

MateFeature fromWorld(const SceneGraph& scene, int objectId, Kind kind, const QVector3D& worldPoint,
                      const QVector3D& worldDirection, float radius) {
    MateFeature f;
    f.objectId = objectId;
    f.kind = kind;
    f.radius = radius;
    const QMatrix4x4 wt = scene.worldTransform(objectId);
    f.point = wt.inverted().map(worldPoint);
    if (directional(kind)) {
        f.direction = (normalLike(kind) ? wt.transposed().mapVector(worldDirection)
                                        : wt.inverted().mapVector(worldDirection)).normalized();
    }
    return f;
}

std::vector<MateType> typesFor(Kind a, Kind b) {
    auto is = [](Kind k, std::initializer_list<Kind> ks) { return std::find(ks.begin(), ks.end(), k) != ks.end(); };
    if (a == Kind::Point || b == Kind::Point) {
        const Kind other = a == Kind::Point ? b : a;
        switch (other) {
        case Kind::Point:  return {MateType::Coincident};
        case Kind::Plane:  return {MateType::Flush};
        case Kind::Axis:   return {MateType::Concentric};
        case Kind::Circle: return {MateType::Coincident, MateType::Concentric, MateType::Flush};
        }
    }
    if (a == Kind::Plane && b == Kind::Plane)
        return {MateType::Flush, MateType::Parallel, MateType::Perpendicular};
    if (a == Kind::Circle && b == Kind::Circle)
        return {MateType::Insert, MateType::Concentric, MateType::Flush, MateType::Coincident, MateType::Parallel};
    if (is(a, {Kind::Axis, Kind::Circle}) && is(b, {Kind::Axis, Kind::Circle}))
        return {MateType::Concentric, MateType::Parallel, MateType::Perpendicular};
    // A plane with a circle or an axis
    const Kind other = a == Kind::Plane ? b : a;
    if (other == Kind::Circle) return {MateType::Flush, MateType::Parallel, MateType::Perpendicular};
    return {MateType::Parallel, MateType::Perpendicular};
}

QString typeLabel(MateType t, Kind a, Kind b) {
    const bool plane = a == Kind::Plane || b == Kind::Plane;
    const bool point = a == Kind::Point || b == Kind::Point;
    const bool line = a == Kind::Axis || b == Kind::Axis;
    switch (t) {
    case MateType::Flush:
        if (point) return "Point on face";
        if (a == Kind::Circle && b == Kind::Circle) return "Edges in one plane";
        if (a == Kind::Circle || b == Kind::Circle) return "Seat on face";
        return "Flush faces";
    case MateType::Concentric:
        if (point) return "Point on axis";
        return "Align axes";
    case MateType::Insert:        return "Insert (axes and centres)";
    case MateType::Coincident:    return point && (a == Kind::Circle || b == Kind::Circle) ? "Point at centre" : "Coincident";
    case MateType::Parallel:
        if (plane && (a != Kind::Plane || b != Kind::Plane)) return "Axis square to face";
        return plane ? "Parallel faces" : line || a == Kind::Circle ? "Parallel axes" : "Parallel";
    case MateType::Perpendicular:
        if (plane && (a != Kind::Plane || b != Kind::Plane)) return "Axis along face";
        return plane ? "Faces at 90°" : "Axes at 90°";
    }
    return mateTypeName(t);
}

bool usesOffset(MateType t, Kind, Kind) {
    return t == MateType::Flush || t == MateType::Insert;
}

bool usesAlignment(MateType t, Kind a, Kind b) {
    if (!directional(a) || !directional(b)) return false;
    return t == MateType::Flush || t == MateType::Concentric || t == MateType::Insert || t == MateType::Parallel;
}

int movingBody(const SceneGraph& scene, const Mate& m, QString* why) {
    auto fail = [&](const QString& reason) { if (why) *why = reason; return -1; };
    const SceneObject* a = scene.getObject(m.moving.objectId);
    const SceneObject* b = scene.getObject(m.fixed.objectId);
    if (!a || !b) return fail("a part it refers to is gone");
    if (a->id == b->id) return fail("both features are on the same part");
    const std::vector<int> pa = pathToRoot(scene, a->id), pb = pathToRoot(scene, b->id);
    const std::set<int> ancestorsB(pb.begin(), pb.end());
    for (size_t i = 0; i < pa.size(); ++i) {
        if (!ancestorsB.count(pa[i])) continue;
        // pa[i] is the lowest shared ancestor; the body is the one below it
        if (i == 0)
            return fail(QString("%1 contains %2 in the hierarchy, so it can't move without it")
                            .arg(a->name, b->name));
        return pa[i - 1];
    }
    return pa.back();   // no shared ancestor: the moving part's whole tree moves
}

bool closestAlignment(const SceneGraph& scene, const Mate& m) {
    if (!directional(m.moving.kind) || !directional(m.fixed.kind)) return m.aligned;
    const WorldFeature a = toWorld(scene, m.moving), b = toWorld(scene, m.fixed);
    return QVector3D::dotProduct(a.direction, b.direction) > 0;
}

Residual residual(const SceneGraph& scene, const Mate& m) {
    Residual r;
    const Correction c = correction(scene, m);
    if (!c.valid || movingBody(scene, m) < 0) { r.valid = false; return r; }
    r.distance = c.translation.length();
    r.angle = c.angle;
    return r;
}

SolveResult solve(SceneGraph& scene, int maxIterations) {
    SolveResult result;
    std::set<int> moved;
    std::vector<const Mate*> mates;
    for (auto& [id, m] : scene.allMates())
        if (m.enabled && movingBody(scene, m) >= 0 && correction(scene, m).valid) mates.push_back(&m);

    for (; result.iterations < maxIterations; ++result.iterations) {
        bool changed = false;
        for (const Mate* m : mates) {
            const Correction c = correction(scene, *m);
            if (c.angle < kApplyAngle && c.translation.length() < kApplyDistance) continue;
            const int body = movingBody(scene, *m);
            QMatrix4x4 delta;
            delta.translate(c.pivot + c.translation);
            delta.rotate(c.rotation);
            delta.translate(-c.pivot);
            scene.setWorldTransform(body, delta * scene.worldTransform(body));
            moved.insert(body);
            changed = true;
        }
        if (!changed) break;
    }

    for (auto& [id, m] : scene.allMates()) {
        if (!m.enabled) continue;
        QString why;
        if (movingBody(scene, m, &why) < 0) {
            result.unsatisfied.push_back(id);
            result.problems << QString("%1: %2").arg(m.name, why);
        } else if (!correction(scene, m).valid) {
            result.unsatisfied.push_back(id);
            result.problems << QString("%1: %2 doesn't apply to these features").arg(m.name, mateTypeName(m.type));
        } else if (!residual(scene, m).satisfied()) {
            result.unsatisfied.push_back(id);
            result.problems << QString("%1: conflicts with other mates").arg(m.name);
        }
    }
    result.moved.assign(moved.begin(), moved.end());
    for (int id : result.moved) emit scene.objectChanged(id);
    return result;
}

QString describe(const SceneGraph& scene, const Mate& m) {
    const SceneObject* a = scene.getObject(m.moving.objectId);
    const SceneObject* b = scene.getObject(m.fixed.objectId);
    return QString("%1: %2 → %3").arg(typeLabel(m.type, m.moving.kind, m.fixed.kind),
                                     a ? a->name : QString("?"), b ? b->name : QString("?"));
}

QString nextName(const SceneGraph& scene) {
    for (int i = 1;; ++i) {
        const QString n = QString("Mate%1").arg(i);
        bool used = false;
        for (auto& [id, m] : scene.allMates()) used = used || m.name == n;
        if (!used) return n;
    }
}

} // namespace assembly
