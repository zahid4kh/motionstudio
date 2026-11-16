#include "Gait.h"
#include "automation/Kinematics.h"
#include "scene/SceneGraph.h"
#include "scene/SceneQuery.h"
#include <QtMath>
#include <algorithm>
#include <cmath>
#include <functional>
#include <set>

namespace gait {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kReachTol = 0.005;   // m: a foot this far from its spot can't reach it

double ease(double u) {                      // 0 → 1 with zero speed at both ends
    u = std::clamp(u, 0.0, 1.0);
    return u * u * (3 - 2 * u);
}
double cycloid(double v) {                   // a foot's way along the floor: lands softly
    v = std::clamp(v, 0.0, 1.0);
    return v - std::sin(2 * kPi * v) / (2 * kPi);
}

QVector3D at(const QMatrix4x4& m) { return m.column(3).toVector3D(); }

Place mid(const Place& a, const Place& b) {
    return {(a.x + b.x) / 2, (a.z + b.z) / 2, a.heading + angleDiff(b.heading, a.heading) / 2};
}

Place lerp(const Place& a, const Place& b, double u) {
    return {a.x + (b.x - a.x) * u, a.z + (b.z - a.z) * u, a.heading + angleDiff(b.heading, a.heading) * u};
}

bool samePlace(const Place& a, const Place& b) {
    return std::hypot(a.x - b.x, a.z - b.z) < 1e-6 && std::fabs(angleDiff(a.heading, b.heading)) < 1e-6;
}

void collect(const SceneGraph& g, int id, std::vector<int>& out) {
    const SceneObject* o = g.getObject(id);
    if (!o) return;
    out.push_back(id);
    for (int c : o->childIds) collect(g, c, out);
}

// Runs `fn` with the given joints at 0 (followers too), then puts everything back
void atZero(SceneGraph& g, const std::vector<int>& joints, const std::function<void()>& fn) {
    std::map<int, float> saved;
    for (auto& [jid, j] : g.allJoints()) saved[jid] = j.currentValue;
    for (int jid : joints)
        if (Joint* j = g.getJoint(jid)) j->currentValue = 0;
    g.applyCouplings();
    fn();
    for (auto& [jid, v] : saved) g.getJoint(jid)->currentValue = v;
    g.applyCouplings();
}

int depthBelow(const SceneGraph& g, int id, int root) {
    int d = 0;
    for (const SceneObject* o = g.getObject(id); o && o->id != root; o = g.getObject(o->parentId)) ++d;
    return d;
}

} // namespace

// ── Places ───────────────────────────────────────────────────────────────────

double angleDiff(double a, double b) {
    double d = std::fmod(a - b, 2 * kPi);
    if (d <= -kPi) d += 2 * kPi;
    if (d > kPi) d -= 2 * kPi;
    return d;
}

Place placeOf(const QMatrix4x4& m) {
    const QVector3D xAxis = m.column(0).toVector3D();
    return {m(0, 3), m(2, 3), std::atan2(-double(xAxis.z()), double(xAxis.x()))};
}

QMatrix4x4 frameOf(const Place& p, double y) {
    QMatrix4x4 m;
    m.translate(float(p.x), float(y), float(p.z));
    m.rotate(float(qRadiansToDegrees(p.heading)), 0, 1, 0);
    return m;
}

Place advance(const Place& p, double forward, double left, double turn) {
    // Forward is (cos h, −sin h) in x, z; the robot's left is its −Z: (−sin h, −cos h)
    const double h = p.heading + turn / 2;
    Place q;
    q.x = p.x + forward * std::cos(h) - left * std::sin(h);
    q.z = p.z - forward * std::sin(h) - left * std::cos(h);
    q.heading = angleDiff(p.heading + turn, 0);
    return q;
}

// ── Legs ─────────────────────────────────────────────────────────────────────

bool findLegs(SceneGraph& g, int root, Legs* legs, QString* error, int leftFoot, int rightFoot) {
    auto fail = [&](const QString& e) {
        if (error) *error = e;
        return false;
    };
    const SceneObject* r = g.getObject(root);
    if (!r) return fail("the robot does not exist");
    Legs L;
    L.root = root;
    std::vector<int> sub;
    collect(g, root, sub);

    // The feet: given, else named
    const int given[2] = {leftFoot, rightFoot};
    for (int side = 0; side < 2; ++side) {
        if (given[side] != -1) {
            if (std::find(sub.begin(), sub.end(), given[side]) == sub.end())
                return fail(QString("the %1 foot is not part of %2").arg(side ? "right" : "left", r->name));
            L.foot[side] = given[side];
            continue;
        }
        const QString exact = side ? "right foot" : "left foot", word = side ? "right" : "left";
        int best = -1, bestScore = -1;
        for (int id : sub) {
            const QString n = g.getObject(id)->name.toLower();
            if (!n.contains("foot") || !n.contains(word)) continue;
            // An exact name wins, then the one nearest the root
            const int score = (n == exact ? 100000 : 0) + 1000 - depthBelow(g, id, root);
            if (score > bestScore) { bestScore = score; best = id; }
        }
        L.foot[side] = best;
    }
    if (L.foot[0] == -1 || L.foot[1] == -1)
        return fail(QString("%1 has no legs to walk on: it needs a \"Left Foot\" and a \"Right Foot\" "
                            "moved by actuated joints (a humanoid built on legs has them)").arg(r->name));
    if (L.foot[0] == L.foot[1]) return fail("the left and the right foot are the same object");

    // Each foot's own joints: the actuated ones above it that the other foot lacks
    const std::vector<int> c0 = kin::chainTo(g, L.foot[0]), c1 = kin::chainTo(g, L.foot[1]);
    const std::set<int> s0(c0.begin(), c0.end()), s1(c1.begin(), c1.end());
    for (int j : c0) if (!s1.count(j)) L.chain[0].push_back(j);
    for (int j : c1) if (!s0.count(j)) L.chain[1].push_back(j);
    for (int side = 0; side < 2; ++side)
        if (L.chain[side].size() < 3)
            return fail(QString("the %1 leg of %2 has %3 actuated joint%4; walking takes at least a hip, a knee "
                                "and an ankle").arg(side ? "right" : "left", r->name)
                            .arg(L.chain[side].size()).arg(L.chain[side].size() == 1 ? "" : "s"));

    // Arm swing: the shoulder pitch actuators, if there are any
    for (int id : sub) {
        for (auto& [jid, j] : g.allJoints()) {
            if (j.childBodyId != id) continue;
            const Motor* m = g.motorForJoint(jid);
            if (!m) continue;
            const QString n = m->name.toLower();
            if (!n.contains("shoulderpitch")) continue;
            const int side = (n.startsWith('l') || n.contains("left")) ? 0
                           : (n.startsWith('r') || n.contains("right")) ? 1 : -1;
            if (side >= 0 && L.arm[side] == -1) L.arm[side] = jid;
        }
    }

    // Geometry with the legs straight (every leg joint at 0)
    std::vector<int> legJoints = L.chain[0];
    legJoints.insert(legJoints.end(), L.chain[1].begin(), L.chain[1].end());
    const QMatrix4x4 rootFrame = kin::toolFrame(g, root);
    const QMatrix4x4 floorInv = frameOf(placeOf(rootFrame), rootFrame(1, 3)).inverted();
    QVector3D hip[2], sMin[2], sMax[2];
    bool hasSole[2] = {false, false};
    atZero(g, legJoints, [&] {
        for (int side = 0; side < 2; ++side) {
            const QMatrix4x4 foot = kin::toolFrame(g, L.foot[side]);
            L.footHome[side] = floorInv * foot;
            const Joint* top = g.getJoint(L.chain[side].front());
            hip[side] = (floorInv * g.worldTransform(top->parentBodyId)).map(top->pivot);
            // The sole: the box around the foot's parts, in the foot's frame
            const QMatrix4x4 inv = foot.inverted();
            std::vector<int> parts;
            collect(g, L.foot[side], parts);
            for (int id : parts) {
                const SceneObject* o = g.getObject(id);
                if (o->type == PrimitiveType::ImportedMesh && !o->mesh) continue;
                const QMatrix4x4 m = inv * g.worldTransform(id);
                const QVector3D a = o->aabbMin(), b = o->aabbMax();
                for (int k = 0; k < 8; ++k) {
                    const QVector3D c = m.map(QVector3D(k & 1 ? b.x() : a.x(), k & 2 ? b.y() : a.y(), k & 4 ? b.z() : a.z()));
                    if (!hasSole[side]) { sMin[side] = sMax[side] = c; hasSole[side] = true; }
                    sMin[side] = QVector3D(std::min(sMin[side].x(), c.x()), std::min(sMin[side].y(), c.y()), std::min(sMin[side].z(), c.z()));
                    sMax[side] = QVector3D(std::max(sMax[side].x(), c.x()), std::max(sMax[side].y(), c.y()), std::max(sMax[side].z(), c.z()));
                }
            }
        }
    });
    L.hip[0] = hip[0];
    L.hip[1] = hip[1];
    L.ankleHeight = (L.footHome[0](1, 3) + L.footHome[1](1, 3)) / 2;
    L.legLength = ((hip[0] - at(L.footHome[0])).length() + (hip[1] - at(L.footHome[1])).length()) / 2;
    L.hipWidth = std::hypot(L.footHome[0](0, 3) - L.footHome[1](0, 3), L.footHome[0](2, 3) - L.footHome[1](2, 3));
    if (L.legLength < 1e-3) return fail(QString("the legs of %1 have no length").arg(r->name));
    if (L.ankleHeight < -1e-4)
        return fail(QString("the feet of %1 are below its root: it must stand on the floor at its root").arg(r->name));
    if (hasSole[0]) {
        L.footLength = sMax[0].x() - sMin[0].x();
        L.footWidth = sMax[0].z() - sMin[0].z();
        L.soleCentre = QVector3D((sMin[0].x() + sMax[0].x()) / 2, float(-L.ankleHeight), (sMin[0].z() + sMax[0].z()) / 2);
    } else {
        L.footLength = 0.35 * L.legLength;
        L.footWidth = 0.14 * L.legLength;
        L.soleCentre = QVector3D(0, float(-L.ankleHeight), 0);
    }
    *legs = std::move(L);
    return true;
}

bool canWalk(SceneGraph& scene, int root) {
    Legs legs;
    return findLegs(scene, root, &legs, nullptr);
}

QString resolve(Params& p, const Legs& legs) {
    const double L = legs.legLength, mm = 1000;
    auto num = [](double v, int prec = 0) { return QString::number(v, 'f', prec); };
    if (p.stepLength <= 0) p.stepLength = 0.4 * L;
    if (p.stepLength > 0.9 * L)
        return QString("a step can be at most %1 mm long for these legs").arg(num(0.9 * L * mm));
    const double maxSide = std::max(0.02, 0.7 * legs.hipWidth);
    if (p.sideStep <= 0) p.sideStep = std::min(0.5 * legs.hipWidth, maxSide);
    if (p.sideStep > maxSide + 1e-9)
        return QString("a side step can be at most %1 mm, or the feet would cross").arg(num(maxSide * mm));
    if (p.turnPerStep <= 0) p.turnPerStep = qDegreesToRadians(20.0);
    if (p.turnPerStep > qDegreesToRadians(45.0) + 1e-9) return "a step can turn at most 45°";
    if (p.stepHeight <= 0) p.stepHeight = 0.08 * L;
    if (p.stepHeight > 0.4 * L) return QString("the feet can lift at most %1 mm").arg(num(0.4 * L * mm));
    if (p.speed > 0) p.stepTime = p.stepLength / p.speed;
    else if (p.stepTime <= 0) p.stepTime = 0.55 * std::sqrt(L / 0.7);
    if (p.stepTime < 0.2) return QString("too fast: a step takes at least 0.2 s (at most %1 mm/s with %2 mm steps)")
                                     .arg(num(p.stepLength / 0.2 * mm)).arg(num(p.stepLength * mm));
    if (p.stepTime > 10) return "too slow: a step takes at most 10 s";
    if (p.sway < 0) p.sway = 0.12 * legs.hipWidth;
    p.sway = std::min(p.sway, 0.5 * legs.hipWidth);
    if (p.armSwing < 0) p.armSwing = (legs.arm[0] != -1 || legs.arm[1] != -1) ? qDegreesToRadians(12.0) : 0.0;
    p.armSwing = std::min(p.armSwing, qDegreesToRadians(60.0));
    p.doubleSupport = std::clamp(p.doubleSupport, 0.0, 0.6);
    // The hips sink so the leg behind and the one in front both reach the floor
    const double reach = 0.985 * L;   // the foot behind reaches further on its toes
    const double off = std::hypot(p.stepLength / 2, p.sideStep / 2 + p.sway);
    if (p.crouch < 0) {
        const double h = reach * reach - off * off;
        p.crouch = std::max(0.02 * L, h > 0 ? L - std::sqrt(h) : 0.5 * L);
    }
    if (p.crouch > 0.5 * L) return QString("the hips can sink at most %1 mm").arg(num(0.5 * L * mm));
    if (std::hypot(L - p.crouch, off) > 0.99 * L)
        return QString("the hips must sink at least %1 mm for %2 mm steps")
            .arg(num((L - std::sqrt(std::max(0.0, 0.98 * L * L - off * off))) * mm)).arg(num(p.stepLength * mm));
    return {};
}

// ── Planning ─────────────────────────────────────────────────────────────────

std::vector<Place> relativePlaces(const Place& start, double forward, double left, double turn, const Params& p) {
    auto count = [](double v, double per) { return per > 0 ? int(std::ceil(std::fabs(v) / per - 1e-9)) : 0; };
    const int n = std::max({count(forward, p.stepLength), count(left, p.sideStep), count(turn, p.turnPerStep)});
    std::vector<Place> out;
    Place cur = start;
    for (int i = 0; i < n; ++i) {
        cur = advance(cur, forward / n, left / n, turn / n);
        out.push_back(cur);
    }
    return out;
}

namespace {

// Turns on the spot from `cur` to `heading`, in equal steps of at most `per`
void turnOnSpot(std::vector<Place>& out, Place& cur, double heading, double per) {
    const double d = angleDiff(heading, cur.heading);
    const int n = int(std::ceil(std::fabs(d) / per - 1e-9));
    const double from = cur.heading;
    for (int i = 0; i < n; ++i) {
        cur.heading = angleDiff(from + d * (i + 1) / n, 0);
        out.push_back(cur);
    }
}

struct Polyline {
    std::vector<QPointF> pts;
    std::vector<double>  len;   // cumulative
    double total() const { return len.empty() ? 0 : len.back(); }
    int segment(double a) const {
        // The segment holding `a`; at a joint, the one after it
        int i = 0;
        while (i + 2 < int(pts.size()) && len[size_t(i) + 1] <= a + 1e-9) ++i;
        return i;
    }
    QPointF pointAt(double a) const {
        const int i = segment(a);
        const double l = len[size_t(i) + 1] - len[size_t(i)];
        const double u = l > 1e-12 ? std::clamp((a - len[size_t(i)]) / l, 0.0, 1.0) : 0.0;
        return pts[size_t(i)] + (pts[size_t(i) + 1] - pts[size_t(i)]) * u;
    }
    double headingAt(double a) const {
        const QPointF d = pts[size_t(segment(a)) + 1] - pts[size_t(segment(a))];
        return std::atan2(-d.y(), d.x());   // QPointF y holds z
    }
};

} // namespace

std::vector<Place> pathPlaces(const Place& start, const std::vector<QPointF>& points, std::optional<double> heading,
                              const Params& p) {
    std::vector<Place> out;
    Place cur = start;
    std::vector<QPointF> pts{QPointF(start.x, start.z)};
    for (const QPointF& q : points)
        if (std::hypot(q.x() - pts.back().x(), q.y() - pts.back().y()) > 1e-3) pts.push_back(q);

    if (pts.size() >= 2) {
        // Corners are rounded, so the robot turns while it walks
        Polyline line;
        line.pts.push_back(pts.front());
        const double radius = 1.5 * p.stepLength;
        for (size_t i = 1; i + 1 < pts.size(); ++i) {
            const QPointF a = pts[i - 1], v = pts[i], b = pts[i + 1];
            const double lin = std::hypot(v.x() - a.x(), v.y() - a.y()), lout = std::hypot(b.x() - v.x(), b.y() - v.y());
            const double d = std::min({radius, 0.45 * lin, 0.45 * lout});
            const QPointF p0 = v - (v - a) * (d / lin), p2 = v + (b - v) * (d / lout);
            for (int k = 0; k <= 8; ++k) {
                const double t = k / 8.0;
                line.pts.push_back(p0 * ((1 - t) * (1 - t)) + v * (2 * (1 - t) * t) + p2 * (t * t));
            }
        }
        line.pts.push_back(pts.back());
        // Drop points that coincide (a corner cut to nothing)
        std::vector<QPointF> clean{line.pts.front()};
        for (size_t i = 1; i < line.pts.size(); ++i)
            if (std::hypot(line.pts[i].x() - clean.back().x(), line.pts[i].y() - clean.back().y()) > 1e-6)
                clean.push_back(line.pts[i]);
        line.pts = clean;
        line.len = {0};
        for (size_t i = 1; i < line.pts.size(); ++i)
            line.len.push_back(line.len.back() + std::hypot(line.pts[i].x() - line.pts[i - 1].x(),
                                                            line.pts[i].y() - line.pts[i - 1].y()));

        // Face the way first when it is more than a step's turn away
        if (line.pts.size() >= 2) {
            const double h0 = line.headingAt(0);
            if (std::fabs(angleDiff(h0, cur.heading)) > p.turnPerStep + 1e-9) turnOnSpot(out, cur, h0, p.turnPerStep);
            double a = 0;
            const double total = line.total();
            for (int guard = 0; a < total - 1e-6 && guard < 100000; ++guard) {
                // The rest in equal steps, shorter where the way bends sharply
                const double rest = total - a;
                double d = rest / std::ceil(rest / p.stepLength - 1e-9);
                double h = line.headingAt(std::min(total, a + d));
                while (std::fabs(angleDiff(h, cur.heading)) > p.turnPerStep + 1e-9 && d > p.stepLength / 4) {
                    d /= 2;
                    h = line.headingAt(std::min(total, a + d));
                }
                if (std::fabs(angleDiff(h, cur.heading)) > p.turnPerStep + 1e-9) {
                    turnOnSpot(out, cur, line.headingAt(std::min(total, a + 1e-4)), p.turnPerStep);
                    h = line.headingAt(std::min(total, a + d));
                    if (std::fabs(angleDiff(h, cur.heading)) > p.turnPerStep + 1e-9) h = cur.heading;
                }
                a = std::min(total, a + d);
                const QPointF q = line.pointAt(a);
                cur = {q.x(), q.y(), angleDiff(h, 0)};
                out.push_back(cur);
            }
        }
    }
    if (heading) turnOnSpot(out, cur, *heading, p.turnPerStep);
    return out;
}

double Plan::duration() const {
    return steps.empty() && !crouchOnly ? 0.0 : 2 * prep + double(steps.size()) * params.stepTime;
}

Plan makePlan(const Place& start, const std::vector<Place>& places, const Params& p, int firstSide) {
    Plan plan;
    plan.start = start;
    plan.params = p;
    plan.prep = std::clamp(0.6 * p.stepTime, 0.25, 1.0);
    plan.body.push_back(start);
    if (places.empty()) return plan;
    int side = firstSide;
    if (side < 0) {
        // Towards the side the walk turns or goes, so the feet never cross
        const Place& q = places.front();
        const double dh = angleDiff(q.heading, start.heading);
        const double left = -(q.x - start.x) * std::sin(start.heading) - (q.z - start.z) * std::cos(start.heading);
        side = dh > 1e-6 ? 0 : dh < -1e-6 ? 1 : left > 1e-6 ? 0 : left < -1e-6 ? 1 : 0;
    }
    Place feet[2] = {start, start};
    for (const Place& q : places) {
        plan.steps.push_back({side, q});
        feet[side] = q;
        plan.body.push_back(mid(feet[0], feet[1]));
        side = 1 - side;
    }
    // The other foot comes alongside
    plan.steps.push_back({side, places.back()});
    plan.body.push_back(places.back());
    return plan;
}

// ── Walker ───────────────────────────────────────────────────────────────────

QMatrix4x4 Walker::target(int side, const Place& at) const {
    return frameOf(at, m_floorY) * m_legs.footHome[side];
}

void Walker::begin(SceneGraph& scene) {
    const QMatrix4x4 rootFrame = scene.worldTransform(m_legs.root);
    m_floorY = rootFrame(1, 3);
    const double toe = std::max(0.01, m_legs.soleCentre.x() + m_legs.footLength / 2);
    m_swingPitch = std::asin(std::clamp(0.7 * m_plan.params.stepHeight / toe, 0.0, 0.5));
    m_rootOffset = frameOf(m_plan.start, m_floorY).inverted() * rootFrame;
    for (int side = 0; side < 2; ++side) {
        m_startFoot[side] = kin::toolFrame(scene, m_legs.foot[side]);
        const Joint* a = scene.getJoint(m_legs.arm[side]);
        m_startArm[side] = a ? a->currentValue : 0.0;
    }
}

std::vector<int> Walker::joints() const {
    std::vector<int> out = m_legs.chain[0];
    out.insert(out.end(), m_legs.chain[1].begin(), m_legs.chain[1].end());
    if (m_plan.params.armSwing > 0)
        for (int a : m_legs.arm) if (a != -1) out.push_back(a);
    return out;
}

int Walker::stepsDone(double t) const {
    const double s = (t - m_plan.prep) / m_plan.params.stepTime;
    return std::clamp(int(std::floor(s + 1e-9)), 0, int(m_plan.steps.size()));
}

void Walker::stopAfter(double t) {
    const int n = int(m_plan.steps.size());
    const double s = (t - m_plan.prep) / m_plan.params.stepTime;
    if (s >= n) return;                     // already standing up
    const int keep = s < 0 ? 0 : int(std::floor(s)) + 1;   // the step under way finishes
    if (keep >= n) return;
    m_plan.steps.resize(size_t(keep));
    m_plan.body.resize(size_t(keep) + 1);
    if (keep == 0) { m_plan.crouchOnly = true; return; }
    // Where the feet are; the one behind comes alongside
    Place feet[2] = {m_plan.start, m_plan.start};
    for (const Footstep& f : m_plan.steps) feet[f.side] = f.at;
    const int last = m_plan.steps.back().side;
    if (!samePlace(feet[0], feet[1])) {
        m_plan.steps.push_back({1 - last, feet[last]});
        m_plan.body.push_back(feet[last]);
    }
}

bool Walker::apply(SceneGraph& scene, double t, std::map<int, double>* joints, double* error) {
    const Params& p = m_plan.params;
    const int n = int(m_plan.steps.size());
    const double T = p.stepTime, P = m_plan.prep;
    const double walkEnd = P + n * T;
    const bool any = m_plan.duration() > 0;

    // Where the feet are: each at its last footprint before step k (its start frame before any)
    auto footBefore = [&](int side, int k) {
        for (int i = std::min(k, n) - 1; i >= 0; --i)
            if (m_plan.steps[size_t(i)].side == side) return target(side, m_plan.steps[size_t(i)].at);
        return m_startFoot[side];
    };

    double crouch = 0, lateral = 0, arm = 0;
    Place body = m_plan.start;
    QMatrix4x4 foot[2];
    // Feet on the floor: where they stand and how far they would roll; the
    // roll used is the nearest to that the leg reaches
    bool onFloor[2] = {true, true};
    QMatrix4x4 standsAt[2];
    double rollFor[2] = {0, 0};
    int liftStep = -1;                   // the step whose foot is about to lift
    // The swinging foot, before its pitch is adjusted to what its leg reaches
    int swingSide = -1;
    double swingV = 0, swingUp = 0;
    QMatrix4x4 swingFrame;
    // Toe and heel clear the floor by a third of the lift at least
    auto clearFloor = [&](QMatrix4x4 m, double up) {
        const QVector3D half(float(m_legs.footLength / 2), 0, 0);
        const double low = std::min(m.map(m_legs.soleCentre + half).y(), m.map(m_legs.soleCentre - half).y());
        const double clear = m_floorY + 0.3 * p.stepHeight * up;
        if (low < clear) {
            QMatrix4x4 raise;
            raise.translate(0, float(clear - low), 0);
            m = raise * m;
        }
        return m;
    };
    if (!any) {
        body = m_plan.start;
        foot[0] = standsAt[0] = m_startFoot[0];
        foot[1] = standsAt[1] = m_startFoot[1];
    } else if (t < P) {
        crouch = p.crouch * ease(t / P);
        foot[0] = standsAt[0] = m_startFoot[0];
        foot[1] = standsAt[1] = m_startFoot[1];
    } else if (t < walkEnd) {
        const int k = std::min(n - 1, int((t - P) / T));
        const double u = std::clamp((t - P - k * T) / T, 0.0, 1.0);
        const Footstep& f = m_plan.steps[size_t(k)];
        crouch = p.crouch;
        body = lerp(m_plan.body[size_t(k)], m_plan.body[size_t(k) + 1], u);
        // The hips lean over the foot that stands
        lateral = p.sway * std::sin(kPi * u) * (f.side == 1 ? 1.0 : -1.0);
        // A foot on the floor rolls onto its toes as the body passes ahead of
        // it (onto its heel walking backwards), so its ankle stays in range
        auto nextStep = [&](int side, int from) {
            for (int i = from; i < n; ++i) if (m_plan.steps[size_t(i)].side == side) return i;
            return -1;
        };
        auto roll = [&](int side, const QMatrix4x4& planted, int next, const Place& b, double lean) {
            if (next < 0) return 0.0;
            const QMatrix4x4 inv = planted.inverted();
            const double travel = (inv * target(side, m_plan.steps[size_t(next)].at))(0, 3);
            const double ahead = inv.map(QVector3D(float(b.x), float(m_floorY), float(b.z))).x();
            QMatrix4x4 hips = frameOf(b, m_floorY);
            hips.translate(0, float(-crouch), float(-lean));
            const double reach = (hips.map(m_legs.hip[side]) - at(planted)).length() / m_legs.legLength;
            return toeOff(ahead, travel, reach);
        };
        const int stance = 1 - f.side;
        const QMatrix4x4 standing = footBefore(stance, k);
        standsAt[stance] = standing;
        rollFor[stance] = roll(stance, standing, nextStep(stance, k + 1), body, lateral);
        foot[stance] = rolled(standing, rollFor[stance]);
        const QMatrix4x4 planted = footBefore(f.side, k), to = target(f.side, f.at);
        const double v = p.doubleSupport < 1 ? std::clamp((u - p.doubleSupport) / (1 - p.doubleSupport), 0.0, 1.0) : 1.0;
        // It lifts rolled as far as it was when it left the floor
        double liftRoll;
        if (v <= 0) {
            liftRoll = roll(f.side, planted, k, body, lateral);
            standsAt[f.side] = planted;
            rollFor[f.side] = liftRoll;
            liftStep = k;
        } else if (m_rollStep[f.side] == k) {
            liftRoll = m_roll[f.side];
        } else {
            const Place lift = lerp(m_plan.body[size_t(k)], m_plan.body[size_t(k) + 1], p.doubleSupport);
            const double liftLean = p.sway * std::sin(kPi * p.doubleSupport) * (f.side == 1 ? 1.0 : -1.0);
            liftRoll = roll(f.side, planted, k, lift, liftLean);
        }
        if (v > 0) onFloor[f.side] = false;
        const QMatrix4x4 from = rolled(planted, liftRoll);
        const double s = v > 0 ? cycloid(v) : 0.0;
        const kin::Target a = kin::targetFrom(from), b = kin::targetFrom(to);
        QMatrix4x4 m;
        const double up = (1 - std::cos(2 * kPi * v)) / 2;   // 0 → 1 → 0
        m.translate(a.position + (b.position - a.position) * float(s) + QVector3D(0, float(p.stepHeight * up), 0));
        // Off its toes it stays rolled early in the swing (the toes clear the
        // floor); off its heel (backwards) it flattens at once. It lands flat.
        const double turn = liftRoll > 0 ? ease((v - 0.25) / 0.75) : ease(v / 0.5);
        m.rotate(QQuaternion::slerp(a.orientation, b.orientation, float(turn)));
        // and its toes point down as it lifts, by no more than keeps them off the floor
        m.rotate(float(-qRadiansToDegrees(m_swingPitch * up)), 0, 0, 1);
        if (v > 0 && v < 1) {
            swingSide = f.side;
            swingV = v;
            swingUp = up;
            swingFrame = m;
        }
        foot[f.side] = v > 0 ? clearFloor(m, up) : from;
        // Arms swing against the legs: the right arm forward with the left foot
        const double env = std::min({1.0, k == 0 ? u : 1.0, k == n - 1 ? 1 - u : 1.0});
        arm = env * -std::cos(kPi * u) * (f.side == 0 ? 1.0 : -1.0);
    } else {
        crouch = p.crouch * (1 - ease((t - walkEnd) / P));
        body = m_plan.end();
        foot[0] = standsAt[0] = footBefore(0, n);
        foot[1] = standsAt[1] = footBefore(1, n);
    }

    // The root, sunk and leaning; the robot's left is its −Z
    QMatrix4x4 shift;
    shift.translate(0, float(-crouch), float(-lateral));
    scene.setWorldTransform(m_legs.root, frameOf(body, m_floorY) * shift * m_rootOffset);

    double worst = 0;
    const double dt = m_lastT >= 0 && t > m_lastT ? t - m_lastT : 0.0;
    m_lastT = t;
    for (int side = 0; side < 2; ++side) {
        auto solve = [&](const QMatrix4x4& f) {
            kin::Target tg = kin::targetFrom(f);
            tg.useOrientation = kin::canOrient(m_legs.chain[side]);
            return kin::solve(scene, m_legs.foot[side], m_legs.chain[side], tg, 40);
        };
        auto good = [](const kin::Result& r) { return r.positionError < 1e-4f && r.angleError < 0.1f; };
        kin::Result r;
        std::vector<float> saved;
        for (int j : m_legs.chain[side]) saved.push_back(scene.getJoint(j)->currentValue);
        auto restore = [&] {
            for (size_t i = 0; i < saved.size(); ++i) scene.getJoint(m_legs.chain[side][i])->currentValue = saved[i];
            scene.applyCouplings();
        };
        // The nearest value to `start` (within ±`most`, in 2.5° steps) for which
        // `frameFor` is reached; the best one when none is
        auto nearest = [&](double start, double most, const std::function<QMatrix4x4(double)>& frameFor) {
            r = solve(frameFor(start));
            if (good(r)) return start;
            kin::Result best = r;
            double bestValue = start;
            const double stepA = qDegreesToRadians(2.5);
            for (int i = 1; i <= 40; ++i) {
                const double b = start + (i % 2 ? 1 : -1) * stepA * ((i + 1) / 2);
                if (std::fabs(b) > most + 1e-9) continue;
                restore();
                const kin::Result x = solve(frameFor(b));
                if (good(x)) { best = x; bestValue = b; break; }
                if (x.positionError + x.angleError * 1e-3 < best.positionError + best.angleError * 1e-3) {
                    best = x;
                    bestValue = b;
                }
            }
            restore();
            r = solve(frameFor(bestValue));
            return bestValue;
        };
        if (onFloor[side]) {
            m_pitchBias[side] = 0;
            // Rolled on its toes or heel (0 = flat): the pivot stays on the floor
            const double used = nearest(rollFor[side], qDegreesToRadians(25.0),
                                        [&](double a) { return rolled(standsAt[side], a); });
            foot[side] = rolled(standsAt[side], used);
            m_roll[side] = used;
            m_rollStep[side] = side == swingSide || (liftStep >= 0 && m_plan.steps[size_t(liftStep)].side == side) ? liftStep : -1;
        } else if (side != swingSide) {
            m_pitchBias[side] = 0;
            r = solve(foot[side]);
        } else {
            // A foot in the air may point its toes further down (or up) when its
            // leg can't reach the planned pitch: the nearest pitch that works,
            // searched from the last scan's, easing back and gone by landing
            const double most = qDegreesToRadians(40.0);
            const double cap = swingV > 0.85 ? most * (1 - swingV) / 0.15 : most;
            auto pitched = [&](double bias) {
                QMatrix4x4 m = swingFrame;
                m.rotate(float(-qRadiansToDegrees(bias)), 0, 0, 1);
                return clearFloor(m, swingUp);
            };
            const double last = std::clamp(m_pitchBias[side], -cap, cap);
            const double eased = last - std::clamp(last, -qDegreesToRadians(90.0) * dt, qDegreesToRadians(90.0) * dt);
            r = solve(pitched(eased));
            double bias = eased;
            if (!good(r)) {
                restore();
                bias = nearest(last, cap, pitched);
            }
            m_pitchBias[side] = bias;
            foot[side] = pitched(bias);
        }
        worst = std::max(worst, double(r.positionError));
    }
    // Standing at the end: the legs go straight when that puts the feet in place
    if (t >= duration() && crouch < 1e-9) {
        std::map<int, float> zero;
        for (int side = 0; side < 2; ++side) for (int j : m_legs.chain[side]) zero[j] = 0;
        bool home = true;
        for (int side = 0; side < 2 && home; ++side) {
            const QMatrix4x4 f = kin::toolFrameAt(scene, m_legs.foot[side], zero);
            if ((at(f) - at(foot[side])).length() > 0.002f) home = false;
        }
        if (home) {
            for (auto& [j, v] : zero) scene.getJoint(j)->currentValue = 0;
            scene.applyCouplings();
            worst = 0;
        }
    }
    if (joints) {
        for (int side = 0; side < 2; ++side)
            for (int j : m_legs.chain[side]) (*joints)[j] = scene.getJoint(j)->currentValue;
        if (p.armSwing > 0) {
            const double deg = qRadiansToDegrees(p.armSwing);
            // Positive pitch raises an arm forward on both sides
            for (int side = 0; side < 2; ++side) {
                if (m_legs.arm[side] == -1) continue;
                const double v = m_startArm[side] + deg * arm * (side == 1 ? 1.0 : -1.0);
                (*joints)[m_legs.arm[side]] = v;
                if (Joint* j = scene.getJoint(m_legs.arm[side])) j->currentValue = float(v);
            }
            scene.applyCouplings();
        }
    }
    if (error) *error = worst;
    return worst <= kReachTol;
}

double Walker::toeOff(double ahead, double travel, double reach) const {
    // Walking forward, the foot behind rolls onto its toes from a quarter of
    // a step behind the body, up to 25° at most of a step: the heel lifts as
    // the shin leans forward over it. Walking backwards, the foot in front
    // rolls onto its heel only as the leg nears its full length (the knee
    // bends forward, so lifting the toes earlier would bend the ankle more).
    const double s = m_plan.params.stepLength, start = 0.25 * s, full = 0.7 * s;
    if (travel > 1e-6 && ahead > 0)
        return qDegreesToRadians(25.0) * std::clamp((ahead - start) / (full - start), 0.0, 1.0);
    if (travel < -1e-6 && ahead < 0) {
        // Past what the leg reaches standing over the foot, crouched
        const double standing = 1 - m_plan.params.crouch / m_legs.legLength;
        const double from = std::max(0.985, standing + 0.005);
        return -qDegreesToRadians(25.0) * std::clamp((reach - from) / 0.01, 0.0, 1.0);
    }
    return 0;
}

QMatrix4x4 Walker::rolled(const QMatrix4x4& foot, double angle) const {
    if (std::fabs(angle) < 1e-9) return foot;
    // About the toe line when rolling forward, the heel line backwards; the
    // foot pitches about its own Z (positive lifts the toes)
    const float half = float(m_legs.footLength / 2);
    const QVector3D pivot = m_legs.soleCentre + QVector3D(angle > 0 ? half : -half, 0, 0);
    QMatrix4x4 m = foot;
    m.translate(pivot);
    m.rotate(float(qRadiansToDegrees(-angle)), 0, 0, 1);
    m.translate(-pivot);
    return m;
}

std::vector<Walker::Print> Walker::prints(double t) const {
    std::vector<Print> out;
    const double T = m_plan.params.stepTime, P = m_plan.prep;
    QMatrix4x4 sole;
    sole.translate(m_legs.soleCentre);
    for (int k = 0; k < int(m_plan.steps.size()); ++k) {
        const Footstep& f = m_plan.steps[size_t(k)];
        Print pr;
        pr.frame = target(f.side, f.at) * sole;
        pr.side = f.side;
        pr.done = t >= P + (k + 1) * T - 1e-9;
        out.push_back(pr);
    }
    return out;
}

} // namespace gait
