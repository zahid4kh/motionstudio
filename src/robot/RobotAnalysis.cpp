#include "RobotAnalysis.h"
#include "../automation/Kinematics.h"
#include "../scene/SceneGraph.h"
#include <QQuaternion>
#include <QtMath>
#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <random>
#include <set>
#include <tuple>

namespace robot {
namespace {

bool isTcpName(const QString& n) { return n.compare("TCP", Qt::CaseInsensitive) == 0 || n.startsWith("TCP", Qt::CaseInsensitive); }

void subtree(const SceneGraph& g, int id, int depth, const std::function<void(int, int)>& f) {
    const SceneObject* o = g.getObject(id);
    if (!o) return;
    f(id, depth);
    for (int c : o->childIds) subtree(g, c, depth + 1, f);
}

// Closest points between two lines (p + t·u, q + s·v); false when parallel
bool closestPoints(QVector3D p, QVector3D u, QVector3D q, QVector3D v, QVector3D& onP, QVector3D& onQ) {
    const double a = QVector3D::dotProduct(u, u), b = QVector3D::dotProduct(u, v), c = QVector3D::dotProduct(v, v);
    const QVector3D w = p - q;
    const double d = QVector3D::dotProduct(u, w), e = QVector3D::dotProduct(v, w);
    const double den = a * c - b * b;
    if (std::fabs(den) < 1e-10) return false;
    const double t = (b * e - c * d) / den, s = (a * e - b * d) / den;
    onP = p + u * float(t);
    onQ = q + v * float(s);
    return true;
}

QVector3D projectOnLine(QVector3D x, QVector3D p, QVector3D u) {
    return p + u * QVector3D::dotProduct(x - p, u);
}

double lineDistance(QVector3D x, QVector3D p, QVector3D u) {
    return (x - projectOnLine(x, p, u)).length();
}

// Eigenvalues of a symmetric m×m matrix (Jacobi rotations)
std::vector<double> eigenvalues(std::vector<std::vector<double>> A) {
    const int m = int(A.size());
    for (int sweep = 0; sweep < 60; ++sweep) {
        double off = 0;
        for (int i = 0; i < m; ++i) for (int j = i + 1; j < m; ++j) off += A[i][j] * A[i][j];
        if (off < 1e-22) break;
        for (int p = 0; p < m; ++p)
            for (int q = p + 1; q < m; ++q) {
                if (std::fabs(A[p][q]) < 1e-300) continue;
                const double theta = (A[q][q] - A[p][p]) / (2 * A[p][q]);
                const double t = (theta >= 0 ? 1 : -1) / (std::fabs(theta) + std::sqrt(theta * theta + 1));
                const double c = 1 / std::sqrt(t * t + 1), s = t * c;
                for (int k = 0; k < m; ++k) {
                    const double akp = A[k][p], akq = A[k][q];
                    A[k][p] = c * akp - s * akq;
                    A[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < m; ++k) {
                    const double apk = A[p][k], aqk = A[q][k];
                    A[p][k] = c * apk - s * aqk;
                    A[q][k] = s * apk + c * aqk;
                }
            }
    }
    std::vector<double> ev(m);
    for (int i = 0; i < m; ++i) ev[i] = std::max(0.0, A[i][i]);
    return ev;
}

QVector3D rotationVector(const QMatrix4x4& from, const QMatrix4x4& to) {
    const QQuaternion a = QQuaternion::fromRotationMatrix(from.toGenericMatrix<3, 3>()).normalized();
    const QQuaternion b = QQuaternion::fromRotationMatrix(to.toGenericMatrix<3, 3>()).normalized();
    QQuaternion d = (b * a.conjugated()).normalized();
    if (d.scalar() < 0) d = -d;
    const QVector3D v = d.vector();
    const double s = v.length();
    if (s < 1e-12) return {};
    return v * float(2.0 * std::atan2(s, double(d.scalar())) / s);
}

void fillAxisLines(const SceneGraph& g, Analysis& a) {
    for (AxisInfo& ax : a.axes) {
        const Joint* j = g.getJoint(ax.jointId);
        if (!j) continue;
        ax.value = j->currentValue;
        const QMatrix4x4 parent = g.worldTransform(j->parentBodyId);
        ax.axis = parent.mapVector(j->axis).normalized();
        ax.point = j->type == JointType::Revolute ? parent.map(j->pivot)
                                                  : g.worldTransform(j->childBodyId).column(3).toVector3D();
    }
}

// Standard DH parameters from the axis lines
void fillDh(const SceneGraph& g, Analysis& a) {
    const int n = int(a.axes.size());
    if (n == 0) return;
    // Where the previous common normal meets this axis; for J1, the origin
    // of the frame J1 turns in (the robot's foot) projected onto it
    const Joint* j1 = g.getJoint(a.axes[0].jointId);
    const QVector3D origin = g.worldTransform(j1->parentBodyId).column(3).toVector3D();
    QVector3D prevFoot = projectOnLine(origin, a.axes[0].point, a.axes[0].axis);
    for (int i = 0; i < n; ++i) {
        AxisInfo& ax = a.axes[i];
        const QVector3D zi = ax.axis;
        if (i == n - 1) {
            const QVector3D foot = projectOnLine(a.tcp, ax.point, zi);
            ax.d = QVector3D::dotProduct(foot - prevFoot, zi);
            ax.a = (a.tcp - foot).length();
            ax.alpha = 0;
            break;
        }
        const AxisInfo& nx = a.axes[i + 1];
        QVector3D fi, fj;
        if (!closestPoints(ax.point, zi, nx.point, nx.axis, fi, fj)) {
            fi = prevFoot;                                   // parallel: keep d = 0
            fj = projectOnLine(fi, nx.point, nx.axis);
        }
        ax.d = QVector3D::dotProduct(fi - prevFoot, zi);
        const QVector3D normal = fj - fi;
        ax.a = normal.length();
        QVector3D x = ax.a > 1e-6 ? normal / float(ax.a) : QVector3D::crossProduct(zi, nx.axis).normalized();
        ax.alpha = x.isNull() ? (QVector3D::dotProduct(zi, nx.axis) < 0 ? 180 : 0)
                              : qRadiansToDegrees(std::atan2(QVector3D::dotProduct(QVector3D::crossProduct(zi, nx.axis), x),
                                                             QVector3D::dotProduct(zi, nx.axis)));
        if (std::fabs(ax.alpha) < 1e-6) ax.alpha = 0;
        prevFoot = fj;
    }
}

} // namespace

double moveTime(double distance, double speed, double accel) {
    distance = std::fabs(distance);
    if (speed <= 0) return 0;
    if (accel <= 0) return distance / speed;
    const double tAcc = speed / accel, dAcc = speed * tAcc;   // both ramps
    if (distance <= dAcc) return 2 * std::sqrt(distance / accel);
    return 2 * tAcc + (distance - dAcc) / speed;
}

int findTool(const SceneGraph& g, int objectId) {
    for (const SceneObject* o = g.getObject(objectId); o; o = g.getObject(o->parentId)) {
        int best = -1;
        size_t bestLen = 0;
        subtree(g, o->id, 0, [&](int id, int) {
            if (!isTcpName(g.getObject(id)->name)) return;
            const size_t len = kin::chainTo(g, id).size();
            if (len > bestLen) { best = id; bestLen = len; }
        });
        if (best != -1) return best;
    }
    // No TCP: the deepest object with the longest chain under the selection
    int best = -1, bestDepth = -1;
    size_t bestLen = 0;
    subtree(g, objectId, 0, [&](int id, int depth) {
        const size_t len = kin::chainTo(g, id).size();
        if (len > bestLen || (len == bestLen && len > 0 && depth > bestDepth)) { best = id; bestLen = len; bestDepth = depth; }
    });
    return best;
}

void analyzePose(SceneGraph& g, Analysis& a) {
    a.warnings.clear();
    if (!a.error.isEmpty()) return;
    fillAxisLines(g, a);
    const QMatrix4x4 tool = kin::toolFrame(g, a.toolId);
    a.tcp = tool.column(3).toVector3D();
    {
        const QVector3D v = a.tcp - a.basePoint;
        a.reachNow = (v - a.baseAxis * QVector3D::dotProduct(v, a.baseAxis)).length();
    }
    fillDh(g, a);

    // Jacobian (central differences); orientation rows scaled to metres
    const int n = int(a.axes.size());
    const bool orient = n >= 6;
    const int m = orient ? 6 : 3;
    const double L = std::max(0.1, a.maxReach * 0.5);
    std::vector<std::vector<double>> J(m, std::vector<double>(n, 0));
    std::map<int, float> saved;
    for (auto& [jid, j] : g.allJoints()) saved[jid] = j.currentValue;
    for (int i = 0; i < n; ++i) {
        Joint* j = g.getJoint(a.axes[i].jointId);
        const double h = j->type == JointType::Revolute ? 0.05 : 1e-4;               // deg or m
        const double unit = j->type == JointType::Revolute ? qDegreesToRadians(h) : h;  // per rad or per m
        const float v0 = saved[j->id];
        j->currentValue = float(v0 + h);
        g.applyCouplings();
        const QMatrix4x4 fp = kin::toolFrame(g, a.toolId);
        j->currentValue = float(v0 - h);
        g.applyCouplings();
        const QMatrix4x4 fm = kin::toolFrame(g, a.toolId);
        j->currentValue = v0;
        g.applyCouplings();
        const QVector3D dp = (fp.column(3) - fm.column(3)).toVector3D() / float(2 * unit);
        J[0][i] = dp.x(); J[1][i] = dp.y(); J[2][i] = dp.z();
        if (orient) {
            const QVector3D dr = rotationVector(fm, fp) / float(2 * unit) * float(L);
            J[3][i] = dr.x(); J[4][i] = dr.y(); J[5][i] = dr.z();
        }
    }
    for (auto& [jid, v] : saved) g.getJoint(jid)->currentValue = v;
    g.applyCouplings();
    std::vector<std::vector<double>> JJ(m, std::vector<double>(m, 0));
    for (int r = 0; r < m; ++r)
        for (int c = 0; c < m; ++c)
            for (int k = 0; k < n; ++k) JJ[r][c] += J[r][k] * J[c][k];
    const std::vector<double> ev = eigenvalues(JJ);
    const double lmax = *std::max_element(ev.begin(), ev.end()), lmin = *std::min_element(ev.begin(), ev.end());
    a.dexterity = lmax > 0 ? std::sqrt(lmin / lmax) : 0;
    double prod = 1;
    for (double e : ev) prod *= std::sqrt(e);
    a.manipulability = prod;

    // Singularities by geometry, then a general one from the Jacobian
    const double tol = std::max(0.002, 0.004 * a.maxReach);
    bool named = false;
    QVector3D wrist = a.tcp;
    bool hasWrist = false;
    if (n >= 6 && !a.axes[3].prismatic && !a.axes[4].prismatic) {
        QVector3D p, q;
        if (closestPoints(a.axes[3].point, a.axes[3].axis, a.axes[4].point, a.axes[4].axis, p, q) && (p - q).length() < tol) {
            wrist = (p + q) / 2;
            hasWrist = true;
        }
    }
    for (int i = 0; i < n; ++i)
        for (int k = i + 1; k < n; ++k) {
            const AxisInfo& A = a.axes[i];
            const AxisInfo& B = a.axes[k];
            if (A.prismatic || B.prismatic) continue;
            if (std::fabs(QVector3D::dotProduct(A.axis, B.axis)) < 0.9995) continue;
            if (lineDistance(B.point, A.point, A.axis) > tol) continue;
            // Consecutive axes that are parallel by design never line up apart from that
            if (n >= 6 && i == 3 && k == 5)
                a.warnings << QString("Wrist singularity: %1 and %2 are in line (%3 ≈ 0°), so turning one undoes the other")
                                  .arg(A.name, B.name, a.axes[4].name);
            else if (i == 0)
                a.warnings << QString("Shoulder singularity: %1 lines up with the base axis %2").arg(B.name, A.name);
            else
                a.warnings << QString("%1 and %2 are in line: turning one undoes the other").arg(A.name, B.name);
            named = true;
        }
    if (hasWrist && lineDistance(wrist, a.axes[0].point, a.axes[0].axis) < tol * 2) {
        a.warnings << QString("Shoulder singularity: the wrist centre is on the %1 axis").arg(a.axes[0].name);
        named = true;
    }
    // Stretched out or folded: two parallel axes in a row and the point they move
    for (int i = 0; i + 1 < n && i < 3; ++i) {
        const AxisInfo& A = a.axes[i];
        const AxisInfo& B = a.axes[i + 1];
        if (A.prismatic || B.prismatic || std::fabs(QVector3D::dotProduct(A.axis, B.axis)) < 0.9995) continue;
        const QVector3D E = hasWrist ? wrist : a.tcp;
        const QVector3D ae = E - A.point, ab = B.point - A.point;
        const double len = (ae - A.axis * QVector3D::dotProduct(ae, A.axis)).length();
        if (len < tol) continue;
        const double off = std::fabs(QVector3D::dotProduct(QVector3D::crossProduct(ae, ab), A.axis)) / len;
        if (off < 0.004 * len) {
            a.warnings << QString("Arm stretched out or folded back at %1: it cannot move along the arm").arg(B.name);
            named = true;
        }
    }
    if (!named && a.dexterity < 0.02)
        a.warnings << "Near a singularity: the tool can hardly move in some direction";

    for (const AxisInfo& ax : a.axes) {
        if (ax.continuous) continue;
        const double range = ax.max - ax.min, margin = std::max(range * 0.01, ax.prismatic ? 0.0005 : 0.5);
        if (ax.value <= ax.min + margin || ax.value >= ax.max - margin)
            a.warnings << QString("%1 is at its %2 limit").arg(ax.name, ax.value <= ax.min + margin ? "lower" : "upper");
    }
}

Analysis analyze(SceneGraph& g, int toolId, int samples) {
    Analysis a;
    a.toolId = toolId;
    const SceneObject* tool = g.getObject(toolId);
    if (!tool) { a.error = "Select a robot (or its TCP) to analyse"; return a; }
    const std::vector<int> chain = kin::chainTo(g, toolId);
    if (chain.empty()) { a.error = QString("%1 has no actuated joints above it").arg(tool->name); return a; }

    for (int jid : chain) {
        const Joint* j = g.getJoint(jid);
        AxisInfo ax;
        ax.jointId = jid;
        const Motor* mo = g.motorForJoint(jid);
        ax.name = mo ? mo->name : j->name;
        ax.prismatic = j->type == JointType::Prismatic;
        ax.continuous = j->type == JointType::Revolute && j->continuous;
        ax.min = ax.continuous ? -180 : std::min(j->limitMin, j->limitMax);
        ax.max = ax.continuous ? 180 : std::max(j->limitMin, j->limitMax);
        ax.speed = mo ? mo->velocity : 0;
        ax.accel = mo ? mo->acceleration : 0;
        ax.moveTime = moveTime(ax.prismatic ? 0.1 : 90, ax.speed, ax.accel);
        a.axes.push_back(ax);
    }
    fillAxisLines(g, a);
    const AxisInfo& j1 = a.axes.front();
    a.verticalBase = !j1.prismatic && std::fabs(QVector3D::dotProduct(j1.axis, QVector3D(0, 1, 0))) > 0.99;
    if (a.verticalBase) {
        a.basePoint = j1.point;
        a.baseAxis = QVector3D(0, 1, 0);
        a.sweep = j1.continuous ? 360 : std::min(360.0, j1.max - j1.min);
    } else {
        int top = toolId;
        for (const SceneObject* o = tool; o && o->parentId != -1; o = g.getObject(o->parentId)) top = o->parentId;
        a.basePoint = g.worldTransform(top).column(3).toVector3D();
        a.sweep = 0;
    }

    // Sample: each axis anywhere in its range, at its limits now and then so
    // the envelope's edges show
    std::map<int, float> saved;
    for (auto& [jid, j] : g.allJoints()) saved[jid] = j.currentValue;
    std::mt19937 rng(20261106);
    std::uniform_real_distribution<double> uni(0, 1);
    std::vector<QVector3D> pts;
    pts.reserve(samples);
    for (int s = 0; s < samples; ++s) {
        for (const AxisInfo& ax : a.axes) {
            const double r = uni(rng);
            double v = ax.min + (ax.max - ax.min) * uni(rng);
            if (!ax.continuous) { if (r < 0.12) v = ax.min; else if (r < 0.24) v = ax.max; }
            g.getJoint(ax.jointId)->currentValue = float(v);
        }
        g.applyCouplings();
        pts.push_back(g.worldTransform(toolId).column(3).toVector3D());
    }
    for (auto& [jid, v] : saved) g.getJoint(jid)->currentValue = v;
    g.applyCouplings();
    a.samples = samples;

    auto radial = [&](const QVector3D& p) {
        const QVector3D v = p - a.basePoint;
        return double((v - a.baseAxis * QVector3D::dotProduct(v, a.baseAxis)).length());
    };
    a.maxReach = 0;
    a.minReach = 1e9;
    a.maxHeight = -1e9;
    a.minHeight = 1e9;
    double xmin = 1e9, xmax = -1e9, zmin = 1e9, zmax = -1e9;
    for (const QVector3D& p : pts) {
        const double r = radial(p);
        a.maxReach = std::max(a.maxReach, r);
        a.minReach = std::min(a.minReach, r);
        a.maxHeight = std::max(a.maxHeight, double(p.y()));
        a.minHeight = std::min(a.minHeight, double(p.y()));
        xmin = std::min(xmin, double(p.x())); xmax = std::max(xmax, double(p.x()));
        zmin = std::min(zmin, double(p.z())); zmax = std::max(zmax, double(p.z()));
    }

    auto makeGrid = [](EnvelopeGrid& e, double x0, double x1, double y0, double y1, int cells) {
        const double span = std::max({x1 - x0, y1 - y0, 1e-3});
        e.cell = span / cells;
        e.x0 = x0 - e.cell;
        e.y0 = y0 - e.cell;
        e.w = int(std::ceil((x1 - x0) / e.cell)) + 3;
        e.h = int(std::ceil((y1 - y0) / e.cell)) + 3;
        e.hit.assign(size_t(e.w) * e.h, 0);
    };
    auto mark = [](EnvelopeGrid& e, double x, double y) {
        const int i = int((x - e.x0) / e.cell), j = int((y - e.y0) / e.cell);
        if (i >= 0 && j >= 0 && i < e.w && j < e.h) e.hit[size_t(j) * e.w + i] = 1;
    };
    // Fills the gaps the sampling left between reached cells: a cell with at
    // least 5 of its 8 neighbours reached is reached too
    auto close = [](EnvelopeGrid& e) {
        for (int pass = 0; pass < 2; ++pass) {
            std::vector<uint8_t> out = e.hit;
            for (int j = 0; j < e.h; ++j)
                for (int i = 0; i < e.w; ++i) {
                    if (e.at(i, j)) continue;
                    int n = 0;
                    for (int dj = -1; dj <= 1; ++dj)
                        for (int di = -1; di <= 1; ++di) n += (di || dj) && e.at(i + di, j + dj);
                    if (n >= 5) out[size_t(j) * e.w + i] = 1;
                }
            e.hit.swap(out);
        }
    };
    const int cells = 72;
    makeGrid(a.side, 0, a.maxReach, a.minHeight, a.maxHeight, cells);
    a.side.x0 = 0;   // r starts at the axis
    for (const QVector3D& p : pts) mark(a.side, radial(p), p.y());
    close(a.side);
    if (a.verticalBase) {
        const double R = a.maxReach;
        makeGrid(a.top, a.basePoint.x() - R, a.basePoint.x() + R, a.basePoint.z() - R, a.basePoint.z() + R, cells);
    } else {
        makeGrid(a.top, xmin, xmax, zmin, zmax, cells);
    }
    for (const QVector3D& p : pts) mark(a.top, p.x(), p.z());
    close(a.top);

    // Volume: the side profile swept by J1, or a voxel count
    if (a.verticalBase) {
        double v = 0;
        const double sweep = qDegreesToRadians(a.sweep);
        for (int j = 0; j < a.side.h; ++j)
            for (int i = 0; i < a.side.w; ++i)
                if (a.side.at(i, j)) v += a.side.cell * a.side.cell * sweep * (a.side.x0 + (i + 0.5) * a.side.cell);
        a.volume = v;
    } else {
        const double span = std::max({xmax - xmin, zmax - zmin, a.maxHeight - a.minHeight, 1e-3});
        const double c = span / 48;
        std::set<std::tuple<int, int, int>> vox;
        for (const QVector3D& p : pts)
            vox.insert({int(std::floor(p.x() / c)), int(std::floor(p.y() / c)), int(std::floor(p.z() / c))});
        a.volume = double(vox.size()) * c * c * c;
    }

    analyzePose(g, a);
    return a;
}

} // namespace robot
