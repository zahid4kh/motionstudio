#include "Kinematics.h"
#include "../scene/SceneGraph.h"
#include "../scene/SceneQuery.h"
#include <QtMath>
#include <algorithm>
#include <array>
#include <cmath>

namespace kin {
namespace {

constexpr double kDeg = M_PI / 180.0;

const Joint* jointAbove(const SceneGraph& scene, int childId) {
    for (auto& [id, j] : scene.allJoints())
        if (j.childBodyId == childId) return &j;
    return nullptr;
}

// Revolute joints work in radians inside the solver so that metres and
// radians are of similar size for an arm about a metre long
double toSolver(const Joint& j, double v) { return j.type == JointType::Revolute ? v * kDeg : v; }
double fromSolver(const Joint& j, double v) { return j.type == JointType::Revolute ? v / kDeg : v; }

bool limited(const Joint& j) { return !(j.type == JointType::Revolute && j.continuous); }

double clampToLimits(const Joint& j, double v) {
    if (!limited(j)) return v;
    return std::clamp(v, double(std::min(j.limitMin, j.limitMax)), double(std::max(j.limitMin, j.limitMax)));
}

// 6D error (position in m, orientation as rotation vector in rad) from the
// current tool frame to the target
void frameError(const QMatrix4x4& f, const Target& t, double e[6]) {
    const QVector3D dp = t.position - f.column(3).toVector3D();
    e[0] = dp.x(); e[1] = dp.y(); e[2] = dp.z();
    e[3] = e[4] = e[5] = 0;
    if (!t.useOrientation) return;
    const QQuaternion cur = QQuaternion::fromRotationMatrix(f.toGenericMatrix<3, 3>()).normalized();
    QQuaternion d = (t.orientation * cur.conjugated()).normalized();
    if (d.scalar() < 0) d = -d;
    // Rotation vector (axis × angle); getAxisAndAngle has no axis at identity
    const double vx = d.x(), vy = d.y(), vz = d.z();
    const double sinHalf = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (sinHalf < 1e-12) return;
    const double k = 2.0 * std::atan2(sinHalf, double(d.scalar())) / sinHalf;
    e[3] = vx * k; e[4] = vy * k; e[5] = vz * k;
}

// Solves (A + λ²I) x = b for a symmetric m×m system (m ≤ 6), Gaussian
// elimination with partial pivoting
bool solveLinear(int m, double A[6][7]) {
    for (int c = 0; c < m; ++c) {
        int p = c;
        for (int r = c + 1; r < m; ++r) if (std::fabs(A[r][c]) > std::fabs(A[p][c])) p = r;
        if (std::fabs(A[p][c]) < 1e-14) return false;
        if (p != c) for (int k = 0; k <= m; ++k) std::swap(A[p][k], A[c][k]);
        for (int r = 0; r < m; ++r) {
            if (r == c) continue;
            const double f = A[r][c] / A[c][c];
            for (int k = c; k <= m; ++k) A[r][k] -= f * A[c][k];
        }
    }
    for (int r = 0; r < m; ++r) A[r][m] /= A[r][r];
    return true;
}

} // namespace

std::vector<int> chainTo(const SceneGraph& scene, int toolId, bool actuatedOnly) {
    std::vector<int> chain;
    for (const SceneObject* o = scene.getObject(toolId); o; o = scene.getObject(o->parentId)) {
        const Joint* j = jointAbove(scene, o->id);
        if (j && (!actuatedOnly || scene.motorForJoint(j->id))) chain.push_back(j->id);
    }
    std::reverse(chain.begin(), chain.end());
    return chain;
}

QMatrix4x4 toolFrame(const SceneGraph& scene, int toolId) {
    return scenequery::stripScale(scene.worldTransform(toolId));
}

QMatrix4x4 toolFrameAt(SceneGraph& scene, int toolId, const std::map<int, float>& jointValues) {
    std::map<int, float> saved;   // every joint: couplings move followers too
    for (auto& [jid, j] : scene.allJoints()) saved[jid] = j.currentValue;
    for (auto& [jid, v] : jointValues)
        if (Joint* j = scene.getJoint(jid)) j->currentValue = v;
    scene.applyCouplings();
    const QMatrix4x4 f = toolFrame(scene, toolId);
    for (auto& [jid, v] : saved) scene.getJoint(jid)->currentValue = v;
    return f;
}

Target targetFrom(const QMatrix4x4& frame) {
    Target t;
    t.position = frame.column(3).toVector3D();
    t.orientation = QQuaternion::fromRotationMatrix(frame.toGenericMatrix<3, 3>()).normalized();
    return t;
}

Result solve(SceneGraph& scene, int toolId, const std::vector<int>& chain, const Target& target,
             int maxIterations) {
    Result res;
    std::vector<Joint*> joints;
    for (int jid : chain)
        if (Joint* j = scene.getJoint(jid)) joints.push_back(j);
    const int n = int(joints.size());
    const int m = target.useOrientation ? 6 : 3;
    // Orientation errors weigh like this many metres per radian
    const double rotWeight = 0.3;
    const double posTol = 1e-5, angTol = 1e-4;   // m, rad

    auto measure = [&](double e[6]) {
        scene.applyCouplings();   // geared joints move with the chain
        frameError(toolFrame(scene, toolId), target, e);
        for (int k = 3; k < 6; ++k) e[k] *= rotWeight;
    };
    auto errorNorm = [&](const double e[6]) {
        double s = 0;
        for (int k = 0; k < m; ++k) s += e[k] * e[k];
        return std::sqrt(s);
    };

    double e[6];
    measure(e);
    std::vector<double> best(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) best[size_t(i)] = joints[size_t(i)]->currentValue;
    double bestNorm = errorNorm(e);
    double lambda = 0.05;

    for (int it = 0; it < maxIterations && n > 0; ++it) {
        res.iterations = it + 1;
        const double posErr = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
        const double angErr = std::sqrt(e[3] * e[3] + e[4] * e[4] + e[5] * e[5]) / rotWeight;
        if (posErr < posTol && (!target.useOrientation || angErr < angTol)) break;

        // Numerical Jacobian, one column per joint
        std::vector<std::array<double, 6>> J(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
            Joint* j = joints[size_t(i)];
            const float saved = j->currentValue;
            const double h = 1e-4;   // rad or m
            j->currentValue = float(fromSolver(*j, toSolver(*j, saved) + h));
            double e2[6];
            measure(e2);
            j->currentValue = saved;
            // e = target − f, so ∂f/∂θ = −∂e/∂θ
            for (int k = 0; k < 6; ++k) J[size_t(i)][size_t(k)] = -(e2[k] - e[k]) / h;
        }

        // Δθ = Jᵀ (J Jᵀ + λ² I)⁻¹ e
        double A[6][7] = {};
        for (int r = 0; r < m; ++r) {
            for (int c = 0; c < m; ++c) {
                double s = 0;
                for (int i = 0; i < n; ++i) s += J[size_t(i)][size_t(r)] * J[size_t(i)][size_t(c)];
                A[r][c] = s + (r == c ? lambda * lambda : 0.0);
            }
            A[r][m] = e[r];
        }
        if (!solveLinear(m, A)) break;
        std::vector<double> step(static_cast<size_t>(n), 0.0);
        double maxStep = 0;
        for (int i = 0; i < n; ++i) {
            for (int r = 0; r < m; ++r) step[size_t(i)] += J[size_t(i)][size_t(r)] * A[r][m];
            maxStep = std::max(maxStep, std::fabs(step[size_t(i)]));
        }
        // Keep steps small enough for the linearisation to hold
        const double scale = maxStep > 0.2 ? 0.2 / maxStep : 1.0;
        std::vector<float> before(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
            Joint* j = joints[size_t(i)];
            before[size_t(i)] = j->currentValue;
            j->currentValue = float(clampToLimits(*j, fromSolver(*j, toSolver(*j, j->currentValue) + step[size_t(i)] * scale)));
        }
        double e2[6];
        measure(e2);
        const double norm = errorNorm(e2);
        if (norm < bestNorm) {
            bestNorm = norm;
            for (int i = 0; i < n; ++i) best[size_t(i)] = joints[size_t(i)]->currentValue;
            std::copy(e2, e2 + 6, e);
            lambda = std::max(1e-3, lambda * 0.5);
        } else {
            // Worse: undo and damp harder (Levenberg–Marquardt style)
            for (int i = 0; i < n; ++i) joints[size_t(i)]->currentValue = before[size_t(i)];
            lambda *= 4;
            if (lambda > 10) break;
        }
    }

    for (int i = 0; i < n; ++i) joints[size_t(i)]->currentValue = float(best[size_t(i)]);
    scene.applyCouplings();
    double ef[6];
    frameError(toolFrame(scene, toolId), target, ef);
    res.positionError = float(std::sqrt(ef[0] * ef[0] + ef[1] * ef[1] + ef[2] * ef[2]));
    res.angleError = target.useOrientation
        ? float(std::sqrt(ef[3] * ef[3] + ef[4] * ef[4] + ef[5] * ef[5]) / kDeg) : 0.0f;
    res.reached = res.positionError < 1e-4f && (!target.useOrientation || res.angleError < 0.05f);
    return res;
}

void teachOnPart(const SceneGraph& scene, Pose* pose, int toolId, int partId) {
    const QMatrix4x4 rel = toolFrame(scene, partId).inverted() * toolFrame(scene, toolId);
    pose->toolId = toolId;
    pose->frameId = partId;
    pose->relPosition = rel.column(3).toVector3D();
    pose->relRotation = QQuaternion::fromRotationMatrix(rel.toGenericMatrix<3, 3>()).normalized();
    pose->jointValues.clear();
    for (int jid : chainTo(scene, toolId))
        if (const Joint* j = scene.getJoint(jid)) pose->jointValues[jid] = j->currentValue;
}

QMatrix4x4 poseTarget(const SceneGraph& scene, const Pose& pose, int partId) {
    return toolFrame(scene, partId) * pose.relative();
}

Result solveFor(SceneGraph& scene, const Pose& pose, int toolId, const QMatrix4x4& target,
                std::map<int, float>* values) {
    std::map<int, float> saved;   // every joint: couplings move followers too
    for (auto& [jid, j] : scene.allJoints()) saved[jid] = j.currentValue;
    const std::vector<int> chain = chainTo(scene, toolId);
    for (int jid : chain) {
        auto it = pose.jointValues.find(jid);
        if (it != pose.jointValues.end()) scene.getJoint(jid)->currentValue = it->second;
    }
    Target t = targetFrom(target);
    t.useOrientation = canOrient(chain);
    const Result r = solve(scene, toolId, chain, t, 200);
    values->clear();
    for (int jid : chain) (*values)[jid] = scene.getJoint(jid)->currentValue;
    for (auto& [jid, v] : saved) scene.getJoint(jid)->currentValue = v;
    scene.applyCouplings();
    return r;
}

} // namespace kin
