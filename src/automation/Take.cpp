#include "Take.h"
#include "AutomationEngine.h"
#include "Kinematics.h"
#include "../robot/Gait.h"
#include "../scene/SceneGraph.h"
#include <QFile>
#include <QSaveFile>
#include <QtMath>
#include <cmath>

namespace take {

namespace {
QString num(double v) {
    const double r = std::round(v * 1000.0) / 1000.0;
    return QString::number(r == 0 ? 0.0 : r, 'g', 12);   // no "-0"
}
} // namespace

QString Take::toCsv() const {
    QString s;
    s.reserve(int(rows.size() * size_t(columns.size()) * 8 + 200));
    s += "# MotionStudio take\n";
    s += "# robot: " + robot + "\n";
    s += "# rate: " + num(rate) + "\n";
    s += "# units: mm, degrees, seconds\n";
    s += columns.join(',') + '\n';
    for (const std::vector<double>& row : rows) {
        for (size_t i = 0; i < row.size(); ++i) {
            if (i) s += ',';
            s += num(row[i]);
        }
        s += '\n';
    }
    return s;
}

bool Take::save(const QString& path, QString* error) const {
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text) || f.write(toCsv().toUtf8()) < 0 || !f.commit()) {
        if (error) *error = f.errorString();
        return false;
    }
    return true;
}

Recorder::Recorder(SceneGraph& scene, int root, std::vector<int> joints, double rate)
    : m_scene(scene), m_root(root), m_joints(std::move(joints))
{
    m_legs = gait::canWalk(scene, root);
    m_take.robot = scene.getObject(root) ? scene.getObject(root)->name : QString();
    m_take.rate = rate > 0 ? rate : 0;
    m_take.columns = QStringList{"t", "x", "y", "z", "rx", "ry", "rz"};
    if (m_legs)
        m_take.columns << "heading" << "walking" << "step" << "swing" << "progress"
                       << "goal_x" << "goal_z" << "goal_heading";
    for (int jid : m_joints) {
        const Motor* m = scene.motorForJoint(jid);
        const Joint* j = scene.getJoint(jid);
        m_take.columns << (m ? m->name : j ? j->name : QString("joint%1").arg(jid));
    }
}

void Recorder::sample(const AutomationEngine& a) {
    if (!m_scene.getObject(m_root)) return;
    const double now = a.time();
    if (m_start < 0) {
        m_start = now;
        m_next = 0;
        if (m_take.rate <= 0 && a.dt() > 0) m_take.rate = 1.0 / a.dt();
    }
    const double t = now - m_start;
    if (t + 1e-9 < m_next) return;
    if (m_take.rate > 0) m_next += 1.0 / m_take.rate;
    while (m_take.rate > 0 && m_next <= t + 1e-9) m_next += 1.0 / m_take.rate;   // after a pause in the scans

    std::vector<double> row;
    row.reserve(size_t(m_take.columns.size()));
    row.push_back(t);
    const QMatrix4x4 frame = kin::toolFrame(m_scene, m_root);
    const Transform tf = SceneGraph::toTransform(frame);
    row.insert(row.end(), {tf.position.x() * 1000.0, tf.position.y() * 1000.0, tf.position.z() * 1000.0,
                           tf.rotation.x(), tf.rotation.y(), tf.rotation.z()});
    if (m_legs) {
        const gait::Place at = gait::placeOf(frame);
        gait::Place goal = at;
        gait::Walker::Phase ph;
        bool walking = false;
        for (const AutomationEngine::WalkView& v : a.walks())
            if (v.root == m_root) { walking = true; ph = v.phase; goal = v.goal; break; }
        row.insert(row.end(), {qRadiansToDegrees(at.heading), walking ? 1.0 : 0.0, double(ph.step), double(ph.side),
                               ph.progress, goal.x * 1000.0, goal.z * 1000.0, qRadiansToDegrees(goal.heading)});
    }
    for (int jid : m_joints) {
        const Joint* j = m_scene.getJoint(jid);
        const double v = j ? j->currentValue : 0.0;
        row.push_back(j && j->type == JointType::Prismatic ? v * 1000.0 : v);
    }
    m_take.rows.push_back(std::move(row));
}

} // namespace take
