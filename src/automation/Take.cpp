#include "Take.h"
#include "AutomationEngine.h"
#include "Kinematics.h"
#include "../robot/Gait.h"
#include "../scene/SceneGraph.h"
#include <QFile>
#include <QJsonArray>
#include <QSaveFile>
#include <QTextStream>
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

bool Take::load(const QString& path, Take* out, QString* error) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (error) *error = f.errorString();
        return false;
    }
    return fromCsv(QString::fromUtf8(f.readAll()), out, error);
}

bool Take::fromCsv(const QString& text, Take* out, QString* error) {
    auto failed = [&](const QString& why) {
        if (error) *error = why;
        return false;
    };
    Take t;
    int line = 0;
    for (QStringView raw : QStringView(text).split(u'\n')) {
        ++line;
        const QStringView s = raw.trimmed();
        if (s.isEmpty()) continue;
        if (s.startsWith(u'#')) {
            const QStringView kv = s.mid(1).trimmed();
            const qsizetype colon = kv.indexOf(u':');
            if (colon < 0) continue;
            const QString key = kv.left(colon).trimmed().toString().toLower();
            const QString value = kv.mid(colon + 1).trimmed().toString();
            if (key == "robot") t.robot = value;
            else if (key == "rate") t.rate = value.toDouble();
            continue;
        }
        const QList<QStringView> cells = s.split(u',');
        if (t.columns.isEmpty()) {
            for (QStringView c : cells) t.columns << c.trimmed().toString();
            continue;
        }
        if (cells.size() != t.columns.size())
            return failed(QString("line %1 has %2 values for %3 columns").arg(line).arg(cells.size()).arg(t.columns.size()));
        std::vector<double> row;
        row.reserve(size_t(cells.size()));
        for (QStringView c : cells) {
            bool ok = false;
            const double v = c.trimmed().toDouble(&ok);
            if (!ok || !std::isfinite(v)) return failed(QString("line %1: \"%2\" is not a number").arg(line).arg(c.trimmed()));
            row.push_back(v);
        }
        t.rows.push_back(std::move(row));
    }
    if (t.columns.isEmpty()) return failed("there is no column row");
    if (t.column("t") != 0) return failed("the first column must be \"t\" (seconds)");
    if (t.rows.size() < 2) return failed("a take needs at least two rows");
    for (size_t i = 1; i < t.rows.size(); ++i)
        if (!(t.rows[i][0] > t.rows[i - 1][0])) return failed(QString("the times must increase (row %1)").arg(i + 1));
    *out = std::move(t);
    return true;
}

QJsonObject Take::toJson() const {
    if (m_packed.isEmpty() && !rows.empty()) {
        QByteArray raw;
        raw.reserve(int(rows.size() * size_t(columns.size()) * sizeof(double)));
        for (const std::vector<double>& row : rows)
            raw.append(reinterpret_cast<const char*>(row.data()), qsizetype(row.size() * sizeof(double)));
        m_packed = QString::fromLatin1(qCompress(raw).toBase64());
    }
    return QJsonObject{{"robot", robot}, {"rate", rate}, {"columns", QJsonArray::fromStringList(columns)},
                       {"rows", int(rows.size())}, {"data", m_packed}};
}

Take Take::fromJson(const QJsonObject& o) {
    Take t;
    t.robot = o["robot"].toString();
    t.rate = o["rate"].toDouble();
    for (const QJsonValue& v : o["columns"].toArray()) t.columns << v.toString();
    const int n = o["rows"].toInt(), cols = int(t.columns.size());
    const QString packed = o["data"].toString();
    const QByteArray raw = qUncompress(QByteArray::fromBase64(packed.toLatin1()));
    if (n <= 0 || cols <= 0 || raw.size() != qsizetype(n) * cols * qsizetype(sizeof(double))) return t;
    t.rows.resize(size_t(n));
    const double* d = reinterpret_cast<const double*>(raw.constData());
    for (int i = 0; i < n; ++i) t.rows[size_t(i)].assign(d + size_t(i) * size_t(cols), d + size_t(i + 1) * size_t(cols));
    t.m_packed = packed;
    return t;
}

bool Take::isMetaColumn(const QString& c) {
    static const QStringList meta{"t", "x", "y", "z", "rx", "ry", "rz", "heading", "walking", "step", "swing",
                                  "progress", "step_x", "step_z", "step_heading", "goal_x", "goal_z", "goal_heading"};
    return meta.contains(c);
}

QStringList Take::jointColumns() const {
    QStringList out;
    for (const QString& c : columns)
        if (!isMetaColumn(c)) out << c;
    return out;
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
                       << "step_x" << "step_z" << "step_heading" << "goal_x" << "goal_z" << "goal_heading";
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
        gait::Place goal = at, target = at;
        gait::Walker::Phase ph;
        bool walking = false;
        for (const AutomationEngine::WalkView& v : a.walks())
            if (v.root == m_root) { walking = true; ph = v.phase; goal = v.goal; target = v.target; break; }
        row.insert(row.end(), {qRadiansToDegrees(at.heading), walking ? 1.0 : 0.0, double(ph.step), double(ph.side),
                               ph.progress, target.x * 1000.0, target.z * 1000.0, qRadiansToDegrees(target.heading),
                               goal.x * 1000.0, goal.z * 1000.0, qRadiansToDegrees(goal.heading)});
    }
    for (int jid : m_joints) {
        const Joint* j = m_scene.getJoint(jid);
        const double v = j ? j->currentValue : 0.0;
        row.push_back(j && j->type == JointType::Prismatic ? v * 1000.0 : v);
    }
    m_take.rows.push_back(std::move(row));
}

} // namespace take
