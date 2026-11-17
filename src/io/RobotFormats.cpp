#include "RobotFormats.h"
#include "../scene/SceneGraph.h"
#include "../scene/SceneQuery.h"
#include "../scene/PrimitiveMesh.h"
#include <QDir>
#include <QDomDocument>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QQuaternion>
#include <QRegularExpression>
#include <QTextStream>
#include <QtEndian>
#include <QtMath>
#include <QVector4D>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <set>

namespace robotio {
namespace {

// ── Small helpers ────────────────────────────────────────────────────────────

QString num(double v) { return QString::number(std::abs(v) < 1e-12 ? 0.0 : v, 'g', 9); }
QString vec(const QVector3D& v) { return num(v.x()) + " " + num(v.y()) + " " + num(v.z()); }

std::vector<double> numbers(const QString& s) {
    std::vector<double> out;
    for (const QString& p : s.split(QRegularExpression("[\\s,]+"), Qt::SkipEmptyParts)) out.push_back(p.toDouble());
    return out;
}
QVector3D vec3(const QString& s, const QVector3D& fallback = {}) {
    const auto n = numbers(s);
    return n.size() >= 3 ? QVector3D(float(n[0]), float(n[1]), float(n[2])) : fallback;
}

// Fixed-axis roll, pitch, yaw (radians) as in URDF: R = Rz(yaw) Ry(pitch) Rx(roll)
QMatrix4x4 rpyMatrix(double r, double p, double y) {
    QMatrix4x4 m;
    m.rotate(float(qRadiansToDegrees(y)), 0, 0, 1);
    m.rotate(float(qRadiansToDegrees(p)), 0, 1, 0);
    m.rotate(float(qRadiansToDegrees(r)), 1, 0, 0);
    return m;
}
void toRpy(const QMatrix4x4& m, double& r, double& p, double& y) {
    const double r20 = std::clamp(double(m(2, 0)), -1.0, 1.0);
    p = std::asin(-r20);
    if (std::abs(std::cos(p)) > 1e-6) {
        r = std::atan2(m(2, 1), m(2, 2));
        y = std::atan2(m(1, 0), m(0, 0));
    } else {
        r = 0;
        y = std::atan2(-m(0, 1), m(1, 1));
    }
}
QMatrix4x4 translation(const QVector3D& t) { QMatrix4x4 m; m.translate(t); return m; }

QString sanitize(const QString& s, const QString& fallback) {
    QString n = s;
    n.replace(QRegularExpression("[^A-Za-z0-9_]"), "_");
    if (n.isEmpty()) n = fallback;
    if (n[0].isDigit()) n.prepend('_');
    return n;
}

// Rigid part of an affine matrix and its per-axis scale
void splitScale(const QMatrix4x4& m, QMatrix4x4* rigid, QVector3D* scale) {
    *rigid = scenequery::stripScale(m);
    *scale = QVector3D(m.column(0).toVector3D().length(), m.column(1).toVector3D().length(),
                       m.column(2).toVector3D().length());
}

// Turns a Y-up scene upright in Z-up formats, and back
QMatrix4x4 yUpToZUp() { QMatrix4x4 m; m.rotate(90, 1, 0, 0); return m; }

// ── Meshes ───────────────────────────────────────────────────────────────────

void finishMesh(MeshGeometry& g) {
    QVector3D mn(1e30f, 1e30f, 1e30f), mx = -mn;
    for (size_t i = 0; i + 5 < g.interleaved.size(); i += 6)
        for (int a = 0; a < 3; ++a) {
            mn[a] = std::min(mn[a], g.interleaved[i + size_t(a)]);
            mx[a] = std::max(mx[a], g.interleaved[i + size_t(a)]);
        }
    g.aabbMin = mn;
    g.aabbMax = mx;
}

void addTriangle(std::vector<float>& soup, const QVector3D& a, const QVector3D& b, const QVector3D& c) {
    QVector3D n = QVector3D::crossProduct(b - a, c - a);
    if (n.lengthSquared() < 1e-30f) return;
    n.normalize();
    for (const QVector3D& v : {a, b, c}) soup.insert(soup.end(), {v.x(), v.y(), v.z(), n.x(), n.y(), n.z()});
}

} // namespace

std::shared_ptr<MeshGeometry> loadMesh(const QString& path, QString* error) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) { if (error) *error = "can't open " + path; return nullptr; }
    const QByteArray data = f.readAll();
    auto g = std::make_shared<MeshGeometry>();
    const QString ext = QFileInfo(path).suffix().toLower();
    if (ext == "stl") {
        quint32 count = data.size() >= 84 ? qFromLittleEndian<quint32>(data.constData() + 80) : 0;
        if (data.size() >= 84 && qint64(data.size()) == 84 + qint64(count) * 50) {
            for (quint32 t = 0; t < count; ++t) {
                const char* p = data.constData() + 84 + t * 50 + 12;
                float v[9];
                std::memcpy(v, p, sizeof v);
                addTriangle(g->interleaved, {v[0], v[1], v[2]}, {v[3], v[4], v[5]}, {v[6], v[7], v[8]});
            }
        } else {
            std::vector<QVector3D> verts;
            QTextStream in(data);
            while (!in.atEnd()) {
                const QString line = in.readLine().trimmed();
                if (!line.startsWith("vertex")) continue;
                verts.push_back(vec3(line.mid(6)));
                if (verts.size() == 3) { addTriangle(g->interleaved, verts[0], verts[1], verts[2]); verts.clear(); }
            }
        }
    } else if (ext == "obj") {
        std::vector<QVector3D> pts;
        QTextStream in(data);
        while (!in.atEnd()) {
            const QString line = in.readLine().trimmed();
            if (line.startsWith("v ")) pts.push_back(vec3(line.mid(2)));
            else if (line.startsWith("f ")) {
                std::vector<int> idx;
                for (const QString& tok : line.mid(2).split(' ', Qt::SkipEmptyParts)) {
                    int i = tok.section('/', 0, 0).toInt();
                    idx.push_back(i < 0 ? int(pts.size()) + i : i - 1);
                }
                for (size_t k = 1; k + 1 < idx.size(); ++k)
                    if (idx[0] >= 0 && idx[k] >= 0 && idx[k + 1] >= 0 && size_t(std::max({idx[0], idx[k], idx[k + 1]})) < pts.size())
                        addTriangle(g->interleaved, pts[size_t(idx[0])], pts[size_t(idx[k])], pts[size_t(idx[k + 1])]);
            }
        }
    } else {
        if (error) *error = QString("%1 meshes are not supported (use STL or OBJ)").arg(ext.toUpper());
        return nullptr;
    }
    if (g->interleaved.empty()) { if (error) *error = "no triangles in " + path; return nullptr; }
    finishMesh(*g);
    return g;
}

bool writeStl(const std::vector<float>& soup, const QString& path) {
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly)) return false;
    QByteArray header(80, ' ');
    header.replace(0, 22, "MotionStudio export   ");
    f.write(header);
    const quint32 count = quint32(soup.size() / 18);
    char buf[4];
    qToLittleEndian(count, buf);
    f.write(buf, 4);
    for (quint32 t = 0; t < count; ++t) {
        const float* v = &soup[size_t(t) * 18];
        const QVector3D a(v[0], v[1], v[2]), b(v[6], v[7], v[8]), c(v[12], v[13], v[14]);
        const QVector3D n = QVector3D::crossProduct(b - a, c - a).normalized();
        float rec[12] = {n.x(), n.y(), n.z(), a.x(), a.y(), a.z(), b.x(), b.y(), b.z(), c.x(), c.y(), c.z()};
        f.write(reinterpret_cast<const char*>(rec), sizeof rec);
        f.write("\0\0", 2);
    }
    return true;
}

namespace {

// ── Import helpers ───────────────────────────────────────────────────────────

struct Importer {
    SceneGraph& scene;
    QString     dir;
    Result      res;
    QHash<QString, std::shared_ptr<MeshGeometry>> meshes;

    // package://pkg/x, file:///x or a relative path, looked up in the
    // file's folder and the folders above it
    QString resolve(const QString& uri) {
        QString p = uri;
        if (p.startsWith("file://")) p = p.mid(7);
        QStringList candidates;
        if (p.startsWith("package://")) {
            const QString rest = p.mid(10);
            const QString pkg = rest.section('/', 0, 0), inner = rest.section('/', 1);
            QDir d(dir);
            for (int up = 0; up < 6; ++up) {
                candidates << d.filePath(pkg + "/" + inner) << d.filePath(inner);
                if (QDir(d).dirName() == pkg) candidates << d.filePath(inner);
                if (!d.cdUp()) break;
            }
        } else {
            candidates << (QFileInfo(p).isAbsolute() ? p : QDir(dir).filePath(p));
        }
        for (const QString& c : candidates)
            if (QFileInfo::exists(c)) return QFileInfo(c).canonicalFilePath();
        return {};
    }

    std::shared_ptr<MeshGeometry> mesh(const QString& uri) {
        const QString path = resolve(uri);
        if (path.isEmpty()) { res.warnings << "Mesh not found: " + uri; return nullptr; }
        auto it = meshes.find(path);
        if (it != meshes.end()) return it.value();
        QString err;
        auto g = loadMesh(path, &err);
        if (!g) res.warnings << err;
        meshes.insert(path, g);
        return g;
    }

    // A geometry object under `parent`: `frame` places the shape's own frame
    // in the parent (rigid); `size` scales the unit primitive
    int addShape(const QString& name, PrimitiveType type, int parent, const QMatrix4x4& frame,
                 const QVector3D& size, std::shared_ptr<MeshGeometry> m, const QColor& color) {
        const int id = scene.addObject(name, type, parent);
        SceneObject* o = scene.getObject(id);
        Transform t = SceneGraph::toTransform(frame);
        t.scale = size;
        o->transform = t;
        o->mesh = std::move(m);
        o->color = color;
        return id;
    }

    int frame(const QString& name, int parent, const QMatrix4x4& m) {
        const int id = scene.addObject(name, PrimitiveType::ImportedMesh, parent);
        Transform t = SceneGraph::toTransform(m);
        t.scale = {1, 1, 1};
        scene.getObject(id)->transform = t;
        return id;
    }

    int root(const QString& name) {
        QMatrix4x4 m;
        m.rotate(-90, 1, 0, 0);   // Z-up → Y-up
        return frame(name, -1, m);
    }

    void addActuator(int jointId, const QString& name, double velocity) {
        Motor m;
        m.name = scene.makeSymbolName(name);
        m.jointId = jointId;
        const Joint* j = scene.getJoint(jointId);
        m.velocity = velocity > 0 ? float(velocity) : (j && j->type == JointType::Prismatic ? 0.1f : 45.0f);
        scene.addMotor(m);
    }
};

QMatrix4x4 urdfOrigin(const QDomElement& parent) {
    const QDomElement o = parent.firstChildElement("origin");
    if (o.isNull()) return {};
    const QVector3D xyz = vec3(o.attribute("xyz"));
    const QVector3D rpy = vec3(o.attribute("rpy"));
    return translation(xyz) * rpyMatrix(rpy.x(), rpy.y(), rpy.z());
}

QColor rgba(const QString& s) {
    const auto n = numbers(s);
    if (n.size() < 3) return {};
    return QColor::fromRgbF(float(std::clamp(n[0], 0.0, 1.0)), float(std::clamp(n[1], 0.0, 1.0)),
                            float(std::clamp(n[2], 0.0, 1.0)));
}

} // namespace

// ── URDF import ──────────────────────────────────────────────────────────────

Result importUrdf(SceneGraph& scene, const QString& path) {
    Importer im{scene, QFileInfo(path).absolutePath(), {}, {}};
    QFile f(path);
    QDomDocument doc;
    if (!f.open(QIODevice::ReadOnly) || !doc.setContent(&f)) { im.res.error = "Not a readable URDF (XML) file."; return im.res; }
    const QDomElement robot = doc.documentElement();
    if (robot.tagName() != "robot") { im.res.error = "The file has no <robot> element."; return im.res; }

    QHash<QString, QColor> materials;
    for (QDomElement m = robot.firstChildElement("material"); !m.isNull(); m = m.nextSiblingElement("material"))
        materials[m.attribute("name")] = rgba(m.firstChildElement("color").attribute("rgba"));

    QHash<QString, QDomElement> links;
    for (QDomElement l = robot.firstChildElement("link"); !l.isNull(); l = l.nextSiblingElement("link"))
        links[l.attribute("name")] = l;
    std::vector<QDomElement> joints;
    std::set<QString> children;
    for (QDomElement j = robot.firstChildElement("joint"); !j.isNull(); j = j.nextSiblingElement("joint")) {
        joints.push_back(j);
        children.insert(j.firstChildElement("child").attribute("link"));
    }
    if (links.isEmpty()) { im.res.error = "The robot has no links."; return im.res; }

    const int root = im.root(robot.attribute("name", QFileInfo(path).completeBaseName()));
    im.res.rootId = root;
    QHash<QString, int> linkObj;

    auto addVisuals = [&](const QDomElement& link, int frameId) {
        int n = 0;
        for (QDomElement v = link.firstChildElement("visual"); !v.isNull(); v = v.nextSiblingElement("visual"), ++n) {
            const QMatrix4x4 origin = urdfOrigin(v);
            const QDomElement geo = v.firstChildElement("geometry");
            QColor color;
            const QDomElement mat = v.firstChildElement("material");
            if (!mat.isNull()) {
                color = rgba(mat.firstChildElement("color").attribute("rgba"));
                if (!color.isValid()) color = materials.value(mat.attribute("name"));
            }
            const QString name = n == 0 ? link.attribute("name") + " visual" : QString("%1 visual %2").arg(link.attribute("name")).arg(n + 1);
            if (QDomElement b = geo.firstChildElement("box"); !b.isNull()) {
                im.addShape(name, PrimitiveType::Cube, frameId, origin, vec3(b.attribute("size"), {0.1f, 0.1f, 0.1f}), nullptr, color);
            } else if (QDomElement c = geo.firstChildElement("cylinder"); !c.isNull()) {
                const float r = c.attribute("radius").toFloat(), l = c.attribute("length").toFloat();
                QMatrix4x4 m = origin;
                m.rotate(90, 1, 0, 0);   // URDF cylinders run along Z, ours along Y
                im.addShape(name, PrimitiveType::Cylinder, frameId, m, {2 * r, l, 2 * r}, nullptr, color);
            } else if (QDomElement s = geo.firstChildElement("sphere"); !s.isNull()) {
                const float d = 2 * s.attribute("radius").toFloat();
                im.addShape(name, PrimitiveType::Sphere, frameId, origin, {d, d, d}, nullptr, color);
            } else if (QDomElement m = geo.firstChildElement("mesh"); !m.isNull()) {
                auto g = im.mesh(m.attribute("filename"));
                if (g) im.addShape(name, PrimitiveType::ImportedMesh, frameId, origin,
                                   vec3(m.attribute("scale"), {1, 1, 1}), g, color);
            }
        }
    };

    // Links in tree order from the root link(s)
    std::function<void(const QString&, int, const QMatrix4x4&)> addLink;
    QHash<QString, int> jointIds;
    struct Mimic { int joint; QString master; double mult, offset; };
    std::vector<Mimic> mimics;
    addLink = [&](const QString& name, int parent, const QMatrix4x4& local) {
        if (linkObj.contains(name) || !links.contains(name)) return;
        const int id = im.frame(name, parent, local);
        linkObj[name] = id;
        addVisuals(links[name], id);
        for (const QDomElement& j : joints) {
            if (j.firstChildElement("parent").attribute("link") != name) continue;
            const QString child = j.firstChildElement("child").attribute("link");
            const QMatrix4x4 origin = urdfOrigin(j);
            addLink(child, id, origin);
            if (!linkObj.contains(child)) continue;
            const QString type = j.attribute("type");
            if (type == "fixed") continue;
            if (type == "floating" || type == "planar") {
                im.res.warnings << QString("Joint %1: %2 joints are imported as fixed").arg(j.attribute("name"), type);
                continue;
            }
            Joint jt;
            jt.name = j.attribute("name");
            jt.parentBodyId = id;
            jt.childBodyId = linkObj[child];
            const QVector3D axis = vec3(j.firstChildElement("axis").attribute("xyz"), {1, 0, 0}).normalized();
            jt.axis = origin.mapVector(axis).normalized();
            const QDomElement lim = j.firstChildElement("limit");
            if (type == "prismatic") {
                jt.type = JointType::Prismatic;
                jt.limitMin = lim.attribute("lower", "-1").toFloat();
                jt.limitMax = lim.attribute("upper", "1").toFloat();
            } else {
                jt.type = JointType::Revolute;
                jt.pivot = origin.column(3).toVector3D();
                jt.continuous = type == "continuous" || lim.isNull();
                jt.limitMin = float(qRadiansToDegrees(lim.attribute("lower", "-3.14159265").toDouble()));
                jt.limitMax = float(qRadiansToDegrees(lim.attribute("upper", "3.14159265").toDouble()));
            }
            const int jid = scene.addJoint(jt);
            jointIds[jt.name] = jid;
            const QDomElement mimic = j.firstChildElement("mimic");
            if (!mimic.isNull()) {
                mimics.push_back({jid, mimic.attribute("joint"), mimic.attribute("multiplier", "1").toDouble(),
                                  mimic.attribute("offset", "0").toDouble()});
            } else {
                double v = lim.attribute("velocity", "0").toDouble();
                if (jt.type == JointType::Revolute) v = qRadiansToDegrees(v);
                im.addActuator(jid, jt.name, std::min(v, jt.type == JointType::Revolute ? 180.0 : 1.0));
            }
        }
    };
    for (auto it = links.begin(); it != links.end(); ++it)
        if (!children.count(it.key())) addLink(it.key(), root, {});
    for (const Mimic& m : mimics) {
        Joint* j = scene.getJoint(m.joint);
        if (!jointIds.contains(m.master)) { im.res.warnings << "Mimic of unknown joint " + m.master; continue; }
        j->coupledTo = jointIds[m.master];
        j->ratio = float(m.mult);
        j->offset = float(j->type == JointType::Revolute ? qRadiansToDegrees(m.offset) : m.offset);
    }
    scene.applyCouplings();
    scene.notifyAutomationChanged();
    im.res.ok = true;
    return im.res;
}

// ── MJCF import ──────────────────────────────────────────────────────────────

Result importMjcf(SceneGraph& scene, const QString& path) {
    Importer im{scene, QFileInfo(path).absolutePath(), {}, {}};
    QFile f(path);
    QDomDocument doc;
    if (!f.open(QIODevice::ReadOnly) || !doc.setContent(&f)) { im.res.error = "Not a readable MJCF (XML) file."; return im.res; }
    const QDomElement model = doc.documentElement();
    if (model.tagName() != "mujoco") { im.res.error = "The file has no <mujoco> element."; return im.res; }

    // <include file="…"/>: the included file's top-level children replace it
    // (paths relative to the main file; includes may nest)
    for (int guard = 0; guard < 1000; ++guard) {
        const QDomNodeList incs = doc.elementsByTagName("include");
        if (incs.isEmpty()) break;
        QDomElement inc = incs.at(0).toElement();
        const QString file = inc.attribute("file");
        QFile incFile(QDir(im.dir).filePath(file));
        QDomDocument part;
        if (incFile.open(QIODevice::ReadOnly) && part.setContent(&incFile)) {
            for (QDomNode n = part.documentElement().firstChild(); !n.isNull(); n = n.nextSibling())
                inc.parentNode().insertBefore(doc.importNode(n, true), inc);
        } else {
            im.res.warnings << "Include not read: " + file;
        }
        inc.parentNode().removeChild(inc);
    }

    // Default classes: <default class="…"> nest; an element takes an
    // attribute from itself, else from its class (its own `class`, else the
    // nearest body `childclass`, else "main") and that class's parents
    QHash<QString, QDomElement> classes;
    QHash<QString, QString> parentClass;
    std::function<void(const QDomElement&, const QString&)> readDefaults = [&](const QDomElement& d, const QString& parent) {
        const QString name = d.attribute("class", parent.isEmpty() ? QString("main") : QString());
        if (name.isEmpty()) return;
        classes[name] = d;
        parentClass[name] = parent;
        for (QDomElement c = d.firstChildElement("default"); !c.isNull(); c = c.nextSiblingElement("default")) readDefaults(c, name);
    };
    for (QDomElement d = model.firstChildElement("default"); !d.isNull(); d = d.nextSiblingElement("default")) readDefaults(d, {});
    auto attr = [&](const QDomElement& e, const QString& name, const QString& childclass, const QString& fallback = {}) {
        if (e.hasAttribute(name)) return e.attribute(name);
        for (QString cls = e.attribute("class", childclass.isEmpty() ? QString("main") : childclass); !cls.isEmpty();
             cls = parentClass.value(cls)) {
            const QDomElement t = classes.value(cls).firstChildElement(e.tagName());
            if (!t.isNull() && t.hasAttribute(name)) return t.attribute(name);
            if (!parentClass.contains(cls)) break;
        }
        return fallback;
    };

    const QDomElement compiler = model.firstChildElement("compiler");
    const bool radians = compiler.attribute("angle", "degree") == "radian";
    const QString eulerseq = compiler.attribute("eulerseq", "xyz");
    QString meshdir = compiler.attribute("meshdir", compiler.attribute("assetdir"));
    auto angle = [&](double v) { return radians ? qRadiansToDegrees(v) : v; };   // to degrees

    // Assets: meshes (file, scale) and materials (colour)
    struct MeshAsset { QString file; QVector3D scale{1, 1, 1}; };
    QHash<QString, MeshAsset> meshAssets;
    QHash<QString, QColor> materials;
    for (QDomElement a = model.firstChildElement("asset"); !a.isNull(); a = a.nextSiblingElement("asset")) {
        for (QDomElement m = a.firstChildElement("mesh"); !m.isNull(); m = m.nextSiblingElement("mesh")) {
            const QString file = m.attribute("file");
            const QString name = m.attribute("name", QFileInfo(file).completeBaseName());
            meshAssets[name] = {meshdir.isEmpty() ? file : meshdir + "/" + file, vec3(attr(m, "scale", {}), {1, 1, 1})};
        }
        for (QDomElement m = a.firstChildElement("material"); !m.isNull(); m = m.nextSiblingElement("material"))
            materials[m.attribute("name")] = rgba(m.attribute("rgba"));
    }

    // Orientation of a body or geom from quat / euler / axisangle
    auto orientation = [&](const QDomElement& e) -> QMatrix4x4 {
        QMatrix4x4 m;
        if (e.hasAttribute("quat")) {
            const auto q = numbers(e.attribute("quat"));
            if (q.size() == 4) m.rotate(QQuaternion(float(q[0]), float(q[1]), float(q[2]), float(q[3])).normalized());
        } else if (e.hasAttribute("euler")) {
            const QVector3D a = vec3(e.attribute("euler"));
            for (int i = 0; i < 3 && i < eulerseq.size(); ++i) {
                const QChar c = eulerseq[i];
                const QVector3D axis(c.toLower() == 'x', c.toLower() == 'y', c.toLower() == 'z');
                // Lowercase: rotations about the moving (intrinsic) axes;
                // uppercase: about the fixed axes, applied on the left
                QMatrix4x4 r;
                r.rotate(float(angle(a[i])), axis);
                m = c.isLower() ? m * r : r * m;
            }
        } else if (e.hasAttribute("axisangle")) {
            const auto v = numbers(e.attribute("axisangle"));
            if (v.size() == 4) m.rotate(float(angle(v[3])), float(v[0]), float(v[1]), float(v[2]));
        } else if (e.hasAttribute("xyaxes")) {
            const auto v = numbers(e.attribute("xyaxes"));
            if (v.size() == 6) {
                const QVector3D x = QVector3D(float(v[0]), float(v[1]), float(v[2])).normalized();
                QVector3D y{float(v[3]), float(v[4]), float(v[5])};
                y = (y - x * QVector3D::dotProduct(x, y)).normalized();
                const QVector3D z = QVector3D::crossProduct(x, y);
                m.setColumn(0, QVector4D(x, 0));
                m.setColumn(1, QVector4D(y, 0));
                m.setColumn(2, QVector4D(z, 0));
            }
        } else if (e.hasAttribute("zaxis")) {
            m.rotate(QQuaternion::rotationTo(QVector3D(0, 0, 1), vec3(e.attribute("zaxis"), {0, 0, 1}).normalized()));
        }
        return m;
    };
    auto placement = [&](const QDomElement& e) { return translation(vec3(e.attribute("pos"))) * orientation(e); };

    const int root = im.root(model.attribute("model", QFileInfo(path).completeBaseName()));
    im.res.rootId = root;
    QHash<QString, int> jointIds;
    std::vector<int> movable;
    bool warnedCapsule = false;

    auto addGeoms = [&](const QDomElement& body, int bodyId, const QString& bodyName, const QString& cc) {
        int n = 0;
        for (QDomElement g = body.firstChildElement("geom"); !g.isNull(); g = g.nextSiblingElement("geom"), ++n) {
            const QString meshName = attr(g, "mesh", cc);
            const QString type = attr(g, "type", cc, meshName.isEmpty() ? "sphere" : "mesh");
            if (type == "plane" || type == "hfield") continue;
            const QString rgbaText = attr(g, "rgba", cc);
            QColor color = !rgbaText.isEmpty() ? rgba(rgbaText) : materials.value(attr(g, "material", cc));
            const auto size = numbers(attr(g, "size", cc));
            auto sz = [&](size_t i, double d) { return i < size.size() ? float(size[i]) : float(d); };
            QMatrix4x4 frame = placement(g);
            float length = 2 * sz(1, 0.05);
            if (!attr(g, "fromto", cc).isEmpty()) {
                const auto ft = numbers(attr(g, "fromto", cc));
                if (ft.size() == 6) {
                    const QVector3D a{float(ft[0]), float(ft[1]), float(ft[2])}, b{float(ft[3]), float(ft[4]), float(ft[5])};
                    frame = translation((a + b) / 2);
                    frame.rotate(QQuaternion::rotationTo(QVector3D(0, 0, 1), (b - a).normalized()));
                    length = (b - a).length();
                }
            }
            const QString name = g.attribute("name", n == 0 ? bodyName + " visual" : QString("%1 visual %2").arg(bodyName).arg(n + 1));
            if (type == "box") {
                im.addShape(name, PrimitiveType::Cube, bodyId, frame, {2 * sz(0, 0.05), 2 * sz(1, 0.05), 2 * sz(2, 0.05)}, nullptr, color);
            } else if (type == "sphere" || type == "ellipsoid") {
                const float r = sz(0, 0.05);
                im.addShape(name, PrimitiveType::Sphere, bodyId, frame,
                            type == "sphere" ? QVector3D(2 * r, 2 * r, 2 * r) : QVector3D(2 * r, 2 * sz(1, r), 2 * sz(2, r)), nullptr, color);
            } else if (type == "cylinder" || type == "capsule") {
                const float r = sz(0, 0.05);
                if (type == "capsule") {
                    length += 2 * r;   // the rounded ends, as a cylinder
                    if (!warnedCapsule) { im.res.warnings << "Capsules are imported as cylinders"; warnedCapsule = true; }
                }
                frame.rotate(90, 1, 0, 0);   // MJCF cylinders run along Z, ours along Y
                im.addShape(name, PrimitiveType::Cylinder, bodyId, frame, {2 * r, length, 2 * r}, nullptr, color);
            } else if (type == "mesh") {
                const MeshAsset a = meshAssets.value(meshName);
                auto m = a.file.isEmpty() ? nullptr : im.mesh(a.file);
                if (!m) { im.res.warnings << "Mesh asset not found: " + meshName; continue; }
                im.addShape(name, PrimitiveType::ImportedMesh, bodyId, frame, a.scale, m, color);
            }
        }
    };

    std::function<void(const QDomElement&, int, const QString&)> addBody = [&](const QDomElement& body, int parentId,
                                                                             const QString& inherited) {
        const QString cc = body.attribute("childclass", inherited);
        const QString name = body.attribute("name", "body");
        const QMatrix4x4 local = placement(body);
        // Its joints act one after another in the body's frame, so several
        // joints become a chain: a frame per joint before the body's own
        std::vector<QDomElement> joints;
        for (QDomElement j = body.firstChildElement("joint"); !j.isNull(); j = j.nextSiblingElement("joint")) {
            const QString type = attr(j, "type", cc, "hinge");
            if (type == "free" || type == "ball") im.res.warnings << QString("Body %1: %2 joints are not imported").arg(name, type);
            else joints.push_back(j);
        }
        int parent = parentId;
        std::vector<int> links;   // the object each joint moves
        for (size_t i = 0; i + 1 < joints.size(); ++i) {
            parent = im.frame(QString("%1 (%2)").arg(name, joints[i].attribute("name", QString::number(i + 1))), parent,
                              i == 0 ? local : QMatrix4x4());
            links.push_back(parent);
        }
        const int id = im.frame(name, parent, joints.size() > 1 ? QMatrix4x4() : local);
        links.push_back(id);
        addGeoms(body, id, name, cc);
        for (size_t i = 0; i < joints.size(); ++i) {
            const QDomElement& j = joints[i];
            const QString type = attr(j, "type", cc, "hinge");
            const QMatrix4x4 inParent = i == 0 ? local : QMatrix4x4();   // the body frame in the joint's parent
            Joint jt;
            jt.name = j.attribute("name", joints.size() > 1 ? QString("%1_joint%2").arg(name).arg(i + 1) : name + "_joint");
            jt.parentBodyId = i == 0 ? parentId : links[i - 1];
            jt.childBodyId = links[i];
            jt.axis = inParent.mapVector(vec3(attr(j, "axis", cc), {0, 0, 1})).normalized();
            const auto range = numbers(attr(j, "range", cc));
            const bool limited = range.size() == 2 && attr(j, "limited", cc, "auto") != "false";
            if (type == "slide") {
                jt.type = JointType::Prismatic;
                jt.limitMin = limited ? float(range[0]) : -1.0f;
                jt.limitMax = limited ? float(range[1]) : 1.0f;
            } else {
                jt.type = JointType::Revolute;
                jt.pivot = inParent.map(vec3(attr(j, "pos", cc)));
                jt.continuous = !limited;
                jt.limitMin = limited ? float(angle(range[0])) : -180.0f;
                jt.limitMax = limited ? float(angle(range[1])) : 180.0f;
            }
            const int jid = scene.addJoint(jt);
            jointIds[jt.name] = jid;
            movable.push_back(jid);
        }
        if (!body.firstChildElement("freejoint").isNull())
            im.res.warnings << QString("Body %1: free joint not imported (add a rigid body to let it move)").arg(name);
        for (QDomElement c = body.firstChildElement("body"); !c.isNull(); c = c.nextSiblingElement("body")) addBody(c, id, cc);
    };
    const QDomElement world = model.firstChildElement("worldbody");
    addGeoms(world, root, "world", {});
    for (QDomElement b = world.firstChildElement("body"); !b.isNull(); b = b.nextSiblingElement("body")) addBody(b, root, {});

    // Joint equalities: joint1 = c0 + c1 · joint2 → couplings
    std::set<int> followers;
    for (QDomElement eq = model.firstChildElement("equality"); !eq.isNull(); eq = eq.nextSiblingElement("equality"))
        for (QDomElement j = eq.firstChildElement("joint"); !j.isNull(); j = j.nextSiblingElement("joint")) {
            const QString a = j.attribute("joint1"), b = j.attribute("joint2");
            if (!jointIds.contains(a) || !jointIds.contains(b)) continue;
            const auto c = numbers(j.attribute("polycoef", "0 1 0 0 0"));
            Joint* follower = scene.getJoint(jointIds[a]);
            follower->coupledTo = jointIds[b];
            follower->ratio = c.size() > 1 ? float(c[1]) : 1.0f;
            const double offset = c.empty() ? 0.0 : c[0];
            follower->offset = float(follower->type == JointType::Revolute ? qRadiansToDegrees(offset) : offset);
            followers.insert(follower->id);
        }
    // Actuators: the joints the file's actuators drive, else every movable joint
    std::set<int> driven;
    for (QDomElement act = model.firstChildElement("actuator"); !act.isNull(); act = act.nextSiblingElement("actuator"))
        for (QDomElement a = act.firstChildElement(); !a.isNull(); a = a.nextSiblingElement())
            if (jointIds.contains(a.attribute("joint"))) driven.insert(jointIds[a.attribute("joint")]);
    for (int jid : movable) {
        if (followers.count(jid) || (!driven.empty() && !driven.count(jid))) continue;
        im.addActuator(jid, scene.getJoint(jid)->name, 0);
    }
    scene.applyCouplings();
    scene.notifyAutomationChanged();
    im.res.ok = true;
    return im.res;
}

// ── Export ───────────────────────────────────────────────────────────────────

namespace {

// The scene in rigid link frames at joint value 0. A jointed object's link
// sits on its joint (rotation origin) with its parent link's orientation, so
// the joint axis is constant in it; other objects' links are their own
// frames without scale.
struct Node {
    int        id = -1, parent = -1;
    QString    name;
    QMatrix4x4 world;      // object frame (with scale), joints at 0
    QMatrix4x4 link;       // rigid link frame in the world
    const Joint* joint = nullptr;
    QVector3D  axisWorld;
};

struct Exporter {
    SceneGraph& scene;
    QString     path, meshDir;
    Result      res;
    std::vector<Node> nodes;                       // parents before children
    std::map<const std::vector<float>*, QString> meshFiles;   // soup → file (no extension)

    bool prepare() {
        const QFileInfo fi(path);
        meshDir = fi.absoluteDir().filePath("meshes");
        if (!QDir().mkpath(meshDir)) { res.error = "Can't create " + meshDir; return false; }
        // Joints at 0 while the frames are taken
        std::map<int, float> saved;
        for (auto& [id, j] : scene.allJoints()) { saved[id] = j.currentValue; scene.getJoint(id)->currentValue = 0; }
        std::set<QString> used;
        std::function<void(int, int)> visit = [&](int id, int parent) {
            const SceneObject* o = scene.getObject(id);
            Node n;
            n.id = id;
            n.parent = parent;
            QString base = sanitize(o->name, "link"), name = base;
            for (int i = 2; used.count(name); ++i) name = QString("%1_%2").arg(base).arg(i);
            used.insert(name);
            n.name = name;
            n.world = scene.worldTransform(id);
            for (auto& [jid, j] : scene.allJoints())
                if (j.childBodyId == id) { n.joint = &j; break; }
            const Node* p = nullptr;
            for (const Node& x : nodes) if (x.id == parent) p = &x;
            if (n.joint && p) {
                const QMatrix4x4 parentWorld = scene.worldTransform(parent);
                const QVector3D at = n.joint->type == JointType::Revolute ? parentWorld.map(n.joint->pivot)
                                                                          : n.world.column(3).toVector3D();
                n.axisWorld = parentWorld.mapVector(n.joint->axis).normalized();
                n.link = p->link;
                n.link.setColumn(3, QVector4D(at, 1));
            } else {
                n.link = scenequery::stripScale(n.world);
            }
            nodes.push_back(n);
            for (int c : o->childIds) visit(c, id);
        };
        for (int r : scene.rootObjects()) visit(r, -1);
        for (auto& [id, v] : saved) scene.getJoint(id)->currentValue = v;
        return true;
    }

    // The STL file (in meshes/) for an object's geometry, written once
    QString meshFile(const SceneObject& o) {
        const std::vector<float>* soup = surfaceOf(o);
        if (!soup) return {};
        auto it = meshFiles.find(soup);
        if (it != meshFiles.end()) return it->second;
        const QString name = o.type == PrimitiveType::ImportedMesh
            ? QString("mesh%1").arg(meshFiles.size()) : QString("primitive_%1").arg(int(o.type));
        if (!writeStl(*soup, QDir(meshDir).filePath(name + ".stl"))) { res.warnings << "Can't write " + name + ".stl"; return {}; }
        ++res.files;
        meshFiles[soup] = name;
        return name;
    }

    const Node* node(int id) const {
        for (const Node& n : nodes) if (n.id == id) return &n;
        return nullptr;
    }
    QString jointName(const Node& n) const {
        const Motor* m = scene.motorForJoint(n.joint->id);
        return m ? m->name : n.name + "_joint";
    }
};

QString colorRgba(const QColor& c) { return QString("%1 %2 %3 1").arg(num(c.redF()), num(c.greenF()), num(c.blueF())); }

QString originXml(const QMatrix4x4& m) {
    double r, p, y;
    toRpy(m, r, p, y);
    return QString("<origin xyz=\"%1\" rpy=\"%2 %3 %4\"/>").arg(vec(m.column(3).toVector3D()), num(r), num(p), num(y));
}

QString quatAttr(const QMatrix4x4& m) {
    const QQuaternion q = QQuaternion::fromRotationMatrix(scenequery::stripScale(m).toGenericMatrix<3, 3>()).normalized();
    return QString("%1 %2 %3 %4").arg(num(q.scalar()), num(q.x()), num(q.y()), num(q.z()));
}

} // namespace

Result exportUrdf(SceneGraph& scene, const QString& path) {
    Exporter ex{scene, path, {}, {}, {}, {}};
    if (!ex.prepare()) return ex.res;
    QString xml;
    QTextStream out(&xml);
    const QString robotName = sanitize(QFileInfo(path).completeBaseName(), "robot");
    out << "<?xml version=\"1.0\"?>\n<!-- Exported by MotionStudio. Joint values are 0 in this pose. -->\n";
    out << "<robot name=\"" << robotName << "\">\n";
    out << "  <link name=\"base_link\"/>\n";
    for (const Node& n : ex.nodes) {
        const SceneObject* o = scene.getObject(n.id);
        out << "  <link name=\"" << n.name << "\">\n";
        const QString mesh = ex.meshFile(*o);
        if (!mesh.isEmpty()) {
            QMatrix4x4 rigid;
            QVector3D scale;
            splitScale(n.link.inverted() * n.world, &rigid, &scale);
            const QColor c = scene.colorOf(n.id);
            for (const char* tag : {"visual", "collision"}) {
                out << "    <" << tag << ">\n      " << originXml(rigid) << "\n";
                out << "      <geometry><mesh filename=\"meshes/" << mesh << ".stl\" scale=\"" << vec(scale) << "\"/></geometry>\n";
                if (QString(tag) == "visual" && c.isValid())
                    out << "      <material name=\"" << c.name().mid(1) << "\"><color rgba=\"" << colorRgba(c) << "\"/></material>\n";
                out << "    </" << tag << ">\n";
            }
        }
        const RigidBody* rb = scene.allRigidBodies().count(n.id) ? &scene.allRigidBodies().at(n.id) : nullptr;
        out << "    <inertial><mass value=\"" << num(rb ? rb->mass : 0.1) << "\"/>"
            << "<inertia ixx=\"0.001\" iyy=\"0.001\" izz=\"0.001\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/></inertial>\n";
        out << "  </link>\n";
    }
    for (const Node& n : ex.nodes) {
        const Node* p = ex.node(n.parent);
        const QMatrix4x4 parentLink = p ? p->link : yUpToZUp().inverted();
        const QMatrix4x4 origin = parentLink.inverted() * n.link;
        if (n.joint && p) {
            const Joint& j = *n.joint;
            const QString type = j.type == JointType::Prismatic ? "prismatic" : (j.continuous ? "continuous" : "revolute");
            const QVector3D axis = scenequery::stripScale(n.link).inverted().mapVector(n.axisWorld).normalized();
            out << "  <joint name=\"" << ex.jointName(n) << "\" type=\"" << type << "\">\n";
            out << "    <parent link=\"" << p->name << "\"/><child link=\"" << n.name << "\"/>\n";
            out << "    " << originXml(origin) << "\n    <axis xyz=\"" << vec(axis) << "\"/>\n";
            const Motor* m = scene.motorForJoint(j.id);
            const double k = j.type == JointType::Revolute ? M_PI / 180.0 : 1.0;
            const double vel = (m ? std::abs(m->velocity) : (j.type == JointType::Revolute ? 45.0 : 0.1)) * k;
            if (type != "continuous")
                out << "    <limit lower=\"" << num(std::min(j.limitMin, j.limitMax) * k) << "\" upper=\""
                    << num(std::max(j.limitMin, j.limitMax) * k) << "\" effort=\"100\" velocity=\"" << num(vel) << "\"/>\n";
            else
                out << "    <limit effort=\"100\" velocity=\"" << num(vel) << "\"/>\n";
            if (j.coupledTo != -1) {
                for (const Node& mn : ex.nodes)
                    if (mn.joint && mn.joint->id == j.coupledTo)
                        out << "    <mimic joint=\"" << ex.jointName(mn) << "\" multiplier=\"" << num(j.ratio)
                            << "\" offset=\"" << num(j.offset * k) << "\"/>\n";
            }
            out << "  </joint>\n";
        } else {
            out << "  <joint name=\"" << n.name << "_fixed\" type=\"fixed\">\n";
            out << "    <parent link=\"" << (p ? p->name : QString("base_link")) << "\"/><child link=\"" << n.name << "\"/>\n";
            out << "    " << originXml(origin) << "\n  </joint>\n";
        }
    }
    out << "</robot>\n";
    out.flush();
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly)) { ex.res.error = "Can't write " + path; return ex.res; }
    f.write(xml.toUtf8());
    if (!scene.allConveyorBelts().empty() || !scene.allSignals().empty() || !scene.allSequences().empty())
        ex.res.warnings << "URDF has no conveyors, signals or sequences; only the mechanism is exported";
    ex.res.ok = true;
    return ex.res;
}

Result exportMjcf(SceneGraph& scene, const QString& path) {
    Exporter ex{scene, path, {}, {}, {}, {}};
    if (!ex.prepare()) return ex.res;
    // Mesh assets per (geometry, scale): MuJoCo scales meshes in the asset
    std::map<std::pair<QString, QString>, QString> assets;
    QString assetXml, bodiesXml, equalityXml, actuatorXml;
    QTextStream a(&assetXml), b(&bodiesXml), e(&equalityXml), act(&actuatorXml);

    std::function<void(const Node&, int)> writeBody = [&](const Node& n, int depth) {
        const QString pad(depth * 2, ' ');
        const Node* p = ex.node(n.parent);
        // Top-level bodies take the Y-up → Z-up turn themselves: a free joint
        // is only allowed directly under the world
        const QMatrix4x4 local = p ? p->link.inverted() * n.link : yUpToZUp() * n.link;
        b << pad << "<body name=\"" << n.name << "\" pos=\"" << vec(local.column(3).toVector3D())
          << "\" quat=\"" << quatAttr(local) << "\">\n";
        const RigidBody* rb = scene.allRigidBodies().count(n.id) ? &scene.allRigidBodies().at(n.id) : nullptr;
        if (rb && !p) b << pad << "  <freejoint/>\n";
        if (n.joint && p) {
            const Joint& j = *n.joint;
            const QVector3D axis = scenequery::stripScale(n.link).inverted().mapVector(n.axisWorld).normalized();
            const double k = j.type == JointType::Revolute ? M_PI / 180.0 : 1.0;
            b << pad << "  <joint name=\"" << ex.jointName(n) << "\" type=\"" << (j.type == JointType::Prismatic ? "slide" : "hinge")
              << "\" axis=\"" << vec(axis) << "\"";
            if (!(j.type == JointType::Revolute && j.continuous))
                b << " range=\"" << num(std::min(j.limitMin, j.limitMax) * k) << " " << num(std::max(j.limitMin, j.limitMax) * k) << "\"";
            b << " damping=\"1\"/>\n";
            if (const Motor* m = scene.motorForJoint(j.id)) {
                if (m->mode == ActuatorMode::Speed)
                    act << "    <velocity name=\"" << m->name << "\" joint=\"" << m->name << "\" kv=\"50\"/>\n";
                else
                    act << "    <position name=\"" << m->name << "\" joint=\"" << m->name << "\" kp=\"500\" kv=\"20\"/>\n";
            }
            if (j.coupledTo != -1)
                for (const Node& mn : ex.nodes)
                    if (mn.joint && mn.joint->id == j.coupledTo)
                        e << "    <joint joint1=\"" << ex.jointName(n) << "\" joint2=\"" << ex.jointName(mn)
                          << "\" polycoef=\"" << num(j.offset * k) << " " << num(j.ratio) << " 0 0 0\"/>\n";
        }
        const SceneObject* o = scene.getObject(n.id);
        const QString mesh = ex.meshFile(*o);
        if (!mesh.isEmpty()) {
            QMatrix4x4 rigid;
            QVector3D scale;
            splitScale(n.link.inverted() * n.world, &rigid, &scale);
            const auto key = std::make_pair(mesh, vec(scale));
            QString asset = assets.count(key) ? assets[key] : QString();
            if (asset.isEmpty()) {
                asset = QString("%1_%2").arg(mesh).arg(assets.size());
                assets[key] = asset;
                a << "    <mesh name=\"" << asset << "\" file=\"" << mesh << ".stl\" scale=\"" << vec(scale) << "\"/>\n";
            }
            const QColor c = scene.colorOf(n.id);
            // Contacts only for rigid bodies and conveyor belts, as in the simulation
            bool underRigid = false;
            for (const SceneObject* q = o; q; q = scene.getObject(q->parentId))
                if (scene.allRigidBodies().count(q->id)) underRigid = true;
            const bool collides = underRigid || scene.allConveyorBelts().count(n.id);
            b << pad << "  <geom type=\"mesh\" mesh=\"" << asset << "\" pos=\"" << vec(rigid.column(3).toVector3D())
              << "\" quat=\"" << quatAttr(rigid) << "\"";
            if (c.isValid()) b << " rgba=\"" << colorRgba(c) << "\"";
            if (rb) b << " mass=\"" << num(rb->mass) << "\"";
            if (!collides) b << " contype=\"0\" conaffinity=\"0\"";
            b << "/>\n";
        }
        for (const Node& c : ex.nodes) if (c.parent == n.id) writeBody(c, depth + 1);
        b << pad << "</body>\n";
    };
    for (const Node& n : ex.nodes) if (n.parent == -1) writeBody(n, 2);

    QString xml;
    QTextStream out(&xml);
    out << "<!-- Exported by MotionStudio (Z-up). Joint values are 0 in this pose. -->\n";
    out << "<mujoco model=\"" << sanitize(QFileInfo(path).completeBaseName(), "model") << "\">\n";
    out << "  <compiler angle=\"radian\" meshdir=\"meshes\" autolimits=\"true\"/>\n";
    out << "  <option timestep=\"0.002\" integrator=\"implicitfast\"/>\n";
    if (!assetXml.isEmpty()) out << "  <asset>\n" << assetXml << "  </asset>\n";
    out << "  <worldbody>\n    <light pos=\"0 0 4\" dir=\"0 0 -1\"/>\n";
    if (scene.floorEnabled()) out << "    <geom name=\"floor\" type=\"plane\" size=\"10 10 0.1\" rgba=\"0.8 0.8 0.8 1\"/>\n";
    out << bodiesXml << "  </worldbody>\n";
    if (!equalityXml.isEmpty()) out << "  <equality>\n" << equalityXml << "  </equality>\n";
    if (!actuatorXml.isEmpty()) out << "  <actuator>\n" << actuatorXml << "  </actuator>\n";
    out << "</mujoco>\n";
    out.flush();
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly)) { ex.res.error = "Can't write " + path; return ex.res; }
    f.write(xml.toUtf8());
    if (!scene.allSignals().empty() || !scene.allSequences().empty())
        ex.res.warnings << "MJCF has no signals or sequences; the mechanism, actuators and couplings are exported";
    ex.res.ok = true;
    return ex.res;
}

} // namespace robotio
