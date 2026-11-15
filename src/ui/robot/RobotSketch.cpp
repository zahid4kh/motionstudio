#include "RobotSketch.h"
#include "automation/Kinematics.h"
#include "scene/PrimitiveMesh.h"
#include "scene/SceneGraph.h"
#include "scene/SceneQuery.h"
#include "ui/Theme.h"
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <functional>
#include <map>

namespace {

// Andrew's monotone chain
QPolygonF convexHull(std::vector<QPointF> p) {
    if (p.size() < 3) return QPolygonF(QList<QPointF>(p.begin(), p.end()));
    std::sort(p.begin(), p.end(), [](const QPointF& a, const QPointF& b) {
        return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
    auto cross = [](const QPointF& o, const QPointF& a, const QPointF& b) {
        return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
    };
    std::vector<QPointF> h(2 * p.size());
    size_t k = 0;
    for (size_t i = 0; i < p.size(); ++i) {
        while (k >= 2 && cross(h[k - 2], h[k - 1], p[i]) <= 0) --k;
        h[k++] = p[i];
    }
    for (size_t i = p.size() - 1, t = k + 1; i > 0; --i) {
        while (k >= t && cross(h[k - 2], h[k - 1], p[i - 1]) <= 0) --k;
        h[k++] = p[i - 1];
    }
    h.resize(k > 0 ? k - 1 : 0);
    return QPolygonF(QList<QPointF>(h.begin(), h.end()));
}

double niceStep(double minimum) {
    for (double s : {0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.25, 0.5, 1.0, 2.0, 5.0})
        if (s >= minimum) return s;
    return 10;
}

QString mm(double metres) { return QString::number(std::round(metres * 1000)); }

RobotSketch::View viewOf(const robot::DimensionLine& d) {
    return d.front ? RobotSketch::View::Front : d.top ? RobotSketch::View::Top : RobotSketch::View::Side;
}

} // namespace

RobotSketch::RobotSketch(QWidget* parent) : QWidget(parent) {
    setMouseTracking(true);
    setCursor(Qt::OpenHandCursor);
    connect(Theme::notifier(), &Theme::Notifier::changed, this, qOverload<>(&QWidget::update));
}

void RobotSketch::setView(View v) {
    if (m_view == v) return;
    m_view = v;
    m_zoom = 1;
    m_pan = {};
    update();
}

QPointF RobotSketch::project(const QVector3D& p) const {
    switch (m_view) {
    case View::Side:  return {p.x(), p.y()};
    case View::Top:   return {p.x(), -p.z()};
    case View::Front: return {-p.z(), p.y()};
    }
    return {};
}

void RobotSketch::clear() {
    m_side.clear();
    m_top.clear();
    m_front.clear();
    m_joints.clear();
    m_tips.clear();
    m_envSide.clear();
    m_envTop.clear();
    update();
}

void RobotSketch::setMessage(const QString& text) { m_message = text; update(); }

void RobotSketch::setJointLabels(bool on) { m_jointLabels = on; update(); }

void RobotSketch::capture(const SceneGraph& g, int baseFrame, int tcpId) {
    capture(g, baseFrame, std::vector<Tip>{{tcpId, "TCP"}});
}

void RobotSketch::capture(const SceneGraph& g, int baseFrame, const std::vector<Tip>& tips) {
    m_side.clear();
    m_top.clear();
    m_front.clear();
    m_joints.clear();
    m_tips.clear();
    m_message.clear();
    const QMatrix4x4 toRobot = scenequery::stripScale(g.worldTransform(baseFrame)).inverted();
    std::function<void(int)> walk = [&](int id) {
        const SceneObject* o = g.getObject(id);
        if (!o || o->parked) return;
        const std::vector<float>* soup = surfaceOf(*o);
        const float opacity = g.opacityOf(id);
        if (soup && !soup->empty() && opacity > 0.01f) {
            const QMatrix4x4 m = toRobot * g.worldTransform(id);
            const size_t verts = soup->size() / 6;
            const size_t step = std::max<size_t>(1, verts / 3000);
            std::vector<QPointF> side, top, front;
            double zs = 0, ys = 0, xs = 0;
            for (size_t v = 0; v < verts; v += step) {
                const QVector3D p = m.map(QVector3D((*soup)[v * 6], (*soup)[v * 6 + 1], (*soup)[v * 6 + 2]));
                side.emplace_back(p.x(), p.y());
                top.emplace_back(p.x(), -p.z());
                front.emplace_back(-p.z(), p.y());
                zs += p.z();
                ys += p.y();
                xs += p.x();
            }
            const double n = double(side.size());
            QColor c = g.colorOf(id);
            if (!c.isValid()) c = QColor(150, 156, 164);
            m_side.push_back({convexHull(side), c, opacity, zs / n});
            m_top.push_back({convexHull(top), c, opacity, ys / n});
            m_front.push_back({convexHull(front), c, opacity, xs / n});
        }
        for (int c : o->childIds) walk(c);
    };
    walk(baseFrame);
    auto byDepth = [](const Shape& a, const Shape& b) { return a.depth < b.depth; };   // far first
    std::stable_sort(m_side.begin(), m_side.end(), byDepth);
    std::stable_sort(m_top.begin(), m_top.end(), byDepth);
    std::stable_sort(m_front.begin(), m_front.end(), byDepth);
    // Chains share their first joints (a humanoid's waist): each joint once,
    // linked to the one before it in its chain
    std::map<int, int> markOf;   // joint id → index in m_joints
    for (const Tip& tip : tips) {
        if (!g.getObject(tip.id)) continue;
        int prev = -1;
        for (int jid : kin::chainTo(g, tip.id, false)) {
            if (auto it = markOf.find(jid); it != markOf.end()) { prev = it->second; continue; }
            const Joint* j = g.getJoint(jid);
            if (!j) continue;
            const QMatrix4x4 parent = toRobot * g.worldTransform(j->parentBodyId);
            JointMark mk;
            mk.axis = parent.mapVector(j->axis).normalized();
            mk.point = j->type == JointType::Revolute ? parent.map(j->pivot)
                                                      : (toRobot * g.worldTransform(j->childBodyId)).column(3).toVector3D();
            mk.prismatic = j->type == JointType::Prismatic;
            const Motor* mo = g.motorForJoint(jid);
            mk.name = mo ? mo->name : j->name;
            mk.passive = !mo;
            mk.prev = prev;
            prev = markOf[jid] = int(m_joints.size());
            m_joints.push_back(mk);
        }
        m_tips.push_back({(toRobot * g.worldTransform(tip.id)).column(3).toVector3D(), tip.label, prev});
    }
    update();
}

void RobotSketch::setDimensions(const std::vector<robot::DimensionLine>& d) { m_dims = d; update(); }

void RobotSketch::setHighlight(const QString& symbol) {
    if (m_highlight == symbol) return;
    m_highlight = symbol;
    update();
}

void RobotSketch::setEnvelope(const robot::Analysis* a, const SceneGraph* g) {
    m_envSide.clear();
    m_envTop.clear();
    if (!a || !a->error.isEmpty() || !g) { update(); return; }
    // The frame the parts were captured in: the first joint's parent
    const Joint* j1 = a->axes.empty() ? nullptr : g->getJoint(a->axes.front().jointId);
    if (!j1) { update(); return; }
    const QMatrix4x4 toRobot = scenequery::stripScale(g->worldTransform(j1->parentBodyId)).inverted();
    const QVector3D base = toRobot.map(a->basePoint);
    const bool mirror = a->verticalBase && a->sweep >= 180;
    auto runs = [](const robot::EnvelopeGrid& e, const std::function<void(int, int, int)>& f) {
        for (int j = 0; j < e.h; ++j)
            for (int i = 0; i < e.w;) {
                if (!e.at(i, j)) { ++i; continue; }
                int k = i;
                while (k < e.w && e.at(k, j)) ++k;
                f(j, i, k);
                i = k;
            }
    };
    const robot::EnvelopeGrid& s = a->side;
    runs(s, [&](int j, int i0, int i1) {
        const double y0 = toRobot.map(QVector3D(a->basePoint.x(), float(s.y0 + j * s.cell), a->basePoint.z())).y();
        const double r0 = s.x0 + i0 * s.cell, r1 = s.x0 + i1 * s.cell;
        if (a->verticalBase) {
            m_envSide.push_back(QPolygonF(QRectF(QPointF(base.x() + r0, y0), QPointF(base.x() + r1, y0 + s.cell))));
            if (mirror) m_envSide.push_back(QPolygonF(QRectF(QPointF(base.x() - r1, y0), QPointF(base.x() - r0, y0 + s.cell))));
        }
    });
    const robot::EnvelopeGrid& t = a->top;
    runs(t, [&](int j, int i0, int i1) {
        const double x0 = t.x0 + i0 * t.cell, x1 = t.x0 + i1 * t.cell, z0 = t.y0 + j * t.cell, z1 = z0 + t.cell;
        const float y = a->basePoint.y();
        QPolygonF q;
        for (auto [x, z] : {std::pair{x0, z0}, {x1, z0}, {x1, z1}, {x0, z1}}) {
            const QVector3D p = toRobot.map(QVector3D(float(x), y, float(z)));
            q << QPointF(p.x(), -p.z());
        }
        m_envTop.push_back(q);
    });
    update();
}

QRectF RobotSketch::contentBounds() const {
    QRectF b;
    auto add = [&](const QRectF& r) { b = b.isNull() ? r : b.united(r); };
    for (const Shape& s : shapes()) add(s.poly.boundingRect());
    if (m_view != View::Front)
        for (const QPolygonF& p : m_view == View::Side ? m_envSide : m_envTop) add(p.boundingRect());
    for (const JointMark& j : m_joints) add(QRectF(project(j.point), QSizeF(1e-4, 1e-4)));
    for (const robot::DimensionLine& d : m_dims) {
        if (viewOf(d) != m_view) continue;
        const QVector3D o = d.offsetDir * float(d.offset * 1.4);
        add(QRectF(project(d.from + o), QSizeF(1e-4, 1e-4)));
        add(QRectF(project(d.to + o), QSizeF(1e-4, 1e-4)));
    }
    if (b.isNull()) b = QRectF(-0.5, -0.1, 1, 1);
    if (m_view != View::Top) add(QRectF(b.left(), 0, b.width(), 1e-4));   // floor
    const double m = std::max(b.width(), b.height()) * 0.06 + 0.02;
    return b.adjusted(-m, -m, m, m);
}

QTransform RobotSketch::toScreen() const {
    const QRectF b = contentBounds();
    const double s = std::min(width() / b.width(), height() / b.height()) * m_zoom;
    QTransform t;
    t.translate(width() / 2.0 + m_pan.x(), height() / 2.0 + m_pan.y());
    t.scale(s, -s);
    t.translate(-b.center().x(), -b.center().y());
    return t;
}

void RobotSketch::paintEvent(QPaintEvent*) {
    const Theme::Colors& c = Theme::colors();
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    QPainterPath frame;
    frame.addRoundedRect(QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5), 10, 10);
    p.fillPath(frame, c.viewport);
    p.setClipPath(frame);
    m_hits.clear();

    const bool side = m_view == View::Side, top = m_view == View::Top;
    if (shapes().empty() && m_joints.empty()) {
        p.setPen(c.muted);
        p.drawText(rect().adjusted(20, 20, -20, -20), Qt::AlignCenter | Qt::TextWordWrap,
                   m_message.isEmpty() ? QString("Nothing to draw") : m_message);
        return;
    }
    const QTransform T = toScreen();
    const double scale = T.m11();

    // Grid and a scale bar
    const double step = niceStep(28 / scale);
    const QTransform inv = T.inverted();
    const QRectF vis = inv.mapRect(QRectF(rect()));
    p.setPen(QPen(c.grid, 1));
    for (double x = std::floor(vis.left() / step) * step; x <= vis.right(); x += step)
        p.drawLine(T.map(QPointF(x, vis.top())), T.map(QPointF(x, vis.bottom())));
    for (double y = std::floor(vis.top() / step) * step; y <= vis.bottom(); y += step)
        p.drawLine(T.map(QPointF(vis.left(), y)), T.map(QPointF(vis.right(), y)));

    // Floor
    if (!top) {
        const QPointF a = T.map(QPointF(vis.left(), 0)), b = T.map(QPointF(vis.right(), 0));
        p.setPen(QPen(c.muted, 1.5));
        p.drawLine(a, b);
        p.setPen(QPen(Theme::soft(c.muted, 0.5), 1));
        for (double x = a.x() - 20; x < b.x(); x += 9) p.drawLine(QPointF(x, a.y() + 1), QPointF(x + 7, a.y() + 8));
    }

    // Workspace
    p.setPen(Qt::NoPen);
    p.setBrush(Theme::soft(c.info, c.dark ? 0.2 : 0.16));
    if (m_view != View::Front)
        for (const QPolygonF& q : side ? m_envSide : m_envTop) p.drawPolygon(T.map(q));

    // Parts
    for (const Shape& s : shapes()) {
        QColor fill = s.color;
        fill.setAlphaF(std::clamp(0.92 * s.opacity, 0.15, 1.0));
        // Outline half way to the text colour, so white parts show on a light background
        QColor edge = QColor::fromRgbF((s.color.redF() + c.text.redF()) / 2, (s.color.greenF() + c.text.greenF()) / 2,
                                       (s.color.blueF() + c.text.blueF()) / 2);
        edge.setAlphaF(std::clamp(0.9 * s.opacity, 0.2, 1.0));
        p.setPen(QPen(edge, 1));
        p.setBrush(fill);
        p.drawPolygon(T.map(s.poly));
    }

    // Kinematic chain
    p.setPen(QPen(c.accent, 2.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.setBrush(Qt::NoBrush);
    for (const JointMark& j : m_joints)
        if (j.prev >= 0) p.drawLine(T.map(project(m_joints[j.prev].point)), T.map(project(j.point)));
    for (const TipMark& t : m_tips)
        if (t.prev >= 0) p.drawLine(T.map(project(m_joints[t.prev].point)), T.map(project(t.point)));
    QFont small = font();
    small.setPointSizeF(font().pointSizeF() * 0.88);
    small.setBold(true);
    p.setFont(small);
    for (const JointMark& j : m_joints) {
        const QPointF P = T.map(project(j.point));
        const QPointF dir = side ? QPointF(j.axis.x(), -j.axis.y())     // screen y down
                          : top  ? QPointF(j.axis.x(), j.axis.z())
                                 : QPointF(-j.axis.z(), -j.axis.y());
        const double inPlane = std::hypot(dir.x(), dir.y());
        if (j.passive) {                                   // follows another joint (coupling, levelling link)
            p.setPen(QPen(c.muted, 1.4));
            p.setBrush(c.viewport);
            p.drawEllipse(P, 4.5, 4.5);
            continue;
        }
        p.setPen(QPen(c.accent, 1.8));
        p.setBrush(c.panel);
        if (j.prismatic) {
            p.drawRect(QRectF(P - QPointF(6, 6), QSizeF(12, 12)));
            if (inPlane > 0.3) {
                const QPointF u = dir / inPlane;
                p.drawLine(P + u * 8, P + u * 20);
                const QPointF n(-u.y(), u.x());
                p.drawLine(P + u * 20, P + u * 15 + n * 4);
                p.drawLine(P + u * 20, P + u * 15 - n * 4);
            }
        } else if (inPlane < 0.35) {
            p.drawEllipse(P, 7, 7);                       // turns in the drawing's plane
            p.setBrush(c.accent);
            p.drawEllipse(P, 2.2, 2.2);
        } else {
            const QPointF u = dir / inPlane;              // turns about an axis in the plane
            p.setPen(QPen(c.accent, 1, Qt::DashLine));
            p.drawLine(P - u * 22, P + u * 22);
            p.setPen(QPen(c.accent, 1.8));
            const double ang = std::atan2(u.y(), u.x()) * 180 / M_PI;
            p.save();
            p.translate(P);
            p.rotate(ang);
            p.drawRoundedRect(QRectF(-9, -5, 18, 10), 2, 2);
            p.restore();
        }
    }
    // Names, one label for joints that coincide in this view (A1 · A2 when the shoulder sits on the base axis)
    std::vector<std::pair<QPointF, QStringList>> labels;
    for (const JointMark& j : m_joints) {
        if (j.passive || !m_jointLabels) continue;
        const QPointF P = T.map(project(j.point));
        auto it = std::find_if(labels.begin(), labels.end(), [&](const auto& l) {
            return std::hypot(l.first.x() - P.x(), l.first.y() - P.y()) < 4;
        });
        if (it == labels.end()) labels.push_back({P, {j.name}});
        else it->second << j.name;
    }
    p.setPen(c.text);
    for (const auto& [P, names] : labels) p.drawText(P + QPointF(10, -9), names.join(" · "));
    for (const TipMark& t : m_tips) {
        if (t.label.isEmpty()) continue;
        const QPointF P = T.map(project(t.point));
        p.setPen(QPen(c.warning, 1.8));
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(P, 5, 5);
        p.drawLine(P - QPointF(9, 0), P + QPointF(9, 0));
        p.drawLine(P - QPointF(0, 9), P + QPointF(0, 9));
        p.setPen(c.warning);
        p.drawText(P + QPointF(9, 14), t.label);
    }

    // Dimensions
    QFont dimFont = font();
    dimFont.setPointSizeF(font().pointSizeF() * 0.9);
    dimFont.setBold(true);
    p.setFont(dimFont);
    const QFontMetricsF fm(dimFont);
    for (const robot::DimensionLine& d : m_dims) {
        if (viewOf(d) != m_view) continue;
        const QPointF p1 = project(d.from), p2 = project(d.to);
        QPointF sd = project(d.offsetDir) - project(QVector3D());
        const double sl = std::hypot(sd.x(), sd.y());
        if (sl < 1e-9) continue;
        sd /= sl;
        auto dot = [](QPointF a, QPointF b) { return a.x() * b.x() + a.y() * b.y(); };
        const double s1 = dot(p1, sd), s2 = dot(p2, sd), s = std::max(s1, s2) + d.offset;
        const QPointF e1 = p1 + sd * (s - s1), e2 = p2 + sd * (s - s2);
        const QPointF delta = (p2 - p1) - sd * dot(p2 - p1, sd);
        const double value = std::hypot(delta.x(), delta.y());
        if (value < 1e-6) continue;
        const bool hot = d.symbol == m_highlight;
        const QColor col = hot ? c.warning : c.info;
        const QPointF P1 = T.map(p1), P2 = T.map(p2), E1 = T.map(e1), E2 = T.map(e2);
        const QPointF n = (E1 - P1) / std::max(1.0, std::hypot((E1 - P1).x(), (E1 - P1).y()));
        const QPointF n2 = (E2 - P2) / std::max(1.0, std::hypot((E2 - P2).x(), (E2 - P2).y()));
        p.setPen(QPen(Theme::soft(col, 0.7), 1, Qt::DashLine));
        p.drawLine(P1 + n * 4, E1 + n * 6);
        p.drawLine(P2 + n2 * 4, E2 + n2 * 6);
        p.setPen(QPen(col, hot ? 2.2 : 1.4));
        p.drawLine(E1, E2);
        const QPointF u = (E2 - E1) / std::hypot((E2 - E1).x(), (E2 - E1).y());
        const QPointF w(-u.y(), u.x());
        p.setBrush(col);
        for (auto [tip, dirv] : {std::pair{E1, u}, {E2, -u}}) {
            QPolygonF arrow;
            arrow << tip << tip + dirv * 9 + w * 3.5 << tip + dirv * 9 - w * 3.5;
            p.drawPolygon(arrow);
        }
        const QString text = QString("%1 = %2").arg(d.symbol, mm(value));
        const QPointF mid = (E1 + E2) / 2;
        QRectF box(0, 0, fm.horizontalAdvance(text) + 12, fm.height() + 4);
        box.moveCenter(mid);
        p.setPen(QPen(hot ? c.warning : c.border, 1));
        p.setBrush(c.panel);
        p.drawRoundedRect(box, 6, 6);
        p.setPen(hot ? c.warning : c.text);
        p.drawText(box, Qt::AlignCenter, text);
        m_hits.push_back({box, d.symbol});
    }

    // Scale bar and view name
    p.setFont(font());
    const double barPx = step * scale;
    const QPointF b0(14, height() - 16);
    p.setPen(QPen(c.text, 1.5));
    p.drawLine(b0, b0 + QPointF(barPx, 0));
    p.drawLine(b0 + QPointF(0, -4), b0 + QPointF(0, 4));
    p.drawLine(b0 + QPointF(barPx, -4), b0 + QPointF(barPx, 4));
    p.setPen(c.muted);
    p.drawText(b0 + QPointF(barPx + 6, 4), QString("%1 mm").arg(mm(step)));
    p.drawText(QRectF(0, 8, width() - 12, 20), Qt::AlignRight,
               side ? "SIDE VIEW  ·  X → , Y ↑" : top ? "TOP VIEW  ·  X → , Z ↓" : "FRONT VIEW  ·  −Z → , Y ↑");
}

void RobotSketch::wheelEvent(QWheelEvent* e) {
    const double k = std::pow(1.15, e->angleDelta().y() / 120.0);
    const double nz = std::clamp(m_zoom * k, 0.3, 30.0);
    const QPointF center(width() / 2.0, height() / 2.0);
    const QPointF q = e->position();
    m_pan = q - center - (q - center - m_pan) * (nz / m_zoom);
    m_zoom = nz;
    update();
}

void RobotSketch::mousePressEvent(QMouseEvent* e) {
    for (const Hit& h : m_hits)
        if (h.rect.contains(e->position())) { emit dimensionClicked(h.symbol); return; }
    if (e->button() == Qt::LeftButton || e->button() == Qt::MiddleButton) {
        m_dragging = true;
        m_dragFrom = e->pos();
        setCursor(Qt::ClosedHandCursor);
    }
}

void RobotSketch::mouseMoveEvent(QMouseEvent* e) {
    if (m_dragging) {
        m_pan += e->pos() - m_dragFrom;
        m_dragFrom = e->pos();
        update();
        return;
    }
    bool overLabel = false;
    for (const Hit& h : m_hits) if (h.rect.contains(e->position())) overLabel = true;
    setCursor(overLabel ? Qt::PointingHandCursor : Qt::OpenHandCursor);
}

void RobotSketch::mouseReleaseEvent(QMouseEvent*) {
    m_dragging = false;
    setCursor(Qt::OpenHandCursor);
}

void RobotSketch::mouseDoubleClickEvent(QMouseEvent*) {
    m_zoom = 1;
    m_pan = {};
    update();
}
