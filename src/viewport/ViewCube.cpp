#include "ViewCube.h"

#include "Camera.h"
#include "ui/Theme.h"

#include <QHelpEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QToolTip>

#include <cmath>

namespace {

constexpr int kWidth = 172;
constexpr int kHeight = 182;
constexpr float kHalf = 32.0f;       // half the cube's edge, in pixels
constexpr float kAxisLength = 82.0f; // the axis lines from the cube's origin corner
// A face is drawn in a 100 x 100 square (-50..50) mapped onto it; the bands outside +-kInner are its edges
// and corners.
constexpr double kInner = 32.0;
constexpr double kArrowGap = 13.0; // from the face-on cube's side to an arrow's base
constexpr double kArrowLength = 8.0;

struct Face
{
    QVector3D normal, right, up; // up and right as the label reads
    const char *label;
};

const Face kFaces[] = {
    {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}, "FRONT"},
    {{0, 0, -1}, {-1, 0, 0}, {0, 1, 0}, "BACK"},
    {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}, "RIGHT"},
    {{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}, "LEFT"},
    {{0, 1, 0}, {1, 0, 0}, {0, 0, -1}, "TOP"},
    {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}, "BOTTOM"},
};

QColor mix(const QColor &a, const QColor &b, double t)
{
    return QColor::fromRgbF(float(a.redF() + (b.redF() - a.redF()) * t), float(a.greenF() + (b.greenF() - a.greenF()) * t),
                            float(a.blueF() + (b.blueF() - a.blueF()) * t));
}

// The world axis (with sign) nearest to v.
QVector3D snapToAxis(const QVector3D &v)
{
    const float ax = std::abs(v.x()), ay = std::abs(v.y()), az = std::abs(v.z());
    if (ax >= ay && ax >= az)
        return QVector3D(v.x() > 0 ? 1 : -1, 0, 0);
    if (ay >= az)
        return QVector3D(0, v.y() > 0 ? 1 : -1, 0);
    return QVector3D(0, 0, v.z() > 0 ? 1 : -1);
}

bool alongAxis(const QVector3D &v)
{
    return std::max({std::abs(v.x()), std::abs(v.y()), std::abs(v.z())}) > 0.9999f;
}

} // namespace

ViewCube::ViewCube(QWidget *parent)
    : QWidget(parent)
{
    setFixedSize(kWidth, kHeight);
    setMouseTracking(true);
    // Only the cube, its arrows and buttons take the mouse; the rest of the corner stays the 3D view's.
    const QPointF o = centre();
    QRegion region(QRect(int(o.x() - 82), int(o.y() - 82), 164, 164), QRegion::Ellipse);
    for (Part part : {Part::Home, Part::RollClockwise, Part::RollCounterClockwise})
        region += iconRect(part).adjusted(-3, -3, 3, 3).toAlignedRect();
    setMask(region);
}

void ViewCube::setCameraRotation(const QQuaternion &rotation)
{
    m_right = rotation.rotatedVector(QVector3D(1, 0, 0));
    m_up = rotation.rotatedVector(QVector3D(0, 1, 0));
    m_back = rotation.rotatedVector(QVector3D(0, 0, 1));
    // The arrows come and go with face views, so what is under the cursor may change.
    if (m_hover.part != Part::None && !m_dragging)
        setHover(hitAt(mapFromGlobal(QCursor::pos())));
    update();
}

QPointF ViewCube::centre() const
{
    return QPointF(kWidth / 2.0, 94.0);
}

QPointF ViewCube::toScreen(const QVector3D &v) const
{
    return QPointF(QVector3D::dotProduct(v, m_right), -QVector3D::dotProduct(v, m_up));
}

bool ViewCube::faceView() const
{
    return alongAxis(m_back) && alongAxis(m_up);
}

QPolygonF ViewCube::arrow(Part part) const
{
    QPointF v;
    switch (part) {
    case Part::Up: v = QPointF(0, -1); break;
    case Part::Down: v = QPointF(0, 1); break;
    case Part::Left: v = QPointF(-1, 0); break;
    case Part::Right: v = QPointF(1, 0); break;
    default: return {};
    }
    const QPointF w(-v.y(), v.x());
    const QPointF base = centre() + v * (kHalf + kArrowGap);
    return QPolygonF({base + w * 7.0, base + v * kArrowLength, base - w * 7.0});
}

QRectF ViewCube::iconRect(Part part) const
{
    const QPointF o = centre();
    switch (part) {
    case Part::Home: return QRectF(6, 6, 18, 18);
    case Part::RollCounterClockwise: return QRectF(o.x() + 34, o.y() - 84, 18, 18);
    case Part::RollClockwise: return QRectF(o.x() + 56, o.y() - 84, 18, 18);
    default: return {};
    }
}

ViewCube::Hit ViewCube::hitAt(const QPointF &pos) const
{
    Hit hit;
    if (iconRect(Part::Home).adjusted(-3, -3, 3, 3).contains(pos)) {
        hit.part = Part::Home;
        return hit;
    }
    if (faceView()) {
        for (Part part : {Part::RollClockwise, Part::RollCounterClockwise}) {
            if (iconRect(part).adjusted(-3, -3, 3, 3).contains(pos)) {
                hit.part = part;
                return hit;
            }
        }
        for (Part part : {Part::Up, Part::Down, Part::Left, Part::Right}) {
            if (arrow(part).boundingRect().adjusted(-4, -4, 4, 4).contains(pos)) {
                hit.part = part;
                return hit;
            }
        }
    }
    const QPointF o = centre();
    for (const Face &f : kFaces) {
        if (QVector3D::dotProduct(f.normal, m_back) <= 1e-3f)
            continue;
        const QPointF n = toScreen(f.normal * kHalf), r = toScreen(f.right * kHalf), d = toScreen(-f.up * kHalf);
        const QTransform t(r.x() / 50, r.y() / 50, d.x() / 50, d.y() / 50, o.x() + n.x(), o.y() + n.y());
        bool ok = false;
        const QTransform inverse = t.inverted(&ok);
        if (!ok)
            continue;
        const QPointF q = inverse.map(pos);
        if (std::abs(q.x()) > 50 || std::abs(q.y()) > 50)
            continue;
        const int i = q.x() < -kInner ? -1 : q.x() > kInner ? 1 : 0;
        const int j = q.y() < -kInner ? 1 : q.y() > kInner ? -1 : 0; // up the face
        hit.part = Part::Cube;
        hit.direction = f.normal + f.right * float(i) + f.up * float(j);
        return hit;
    }
    return hit;
}

void ViewCube::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const Theme::Colors &c = Theme::colors();
    const QPointF o = centre();

    // The axes run from the cube's origin corner along its edges and beyond: drawn (with their names) under the
    // cube, then again over it along the edges that face the viewer.
    const QVector3D corner = QVector3D(-1, -1, -1) * kHalf;
    const struct
    {
        QVector3D axis;
        QColor color;
        const char *name;
    } axes[] = {{{1, 0, 0}, c.axisX, "X"}, {{0, 1, 0}, c.axisY, "Y"}, {{0, 0, 1}, c.axisZ, "Z"}};
    auto axisPen = [](const QColor &color) {
        QPen pen(color, 2.0);
        pen.setCapStyle(Qt::RoundCap);
        return pen;
    };
    for (const auto &a : axes) {
        if (std::abs(QVector3D::dotProduct(a.axis, m_back)) > 0.97f)
            continue; // pointing at the viewer: a dot
        p.setPen(axisPen(a.color));
        const QPointF end = o + toScreen(corner + a.axis * kAxisLength);
        p.drawLine(o + toScreen(corner), end);
        QPointF along = toScreen(a.axis);
        along /= std::max(1e-6, std::hypot(along.x(), along.y()));
        QFont axisFont = font();
        axisFont.setBold(true);
        axisFont.setPixelSize(11);
        p.setFont(axisFont);
        p.drawText(QRectF(end + along * 8 - QPointF(8, 8), QSizeF(16, 16)), Qt::AlignCenter, QLatin1String(a.name));
    }

    QFont labelFont = font();
    labelFont.setBold(true);
    labelFont.setPixelSize(17);
    for (const Face &f : kFaces) {
        const float facing = QVector3D::dotProduct(f.normal, m_back);
        if (facing <= 1e-3f)
            continue;
        const QPointF n = toScreen(f.normal * kHalf), r = toScreen(f.right * kHalf), d = toScreen(-f.up * kHalf);
        p.setTransform(QTransform(r.x() / 50, r.y() / 50, d.x() / 50, d.y() / 50, o.x() + n.x(), o.y() + n.y()));
        // Faces turned away from the viewer are a little darker, so the cube reads as a solid.
        p.setPen(Qt::NoPen);
        p.setBrush(mix(c.panel, c.hoverStrong, 0.9 * (1.0 - facing)));
        p.drawRect(QRectF(-50, -50, 100, 100));
        if (m_hover.part == Part::Cube) {
            QColor hover = c.info;
            hover.setAlphaF(0.55f);
            p.setBrush(hover);
            const double edges[] = {-50, -kInner, kInner, 50};
            for (int i = -1; i <= 1; ++i) {
                for (int j = -1; j <= 1; ++j) {
                    const QVector3D dir = f.normal + f.right * float(i) + f.up * float(j);
                    if ((dir - m_hover.direction).lengthSquared() > 1e-4f)
                        continue;
                    p.drawRect(QRectF(QPointF(edges[i + 1], edges[1 - j]), QPointF(edges[i + 2], edges[2 - j])));
                }
            }
        }
        QPen outline(c.handle, 1.2);
        outline.setCosmetic(true);
        outline.setJoinStyle(Qt::MiterJoin);
        p.setPen(outline);
        p.setBrush(Qt::NoBrush);
        p.drawRect(QRectF(-50, -50, 100, 100));
        const QString label = QString::fromLatin1(f.label);
        QFont fitted = labelFont;
        const int width = QFontMetrics(fitted).horizontalAdvance(label);
        if (width > 88)
            fitted.setPixelSize(std::max(8, 17 * 88 / width));
        p.setFont(fitted);
        p.setPen(c.text);
        p.drawText(QRectF(-50, -50, 100, 100), Qt::AlignCenter, label);
        p.resetTransform();
    }

    for (const auto &a : axes) {
        if (std::abs(QVector3D::dotProduct(a.axis, m_back)) > 0.97f)
            continue;
        // The edge lies on the two faces whose normals are minus the other axes.
        bool seen = false;
        for (const auto &other : axes) {
            if (other.axis != a.axis && QVector3D::dotProduct(-other.axis, m_back) > 1e-3f)
                seen = true;
        }
        if (seen) {
            p.setPen(axisPen(a.color));
            p.drawLine(o + toScreen(corner), o + toScreen(corner + a.axis * (2 * kHalf)));
        }
    }

    if (faceView()) {
        p.setPen(Qt::NoPen);
        for (Part part : {Part::Up, Part::Down, Part::Left, Part::Right}) {
            p.setBrush(m_hover.part == part ? c.accent : c.muted);
            p.drawPolygon(arrow(part));
        }
        for (Part part : {Part::RollCounterClockwise, Part::RollClockwise}) {
            const QString name = part == Part::RollClockwise ? QStringLiteral("rotate-cw") : QStringLiteral("rotate-ccw");
            Theme::icon(name, m_hover.part == part ? Theme::Tint::Text : Theme::Tint::Muted)
                .paint(&p, iconRect(part).toRect());
        }
    }
    Theme::icon(QStringLiteral("house"), m_hover.part == Part::Home ? Theme::Tint::Text : Theme::Tint::Muted)
        .paint(&p, iconRect(Part::Home).toRect());
}

void ViewCube::activate(const Hit &hit)
{
    const QVector3D right = snapToAxis(m_right), up = snapToAxis(m_up), back = snapToAxis(m_back);
    switch (hit.part) {
    case Part::None: return;
    case Part::Home: emit homeRequested(); return;
    case Part::Cube: {
        const QVector3D dir = hit.direction.normalized();
        QVector3D screenUp(0, 1, 0);
        if (std::abs(dir.y()) > 0.999f) {
            // Straight down or up: keep what is at the top of the screen (or, coming from a side view, what was
            // ahead) at the top, along the nearest horizontal axis.
            QVector3D keep = m_up - dir * QVector3D::dotProduct(m_up, dir);
            if (keep.lengthSquared() < 0.01f)
                keep = m_back * (dir.y() > 0 ? -1.0f : 1.0f);
            keep.setY(0);
            screenUp = snapToAxis(keep.lengthSquared() > 1e-8f ? keep : QVector3D(0, 0, -1));
        }
        emit rotationRequested(Camera::lookRotation(dir, screenUp), true);
        return;
    }
    case Part::Up: emit rotationRequested(Camera::lookRotation(up, -back), false); return;
    case Part::Down: emit rotationRequested(Camera::lookRotation(-up, back), false); return;
    case Part::Right: emit rotationRequested(Camera::lookRotation(right, up), false); return;
    case Part::Left: emit rotationRequested(Camera::lookRotation(-right, up), false); return;
    // Clockwise turns the picture clockwise: what was up ends up on the right.
    case Part::RollClockwise: emit rotationRequested(Camera::lookRotation(back, -right), false); return;
    case Part::RollCounterClockwise: emit rotationRequested(Camera::lookRotation(back, right), false); return;
    }
}

void ViewCube::setHover(const Hit &hit)
{
    if (hit == m_hover)
        return;
    m_hover = hit;
    setCursor(hit.part == Part::None ? Qt::ArrowCursor : Qt::PointingHandCursor);
    update();
}

void ViewCube::mousePressEvent(QMouseEvent *event)
{
    const Hit hit = hitAt(event->position());
    if (event->button() != Qt::LeftButton || hit.part == Part::None) {
        event->ignore(); // the 3D view orbits / pans from here too
        return;
    }
    m_pressed = hit;
    m_pressPos = m_lastPos = event->position().toPoint();
    m_dragging = false;
}

void ViewCube::mouseMoveEvent(QMouseEvent *event)
{
    const QPoint pos = event->position().toPoint();
    if (event->buttons().testFlag(Qt::LeftButton) && m_pressed.part == Part::Cube) {
        // Dragging the cube orbits the view.
        if (!m_dragging && (pos - m_pressPos).manhattanLength() > 3) {
            m_dragging = true;
            setHover({});
            setCursor(Qt::ClosedHandCursor);
        }
        if (m_dragging)
            emit orbitRequested(float(pos.x() - m_lastPos.x()), float(pos.y() - m_lastPos.y()));
        m_lastPos = pos;
        return;
    }
    if (event->buttons() != Qt::NoButton && m_pressed.part == Part::None) {
        event->ignore();
        return;
    }
    setHover(hitAt(event->position()));
}

void ViewCube::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() != Qt::LeftButton || m_pressed.part == Part::None) {
        event->ignore();
        return;
    }
    const Hit pressed = m_pressed;
    m_pressed = {};
    if (m_dragging) {
        m_dragging = false;
        setHover(hitAt(event->position()));
        setCursor(m_hover.part == Part::None ? Qt::ArrowCursor : Qt::PointingHandCursor);
        return;
    }
    if (hitAt(event->position()) == pressed)
        activate(pressed);
}

void ViewCube::leaveEvent(QEvent *event)
{
    QWidget::leaveEvent(event);
    if (!m_dragging)
        setHover({});
}

bool ViewCube::event(QEvent *event)
{
    if (event->type() != QEvent::ToolTip)
        return QWidget::event(event);
    auto *help = static_cast<QHelpEvent *>(event);
    const Hit hit = hitAt(help->pos());
    QString text;
    switch (hit.part) {
    case Part::None: break;
    case Part::Home: text = QStringLiteral("Home view (isometric, fit all)"); break;
    case Part::Up: text = QStringLiteral("Turn to the face above"); break;
    case Part::Down: text = QStringLiteral("Turn to the face below"); break;
    case Part::Left: text = QStringLiteral("Turn to the face on the left"); break;
    case Part::Right: text = QStringLiteral("Turn to the face on the right"); break;
    case Part::RollClockwise: text = QStringLiteral("Rotate the view 90° clockwise"); break;
    case Part::RollCounterClockwise: text = QStringLiteral("Rotate the view 90° counter-clockwise"); break;
    case Part::Cube: {
        const QVector3D d = hit.direction;
        QStringList sides;
        if (d.y() != 0)
            sides << (d.y() > 0 ? QStringLiteral("top") : QStringLiteral("bottom"));
        if (d.z() != 0)
            sides << (d.z() > 0 ? QStringLiteral("front") : QStringLiteral("back"));
        if (d.x() != 0)
            sides << (d.x() > 0 ? QStringLiteral("right") : QStringLiteral("left"));
        text = sides.size() == 1 ? QStringLiteral("Look at the %1").arg(sides.join(QString()))
               : sides.size() == 2 ? QStringLiteral("Look at the %1 edge").arg(sides.join(QStringLiteral(" ")))
                                   : QStringLiteral("Isometric view from the %1 corner").arg(sides.join(QStringLiteral(" ")));
        text += QStringLiteral("<br><small>Drag the cube to orbit</small>");
        break;
    }
    }
    if (text.isEmpty())
        QToolTip::hideText();
    else
        QToolTip::showText(help->globalPos(), text, this);
    return true;
}
