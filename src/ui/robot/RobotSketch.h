#pragma once
#include <QMatrix4x4>
#include <QPolygonF>
#include <QWidget>
#include <vector>
#include "robot/RobotAnalysis.h"
#include "robot/RobotBuilder.h"

class SceneGraph;

// A 2D drawing of a robot in its own frame: side view (X right, Y up), top
// view (X right, Z down) or front view (looking at the robot's face: −Z
// right, Y up). It shows the parts' silhouettes, the kinematic
// chain with its joint axes, dimension lines with their values and the
// workspace the TCP reaches. Wheel zooms, drag pans, double-click refits.
class RobotSketch : public QWidget {
    Q_OBJECT
public:
    enum class View { Side, Top, Front };

    explicit RobotSketch(QWidget* parent = nullptr);

    void setView(View v);
    View view() const { return m_view; }
    // Captures the parts under `baseFrame` and every joint between it and
    // `tcpId` (actuated or following another) as they are now, in the base
    // frame's coordinates
    void capture(const SceneGraph& g, int baseFrame, int tcpId);
    // The same for several end points (a humanoid's hands and feet): the
    // joints of every chain from the base to a tip. Tips with a label are
    // marked.
    struct Tip { int id; QString label; };
    void capture(const SceneGraph& g, int baseFrame, const std::vector<Tip>& tips);
    void clear();
    void setDimensions(const std::vector<robot::DimensionLine>& d);
    void setEnvelope(const robot::Analysis* a, const SceneGraph* g);   // null hides it
    void setHighlight(const QString& symbol);
    void setMessage(const QString& text);   // shown instead of a drawing when nothing is captured
    void setJointLabels(bool on);           // names next to the joints (default on)

    QSize sizeHint() const override { return {460, 380}; }
    QSize minimumSizeHint() const override { return {240, 200}; }

signals:
    void dimensionClicked(const QString& symbol);

protected:
    void paintEvent(QPaintEvent*) override;
    void wheelEvent(QWheelEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void mouseDoubleClickEvent(QMouseEvent* e) override;

private:
    struct Shape { QPolygonF poly; QColor color; float opacity = 1; double depth = 0; };
    struct JointMark { QVector3D point, axis; QString name; bool prismatic = false; bool passive = false; int prev = -1; };
    struct TipMark { QVector3D point; QString label; int prev = -1; };

    QPointF project(const QVector3D& p) const;   // robot frame → drawing (y up)
    QRectF  contentBounds() const;
    QTransform toScreen() const;
    const std::vector<Shape>& shapes() const { return m_view == View::Side ? m_side : m_view == View::Top ? m_top : m_front; }

    View m_view = View::Side;
    std::vector<Shape> m_side, m_top, m_front;
    std::vector<JointMark> m_joints;
    std::vector<TipMark> m_tips;
    bool m_jointLabels = true;
    std::vector<robot::DimensionLine> m_dims;
    std::vector<QPolygonF> m_envSide, m_envTop;   // drawing coordinates
    QString m_highlight;
    QString m_message;
    double m_zoom = 1;
    QPointF m_pan;
    QPoint m_dragFrom;
    bool m_dragging = false;
    struct Hit { QRectF rect; QString symbol; };
    std::vector<Hit> m_hits;
};
