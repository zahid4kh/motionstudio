#include "Island.h"
#include "Theme.h"
#include <QEvent>
#include <QPainter>
#include <QPainterPath>
#include <QVBoxLayout>

namespace {

constexpr qreal kRadius = 12;

// Covers its target, painting only the corners and the border. Follows the
// target's size and stays on top of children added later.
class Corners : public QWidget {
public:
    explicit Corners(QWidget* target) : QWidget(target) {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        target->installEventFilter(this);
        setGeometry(target->rect());
        raise();
    }

protected:
    bool eventFilter(QObject* obj, QEvent* e) override {
        if (obj == parent()) {
            switch (e->type()) {
                case QEvent::Resize:
                    setGeometry(parentWidget()->rect());
                    raise();
                    break;
                case QEvent::Show:
                case QEvent::ChildAdded:
                case QEvent::LayoutRequest:
                    raise();
                    break;
                default: break;
            }
        }
        return QWidget::eventFilter(obj, e);
    }

    void paintEvent(QPaintEvent*) override {
        const Theme::Colors& c = Theme::colors();
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        const QRectF r(rect());
        QPainterPath outer;
        outer.addRect(r);
        QPainterPath inner;
        inner.addRoundedRect(r, kRadius, kRadius);
        p.fillPath(outer.subtracted(inner), c.frame);
        p.setPen(QPen(c.border, 1));
        p.setBrush(Qt::NoBrush);
        p.drawRoundedRect(r.adjusted(0.5, 0.5, -0.5, -0.5), kRadius - 0.5, kRadius - 0.5);
    }
};

} // namespace

Island::Island(QWidget* content, QWidget* parent) : QWidget(parent) {
    setObjectName("island");
    setAttribute(Qt::WA_StyledBackground);
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(content);
    new Corners(this);
}

void Island::decorate(QWidget* target) {
    new Corners(target);
}
