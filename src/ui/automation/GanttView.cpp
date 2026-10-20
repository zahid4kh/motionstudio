#include "GanttView.h"
#include <QMouseEvent>
#include <QPainter>
#include <algorithm>
#include <cmath>

GanttView::GanttView(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(80);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMouseTracking(false);
}

void GanttView::setBars(std::vector<Bar> bars, double now, bool live) {
    m_bars = std::move(bars);
    m_now  = now;
    m_live = live;
    setMinimumHeight(std::max(80, kHeader + int(m_bars.size()) * kRowH + 6));
    update();
}

int GanttView::rowAt(int y) const {
    if (y < kHeader) return -1;
    int r = (y - kHeader) / kRowH;
    return r < int(m_bars.size()) ? r : -1;
}

void GanttView::mousePressEvent(QMouseEvent* e) {
    int r = rowAt(int(e->position().y()));
    if (r >= 0) emit rowClicked(r);
}

static double niceStep(double range, int targetTicks) {
    double raw = range / std::max(1, targetTicks);
    double mag = std::pow(10.0, std::floor(std::log10(std::max(raw, 1e-9))));
    for (double m : {1.0, 2.0, 5.0, 10.0})
        if (raw <= m * mag) return m * mag;
    return 10 * mag;
}

void GanttView::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor("#ffffff"));
    QFont small = font();
    small.setPointSizeF(std::max(7.0, small.pointSizeF() - 1));
    p.setFont(small);

    if (m_bars.empty()) {
        p.setPen(QColor("#99a"));
        p.drawText(rect(), Qt::AlignCenter, "No operations");
        return;
    }

    double tMax = 1.0;
    for (const auto& b : m_bars) {
        if (b.end > tMax) tMax = b.end;
        if (b.start > tMax) tMax = b.start;
    }
    if (m_live) tMax = std::max(tMax, m_now);
    tMax *= 1.08;

    const int x0 = kLabelW, x1 = width() - 8;
    auto X = [&](double t) { return x0 + (x1 - x0) * t / tMax; };

    // Time grid
    double step = niceStep(tMax, std::max(2, (x1 - x0) / 70));
    p.setPen(QColor("#e4e8f0"));
    for (double t = 0; t <= tMax + 1e-9; t += step) {
        int x = int(X(t));
        p.setPen(QColor("#e4e8f0"));
        p.drawLine(x, kHeader - 2, x, height());
        p.setPen(QColor("#889"));
        p.drawText(QRect(x - 30, 0, 60, kHeader - 2), Qt::AlignCenter, QString("%1 s").arg(t, 0, 'g', 4));
    }

    bool anyTiming = false;
    for (int i = 0; i < int(m_bars.size()); ++i) {
        const Bar& b = m_bars[size_t(i)];
        int y = kHeader + i * kRowH;
        if (i == m_selected) p.fillRect(0, y, width(), kRowH, QColor("#e6f0fc"));
        else if (i % 2) p.fillRect(0, y, width(), kRowH, QColor("#f8f9fc"));

        p.setPen(QColor("#334"));
        p.drawText(QRect(6, y, kLabelW - 10, kRowH), Qt::AlignVCenter | Qt::AlignLeft,
                   p.fontMetrics().elidedText(b.label, Qt::ElideRight, kLabelW - 10));

        if (b.start < 0) continue;
        anyTiming = true;
        double end = b.end >= 0 ? b.end : (m_live ? m_now : b.start);
        QRectF r(X(b.start), y + 4, std::max(3.0, X(end) - X(b.start)), kRowH - 8);
        QColor fill = b.state == 2 ? QColor("#58b368") : (b.state == 1 ? QColor("#3d8fe0") : QColor("#c0c8d8"));
        if (b.fromPreviousCycle) fill.setAlpha(110);
        p.setPen(Qt::NoPen);
        p.setBrush(fill);
        p.drawRoundedRect(r, 3, 3);
        if (b.state == 1 && b.progress > 0 && b.progress < 1) {
            p.setBrush(QColor(255, 255, 255, 90));
            QRectF pr = r;
            pr.setWidth(r.width() * b.progress);
            p.drawRoundedRect(pr, 3, 3);
        }
    }

    if (m_live) {
        p.setPen(QPen(QColor("#d04040"), 1.2));
        int x = int(X(m_now));
        p.drawLine(x, kHeader - 2, x, kHeader + int(m_bars.size()) * kRowH);
    }
    if (!anyTiming) {
        p.setPen(QColor("#99a"));
        p.drawText(QRect(x0, kHeader, x1 - x0, int(m_bars.size()) * kRowH), Qt::AlignCenter,
                   "Press Play to see operation timing");
    }
}
