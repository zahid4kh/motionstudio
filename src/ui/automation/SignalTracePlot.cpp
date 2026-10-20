#include "SignalTracePlot.h"
#include "ui/Theme.h"
#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <cmath>

SignalTracePlot::SignalTracePlot(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(90);
}

void SignalTracePlot::setTraces(const QStringList& names, const std::vector<bool>& isBool) {
    std::vector<Lane> lanes;
    for (int i = 0; i < names.size(); ++i) {
        Lane l;
        l.name = names[i];
        l.isBool = i < int(isBool.size()) && isBool[size_t(i)];
        // Keep existing history for traces that stay
        for (auto& old : m_lanes)
            if (old.name == l.name) l.samples = old.samples;
        lanes.push_back(std::move(l));
    }
    m_lanes = std::move(lanes);
    m_names = names;
    update();
}

void SignalTracePlot::addSample(double t, const std::vector<double>& values) {
    m_latest = t;
    for (size_t i = 0; i < m_lanes.size() && i < values.size(); ++i) {
        auto& s = m_lanes[i].samples;
        s.emplace_back(t, values[i]);
        while (!s.empty() && s.front().first < t - 120.0) s.pop_front();
    }
}

void SignalTracePlot::clear() {
    for (auto& l : m_lanes) l.samples.clear();
    m_latest = 0;
    update();
}

void SignalTracePlot::paintEvent(QPaintEvent*) {
    const Theme::Colors& c = Theme::colors();
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), c.island);
    QFont small = font();
    small.setPointSizeF(std::max(7.0, small.pointSizeF() - 1));
    p.setFont(small);

    if (m_lanes.empty()) {
        p.setPen(c.muted);
        p.drawText(rect().adjusted(8, 0, -8, 0), Qt::AlignCenter | Qt::TextWordWrap,
                   "Tick “Plot” next to signals in the watch list to trace them here.");
        return;
    }

    const int labelW = 120, axisH = 14;
    const int n = int(m_lanes.size());
    const double laneH = double(height() - axisH) / n;
    const double t1 = std::max(m_latest, m_window), t0 = t1 - m_window;
    const int x0 = labelW, x1 = width() - 6;
    auto X = [&](double t) { return x0 + (x1 - x0) * (t - t0) / m_window; };

    // Time axis
    double step = m_window <= 10 ? 1 : (m_window <= 30 ? 5 : 10);
    for (double t = std::ceil(t0 / step) * step; t <= t1 + 1e-9; t += step) {
        int x = int(X(t));
        p.setPen(c.border);
        p.drawLine(x, 0, x, height() - axisH);
        p.setPen(c.muted);
        p.drawText(QRect(x - 25, height() - axisH, 50, axisH), Qt::AlignCenter, QString("%1").arg(t, 0, 'g', 4));
    }

    for (int i = 0; i < n; ++i) {
        const Lane& l = m_lanes[size_t(i)];
        const QColor col = c.series[size_t(i) % c.series.size()];
        double top = i * laneH, bot = (i + 1) * laneH;
        if (i % 2) p.fillRect(QRectF(0, top, width(), laneH), c.island.lighter(c.dark ? 118 : 104));
        p.setPen(c.border);
        p.drawLine(QPointF(0, bot), QPointF(width(), bot));

        // Visible range
        double lo = 1e300, hi = -1e300, last = 0;
        bool any = false;
        for (auto& [t, v] : l.samples) {
            if (t < t0) continue;
            lo = std::min(lo, v); hi = std::max(hi, v); last = v; any = true;
        }
        if (l.isBool) { lo = 0; hi = 1; }
        if (!any) { lo = 0; hi = 1; }
        if (hi - lo < 1e-9) { lo -= 0.5; hi += 0.5; }
        double pad = l.isBool ? 0.2 : 0.1;
        double range = hi - lo;
        lo -= range * pad; hi += range * pad;
        auto Y = [&](double v) { return bot - 3 - (laneH - 6) * (v - lo) / (hi - lo); };

        p.setPen(col);
        QString label = l.name;
        p.drawText(QRectF(6, top, labelW - 10, laneH / 2 + 4), Qt::AlignLeft | Qt::AlignBottom,
                   p.fontMetrics().elidedText(label, Qt::ElideRight, labelW - 10));
        p.setPen(c.text);
        QString val = !any ? "—" : (l.isBool ? (last != 0 ? "true" : "false") : QString::number(last, 'f', 3));
        p.drawText(QRectF(6, top + laneH / 2, labelW - 10, laneH / 2), Qt::AlignLeft | Qt::AlignTop, val);
        if (!l.isBool && any) {
            p.setPen(c.muted);
            p.drawText(QRectF(x0 + 2, top + 1, 80, 12), Qt::AlignLeft, QString::number(hi - range * pad, 'g', 4));
            p.drawText(QRectF(x0 + 2, bot - 13, 80, 12), Qt::AlignLeft, QString::number(lo + range * pad, 'g', 4));
        }

        QPainterPath path;
        bool started = false;
        double prevV = 0;
        for (auto& [t, v] : l.samples) {
            if (t < t0) { prevV = v; continue; }
            if (!started) {
                path.moveTo(X(std::max(t0, t)), Y(l.isBool ? prevV : v));
                started = true;
            }
            if (l.isBool) path.lineTo(X(t), Y(prevV));   // step
            path.lineTo(X(t), Y(v));
            prevV = v;
        }
        p.setPen(QPen(col, 1.6));
        p.setBrush(Qt::NoBrush);
        p.drawPath(path);
    }
}
