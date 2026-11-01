#pragma once
#include <QWidget>
#include <QStringList>
#include <deque>
#include <vector>

// Scrolling multi-lane plot of signal values over simulation time. Each
// traced signal gets its own lane: booleans draw as a digital trace, numbers
// auto-scale to the visible window.
class SignalTracePlot : public QWidget {
    Q_OBJECT
public:
    explicit SignalTracePlot(QWidget* parent = nullptr);

    void setTraces(const QStringList& names, const std::vector<bool>& isBool);
    const QStringList& traces() const { return m_names; }
    void addSample(double t, const std::vector<double>& values);   // one value per trace
    void clear();
    void setWindow(double seconds) { m_window = seconds; update(); }
    // Marks a time (scrubbing the recorded run) and keeps it in view; < 0 = none
    void setPlayhead(double t) { if (t != m_playhead) { m_playhead = t; update(); } }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    struct Lane {
        QString name;
        bool    isBool = false;
        std::deque<std::pair<double, double>> samples;
    };
    std::vector<Lane> m_lanes;
    QStringList       m_names;
    double            m_window = 20.0;
    double            m_latest = 0.0;
    double            m_playhead = -1.0;
};
