#pragma once
#include <QWidget>

class SimulationEngine;
class Viewport;
class QSlider;
class QLabel;
class QPushButton;

// Strip under the viewport while a simulation is active: the recorded run as
// a scrub slider (dragging pauses and shows that moment in the scene), a
// button back to the latest scan, and video/GIF export of the run.
class TimelineBar : public QWidget {
    Q_OBJECT
public:
    TimelineBar(SimulationEngine* sim, Viewport* viewport, QWidget* parent = nullptr);

signals:
    void statusMessage(const QString& msg, bool ok);

private:
    void sync();
    void exportRun();

    SimulationEngine* m_sim;
    Viewport*         m_viewport;
    QLabel*           m_rec = nullptr;
    QSlider*          m_slider = nullptr;
    QLabel*           m_time = nullptr;
    QPushButton*      m_live = nullptr;
    QPushButton*      m_export = nullptr;
    bool              m_updating = false;
};
