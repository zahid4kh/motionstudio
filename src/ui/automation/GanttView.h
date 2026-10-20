#pragma once
#include <QWidget>
#include <vector>

// Lane chart of a sequence's operations against time within the current (or
// last completed) cycle, with live progress while the simulation runs.
class GanttView : public QWidget {
    Q_OBJECT
public:
    struct Bar {
        QString label;
        double  start = -1, end = -1;     // seconds from cycle start; end < 0 = still running
        int     state = 0;                // 0 waiting, 1 running, 2 done
        double  progress = 0;
        bool    fromPreviousCycle = false;
        QString detail;
    };

    explicit GanttView(QWidget* parent = nullptr);
    void setBars(std::vector<Bar> bars, double now, bool live);
    void setSelectedRow(int row) { m_selected = row; update(); }

signals:
    void rowClicked(int row);

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent* e) override;
    QSize sizeHint() const override { return {400, 140}; }

private:
    int rowAt(int y) const;

    std::vector<Bar> m_bars;
    double m_now = 0;
    bool   m_live = false;
    int    m_selected = -1;
    static constexpr int kRowH = 20, kHeader = 18, kLabelW = 130;
};
