#pragma once
#include <QString>
#include <QStringList>
#include <vector>

class AutomationEngine;
class SceneGraph;

// A take: what one robot did while the simulation ran, sampled into rows of
// numbers (for analysis, or to teach a model to move like it). Recorded by
// the app's Record Values button and the API's take.start / take.stop.
//
// Columns, in API units (mm, degrees, seconds):
//   t                        seconds since the take started
//   x y z rx ry rz           the robot's top object in the world (Euler XYZ)
//   heading walking step swing progress step_x step_z step_heading
//   goal_x goal_z goal_heading
//                            only for a robot with legs: where it faces, 1
//                            while it walks, the step under way (-1 none),
//                            its swinging foot (0 left, 1 right, -1 none),
//                            how far through the step (0..1), where the
//                            step puts that foot (the place it would stand
//                            at with that foot on its home spot; the goal
//                            between steps), and where the walk ends (its
//                            place when standing)
//   <actuator names>         each joint (degrees, or mm for sliding joints)
namespace take {

struct Take {
    QString robot;
    double  rate = 0;                         // samples per second
    QStringList columns;
    std::vector<std::vector<double>> rows;

    int     column(const QString& name) const { return int(columns.indexOf(name)); }
    double  duration() const { return rows.empty() ? 0.0 : rows.back().front(); }
    // CSV with "# key: value" header lines (numpy.loadtxt and pandas read it
    // with comments="#")
    QString toCsv() const;
    bool    save(const QString& path, QString* error = nullptr) const;
};

// Samples a robot after each scan
class Recorder {
public:
    // The robot at `root`, with its actuated `joints` (named after their
    // actuators); `rate` samples per second, 0 = every scan
    Recorder(SceneGraph& scene, int root, std::vector<int> joints, double rate = 0);

    // Call after a scan: keeps it when the rate asks for one
    void sample(const AutomationEngine& automation);
    const Take& take() const { return m_take; }
    Take&       take() { return m_take; }
    int  root() const { return m_root; }
    int  frames() const { return int(m_take.rows.size()); }

private:
    SceneGraph& m_scene;
    int    m_root;
    std::vector<int> m_joints;
    bool   m_legs = false;
    double m_start = -1, m_next = 0;
    Take   m_take;
};

} // namespace take
