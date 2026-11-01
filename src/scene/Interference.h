#pragma once
#include <QMatrix4x4>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

class SceneGraph;

// Finds parts that pass through each other while mechanisms move: the "the
// robot clips the fixture" check, separate from physics contacts. Only
// parts that can move relative to each other are paired (different joints,
// or a joint and the static scene); rigid bodies are left to the physics.
// Pairs that already touch when the check starts are taken as assembled
// that way (a link and its housing) and ignored.
class InterferenceChecker {
public:
    struct Event {
        int    a = -1, b = -1;
        double start = 0, end = 0;   // seconds; end is updated while it lasts
        bool   ongoing = false;
    };

    InterferenceChecker();
    ~InterferenceChecker();   // Bvh is defined in the .cpp

    void   begin(const SceneGraph& scene);
    void   check(const SceneGraph& scene, double time);
    void   clear();
    bool   active() const { return m_active; }
    int    pairCount() const { return int(m_pairs.size()); }
    const std::vector<Event>& events() const { return m_events; }
    // Objects interfering at the last check
    const std::set<int>& interfering() const { return m_now; }

    // Exact test: do the two objects' triangles intersect in the current pose?
    bool   intersect(const SceneGraph& scene, int a, int b);

private:
    struct Bvh;
    const Bvh* bvhFor(const std::vector<float>* soup);

    bool m_active = false;
    std::vector<std::pair<int, int>> m_pairs;
    std::map<std::pair<int, int>, size_t> m_open;   // pair → index of its ongoing event
    std::vector<Event> m_events;
    std::set<int>      m_now;
    std::unordered_map<const std::vector<float>*, std::unique_ptr<Bvh>> m_bvhs;
};
