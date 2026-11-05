#pragma once
#include <unordered_map>
#include <vector>
#include "../scene/SceneObject.h"

class SceneGraph;

// What a simulation run did, one frame per scan: joint values, object
// transforms and every automation slot value. Each is stored as a change
// log (frame, value), so only what moves costs memory; looking up a frame is
// a binary search. Used to scrub back through a run and to export it.
class Recording {
public:
    // 20 minutes at one frame per 16 ms scan; capture() stops after that
    static constexpr int kMaxFrames = 75000;

    void   clear();
    int    frames() const { return int(m_times.size()); }
    bool   full() const { return frames() >= kMaxFrames; }
    double time(int frame) const;
    double duration() const { return m_times.empty() ? 0.0 : m_times.back(); }
    // Last frame at or before `t` (0 when before the start)
    int    frameAt(double t) const;

    // Appends a frame with the scene's current state and the slot values
    void   capture(const SceneGraph& scene, double time, const std::vector<double>& slotValues);
    // Puts every recorded joint and object back to its state at `frame`; an
    // object made after `frame` (by a source) is parked
    void   apply(SceneGraph& scene, int frame) const;
    double value(int slot, int frame) const;

private:
    template <typename T>
    struct Track {
        std::vector<int> frames;
        std::vector<T>   values;
        const T* at(int frame) const;
    };

    std::vector<double>                       m_times;
    std::unordered_map<int, Track<float>>     m_joints;
    std::unordered_map<int, Track<Transform>> m_objects;
    std::unordered_map<int, Track<float>>     m_parked;     // 1 = parked
    std::vector<Track<double>>                m_slots;
};
