#include "Recording.h"
#include "../scene/SceneGraph.h"
#include <algorithm>

namespace {
bool same(float a, float b) { return a == b; }
bool same(double a, double b) { return a == b; }
bool same(const Transform& a, const Transform& b) {
    return a.position == b.position && a.rotation == b.rotation && a.scale == b.scale;
}

// Appends `v` at `frame` unless it equals the last recorded value
template <typename Track, typename T>
void record(Track& track, int frame, const T& v) {
    if (!track.values.empty() && same(track.values.back(), v)) return;
    track.frames.push_back(frame);
    track.values.push_back(v);
}
} // namespace

template <typename T>
const T* Recording::Track<T>::at(int frame) const {
    auto it = std::upper_bound(frames.begin(), frames.end(), frame);
    if (it == frames.begin()) return nullptr;
    return &values[size_t(it - frames.begin() - 1)];
}

void Recording::clear() {
    m_times.clear();
    m_joints.clear();
    m_objects.clear();
    m_parked.clear();
    m_slots.clear();
}

double Recording::time(int frame) const {
    if (m_times.empty()) return 0.0;
    return m_times[size_t(std::clamp(frame, 0, frames() - 1))];
}

int Recording::frameAt(double t) const {
    auto it = std::upper_bound(m_times.begin(), m_times.end(), t + 1e-9);
    return std::max(0, int(it - m_times.begin()) - 1);
}

void Recording::capture(const SceneGraph& scene, double time, const std::vector<double>& slotValues) {
    if (full()) return;
    const int f = frames();
    m_times.push_back(time);
    for (auto& [id, j] : scene.allJoints()) record(m_joints[id], f, j.currentValue);
    for (auto& [id, o] : scene.allObjects()) {
        if (o.parked && !m_parked.count(id)) {   // parked from the start: a template
            record(m_parked[id], f, 1.0f);
            continue;
        }
        record(m_objects[id], f, o.transform);
        record(m_parked[id], f, o.parked ? 1.0f : 0.0f);
    }
    if (m_slots.size() < slotValues.size()) m_slots.resize(slotValues.size());
    for (size_t i = 0; i < slotValues.size(); ++i) record(m_slots[i], f, slotValues[i]);
}

void Recording::apply(SceneGraph& scene, int frame) const {
    for (auto& [id, track] : m_joints)
        if (Joint* j = scene.getJoint(id))
            if (const float* v = track.at(frame)) j->currentValue = *v;
    for (auto& [id, track] : m_objects)
        if (SceneObject* o = scene.getObject(id))
            if (const Transform* t = track.at(frame)) o->transform = *t;
    for (auto& [id, track] : m_parked)
        if (SceneObject* o = scene.getObject(id)) {
            const float* p = track.at(frame);
            o->parked = p ? *p != 0.0f : true;   // not made yet
        }
}

double Recording::value(int slot, int frame) const {
    if (slot < 0 || slot >= int(m_slots.size())) return 0.0;
    const double* v = m_slots[size_t(slot)].at(frame);
    return v ? *v : 0.0;
}
