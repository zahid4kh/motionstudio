#pragma once
#include <algorithm>
#include <cmath>

// Trapezoidal (or, with accel = 0, rectangular) velocity profile integrator
// for one actuated axis. Units are whatever the joint uses (deg or m).
namespace motion {

struct State {
    double pos = 0.0;
    double vel = 0.0;
};

inline double approach(double v, double target, double maxDelta) {
    if (v < target) return std::min(v + maxDelta, target);
    return std::max(v - maxDelta, target);
}

// Moves toward `target`, cruising at most at `vmax`, accelerating/braking at `accel`.
// Lands exactly on the target (no overshoot) and reports zero velocity there.
inline void stepPosition(State& s, double target, double vmax, double accel, double dt) {
    vmax = std::fabs(vmax);
    double dist = target - s.pos;
    if (vmax <= 0.0) {
        s.vel = accel > 0.0 ? approach(s.vel, 0.0, accel * dt) : 0.0;
        s.pos += s.vel * dt;
        return;
    }
    if (accel <= 0.0) {
        double step = vmax * dt;
        if (std::fabs(dist) <= step) { s.pos = target; s.vel = 0.0; return; }
        s.vel = dist > 0 ? vmax : -vmax;
        s.pos += s.vel * dt;
        return;
    }
    double dir    = dist > 0 ? 1.0 : -1.0;
    // Highest speed from which we can still brake to rest exactly at the target
    double vBrake = std::sqrt(2.0 * accel * std::fabs(dist));
    double vDes   = dir * std::min(vmax, vBrake);
    s.vel = approach(s.vel, vDes, accel * dt);
    double newPos = s.pos + s.vel * dt;
    // Braking near the target leaves only ~accel·dt of speed, so landing on
    // the target when this step would cross it is a negligible correction.
    if ((target - newPos) * dist <= 0.0) {
        s.pos = target;
        s.vel = 0.0;
        return;
    }
    s.pos = newPos;
}

// Runs at `speed` (signed), ramping at `accel` (0 = instant).
inline void stepSpeed(State& s, double speed, double accel, double dt) {
    s.vel = accel > 0.0 ? approach(s.vel, speed, accel * dt) : speed;
    s.pos += s.vel * dt;
}

// Duration of a point-to-point move of distance d with (vmax, accel).
inline double moveTime(double d, double vmax, double accel) {
    d = std::fabs(d);
    vmax = std::fabs(vmax);
    if (d <= 0.0) return 0.0;
    if (vmax <= 0.0) return 1e30;
    if (accel <= 0.0) return d / vmax;
    if (d >= vmax * vmax / accel) return d / vmax + vmax / accel;   // reaches cruise speed
    return 2.0 * std::sqrt(d / accel);                              // triangular profile
}

} // namespace motion
