#pragma once
#include <QString>

// A Motor is the actuator driving one joint (shown as "Actuator" in the UI).
// It never moves on its own in Position mode: during simulation the
// AutomationEngine commands it — from operations in a sequence, or from the
// expression bindings below. A Speed-mode actuator that is enabled runs at
// `velocity` from the moment the simulation starts (conveyors, fans…).
//
// `name` doubles as the symbol prefix used in expressions (J1.pos, J1.inPosition…),
// so it must be a valid identifier and unique among actuators, signals and sequences.
enum class ActuatorMode { Position, Speed };

struct Motor {
    int          id           = -1;
    QString      name;
    int          jointId      = -1;

    ActuatorMode mode         = ActuatorMode::Position;
    float        velocity     = 45.0f;  // Position: default max speed. Speed: signed speed. deg/s or m/s
    float        acceleration = 0.0f;   // deg/s² or m/s²; 0 = instant velocity changes
    bool         enabled      = true;   // "active": when false the actuator holds still

    // Optional expression bindings, re-evaluated every scan. A non-empty
    // binding overrides operation commands for that property.
    QString      targetExpr;    // position target (forces Position mode)
    QString      velocityExpr;  // max speed (Position) or signed speed (Speed)
    QString      activeExpr;
};
