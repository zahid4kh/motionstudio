#pragma once
#include <QString>
#include <QVector3D>

enum class JointType { Revolute, Prismatic };

struct Joint {
    int       id            = -1;
    QString   name;
    JointType type          = JointType::Revolute;
    QVector3D axis          = {0.0f, 1.0f, 0.0f};
    float     limitMin      = -180.0f;
    float     limitMax      =  180.0f;
    float     initialAngle  = 0.0f;    // revolute only, degrees
    int       parentBodyId  = -1;
    int       childBodyId   = -1;

    // Rotation origin (revolute only), expressed in the parent body's local
    // frame. (0,0,0) reproduces the original behavior of rotating about the
    // parent's own origin.
    QVector3D pivot         = {0.0f, 0.0f, 0.0f};

    // Revolute only: ignore limitMin/limitMax and allow unlimited rotation
    // (wheels, turntables, orbits).
    bool      continuous    = false;

    // Coupling (gear, belt, rack and pinion, cam linkage): when `coupledTo`
    // is another joint, this one follows it: value = ratio · master + offset,
    // in this joint's units (deg or m). A follower has no actuator.
    int       coupledTo     = -1;
    float     ratio         = 1.0f;
    float     offset        = 0.0f;

    // Runtime state (not persisted directly; restored from motor)
    float     currentValue  = 0.0f;    // degrees for revolute, units for prismatic
};
