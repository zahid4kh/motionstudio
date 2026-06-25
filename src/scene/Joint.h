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

    // Runtime state (not persisted directly; restored from motor)
    float     currentValue  = 0.0f;    // degrees for revolute, units for prismatic
};
