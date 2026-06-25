#pragma once
#include <QVector3D>

struct ConveyorBelt {
    float     velocity  = 1.0f;           // m/s
    QVector3D direction { 1.0f, 0, 0 };   // local-space push direction
    bool      enabled   = true;
};
