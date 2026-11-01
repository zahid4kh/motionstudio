#pragma once
#include <QString>
#include <QVector3D>

struct ConveyorBelt {
    float     velocity  = 1.0f;           // m/s
    QVector3D direction { 1.0f, 0, 0 };   // local-space push direction
    bool      enabled   = true;
    // Optional expression evaluated every scan while simulating: the belt's
    // speed in m/s (overrides `velocity`), e.g. PartAtEnd ? 0 : 0.3
    QString   speedExpr;
};
