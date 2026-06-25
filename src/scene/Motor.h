#pragma once
#include <QString>

struct Motor {
    int     id             = -1;
    QString name;
    float   velocity       = 45.0f;  // deg/s for revolute, units/s for prismatic
    float   targetPosition = 0.0f;
    int     jointId        = -1;
    bool    enabled        = true;
};
