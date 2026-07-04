#pragma once
#include <QString>
#include <QVector3D>
#include <memory>
#include <vector>
#include "../scene/SceneObject.h"

// Imports a STEP file (.step/.stp) into a tree of parts that mirrors the
// CAD assembly structure. Built on OpenCASCADE XCAF; when compiled without
// OCCT_ENABLED, importFile() fails with an explanatory error.
namespace StepImporter {

struct Part {
    QString   name;
    QVector3D position;                    // local, metres
    QVector3D rotationDeg;                 // local Euler XYZ, degrees
    std::shared_ptr<MeshGeometry> mesh;    // null for pure assembly nodes
    std::vector<Part> children;
};

bool available();

// On success fills rootOut (single root; synthetic root if the file has
// multiple top-level shapes). All geometry is converted to metres.
bool importFile(const QString& path, Part& rootOut, QString& errorOut);

} // namespace StepImporter
