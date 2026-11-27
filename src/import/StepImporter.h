#pragma once
#include <QString>
#include <QVector3D>
#include <functional>
#include <memory>
#include <vector>
#include "../scene/SceneObject.h"

// Imports a STEP file (.step/.stp) into a tree of parts that mirrors the
// CAD assembly structure. Built on OpenCASCADE XCAF; when compiled without
// OCCT_ENABLED, importFile() fails with an explanatory error.
class SceneGraph;

namespace StepImporter {

struct Part {
    QString   name;
    QVector3D position;                    // local, metres
    QVector3D rotationDeg;                 // local Euler XYZ, degrees
    std::shared_ptr<MeshGeometry> mesh;    // null for pure assembly nodes
    QColor    color;                       // from the file's colours; invalid = none
    std::vector<Part> children;
};

bool available();

// Where an import is. Reading the file can't report how far it is
// (stageFraction stays -1); translating and meshing do.
struct Progress {
    enum class Stage { Reading, Translating, Meshing };
    Stage  stage = Stage::Reading;
    double stageFraction = -1.0;   // 0..1 within the stage, -1 = unknown
    int    done = 0, total = 0;    // meshing: parts meshed of the distinct parts
    QString part;                  // meshing: the largest part still being meshed
    int    partFaces = 0;          // and its face count
};
// Called on the importing thread (throttled). Return false to cancel; the
// import then stops at the next point OCCT can stop and fails with
// errorOut = cancelledMessage().
using ProgressFn = std::function<bool(const Progress&)>;
QString cancelledMessage();

// On success fills rootOut (single root; synthetic root if the file has
// multiple top-level shapes). All geometry is converted to metres.
bool importFile(const QString& path, Part& rootOut, QString& errorOut,
                const ProgressFn& progress = {});

// The axis that points up in the CAD file. MotionStudio is Y-up; most CAD
// systems (SolidWorks, NX, Creo, Inventor, CATIA) are Z-up, so a Z-up model
// is turned -90° about X to stand upright. Applied to the root part, so the
// parts keep their CAD positions relative to it.
enum class UpAxis { Y, Z };
void applyUpAxis(Part& root, UpAxis up);

// Adds an imported tree to a scene under `parentId` (-1 = the top); returns
// the new top object
int addToScene(SceneGraph& scene, const Part& part, int parentId = -1);

// Turns stage progress into an overall fraction and a time-left estimate.
// Reading can't report progress and translating reports it too unevenly to
// extrapolate, so their lengths are guessed from the file size and the rates
// seen in earlier imports (`learn`); meshing extrapolates from its own
// progress. Times are seconds since the import began.
class Estimator {
public:
    struct Rates {
        double readSecPerMB   = 0.12;  // reading the file
        double translateRatio = 2.5;   // translating time / reading time
        double meshRatio      = 0.5;   // meshing time / reading time
    };
    Estimator(qint64 fileBytes, const Rates& rates);

    void update(const Progress& p, double now);
    double fraction() const { return m_fraction; }     // 0..1, never goes back
    double secondsLeft() const { return m_left; }
    // Whether secondsLeft is worth showing: false in the first second, when
    // a stage runs past its guess, or when progress has stopped moving (one
    // big part can mesh for a long time without reporting anything)
    bool confident() const { return m_confident; }
    // Rates blended with what this import measured (call once it finished)
    Rates learn(double now) const;

private:
    double expectedRead() const;
    qint64 m_bytes;
    Rates  m_rates;
    double m_now = 0.0;
    double m_readEnd = -1.0, m_translateEnd = -1.0;   // when those stages ended
    double m_meshFrac = -1.0, m_meshFracSince = 0.0;  // last meshing fraction, since when
    double m_fraction = 0.0, m_left = 0.0;
    bool   m_confident = false;
};

} // namespace StepImporter
