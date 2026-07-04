#include "StepImporter.h"
#include <QFileInfo>

#ifdef OCCT_ENABLED

#include <STEPCAFControl_Reader.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDocStd_Document.hxx>
#include <TDataStd_Name.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDF_Tool.hxx>
#include <TCollection_AsciiString.hxx>
#include <Interface_Static.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopLoc_Location.hxx>
#include <Poly_Triangulation.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GeomAbs_CurveType.hxx>
#include <gp_Circ.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>

#include <QtMath>
#include <limits>
#include <map>

namespace {

QString labelName(const TDF_Label& label) {
    Handle(TDataStd_Name) nameAttr;
    if (label.FindAttribute(TDataStd_Name::GetID(), nameAttr)) {
        const TCollection_ExtendedString& ext = nameAttr->Get();
        return QString::fromUtf16(reinterpret_cast<const char16_t*>(ext.ToExtString()),
                                  ext.Length()).trimmed();
    }
    return {};
}

// Decompose an OCCT location into our Transform convention:
// translate → rotate X → rotate Y → rotate Z (intrinsic XYZ, degrees)
void decomposeLocation(const TopLoc_Location& loc, QVector3D& pos, QVector3D& rotDeg) {
    const gp_Trsf& t = loc.Transformation();
    gp_XYZ tr = t.TranslationPart();
    pos = { float(tr.X()), float(tr.Y()), float(tr.Z()) };

    double rx = 0, ry = 0, rz = 0;
    t.GetRotation().GetEulerAngles(gp_Intrinsic_XYZ, rx, ry, rz);
    rotDeg = { float(qRadiansToDegrees(rx)),
               float(qRadiansToDegrees(ry)),
               float(qRadiansToDegrees(rz)) };
}

std::shared_ptr<MeshGeometry> triangulateShape(const TopoDS_Shape& shape) {
    if (shape.IsNull()) return nullptr;

    // Deflection relative to model size so small and large parts mesh sensibly
    Bnd_Box bbox;
    BRepBndLib::Add(shape, bbox);
    if (bbox.IsVoid()) return nullptr;
    double diag = std::sqrt(bbox.SquareExtent());
    double deflection = std::max(diag * 0.004, 1e-5);

    BRepMesh_IncrementalMesh mesher(shape, deflection, Standard_False, 0.5, Standard_True);

    auto geo = std::make_shared<MeshGeometry>();
    QVector3D mn( std::numeric_limits<float>::max(),  std::numeric_limits<float>::max(),  std::numeric_limits<float>::max());
    QVector3D mx(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face& face = TopoDS::Face(ex.Current());
        TopLoc_Location faceLoc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, faceLoc);
        if (tri.IsNull()) continue;

        const gp_Trsf& trsf = faceLoc.Transformation();
        const bool reversed = (face.Orientation() == TopAbs_REVERSED);

        for (int i = 1; i <= tri->NbTriangles(); i++) {
            int n1, n2, n3;
            tri->Triangle(i).Get(n1, n2, n3);
            if (reversed) std::swap(n2, n3);

            gp_Pnt p1 = tri->Node(n1).Transformed(trsf);
            gp_Pnt p2 = tri->Node(n2).Transformed(trsf);
            gp_Pnt p3 = tri->Node(n3).Transformed(trsf);

            QVector3D a(float(p1.X()), float(p1.Y()), float(p1.Z()));
            QVector3D b(float(p2.X()), float(p2.Y()), float(p2.Z()));
            QVector3D c(float(p3.X()), float(p3.Y()), float(p3.Z()));

            QVector3D nrm = QVector3D::crossProduct(b - a, c - a);
            if (nrm.lengthSquared() < 1e-20f) continue;   // degenerate
            nrm.normalize();

            for (const QVector3D& v : {a, b, c}) {
                geo->interleaved.insert(geo->interleaved.end(),
                    {v.x(), v.y(), v.z(), nrm.x(), nrm.y(), nrm.z()});
                mn.setX(std::min(mn.x(), v.x())); mx.setX(std::max(mx.x(), v.x()));
                mn.setY(std::min(mn.y(), v.y())); mx.setY(std::max(mx.y(), v.y()));
                mn.setZ(std::min(mn.z(), v.z())); mx.setZ(std::max(mx.z(), v.z()));
            }
        }
    }

    if (geo->interleaved.empty()) return nullptr;
    geo->aabbMin = mn;
    geo->aabbMax = mx;

    // Circular edges (hole rims, cylindrical boss/shaft faces) — exact
    // rotation-origin candidates from the real B-Rep curve, not guessed from
    // the tessellated mesh. TopExp_Explorer visits each unique edge once.
    for (TopExp_Explorer ex(shape, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(ex.Current());
        BRepAdaptor_Curve curve(edge);
        if (curve.GetType() != GeomAbs_Circle) continue;
        gp_Circ circ = curve.Circle();
        gp_Pnt  c    = circ.Location();
        geo->circles.push_back({
            QVector3D(float(c.X()), float(c.Y()), float(c.Z())),
            float(circ.Radius())
        });
    }

    return geo;
}

// Meshes for referred (prototype) labels are cached so that repeated
// instances of the same part in an assembly share one MeshGeometry.
using MeshCache = std::map<QString, std::shared_ptr<MeshGeometry>>;

QString labelEntry(const TDF_Label& label) {
    TCollection_AsciiString entry;
    TDF_Tool::Entry(label, entry);
    return QString::fromLatin1(entry.ToCString());
}

// Builds the Part subtree for a prototype label (assembly or simple part).
// The instance transform lives on the *component* label and is applied by the caller.
StepImporter::Part processLabel(const TDF_Label& label,
                                const Handle(XCAFDoc_ShapeTool)& shapeTool,
                                MeshCache& cache,
                                int depth) {
    StepImporter::Part part;
    part.name = labelName(label);

    if (depth > 64) return part;   // guard against cyclic references

    if (shapeTool->IsAssembly(label)) {
        TDF_LabelSequence comps;
        shapeTool->GetComponents(label, comps);
        for (int i = 1; i <= comps.Length(); i++) {
            const TDF_Label comp = comps.Value(i);
            TDF_Label ref;
            if (!shapeTool->GetReferredShape(comp, ref)) continue;

            StepImporter::Part child = processLabel(ref, shapeTool, cache, depth + 1);
            // Component label usually carries the instance name; fall back to prototype
            QString compName = labelName(comp);
            if (!compName.isEmpty()) child.name = compName;
            if (child.name.isEmpty()) child.name = QString("Part_%1").arg(i);

            decomposeLocation(shapeTool->GetLocation(comp), child.position, child.rotationDeg);
            part.children.push_back(std::move(child));
        }
    } else {
        const QString key = labelEntry(label);
        auto it = cache.find(key);
        if (it != cache.end()) {
            part.mesh = it->second;
        } else {
            part.mesh = triangulateShape(shapeTool->GetShape(label));
            cache[key] = part.mesh;
        }
    }
    return part;
}

} // namespace

namespace StepImporter {

bool available() { return true; }

bool importFile(const QString& path, Part& rootOut, QString& errorOut) {
    // Translate geometry into metres (STEP files are typically authored in mm)
    STEPCAFControl_Reader reader;
    reader.SetNameMode(Standard_True);

    // Must be set after the reader is constructed (that's what registers the
    // "xstep.cascade.unit" static parameter) and before ReadFile/Transfer.
    Interface_Static::SetCVal("xstep.cascade.unit", "M");

    if (reader.ReadFile(path.toUtf8().constData()) != IFSelect_RetDone) {
        errorOut = "Could not read STEP file (unrecognized or corrupt format).";
        return false;
    }

    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
    Handle(TDocStd_Document) doc;
    app->NewDocument("MDTV-XCAF", doc);

    if (!reader.Transfer(doc)) {
        errorOut = "STEP translation failed — the file contains no transferable shapes.";
        return false;
    }

    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);
    if (freeShapes.Length() == 0) {
        errorOut = "STEP file contains no top-level shapes.";
        return false;
    }

    MeshCache cache;
    const QString baseName = QFileInfo(path).completeBaseName();

    if (freeShapes.Length() == 1) {
        rootOut = processLabel(freeShapes.Value(1), shapeTool, cache, 0);
        decomposeLocation(shapeTool->GetLocation(freeShapes.Value(1)),
                          rootOut.position, rootOut.rotationDeg);
        if (rootOut.name.isEmpty()) rootOut.name = baseName;
    } else {
        // Multiple top-level shapes → synthetic root named after the file
        rootOut = Part{};
        rootOut.name = baseName;
        for (int i = 1; i <= freeShapes.Length(); i++) {
            Part child = processLabel(freeShapes.Value(i), shapeTool, cache, 0);
            decomposeLocation(shapeTool->GetLocation(freeShapes.Value(i)),
                              child.position, child.rotationDeg);
            if (child.name.isEmpty()) child.name = QString("%1_%2").arg(baseName).arg(i);
            rootOut.children.push_back(std::move(child));
        }
    }

    // A root with no mesh and no children means nothing usable was translated
    if (!rootOut.mesh && rootOut.children.empty()) {
        errorOut = "STEP file translated, but produced no visualizable geometry.";
        return false;
    }
    return true;
}

} // namespace StepImporter

#else // !OCCT_ENABLED

namespace StepImporter {

bool available() { return false; }

bool importFile(const QString&, Part&, QString& errorOut) {
    errorOut = "STEP import is unavailable: MotionStudio was built without OpenCASCADE. "
               "Install the libocct-*-dev packages and rebuild.";
    return false;
}

} // namespace StepImporter

#endif
