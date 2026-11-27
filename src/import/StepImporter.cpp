#include "StepImporter.h"
#include "../scene/SceneGraph.h"
#include <QMatrix4x4>
#include <QFileInfo>
#include <algorithm>
#include <chrono>
#include <cmath>

#ifdef OCCT_ENABLED

#include <STEPCAFControl_Reader.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <Quantity_Color.hxx>
#include <TDocStd_Document.hxx>
#include <TDataStd_Name.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDF_Tool.hxx>
#include <TCollection_AsciiString.hxx>
#include <Interface_Static.hxx>
#include <XSControl_TransferReader.hxx>
#include <XSControl_WorkSession.hxx>
#include <StepRepr_RepresentationItem.hxx>
#include <TCollection_HAsciiString.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressScope.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepMesh_Context.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopLoc_Location.hxx>
#include <Poly_Triangulation.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>

#include <QDir>
#include <QFile>
#include <QStandardPaths>
#include <QtMath>
#include <atomic>
#include <limits>
#include <map>
#include <mutex>
#include <thread>

namespace {

QString labelName(const TDF_Label& label) {
    Handle(TDataStd_Name) nameAttr;
    if (label.FindAttribute(TDataStd_Name::GetID(), nameAttr)) {
        const TCollection_ExtendedString& ext = nameAttr->Get();
        const QString name = QString::fromUtf16(reinterpret_cast<const char16_t*>(ext.ToExtString()),
                                                ext.Length()).trimmed();
        // OCCT names an assembly's own geometry (a product with both parts and
        // bodies) after its label entry, "=>[0:1:1:5]"
        if (name.startsWith("=>[")) return QStringLiteral("Bodies");
        return name;
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

// Forwards OCCT's progress to a ProgressFn (at most every 50 ms) and turns
// its "false" into a user break. OCCT may call Show from mesher threads; it
// serialises those calls itself.
class Indicator : public Message_ProgressIndicator {
public:
    Indicator(const StepImporter::ProgressFn& fn) : m_fn(fn) {}

    StepImporter::Progress state;
    bool fromOcct = true;   // stageFraction is OCCT's position (else set by the caller)

    // Reports `state`; false once cancelled
    bool report(bool force = false) {
        if (!m_fn || m_cancelled) return !m_cancelled;
        const auto t = std::chrono::steady_clock::now();
        if (!force && t - m_last < std::chrono::milliseconds(50)) return true;
        m_last = t;
        if (fromOcct) state.stageFraction = std::clamp(GetPosition(), 0.0, 1.0);
        if (!m_fn(state)) m_cancelled = true;
        return !m_cancelled;
    }
    bool cancelled() const { return m_cancelled; }

    Standard_Boolean UserBreak() override { return m_cancelled; }
protected:
    void Show(const Message_ProgressScope&, const Standard_Boolean force) override { report(force); }
private:
    const StepImporter::ProgressFn& m_fn;
    std::chrono::steady_clock::time_point m_last{};
    bool m_cancelled = false;
};

// One part's meshing progress, read by the thread waiting for the workers
class PartIndicator : public Message_ProgressIndicator {
public:
    PartIndicator(std::atomic<double>& position, const std::atomic<bool>& cancel)
        : m_position(position), m_cancel(cancel) {}
    Standard_Boolean UserBreak() override { return m_cancel.load(); }
protected:
    void Show(const Message_ProgressScope&, const Standard_Boolean) override {
        m_position.store(std::clamp(GetPosition(), 0.0, 1.0));
    }
private:
    std::atomic<double>& m_position;
    const std::atomic<bool>& m_cancel;
};

std::shared_ptr<MeshGeometry> triangulateShape(const TopoDS_Shape& shape, bool manyHoles,
                                               const Message_ProgressRange& range = {}) {
    if (shape.IsNull()) return nullptr;

    // Deflection relative to model size so small and large parts mesh sensibly
    Bnd_Box bbox;
    BRepBndLib::Add(shape, bbox);
    if (bbox.IsVoid()) return nullptr;
    double diag = std::sqrt(bbox.SquareExtent());
    double deflection = std::max(diag * 0.004, 1e-5);

    IMeshTools_Parameters params;
    params.Deflection = deflection;
    params.Angle      = 0.5;
    params.InParallel = Standard_True;
    BRepMesh_IncrementalMesh mesher;
    mesher.SetShape(shape);
    mesher.ChangeParameters() = params;
    if (manyHoles) {
        // OCCT's default triangulator slows down badly on a face with
        // thousands of holes (a grille: 13 s for one face); Delabella does it
        // in a fraction of that. Faces it fails on get the default below.
        Message_ProgressScope scope(range, "Meshing", 2);
        mesher.Perform(new BRepMesh_Context(IMeshTools_MeshAlgoType_Delabella), scope.Next());
        if (scope.UserBreak()) return nullptr;
        bool missing = false;
        for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More() && !missing; ex.Next()) {
            TopLoc_Location loc;
            missing = BRep_Tool::Triangulation(TopoDS::Face(ex.Current()), loc).IsNull();
        }
        if (missing) mesher.Perform(scope.Next());
    } else {
        mesher.Perform(range);
    }
    if (range.UserBreak()) return nullptr;

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

    return geo;
}

// Meshes for referred (prototype) labels are cached so that repeated
// instances of the same part in an assembly share one MeshGeometry.
using MeshCache = std::map<QString, std::shared_ptr<MeshGeometry>>;

QColor toQColor(const Quantity_Color& c) {
    double r, g, b;
    c.Values(r, g, b, Quantity_TOC_sRGB);
    return QColor::fromRgbF(float(r), float(g), float(b));
}

// A label's colour: surface or generic colour of the label (or of its
// shape), else the colour of its first coloured face
QColor labelColor(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool,
                  const Handle(XCAFDoc_ColorTool)& colorTool, bool searchFaces) {
    if (colorTool.IsNull()) return {};
    Quantity_Color c;
    for (XCAFDoc_ColorType t : {XCAFDoc_ColorSurf, XCAFDoc_ColorGen})
        if (colorTool->GetColor(label, t, c)) return toQColor(c);
    const TopoDS_Shape shape = shapeTool->GetShape(label);
    if (shape.IsNull()) return {};
    for (XCAFDoc_ColorType t : {XCAFDoc_ColorSurf, XCAFDoc_ColorGen})
        if (colorTool->GetColor(shape, t, c)) return toQColor(c);
    if (!searchFaces) return {};
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next())
        if (colorTool->GetColor(ex.Current(), XCAFDoc_ColorSurf, c)) return toQColor(c);
    return {};
}

// A shape's own colour, else its first coloured face's
QColor shapeColor(const TopoDS_Shape& shape, const Handle(XCAFDoc_ColorTool)& colorTool) {
    if (colorTool.IsNull()) return {};
    Quantity_Color c;
    for (XCAFDoc_ColorType t : {XCAFDoc_ColorSurf, XCAFDoc_ColorGen})
        if (colorTool->GetColor(shape, t, c)) return toQColor(c);
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next())
        if (colorTool->GetColor(ex.Current(), XCAFDoc_ColorSurf, c)) return toQColor(c);
    return {};
}

QString labelEntry(const TDF_Label& label) {
    TCollection_AsciiString entry;
    TDF_Tool::Entry(label, entry);
    return QString::fromLatin1(entry.ToCString());
}

// A part's pieces when its shape holds several bodies (CAD systems such as NX
// export a multi-body part, or a whole station, as one product): the solids,
// shells and loose faces of its compound, nested compounds flattened, each
// with its placement. Empty when the shape is one body, which stays one mesh.
void collectPieces(const TopoDS_Shape& shape, std::vector<TopoDS_Shape>& out) {
    if (shape.ShapeType() != TopAbs_COMPOUND) { out.push_back(shape); return; }
    for (TopoDS_Iterator it(shape); it.More(); it.Next()) collectPieces(it.Value(), out);
}
std::vector<TopoDS_Shape> pieces(const TopoDS_Shape& shape) {
    std::vector<TopoDS_Shape> out;
    if (!shape.IsNull()) collectPieces(shape, out);
    if (out.size() < 2) out.clear();
    return out;
}

// Each distinct piece's mesh, keyed by its TShape so repeated bodies share
// one mesh. The mesh is centred on its box (CAD files usually bake a body's
// placement into its geometry), so the object sits where the body is;
// `centre` is that box centre in the piece's own frame.
struct PieceMesh { std::shared_ptr<MeshGeometry> mesh; gp_Vec centre; };
using PieceMeshes = std::map<const void*, PieceMesh>;

// Moves a mesh's vertices so its box is centred on the origin; returns the old centre
gp_Vec centreMesh(MeshGeometry& m) {
    const QVector3D c = (m.aabbMin + m.aabbMax) * 0.5f;
    for (size_t i = 0; i + 5 < m.interleaved.size(); i += 6) {
        m.interleaved[i] -= c.x(); m.interleaved[i + 1] -= c.y(); m.interleaved[i + 2] -= c.z();
    }
    m.aabbMin -= c;
    m.aabbMax -= c;
    return gp_Vec(c.x(), c.y(), c.z());
}
// Each piece's name in the file (its solid's), keyed by its TShape
using PieceNames = std::map<const void*, QString>;

// The distinct simple shapes under a label (each meshed once), in tree order
void collectShapes(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool,
                   std::map<QString, TDF_Label>& seen, std::vector<TDF_Label>& out, int depth) {
    if (depth > 64) return;
    if (shapeTool->IsAssembly(label)) {
        TDF_LabelSequence comps;
        shapeTool->GetComponents(label, comps);
        for (int i = 1; i <= comps.Length(); i++) {
            TDF_Label ref;
            if (shapeTool->GetReferredShape(comps.Value(i), ref))
                collectShapes(ref, shapeTool, seen, out, depth + 1);
        }
    } else if (seen.emplace(labelEntry(label), label).second) {
        out.push_back(label);
    }
}

// What meshing a shape is likely to cost. A face costs more than linearly
// in its edge count (a face with thousands of holes dominates a part), so
// each face counts edges^1.5. Also says whether a face has many holes.
struct MeshCost { int faces = 0; double weight = 0; bool manyHoles = false; };
MeshCost meshCost(const TopoDS_Shape& shape) {
    MeshCost c;
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
        int edges = 0, wires = 0;
        for (TopExp_Explorer e(ex.Current(), TopAbs_EDGE); e.More(); e.Next()) edges++;
        for (TopExp_Explorer w(ex.Current(), TopAbs_WIRE); w.More(); w.Next()) wires++;
        c.faces++;
        c.weight += std::pow(double(std::max(edges, 1)), 1.5);
        if (wires > 50) c.manyHoles = true;
    }
    c.weight = std::max(c.weight, 1.0);
    return c;
}

// Builds the Part subtree for a prototype label (assembly or simple part).
// The instance transform lives on the *component* label and is applied by the caller.
StepImporter::Part processLabel(const TDF_Label& label,
                                const Handle(XCAFDoc_ShapeTool)& shapeTool,
                                const Handle(XCAFDoc_ColorTool)& colorTool,
                                MeshCache& cache, const PieceMeshes& pieceMeshes,
                                const PieceNames& pieceNameOf, int depth) {
    StepImporter::Part part;
    part.name = labelName(label);
    part.color = labelColor(label, shapeTool, colorTool, !shapeTool->IsAssembly(label));

    if (depth > 64) return part;   // guard against cyclic references

    if (shapeTool->IsAssembly(label)) {
        TDF_LabelSequence comps;
        shapeTool->GetComponents(label, comps);
        for (int i = 1; i <= comps.Length(); i++) {
            const TDF_Label comp = comps.Value(i);
            TDF_Label ref;
            if (!shapeTool->GetReferredShape(comp, ref)) continue;

            StepImporter::Part child = processLabel(ref, shapeTool, colorTool, cache, pieceMeshes, pieceNameOf, depth + 1);
            // A colour on the instance overrides the part's own
            const QColor instance = labelColor(comp, shapeTool, colorTool, false);
            if (instance.isValid()) child.color = instance;
            // Component label usually carries the instance name; fall back to prototype
            QString compName = labelName(comp);
            if (!compName.isEmpty()) child.name = compName;
            if (child.name.isEmpty()) child.name = QString("Part_%1").arg(i);

            decomposeLocation(shapeTool->GetLocation(comp), child.position, child.rotationDeg);
            part.children.push_back(std::move(child));
        }
    } else if (const std::vector<TopoDS_Shape> ps = pieces(shapeTool->GetShape(label)); !ps.empty()) {
        // Several bodies: one child each, named by the file (repeated names
        // numbered like assembly instances), coloured by the body or its faces
        std::vector<QString> pieceNames(ps.size());
        std::map<QString, int> count;
        for (size_t i = 0; i < ps.size(); i++) {
            if (auto n = pieceNameOf.find(ps[i].TShape().get()); n != pieceNameOf.end()) pieceNames[i] = n->second;
            if (pieceNames[i].isEmpty()) pieceNames[i] = part.name.isEmpty() ? "Body" : part.name + " body";
            count[pieceNames[i]]++;
        }
        std::map<QString, int> seen;
        for (size_t i = 0; i < ps.size(); i++) {
            auto mesh = pieceMeshes.find(ps[i].TShape().get());
            if (mesh == pieceMeshes.end() || !mesh->second.mesh) continue;
            StepImporter::Part child;
            child.name = count[pieceNames[i]] > 1 ? QString("%1:%2").arg(pieceNames[i]).arg(++seen[pieceNames[i]])
                                                  : pieceNames[i];
            child.mesh = mesh->second.mesh;
            child.color = shapeColor(ps[i], colorTool);
            if (!child.color.isValid()) child.color = part.color;
            gp_Trsf centre;
            centre.SetTranslation(mesh->second.centre);
            decomposeLocation(ps[i].Location() * TopLoc_Location(centre), child.position, child.rotationDeg);
            part.children.push_back(std::move(child));
        }
    } else {
        const QString key = labelEntry(label);
        auto it = cache.find(key);
        if (it != cache.end()) {
            part.mesh = it->second;
        } else {
            part.mesh = triangulateShape(shapeTool->GetShape(label), false);
            cache[key] = part.mesh;
        }
    }
    return part;
}

// OCCT's STEP shape-healing settings (its XSTEPResource/STEP file) with one
// change: no wire self-intersection fixing. That check took most of the
// translation time on large assemblies (34 of 51 s on a 30 MB tractor) and
// left the meshes identical. OCCT reads it from $CSF_STEPDefaults/STEP; when
// that isn't set it falls back to full healing, so we write our copy to the
// cache folder and point the variable there (unless the user set it).
const char* kHealingResource =
    "ToSTEP.exec.op : SplitCommonVertex,DirectFaces\n"
    "FromSTEP.exec.op : FixShape\n"
    "FromSTEP.FixShape.Tolerance3d : &Runtime.Tolerance\n"
    "FromSTEP.FixShape.MaxTolerance3d : &Runtime.MaxTolerance\n"
    "FromSTEP.FixShape.MinTolerance3d : 1.e-7\n"
    "FromSTEP.FixShape.FixFreeShellMode : -1\n"
    "FromSTEP.FixShape.FixFreeFaceMode : -1\n"
    "FromSTEP.FixShape.FixFreeWireMode : -1\n"
    "FromSTEP.FixShape.FixSameParameterMode : -1\n"
    "FromSTEP.FixShape.FixSolidMode : -1\n"
    "FromSTEP.FixShape.FixShellOrientationMode : -1\n"
    "FromSTEP.FixShape.CreateOpenSolidMode : 0\n"
    "FromSTEP.FixShape.FixShellMode : -1\n"
    "FromSTEP.FixShape.FixFaceOrientationMode : -1\n"
    "FromSTEP.FixShape.FixFaceMode : -1\n"
    "FromSTEP.FixShape.FixWireMode : -1\n"
    "FromSTEP.FixShape.FixOrientationMode : -1\n"
    "FromSTEP.FixShape.FixAddNaturalBoundMode : -1\n"
    "FromSTEP.FixShape.FixMissingSeamMode : -1\n"
    "FromSTEP.FixShape.FixSmallAreaWireMode : -1\n"
    "FromSTEP.FixShape.RemoveSmallAreaFaceMode : -1\n"
    "FromSTEP.FixShape.FixIntersectingWiresMode : -1\n"
    "FromSTEP.FixShape.FixLoopWiresMode : -1\n"
    "FromSTEP.FixShape.FixSplitFaceMode : -1\n"
    "FromSTEP.FixShape.AutoCorrectPrecisionMode : 1\n"
    "FromSTEP.FixShape.ModifyTopologyMode : 0\n"
    "FromSTEP.FixShape.ModifyGeometryMode : 1\n"
    "FromSTEP.FixShape.ClosedWireMode : 1\n"
    "FromSTEP.FixShape.PreferencePCurveMode : 1\n"
    "FromSTEP.FixShape.FixReorderMode : -1\n"
    "FromSTEP.FixShape.FixSmallMode : -1\n"
    "FromSTEP.FixShape.FixConnectedMode : -1\n"
    "FromSTEP.FixShape.FixEdgeCurvesMode : -1\n"
    "FromSTEP.FixShape.FixDegeneratedMode : -1\n"
    "FromSTEP.FixShape.FixLackingMode : -1\n"
    "FromSTEP.FixShape.FixSelfIntersectionMode : 0\n"
    "FromSTEP.FixShape.RemoveLoopMode : -1\n"
    "FromSTEP.FixShape.FixReversed2dMode : -1\n"
    "FromSTEP.FixShape.FixRemovePCurveMode : -1\n"
    "FromSTEP.FixShape.FixRemoveCurve3dMode : -1\n"
    "FromSTEP.FixShape.FixAddPCurveMode : -1\n"
    "FromSTEP.FixShape.FixAddCurve3dMode : -1\n"
    "FromSTEP.FixShape.FixSeamMode : -1\n"
    "FromSTEP.FixShape.FixShiftedMode : -1\n"
    "FromSTEP.FixShape.FixEdgeSameParameterMode : 0\n"
    "FromSTEP.FixShape.FixNotchedEdgesMode : -1\n"
    "FromSTEP.FixShape.FixTailMode : 0\n"
    "FromSTEP.FixShape.MaxTailAngle : 0\n"
    "FromSTEP.FixShape.MaxTailWidth : -1\n"
    "FromSTEP.FixShape.FixSelfIntersectingEdgeMode : -1\n"
    "FromSTEP.FixShape.FixIntersectingEdgesMode : -1\n"
    "FromSTEP.FixShape.FixNonAdjacentIntersectingEdgesMode : -1\n"
    "FromSTEP.FixShape.FixVertexPositionMode : 0\n"
    "FromSTEP.FixShape.FixVertexToleranceMode : -1\n";

void useFastHealing() {
    static std::once_flag once;
    std::call_once(once, []{
        if (qEnvironmentVariableIsSet("CSF_STEPDefaults")) return;
        QString dir = QStandardPaths::writableLocation(QStandardPaths::CacheLocation);
        if (dir.isEmpty()) dir = QDir::tempPath() + "/MotionStudio";
        dir += "/occt";
        QFile file(dir + "/STEP");
        if (!QDir().mkpath(dir) || !file.open(QIODevice::WriteOnly | QIODevice::Truncate)) return;
        file.write(kHealingResource);
        file.close();
        qputenv("CSF_STEPDefaults", QFile::encodeName(QDir::toNativeSeparators(dir)));
    });
}

} // namespace

namespace StepImporter {

bool available() { return true; }

bool importFile(const QString& path, Part& rootOut, QString& errorOut, const ProgressFn& progress) {
    Handle(Indicator) indicator = new Indicator(progress);
    auto cancelled = [&]{
        if (!indicator->cancelled()) return false;
        errorOut = cancelledMessage();
        return true;
    };

    useFastHealing();

    // Translate geometry into metres (STEP files are typically authored in mm)
    STEPCAFControl_Reader reader;
    reader.SetNameMode(Standard_True);
    reader.SetColorMode(Standard_True);

    // Must be set after the reader is constructed (that's what registers the
    // "xstep.cascade.unit" static parameter) and before ReadFile/Transfer.
    Interface_Static::SetCVal("xstep.cascade.unit", "M");

    // Reading can't report progress or stop part way
    indicator->state.stage = Progress::Stage::Reading;
    indicator->report(true);
    if (reader.ReadFile(path.toUtf8().constData()) != IFSelect_RetDone) {
        errorOut = "Could not read STEP file (unrecognized or corrupt format).";
        return false;
    }

    indicator->state.stage = Progress::Stage::Translating;
    if (!indicator->report(true) && cancelled()) return false;

    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
    Handle(TDocStd_Document) doc;
    app->NewDocument("MDTV-XCAF", doc);
    // Close the document however we leave, so the application doesn't keep it
    struct Closer {
        Handle(XCAFApp_Application) app; Handle(TDocStd_Document) doc;
        ~Closer() { if (!doc.IsNull() && doc->IsOpened()) app->Close(doc); }
    } closer{app, doc};

    const bool transferred = reader.Transfer(doc, indicator->Start());
    if (cancelled()) return false;
    if (!transferred) {
        errorOut = "STEP translation failed — the file contains no transferable shapes.";
        return false;
    }

    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    Handle(XCAFDoc_ColorTool) colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);
    if (freeShapes.Length() == 0) {
        errorOut = "STEP file contains no top-level shapes.";
        return false;
    }

    // Mesh every distinct part once, several at a time, then build the tree
    // from the cache. Progress is weighted by each part's expected cost
    // (meshCost), including how far each running part is.
    std::map<QString, TDF_Label> seen;
    std::vector<TDF_Label> labels;
    for (int i = 1; i <= freeShapes.Length(); i++)
        collectShapes(freeShapes.Value(i), shapeTool, seen, labels, 0);
    // Labels sharing one shape are meshed once (meshing writes into the
    // shape, so one shape must never be meshed by two threads)
    std::vector<TopoDS_Shape> shapes;
    std::vector<QString> names;
    std::vector<int> shapeOf(labels.size()), faces;
    std::vector<double> weight;
    std::vector<char> manyHoles;
    std::map<const void*, int> byTShape;
    double totalWeight = 0;
    auto addShape = [&](const TopoDS_Shape& shape, const QString& name) {
        auto [it, added] = byTShape.emplace(shape.TShape().get(), int(shapes.size()));
        if (!added) return it->second;
        shapes.push_back(shape);
        names.push_back(name);
        const MeshCost cost = meshCost(shape);
        faces.push_back(cost.faces);
        weight.push_back(cost.weight);
        manyHoles.push_back(cost.manyHoles);
        totalWeight += cost.weight;
        return it->second;
    };
    std::vector<int> pieceShapes;   // shapes that are pieces of a multi-body part
    PieceNames pieceNameOf;
    const Handle(XSControl_TransferReader) transfer = reader.ChangeReader().WS()->TransferReader();
    auto nameInFile = [&](const TopoDS_Shape& piece) {
        if (transfer.IsNull()) return QString();
        for (int mode : {0, 1, -1}) {
            Handle(StepRepr_RepresentationItem) item =
                Handle(StepRepr_RepresentationItem)::DownCast(transfer->EntityFromShapeResult(piece, mode));
            if (!item.IsNull() && !item->Name().IsNull())
                return QString::fromUtf8(item->Name()->ToCString()).trimmed();
        }
        return QString();
    };
    for (size_t i = 0; i < labels.size(); i++) {
        const TopoDS_Shape shape = shapeTool->GetShape(labels[i]);
        const std::vector<TopoDS_Shape> ps = pieces(shape);
        if (ps.empty()) {
            shapeOf[i] = addShape(shape, labelName(labels[i]));
            continue;
        }
        shapeOf[i] = -1;
        for (const TopoDS_Shape& p : ps) {
            pieceShapes.push_back(addShape(p.Located(TopLoc_Location()), labelName(labels[i])));
            QString& name = pieceNameOf[p.TShape().get()];
            if (name.isEmpty()) name = nameInFile(p);
        }
    }

    const int n = int(shapes.size());
    std::vector<std::shared_ptr<MeshGeometry>> meshes(n);
    std::unique_ptr<std::atomic<double>[]> position(new std::atomic<double>[n]);
    std::unique_ptr<std::atomic<int>[]> state(new std::atomic<int>[n]);   // 0 waiting, 1 running, 2 done
    for (int i = 0; i < n; i++) { position[i] = 0.0; state[i] = 0; }
    std::atomic<int> next{0}, finished{0};
    std::atomic<bool> cancel{false};

    // Heaviest first, so a big part doesn't start last and finish alone
    std::vector<int> order(n);
    for (int i = 0; i < n; i++) order[i] = i;
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return weight[a] > weight[b]; });

    auto worker = [&]{
        for (int k; (k = next.fetch_add(1)) < n && !cancel.load(); ) {
            const int i = order[k];
            state[i] = 1;
            Handle(PartIndicator) partProgress = new PartIndicator(position[i], cancel);
            meshes[i] = triangulateShape(shapes[i], manyHoles[i], partProgress->Start());
            state[i] = 2;
            finished.fetch_add(1);
        }
    };
    const int threads = std::clamp(int(std::thread::hardware_concurrency()), 1, std::max(1, n));
    std::vector<std::thread> pool;
    for (int t = 0; t < threads; t++) pool.emplace_back(worker);

    indicator->state.stage = Progress::Stage::Meshing;
    indicator->state.total = n;
    indicator->fromOcct = false;
    for (;;) {
        const int doneCount = finished.load();
        double doneWeight = 0;
        int largest = -1;   // the biggest part still meshing, named in the progress
        for (int i = 0; i < n; i++) {
            doneWeight += weight[i] * (state[i] == 2 ? 1.0 : state[i] == 1 ? position[i].load() : 0.0);
            if (state[i] == 1 && (largest < 0 || weight[i] > weight[largest])) largest = i;
        }
        indicator->state.done = doneCount;
        indicator->state.part = largest >= 0 ? names[largest] : QString();
        indicator->state.partFaces = largest >= 0 ? faces[largest] : 0;
        indicator->state.stageFraction = totalWeight > 0 ? doneWeight / totalWeight : 1.0;
        if (!indicator->report(doneCount == n)) cancel = true;
        if (doneCount == n || cancel) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    for (std::thread& t : pool) t.join();
    if (cancelled()) return false;

    MeshCache cache;
    for (size_t i = 0; i < labels.size(); i++)
        if (shapeOf[i] >= 0) cache[labelEntry(labels[i])] = meshes[shapeOf[i]];
    PieceMeshes pieceMeshes;
    for (int k : pieceShapes) {
        PieceMesh& pm = pieceMeshes[shapes[k].TShape().get()];
        if (pm.mesh || !meshes[k]) continue;
        pm.mesh = meshes[k];
        pm.centre = centreMesh(*pm.mesh);
    }

    const QString baseName = QFileInfo(path).completeBaseName();
    if (freeShapes.Length() == 1) {
        rootOut = processLabel(freeShapes.Value(1), shapeTool, colorTool, cache, pieceMeshes, pieceNameOf, 0);
        decomposeLocation(shapeTool->GetLocation(freeShapes.Value(1)),
                          rootOut.position, rootOut.rotationDeg);
        if (rootOut.name.isEmpty()) rootOut.name = baseName;
    } else {
        // Multiple top-level shapes → synthetic root named after the file
        rootOut = Part{};
        rootOut.name = baseName;
        for (int i = 1; i <= freeShapes.Length(); i++) {
            Part child = processLabel(freeShapes.Value(i), shapeTool, colorTool, cache, pieceMeshes, pieceNameOf, 0);
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

bool importFile(const QString&, Part&, QString& errorOut, const ProgressFn&) {
    errorOut = "STEP import is unavailable: MotionStudio was built without OpenCASCADE. "
               "Install the libocct-*-dev packages and rebuild.";
    return false;
}

} // namespace StepImporter

#endif

// ── Shared with and without OCCT ─────────────────────────────────────────────

namespace StepImporter {

QString cancelledMessage() { return QStringLiteral("Import cancelled."); }

void applyUpAxis(Part& root, UpAxis up) {
    if (up == UpAxis::Y) return;
    QMatrix4x4 local;
    local.translate(root.position);
    local.rotate(root.rotationDeg.x(), 1, 0, 0);
    local.rotate(root.rotationDeg.y(), 0, 1, 0);
    local.rotate(root.rotationDeg.z(), 0, 0, 1);
    QMatrix4x4 upright;
    upright.rotate(-90, 1, 0, 0);   // CAD +Z becomes +Y, CAD +Y becomes -Z
    const Transform t = SceneGraph::toTransform(upright * local);
    root.position = t.position;
    root.rotationDeg = t.rotation;
}

int addToScene(SceneGraph& scene, const Part& part, int parentId) {
    const int id = scene.addObject(part.name.isEmpty() ? QString("Part") : part.name, PrimitiveType::ImportedMesh, parentId);
    if (SceneObject* o = scene.getObject(id)) {
        o->transform.position = part.position;
        o->transform.rotation = part.rotationDeg;
        o->transform.scale    = {1, 1, 1};
        o->mesh               = part.mesh;
        o->color              = part.color;
    }
    for (const Part& c : part.children) addToScene(scene, c, id);
    emit scene.objectChanged(id);
    return id;
}

Estimator::Estimator(qint64 fileBytes, const Rates& rates)
    : m_bytes(std::max<qint64>(fileBytes, 1)), m_rates(rates) {}

double Estimator::expectedRead() const {
    return std::max(0.05, m_rates.readSecPerMB * double(m_bytes) / (1024.0 * 1024.0));
}

void Estimator::update(const Progress& p, double now) {
    using S = Progress::Stage;
    m_now = std::max(m_now, now);
    if (p.stage != S::Reading && m_readEnd < 0) m_readEnd = m_now;
    if (p.stage == S::Meshing && m_translateEnd < 0) {
        m_translateEnd = m_now;
        m_meshFracSince = m_now;
    }
    if (p.stage == S::Meshing && p.stageFraction > m_meshFrac + 1e-3) {
        m_meshFrac = p.stageFraction;
        m_meshFracSince = m_now;
    }

    // Expected length of each stage: measured once it's over, guessed from
    // the rates before. A stage running past its guess is assumed 90% done.
    bool overdue = false;
    auto guessed = [&](double start, double guess) {
        const double spent = m_now - start;
        if (spent > guess) overdue = true;
        return std::max(guess, spent / 0.9);
    };
    double read, translate, mesh;
    bool stalled = false;
    if (m_readEnd < 0) {
        read = guessed(0.0, expectedRead());
        translate = read * m_rates.translateRatio;
        mesh = read * m_rates.meshRatio;
    } else if (m_translateEnd < 0) {
        read = m_readEnd;
        translate = guessed(m_readEnd, read * m_rates.translateRatio);
        mesh = read * m_rates.meshRatio;
    } else {
        read = m_readEnd;
        translate = m_translateEnd - m_readEnd;
        const double spent = m_now - m_translateEnd;
        const double still = m_now - m_meshFracSince;   // since the fraction last moved
        stalled = still > 2.0;
        if (m_meshFrac > 0.05 && spent > 0.25)
            // Extrapolate; while stuck, expect at least as long again as it's
            // been stuck (so the estimate grows instead of claiming "1 s left")
            mesh = spent + std::max(spent / m_meshFrac - spent, stalled ? still : 0.0);
        else
            mesh = guessed(m_translateEnd, read * m_rates.meshRatio);
    }
    const double total = read + translate + mesh;
    m_left = std::max(0.0, total - m_now);
    m_fraction = std::clamp(std::max(m_fraction, m_now / total), 0.0, 0.99);
    m_confident = m_now >= 1.0 && !overdue && !stalled;
}

Estimator::Rates Estimator::learn(double now) const {
    Rates r = m_rates;
    if (m_readEnd <= 0.05 || m_translateEnd < m_readEnd || now < m_translateEnd) return r;
    const double read = m_readEnd;
    const double mb = std::max(0.01, double(m_bytes) / (1024.0 * 1024.0));
    // Half the old rate, half this import's, so one odd file doesn't skew it
    r.readSecPerMB   = 0.5 * r.readSecPerMB   + 0.5 * read / mb;
    r.translateRatio = 0.5 * r.translateRatio + 0.5 * (m_translateEnd - m_readEnd) / read;
    r.meshRatio      = 0.5 * r.meshRatio      + 0.5 * (now - m_translateEnd) / read;
    return r;
}

} // namespace StepImporter
