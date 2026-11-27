#pragma once
// Helpers shared by the API server's method files (ApiServer.cpp, ApiScene.cpp)
#include "ApiServer.h"
#include "../scene/SceneGraph.h"
#include <QJsonArray>
#include <QJsonObject>
#include <QStringList>
#include <cmath>
#include <vector>

namespace apicommon {

constexpr double kMm = 1000.0;                 // API lengths are millimetres

struct ApiError {
    QString code, message;
};
[[noreturn]] inline void fail(const QString& code, const QString& message) { throw ApiError{code, message}; }

// Joint values: degrees, or millimetres for prismatic joints
inline double round6(double v) { return std::round(v * 1e6) / 1e6; }
// Lengths kept as float metres, in mm: to 0.1 µm (a float's own precision)
inline double roundMm(double mm) { return std::round(mm * 1e4) / 1e4; }
inline double toApi(const Joint& j, double v)   { return j.type == JointType::Prismatic ? roundMm(v * kMm) : round6(v); }
inline double fromApi(const Joint& j, double v) { return j.type == JointType::Prismatic ? v / kMm : v; }

inline bool inSubtree(const SceneGraph& g, int root, int id) {
    for (const SceneObject* o = g.getObject(id); o; o = g.getObject(o->parentId))
        if (o->id == root) return true;
    return false;
}

inline void collect(const SceneGraph& g, int id, std::vector<int>& out) {
    const SceneObject* o = g.getObject(id);
    if (!o) return;
    out.push_back(id);
    for (int c : o->childIds) collect(g, c, out);
}

inline bool given(const QJsonObject& p, const QString& key) {
    const QJsonValue v = p.value(key);
    return !v.isUndefined() && !v.isNull();
}
inline double number(const QJsonObject& p, const QString& key, double fallback) {
    const QJsonValue v = p.value(key);
    if (v.isUndefined() || v.isNull()) return fallback;
    if (v.isBool()) return v.toBool() ? 1 : 0;
    if (!v.isDouble()) fail("bad_request", QString("\"%1\" must be a number").arg(key));
    return v.toDouble();
}
inline QString text(const QJsonObject& p, const QString& key, bool required = true) {
    const QJsonValue v = p.value(key);
    if ((v.isUndefined() || v.isNull()) && !required) return {};
    if (!v.isString() || v.toString().trimmed().isEmpty())
        fail("bad_request", QString("\"%1\" must be a name").arg(key));
    return v.toString().trimmed();
}
inline bool flag(const QJsonObject& p, const QString& key, bool fallback) {
    const QJsonValue v = p.value(key);
    if (v.isUndefined() || v.isNull()) return fallback;
    if (!v.isBool()) fail("bad_request", QString("\"%1\" must be true or false").arg(key));
    return v.toBool();
}

inline QString listOf(QStringList names, int max = 12) {
    names.sort(Qt::CaseInsensitive);
    if (names.size() > max) { names = names.mid(0, max); names << "…"; }
    return names.join(", ");
}

} // namespace apicommon

struct ApiServer::Call {
    QTcpSocket* socket = nullptr;
    QJsonValue  id;
    bool        deferred = false;   // answered later from checkPending (`pending`)
    bool        async    = false;   // answered later by the method itself (answerLater)
    Pending     pending;
};
