#pragma once
#include <QHostAddress>
#include <QJsonObject>
#include <QJsonValue>
#include <QObject>
#include <QHash>
#include <QStringList>
#include <functional>
#include <map>
#include <vector>
#include "../automation/AutomationEngine.h"
#include "../automation/Take.h"
#include <memory>

class QTcpServer;
class QTcpSocket;
class SceneGraph;
class SimulationEngine;

// The robot API: lets programs (the Python, C++ and Kotlin libraries in
// api/) drive the model over TCP. One JSON object per line each way:
//
//   → {"id": 7, "method": "tool.moveL", "params": {"robot": "Robot", "target": {"z": 100}}}
//   ← {"id": 7, "result": {"motion": 3, "state": "done"}}
//   ← {"id": 7, "error": {"code": "out_of_reach", "message": "the target is out of reach (12.0 mm off)"}}
//
// Lengths are millimetres, angles degrees, speeds per second; rotations are
// Euler XYZ like object transforms. Robots are found by the names of their
// root objects, tools and frames by object name or path ("Cell/Robot/TCP"),
// joints by their actuators' names. A motion with "wait" (the default) is
// answered when it ends; requests from one client are answered in any order,
// so a client matches replies by id. api/PROTOCOL.md lists every method.
//
// Commands that need the simulation (moves, signal writes, waits) start it
// when it is not running.
class ApiServer : public QObject {
    Q_OBJECT
public:
    static constexpr quint16 kDefaultPort = 5800;
    static constexpr int     kProtocol    = 1;

    ApiServer(SceneGraph* scene, SimulationEngine* sim, QObject* parent = nullptr);
    ~ApiServer() override;

    bool    start(quint16 port = kDefaultPort, const QHostAddress& address = QHostAddress::LocalHost);
    void    stop();
    bool    isListening() const;
    quint16 port() const;
    int     clientCount() const { return int(m_clients.size()); }
    QString lastError() const { return m_error; }
    long    requestCount() const { return m_requests; }
    // The project file the scene was opened from or saved to ("" = none):
    // what scene.save writes when it is given no path
    void    setProjectPath(const QString& path) { m_projectPath = path; }
    QString projectPath() const { return m_projectPath; }
    // Whether the open project has unsaved changes (the app's flag): then
    // scene.new and scene.open need "discard": true
    void    setUnsavedCheck(std::function<bool()> check) { m_unsaved = std::move(check); }

    // A mechanism as the API sees it: a root object and the actuated joints
    // below it that form one machine (a robot arm, a humanoid, a turntable)
    struct Robot {
        int              root = -1;
        QString          name, path;
        std::vector<int> joints;   // actuated, in actuator order
        std::vector<int> tools;    // objects named …TCP…, else the end of the longest chain
    };
    static std::vector<Robot> findRobots(const SceneGraph& scene);
    static QString pathOf(const SceneGraph& scene, int id);

signals:
    void clientsChanged(int count);
    void activity();
    // A program replaced the whole scene (scene.new, scene.open) or saved it
    void projectReplaced(const QString& path);
    void projectSaved(const QString& path);
    // A program imported a model (scene.import); its top object
    void imported(int rootId);

private:
    struct Call;
    struct Pending {
        QTcpSocket*      socket = nullptr;
        QJsonValue       id;
        enum class Kind { Motion, Time, Condition } kind = Kind::Motion;
        int              motion = -1;
        double           until = 0;      // Time: simulation time to answer at
        double           timeoutAt = -1; // simulation time, -1 = none
        expr::Expression condition;
        QString          conditionText;
    };
    using Handler = std::function<QJsonValue(ApiServer*, const QJsonObject&, Call&)>;

    void onNewConnection();
    void onReadyRead(QTcpSocket* socket);
    void onDisconnected(QTcpSocket* socket);
    void handleLine(QTcpSocket* socket, const QByteArray& line);
    void send(QTcpSocket* socket, const QJsonObject& msg);
    void checkPending();
    static const std::map<QString, Handler>& handlers();

    // Method helpers (throw ApiError)
    void   ensureRunning();
    Robot  robotParam(const QJsonObject& p, bool required = true) const;
    int    objectParam(const QString& what, const QString& name, int within = -1) const;
    int    toolParam(const QJsonObject& p, const Robot& r) const;
    int    jointByName(const QString& name, const Robot* r) const;
    QJsonObject poseOf(int objectId, int frameId) const;
    QJsonObject motionReply(int id) const;
    QJsonValue  submit(const QJsonObject& p, Call& call, const AutomationEngine::MotionRequest& req);

    // Methods
    QJsonValue hello(const QJsonObject& p, Call& c);
    QJsonValue simState(const QJsonObject& p, Call& c);
    QJsonValue simPlay(const QJsonObject& p, Call& c);
    QJsonValue simPause(const QJsonObject& p, Call& c);
    QJsonValue simReset(const QJsonObject& p, Call& c);
    QJsonValue simSpeed(const QJsonObject& p, Call& c);
    QJsonValue simWait(const QJsonObject& p, Call& c);
    QJsonValue sceneRobots(const QJsonObject& p, Call& c);
    QJsonValue robotInfo(const QJsonObject& p, Call& c);
    QJsonValue robotJoints(const QJsonObject& p, Call& c);
    QJsonValue robotMoveJoints(const QJsonObject& p, Call& c);
    QJsonValue robotMoveToPose(const QJsonObject& p, Call& c);
    QJsonValue robotHome(const QJsonObject& p, Call& c);
    QJsonValue robotStop(const QJsonObject& p, Call& c);
    QJsonValue toolPose(const QJsonObject& p, Call& c);
    QJsonValue toolMove(const QJsonObject& p, Call& c, bool linear);
    QJsonValue motionStatus(const QJsonObject& p, Call& c);
    QJsonValue motionWait(const QJsonObject& p, Call& c);
    QJsonValue motionCancel(const QJsonObject& p, Call& c);
    QJsonValue actuatorGet(const QJsonObject& p, Call& c);
    QJsonValue actuatorRun(const QJsonObject& p, Call& c);
    QJsonValue signalList(const QJsonObject& p, Call& c);
    QJsonValue signalGet(const QJsonObject& p, Call& c);
    QJsonValue signalSet(const QJsonObject& p, Call& c);
    QJsonValue signalWaitUntil(const QJsonObject& p, Call& c);
    QJsonValue humanoidWalk(const QJsonObject& p, Call& c);
    QJsonValue humanoidWalkTo(const QJsonObject& p, Call& c);
    QJsonValue humanoidWalkPath(const QJsonObject& p, Call& c);
    QJsonValue humanoidSteps(const QJsonObject& p, Call& c);
    QJsonValue humanoidStop(const QJsonObject& p, Call& c);
    QJsonValue humanoidState(const QJsonObject& p, Call& c);
    QJsonValue robotPlay(const QJsonObject& p, Call& c);
    QJsonValue takeStart(const QJsonObject& p, Call& c);
    QJsonValue takeStop(const QJsonObject& p, Call& c);
    QJsonValue takeStatus(const QJsonObject& p, Call& c);
    void       sampleTakes();

    // Building scenes (ApiScene.cpp)
    void        requireEditable() const;
    QMatrix4x4  frameParam(const QJsonObject& p, const QString& key, int parentId, const QString& fallback) const;
    QJsonObject objectInfo(int id) const;
    QJsonObject jointInfo(int jointId, const QMatrix4x4& frame) const;
    int         jointParam(const QJsonObject& p, const QString& key) const;
    int         objectOrNull(const QJsonObject& p, const QString& key) const;
    void        applyObject(int id, const QJsonObject& p, bool creating);
    void        applyJoint(int jointId, const QJsonObject& p, bool creating);
    void        readSensor(SensorConfig& s, const QJsonObject& p, bool zoneOnly);
    QJsonObject sensorJson(const SensorConfig& s) const;
    Operation   readOperation(const QJsonObject& o, const std::vector<Operation>& before);
    QJsonObject operationJson(const Operation& op, const Sequence& seq) const;
    void        answerLater(QTcpSocket* socket, const QJsonValue& id, const QJsonObject& reply);
    QJsonValue  sceneNew(const QJsonObject& p, Call& c);
    QJsonValue  sceneOpen(const QJsonObject& p, Call& c);
    QJsonValue  sceneSave(const QJsonObject& p, Call& c);
    QJsonValue  sceneInfo(const QJsonObject& p, Call& c);
    QJsonValue  sceneCheck(const QJsonObject& p, Call& c);
    QJsonValue  sceneObjects(const QJsonObject& p, Call& c);
    QJsonValue  sceneImport(const QJsonObject& p, Call& c);
    QJsonValue  objectAdd(const QJsonObject& p, Call& c);
    QJsonValue  objectGet(const QJsonObject& p, Call& c);
    QJsonValue  objectSet(const QJsonObject& p, Call& c);
    QJsonValue  objectRemove(const QJsonObject& p, Call& c);
    QJsonValue  objectGroup(const QJsonObject& p, Call& c);
    QJsonValue  objectDuplicate(const QJsonObject& p, Call& c);
    QJsonValue  objectBounds(const QJsonObject& p, Call& c);
    QJsonValue  objectFeatures(const QJsonObject& p, Call& c);
    QJsonValue  jointAdd(const QJsonObject& p, Call& c);
    QJsonValue  jointGet(const QJsonObject& p, Call& c);
    QJsonValue  jointSet(const QJsonObject& p, Call& c);
    QJsonValue  jointRemove(const QJsonObject& p, Call& c);
    QJsonValue  bodySet(const QJsonObject& p, Call& c);
    QJsonValue  beltSet(const QJsonObject& p, Call& c);
    QJsonValue  signalAdd(const QJsonObject& p, Call& c);
    QJsonValue  sensorAdd(const QJsonObject& p, Call& c);
    QJsonValue  poseAdd(const QJsonObject& p, Call& c);
    QJsonValue  sequenceAdd(const QJsonObject& p, Call& c);
    QJsonValue  sequenceGet(const QJsonObject& p, Call& c);
    QJsonValue  sourceAdd(const QJsonObject& p, Call& c);
    QJsonValue  sinkAdd(const QJsonObject& p, Call& c);
    QJsonValue  automationRemove(const QJsonObject& p, Call& c);
    // A walk request for the robot in `p` with the gait options in `p`
    AutomationEngine::WalkRequest walkRequest(const QJsonObject& p, const Robot& r) const;
    QJsonValue submitWalk(const QJsonObject& p, Call& c, AutomationEngine::WalkRequest w, const QString& label);

    SceneGraph*       m_scene;
    SimulationEngine* m_sim;
    QTcpServer*       m_server = nullptr;
    std::vector<QTcpSocket*> m_clients;
    QHash<QTcpSocket*, QByteArray> m_buffers;
    std::vector<Pending> m_pending;
    struct TakeRt {
        QTcpSocket* socket = nullptr;            // dropped when it disconnects
        std::unique_ptr<take::Recorder> recorder;
    };
    std::map<int, TakeRt> m_takes;
    int               m_nextTake = 1;
    QString           m_error;
    QString           m_projectPath;
    std::function<bool()> m_unsaved;
    long              m_requests = 0;
};
