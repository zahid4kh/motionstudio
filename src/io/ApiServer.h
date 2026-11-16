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
    long              m_requests = 0;
};
