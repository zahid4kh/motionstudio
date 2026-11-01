#pragma once
#include <QObject>
#include <QHostAddress>
#include <QHash>
#include <QStringList>
#include <map>
#include "../automation/Automation.h"

class QTcpServer;
class QTcpSocket;
class SceneGraph;
class SimulationEngine;

// Modbus TCP server exposing signals to an external controller (a real PLC,
// CODESYS, OpenPLC…) for virtual commissioning. Each signal with an I/O
// address (Signal::ioAddress) appears in one table, by kind and type:
//
//   Input  Bool         → coil              (PLC writes, function 5/15; read 1)
//   Input  Int/Real     → holding register  (PLC writes, function 6/16; read 3)
//   other  Bool         → discrete input    (PLC reads, function 2)
//   other  Int/Real     → input register    (PLC reads, function 4)
//
// Int is one 16-bit register (signed, clamped); Real is a 32-bit float in two
// registers, high word first. While the simulation is stopped, reads return
// initial values and writes are accepted but have no effect.
class ModbusServer : public QObject {
    Q_OBJECT
public:
    enum class Table { Coil, DiscreteInput, HoldingRegister, InputRegister };
    struct Entry {
        Table   table = Table::Coil;
        int     address = -1;
        int     words = 1;      // registers taken (2 for Real)
        QString signal;
        SignalType type = SignalType::Bool;
    };

    ModbusServer(SceneGraph* scene, SimulationEngine* sim, QObject* parent = nullptr);
    ~ModbusServer() override;

    bool    start(quint16 port, const QHostAddress& address = QHostAddress::LocalHost);
    void    stop();
    bool    isListening() const;
    quint16 port() const;
    int     clientCount() const { return int(m_clients.size()); }
    QString lastError() const { return m_error; }
    // Requests answered since start()
    long    requestCount() const { return m_requests; }

    // The table a signal lands in, and the map of every addressed signal
    static Table tableFor(const Signal& s);
    static QString tableName(Table t);
    static std::vector<Entry> buildMap(const SceneGraph& scene);
    // "Where: problem" lines for overlapping addresses
    static QStringList validate(const SceneGraph& scene);
    // Gives every signal without an address the next free one in its table
    static int assignAddresses(SceneGraph& scene);

signals:
    void clientsChanged(int count);
    void activity();

private:
    void onNewConnection();
    void onReadyRead(QTcpSocket* socket);
    QByteArray handle(const QByteArray& pdu);
    double  readValue(const Entry& e) const;
    void    writeValue(const Entry& e, double v);

    SceneGraph*       m_scene;
    SimulationEngine* m_sim;
    QTcpServer*       m_server = nullptr;
    std::vector<QTcpSocket*> m_clients;
    QHash<QTcpSocket*, QByteArray> m_buffers;
    QString           m_error;
    long              m_requests = 0;
    // Holding-register words as last written, so the halves of a Real can
    // arrive in separate requests
    std::map<int, quint16> m_holdingImage;
};
