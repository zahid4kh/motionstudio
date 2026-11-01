#include "ModbusServer.h"
#include "../scene/SceneGraph.h"
#include "../physics/SimulationEngine.h"
#include <QTcpServer>
#include <QTcpSocket>
#include <QtEndian>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

enum Exception : quint8 { IllegalFunction = 1, IllegalAddress = 2, IllegalValue = 3 };

quint16 be16(const QByteArray& b, int at) {
    return quint16((quint8(b[at]) << 8) | quint8(b[at + 1]));
}
void put16(QByteArray& b, quint16 v) {
    b.append(char(v >> 8));
    b.append(char(v & 0xff));
}

QByteArray exception(quint8 fc, quint8 code) {
    QByteArray r;
    r.append(char(fc | 0x80));
    r.append(char(code));
    return r;
}

// Register words of a value: Int → one signed word, Real → float32 high word first
void toWords(SignalType t, double v, quint16 out[2]) {
    if (t == SignalType::Real) {
        const float f = float(v);
        quint32 bits;
        std::memcpy(&bits, &f, 4);
        out[0] = quint16(bits >> 16);
        out[1] = quint16(bits & 0xffff);
    } else {
        const double c = std::clamp(std::round(v), -32768.0, 32767.0);
        out[0] = quint16(qint16(c));
        out[1] = 0;
    }
}
double fromWords(SignalType t, const quint16 w[2]) {
    if (t == SignalType::Real) {
        const quint32 bits = (quint32(w[0]) << 16) | w[1];
        float f;
        std::memcpy(&f, &bits, 4);
        return double(f);
    }
    return double(qint16(w[0]));
}

} // namespace

ModbusServer::ModbusServer(SceneGraph* scene, SimulationEngine* sim, QObject* parent)
    : QObject(parent), m_scene(scene), m_sim(sim) {}

ModbusServer::~ModbusServer() { stop(); }

// ── Map ──────────────────────────────────────────────────────────────────────

ModbusServer::Table ModbusServer::tableFor(const Signal& s) {
    const bool input = s.kind == SignalKind::Input;
    const bool bit = s.type == SignalType::Bool;
    if (input) return bit ? Table::Coil : Table::HoldingRegister;
    return bit ? Table::DiscreteInput : Table::InputRegister;
}

QString ModbusServer::tableName(Table t) {
    switch (t) {
        case Table::Coil:            return "Coil";
        case Table::DiscreteInput:   return "Discrete input";
        case Table::HoldingRegister: return "Holding register";
        case Table::InputRegister:   return "Input register";
    }
    return {};
}

std::vector<ModbusServer::Entry> ModbusServer::buildMap(const SceneGraph& scene) {
    std::vector<Entry> out;
    for (auto& [id, s] : scene.allSignals()) {
        if (s.ioAddress < 0) continue;
        Entry e;
        e.table = tableFor(s);
        e.address = s.ioAddress;
        e.words = s.type == SignalType::Real ? 2 : 1;
        e.signal = s.name;
        e.type = s.type;
        out.push_back(e);
    }
    std::sort(out.begin(), out.end(), [](const Entry& a, const Entry& b) {
        return std::make_pair(int(a.table), a.address) < std::make_pair(int(b.table), b.address);
    });
    return out;
}

QStringList ModbusServer::validate(const SceneGraph& scene) {
    QStringList errs;
    const auto map = buildMap(scene);
    for (size_t i = 0; i + 1 < map.size(); ++i) {
        const Entry& a = map[i];
        const Entry& b = map[i + 1];
        if (a.table == b.table && b.address < a.address + a.words)
            errs << QString("Signal %1: %2 %3 is also used by %4").arg(b.signal, tableName(b.table).toLower())
                        .arg(b.address).arg(a.signal);
    }
    for (const Entry& e : map)
        if (e.address + e.words - 1 > 65535) errs << QString("Signal %1: address above 65535").arg(e.signal);
    return errs;
}

int ModbusServer::assignAddresses(SceneGraph& scene) {
    std::map<Table, int> next;
    for (const Entry& e : buildMap(scene)) next[e.table] = std::max(next[e.table], e.address + e.words);
    int assigned = 0;
    for (auto& [id, s] : scene.allSignals()) {
        if (s.ioAddress >= 0) continue;
        Signal* sig = scene.getSignal(id);
        const Table t = tableFor(*sig);
        sig->ioAddress = next[t];
        next[t] += sig->type == SignalType::Real ? 2 : 1;
        ++assigned;
    }
    if (assigned) scene.notifyAutomationChanged();
    return assigned;
}

// ── Server ───────────────────────────────────────────────────────────────────

bool ModbusServer::start(quint16 port, const QHostAddress& address) {
    stop();
    m_server = new QTcpServer(this);
    if (!m_server->listen(address, port)) {
        m_error = m_server->errorString();
        delete m_server;
        m_server = nullptr;
        return false;
    }
    m_error.clear();
    m_requests = 0;
    m_holdingImage.clear();
    connect(m_server, &QTcpServer::newConnection, this, &ModbusServer::onNewConnection);
    return true;
}

void ModbusServer::stop() {
    for (QTcpSocket* s : m_clients) {
        s->disconnect(this);
        s->abort();
        s->deleteLater();
    }
    const bool had = !m_clients.empty();
    m_clients.clear();
    m_buffers.clear();
    if (m_server) {
        m_server->close();
        m_server->deleteLater();
        m_server = nullptr;
    }
    if (had) emit clientsChanged(0);
}

bool ModbusServer::isListening() const { return m_server && m_server->isListening(); }
quint16 ModbusServer::port() const { return m_server ? m_server->serverPort() : 0; }

void ModbusServer::onNewConnection() {
    while (QTcpSocket* s = m_server->nextPendingConnection()) {
        m_clients.push_back(s);
        connect(s, &QTcpSocket::readyRead, this, [this, s] { onReadyRead(s); });
        connect(s, &QTcpSocket::disconnected, this, [this, s] {
            m_clients.erase(std::remove(m_clients.begin(), m_clients.end(), s), m_clients.end());
            m_buffers.remove(s);
            s->deleteLater();
            emit clientsChanged(int(m_clients.size()));
        });
        emit clientsChanged(int(m_clients.size()));
    }
}

void ModbusServer::onReadyRead(QTcpSocket* socket) {
    QByteArray& buf = m_buffers[socket];
    buf.append(socket->readAll());
    // MBAP header: transaction (2), protocol (2) = 0, length (2), unit (1)
    while (buf.size() >= 7) {
        const quint16 length = be16(buf, 4);
        if (length < 2 || length > 256) { socket->abort(); return; }   // not Modbus
        if (buf.size() < 6 + length) return;
        const QByteArray frame = buf.left(6 + length);
        buf.remove(0, 6 + length);
        if (be16(frame, 2) != 0) continue;
        const QByteArray resp = handle(frame.mid(7));
        QByteArray out = frame.left(4);
        put16(out, quint16(resp.size() + 1));
        out.append(frame[6]);
        out.append(resp);
        socket->write(out);
        ++m_requests;
        emit activity();
    }
}

double ModbusServer::readValue(const Entry& e) const {
    if (m_sim->automation()->isActive()) return m_sim->automation()->value(e.signal);
    for (auto& [id, s] : m_scene->allSignals())
        if (s.name == e.signal) return s.kind == SignalKind::Input || s.kind == SignalKind::Internal ? s.initialValue : 0.0;
    return 0.0;
}

void ModbusServer::writeValue(const Entry& e, double v) {
    if (m_sim->automation()->isActive()) m_sim->automation()->setInput(e.signal, v);
}

QByteArray ModbusServer::handle(const QByteArray& pdu) {
    if (pdu.isEmpty()) return exception(0, IllegalFunction);
    const quint8 fc = quint8(pdu[0]);
    const std::vector<Entry> map = buildMap(*m_scene);
    auto entryAt = [&](Table t, int address) -> const Entry* {
        for (const Entry& e : map)
            if (e.table == t && address >= e.address && address < e.address + e.words) return &e;
        return nullptr;
    };
    auto wordAt = [&](Table t, int address) -> quint16 {
        const Entry* e = entryAt(t, address);
        if (!e) return 0;
        quint16 w[2];
        toWords(e->type, readValue(*e), w);
        return w[address - e->address];
    };

    switch (fc) {
        case 1: case 2: {   // read coils / discrete inputs
            if (pdu.size() < 5) return exception(fc, IllegalValue);
            const int start = be16(pdu, 1), qty = be16(pdu, 3);
            if (qty < 1 || qty > 2000) return exception(fc, IllegalValue);
            if (start + qty > 65536) return exception(fc, IllegalAddress);
            const Table t = fc == 1 ? Table::Coil : Table::DiscreteInput;
            QByteArray bits((qty + 7) / 8, '\0');
            for (int i = 0; i < qty; ++i) {
                const Entry* e = entryAt(t, start + i);
                if (e && readValue(*e) != 0.0) bits[i / 8] = char(quint8(bits[i / 8]) | (1 << (i % 8)));
            }
            QByteArray r;
            r.append(char(fc));
            r.append(char(bits.size()));
            r.append(bits);
            return r;
        }
        case 3: case 4: {   // read holding / input registers
            if (pdu.size() < 5) return exception(fc, IllegalValue);
            const int start = be16(pdu, 1), qty = be16(pdu, 3);
            if (qty < 1 || qty > 125) return exception(fc, IllegalValue);
            if (start + qty > 65536) return exception(fc, IllegalAddress);
            const Table t = fc == 3 ? Table::HoldingRegister : Table::InputRegister;
            QByteArray r;
            r.append(char(fc));
            r.append(char(qty * 2));
            for (int i = 0; i < qty; ++i) put16(r, wordAt(t, start + i));
            return r;
        }
        case 5: {           // write single coil
            if (pdu.size() < 5) return exception(fc, IllegalValue);
            const int address = be16(pdu, 1);
            const quint16 v = be16(pdu, 3);
            if (v != 0xFF00 && v != 0x0000) return exception(fc, IllegalValue);
            const Entry* e = entryAt(Table::Coil, address);
            if (!e) return exception(fc, IllegalAddress);
            writeValue(*e, v == 0xFF00 ? 1.0 : 0.0);
            return pdu.left(5);
        }
        case 15: {          // write multiple coils
            if (pdu.size() < 6) return exception(fc, IllegalValue);
            const int start = be16(pdu, 1), qty = be16(pdu, 3), bytes = quint8(pdu[5]);
            if (qty < 1 || qty > 1968 || bytes != (qty + 7) / 8 || pdu.size() < 6 + bytes)
                return exception(fc, IllegalValue);
            for (int i = 0; i < qty; ++i)
                if (const Entry* e = entryAt(Table::Coil, start + i))
                    writeValue(*e, (quint8(pdu[6 + i / 8]) >> (i % 8)) & 1 ? 1.0 : 0.0);
            return pdu.left(5);
        }
        case 6: case 16: {  // write single / multiple registers
            int start, qty, data;
            if (fc == 6) {
                if (pdu.size() < 5) return exception(fc, IllegalValue);
                start = be16(pdu, 1); qty = 1; data = 3;
                if (!entryAt(Table::HoldingRegister, start)) return exception(fc, IllegalAddress);
            } else {
                if (pdu.size() < 6) return exception(fc, IllegalValue);
                start = be16(pdu, 1); qty = be16(pdu, 3); data = 6;
                if (qty < 1 || qty > 123 || quint8(pdu[5]) != qty * 2 || pdu.size() < 6 + qty * 2)
                    return exception(fc, IllegalValue);
            }
            // Words of every touched signal: as they are now, then the
            // written ones; a Real's two halves may arrive separately
            std::vector<const Entry*> touched;
            for (int i = 0; i < qty; ++i)
                if (const Entry* e = entryAt(Table::HoldingRegister, start + i))
                    if (std::find(touched.begin(), touched.end(), e) == touched.end()) touched.push_back(e);
            for (const Entry* e : touched)
                for (int k = 0; k < e->words; ++k)
                    if (!m_holdingImage.count(e->address + k)) m_holdingImage[e->address + k] = wordAt(Table::HoldingRegister, e->address + k);
            for (int i = 0; i < qty; ++i) m_holdingImage[start + i] = be16(pdu, data + 2 * i);
            for (const Entry* e : touched) {
                const quint16 w[2] = {m_holdingImage[e->address], e->words > 1 ? m_holdingImage[e->address + 1] : quint16(0)};
                writeValue(*e, fromWords(e->type, w));
            }
            return pdu.left(5);   // echo of address and value / start and quantity
        }
        default:
            return exception(fc, IllegalFunction);
    }
}
