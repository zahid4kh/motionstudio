#include "ModbusDialog.h"
#include "Theme.h"
#include "io/ModbusServer.h"
#include "scene/SceneGraph.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSettings>
#include <QSpinBox>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

namespace {
enum Col { C_Signal, C_Kind, C_Table, C_Address, C_Access, C_Encoding, C_Count };
} // namespace

void ModbusDialog::restore(ModbusServer* server) {
    QSettings s("MotionStudio", "MotionStudio");
    if (!s.value("modbus/enabled", false).toBool()) return;
    server->start(quint16(s.value("modbus/port", 5020).toInt()),
                  s.value("modbus/anyInterface", false).toBool() ? QHostAddress::Any : QHostAddress::LocalHost);
}

ModbusDialog::ModbusDialog(SceneGraph* scene, ModbusServer* server, QWidget* parent)
    : QDialog(parent), m_scene(scene), m_server(server)
{
    setWindowTitle("External I/O");
    setMinimumSize(640, 460);
    auto* vb = new QVBoxLayout(this);
    vb->setContentsMargins(18, 16, 18, 16);
    vb->setSpacing(10);

    auto* title = new QLabel("External I/O — Modbus TCP");
    title->setObjectName("dialogTitle");
    vb->addWidget(title);
    auto* sub = new QLabel("Let a real PLC (or CODESYS, OpenPLC…) drive the model. The PLC writes "
                           "<b>Input</b> signals (coils, holding registers) and reads every other signal "
                           "with an address (discrete inputs, input registers), sensors included. "
                           "Int = one 16-bit register; Real = float32 in two registers, high word first.");
    sub->setObjectName("hint");
    sub->setWordWrap(true);
    vb->addWidget(sub);

    auto* row = new QHBoxLayout;
    QSettings s("MotionStudio", "MotionStudio");
    m_run = new QCheckBox("Run the server");
    m_run->setChecked(m_server->isListening());
    m_port = new QSpinBox;
    m_port->setRange(1, 65535);
    m_port->setValue(m_server->isListening() ? m_server->port() : s.value("modbus/port", 5020).toInt());
    m_port->setToolTip("Modbus TCP uses port 502, which needs administrator rights on Linux; 5020 does not");
    m_bind = new QComboBox;
    m_bind->addItem("This computer only", false);
    m_bind->addItem("All network interfaces", true);
    m_bind->setCurrentIndex(s.value("modbus/anyInterface", false).toBool() ? 1 : 0);
    m_bind->setToolTip("All network interfaces lets a PLC on another machine connect");
    row->addWidget(m_run);
    row->addSpacing(12);
    row->addWidget(new QLabel("Port"));
    row->addWidget(m_port);
    row->addWidget(m_bind);
    row->addStretch();
    vb->addLayout(row);
    m_status = new QLabel;
    m_status->setWordWrap(true);
    vb->addWidget(m_status);

    m_table = new QTableWidget(0, C_Count);
    m_table->setHorizontalHeaderLabels({"Signal", "Kind", "Table", "Address", "PLC", "Encoding"});
    m_table->verticalHeader()->hide();
    m_table->setAlternatingRowColors(true);
    m_table->setSelectionMode(QAbstractItemView::NoSelection);
    m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setStretchLastSection(true);
    m_table->verticalHeader()->setDefaultSectionSize(24);
    vb->addWidget(m_table, 1);

    auto* bar = new QHBoxLayout;
    auto* assign = new QPushButton(Theme::icon("list-ordered"), "Assign Free Addresses");
    assign->setToolTip("Give every signal without an address the next free one in its table");
    auto* clear = new QPushButton(Theme::icon("eraser"), "Clear All");
    clear->setToolTip("Remove every signal's address");
    auto* btns = new QDialogButtonBox(QDialogButtonBox::Close);
    bar->addWidget(assign);
    bar->addWidget(clear);
    bar->addStretch();
    bar->addWidget(btns);
    vb->addLayout(bar);

    connect(btns, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(m_run, &QCheckBox::toggled, this, &ModbusDialog::applyServer);
    connect(m_port, &QSpinBox::editingFinished, this, [this]{ if (m_run->isChecked()) applyServer(); });
    connect(m_bind, &QComboBox::currentIndexChanged, this, [this]{ if (m_run->isChecked()) applyServer(); });
    connect(assign, &QPushButton::clicked, this, [this]{
        ModbusServer::assignAddresses(*m_scene);
        rebuildTable();
    });
    connect(clear, &QPushButton::clicked, this, [this]{
        for (auto& [id, sg] : m_scene->allSignals()) m_scene->getSignal(id)->ioAddress = -1;
        m_scene->notifyAutomationChanged();
        rebuildTable();
    });
    connect(m_table, &QTableWidget::cellChanged, this, [this](int r, int c){
        if (m_building || c != C_Address) return;
        Signal* sg = m_scene->getSignal(m_table->item(r, C_Signal)->data(Qt::UserRole).toInt());
        if (!sg) return;
        const QString text = m_table->item(r, c)->text().trimmed();
        bool ok = false;
        const int v = text.toInt(&ok);
        sg->ioAddress = text.isEmpty() ? -1 : (ok && v >= 0 && v <= 65535 ? v : sg->ioAddress);
        m_scene->notifyAutomationChanged();
        QTimer::singleShot(0, this, &ModbusDialog::rebuildTable);
    });
    connect(m_server, &ModbusServer::clientsChanged, this, &ModbusDialog::updateStatus);
    auto* poll = new QTimer(this);
    poll->setInterval(500);
    connect(poll, &QTimer::timeout, this, &ModbusDialog::updateStatus);
    poll->start();

    rebuildTable();
    updateStatus();
}

void ModbusDialog::applyServer() {
    QSettings s("MotionStudio", "MotionStudio");
    s.setValue("modbus/port", m_port->value());
    s.setValue("modbus/anyInterface", m_bind->currentData().toBool());
    s.setValue("modbus/enabled", m_run->isChecked());
    if (!m_run->isChecked()) {
        m_server->stop();
    } else if (!m_server->start(quint16(m_port->value()),
                                m_bind->currentData().toBool() ? QHostAddress::Any : QHostAddress::LocalHost)) {
        const QSignalBlocker block(m_run);
        m_run->setChecked(false);
        s.setValue("modbus/enabled", false);
    }
    updateStatus();
}

void ModbusDialog::updateStatus() {
    const Theme::Colors& c = Theme::colors();
    QString text;
    if (m_server->isListening())
        text = QString("<span style='color:%1'>● Listening on port %2</span> — %3 client%4 connected, %5 request%6 answered")
                   .arg(Theme::css(c.success)).arg(m_server->port()).arg(m_server->clientCount())
                   .arg(m_server->clientCount() == 1 ? "" : "s").arg(m_server->requestCount())
                   .arg(m_server->requestCount() == 1 ? "" : "s");
    else if (!m_server->lastError().isEmpty())
        text = QString("<span style='color:%1'>Could not start: %2</span>").arg(Theme::css(c.danger), m_server->lastError().toHtmlEscaped());
    else
        text = QString("<span style='color:%1'>Stopped.</span> Values are live while the simulation runs; "
                       "while it is stopped, reads return initial values.").arg(Theme::css(c.muted));
    const QStringList problems = ModbusServer::validate(*m_scene);
    if (!problems.isEmpty())
        text += QString("<br><span style='color:%1'>⚠ %2</span>").arg(Theme::css(c.danger), problems.join("<br>⚠ ").toHtmlEscaped());
    m_status->setText(text);
}

void ModbusDialog::rebuildTable() {
    m_building = true;
    m_table->setRowCount(0);
    int r = 0;
    for (auto& [id, sg] : m_scene->allSignals()) {
        m_table->insertRow(r);
        const ModbusServer::Table t = ModbusServer::tableFor(sg);
        const bool writes = t == ModbusServer::Table::Coil || t == ModbusServer::Table::HoldingRegister;
        auto ro = [](const QString& s) {
            auto* it = new QTableWidgetItem(s);
            it->setFlags(Qt::ItemIsEnabled);
            return it;
        };
        auto* name = ro(sg.name);
        name->setData(Qt::UserRole, id);
        QFont f = name->font(); f.setBold(true); name->setFont(f);
        m_table->setItem(r, C_Signal, name);
        m_table->setItem(r, C_Kind, ro(signalKindName(sg.kind)));
        m_table->setItem(r, C_Table, ro(ModbusServer::tableName(t)));
        auto* addr = new QTableWidgetItem(sg.ioAddress >= 0 ? QString::number(sg.ioAddress) : QString());
        addr->setToolTip("Double-click to type an address; leave it empty to keep the signal off the bus");
        if (sg.ioAddress < 0) addr->setForeground(Theme::colors().muted);
        m_table->setItem(r, C_Address, addr);
        m_table->setItem(r, C_Access, ro(writes ? "writes" : "reads"));
        m_table->setItem(r, C_Encoding, ro(sg.type == SignalType::Bool ? "bit"
                                          : sg.type == SignalType::Int ? "int16, 1 register"
                                                                       : "float32, 2 registers (high word first)"));
        ++r;
    }
    if (r == 0) {
        m_table->insertRow(0);
        auto* it = new QTableWidgetItem("No signals yet: add some on the Signals tab.");
        it->setFlags(Qt::ItemIsEnabled);
        m_table->setItem(0, 0, it);
        m_table->setSpan(0, 0, 1, C_Count);
    }
    m_building = false;
    updateStatus();
}
