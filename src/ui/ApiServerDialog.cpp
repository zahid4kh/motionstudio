#include "ApiServerDialog.h"
#include "HelpDialog.h"
#include "Theme.h"
#include "io/ApiServer.h"
#include "robot/Gait.h"
#include "scene/SceneGraph.h"
#include <QApplication>
#include <QCheckBox>
#include <QClipboard>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSettings>
#include <QSpinBox>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

namespace {

enum Col { C_Robot, C_Joints, C_Tools, C_Count };

// Example values for the quick start: the scene's first robot, else placeholders
struct Example {
    QString robot = "Robot", tool, joint = "J1";
    QString walker;                 // a robot with legs, if there is one
    int port = ApiServer::kDefaultPort;
};

QString quote(const QString& s) { return '"' + QString(s).replace('"', "\\\"") + '"'; }

// Code lines with their comments lined up in one column
QString aligned(const std::vector<std::pair<QString, QString>>& lines, const QString& mark) {
    int width = 0;
    for (const auto& [code, comment] : lines) if (!comment.isEmpty()) width = std::max(width, int(code.size()));
    QString out;
    for (const auto& [code, comment] : lines)
        out += (comment.isEmpty() ? code : code.leftJustified(width + 2) + mark + ' ' + comment) + '\n';
    return out;
}

// The example moves work from any home pose: up and back down in straight lines
QString pythonCode(const Example& e) {
    const QString port = e.port == ApiServer::kDefaultPort ? QString() : QString("port=%1").arg(e.port);
    const QString t = e.tool.isEmpty() ? "robot" : "tool";
    std::vector<std::pair<QString, QString>> l = {
        {"# pip install motionstudio-api", ""},
        {"import motionstudio as ms", ""}, {"", ""},
        {QString("with ms.connect(%1) as studio:").arg(port), ""},
        {QString("    robot = studio.robot(%1)").arg(quote(e.robot)), ""}};
    if (!e.tool.isEmpty()) l.push_back({QString("    tool = robot.tool(%1)").arg(quote(e.tool)), "one of its tools"});
    l.push_back({QString("    print(%1.pose())").arg(t), "mm and degrees, Y up"});
    l.push_back({QString("    %1.move_by(y=50)").arg(t), "50 mm up in a straight line"});
    l.push_back({QString("    %1.move_by(y=-50, speed=100)").arg(t), "and back down at 100 mm/s"});
    l.push_back({QString("    robot.move_joints(%1=30)").arg(e.joint), "a joint move, in degrees"});
    l.push_back({"    robot.home()", ""});
    if (!e.walker.isEmpty()) {
        l.push_back({"", ""});
        l.push_back({QString("    walker = studio.humanoid(%1)").arg(quote(e.walker)), "it has legs"});
        l.push_back({"    walker.walk(1000)", "a metre ahead"});
        l.push_back({"    walker.turn(90)", "a quarter turn to its left"});
        l.push_back({"    walker.walk_path([(800, 0), (800, 800)], frame=\"robot\")", "x ahead, z to its right"});
        l.push_back({"    print(walker.place)", "x, z (mm) and heading"});
    }
    return aligned(l, "#");
}

QString cppCode(const Example& e) {
    const QString port = e.port == ApiServer::kDefaultPort ? QString() : QString("\"localhost\", %1").arg(e.port);
    const QString t = e.tool.isEmpty() ? "robot" : "tool";
    std::vector<std::pair<QString, QString>> l = {
        {"// find_package(motionstudio 0.3), link motionstudio::motionstudio (Help › Robot API)", ""},
        {"#include <motionstudio/motionstudio.hpp>", ""}, {"#include <iostream>", ""},
        {"namespace ms = motionstudio;", ""}, {"", ""}, {"int main() {", ""},
        {QString("    auto studio = ms::Studio::connect(%1);").arg(port), ""},
        {QString("    auto robot = studio.robot(%1);").arg(quote(e.robot)), ""}};
    if (!e.tool.isEmpty()) l.push_back({QString("    auto tool = robot.tool(%1);").arg(quote(e.tool)), "one of its tools"});
    l.push_back({QString("    std::cout << %1.pose() << \"\\n\";").arg(t), "mm and degrees, Y up"});
    l.push_back({QString("    %1.moveBy(ms::Target().y(50));").arg(t), "50 mm up in a straight line"});
    l.push_back({QString("    %1.moveBy(ms::Target().y(-50), {.speed = 100});").arg(t), "and back down at 100 mm/s"});
    l.push_back({QString("    robot.moveJoints({{\"%1\", 30}});").arg(e.joint), "a joint move, in degrees"});
    l.push_back({"    robot.home();", ""});
    if (!e.walker.isEmpty()) {
        l.push_back({"", ""});
        l.push_back({QString("    auto walker = studio.humanoid(%1);").arg(quote(e.walker)), "it has legs"});
        l.push_back({"    walker.walk(1000);", "a metre ahead"});
        l.push_back({"    walker.turn(90);", "a quarter turn to its left"});
        l.push_back({"    walker.walkPath({{800, 0}, {800, 800}}, {}, {.frame = \"robot\"});", "x ahead, z to its right"});
        l.push_back({"    std::cout << walker.place() << \"\\n\";", "x, z (mm) and heading"});
    }
    l.push_back({"}", ""});
    return aligned(l, "//");
}

QString kotlinCode(const Example& e) {
    const QString port = e.port == ApiServer::kDefaultPort ? QString() : QString("port = %1").arg(e.port);
    const QString t = e.tool.isEmpty() ? "robot" : "tool";
    std::vector<std::pair<QString, QString>> l = {
        {"// Maven Central: implementation(\"io.github.zahid4kh:motionstudio:0.3.0\")", ""},
        {"import io.motionstudio.MotionStudio", ""}, {"", ""},
        {QString("fun main() = MotionStudio.connect(%1).use { studio ->").arg(port), ""},
        {QString("    val robot = studio.robot(%1)").arg(quote(e.robot)), ""}};
    if (!e.tool.isEmpty()) l.push_back({QString("    val tool = robot.tool(%1)").arg(quote(e.tool)), "one of its tools"});
    l.push_back({QString("    println(%1.pose())").arg(t), "mm and degrees, Y up"});
    l.push_back({QString("    %1.moveBy(y = 50)").arg(t), "50 mm up in a straight line"});
    l.push_back({QString("    %1.moveBy(y = -50, speed = 100)").arg(t), "and back down at 100 mm/s"});
    l.push_back({QString("    robot.moveJoints(\"%1\" to 30)").arg(e.joint), "a joint move, in degrees"});
    l.push_back({"    robot.home()", ""});
    if (!e.walker.isEmpty()) {
        l.push_back({"", ""});
        l.push_back({QString("    val walker = studio.humanoid(%1)").arg(quote(e.walker)), "it has legs"});
        l.push_back({"    walker.walk(1000)", "a metre ahead"});
        l.push_back({"    walker.turn(90)", "a quarter turn to its left"});
        l.push_back({"    walker.walkPath(800 to 0, 800 to 800, frame = \"robot\")", "x ahead, z to its right"});
        l.push_back({"    println(walker.place)", "x, z (mm) and heading"});
    }
    l.push_back({"}", ""});
    return aligned(l, "//");
}

} // namespace

void ApiServerDialog::restore(ApiServer* server) {
    QSettings s("MotionStudio", "MotionStudio");
    if (!s.value("api/enabled", true).toBool()) return;
    server->start(quint16(s.value("api/port", ApiServer::kDefaultPort).toInt()),
                  s.value("api/anyInterface", false).toBool() ? QHostAddress::Any : QHostAddress::LocalHost);
}

ApiServerDialog::ApiServerDialog(SceneGraph* scene, ApiServer* server, QWidget* parent)
    : QDialog(parent), m_scene(scene), m_server(server)
{
    setWindowTitle("Robot API");
    setMinimumSize(720, 620);
    auto* vb = new QVBoxLayout(this);
    vb->setContentsMargins(18, 16, 18, 16);
    vb->setSpacing(10);

    auto* title = new QLabel("Robot API — control the model from code");
    title->setObjectName("dialogTitle");
    vb->addWidget(title);
    auto* sub = new QLabel("Programs in <b>Python</b>, <b>C++</b> or <b>Kotlin</b> connect here to move robots "
                           "(straight lines, joint moves, taught poses), walk humanoids along paths, and read and "
                           "write signals. The libraries "
                           "are in <b>api/</b> of the MotionStudio source. Lengths are millimetres, angles degrees; "
                           "Y is up. A move starts the simulation if it isn't running.");
    sub->setObjectName("hint");
    sub->setWordWrap(true);
    vb->addWidget(sub);

    auto* row = new QHBoxLayout;
    QSettings s("MotionStudio", "MotionStudio");
    m_run = new QCheckBox("Run the server");
    m_run->setChecked(m_server->isListening());
    m_port = new QSpinBox;
    m_port->setRange(1, 65535);
    m_port->setValue(m_server->isListening() ? m_server->port() : s.value("api/port", ApiServer::kDefaultPort).toInt());
    m_bind = new QComboBox;
    m_bind->addItem("This computer only", false);
    m_bind->addItem("All network interfaces", true);
    m_bind->setCurrentIndex(s.value("api/anyInterface", false).toBool() ? 1 : 0);
    m_bind->setToolTip("All network interfaces lets programs on other machines connect: anyone on the network "
                       "can then move the model");
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

    auto* robotsLabel = new QLabel("Robots in this scene (address them by name, or by path when names repeat)");
    robotsLabel->setObjectName("fieldLabel");
    vb->addWidget(robotsLabel);
    m_robots = new QTableWidget(0, C_Count);
    m_robots->setHorizontalHeaderLabels({"Robot", "Joints (actuators)", "Tools"});
    m_robots->verticalHeader()->hide();
    m_robots->setAlternatingRowColors(true);
    m_robots->setSelectionMode(QAbstractItemView::NoSelection);
    m_robots->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_robots->horizontalHeader()->setSectionResizeMode(C_Robot, QHeaderView::ResizeToContents);
    m_robots->horizontalHeader()->setSectionResizeMode(C_Joints, QHeaderView::Stretch);
    m_robots->horizontalHeader()->setSectionResizeMode(C_Tools, QHeaderView::ResizeToContents);
    m_robots->verticalHeader()->setDefaultSectionSize(24);
    m_robots->setWordWrap(false);
    vb->addWidget(m_robots, 1);

    auto* quickLabel = new QLabel("Quick start for this scene");
    quickLabel->setObjectName("fieldLabel");
    vb->addWidget(quickLabel);
    m_langs = new QTabWidget;
    for (const char* lang : {"Python", "C++", "Kotlin"}) {
        auto* code = new QPlainTextEdit;
        code->setObjectName("code");
        code->setReadOnly(true);
        code->setLineWrapMode(QPlainTextEdit::NoWrap);
        m_code.push_back(code);
        m_langs->addTab(code, lang);
    }
    vb->addWidget(m_langs, 2);

    auto* bar = new QHBoxLayout;
    auto* copy = new QPushButton(Theme::icon("copy"), "Copy Code");
    auto* guide = new QPushButton(Theme::icon("book-open"), "Guide");
    guide->setToolTip("Every command, in all three languages (Help › Robot API)");
    auto* btns = new QDialogButtonBox(QDialogButtonBox::Close);
    bar->addWidget(copy);
    bar->addWidget(guide);
    bar->addStretch();
    bar->addWidget(btns);
    vb->addLayout(bar);

    connect(btns, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(copy, &QPushButton::clicked, this, [this] {
        QApplication::clipboard()->setText(m_code[size_t(m_langs->currentIndex())]->toPlainText());
    });
    connect(guide, &QPushButton::clicked, this, [this] { HelpDialog::showTopic("api", this); });
    connect(m_run, &QCheckBox::toggled, this, &ApiServerDialog::applyServer);
    connect(m_port, &QSpinBox::editingFinished, this, [this] { if (m_run->isChecked()) applyServer(); });
    connect(m_bind, &QComboBox::currentIndexChanged, this, [this] { if (m_run->isChecked()) applyServer(); });
    connect(m_server, &ApiServer::clientsChanged, this, &ApiServerDialog::updateStatus);
    auto* poll = new QTimer(this);
    poll->setInterval(500);
    connect(poll, &QTimer::timeout, this, &ApiServerDialog::updateStatus);
    poll->start();

    rebuild();
    updateStatus();
}

void ApiServerDialog::applyServer() {
    QSettings s("MotionStudio", "MotionStudio");
    s.setValue("api/port", m_port->value());
    s.setValue("api/anyInterface", m_bind->currentData().toBool());
    s.setValue("api/enabled", m_run->isChecked());
    if (!m_run->isChecked()) {
        m_server->stop();
    } else if (!m_server->start(quint16(m_port->value()),
                                m_bind->currentData().toBool() ? QHostAddress::Any : QHostAddress::LocalHost)) {
        const QSignalBlocker block(m_run);
        m_run->setChecked(false);
    }
    rebuild();
    updateStatus();
}

void ApiServerDialog::updateStatus() {
    const Theme::Colors& c = Theme::colors();
    QString text;
    if (m_server->isListening())
        text = QString("<span style='color:%1'>● Listening on port %2</span> — %3 program%4 connected, %5 request%6 answered")
                   .arg(Theme::css(c.success)).arg(m_server->port()).arg(m_server->clientCount())
                   .arg(m_server->clientCount() == 1 ? "" : "s").arg(m_server->requestCount())
                   .arg(m_server->requestCount() == 1 ? "" : "s");
    else if (!m_server->lastError().isEmpty())
        text = QString("<span style='color:%1'>Could not start: %2</span> (is another MotionStudio using the port?)")
                   .arg(Theme::css(c.danger), m_server->lastError().toHtmlEscaped());
    else
        text = QString("<span style='color:%1'>Stopped.</span> Programs can't connect.").arg(Theme::css(c.muted));
    m_status->setText(text);
}

void ApiServerDialog::rebuild() {
    m_robots->setRowCount(0);
    const auto robots = ApiServer::findRobots(*m_scene);
    Example ex;
    if (m_server->isListening()) ex.port = m_server->port();
    else ex.port = m_port->value();
    int r = 0;
    for (const auto& rb : robots) {
        QStringList joints, tools;
        for (int j : rb.joints) if (const Motor* m = m_scene->motorForJoint(j)) joints << m->name;
        for (int t : rb.tools) tools << m_scene->getObject(t)->name;
        m_robots->insertRow(r);
        auto* name = new QTableWidgetItem(rb.path);
        QFont f = name->font(); f.setBold(true); name->setFont(f);
        if (gait::canWalk(*m_scene, rb.root)) {
            name->setIcon(Theme::icon("footprints"));
            name->setToolTip("It walks: humanoid.walk / walkTo / walkPath");
            if (ex.walker.isEmpty()) ex.walker = rb.path;
        }
        m_robots->setItem(r, C_Robot, name);
        auto* j = new QTableWidgetItem(joints.join(", "));
        j->setToolTip(joints.join(", "));
        m_robots->setItem(r, C_Joints, j);
        m_robots->setItem(r, C_Tools, new QTableWidgetItem(tools.isEmpty() ? QString("—") : tools.join(", ")));
        ++r;
    }
    // The first robot with a tool makes the example
    for (const auto& rb : robots)
        if (!rb.tools.empty()) {
            QStringList tools;
            for (int t : rb.tools) tools << m_scene->getObject(t)->name;
            ex.robot = rb.path;
            if (const Motor* m = rb.joints.empty() ? nullptr : m_scene->motorForJoint(rb.joints.front())) ex.joint = m->name;
            ex.tool = tools.size() > 1 ? tools.back() : QString();
            break;
        }
    if (r == 0) {
        m_robots->insertRow(0);
        auto* it = new QTableWidgetItem("No robots yet: build one with Tools › Robot Builder or Humanoid Builder, "
                                        "or give a joint an actuator.");
        m_robots->setItem(0, 0, it);
        m_robots->setSpan(0, 0, 1, C_Count);
    }
    m_code[0]->setPlainText(pythonCode(ex));
    m_code[1]->setPlainText(cppCode(ex));
    m_code[2]->setPlainText(kotlinCode(ex));
}
