#include "ValueRecorder.h"
#include "Theme.h"
#include "../automation/Take.h"
#include "../io/ApiServer.h"
#include "../physics/SimulationEngine.h"
#include "../robot/Gait.h"
#include "../scene/SceneGraph.h"
#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSettings>
#include <QTimer>
#include <QVBoxLayout>
#include <cmath>

namespace {
constexpr double kStillFor = 0.5;    // s without motion that ends a take with "stop when it stands still"
constexpr double kStillTol = 0.01;   // mm or degrees
}

ValueRecorder::ValueRecorder(SceneGraph* scene, SimulationEngine* sim, QWidget* window)
    : QObject(window), m_scene(scene), m_sim(sim), m_window(window)
{
    m_action = new QAction(this);
    m_action->setCheckable(true);
    connect(m_action, &QAction::triggered, this, &ValueRecorder::toggle);
    connect(m_sim->automation(), &AutomationEngine::scanned, this, &ValueRecorder::onScan);
    connect(m_sim, &SimulationEngine::activeChanged, this, [this](bool active) {
        if (!active && m_rec) finish("the simulation was reset");
    });
    sync();
}

ValueRecorder::~ValueRecorder() = default;

void ValueRecorder::sync() {
    const bool on = recording();
    m_action->setChecked(on);
    m_action->setText(on ? "Stop Recording" : "Record Values");
    m_action->setIconText(on ? "Stop" : "Record");   // the toolbar's short label
    m_action->setIcon(Theme::icon(on ? "circle-stop" : "circle-dot"));
    m_action->setToolTip(on ? "Stop recording and save the values"
                            : "Record a robot's joint values and where it stands while it moves,\n"
                              "and save them to a file (a CSV take for analysis or machine learning)");
}

void ValueRecorder::toggle() {
    if (m_rec) finish({});
    else if (!begin()) sync();
}

bool ValueRecorder::begin() {
    const std::vector<ApiServer::Robot> robots = ApiServer::findRobots(*m_scene);
    if (robots.empty()) {
        QMessageBox::information(m_window, "Record Values",
                                 "There is no robot in the scene: nothing with actuated joints to record.\n\n"
                                 "Build one with Tools › Humanoid Builder or Tools › Robot Builder.");
        return false;
    }
    QSettings settings("MotionStudio", "MotionStudio");
    QDialog dlg(m_window);
    dlg.setWindowTitle("Record Values");
    auto* vb = new QVBoxLayout(&dlg);
    vb->setContentsMargins(18, 16, 18, 16);
    vb->setSpacing(10);
    auto* title = new QLabel("Record a robot's values");
    title->setObjectName("dialogTitle");
    vb->addWidget(title);
    auto* sub = new QLabel("While the simulation runs, its joint values and where it stands are written to a CSV "
                           "file, one row per sample. A walking robot also gets its step, its swinging foot and "
                           "where the walk goes. Recording starts Play; press Stop to save.");
    sub->setObjectName("hint");
    sub->setWordWrap(true);
    sub->setMinimumWidth(440);
    vb->addWidget(sub);

    auto* form = new QFormLayout;
    form->setLabelAlignment(Qt::AlignRight);
    auto* robot = new QComboBox;
    const QString lastRobot = settings.value("take/robot").toString();
    for (size_t i = 0; i < robots.size(); ++i) {
        const bool walks = gait::canWalk(*m_scene, robots[i].root);
        robot->addItem(QString("%1  ·  %2 joints%3").arg(robots[i].name).arg(robots[i].joints.size())
                           .arg(walks ? ", walks" : ""), int(i));
        if (robots[i].name == lastRobot) robot->setCurrentIndex(int(i));
    }
    // Prefer the selected robot
    for (size_t i = 0; i < robots.size(); ++i)
        for (const SceneObject* o = m_scene->getObject(m_scene->selectedObjectId()); o; o = m_scene->getObject(o->parentId))
            if (o->id == robots[i].root) robot->setCurrentIndex(int(i));
    auto* rate = new QComboBox;
    rate->addItem("Every scan (62.5 per second)", 0.0);
    for (double r : {50.0, 25.0, 10.0}) rate->addItem(QString("%1 per second").arg(r), r);
    rate->setCurrentIndex(std::max(0, rate->findData(settings.value("take/rate", 0.0).toDouble())));
    auto* file = new QLineEdit;
    auto* browse = new QPushButton("Browse…");
    browse->setAutoDefault(false);   // Enter in the File field records, it doesn't browse
    auto* fileRow = new QHBoxLayout;
    fileRow->addWidget(file, 1);
    fileRow->addWidget(browse);
    const QString dir = settings.value("take/dir", QDir::homePath()).toString();
    auto suggest = [&] {
        const QString name = robots[size_t(robot->currentData().toInt())].name;
        file->setText(QDir(dir).filePath(QString("%1-%2.csv").arg(QString(name).replace(' ', '-').toLower(),
                                                                  QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss"))));
    };
    suggest();
    connect(robot, &QComboBox::currentIndexChanged, &dlg, suggest);
    connect(browse, &QPushButton::clicked, &dlg, [&] {
        const QString f = QFileDialog::getSaveFileName(&dlg, "Save the values as", file->text(), "CSV take (*.csv)");
        if (!f.isEmpty()) file->setText(QFileInfo(f).suffix().isEmpty() ? f + ".csv" : f);
    });
    auto* still = new QCheckBox("Stop when it stands still again");
    still->setToolTip(QString("Ends the take %1 s after the robot stops moving (once it has moved),\n"
                              "e.g. when a walk from A to B arrives").arg(kStillFor));
    still->setChecked(settings.value("take/stopWhenStill", true).toBool());
    form->addRow("Robot", robot);
    form->addRow("Samples", rate);
    form->addRow("File", fileRow);
    form->addRow("", still);
    vb->addLayout(form);
    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    btns->button(QDialogButtonBox::Ok)->setText("Record");
    btns->button(QDialogButtonBox::Ok)->setIcon(Theme::icon("circle-dot"));
    btns->button(QDialogButtonBox::Ok)->setDefault(true);
    btns->button(QDialogButtonBox::Cancel)->setAutoDefault(false);   // Enter records, never cancels
    vb->addWidget(btns);
    connect(file, &QLineEdit::returnPressed, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    if (dlg.exec() != QDialog::Accepted) return false;

    const ApiServer::Robot& r = robots[size_t(robot->currentData().toInt())];
    m_path = file->text().trimmed();
    if (m_path.isEmpty()) return false;
    if (QFileInfo(m_path).suffix().isEmpty()) m_path += ".csv";
    settings.setValue("take/robot", r.name);
    settings.setValue("take/rate", rate->currentData().toDouble());
    settings.setValue("take/dir", QFileInfo(m_path).absolutePath());
    settings.setValue("take/stopWhenStill", still->isChecked());

    m_rec = std::make_unique<take::Recorder>(*m_scene, r.root, r.joints, rate->currentData().toDouble());
    m_stopWhenStill = still->isChecked();
    m_moved = false;
    m_stillSince = -1;
    m_compare.clear();
    const QStringList& cols = m_rec->take().columns;
    for (int i = 1; i < cols.size(); ++i)
        if (cols[i] != "progress" && cols[i] != "step" && cols[i] != "swing") m_compare.push_back(i);
    sync();
    if (!m_sim->isRunning()) m_sim->play();
    if (!m_sim->isActive()) {   // the automation has errors (MainWindow says which)
        m_rec.reset();
        sync();
        return true;
    }
    emit statusMessage(QString("Recording %1 — press Stop to save").arg(r.name), true);
    return true;
}

void ValueRecorder::onScan() {
    if (!m_rec) return;
    const int before = m_rec->frames();
    m_rec->sample(*m_sim->automation());
    if (!m_stopWhenStill || m_rec->frames() == before || m_rec->frames() < 2) return;
    const auto& rows = m_rec->take().rows;
    const std::vector<double>& a = rows[rows.size() - 2];
    const std::vector<double>& b = rows.back();
    bool moving = false;
    for (int i : m_compare)
        if (std::fabs(a[size_t(i)] - b[size_t(i)]) > kStillTol) { moving = true; break; }
    if (moving) { m_moved = true; m_stillSince = -1; return; }
    if (!m_moved) return;
    if (m_stillSince < 0) m_stillSince = b.front();
    if (b.front() - m_stillSince >= kStillFor - 1e-9) finish("it stands still");
}

void ValueRecorder::finish(const QString& why) {
    if (!m_rec) return;
    std::unique_ptr<take::Recorder> rec = std::move(m_rec);
    sync();
    const take::Take& t = rec->take();
    if (t.rows.empty()) {
        emit statusMessage("Recording stopped — nothing was recorded (the simulation did not run)", false);
        return;
    }
    QString error;
    if (!t.save(m_path, &error)) {
        QMessageBox::warning(m_window, "Record Values", QString("Could not save %1:\n\n%2").arg(m_path, error));
        return;
    }
    const QString msg = QString("Saved %1 rows × %2 values of %3 (%4 s)%5 to %6")
                            .arg(t.rows.size()).arg(t.columns.size()).arg(t.robot)
                            .arg(t.duration(), 0, 'f', 2).arg(why.isEmpty() ? QString() : " — stopped: " + why)
                            .arg(QDir::toNativeSeparators(m_path));
    // After the message of whatever stopped it (Reset says it restored the scene)
    QTimer::singleShot(0, this, [this, msg] { emit statusMessage(msg, true); });
}
