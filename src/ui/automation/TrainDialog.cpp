#include "TrainDialog.h"
#include "io/ApiServer.h"
#include "scene/SceneGraph.h"
#include "ui/Theme.h"
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QProgressBar>
#include <QPushButton>
#include <QSettings>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrentRun>

namespace {
struct Effort { const char* label; int epochs; };
const Effort kEfforts[] = {
    {"Quick (150 passes)", 150},
    {"Standard (400 passes)", 400},
    {"Thorough (1200 passes)", 1200},
};
} // namespace

TrainDialog::TrainDialog(SceneGraph* scene, int robot, QWidget* parent)
    : QDialog(parent), m_scene(scene)
{
    setWindowTitle("Train");
    auto* vb = new QVBoxLayout(this);
    vb->setContentsMargins(18, 16, 18, 16);
    vb->setSpacing(10);
    auto* title = new QLabel("Teach a model your motions");
    title->setObjectName("dialogTitle");
    vb->addWidget(title);
    auto* sub = new QLabel("A small neural network learns the chosen motions of one robot: its joints, and how it "
                           "moves over the floor. Each motion becomes a clip that a <b>Play Motion</b> operation plays "
                           "at any length, or blended with another (half a wave, half a bow). Record a motion a few "
                           "times, or several different ones, and train them together.");
    sub->setObjectName("hint");
    sub->setWordWrap(true);
    sub->setMinimumWidth(460);
    vb->addWidget(sub);

    auto* form = new QFormLayout;
    form->setLabelAlignment(Qt::AlignRight);
    m_robot = new QComboBox;
    for (const ApiServer::Robot& r : ApiServer::findRobots(*scene)) {
        int count = 0;
        for (const auto& [id, m] : scene->allMotions()) count += m.robotId == r.root;
        if (count == 0) continue;
        m_robot->addItem(Theme::icon("bot"), QString("%1  ·  %2 motion%3").arg(r.name).arg(count).arg(count == 1 ? "" : "s"),
                         r.root);
        if (r.root == robot) m_robot->setCurrentIndex(m_robot->count() - 1);
    }
    m_motions = new QListWidget;
    m_motions->setMinimumHeight(140);
    m_motions->setToolTip("The motions it learns; each becomes a clip of the model, named after it");
    m_name = new QLineEdit;
    m_effort = new QComboBox;
    for (const Effort& e : kEfforts) m_effort->addItem(e.label, e.epochs);
    QSettings settings("MotionStudio", "MotionStudio");
    m_effort->setCurrentIndex(std::clamp(settings.value("learn/effort", 1).toInt(), 0, 2));
    m_effort->setToolTip("How many times it goes over the motions. More passes follow them more closely and take longer\n"
                         "(seconds for a few short motions)");
    form->addRow("Robot", m_robot);
    form->addRow("Motions", m_motions);
    form->addRow("Name", m_name);
    form->addRow("Training", m_effort);
    vb->addLayout(form);

    m_progress = new QProgressBar;
    m_progress->setRange(0, 1000);
    m_progress->setValue(0);
    m_progress->setTextVisible(false);
    m_progress->hide();
    vb->addWidget(m_progress);
    m_status = new QLabel;
    m_status->setObjectName("hint");
    m_status->setWordWrap(true);
    vb->addWidget(m_status);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Cancel);
    m_train = btns->addButton("Train", QDialogButtonBox::AcceptRole);
    m_train->setIcon(Theme::icon("lightbulb"));
    m_train->setDefault(true);
    vb->addWidget(btns);
    connect(btns, &QDialogButtonBox::rejected, this, &TrainDialog::reject);
    connect(m_train, &QPushButton::clicked, this, &TrainDialog::train);
    connect(m_robot, &QComboBox::currentIndexChanged, this, &TrainDialog::fillMotions);
    connect(&m_watcher, &QFutureWatcher<void>::finished, this, &TrainDialog::finished);
    m_poll.setInterval(100);
    connect(&m_poll, &QTimer::timeout, this, [this] {
        if (!m_job) return;
        const int e = m_job->epoch, total = std::max(1, int(m_job->epochs));
        m_progress->setValue(1000 * e / total);
        const double secs = m_clock.elapsed() / 1000.0;
        const QString left = e > 0 ? QString(", about %1 s left").arg(std::max(0.0, secs * (total - e) / e), 0, 'f', 0) : QString();
        m_status->setText(QString("Pass %1 of %2 — error %3%4").arg(e).arg(total).arg(m_job->loss.load(), 0, 'g', 3).arg(left));
    });
    fillMotions();
    if (m_robot->count() == 0) {
        m_status->setText("No robot has motions yet: record some with Record Values first.");
        m_train->setEnabled(false);
    }
}

TrainDialog::~TrainDialog() {
    if (m_job) m_job->cancel = true;
    m_watcher.waitForFinished();
}

void TrainDialog::fillMotions() {
    m_motions->clear();
    const int root = m_robot->currentData().toInt();
    for (const auto& [id, m] : m_scene->allMotions()) {
        if (m.robotId != root || !m.take) continue;
        auto* it = new QListWidgetItem(Theme::icon("film"), QString("%1  ·  %2 s").arg(m.name).arg(m.take->duration(), 0, 'f', 1));
        it->setData(Qt::UserRole, id);
        it->setFlags(it->flags() | Qt::ItemIsUserCheckable);
        it->setCheckState(Qt::Checked);
        m_motions->addItem(it);
    }
    const SceneObject* o = m_scene->getObject(root);
    m_name->setText(m_scene->makeMotionName(o ? o->name + " moves" : QString("Learned")));
}

void TrainDialog::train() {
    if (m_job) return;
    std::vector<std::pair<QString, take::Take>> takes;
    for (int i = 0; i < m_motions->count(); ++i) {
        const QListWidgetItem* it = m_motions->item(i);
        if (it->checkState() != Qt::Checked) continue;
        if (const MotionClip* m = m_scene->getMotion(it->data(Qt::UserRole).toInt()); m && m->take)
            takes.push_back({m->name, *m->take});
    }
    if (takes.empty()) {
        m_status->setText("Tick the motions to learn.");
        return;
    }
    QSettings("MotionStudio", "MotionStudio").setValue("learn/effort", m_effort->currentIndex());
    learn::AnimationModel::Options opt;
    opt.epochs = m_effort->currentData().toInt();
    m_job = std::make_shared<Job>();
    m_job->epochs = opt.epochs;
    for (QWidget* w : std::initializer_list<QWidget*>{m_robot, m_motions, m_name, m_effort, m_train}) w->setEnabled(false);
    m_progress->show();
    m_clock.start();
    m_poll.start();
    std::shared_ptr<Job> job = m_job;
    // One thread, so a long training keeps the machine cool and the app responsive
    m_watcher.setFuture(QtConcurrent::run([job, takes = std::move(takes), opt] {
        job->ok = learn::AnimationModel::train(takes, opt, &job->model, &job->error, [&](int e, int total, double loss) {
            job->epoch = e;
            job->epochs = total;
            job->loss = loss;
            return !job->cancel;
        });
    }));
}

void TrainDialog::finished() {
    m_poll.stop();
    std::shared_ptr<Job> job = std::move(m_job);
    if (!job) return;
    if (job->cancel) {   // closed while training
        QDialog::reject();
        return;
    }
    if (!job->ok) {
        m_progress->hide();
        m_status->setText("Could not train: " + job->error);
        for (QWidget* w : std::initializer_list<QWidget*>{m_robot, m_motions, m_name, m_effort, m_train}) w->setEnabled(true);
        return;
    }
    LearnedMotion lm;
    lm.name = m_name->text().trimmed();
    lm.robotId = m_robot->currentData().toInt();
    lm.model = std::make_shared<learn::AnimationModel>(std::move(job->model));
    m_learned = m_scene->addLearned(lm);
    accept();
}

void TrainDialog::reject() {
    if (m_job) {   // stop after the pass under way; finished() closes
        m_job->cancel = true;
        m_status->setText("Stopping…");
        return;
    }
    QDialog::reject();
}
