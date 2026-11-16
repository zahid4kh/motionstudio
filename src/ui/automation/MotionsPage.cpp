#include "MotionsPage.h"
#include "TrainDialog.h"
#include "io/ApiServer.h"
#include "physics/SimulationEngine.h"
#include "ui/Theme.h"
#include <QAction>
#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSettings>
#include <QSplitter>
#include <QTableWidget>
#include <QToolButton>
#include <QVBoxLayout>

namespace {
enum Col { C_Name, C_Robot, C_Length, C_Joints, C_Count };

QString nameOf(const SceneGraph& scene, int id) {
    const SceneObject* o = scene.getObject(id);
    return o ? o->name : QString("(gone)");
}
} // namespace

MotionsPage::MotionsPage(SceneGraph* scene, SimulationEngine* sim, QWidget* parent)
    : Page(scene, sim, parent)
{
    auto* root = new QHBoxLayout(this);
    root->setContentsMargins(6, 6, 6, 6);
    auto* split = new QSplitter(Qt::Horizontal);
    root->addWidget(split);

    // ── Left: the motions ───────────────────────────────────────────────────
    auto* left = new QWidget;
    auto* lv = new QVBoxLayout(left);
    lv->setContentsMargins(0, 0, 0, 0);
    lv->setSpacing(4);
    auto* bar = new QHBoxLayout;
    m_record = new QToolButton;
    m_record->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    m_record->hide();
    m_import = new QPushButton(Theme::icon("file-input"), "Import CSV…");
    m_import->setToolTip("Add a take saved as CSV (by Record Values, robot.record() or your own program)");
    m_export = new QPushButton(Theme::icon("download"), "Export CSV…");
    m_export->setToolTip("Save the motion as a CSV take, for analysis or your own learning code");
    m_train = new QPushButton(Theme::icon("lightbulb"), "Train…");
    m_train->setToolTip("Teach a model a robot's motions: it then plays each at any length, or blends two");
    m_delete = new QPushButton(Theme::icon("trash-2", Theme::Tint::Danger), "Delete");
    m_delete->setProperty("danger", true);
    bar->addWidget(m_record);
    bar->addWidget(m_import);
    bar->addWidget(m_export);
    bar->addWidget(m_train);
    bar->addStretch();
    bar->addWidget(m_delete);
    lv->addLayout(bar);
    m_list = new QTableWidget(0, C_Count);
    m_list->setHorizontalHeaderLabels({"Motion", "Robot", "Length", "Joints"});
    m_list->verticalHeader()->hide();
    m_list->setAlternatingRowColors(true);
    m_list->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_list->setSelectionMode(QAbstractItemView::SingleSelection);
    m_list->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_list->verticalHeader()->setDefaultSectionSize(22);
    m_list->setColumnWidth(C_Name, 170);
    m_list->setColumnWidth(C_Robot, 130);
    m_list->setColumnWidth(C_Length, 70);
    m_list->horizontalHeader()->setSectionResizeMode(C_Joints, QHeaderView::Stretch);
    lv->addWidget(m_list, 1);
    m_empty = new QLabel("No motions yet. Press <b>Record</b>, pick a robot and make it move (with a sequence, "
                         "the Actuators &amp; Poses jog, or a program); when you stop, the motion is kept here. "
                         "A <b>Play Motion</b> operation then plays it in any sequence, and <b>Train</b> teaches a "
                         "model several of them, to play at any length or blended.");
    m_empty->setObjectName("hint");
    m_empty->setWordWrap(true);
    lv->addWidget(m_empty);
    split->addWidget(left);

    // ── Right: the selected motion ──────────────────────────────────────────
    auto* box = new QGroupBox("Motion");
    auto* bv = new QVBoxLayout(box);
    bv->setContentsMargins(6, 12, 6, 6);
    auto* form = new QFormLayout;
    m_form = form;
    form->setLabelAlignment(Qt::AlignRight);
    form->setVerticalSpacing(4);
    m_name = new QLineEdit;
    m_robot = new QComboBox;
    m_robot->setToolTip("The robot that plays it here and in Play Motion operations (unless one chooses another).\n"
                        "Its actuators are matched to the motion's columns by name.");
    form->addRow("Name", m_name);
    form->addRow("Robot", m_robot);
    m_clip = new QComboBox;
    m_clip->setToolTip("The clip to play or export: one of the motions the model learned");
    form->addRow("Clip", m_clip);
    bv->addLayout(form);
    m_play = new QPushButton(Theme::icon("play"), "Play on the robot");
    m_play->setToolTip("Starts the simulation and plays the motion from where the robot stands.\n"
                       "Reset puts everything back.");
    auto* pr = new QHBoxLayout;
    pr->addWidget(m_play);
    pr->addStretch();
    bv->addLayout(pr);
    m_info = new QLabel;
    m_info->setObjectName("hint");
    m_info->setWordWrap(true);
    m_info->setTextFormat(Qt::RichText);
    bv->addWidget(m_info);
    bv->addStretch();
    split->addWidget(box);
    split->setStretchFactor(0, 3);
    split->setStretchFactor(1, 2);

    connect(m_list, &QTableWidget::currentCellChanged, this, [this](int row) {
        if (row < 0 || !m_list->item(row, C_Name)) return;
        m_selected = m_list->item(row, C_Name)->data(Qt::UserRole).toInt();
        loadDetails();
    });
    connect(m_list, &QTableWidget::cellDoubleClicked, this, [this] { play(); });
    connect(m_name, &QLineEdit::editingFinished, this, &MotionsPage::rename);
    connect(m_robot, &QComboBox::currentIndexChanged, this, &MotionsPage::setRobot);
    connect(m_import, &QPushButton::clicked, this, &MotionsPage::importCsv);
    connect(m_export, &QPushButton::clicked, this, &MotionsPage::exportCsv);
    connect(m_delete, &QPushButton::clicked, this, &MotionsPage::remove);
    connect(m_train, &QPushButton::clicked, this, &MotionsPage::train);
    connect(m_play, &QPushButton::clicked, this, &MotionsPage::play);
    refresh();
}

void MotionsPage::setRecordAction(QAction* record) {
    m_record->setDefaultAction(record);
    m_record->show();
}

void MotionsPage::setLocked(bool locked) {
    // Motions don't change what a run compiled, so they stay editable
    Page::setLocked(locked);
}

int MotionsPage::currentId() const {
    return m_scene->getMotion(m_selected) || m_scene->getLearned(m_selected) ? m_selected : -1;
}

QString* MotionsPage::nameRef(int id) {
    if (MotionClip* m = m_scene->getMotion(id)) return &m->name;
    if (LearnedMotion* m = m_scene->getLearned(id)) return &m->name;
    return nullptr;
}

int* MotionsPage::robotRef(int id) {
    if (MotionClip* m = m_scene->getMotion(id)) return &m->robotId;
    if (LearnedMotion* m = m_scene->getLearned(id)) return &m->robotId;
    return nullptr;
}

Operation MotionsPage::previewOperation() const {
    Operation op;
    op.type = OperationType::PlayMotion;
    op.motionId = currentId();
    op.relative = true;
    if (m_scene->getLearned(op.motionId)) op.clip = m_clip->currentText();
    return op;
}

void MotionsPage::selectMotion(int id) {
    m_selected = id;
    refresh();
}

void MotionsPage::refresh() {
    const QSignalBlocker block(m_list);
    m_list->setRowCount(0);
    int row = -1;
    for (const auto& [id, m] : m_scene->allMotions()) {
        const int i = m_list->rowCount();
        m_list->insertRow(i);
        auto* name = new QTableWidgetItem(Theme::icon("film"), m.name);
        name->setData(Qt::UserRole, id);
        m_list->setItem(i, C_Name, name);
        m_list->setItem(i, C_Robot, new QTableWidgetItem(nameOf(*m_scene, m.robotId)));
        m_list->setItem(i, C_Length, new QTableWidgetItem(m.take ? QString("%1 s").arg(m.take->duration(), 0, 'f', 1) : QString()));
        m_list->setItem(i, C_Joints, new QTableWidgetItem(m.take ? QString::number(m.take->jointColumns().size()) : QString()));
        if (id == m_selected) row = i;
    }
    for (const auto& [id, m] : m_scene->allLearned()) {
        const int i = m_list->rowCount();
        m_list->insertRow(i);
        auto* name = new QTableWidgetItem(Theme::icon("lightbulb"), m.name);
        name->setData(Qt::UserRole, id);
        name->setToolTip("A model trained on motions");
        m_list->setItem(i, C_Name, name);
        m_list->setItem(i, C_Robot, new QTableWidgetItem(nameOf(*m_scene, m.robotId)));
        const int clips = m.model ? int(m.model->clips.size()) : 0;
        m_list->setItem(i, C_Length, new QTableWidgetItem(QString("%1 clip%2").arg(clips).arg(clips == 1 ? "" : "s")));
        m_list->setItem(i, C_Joints, new QTableWidgetItem(m.model ? QString::number(m.model->joints.size()) : QString()));
        if (id == m_selected) row = i;
    }
    if (row < 0 && m_list->rowCount() > 0) {
        row = m_list->rowCount() - 1;   // the newest
        m_selected = m_list->item(row, C_Name)->data(Qt::UserRole).toInt();
    }
    if (row >= 0) m_list->selectRow(row);
    m_empty->setVisible(m_list->rowCount() == 0);
    m_train->setEnabled(!m_scene->allMotions().empty());
    loadDetails();
}

void MotionsPage::loadDetails() {
    const int id = currentId();
    const MotionClip* m = m_scene->getMotion(id);
    const LearnedMotion* lm = m_scene->getLearned(id);
    const bool any = m || lm;
    m_name->setEnabled(any);
    m_robot->setEnabled(any);
    m_play->setEnabled(any);
    m_export->setEnabled(any);
    m_delete->setEnabled(any);
    m_form->setRowVisible(m_clip, lm);
    m_export->setToolTip(lm ? "Save the chosen clip, as the model plays it, as a CSV take"
                            : "Save the motion as a CSV take, for analysis or your own learning code");
    const QSignalBlocker b1(m_name), b2(m_robot), b3(m_clip);
    m_name->setText(any ? *const_cast<MotionsPage*>(this)->nameRef(id) : QString());
    m_robot->clear();
    if (!any) {
        m_info->setText(m_scene->allMotions().empty() && m_scene->allLearned().empty() ? QString() : QString("Select a motion."));
        return;
    }
    const int robot = m ? m->robotId : lm->robotId;
    for (const ApiServer::Robot& r : ApiServer::findRobots(*m_scene))
        m_robot->addItem(Theme::icon("bot"), r.name, r.root);
    int idx = m_robot->findData(robot);
    if (idx < 0) {
        const QString was = m && m->take ? m->take->robot : lm && lm->model ? lm->model->robot : QString();
        m_robot->insertItem(0, !was.isEmpty() ? QString("(%1 is gone: choose)").arg(was) : QString("(choose)"), robot);
        idx = 0;
    }
    m_robot->setCurrentIndex(idx);
    if (lm) {
        const QString was = m_clip->currentText();
        m_clip->clear();
        if (lm->model) m_clip->addItems(lm->model->clips);
        m_clip->setCurrentIndex(std::max(0, m_clip->findText(was)));
    }
    m_info->setText(m ? describe(*m) : describe(*lm));
}

QString MotionsPage::fit(int robot, const QStringList& joints, const QString& verb, const QString& more) const {
    const std::vector<int> mine = AutomationEngine::actuatedJointsBelow(*m_scene, robot);
    QStringList matched, unused;
    for (int jid : mine)
        if (const Motor* mo = m_scene->motorForJoint(jid)) (joints.contains(mo->name) ? matched : unused) << mo->name;
    QStringList missing;
    for (const QString& c : joints)
        if (!matched.contains(c)) missing << c;
    QStringList lines;
    lines << QString("%1 %2 of %3's %4 actuators%5.").arg(verb).arg(matched.size()).arg(nameOf(*m_scene, robot)).arg(mine.size()).arg(more);
    if (!unused.isEmpty() && unused.size() <= 12) lines << "Left where they are: " + unused.join(", ") + ".";
    if (!missing.isEmpty())
        lines << QString("<span style='color:%1'>No actuator for: %2</span>")
                     .arg(Theme::css(Theme::colors().warning), missing.mid(0, 12).join(", ") + (missing.size() > 12 ? " …" : ""));
    return lines.join("<br>");
}

QString MotionsPage::describe(const LearnedMotion& m) const {
    if (!m.model) return {};
    const learn::AnimationModel& a = *m.model;
    QStringList clips;
    for (int i = 0; i < a.clips.size(); ++i) clips << QString("%1 (%2 s)").arg(a.clips[i]).arg(a.durations[size_t(i)], 0, 'f', 1);
    QStringList lines;
    lines << QString("A model trained on %1 motion%2: %3.").arg(a.clips.size()).arg(a.clips.size() == 1 ? "" : "s").arg(clips.join(", "));
    lines << QString("It learned from %1 samples; error after training %2 (0 = exact).").arg(a.samples).arg(a.net.loss(), 0, 'g', 2);
    if (m_scene->getObject(m.robotId)) {
        lines << fit(m.robotId, a.joints, "Its clips move", a.place ? " and carry it over the floor" : "");
    }
    lines << "A <b>Play Motion</b> operation plays a clip from where the robot stands, at any length, or blends two.";
    return lines.join("<br>");
}

QString MotionsPage::describe(const MotionClip& m) const {
    if (!m.take) return {};
    const take::Take& t = *m.take;
    QStringList lines;
    lines << QString("%1 samples over %2 s (%3 per second).")
                 .arg(t.rows.size()).arg(t.duration(), 0, 'f', 2)
                 .arg(t.rate > 0 ? QString::number(t.rate, 'g', 4) : QString("?"));
    const QStringList joints = t.jointColumns();
    if (!m_scene->getObject(m.robotId)) {
        lines << QString("Its %1 joint columns: %2.").arg(joints.size()).arg(joints.join(", "));
        return lines.join("<br>");
    }
    const bool moves = t.column("x") >= 0 && t.column("z") >= 0;
    lines << fit(m.robotId, joints, "It moves", moves ? " and carries it over the floor" : "");
    if (t.column("walking") >= 0) {
        int walking = 0;
        for (const auto& row : t.rows) walking += row[size_t(t.column("walking"))] > 0;
        if (walking > 0) lines << QString("It walks for %1 s of it.").arg(walking * t.duration() / std::max<size_t>(1, t.rows.size() - 1), 0, 'f', 1);
    }
    lines << "Use it in a sequence with a <b>Play Motion</b> operation.";
    return lines.join("<br>");
}

void MotionsPage::refreshRuntime() {
    // A preview that ended frees the Play button for the next
    if (m_playing && !m_sim->automation()->hasMotion(m_playing)) m_playing = 0;
}

void MotionsPage::rename() {
    const int id = currentId();
    QString* current = nameRef(id);
    if (!current) return;
    const QString name = m_name->text().trimmed();
    if (name.isEmpty() || name == *current) { m_name->setText(*current); return; }
    edit([&] {
        *current = m_scene->makeMotionName(name, id);
        m_scene->notifyAutomationChanged();
    });
    m_name->setText(*current);
    if (m_list->currentRow() >= 0)
        if (QTableWidgetItem* it = m_list->item(m_list->currentRow(), C_Name)) it->setText(*current);
}

void MotionsPage::setRobot() {
    int* robot = robotRef(currentId());
    const int root = m_robot->currentData().toInt();
    if (!robot || root == *robot || !m_scene->getObject(root)) return;
    edit([&] {
        *robot = root;
        m_scene->notifyAutomationChanged();
    });
    refresh();
}

void MotionsPage::train() {
    const int* robot = robotRef(currentId());
    TrainDialog dlg(m_scene, robot ? *robot : -1, this);
    m_selfEdit = true;   // the new model is selected below
    const bool ok = dlg.exec() == QDialog::Accepted;
    m_selfEdit = false;
    if (!ok || dlg.learnedId() == -1) return;
    selectMotion(dlg.learnedId());
    const LearnedMotion* lm = m_scene->getLearned(dlg.learnedId());
    if (lm && lm->model)
        emit statusMessage(QString("Trained “%1” on %2 motion%3 — play its clips with a Play Motion operation")
                               .arg(lm->name).arg(lm->model->clips.size()).arg(lm->model->clips.size() == 1 ? "" : "s"), true);
}

void MotionsPage::importCsv() {
    QSettings settings("MotionStudio", "MotionStudio");
    const QStringList files = QFileDialog::getOpenFileNames(this, "Import motions", settings.value("take/dir").toString(),
                                                            "CSV take (*.csv);;All files (*)");
    if (files.isEmpty()) return;
    settings.setValue("take/dir", QFileInfo(files.front()).absolutePath());
    const std::vector<ApiServer::Robot> robots = ApiServer::findRobots(*m_scene);
    int last = -1;
    QStringList problems;
    for (const QString& f : files) {
        auto t = std::make_shared<take::Take>();
        QString err;
        if (!take::Take::load(f, t.get(), &err)) { problems << QFileInfo(f).fileName() + ": " + err; continue; }
        // The robot it names, else the one whose actuators fit it best
        int robot = -1, best = 0;
        const QStringList cols = t->jointColumns();
        for (const ApiServer::Robot& r : robots) {
            if (r.name == t->robot) { robot = r.root; break; }
            int fit = 0;
            for (int jid : r.joints)
                if (const Motor* mo = m_scene->motorForJoint(jid); mo && cols.contains(mo->name)) ++fit;
            if (fit > best) { best = fit; robot = r.root; }
        }
        MotionClip m;
        m.name = QFileInfo(f).completeBaseName();
        m.robotId = robot;
        m.take = std::move(t);
        edit([&] { last = m_scene->addMotion(m); });
    }
    if (last != -1) selectMotion(last);
    if (!problems.isEmpty())
        QMessageBox::warning(this, "Import motions", "Some files could not be read:\n\n" + problems.join("\n"));
    else
        emit statusMessage(QString("Imported %1 motion%2").arg(files.size()).arg(files.size() == 1 ? "" : "s"), true);
}

void MotionsPage::exportCsv() {
    const MotionClip* m = m_scene->getMotion(currentId());
    const LearnedMotion* lm = m_scene->getLearned(currentId());
    if (lm && lm->model) {
        const learn::AnimationModel& a = *lm->model;
        const std::vector<double> w = a.weights(m_clip->currentText());
        if (w.empty()) return;
        QSettings settings("MotionStudio", "MotionStudio");
        const QString suggested = QDir(settings.value("take/dir", QDir::homePath()).toString())
                                      .filePath(QString("%1-%2.csv").arg(lm->name, m_clip->currentText()).replace(' ', '-').toLower());
        QString path = QFileDialog::getSaveFileName(this, "Export clip", suggested, "CSV take (*.csv)");
        if (path.isEmpty()) return;
        if (QFileInfo(path).suffix().isEmpty()) path += ".csv";
        settings.setValue("take/dir", QFileInfo(path).absolutePath());
        take::Take t = a.generate(w, 0, 50);
        t.robot = nameOf(*m_scene, lm->robotId);
        QString err;
        if (!t.save(path, &err)) QMessageBox::warning(this, "Export clip", QString("Could not save %1:\n\n%2").arg(path, err));
        else emit statusMessage(QString("Saved the clip %1 to %2").arg(m_clip->currentText(), QDir::toNativeSeparators(path)), true);
        return;
    }
    if (!m || !m->take) return;
    QSettings settings("MotionStudio", "MotionStudio");
    const QString suggested = QDir(settings.value("take/dir", QDir::homePath()).toString())
                                  .filePath(QString(m->name).replace(' ', '-').toLower() + ".csv");
    QString path = QFileDialog::getSaveFileName(this, "Export motion", suggested, "CSV take (*.csv)");
    if (path.isEmpty()) return;
    if (QFileInfo(path).suffix().isEmpty()) path += ".csv";
    settings.setValue("take/dir", QFileInfo(path).absolutePath());
    take::Take t = *m->take;
    t.robot = nameOf(*m_scene, m->robotId);
    QString err;
    if (!t.save(path, &err)) QMessageBox::warning(this, "Export motion", QString("Could not save %1:\n\n%2").arg(path, err));
    else emit statusMessage(QString("Saved %1 to %2").arg(m->name, QDir::toNativeSeparators(path)), true);
}

void MotionsPage::remove() {
    const int id = currentId();
    const QString* name = nameRef(id);
    if (!name) return;
    int uses = 0;
    for (const auto& [sid, s] : m_scene->allSequences())
        for (const Operation& op : s.operations) uses += op.type == OperationType::PlayMotion && op.motionId == id;
    if (uses && QMessageBox::question(this, "Delete motion",
                                      QString("%1 is played by %2 operation%3. Delete it anyway?")
                                          .arg(*name).arg(uses).arg(uses == 1 ? "" : "s")) != QMessageBox::Yes)
        return;
    edit([&] {
        if (m_scene->getMotion(id)) m_scene->removeMotion(id);
        else m_scene->removeLearned(id);
    });
    m_selected = -1;
    refresh();
}

void MotionsPage::play() {
    const int id = currentId();
    const QString* name = nameRef(id);
    if (!name) return;
    const Operation op = previewOperation();
    const int root = AutomationEngine::playMotionRobot(*m_scene, op);
    if (!m_scene->getObject(root)) {
        emit statusMessage("Choose the robot that plays " + *name, false);
        return;
    }
    QString err;
    if (!AutomationEngine::playMotionRequest(*m_scene, op, root, 0, 0, &err)) {   // checked here; made when it starts
        emit statusMessage(*name + ": " + err, false);
        return;
    }
    if (!m_sim->isRunning()) {
        if (!m_sim->isActive()) {
            const QStringList errs = AutomationEngine::validateAll(*m_scene);
            if (!errs.isEmpty()) {
                emit statusMessage("The simulation can't start: " + errs.front(), false);
                return;
            }
        }
        m_sim->play();
        if (!m_sim->isRunning()) return;
    }
    // From where the robot stands now that the simulation runs
    std::shared_ptr<AutomationEngine::PlayRequest> req = AutomationEngine::playMotionRequest(*m_scene, op, root, 0, 0, &err);
    if (!req) return;
    AutomationEngine* a = m_sim->automation();
    if (m_playing && a->hasMotion(m_playing)) a->cancelMotion(m_playing, "another motion was played");
    AutomationEngine::MotionRequest r;
    r.kind = AutomationEngine::MotionKind::Play;
    r.play = req;
    r.label = "play " + *name;
    QString code;
    m_playing = a->submitMotion(r, &code, &err);
    const QString what = op.clip.isEmpty() ? *name : QString("%1 (%2)").arg(*name, op.clip);
    if (!m_playing) emit statusMessage(what + ": " + err, false);
    else emit statusMessage(QString("Playing %1 on %2 — Reset puts the robot back").arg(what, nameOf(*m_scene, root)), true);
}
