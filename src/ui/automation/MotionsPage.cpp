#include "MotionsPage.h"
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
    m_delete = new QPushButton(Theme::icon("trash-2", Theme::Tint::Danger), "Delete");
    m_delete->setProperty("danger", true);
    bar->addWidget(m_record);
    bar->addWidget(m_import);
    bar->addWidget(m_export);
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
                         "A <b>Play Motion</b> operation then plays it in any sequence.");
    m_empty->setObjectName("hint");
    m_empty->setWordWrap(true);
    lv->addWidget(m_empty);
    split->addWidget(left);

    // ── Right: the selected motion ──────────────────────────────────────────
    auto* box = new QGroupBox("Motion");
    auto* bv = new QVBoxLayout(box);
    bv->setContentsMargins(6, 12, 6, 6);
    auto* form = new QFormLayout;
    form->setLabelAlignment(Qt::AlignRight);
    form->setVerticalSpacing(4);
    m_name = new QLineEdit;
    m_robot = new QComboBox;
    m_robot->setToolTip("The robot that plays it here and in Play Motion operations (unless one chooses another).\n"
                        "Its actuators are matched to the motion's columns by name.");
    form->addRow("Name", m_name);
    form->addRow("Robot", m_robot);
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
    return m_scene->getMotion(m_selected) ? m_selected : -1;
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
    if (row < 0 && m_list->rowCount() > 0) {
        row = m_list->rowCount() - 1;   // the newest
        m_selected = m_list->item(row, C_Name)->data(Qt::UserRole).toInt();
    }
    if (row >= 0) m_list->selectRow(row);
    m_empty->setVisible(m_list->rowCount() == 0);
    loadDetails();
}

void MotionsPage::loadDetails() {
    const MotionClip* m = m_scene->getMotion(currentId());
    m_name->setEnabled(m);
    m_robot->setEnabled(m);
    m_play->setEnabled(m);
    m_export->setEnabled(m);
    m_delete->setEnabled(m);
    const QSignalBlocker b1(m_name), b2(m_robot);
    m_name->setText(m ? m->name : QString());
    m_robot->clear();
    if (!m) {
        m_info->setText(m_scene->allMotions().empty() ? QString() : QString("Select a motion."));
        return;
    }
    for (const ApiServer::Robot& r : ApiServer::findRobots(*m_scene))
        m_robot->addItem(Theme::icon("bot"), r.name, r.root);
    int idx = m_robot->findData(m->robotId);
    if (idx < 0) {
        m_robot->insertItem(0, m->take && !m->take->robot.isEmpty() ? QString("(%1 is gone: choose)").arg(m->take->robot)
                                                                    : QString("(choose)"), m->robotId);
        idx = 0;
    }
    m_robot->setCurrentIndex(idx);
    m_info->setText(describe(*m));
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
    const std::vector<int> mine = AutomationEngine::actuatedJointsBelow(*m_scene, m.robotId);
    QStringList matched, unused;
    for (int jid : mine)
        if (const Motor* mo = m_scene->motorForJoint(jid)) (joints.contains(mo->name) ? matched : unused) << mo->name;
    QStringList missing;
    for (const QString& c : joints)
        if (!matched.contains(c)) missing << c;
    const bool moves = t.column("x") >= 0 && t.column("z") >= 0;
    lines << QString("It moves %1 of %2's %3 actuators%4.")
                 .arg(matched.size()).arg(nameOf(*m_scene, m.robotId)).arg(mine.size())
                 .arg(moves ? " and carries it over the floor" : "");
    if (!unused.isEmpty() && unused.size() <= 12) lines << "Left where they are: " + unused.join(", ") + ".";
    if (!missing.isEmpty())
        lines << QString("<span style='color:%1'>No actuator for: %2</span>")
                     .arg(Theme::css(Theme::colors().warning), missing.mid(0, 12).join(", ") + (missing.size() > 12 ? " …" : ""));
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
    MotionClip* m = m_scene->getMotion(currentId());
    if (!m) return;
    const QString name = m_name->text().trimmed();
    if (name.isEmpty() || name == m->name) { m_name->setText(m->name); return; }
    edit([&] {
        m->name = m_scene->makeMotionName(name, m->id);
        m_scene->notifyAutomationChanged();
    });
    m_name->setText(m->name);
    if (QTableWidgetItem* it = m_list->currentItem() ? m_list->item(m_list->currentRow(), C_Name) : nullptr)
        it->setText(m->name);
}

void MotionsPage::setRobot() {
    MotionClip* m = m_scene->getMotion(currentId());
    const int root = m_robot->currentData().toInt();
    if (!m || root == m->robotId || !m_scene->getObject(root)) return;
    edit([&] {
        m->robotId = root;
        m_scene->notifyAutomationChanged();
    });
    refresh();
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
    const MotionClip* m = m_scene->getMotion(currentId());
    if (!m) return;
    int uses = 0;
    for (const auto& [sid, s] : m_scene->allSequences())
        for (const Operation& op : s.operations) uses += op.type == OperationType::PlayMotion && op.motionId == m->id;
    if (uses && QMessageBox::question(this, "Delete motion",
                                      QString("%1 is played by %2 operation%3. Delete it anyway?")
                                          .arg(m->name).arg(uses).arg(uses == 1 ? "" : "s")) != QMessageBox::Yes)
        return;
    const int id = m->id;
    edit([&] { m_scene->removeMotion(id); });
    m_selected = -1;
    refresh();
}

void MotionsPage::play() {
    const MotionClip* m = m_scene->getMotion(currentId());
    if (!m || !m->take) return;
    if (!m_scene->getObject(m->robotId)) {
        emit statusMessage("Choose the robot that plays " + m->name, false);
        return;
    }
    QString err;
    std::shared_ptr<AutomationEngine::PlayRequest> req = AutomationEngine::playRequestFor(
        *m_scene, m->robotId, AutomationEngine::actuatedJointsBelow(*m_scene, m->robotId), *m->take, true, &err);
    if (!req) {
        emit statusMessage(m->name + ": " + err, false);
        return;
    }
    req->relative = true;
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
    AutomationEngine* a = m_sim->automation();
    if (m_playing && a->hasMotion(m_playing)) a->cancelMotion(m_playing, "another motion was played");
    AutomationEngine::MotionRequest r;
    r.kind = AutomationEngine::MotionKind::Play;
    r.play = req;
    r.label = "play " + m->name;
    QString code;
    m_playing = a->submitMotion(r, &code, &err);
    if (!m_playing) emit statusMessage(m->name + ": " + err, false);
    else emit statusMessage(QString("Playing %1 on %2 — Reset puts the robot back").arg(m->name, nameOf(*m_scene, m->robotId)), true);
}
