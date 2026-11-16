#pragma once
#include <QDialog>
#include <QElapsedTimer>
#include <QFutureWatcher>
#include <QTimer>
#include <atomic>
#include <memory>
#include "automation/Learn.h"

class SceneGraph;
class QComboBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QProgressBar;
class QPushButton;

// Motions tab › Train: teaches a model the motions of one robot, on a
// worker thread with progress, and adds it to the project as a
// LearnedMotion whose clips Play Motion operations play.
class TrainDialog : public QDialog {
    Q_OBJECT
public:
    // `robot` is preselected (-1: the first robot with motions)
    TrainDialog(SceneGraph* scene, int robot, QWidget* parent = nullptr);
    ~TrainDialog() override;

    int learnedId() const { return m_learned; }   // the model added, or -1

private:
    void fillMotions();
    void train();
    void finished();
    void reject() override;

    SceneGraph*   m_scene;
    QComboBox*    m_robot = nullptr;
    QListWidget*  m_motions = nullptr;
    QLineEdit*    m_name = nullptr;
    QComboBox*    m_effort = nullptr;
    QProgressBar* m_progress = nullptr;
    QLabel*       m_status = nullptr;
    QPushButton*  m_train = nullptr;
    int           m_learned = -1;

    struct Job {
        std::atomic<int>    epoch{0};
        std::atomic<int>    epochs{1};
        std::atomic<double> loss{0};
        std::atomic<bool>   cancel{false};
        learn::AnimationModel model;
        QString error;
        bool    ok = false;
    };
    std::shared_ptr<Job>  m_job;
    QFutureWatcher<void>  m_watcher;
    QTimer                m_poll;
    QElapsedTimer         m_clock;
};
