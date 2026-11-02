#pragma once
#include <QDialog>
#include <QElapsedTimer>
#include <QTimer>
#include <memory>
#include "../import/StepImporter.h"

class QLabel;
class QProgressBar;
class QPushButton;

// The window shown while a STEP file imports on a worker thread: which of the
// three stages it's in (read, build, mesh), the percentage and time left
// above the bar, the part being meshed, elapsed time and Cancel.
//
// `callback()` is handed to StepImporter::importFile on the worker; it only
// stores the latest progress and reads the cancel flag, and the dialog picks
// both up on its own timer.
class ImportProgressDialog : public QDialog {
    Q_OBJECT
public:
    ImportProgressDialog(const QString& path, QWidget* parent);
    ~ImportProgressDialog() override;

    StepImporter::ProgressFn callback() const;
    bool cancelRequested() const;
    // The import ended; on success remembers how long the stages took, to
    // estimate the next import better
    void finish(bool ok);
    double elapsedSeconds() const { return m_clock.elapsed() / 1000.0; }

    // "About 40 s left", "About 2 min left", "A few seconds left"
    static QString formatLeft(double seconds);

protected:
    void reject() override;   // Esc / close button = Cancel

private:
    struct Shared;
    void tick();
    void requestCancel();

    std::shared_ptr<Shared> m_shared;
    StepImporter::Estimator m_estimator;
    QElapsedTimer m_clock;
    QTimer        m_timer;
    double        m_shownLeft = -1.0;

    QLabel*       m_steps[3] = {};
    QLabel*       m_stage = nullptr;
    QLabel*       m_percent = nullptr;
    QLabel*       m_left = nullptr;
    QProgressBar* m_bar = nullptr;
    QLabel*       m_detail = nullptr;
    QLabel*       m_elapsed = nullptr;
    QPushButton*  m_cancel = nullptr;
};
