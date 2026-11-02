#include "ImportProgressDialog.h"
#include "Theme.h"
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QLocale>
#include <QMutex>
#include <QProgressBar>
#include <QPushButton>
#include <QSettings>
#include <QVBoxLayout>
#include <atomic>
#include <cmath>

using StepImporter::Progress;

struct ImportProgressDialog::Shared {
    QMutex            mutex;
    Progress          progress;
    std::atomic<bool> cancel{false};
};

namespace {

StepImporter::Estimator::Rates loadRates() {
    QSettings s("MotionStudio", "MotionStudio");
    StepImporter::Estimator::Rates r;
    r.readSecPerMB   = s.value("import/step/readSecPerMB",   r.readSecPerMB).toDouble();
    r.translateRatio = s.value("import/step/translateRatio", r.translateRatio).toDouble();
    r.meshRatio      = s.value("import/step/meshRatio",      r.meshRatio).toDouble();
    return r;
}

QString formatElapsed(double seconds) {
    const int s = int(seconds);
    return QString("%1:%2 elapsed").arg(s / 60).arg(s % 60, 2, 10, QChar('0'));
}

} // namespace

ImportProgressDialog::ImportProgressDialog(const QString& path, QWidget* parent)
    : QDialog(parent), m_shared(std::make_shared<Shared>()),
      m_estimator(QFileInfo(path).size(), loadRates())
{
    const QFileInfo info(path);
    setWindowTitle("STEP Import");
    setWindowFlags(Qt::Dialog | Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowCloseButtonHint);
    setModal(true);

    auto* vb = new QVBoxLayout(this);
    vb->setContentsMargins(20, 18, 20, 16);
    vb->setSpacing(6);

    auto* title = new QLabel("Importing " + info.fileName());
    title->setObjectName("dialogTitle");
    title->setTextInteractionFlags(Qt::NoTextInteraction);
    vb->addWidget(title);
    auto* size = new QLabel(QLocale().formattedDataSize(info.size(), 1, QLocale::DataSizeTraditionalFormat));
    size->setObjectName("hint");
    vb->addWidget(size);
    vb->addSpacing(8);

    // 1 Read file · 2 Build shapes · 3 Mesh parts
    auto* steps = new QHBoxLayout;
    steps->setSpacing(14);
    for (int i = 0; i < 3; i++) {
        m_steps[i] = new QLabel;
        m_steps[i]->setObjectName("importStep");
        steps->addWidget(m_steps[i]);
    }
    steps->addStretch();
    vb->addLayout(steps);
    vb->addSpacing(6);

    m_stage = new QLabel;
    vb->addWidget(m_stage);

    // Percentage and time left, above the bar
    auto* above = new QHBoxLayout;
    m_percent = new QLabel("0%");
    m_percent->setObjectName("importPercent");
    m_left = new QLabel("Estimating…");
    m_left->setObjectName("fieldLabel");
    above->addWidget(m_percent, 0, Qt::AlignBottom);
    above->addStretch();
    above->addWidget(m_left, 0, Qt::AlignBottom);
    vb->addLayout(above);

    m_bar = new QProgressBar;
    m_bar->setRange(0, 1000);
    m_bar->setTextVisible(false);
    vb->addWidget(m_bar);

    m_detail = new QLabel;
    m_detail->setObjectName("hint");
    m_detail->setWordWrap(true);
    m_detail->setMinimumHeight(m_detail->fontMetrics().height() * 2);
    m_detail->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    vb->addWidget(m_detail);

    auto* bottom = new QHBoxLayout;
    m_elapsed = new QLabel(formatElapsed(0));
    m_elapsed->setObjectName("hint");
    m_cancel = new QPushButton("Cancel");
    m_cancel->setToolTip("Stop the import (Esc). Reading the file can't be interrupted, "
                         "so a cancel then takes effect once the file is read.");
    connect(m_cancel, &QPushButton::clicked, this, &ImportProgressDialog::requestCancel);
    bottom->addWidget(m_elapsed);
    bottom->addStretch();
    bottom->addWidget(m_cancel);
    vb->addSpacing(4);
    vb->addLayout(bottom);

    setFixedWidth(440);
    setFixedHeight(sizeHint().height());

    m_clock.start();
    m_timer.setInterval(100);
    connect(&m_timer, &QTimer::timeout, this, &ImportProgressDialog::tick);
    m_timer.start();
    tick();
}

ImportProgressDialog::~ImportProgressDialog() = default;

StepImporter::ProgressFn ImportProgressDialog::callback() const {
    // Holds the shared state, not the dialog, so a late call is harmless
    std::shared_ptr<Shared> shared = m_shared;
    return [shared](const Progress& p) {
        QMutexLocker lock(&shared->mutex);
        shared->progress = p;
        return !shared->cancel.load();
    };
}

bool ImportProgressDialog::cancelRequested() const { return m_shared->cancel.load(); }

void ImportProgressDialog::requestCancel() {
    if (m_shared->cancel.exchange(true)) return;
    m_cancel->setEnabled(false);
    m_cancel->setText("Cancelling…");
    tick();
}

void ImportProgressDialog::reject() { requestCancel(); }   // closes when the worker stops

void ImportProgressDialog::finish(bool ok) {
    m_timer.stop();
    if (ok) {
        const auto r = m_estimator.learn(m_clock.elapsed() / 1000.0);
        QSettings s("MotionStudio", "MotionStudio");
        s.setValue("import/step/readSecPerMB",   r.readSecPerMB);
        s.setValue("import/step/translateRatio", r.translateRatio);
        s.setValue("import/step/meshRatio",      r.meshRatio);
    }
}

QString ImportProgressDialog::formatLeft(double seconds) {
    if (seconds < 5) return "A few seconds left";
    if (seconds < 20) return QString("About %1 s left").arg(int(std::ceil(seconds)));
    if (seconds < 60) return QString("About %1 s left").arg(int(std::round(seconds / 5)) * 5);
    const int minutes = int(std::round(seconds / 60));
    return QString("About %1 min left").arg(std::max(1, minutes));
}

void ImportProgressDialog::tick() {
    Progress p;
    {
        QMutexLocker lock(&m_shared->mutex);
        p = m_shared->progress;
    }
    const double now = m_clock.elapsed() / 1000.0;
    m_estimator.update(p, now);

    const int stage = int(p.stage);
    static const char* names[3] = {"Read file", "Build shapes", "Mesh parts"};
    for (int i = 0; i < 3; i++) {
        m_steps[i]->setText((i < stage ? QString("✓  ") : QString("%1  ").arg(i + 1)) + names[i]);
        m_steps[i]->setProperty("active", i == stage);
        m_steps[i]->setProperty("done", i < stage);
        Theme::repolish(m_steps[i]);
    }

    const bool cancelling = m_shared->cancel.load();
    QString stageText, detail;
    switch (p.stage) {
    case Progress::Stage::Reading:
        stageText = "Reading the file…";
        detail = cancelling ? "Stopping once the file is read (reading can't be interrupted)."
                            : "Parsing the STEP data.";
        break;
    case Progress::Stage::Translating:
        stageText = "Building shapes…";
        detail = "Turning the file's assembly, parts and faces into solid shapes.";
        break;
    case Progress::Stage::Meshing:
        stageText = p.total > 0 ? QString("Meshing parts · %1 of %2 done").arg(p.done).arg(p.total)
                                : QString("Meshing parts…");
        if (!p.part.isEmpty() || p.partFaces > 0)
            detail = QString("Meshing %1 (%2 faces)%3")
                         .arg(p.part.isEmpty() ? QString("a part") : "“" + p.part + "”",
                              QLocale().toString(p.partFaces),
                              m_estimator.confident() || now < 1.0 ? QString()
                                  : QString(" — a large part, this one takes a while"));
        break;
    }
    if (cancelling && p.stage != Progress::Stage::Reading) detail = "Stopping…";
    m_stage->setText(stageText);
    m_detail->setText(detail);

    const double f = m_estimator.fraction();
    m_bar->setValue(int(f * 1000));
    m_percent->setText(QString("%1%").arg(int(f * 100)));

    // Smooth the estimate so it counts down steadily instead of jittering
    if (m_estimator.confident()) {
        const double target = m_estimator.secondsLeft();
        m_shownLeft = m_shownLeft < 0 ? target : m_shownLeft + 0.25 * (target - m_shownLeft);
        m_left->setText(formatLeft(m_shownLeft));
    } else {
        m_shownLeft = -1.0;
        m_left->setText(now < 1.0 ? "Estimating…" : "Working…");
    }
    if (cancelling) m_left->setText({});
    m_elapsed->setText(formatElapsed(now));
}
