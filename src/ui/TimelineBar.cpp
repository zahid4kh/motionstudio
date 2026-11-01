#include "TimelineBar.h"
#include "Theme.h"
#include "physics/SimulationEngine.h"
#include "viewport/Viewport.h"
#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QProcess>
#include <QProgressDialog>
#include <QPushButton>
#include <QSlider>
#include <QMouseEvent>
#include <QStandardPaths>
#include <QStyle>
#include <QStyleOptionSlider>
#include <QVBoxLayout>
#include <cmath>

namespace {
// A click on the groove jumps there (a timeline, not a page-stepping slider)
class ScrubSlider : public QSlider {
public:
    using QSlider::QSlider;
protected:
    void mousePressEvent(QMouseEvent* e) override {
        if (e->button() == Qt::LeftButton) {
            QStyleOptionSlider opt;
            initStyleOption(&opt);
            const QRect handle = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);
            if (!handle.contains(e->pos())) {
                const int span = width() - handle.width();
                const int x = int(e->position().x()) - handle.width() / 2;
                setValue(QStyle::sliderValueFromPosition(minimum(), maximum(), x, span));
            }
        }
        QSlider::mousePressEvent(e);
    }
};
} // namespace

TimelineBar::TimelineBar(SimulationEngine* sim, Viewport* viewport, QWidget* parent)
    : QWidget(parent), m_sim(sim), m_viewport(viewport)
{
    setObjectName("island");
    setAttribute(Qt::WA_StyledBackground);
    auto* h = new QHBoxLayout(this);
    h->setContentsMargins(12, 6, 8, 6);
    h->setSpacing(8);

    m_rec = new QLabel("● REC");
    m_rec->setObjectName("recChip");
    m_rec->setToolTip("Every scan is recorded until Reset. Pause, then drag the timeline to look back.");
    m_slider = new ScrubSlider(Qt::Horizontal);
    m_slider->setToolTip("Drag to look back through the run (pauses the simulation). "
                         "Play or Step continue from the latest scan.");
    m_time = new QLabel;
    m_time->setObjectName("timeChip");
    m_live = new QPushButton(Theme::icon("chevrons-right"), "Live");
    m_live->setToolTip("Back to the latest scan");
    m_export = new QPushButton(Theme::icon("film"), "Export Video…");
    m_export->setToolTip("Save the recorded run as an MP4 video, an animated GIF or PNG frames");
    h->addWidget(m_rec);
    h->addWidget(m_slider, 1);
    h->addWidget(m_time);
    h->addWidget(m_live);
    h->addWidget(m_export);

    connect(m_slider, &QSlider::valueChanged, this, [this](int v) {
        if (!m_updating) m_sim->seek(v);
    });
    connect(m_live, &QPushButton::clicked, m_sim, &SimulationEngine::seekLive);
    connect(m_export, &QPushButton::clicked, this, &TimelineBar::exportRun);
    connect(m_sim, &SimulationEngine::stepped, this, &TimelineBar::sync);
    connect(m_sim, &SimulationEngine::scrubbed, this, &TimelineBar::sync);
    connect(m_sim, &SimulationEngine::stateChanged, this, &TimelineBar::sync);
    connect(m_sim, &SimulationEngine::activeChanged, this, [this](bool active) {
        setVisible(active);
        sync();
    });
    setVisible(false);
}

void TimelineBar::sync() {
    const Recording& rec = m_sim->recording();
    const int last = std::max(0, rec.frames() - 1);
    const int frame = m_sim->viewFrame();
    m_updating = true;
    m_slider->setRange(0, last);
    m_slider->setValue(std::max(0, frame));
    m_updating = false;
    const bool running = m_sim->isRunning();
    m_rec->setProperty("active", running && !rec.full());
    m_rec->setText(rec.full() ? "■ FULL" : running ? "● REC" : "❚❚");
    m_rec->setToolTip(rec.full() ? "The recording is full (20 minutes); later scans are not recorded."
                                 : "Every scan is recorded until Reset. Pause, then drag the timeline to look back.");
    Theme::repolish(m_rec);
    m_time->setText(QString("%1 / %2 s").arg(rec.time(frame), 0, 'f', 2).arg(rec.duration(), 0, 'f', 2));
    m_live->setEnabled(m_sim->isScrubbing());
    m_export->setEnabled(!running && rec.frames() > 1);
}

// ── Export ───────────────────────────────────────────────────────────────────

void TimelineBar::exportRun() {
    const Recording& rec = m_sim->recording();
    if (rec.frames() < 2) return;
    const QString ffmpeg = QStandardPaths::findExecutable("ffmpeg");

    // Settings
    QDialog dlg(this);
    dlg.setWindowTitle("Export Video");
    auto* vb = new QVBoxLayout(&dlg);
    vb->setContentsMargins(18, 16, 18, 16);
    vb->setSpacing(10);
    auto* title = new QLabel("Export the recorded run");
    title->setObjectName("dialogTitle");
    vb->addWidget(title);
    auto* sub = new QLabel(ffmpeg.isEmpty()
        ? "ffmpeg was not found, so only PNG frames can be saved. Install ffmpeg for MP4 and GIF."
        : "Frames are rendered from the viewport as it looks now: same camera, size and overlays.");
    sub->setObjectName("hint");
    sub->setWordWrap(true);
    vb->addWidget(sub);
    auto* form = new QFormLayout;
    form->setLabelAlignment(Qt::AlignRight);
    auto* format = new QComboBox;
    if (!ffmpeg.isEmpty()) {
        format->addItem("MP4 video (H.264)", "mp4");
        format->addItem("Animated GIF", "gif");
    }
    format->addItem("PNG frames (a folder)", "png");
    auto* fps = new QComboBox;
    for (int f : {15, 24, 30, 60}) fps->addItem(QString("%1 fps").arg(f), f);
    fps->setCurrentIndex(2);
    auto* scale = new QComboBox;
    for (int s : {100, 75, 50}) scale->addItem(QString("%1 %").arg(s), s);
    auto* from = new QDoubleSpinBox;
    auto* to = new QDoubleSpinBox;
    for (auto* sp : {from, to}) {
        sp->setRange(0, rec.duration());
        sp->setDecimals(2);
        sp->setSuffix(" s");
        sp->setSingleStep(0.5);
    }
    to->setValue(rec.duration());
    auto* range = new QHBoxLayout;
    range->addWidget(from);
    range->addWidget(new QLabel("to"));
    range->addWidget(to);
    form->addRow("Format", format);
    form->addRow("Frame rate", fps);
    form->addRow("Size", scale);
    form->addRow("Time", range);
    vb->addLayout(form);
    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    btns->button(QDialogButtonBox::Ok)->setText("Export…");
    btns->button(QDialogButtonBox::Ok)->setDefault(true);
    vb->addWidget(btns);
    btns->button(QDialogButtonBox::Ok)->setFocus();
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    if (dlg.exec() != QDialog::Accepted) return;

    const QString kind = format->currentData().toString();
    const int rate = fps->currentData().toInt();
    const double pct = scale->currentData().toInt() / 100.0;
    const double t0 = std::min(from->value(), to->value()), t1 = std::max(from->value(), to->value());

    QString out;
    if (kind == "png") {
        out = QFileDialog::getExistingDirectory(this, "Folder for the PNG frames", QDir::homePath());
    } else {
        const QString filter = kind == "mp4" ? "MP4 video (*.mp4)" : "Animated GIF (*.gif)";
        out = QFileDialog::getSaveFileName(this, "Export Video", QDir::home().filePath("motionstudio." + kind), filter);
        if (!out.isEmpty() && QFileInfo(out).suffix().toLower() != kind) out += "." + kind;
    }
    if (out.isEmpty()) return;

    // Frame size: even numbers (H.264 needs them)
    QImage probe = m_viewport->grabFramebuffer();
    QSize size = (QSizeF(probe.size()) * pct).toSize();
    size = QSize(std::max(2, size.width() & ~1), std::max(2, size.height() & ~1));

    QProcess proc;
    if (kind != "png") {
        QStringList args = {"-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgba",
                            "-s", QString("%1x%2").arg(size.width()).arg(size.height()),
                            "-r", QString::number(rate), "-i", "-"};
        if (kind == "mp4")
            args << "-c:v" << "libx264" << "-pix_fmt" << "yuv420p" << "-crf" << "20" << out;
        else
            args << "-vf" << "split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer"
                 << "-loop" << "0" << out;
        proc.start(ffmpeg, args);
        if (!proc.waitForStarted()) {
            QMessageBox::warning(this, "Export Video", "Could not start ffmpeg.");
            return;
        }
    }

    const int restore = m_sim->isScrubbing() ? m_sim->viewFrame() : -1;
    const int count = std::max(1, int(std::floor((t1 - t0) * rate)) + 1);
    QProgressDialog progress("Rendering frames…", "Cancel", 0, count, this);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(0);
    bool cancelled = false;
    for (int i = 0; i < count; ++i) {
        progress.setValue(i);
        QApplication::processEvents();
        if (progress.wasCanceled()) { cancelled = true; break; }
        m_sim->seek(rec.frameAt(t0 + double(i) / rate));
        QImage img = m_viewport->grabFramebuffer();
        if (img.size() != size) img = img.scaled(size, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
        if (kind == "png") {
            img.save(QDir(out).filePath(QString("frame_%1.png").arg(i + 1, 5, 10, QChar('0'))));
        } else {
            img = img.convertToFormat(QImage::Format_RGBA8888);
            for (int y = 0; y < img.height(); ++y)
                proc.write(reinterpret_cast<const char*>(img.constScanLine(y)), qint64(img.width()) * 4);
            while (proc.bytesToWrite() > (8 << 20)) proc.waitForBytesWritten(1000);
        }
    }
    progress.setLabelText("Encoding…");
    QApplication::processEvents();
    QString error;
    if (kind != "png") {
        proc.closeWriteChannel();
        if (cancelled) proc.kill();
        proc.waitForFinished(-1);
        if (!cancelled && (proc.exitStatus() != QProcess::NormalExit || proc.exitCode() != 0))
            error = QString::fromLocal8Bit(proc.readAllStandardError()).trimmed();
    }
    progress.setValue(count);
    if (restore >= 0) m_sim->seek(restore); else m_sim->seekLive();

    if (cancelled) { emit statusMessage("Export cancelled", false); return; }
    if (!error.isEmpty()) {
        QMessageBox::warning(this, "Export Video", "ffmpeg failed:\n\n" + error.left(800));
        return;
    }
    emit statusMessage(QString("Exported %1 frames to %2").arg(count).arg(QDir::toNativeSeparators(out)), true);
}
