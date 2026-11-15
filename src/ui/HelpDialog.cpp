#include "HelpDialog.h"
#include "Theme.h"
#include <QApplication>
#include <QClipboard>
#include <QCursor>
#include <QDesktopServices>
#include <QFile>
#include <QFontDatabase>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPointer>
#include <QTextBlock>
#include <QTextBrowser>
#include <QTextCursor>
#include <QTextDocument>
#include <QTextFrame>
#include <QTextTable>
#include <QToolTip>
#include <QUrl>
#include <QVBoxLayout>

namespace {
struct Topic { const char* id; const char* title; const char* icon; };
const Topic kTopics[] = {
    {"start",       "Getting Started",        "book-open"},
    {"actuators",   "Joints & Actuators",     "cog"},
    {"assembly",    "Assembling Parts",       "puzzle"},
    {"poses",       "Poses",                  "camera"},
    {"kinematics",  "Robot Tools & IK",       "move-diagonal-2"},
    {"robots",      "Robot Builder & Analyzer", "bot"},
    {"humanoids",   "Humanoid Builder",       "person-standing"},
    {"sequences",   "Sequences & Operations", "list-ordered"},
    {"signals",     "Signals",                "toggle-right"},
    {"sensors",     "Sensors & Gripping",     "radio"},
    {"flow",        "Part Sources & Sinks",   "boxes"},
    {"expressions", "Expression Reference",   "code-xml"},
    {"recipes",     "Recipes",                "copy"},
    {"runtime",     "Running & Debugging",    "activity"},
    {"io",          "External I/O (Modbus)",  "network"},
    {"api",         "Robot API (code)",       "square-terminal"},
    {"exchange",    "Import & Export",        "download"},
    {"shortcuts",   "Keyboard & Mouse",       "keyboard"},
};

QString topicSource(const QString& id) {
    QFile f(":/help/" + id + ".md");
    return f.open(QIODevice::ReadOnly) ? QString::fromUtf8(f.readAll()) : QString("# Not found\n\nNo help topic '%1'.").arg(id);
}

QPointer<HelpDialog> s_instance;
} // namespace

void HelpDialog::showTopic(const QString& topic, QWidget* parent) {
    if (!s_instance) s_instance = new HelpDialog(parent ? parent->window() : nullptr);
    s_instance->setTopic(topic);
    s_instance->show();
    s_instance->raise();
    s_instance->activateWindow();
}

HelpDialog::HelpDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle("MotionStudio Guide");
    setWindowFlag(Qt::WindowMaximizeButtonHint);
    resize(900, 640);

    auto* root = new QHBoxLayout(this);
    root->setContentsMargins(14, 14, 14, 14);
    root->setSpacing(12);

    auto* side = new QVBoxLayout;
    side->setSpacing(8);
    auto* title = new QLabel("Guide");
    title->setObjectName("dialogTitle");
    side->addWidget(title);
    m_search = new QLineEdit;
    m_search->setPlaceholderText("Search…");
    m_search->setClearButtonEnabled(true);
    side->addWidget(m_search);
    m_topics = new QListWidget;
    m_topics->setObjectName("templateList");
    m_topics->setIconSize(QSize(16, 16));
    m_topics->setFixedWidth(220);
    for (const Topic& t : kTopics) {
        auto* it = new QListWidgetItem(Theme::icon(t.icon), t.title);
        it->setData(Qt::UserRole, QString(t.id));
        m_topics->addItem(it);
    }
    side->addWidget(m_topics, 1);
    auto* hint = new QLabel("Click Copy under an example to put it on the clipboard.");
    hint->setObjectName("hint");
    hint->setWordWrap(true);
    hint->setMaximumWidth(220);
    side->addWidget(hint);
    root->addLayout(side);

    m_view = new QTextBrowser;
    m_view->setObjectName("helpView");
    m_view->setOpenLinks(false);
    m_view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);   // everything wraps
    m_view->document()->setDocumentMargin(18);
    root->addWidget(m_view, 1);

    connect(m_topics, &QListWidget::currentItemChanged, this, [this](QListWidgetItem* it) {
        if (it) setTopic(it->data(Qt::UserRole).toString());
    });
    connect(m_search, &QLineEdit::textChanged, this, &HelpDialog::filterTopics);
    connect(m_search, &QLineEdit::returnPressed, this, [this] {
        if (!m_search->text().isEmpty() && !m_view->find(m_search->text())) {
            m_view->moveCursor(QTextCursor::Start);   // wrap around
            m_view->find(m_search->text());
        }
    });
    connect(m_view, &QTextBrowser::anchorClicked, this, &HelpDialog::openLink);
    connect(Theme::notifier(), &Theme::Notifier::changed, this, &HelpDialog::render);
}

void HelpDialog::setTopic(const QString& topic) {
    for (int i = 0; i < m_topics->count(); ++i) {
        QListWidgetItem* it = m_topics->item(i);
        if (it->data(Qt::UserRole).toString() != topic) continue;
        it->setHidden(false);
        if (m_topics->currentItem() != it) {
            m_topics->setCurrentItem(it);   // re-enters through currentItemChanged
            return;
        }
    }
    if (topic == m_topic) return;
    m_topic = topic;
    render();
    if (!m_search->text().isEmpty()) m_view->find(m_search->text());
}

void HelpDialog::render() {
    if (m_topic.isEmpty()) return;

    // A Copy link after every fenced code block
    m_snippets.clear();
    QStringList out, code;
    bool inFence = false;
    for (const QString& line : topicSource(m_topic).split('\n')) {
        // Code lines are indented, as a block format has no padding
        out << (inFence && !line.startsWith("```") ? "  " + line : line);
        if (line.startsWith("```")) {
            if (inFence) {
                m_snippets << code.join('\n');
                out << "" << QString("[Copy](copy:%1)").arg(m_snippets.size() - 1) << "";
            }
            inFence = !inFence;
            code.clear();
        } else if (inFence) {
            code << line;
        }
    }

    QTextDocument* doc = m_view->document();
    doc->setMarkdown(out.join('\n'), QTextDocument::MarkdownDialectGitHub);

    // Markdown import knows nothing about the palette: colour code, quotes,
    // links and tables here (and again after a theme switch)
    const Theme::Colors& c = Theme::colors();
    QFont mono = QFontDatabase::systemFont(QFontDatabase::FixedFont);
    mono.setPointSizeF(QApplication::font().pointSizeF() * 0.92);
    auto isCode = [](const QTextBlock& b) { return b.isValid() && b.blockFormat().hasProperty(QTextFormat::BlockCodeFence); };
    QTextCursor cur(doc);
    for (QTextBlock b = doc->begin(); b.isValid(); b = b.next()) {
        QTextBlockFormat bf = b.blockFormat();
        const bool codeBlock = bf.hasProperty(QTextFormat::BlockCodeFence);
        const bool quote = bf.hasProperty(QTextFormat::BlockQuoteLevel) && bf.intProperty(QTextFormat::BlockQuoteLevel) > 0;
        if (codeBlock || quote) {
            bf.setBackground(codeBlock ? c.panel : Theme::soft(c.info, 0.14));
            bf.setLeftMargin(0);
            // One block per code line: only the first and last get outer spacing
            bf.setTopMargin(codeBlock && isCode(b.previous()) ? 0 : 6);
            bf.setBottomMargin(codeBlock && isCode(b.next()) ? 0 : (codeBlock ? 2 : 6));
            bf.setNonBreakableLines(false);
            cur.setPosition(b.position());
            cur.setBlockFormat(bf);
        }
        if (bf.headingLevel() > 0) {
            bf.setTopMargin(bf.headingLevel() == 1 ? 0 : 16);
            bf.setBottomMargin(6);
            cur.setPosition(b.position());
            cur.setBlockFormat(bf);
        }
        for (auto it = b.begin(); !it.atEnd(); ++it) {
            const QTextFragment frag = it.fragment();
            QTextCharFormat cf = frag.charFormat();
            if (cf.isAnchor()) {
                const bool copy = cf.anchorHref().startsWith("copy:");
                cf.setForeground(c.info);
                cf.setFontUnderline(!copy);
                if (copy) {
                    cf.setFontPointSize(QApplication::font().pointSizeF() * 0.9);
                    cf.setFontWeight(QFont::DemiBold);
                }
            } else if (codeBlock) {
                cf.setFont(mono);
                cf.setForeground(c.text);
            } else if (cf.fontFixedPitch()) {
                cf.setFont(mono);
                cf.setBackground(Theme::soft(c.text, 0.10));
            } else {
                continue;
            }
            cur.setPosition(frag.position());
            cur.setPosition(frag.position() + frag.length(), QTextCursor::KeepAnchor);
            cur.setCharFormat(cf);
        }
    }
    for (QTextFrame* f : doc->rootFrame()->childFrames()) {
        auto* table = qobject_cast<QTextTable*>(f);
        if (!table) continue;
        QTextTableFormat tf = table->format();
        tf.setBorderCollapse(true);
        tf.setBorder(1);
        tf.setBorderBrush(c.border);
        tf.setBorderStyle(QTextFrameFormat::BorderStyle_Solid);
        tf.setCellPadding(6);
        tf.setCellSpacing(0);
        tf.setTopMargin(4);
        tf.setBottomMargin(8);
        tf.setWidth(QTextLength(QTextLength::PercentageLength, 100));
        table->setFormat(tf);
        for (int col = 0; col < table->columns(); ++col) {
            QTextTableCell cell = table->cellAt(0, col);
            QTextCharFormat hf = cell.format();
            hf.setBackground(c.panel);
            cell.setFormat(hf);
        }
    }
    m_view->moveCursor(QTextCursor::Start);
}

void HelpDialog::filterTopics(const QString& text) {
    const QString needle = text.trimmed();
    QListWidgetItem* firstVisible = nullptr;
    for (int i = 0; i < m_topics->count(); ++i) {
        QListWidgetItem* it = m_topics->item(i);
        const bool match = needle.isEmpty() || it->text().contains(needle, Qt::CaseInsensitive) ||
                           topicSource(it->data(Qt::UserRole).toString()).contains(needle, Qt::CaseInsensitive);
        it->setHidden(!match);
        if (match && !firstVisible) firstVisible = it;
    }
    QListWidgetItem* cur = m_topics->currentItem();
    if (firstVisible && (!cur || cur->isHidden()))
        m_topics->setCurrentItem(firstVisible);
    else if (!needle.isEmpty()) {
        m_view->moveCursor(QTextCursor::Start);
        m_view->find(needle);
    }
}

void HelpDialog::openLink(const QUrl& url) {
    if (url.scheme() == "copy") {
        const int i = url.path().toInt();
        if (i < 0 || i >= m_snippets.size()) return;
        QApplication::clipboard()->setText(m_snippets[i]);
        QToolTip::showText(QCursor::pos(), "Copied", m_view);
    } else if (url.scheme() == "help") {
        setTopic(url.path());
    } else {
        QDesktopServices::openUrl(url);
    }
}
