#pragma once
#include <QDialog>
#include <QStringList>

class QLineEdit;
class QListWidget;
class QTextBrowser;
class QUrl;

// The user guide: topics on the left, the selected one on the right. Topics
// are Markdown files in resources/help (":/help/<id>.md"). Every fenced code
// block gets a Copy link below it, and "help:<id>" links jump to a topic.
// One shared, non-modal window, so it can stay open beside the app.
class HelpDialog : public QDialog {
    Q_OBJECT
public:
    // Opens (or raises) the guide at `topic`, e.g. "start", "actuators", "recipes"
    static void showTopic(const QString& topic, QWidget* parent = nullptr);

private:
    explicit HelpDialog(QWidget* parent);

    void setTopic(const QString& topic);
    // Renders m_topic again, e.g. with the new palette after a theme switch
    void render();
    void filterTopics(const QString& text);
    void openLink(const QUrl& url);

    QLineEdit*    m_search = nullptr;
    QListWidget*  m_topics = nullptr;
    QTextBrowser* m_view   = nullptr;
    QString       m_topic;
    QStringList   m_snippets;   // code blocks of the shown topic, by Copy link index
};
