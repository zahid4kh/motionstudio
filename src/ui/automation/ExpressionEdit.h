#pragma once
#include <QLineEdit>
#include <functional>

class QCompleter;
class QStringListModel;

// Single-line expression editor: validates as you type (red border + tooltip
// with the error), autocompletes symbol names at the cursor, and emits
// committed() when editing finishes with changed text.
class ExpressionEdit : public QLineEdit {
    Q_OBJECT
public:
    using Validator = std::function<QString(const QString&)>;     // returns error or ""
    using WordSource = std::function<QStringList()>;

    explicit ExpressionEdit(QWidget* parent = nullptr);

    void setValidator(Validator v) { m_validator = std::move(v); revalidate(); }
    void setWordSource(WordSource s) { m_words = std::move(s); }
    // Sets text without emitting committed()
    void setExpression(const QString& text);
    void revalidate();
    void setHelpText(const QString& help);   // tooltip shown above any error
    bool isValid() const { return m_error.isEmpty(); }

signals:
    void committed(const QString& text);

protected:
    void keyPressEvent(QKeyEvent* e) override;
    void focusInEvent(QFocusEvent* e) override;

private:
    QString wordUnderCursor(int* start = nullptr) const;
    void insertCompletion(const QString& word);
    void updateCompletion();

    Validator         m_validator;
    WordSource        m_words;
    QCompleter*       m_completer = nullptr;
    QStringListModel* m_model     = nullptr;
    QString           m_error;
    QString           m_lastCommitted;
    QString           m_baseTip;
};
