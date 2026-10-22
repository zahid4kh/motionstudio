#include "ExpressionEdit.h"
#include "automation/Expression.h"
#include "ui/Theme.h"
#include <QAbstractItemView>
#include <QCompleter>
#include <QKeyEvent>
#include <QScrollBar>
#include <QStringListModel>

ExpressionEdit::ExpressionEdit(QWidget* parent) : QLineEdit(parent) {
    setClearButtonEnabled(false);

    m_model = new QStringListModel(this);
    m_completer = new QCompleter(m_model, this);
    m_completer->setWidget(this);
    m_completer->setCompletionMode(QCompleter::PopupCompletion);
    m_completer->setCaseSensitivity(Qt::CaseInsensitive);
    m_completer->setFilterMode(Qt::MatchContains);
    connect(m_completer, QOverload<const QString&>::of(&QCompleter::activated),
            this, &ExpressionEdit::insertCompletion);

    connect(this, &QLineEdit::textChanged, this, [this]{ revalidate(); });
    connect(this, &QLineEdit::textEdited, this, [this]{ updateCompletion(); });
    connect(this, &QLineEdit::editingFinished, this, [this]{
        if (m_completer->popup()->isVisible()) return;
        if (text() == m_lastCommitted) return;
        m_lastCommitted = text();
        emit committed(text());
    });
}

void ExpressionEdit::setExpression(const QString& t) {
    m_lastCommitted = t;
    if (text() != t) {
        QSignalBlocker b(this);
        setText(t);
        setCursorPosition(0);
    }
    revalidate();
}

void ExpressionEdit::revalidate() {
    m_error = m_validator ? m_validator(text()) : QString();
    // Styled by the theme through the "error" property (ExpressionEdit[error="true"])
    const bool hadError = property("error").toBool();
    if (m_error.isEmpty()) {
        setToolTip(m_baseTip);
    } else {
        setToolTip((m_baseTip.isEmpty() ? QString() : m_baseTip + "\n\n") + "⚠ " + m_error);
    }
    if (hadError != !m_error.isEmpty()) {
        setProperty("error", !m_error.isEmpty());
        Theme::repolish(this);
    }
}

QString ExpressionEdit::wordUnderCursor(int* start) const {
    const QString t = text();
    int end = cursorPosition();
    int st = end;
    while (st > 0 && (t[st - 1].isLetterOrNumber() || t[st - 1] == '_' || t[st - 1] == '.')) --st;
    if (start) *start = st;
    return t.mid(st, end - st);
}

void ExpressionEdit::updateCompletion() {
    QString prefix = wordUnderCursor();
    if (prefix.isEmpty() || !m_words || prefix[0].isDigit()) {
        m_completer->popup()->hide();
        return;
    }
    QStringList words = m_words();
    words << "true" << "false";
    for (const QString& f : expr::functionNames()) words << f + "(";
    m_model->setStringList(words);
    m_completer->setCompletionPrefix(prefix);
    if (m_completer->completionCount() == 0 ||
        (m_completer->completionCount() == 1 && m_completer->currentCompletion() == prefix)) {
        m_completer->popup()->hide();
        return;
    }
    QRect r = cursorRect();
    r.setWidth(m_completer->popup()->sizeHintForColumn(0) +
               m_completer->popup()->verticalScrollBar()->sizeHint().width() + 12);
    m_completer->complete(r);
    // Preselect the best match so Enter/Tab accept it right away
    m_completer->popup()->setCurrentIndex(m_completer->completionModel()->index(0, 0));
}

void ExpressionEdit::insertCompletion(const QString& word) {
    int st = 0;
    QString prefix = wordUnderCursor(&st);
    QString t = text();
    int end = cursorPosition();
    t.replace(st, end - st, word);
    setText(t);
    setCursorPosition(st + int(word.size()));
    Q_UNUSED(prefix);
}

void ExpressionEdit::keyPressEvent(QKeyEvent* e) {
    if (m_completer->popup()->isVisible()) {
        switch (e->key()) {
            case Qt::Key_Enter: case Qt::Key_Return: case Qt::Key_Escape:
            case Qt::Key_Tab:   case Qt::Key_Backtab:
                e->ignore();   // let the completer handle it
                return;
            default: break;
        }
    }
    if (e->key() == Qt::Key_Escape) {
        setExpression(m_lastCommitted);
        clearFocus();
        return;
    }
    QLineEdit::keyPressEvent(e);
}

void ExpressionEdit::focusInEvent(QFocusEvent* e) {
    QLineEdit::focusInEvent(e);
    revalidate();   // symbols may have changed since this field was last checked
}

void ExpressionEdit::setHelpText(const QString& help) {
    m_baseTip = help;
    revalidate();
}
