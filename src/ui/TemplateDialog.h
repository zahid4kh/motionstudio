#pragma once
#include <QDialog>
#include <QListWidget>
#include <QLabel>

class TemplateDialog : public QDialog {
    Q_OBJECT
public:
    explicit TemplateDialog(QWidget* parent = nullptr);

    // Returns the Qt resource path of the chosen template, e.g. ":/templates/robotic_arm.mstudio"
    QString selectedPath() const;

private:
    struct TemplateInfo {
        QString name;
        QString description;
        QString path;
    };

    void populate();
    void onItemChanged(QListWidgetItem* item);

    QListWidget* m_list  = nullptr;
    QLabel*      m_desc  = nullptr;

    static const QList<TemplateInfo> s_templates;
};
