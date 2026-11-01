#pragma once
#include <QDialog>
#include <QListWidget>
#include <QLabel>

// Picks a starter project (Templates: replaces the scene) or a component
// to insert into the current scene (Components: the library)
class TemplateDialog : public QDialog {
    Q_OBJECT
public:
    enum class Kind { Templates, Components };
    explicit TemplateDialog(QWidget* parent = nullptr, Kind kind = Kind::Templates);

    // Returns the Qt resource path of the choice, e.g. ":/templates/robotic_arm.mstudio"
    QString selectedPath() const;
    QString selectedName() const;

private:
    struct TemplateInfo {
        QString name;
        QString description;
        QString path;
        QString icon;   // Lucide icon name (Theme::icon)
    };

    void populate();
    void onItemChanged(QListWidgetItem* item);

    QListWidget* m_list  = nullptr;
    QLabel*      m_desc  = nullptr;
    const QList<TemplateInfo>* m_items = nullptr;

    static const QList<TemplateInfo> s_templates;
    static const QList<TemplateInfo> s_components;
};
