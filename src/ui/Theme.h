#pragma once
#include <QColor>
#include <QIcon>
#include <QObject>
#include <QString>
#include <array>

class QApplication;
class QLabel;
class QAbstractButton;
class QWidget;

// The application look: monochrome dark or light "islands" (rounded panels
// floating on a darker frame, as in wowcad / UbuntuBuddy), the bundled Plus
// Jakarta Sans font and Lucide icons.
// Widgets never hard-code a colour: they get an objectName (or a dynamic
// property) that the stylesheet in Theme.cpp styles, or read colors() when
// they paint themselves — at paint time, so a theme switch only needs a repaint.
namespace Theme {

enum class Mode { Dark, Light };

struct Colors {
    bool dark;
    // Surfaces and text
    QColor frame;        // window background the islands float on
    QColor island;       // island surface
    QColor panel;        // cards, menus, raised surfaces
    QColor hover;
    QColor hoverStrong;  // hovered buttons
    QColor selection;
    QColor selectionHover;
    QColor border;
    QColor inputBorder;
    QColor handle;       // scroll bar handles, strong borders
    QColor text;
    QColor muted;
    QColor accent;       // primary buttons, checked boxes, focus rings
    QColor accentHover;
    QColor onAccent;

    // Meaning
    QColor success;      // running, done, true
    QColor info;         // running operations, links
    QColor warning;      // paused, locked, warnings
    QColor danger;       // errors, destructive actions
    QColor physics;      // rigid bodies
    QColor axisX, axisY, axisZ;

    // 3D view
    QColor viewport;     // clear colour
    QColor grid;
    QColor gridFloor;    // grid while the floor collider is on
    QColor floorFill;

    // Trace plot series
    std::array<QColor, 8> series;
};

// The active palette
const Colors& colors();
Mode mode();

// Emits changed() after a theme switch, once the palette and stylesheet are
// applied. Widgets that bake colours into data (item backgrounds, rich text)
// listen to it; everything else just repaints.
class Notifier : public QObject {
    Q_OBJECT
signals:
    void changed();
};
Notifier* notifier();

// Loads the fonts, sets the style and applies the saved theme (dark by default)
void apply(QApplication& app);
// Switches the theme, remembers it for the next start and repaints every widget
void setMode(Mode mode);
void toggle();

// Which palette colour an icon is drawn in. Icons are tinted when painted,
// so they follow theme switches.
enum class Tint { Text, Muted, OnAccent, Success, Warning, Danger, Physics };

// A Lucide icon from resources/icons (file name without ".svg"); disabled icons are muted
QIcon icon(const QString& name, Tint tint = Tint::Text);
// Shows an icon in a label and re-renders it after theme switches (labels keep a pixmap)
void setLabelIcon(QLabel* label, const QString& name, int size = 16, Tint tint = Tint::Text);

// "#rrggbb", for rich text
QString css(const QColor& color);
// The colour at low opacity, for soft state backgrounds (table cells, chips)
QColor soft(const QColor& color, qreal alpha = 0.22);

// A checkable button's icon in the text colour, or on-accent while checked
// (for #segment buttons, whose checked background is the accent)
void segmentIcon(QAbstractButton* button, const QString& name);

// Re-applies the stylesheet after a dynamic property used in a selector changed
void repolish(QWidget* w);

} // namespace Theme
