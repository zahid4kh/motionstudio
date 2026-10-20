#pragma once
#include <QWidget>

// Wraps a widget in a rounded "island" floating on the window frame: an
// overlay repaints the outer corners in the frame colour and draws a hairline
// border, so the (square) content looks clipped with smooth, antialiased
// corners. Works over the OpenGL viewport too.
class Island : public QWidget {
public:
    explicit Island(QWidget* content, QWidget* parent = nullptr);

    // Rounds an existing widget the same way (e.g. a QDockWidget, so its title
    // bar and contents form one island while it stays movable/floatable)
    static void decorate(QWidget* target);
};
