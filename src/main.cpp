#include <QApplication>
#include <QIcon>
#include <QSurfaceFormat>
#include "MainWindow.h"
#include "ui/Theme.h"

int main(int argc, char* argv[]) {
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setSamples(4); // MSAA
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);
    app.setApplicationName("MotionStudio");
    app.setApplicationVersion("0.1.0");
    app.setDesktopFileName("motionstd");
    app.setWindowIcon(QIcon(":/icon.png"));
    Theme::apply(app);

    MainWindow window;
    window.show();

    // Optional: project file to open on startup
    const QStringList args = app.arguments();
    if (args.size() > 1 && !args.at(1).startsWith('-'))
        window.openFile(args.at(1));

    return app.exec();
}
