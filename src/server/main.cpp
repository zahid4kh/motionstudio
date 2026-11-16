// motionstudio-server: the robot API without the app window. Loads a project
// (or builds a robot from a builder preset) and serves the API on a port, so
// robot programs can run headless, in CI or on another machine.
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QTextStream>
#include <cstdio>
#include "io/ApiServer.h"
#include "physics/SimulationEngine.h"
#include "robot/Gait.h"
#include "robot/HumanoidBuilder.h"
#include "robot/RobotBuilder.h"
#include "scene/SceneGraph.h"
#include "serialization/ProjectSerializer.h"

namespace {

void print(const QString& s) {
    std::fputs(qPrintable(s + '\n'), stdout);
    std::fflush(stdout);
}

QString presetList() {
    QStringList lines{"Robot presets (--robot):"};
    for (const auto& p : robot::presets()) lines << "  " + p.name;
    lines << "Humanoid presets (--humanoid):";
    for (const auto& p : humanoid::presets()) lines << "  " + p.name;
    return lines.join('\n');
}

} // namespace

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    app.setApplicationName("motionstudio-server");
    app.setApplicationVersion(MS_VERSION);

    QCommandLineParser cli;
    cli.setApplicationDescription("Serves the MotionStudio robot API for a project, without the app window.");
    cli.addHelpOption();
    cli.addVersionOption();
    cli.addPositionalArgument("project", "The .mstudio project to load.", "[project]");
    QCommandLineOption robotOpt("robot", "Build a robot from a Robot Builder preset instead (see --presets).", "preset");
    QCommandLineOption humanoidOpt("humanoid", "Build a humanoid from a Humanoid Builder preset instead.", "preset");
    QCommandLineOption portOpt({"p", "port"}, "TCP port (default 5800).", "port", QString::number(ApiServer::kDefaultPort));
    QCommandLineOption anyOpt("any", "Listen on every network interface, not only this computer.");
    QCommandLineOption speedOpt("speed", "Simulation speed factor, 0.05 … 20 (default 1 = real time).", "factor", "1");
    QCommandLineOption presetsOpt("presets", "List the robot and humanoid presets.");
    cli.addOptions({robotOpt, humanoidOpt, portOpt, anyOpt, speedOpt, presetsOpt});
    cli.process(app);

    if (cli.isSet(presetsOpt)) { print(presetList()); return 0; }

    SceneGraph scene;
    QString what;
    if (cli.isSet(robotOpt) || cli.isSet(humanoidOpt)) {
        const bool human = cli.isSet(humanoidOpt);
        const QString name = cli.value(human ? humanoidOpt : robotOpt);
        // An exact name, else the first preset whose name contains it ("cobot")
        auto pick = [&](const QStringList& names) {
            for (int i = 0; i < names.size(); ++i) if (names[i].compare(name, Qt::CaseInsensitive) == 0) return i;
            for (int i = 0; i < names.size(); ++i) if (names[i].contains(name, Qt::CaseInsensitive)) return i;
            return -1;
        };
        QStringList names;
        if (human) for (const auto& p : humanoid::presets()) names << p.name;
        else       for (const auto& p : robot::presets()) names << p.name;
        const int i = pick(names);
        const bool found = i >= 0;
        if (found) {
            if (human) {
                // Without the looping demo sequence, which would fight the program's moves
                humanoid::Spec spec = humanoid::presets()[size_t(i)].spec;
                spec.demo = false;
                humanoid::build(scene, spec);
            }
            else       robot::build(scene, robot::presets()[size_t(i)].spec);
        }
        if (!found) {
            print(QString("No preset \"%1\".\n%2").arg(name, presetList()));
            return 2;
        }
        what = QString("%1 preset \"%2\"").arg(human ? "humanoid" : "robot", names[i]);
    } else if (cli.positionalArguments().size() == 1) {
        what = cli.positionalArguments().front();
        if (!ProjectSerializer::load(scene, what)) {
            print("Can't open " + what);
            return 2;
        }
    } else {
        cli.showHelp(2);
    }

    SimulationEngine sim(&scene);
    sim.setSpeedFactor(cli.value(speedOpt).toDouble());
    QObject::connect(&sim, &SimulationEngine::startFailed, [](const QStringList& errs) {
        print("The simulation can't start:\n  " + errs.join("\n  "));
    });

    ApiServer server(&scene, &sim);
    const quint16 port = quint16(cli.value(portOpt).toUInt());
    if (!server.start(port, cli.isSet(anyOpt) ? QHostAddress::Any : QHostAddress::LocalHost)) {
        print(QString("Can't listen on port %1: %2").arg(port).arg(server.lastError()));
        return 1;
    }
    print(QString("Serving %1 on port %2").arg(what).arg(server.port()));
    for (const auto& r : ApiServer::findRobots(scene)) {
        QStringList tools;
        for (int t : r.tools) tools << scene.getObject(t)->name;
        print(QString("  %1: %2 joints%3%4").arg(r.path).arg(r.joints.size())
                  .arg(tools.isEmpty() ? QString() : ", tool " + tools.join(", "))
                  .arg(gait::canWalk(scene, r.root) ? ", walks" : ""));
    }
    QObject::connect(&server, &ApiServer::clientsChanged, [](int n) {
        print(QString("%1 client%2 connected").arg(n).arg(n == 1 ? "" : "s"));
    });
    return app.exec();
}
