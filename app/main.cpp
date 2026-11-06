#include <QCommandLineParser>
#include <QDir>
#include <QFileSystemWatcher>
#include <QGuiApplication>
#include <QImage>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickWindow>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QUrl>
#include <QVariantMap>
#include <QVector3D>
#include <QtQml/qqml.h>

#include <cstdio>

#include "arm_controller.hpp"
#include "geometry/rounded_box_geometry.hpp"
#include "geometry/tube_geometry.hpp"
#include "hand_controller.hpp"

#ifndef APP_QML_DIR
#error "APP_QML_DIR must be defined by the build"
#endif
#ifndef PROJECT_ROOT_DIR
#error "PROJECT_ROOT_DIR must be defined by the build"
#endif

namespace {

// "a,b,c" -> {a, b, c}; empty list on any parse error.
QList<double> parse_number_list(const QString &text) {
  QList<double> values;
  for (const QString &part : text.split(',', Qt::SkipEmptyParts)) {
    bool ok = false;
    const double value = part.trimmed().toDouble(&ok);
    if (!ok) {
      return {};
    }
    values.append(value);
  }
  return values;
}

} // namespace

int main(int argc, char *argv[]) {
  QGuiApplication app(argc, argv);
  QGuiApplication::setApplicationName("Humanoid Arm Load Analyzer");

  // Optional flags - all of them exist to make reproducible screenshots
  // (README images, visual checks) without clicking through the UI.
  QCommandLineParser parser;
  parser.setApplicationDescription("Humanoid arm + hand load analyzer");
  parser.addHelpOption();
  const QCommandLineOption poseOption("pose", "Initial joint angles in degrees, e.g. -30,70,-10.", "degrees");
  const QCommandLineOption payloadOption("payload", "Initial payload in kg.", "kg");
  const QCommandLineOption openHandOption("open-hand", "Start with the hand open (not holding the payload).");
  const QCommandLineOption cameraOption("camera", "Initial camera yaw,pitch,distance (degrees, degrees, meters).",
                                        "yaw,pitch,distance");
  const QCommandLineOption focusOption("focus", "Point the camera orbits around, x,y,z in meters.", "x,y,z");
  const QCommandLineOption screenshotOption("screenshot", "Save a PNG of the window after --delay ms, then quit.",
                                            "file.png");
  const QCommandLineOption delayOption("delay", "Milliseconds to wait before --screenshot (default 2500).", "ms",
                                       "2500");
  const QCommandLineOption scanOption("scan", "Run a payload-capacity scan with this many samples at startup.",
                                      "samples");
  const QCommandLineOption sizeOption("size", "Window size, e.g. 1600x1000.", "WxH");
  parser.addOptions({poseOption, payloadOption, openHandOption, cameraOption, focusOption, screenshotOption,
                     delayOption, scanOption, sizeOption});
  parser.process(app);

  // Lets QML say `Model { geometry: RoundedBoxGeometry { ... } }`.
  qmlRegisterType<robotarm::RoundedBoxGeometry>("Humanoid.Geometry", 1, 0, "RoundedBoxGeometry");
  qmlRegisterType<robotarm::TubeGeometry>("Humanoid.Geometry", 1, 0, "TubeGeometry");

  robotarm::ArmController controller(std::string(PROJECT_ROOT_DIR) + "/config/humanoid_arm.json",
                                     std::string(PROJECT_ROOT_DIR) + "/ml/models/humanoid_arm.surrogate.json",
                                     std::string(PROJECT_ROOT_DIR) + "/ml/models/humanoid_arm_dynamic.surrogate.json");

  robotarm::HandController handController(std::string(PROJECT_ROOT_DIR) + "/config/humanoid_hand.json");

  // Start in a natural carrying pose (upper arm angled down-forward,
  // elbow bent, hand slightly up - a dumbbell curl) rather than the arm
  // stuck straight out; --pose overrides it.
  const QList<double> pose = parser.isSet(poseOption) ? parse_number_list(parser.value(poseOption))
                                                      : QList<double>{-35.0, 80.0, -10.0};
  for (int i = 0; i < pose.size(); i++) {
    controller.setJointAngleDegrees(i, pose[i]);
  }
  if (parser.isSet(payloadOption)) {
    controller.setPayloadKg(parser.value(payloadOption).toDouble());
  }
  if (parser.isSet(openHandOption)) {
    controller.setGripperOpen(true);
  }
  if (parser.isSet(scanOption)) {
    controller.runPayloadScan(parser.value(scanOption).toInt());
  }

  QVariantMap launchOptions;
  if (parser.isSet(cameraOption)) {
    const QList<double> camera = parse_number_list(parser.value(cameraOption));
    if (camera.size() == 3) {
      launchOptions["cameraYaw"] = camera[0];
      launchOptions["cameraPitch"] = camera[1];
      launchOptions["cameraDistance"] = camera[2];
    }
  }
  if (parser.isSet(focusOption)) {
    const QList<double> focus = parse_number_list(parser.value(focusOption));
    if (focus.size() == 3) {
      launchOptions["focus"] = QVector3D(focus[0], focus[1], focus[2]);
    }
  }
  if (parser.isSet(sizeOption)) {
    const QStringList parts = parser.value(sizeOption).split('x');
    if (parts.size() == 2) {
      launchOptions["width"] = parts[0].toInt();
      launchOptions["height"] = parts[1].toInt();
    }
  }

  QQmlApplicationEngine engine;
  engine.rootContext()->setContextProperty("armController", &controller);
  engine.rootContext()->setContextProperty("handController", &handController);
  engine.rootContext()->setContextProperty("launchOptions", launchOptions);

  const QString qmlDirPath(APP_QML_DIR);
  const QUrl mainQmlUrl = QUrl::fromLocalFile(qmlDirPath + "/Main.qml");

  // Hot reload: watch every .qml file and reload on change.
  QFileSystemWatcher qmlWatcher;
  const QDir qmlDir(qmlDirPath);
  for (const QString &fileName : qmlDir.entryList(QStringList() << "*.qml", QDir::Files)) {
    qmlWatcher.addPath(qmlDir.filePath(fileName));
  }
  QObject::connect(&qmlWatcher, &QFileSystemWatcher::fileChanged, &engine,
                   [&engine, &mainQmlUrl, &qmlWatcher](const QString &path) {
                     if (!qmlWatcher.files().contains(path)) qmlWatcher.addPath(path);
                     engine.clearComponentCache();
                     engine.load(mainQmlUrl);
                   });

  engine.load(mainQmlUrl);
  if (engine.rootObjects().isEmpty()) return -1;

  if (parser.isSet(screenshotOption)) {
    const QString path = parser.value(screenshotOption);
    QTimer::singleShot(parser.value(delayOption).toInt(), &app, [&engine, path]() {
      auto *window = qobject_cast<QQuickWindow *>(engine.rootObjects().constLast());
      int status = 1;
      if (window != nullptr) {
        const QImage image = window->grabWindow();
        if (image.save(path)) {
          std::printf("saved %s (%dx%d)\n", qPrintable(path), image.width(), image.height());
          status = 0;
        }
      }
      QCoreApplication::exit(status);
    });
  }

  return app.exec();
}
