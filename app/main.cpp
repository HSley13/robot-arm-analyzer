#include <QCommandLineParser>
#include <QGuiApplication>
#include <QImage>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickWindow>
#include <QStandardPaths>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QUrl>
#include <QVariantMap>
#include <QVector3D>
#include <QtQml/qqmlextensionplugin.h>

#include <algorithm>
#include <cstdio>

#include "arm_controller.hpp"
#include "hand_controller.hpp"
#include "qml_hot_reloader.hpp"
#ifdef ROBOTARM_HAS_SENSOR_BRIDGE
#include "sensor_bridge_server.hpp"
#include "session_recorder.hpp"
#endif
#include "task_controller.hpp"
#include "webcam_controller.hpp"

#ifndef APP_QML_DIR
#error "APP_QML_DIR must be defined by the build"
#endif
#ifndef PROJECT_ROOT_DIR
#error "PROJECT_ROOT_DIR must be defined by the build"
#endif

// The controllers' QML module (Humanoid.Backend) is a static library:
// this links in its type registrations.
Q_IMPORT_QML_PLUGIN(Humanoid_BackendPlugin)

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
  const QCommandLineOption pickOption("pick", "Pick up object N (1-based, in creation order) at startup.", "N");
  const QCommandLineOption putOption("put", "After --pick, put the object down at x,z (meters).", "x,z");
  const QCommandLineOption atOption("at", "Stop the --pick/--put motion this many seconds in (default: run it to the end).",
                                    "seconds");
  const QCommandLineOption sizeOption("size", "Window size, e.g. 1600x1000.", "WxH");
  const QCommandLineOption industrialOption("industrial", "Start with the warehouse arm with N joints.", "N");
  const QCommandLineOption tableOption("table", "Add a table: kind,x,z (kind: table, coffee table, counter). Repeatable.",
                                       "kind,x,z");
  const QCommandLineOption addOption("add", "Add an object: type,x,z (on the floor or a table). Repeatable.", "type,x,z");
  // The sensor bridge: outside programs (tools/webcam_teleop) drive the
  // robot over a WebSocket on this port.
  const QCommandLineOption bridgePortOption("bridge-port", "Port for outside control, e.g. the webcam (default 8765).",
                                            "port", "8765");
  const QCommandLineOption bridgeLanOption("bridge-lan",
                                           "Accept outside control from other computers on the network, not just "
                                           "this one (no authentication: trusted networks only).");
  const QCommandLineOption noBridgeOption("no-bridge", "Don't accept outside control at all.");
  // Every outside-control session is written down (see session_recorder.hpp).
  const QCommandLineOption recordDirOption(
      "record-dir", "Where outside-control sessions are recorded (default: the app's data folder, sessions/).", "dir");
  const QCommandLineOption noRecordOption("no-record", "Don't record outside-control sessions.");
  // Webcam control runs inside the app (see webcam_controller.hpp).
  const QCommandLineOption noWebcamOption("no-webcam",
                                          "Don't start webcam control at launch (it can still be turned on in the window).");
  parser.addOptions({poseOption, payloadOption, openHandOption, cameraOption, focusOption, screenshotOption,
                     delayOption, scanOption, pickOption, putOption, atOption, sizeOption,
                     industrialOption, tableOption, addOption, bridgePortOption,
                     bridgeLanOption, noBridgeOption, recordDirOption, noRecordOption, noWebcamOption});
  parser.process(app);


  robotarm::ArmController controller(std::string(PROJECT_ROOT_DIR) + "/config/humanoid_arm.json",
                                     std::string(PROJECT_ROOT_DIR) + "/ml/models/humanoid_arm.surrogate.json",
                                     std::string(PROJECT_ROOT_DIR) + "/ml/models/humanoid_arm_dynamic.surrogate.json");

  robotarm::HandController handController(std::string(PROJECT_ROOT_DIR) + "/config/humanoid_hand.json");

  if (parser.isSet(industrialOption)) {
    controller.convertToIndustrial();
    const int joints = std::max(1, parser.value(industrialOption).toInt());
    while (controller.jointCount() < joints) controller.addJoint();
    while (controller.jointCount() > joints) controller.removeLastJoint();
  }

  // Start in a relaxed ready pose (upper arm down, forearm forward) rather
  // than the arm stuck straight out; --pose overrides it.
  const QList<double> pose = parser.isSet(poseOption) ? parse_number_list(parser.value(poseOption))
                                                      : QList<double>{-70.0, 65.0, 5.0};
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

  // The body (waist / lift / hip) and the objects around the robot, plus
  // a few objects to start with so there's something to pick up.
  robotarm::TaskController taskController(&controller, &handController);
  taskController.addObject("bottle", 1.0, 0.62, 0.22);
  taskController.addObject("dumbbell", 6.0, 0.80, 0.52);
  taskController.addObject("can", 0.35, 0.40, -0.50);
  taskController.addObject("ball", 0.45, 0.25, 0.66);
  for (const QString &value : parser.values(tableOption)) {
    const QStringList parts = value.split(',');
    if (parts.size() == 3) {
      taskController.addTable(parts[0].trimmed(), parts[1].toDouble(), parts[2].toDouble());
    }
  }
  for (const QString &value : parser.values(addOption)) {
    const QStringList parts = value.split(',');
    if (parts.size() == 3) {
      taskController.addObject(parts[0].trimmed(), 0.0, parts[1].toDouble(), parts[2].toDouble());
    }
  }
  taskController.selectObject(-1);
  // Scripted pick / put-down for screenshots: the motion is stepped
  // through directly (advance()), not played in real time, so the shot is
  // the same however fast the machine renders.
  if (parser.isSet(pickOption)) {
    const double at = parser.isSet(atOption) ? parser.value(atOption).toDouble() : 1e9;
    double remaining = at;
    if (taskController.pickUp(parser.value(pickOption).toInt())) {
      while (taskController.busy() && remaining > 0.0) {
        taskController.advance(1.0 / 60.0);
        remaining -= 1.0 / 60.0;
      }
    }
    if (parser.isSet(putOption) && remaining > 0.0) {
      const QList<double> spot = parse_number_list(parser.value(putOption));
      if (spot.size() == 2 && taskController.putDown(spot[0], spot[1])) {
        while (taskController.busy() && remaining > 0.0) {
          taskController.advance(1.0 / 60.0);
          remaining -= 1.0 / 60.0;
        }
      }
    }
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

  // QML reaches these three as singletons of `import Humanoid.Backend`
  // (ArmController, HandController, TaskController) - these instances.
  robotarm::ArmController::setQmlInstance(&controller);
  robotarm::HandController::setQmlInstance(&handController);
  robotarm::TaskController::setQmlInstance(&taskController);

  // The webcam program connects to the sensor bridge, so it needs it running.
  robotarm::WebcamController::Launch webcamLaunch;
  webcamLaunch.unavailableReason = QStringLiteral("Webcam control needs Qt WebSockets (not in this build)");
#ifdef ROBOTARM_HAS_SENSOR_BRIDGE
  // Outside control (the webcam program, scripts...) - see sensor_bridge_server.hpp.
  // The recorder first, so it outlives the bridge that writes to it.
  const QString recordDir =
      parser.isSet(recordDirOption)
          ? parser.value(recordDirOption)
          : QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + QStringLiteral("/sessions");
  robotarm::SessionRecorder sessionRecorder(&taskController, &controller, recordDir);
  robotarm::SensorBridgeServer sensorBridge(&taskController, &controller,
                                            static_cast<quint16>(parser.value(bridgePortOption).toUInt()));
  if (!parser.isSet(noRecordOption)) {
    sensorBridge.setRecorder(&sessionRecorder);
  }
  webcamLaunch.unavailableReason = QStringLiteral("Webcam control needs outside control (started with --no-bridge)");
  if (!parser.isSet(noBridgeOption) && sensorBridge.start(parser.isSet(bridgeLanOption))) {
    webcamLaunch = robotarm::WebcamController::defaultLaunch(QStringLiteral(PROJECT_ROOT_DIR "/tools/webcam_teleop"),
                                                   sensorBridge.port());
    QObject::connect(&sensorBridge, &robotarm::SensorBridgeServer::clientCountChanged, &taskController,
                     [&sensorBridge, &taskController]() {
                       taskController.notify(sensorBridge.clientCount() > 0
                                                 ? QStringLiteral("Outside control connected (webcam)")
                                                 : QStringLiteral("Outside control disconnected"));
                     });
  }
#endif
  // After the bridge, so it's gone (and its program stopped) first.
  robotarm::WebcamController webcam(webcamLaunch);
  robotarm::WebcamController::setQmlInstance(&webcam);
#ifdef ROBOTARM_HAS_SENSOR_BRIDGE
  QObject::connect(&sensorBridge, &robotarm::SensorBridgeServer::previewFrame, &webcam,
                   &robotarm::WebcamController::showPreview);
#endif
  if (!parser.isSet(noWebcamOption) && !parser.isSet(screenshotOption)) {
    webcam.start();
  }

  QQmlApplicationEngine engine;
  engine.addImportPath(QStringLiteral("qrc:/qt/qml")); // the default from Qt 6.5 on
  engine.rootContext()->setContextProperty("launchOptions", launchOptions);

  const QString qmlDirPath(APP_QML_DIR);
  const QUrl mainQmlUrl = QUrl::fromLocalFile(qmlDirPath + "/Main.qml");

  engine.load(mainQmlUrl);
  if (engine.rootObjects().isEmpty()) return -1;

  // Save a .qml file (from any editor) and the window reloads in place.
  robotarm::QmlHotReloader hotReloader(&engine, qmlDirPath, mainQmlUrl);

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
