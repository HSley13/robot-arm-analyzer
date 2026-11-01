#include <QDir>
#include <QFileSystemWatcher>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QString>
#include <QStringList>
#include <QUrl>

#include "arm_controller.hpp"

#ifndef APP_QML_DIR
#error "APP_QML_DIR must be defined by the build"
#endif
#ifndef PROJECT_ROOT_DIR
#error "PROJECT_ROOT_DIR must be defined by the build"
#endif

int main(int argc, char *argv[]) {
  QGuiApplication app(argc, argv);

  robotarm::ArmController controller(std::string(PROJECT_ROOT_DIR) + "/config/three_dof_arm.json",
                                     std::string(PROJECT_ROOT_DIR) + "/ml/models/three_dof_arm.surrogate.json",
                                     std::string(PROJECT_ROOT_DIR) +
                                         "/ml/models/three_dof_arm_dynamic.surrogate.json");

  QQmlApplicationEngine engine;
  engine.rootContext()->setContextProperty("armController", &controller);

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

  return app.exec();
}
