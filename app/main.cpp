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

  // Phase 1 loads one hardcoded arm - the config/AI-surrogate pairing is
  // what makes the Exact/AI toggle meaningful; swapping arms from the UI
  // is a stretch goal, not required here.
  robotarm::ArmController controller(std::string(PROJECT_ROOT_DIR) + "/config/two_link_planar.json",
                                     std::string(PROJECT_ROOT_DIR) + "/ml/models/two_link_planar.surrogate.json");

  QQmlApplicationEngine engine;
  engine.rootContext()->setContextProperty("armController", &controller);

  const QString qmlDirPath(APP_QML_DIR);
  const QUrl mainQmlUrl = QUrl::fromLocalFile(qmlDirPath + "/Main.qml");

  // Hot reload: watch every .qml file on disk and reload the whole tree
  // on change, so editing in nvim and saving is enough to see the result
  // without a rebuild+rerun. Loses transient UI state (e.g. a slider's
  // mid-drag position) on reload, but that's a fine trade for not needing
  // Qt Creator's Live Preview.
  QFileSystemWatcher qmlWatcher;
  const QDir qmlDir(qmlDirPath);
  for (const QString &fileName : qmlDir.entryList(QStringList() << "*.qml", QDir::Files)) {
    qmlWatcher.addPath(qmlDir.filePath(fileName));
  }
  QObject::connect(&qmlWatcher, &QFileSystemWatcher::fileChanged, &engine,
                   [&engine, &mainQmlUrl, &qmlWatcher](const QString &path) {
                     // Some editors replace the file (write a new inode)
                     // rather than write in place, which drops it from
                     // the watch list - re-add it so the next edit still
                     // triggers a reload.
                     if (!qmlWatcher.files().contains(path)) qmlWatcher.addPath(path);
                     engine.clearComponentCache();
                     engine.load(mainQmlUrl);
                   });

  engine.load(mainQmlUrl);
  if (engine.rootObjects().isEmpty()) return -1;

  return app.exec();
}
