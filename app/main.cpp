#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QUrl>

#ifndef APP_QML_DIR
#error "APP_QML_DIR must be defined by the build"
#endif

int main(int argc, char *argv[]) {
  QGuiApplication app(argc, argv);

  QQmlApplicationEngine engine;
  const QUrl mainQmlUrl = QUrl::fromLocalFile(QString(APP_QML_DIR) + "/Main.qml");
  engine.load(mainQmlUrl);
  if (engine.rootObjects().isEmpty()) return -1;

  return app.exec();
}
