#include "qml_hot_reloader.hpp"

#include <QDir>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickWindow>

#include <cstdio>

namespace robotarm {

QmlHotReloader::QmlHotReloader(QQmlApplicationEngine *engine, const QString &qmlDir, const QUrl &mainQml,
                               QObject *parent)
    : QObject(parent), _engine(engine), _qmlDir(qmlDir), _mainQml(mainQml) {
  _debounce.setSingleShot(true);
  _debounce.setInterval(150);
  connect(&_debounce, &QTimer::timeout, this, &QmlHotReloader::reload);

  // Any event on a file or the folder: make sure everything is (still)
  // watched, then (re)start the countdown to one reload.
  const auto changed = [this](const QString &) {
    watchFiles();
    _debounce.start();
  };
  connect(&_watcher, &QFileSystemWatcher::fileChanged, this, changed);
  connect(&_watcher, &QFileSystemWatcher::directoryChanged, this, changed);

  _watcher.addPath(_qmlDir);
  watchFiles();
}

void QmlHotReloader::watchFiles() {
  const QDir dir(_qmlDir);
  const QStringList watched = _watcher.files();
  for (const QString &name : dir.entryList({QStringLiteral("*.qml")}, QDir::Files)) {
    const QString path = dir.filePath(name);
    // A file replaced by a rename is a new file to the watcher: add it
    // again. (addPath() on one already watched would just warn.)
    if (!watched.contains(path)) {
      _watcher.addPath(path);
    }
  }
}

void QmlHotReloader::reload() {
  // Remember where the window was, then destroy it - right now, not
  // later: the component cache only lets go of components nothing uses
  // any more, so a still-alive old window would get its stale Main.qml
  // back instead of the edited one.
  QRect geometry;
  for (QObject *root : _engine->rootObjects()) {
    if (auto *window = qobject_cast<QQuickWindow *>(root)) {
      geometry = window->geometry();
    }
  }
  // Between windows (or if the edit doesn't compile) there's no window at
  // all - don't let that quit the app.
  QGuiApplication::setQuitOnLastWindowClosed(false);
  const QList<QObject *> oldRoots = _engine->rootObjects();
  for (QObject *root : oldRoots) {
    delete root; // the engine drops it from rootObjects() when it's destroyed
  }

  // Forget the compiled components so the edited files are read again.
  _engine->clearComponentCache();
  _engine->load(_mainQml);

  const QList<QObject *> roots = _engine->rootObjects();
  if (roots.isEmpty()) {
    // A syntax error (the engine has printed it). Keep watching: the
    // window comes back on the next save that compiles.
    std::fprintf(stderr, "[hot reload] failed - fix the error above and save again\n");
    return;
  }
  if (auto *window = qobject_cast<QQuickWindow *>(roots.constLast()); window != nullptr && geometry.isValid()) {
    window->setGeometry(geometry);
  }
  QGuiApplication::setQuitOnLastWindowClosed(true);
  std::fprintf(stderr, "[hot reload] reloaded\n");
}

} // namespace robotarm
