#pragma once

#include <QFileSystemWatcher>
#include <QObject>
#include <QString>
#include <QTimer>
#include <QUrl>

class QQmlApplicationEngine;

namespace robotarm {

// Reloads the QML UI whenever a .qml file in `qmlDir` is saved - edit in
// Neovim (or anything else), :w, and the window updates in place.
//
// What makes it work with real editors:
//   * Editors often save by writing a new file and renaming it over the
//     old one (Neovim's default for many files). A file watcher follows
//     the old file and goes quiet after that, so the files are re-added
//     on every change, and the directory itself is watched too (which
//     also catches brand-new .qml files).
//   * One save can produce several change events (write, rename, chmod),
//     so reloads are debounced: one reload ~150 ms after the last event.
//   * The old window is replaced by the new one at the same place and
//     size - not a new window stacked on top each time.
//   * Saving something that doesn't compile prints the error and the
//     window disappears (the app keeps running); the next good save
//     brings it back.
// C++ state (the controllers: arm pose, objects, tables...) lives outside
// QML, so it survives a reload; only QML-side state (camera, open menus)
// starts over.
class QmlHotReloader : public QObject {
  Q_OBJECT

public:
  QmlHotReloader(QQmlApplicationEngine *engine, const QString &qmlDir, const QUrl &mainQml,
                 QObject *parent = nullptr);

private:
  void watchFiles();
  void reload();

  QQmlApplicationEngine *_engine;
  QString _qmlDir;
  QUrl _mainQml;
  QFileSystemWatcher _watcher;
  QTimer _debounce;
};

} // namespace robotarm
