#pragma once

#include <QByteArray>
#include <QObject>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QtQml/qqmlregistration.h>

class QQmlEngine;
class QJSEngine;

namespace robotarm {

// Webcam control from inside the app: runs the tracking program
// (tools/webcam_teleop/webcam_teleop.py - MediaPipe lives in Python) as a
// child process that connects back to the app's sensor bridge, shows its
// camera preview in the window, and turns it on and off.
//
// The program sends its preview (the mirrored camera image with the
// tracked arm drawn on it) over the bridge as small JPEG frames;
// SensorBridgeServer hands them to showPreview().
class WebcamController : public QObject {
  Q_OBJECT
  // QML: `import Humanoid.Backend`, then WebcamController.<property>.
  QML_ELEMENT
  QML_SINGLETON
  // False if webcam control can't run here (no sensor bridge, or the
  // program isn't there) - `status` says why.
  Q_PROPERTY(bool available READ available CONSTANT)
  Q_PROPERTY(bool running READ running WRITE setRunning NOTIFY runningChanged)
  // What the program last said ("connected", "can't open camera"...).
  Q_PROPERTY(QString status READ status NOTIFY statusChanged)
  // The latest preview frame as a data: URL for an Image, "" for none.
  Q_PROPERTY(QString preview READ preview NOTIFY previewChanged)

public:
  // How to start the program; an empty program = not available.
  struct Launch {
    QString program;
    QStringList arguments;
    QString workingDirectory;
    QString unavailableReason; // when program is empty
  };

  // The tracking script in `script_dir` (webcam_teleop.py), connecting to
  // the bridge on `port`, run by the folder's own virtual environment
  // (.venv) if it has one, else by python3 from the PATH.
  static Launch defaultLaunch(const QString &script_dir, quint16 port);

  static WebcamController *create(QQmlEngine *, QJSEngine *);
  static void setQmlInstance(WebcamController *instance) { s_qmlInstance = instance; }

  explicit WebcamController(Launch launch, QObject *parent = nullptr);
  ~WebcamController() override;

  bool available() const { return !_launch.program.isEmpty(); }
  bool running() const { return _process.state() != QProcess::NotRunning; }
  QString status() const { return _status; }
  QString preview() const { return _preview; }

public slots:
  void start();
  void stop();
  void setRunning(bool on) { on ? start() : stop(); }
  // One preview frame, base64 JPEG (as the program sends it).
  void showPreview(const QByteArray &jpeg_base64);

signals:
  void runningChanged();
  void statusChanged();
  void previewChanged();

private:
  void setStatus(const QString &status);
  void readOutput();

  Launch _launch;
  QProcess _process;
  bool _stopping = false;
  QString _status;
  QString _preview;

  static inline WebcamController *s_qmlInstance = nullptr;
};

} // namespace robotarm
