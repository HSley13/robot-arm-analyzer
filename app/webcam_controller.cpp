#include "webcam_controller.hpp"

#include <QDir>
#include <QFileInfo>
#include <QStandardPaths>

#include "qml_singleton.hpp"

namespace robotarm {

WebcamController *WebcamController::create(QQmlEngine *, QJSEngine *) { return existingQmlSingleton(s_qmlInstance); }

WebcamController::Launch WebcamController::defaultLaunch(const QString &script_dir, quint16 port) {
  Launch launch;
  const QDir dir(script_dir);
  if (!QFileInfo::exists(dir.filePath(QStringLiteral("webcam_teleop.py")))) {
    launch.unavailableReason = QStringLiteral("Webcam program not found in %1").arg(script_dir);
    return launch;
  }
  const QString venv = dir.filePath(QStringLiteral(".venv/bin/python3"));
  launch.program = QFileInfo::exists(venv) ? venv : QStandardPaths::findExecutable(QStringLiteral("python3"));
  if (launch.program.isEmpty()) {
    launch.unavailableReason = QStringLiteral("No python3 found to run the webcam program");
    return launch;
  }
  // -u: unbuffered, so its messages show up as they're printed. No window
  // of its own: the preview comes into the app.
  launch.arguments = {QStringLiteral("-u"),           QStringLiteral("webcam_teleop.py"),
                      QStringLiteral("--port"),       QString::number(port),
                      QStringLiteral("--no-preview"), QStringLiteral("--stream-preview")};
  launch.workingDirectory = script_dir;
  return launch;
}

WebcamController::WebcamController(Launch launch, QObject *parent) : QObject(parent), _launch(std::move(launch)) {
  _status = available() ? QStringLiteral("Webcam off") : _launch.unavailableReason;
  _process.setProcessChannelMode(QProcess::MergedChannels);
  connect(&_process, &QProcess::readyReadStandardOutput, this, &WebcamController::readOutput);
  connect(&_process, &QProcess::stateChanged, this, &WebcamController::runningChanged);
  connect(&_process, &QProcess::errorOccurred, this, [this](QProcess::ProcessError error) {
    if (error == QProcess::FailedToStart) {
      setStatus(QStringLiteral("Can't start the webcam program: %1").arg(_process.errorString()));
    }
  });
  connect(&_process, &QProcess::finished, this, [this](int code, QProcess::ExitStatus how) {
    if (!_preview.isEmpty()) {
      _preview.clear();
      emit previewChanged();
    }
    if (_stopping) {
      setStatus(QStringLiteral("Webcam off"));
    } else if (how == QProcess::CrashExit || code != 0) {
      // Its last words usually say why (no camera, no MediaPipe...).
      setStatus(QStringLiteral("Webcam stopped: %1").arg(_status));
    } else {
      setStatus(QStringLiteral("Webcam off"));
    }
    _stopping = false;
  });
}

WebcamController::~WebcamController() {
  disconnect(&_process, nullptr, this, nullptr);
  stop();
}

void WebcamController::start() {
  if (!available() || running()) {
    return;
  }
  _stopping = false;
  setStatus(QStringLiteral("Starting the webcam..."));
  _process.setWorkingDirectory(_launch.workingDirectory);
  _process.start(_launch.program, _launch.arguments);
}

void WebcamController::stop() {
  if (!running()) {
    return;
  }
  _stopping = true;
  _process.terminate();
  if (!_process.waitForFinished(1500)) {
    _process.kill();
    _process.waitForFinished(500);
  }
}

void WebcamController::showPreview(const QByteArray &jpeg_base64) {
  if (!running()) {
    return; // a last frame arriving after it was turned off
  }
  _preview = QStringLiteral("data:image/jpeg;base64,") + QString::fromLatin1(jpeg_base64);
  emit previewChanged();
}

void WebcamController::readOutput() {
  // Each line it prints is its latest news; the last one is the status.
  QString last;
  while (_process.canReadLine()) {
    const QString line = QString::fromUtf8(_process.readLine()).trimmed();
    if (!line.isEmpty()) {
      last = line;
    }
  }
  if (!last.isEmpty()) {
    setStatus(last);
  }
}

void WebcamController::setStatus(const QString &status) {
  if (status == _status) {
    return;
  }
  _status = status;
  emit statusChanged();
}

} // namespace robotarm
