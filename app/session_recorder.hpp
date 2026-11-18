#pragma once

#include <QElapsedTimer>
#include <QFile>
#include <QJsonObject>
#include <QObject>
#include <QString>

namespace robotarm {

class ArmController;
class TaskController;

// Writes down every live-control session (a webcam connected to the
// sensor bridge), for learning later what people reach for and how - the
// data the assist's next stage will be trained on.
//
// One file per session, `session-<date>-<time>.jsonl` in the directory
// given: one JSON object per line, each with "t" (seconds since the
// session started) and "type":
//
//   "start"    the scene: robot kind, objects (id, type, mass, x, y, z),
//              tables; plus "time" (ISO 8601) and "version" (the format's)
//   "input"    one command from the client, as received ("message"), with
//              what the robot made of it: the body pose after it ("pose":
//              yaw, hip height, hip bend, hand roll, arm joints - radians
//              and metres), the assist's guess ("intent": id, confidence,
//              strength) and what's held ("held", -1 = nothing)
//   "objects"  the objects again, whenever they change (a grab, a release)
//   "end"      the session is over
//
// Nothing about the person is recorded: no images, only the commands the
// client sent.
class SessionRecorder : public QObject {
  Q_OBJECT

public:
  static constexpr int kFormatVersion = 1;

  // The controllers are not owned; they must outlive the recorder.
  SessionRecorder(TaskController *task, ArmController *arm, QString directory, QObject *parent = nullptr);
  ~SessionRecorder() override;

  // Starts a new session file (ending any running one). False (and logs
  // why) if the directory or file can't be made.
  bool start();
  void stop();
  bool recording() const { return _file.isOpen(); }
  // The running (or last) session's file.
  QString path() const { return _file.fileName(); }

  // One client command, as handled - call after the controllers acted on it.
  void record(const QJsonObject &message);

private:
  void write(QJsonObject line);
  void writeObjects();
  QJsonObject pose() const;

  TaskController *_task;
  ArmController *_arm;
  QString _directory;
  QFile _file;
  QElapsedTimer _clock;
};

} // namespace robotarm
