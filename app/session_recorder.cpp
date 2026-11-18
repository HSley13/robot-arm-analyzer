#include "session_recorder.hpp"

#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>

#include "arm_controller.hpp"
#include "task_controller.hpp"

namespace robotarm {

namespace {

constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

QJsonArray objectsOf(const TaskController &task) {
  QJsonArray objects;
  for (const QVariant &value : task.objects()) {
    const QVariantMap object = value.toMap();
    objects.append(QJsonObject{{"id", object["id"].toInt()},
                               {"type", object["type"].toString()},
                               {"mass", object["mass"].toDouble()},
                               {"x", object["x"].toDouble()},
                               {"y", object["y"].toDouble()},
                               {"z", object["z"].toDouble()},
                               {"grip_height", object["gripHeight"].toDouble()},
                               {"held", object["held"].toBool()}});
  }
  return objects;
}

} // namespace

SessionRecorder::SessionRecorder(TaskController *task, ArmController *arm, QString directory, QObject *parent)
    : QObject(parent), _task(task), _arm(arm), _directory(std::move(directory)) {
  // A grab or a release moves an object: the outcome a guess is judged by.
  connect(_task, &TaskController::objectsChanged, this, [this]() {
    if (recording()) {
      writeObjects();
    }
  });
}

SessionRecorder::~SessionRecorder() { stop(); }

bool SessionRecorder::start() {
  stop();
  if (!QDir().mkpath(_directory)) {
    qWarning() << "SessionRecorder: can't make" << _directory;
    return false;
  }
  const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd-HHmmss"));
  QString name = QStringLiteral("session-%1.jsonl").arg(stamp);
  for (int n = 2; QDir(_directory).exists(name); n++) {
    name = QStringLiteral("session-%1-%2.jsonl").arg(stamp).arg(n); // two in the same second
  }
  _file.setFileName(QDir(_directory).filePath(name));
  if (!_file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    qWarning() << "SessionRecorder: can't write" << _file.fileName() << "-" << _file.errorString();
    return false;
  }
  _clock.start();

  QJsonArray tables;
  for (const QVariant &value : _task->tables()) {
    tables.append(QJsonObject::fromVariantMap(value.toMap()));
  }
  write({{"type", "start"},
         {"version", kFormatVersion},
         {"time", QDateTime::currentDateTime().toString(Qt::ISODateWithMs)},
         {"robot", _arm->armStyle().isEmpty() ? QStringLiteral("humanoid") : _arm->armStyle()},
         {"objects", objectsOf(*_task)},
         {"tables", tables}});
  qInfo().noquote() << "SessionRecorder: recording to" << _file.fileName();
  return true;
}

void SessionRecorder::stop() {
  if (!recording()) {
    return;
  }
  write({{"type", "end"}});
  _file.close();
}

void SessionRecorder::record(const QJsonObject &message) {
  if (!recording()) {
    return;
  }
  write({{"type", "input"},
         {"message", message},
         {"pose", pose()},
         {"intent", QJsonObject{{"id", _task->intentId()},
                                {"confidence", _task->intentConfidence()},
                                {"strength", _task->assistStrength()}}},
         {"held", _task->heldId()},
         {"held_left", _task->leftHeldId()}});
}

void SessionRecorder::writeObjects() { write({{"type", "objects"}, {"objects", objectsOf(*_task)}}); }

QJsonObject SessionRecorder::pose() const {
  QJsonArray arm;
  for (const double angle : _arm->jointAnglesRadians()) {
    arm.append(angle);
  }
  QJsonObject pose{{"yaw", _task->waistYawDegrees() * kDegreesToRadians},
                   {"hip_height", _task->hipHeight()},
                   {"hip_bend", _task->hipBendDegrees() * kDegreesToRadians},
                   {"hand_roll", _task->handRollDegrees() * kDegreesToRadians},
                   {"arm", arm}};
  if (const ArmController *left = _task->leftArm()) {
    QJsonArray left_arm;
    for (const double angle : left->jointAnglesRadians()) {
      left_arm.append(angle);
    }
    pose.insert(QStringLiteral("left_arm"), left_arm);
    pose.insert(QStringLiteral("left_hand_roll"), _task->leftHandRollDegrees() * kDegreesToRadians);
  }
  return pose;
}

void SessionRecorder::write(QJsonObject line) {
  line.insert(QStringLiteral("t"), static_cast<double>(_clock.nsecsElapsed()) / 1e9);
  _file.write(QJsonDocument(line).toJson(QJsonDocument::Compact));
  _file.write("\n");
  _file.flush(); // a crash still leaves the session up to here
}

} // namespace robotarm
