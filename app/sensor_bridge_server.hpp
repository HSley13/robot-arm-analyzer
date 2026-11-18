#pragma once

#include <QJsonObject>
#include <QList>
#include <QObject>
#include <QString>

class QWebSocket;
class QWebSocketServer;

namespace robotarm {

class ArmController;
class SessionRecorder;
class TaskController;

// The robot's door to the outside world: a WebSocket server that takes
// JSON commands from another program - the webcam teleop client in
// tools/webcam_teleop/, an IMU wand, a script - and turns them into calls
// on the controllers. One JSON object per text frame:
//
//   {"type": "reach", "x": <m>, "y": <m>, "z": <m>}  move the hand there
//                                                     (TaskController::guidedReach:
//                                                     planned, smooth, safe, and
//                                                     helped onto the object
//                                                     you're going for)
//     + "pitch": <rad>                                 optional: and point it
//                                                     that far up from level
//     + "roll": <rad>                                  optional: and turn the
//                                                     hand about its own axis,
//                                                     + palm up (setHandRoll)
//   {"type": "grip", "amount": <0..1>}                close / open the hand,
//                                                     grabbing and letting go
//                                                     (TaskController::setGrip)
//   {"type": "assist", "on": <bool>}                  that help on / off
//                                                     (TaskController::assist)
//   {"type": "hold"}                                  stop where it is
//   {"type": "ping"}                                  answered with {"type": "pong"}
//
// On connecting, a client gets {"type": "hello", ...} describing the robot
// (its standing shoulder position, the arm's reach and how much of that
// is the hand) so it can scale a person's arm to the robot's. Malformed messages and unknown types are
// logged and ignored, never fatal: a flaky sensor link should degrade to
// "nothing happens", not crash the app.
class SensorBridgeServer : public QObject {
  Q_OBJECT

public:
  // The controllers are not owned; they must outlive this server.
  SensorBridgeServer(TaskController *task, ArmController *arm, quint16 port, QObject *parent = nullptr);
  ~SensorBridgeServer() override;

  // Starts listening. Only on this computer unless `lan` is true - there
  // is no authentication, so anyone who can reach the port can move the
  // robot. Returns false (and logs why) if the port can't be bound.
  bool start(bool lan = false);
  bool isListening() const;
  // The port it listens on (the one asked for, or the free one picked
  // when started with port 0).
  quint16 port() const;
  int clientCount() const { return static_cast<int>(_clients.size()); }

  // Applies one JSON command; returns the reply to send back (empty for
  // none). Public so tests can drive it without a real connection.
  QString handleMessage(const QString &text);
  // What a client is told when it connects.
  QJsonObject hello() const;
  // Writes every session down (not owned; nullptr = don't): it starts
  // when the first client connects, logs each command the controllers
  // acted on (not pings or ignored ones), and ends when the last leaves.
  void setRecorder(SessionRecorder *recorder) { _recorder = recorder; }

signals:
  void clientCountChanged();

private:
  void onNewConnection();
  void onClientDisconnected();
  void record(const QJsonObject &message);

  TaskController *_task;
  ArmController *_arm;
  quint16 _port;
  QWebSocketServer *_server;
  QList<QWebSocket *> _clients;
  SessionRecorder *_recorder = nullptr;
};

} // namespace robotarm
