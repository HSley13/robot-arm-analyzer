#pragma once

#include <QObject>
#include <QString>
#include <QWebSocket>
#include <QWebSocketServer>

#include "arm_controller.hpp"

namespace robotarm {

// Listens for JSON command messages over WebSocket and forwards them to an
// ArmController - the entry point for external hardware (an IMU wand, a
// camera-based gripper controller, ...) to drive the arm over the network
// instead of through the QML UI. Accepted messages, one JSON object per
// WebSocket text frame:
//
//   {"type": "jointAngle", "joint": <int>, "deg": <number>}
//   {"type": "moveTarget", "x": <number>, "y": <number>}
//   {"type": "gripper", "open": <bool>}
//   {"type": "ping"}
//
// Unknown types and malformed messages are logged and ignored, never fatal -
// a flaky sensor link should degrade to "nothing happens," not crash the app.
// handleMessage() is exposed separately from the socket plumbing so it can be
// unit-tested without a real WebSocket connection.
class SensorBridgeServer : public QObject {
  Q_OBJECT

public:
  // controller is not owned; it must outlive this server.
  SensorBridgeServer(ArmController *controller, quint16 port, QObject *parent = nullptr);
  ~SensorBridgeServer() override;

  // Starts listening on the port given at construction, on all network
  // interfaces (not just localhost) so a device elsewhere on the LAN can
  // connect. Returns false (and logs why) if the port could not be bound.
  bool start();
  quint16 port() const { return _port; }
  bool isListening() const;

  // Parses one JSON command and applies it to the controller. Public so
  // tests can exercise the dispatch logic directly.
  void handleMessage(const QString &text);

private slots:
  void onNewConnection();
  void onClientDisconnected();

private:
  ArmController *_controller;
  quint16 _port;
  QWebSocketServer _server;
  QList<QWebSocket *> _clients;
};

} // namespace robotarm
