#include "sensor_bridge_server.hpp"

#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>

namespace robotarm {

SensorBridgeServer::SensorBridgeServer(ArmController *controller, quint16 port, QObject *parent)
    : QObject(parent), _controller(controller), _port(port),
      _server(QStringLiteral("robotarm-sensor-bridge"), QWebSocketServer::NonSecureMode, this) {
  connect(&_server, &QWebSocketServer::newConnection, this, &SensorBridgeServer::onNewConnection);
}

SensorBridgeServer::~SensorBridgeServer() {
  qDeleteAll(_clients);
}

bool SensorBridgeServer::start() {
  if (!_server.listen(QHostAddress::Any, _port)) {
    qWarning() << "SensorBridgeServer: failed to listen on port" << _port << "-" << _server.errorString();
    return false;
  }
  qInfo() << "SensorBridgeServer: listening on port" << _port;
  return true;
}

bool SensorBridgeServer::isListening() const {
  return _server.isListening();
}

void SensorBridgeServer::onNewConnection() {
  QWebSocket *socket = _server.nextPendingConnection();
  if (!socket) return;

  connect(socket, &QWebSocket::textMessageReceived, this, &SensorBridgeServer::handleMessage);
  connect(socket, &QWebSocket::disconnected, this, &SensorBridgeServer::onClientDisconnected);
  _clients.append(socket);
  qInfo() << "SensorBridgeServer: client connected from" << socket->peerAddress().toString();
}

void SensorBridgeServer::onClientDisconnected() {
  QWebSocket *socket = qobject_cast<QWebSocket *>(sender());
  if (!socket) return;
  _clients.removeAll(socket);
  socket->deleteLater();
}

void SensorBridgeServer::handleMessage(const QString &text) {
  QJsonParseError error;
  const QJsonDocument doc = QJsonDocument::fromJson(text.toUtf8(), &error);
  if (error.error != QJsonParseError::NoError || !doc.isObject()) {
    qWarning() << "SensorBridgeServer: ignoring malformed message:" << error.errorString();
    return;
  }

  const QJsonObject obj = doc.object();
  const QString type = obj.value("type").toString();

  if (type == QStringLiteral("jointAngle")) {
    if (!obj.contains("joint") || !obj.contains("deg")) {
      qWarning() << "SensorBridgeServer: jointAngle message missing joint/deg";
      return;
    }
    _controller->setJointAngleDegrees(obj.value("joint").toInt(), obj.value("deg").toDouble());
  } else if (type == QStringLiteral("moveTarget")) {
    if (!obj.contains("x") || !obj.contains("y")) {
      qWarning() << "SensorBridgeServer: moveTarget message missing x/y";
      return;
    }
    _controller->moveToTarget(obj.value("x").toDouble(), obj.value("y").toDouble());
  } else if (type == QStringLiteral("gripper")) {
    if (!obj.contains("open")) {
      qWarning() << "SensorBridgeServer: gripper message missing open";
      return;
    }
    _controller->setGripperOpen(obj.value("open").toBool());
  } else if (type == QStringLiteral("ping")) {
    // Connectivity check only - no state change.
  } else {
    qWarning() << "SensorBridgeServer: ignoring unknown message type:" << type;
  }
}

} // namespace robotarm
