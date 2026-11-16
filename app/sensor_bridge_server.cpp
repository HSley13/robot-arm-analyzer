#include "sensor_bridge_server.hpp"

#include <QDebug>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QWebSocket>
#include <QWebSocketServer>

#include <cmath>

#include "arm_controller.hpp"
#include "task_controller.hpp"

namespace robotarm {

namespace {

QString toText(const QJsonObject &object) {
  return QString::fromUtf8(QJsonDocument(object).toJson(QJsonDocument::Compact));
}

// The message's numbers, or false if any is missing or not a finite number
// (NaN from a confused tracker must never reach the planner).
bool readNumbers(const QJsonObject &message, std::initializer_list<const char *> keys, double *out) {
  for (const char *key : keys) {
    const QJsonValue value = message.value(QLatin1String(key));
    if (!value.isDouble() || !std::isfinite(value.toDouble())) {
      return false;
    }
    *out = value.toDouble();
    out++;
  }
  return true;
}

} // namespace

SensorBridgeServer::SensorBridgeServer(TaskController *task, ArmController *arm, quint16 port, QObject *parent)
    : QObject(parent), _task(task), _arm(arm), _port(port),
      _server(new QWebSocketServer(QStringLiteral("robotarm-sensor-bridge"), QWebSocketServer::NonSecureMode, this)) {
  connect(_server, &QWebSocketServer::newConnection, this, &SensorBridgeServer::onNewConnection);
}

SensorBridgeServer::~SensorBridgeServer() {
  qDeleteAll(_clients);
}

bool SensorBridgeServer::start(bool lan) {
  if (!_server->listen(lan ? QHostAddress::Any : QHostAddress::LocalHost, _port)) {
    qWarning() << "SensorBridgeServer: can't listen on port" << _port << "-" << _server->errorString();
    return false;
  }
  qInfo().noquote() << "SensorBridgeServer: listening on" << (lan ? "all interfaces," : "localhost,") << "port"
                    << port();
  return true;
}

quint16 SensorBridgeServer::port() const {
  return _server->isListening() ? _server->serverPort() : _port;
}

bool SensorBridgeServer::isListening() const {
  return _server->isListening();
}

QJsonObject SensorBridgeServer::hello() const {
  // Where the right shoulder is when the robot stands facing forward - the
  // fixed reference a client maps a person's shoulder onto.
  const QVariantMap body = _task->body();
  const double shoulderY = body["standingHipHeight"].toDouble() + body["hipToShoulder"].toDouble();
  double reach = 0.0;
  double hand = 0.0; // the last link: wrist to the hand's end-effector point
  for (const QVariant &length : _arm->linkLengths()) {
    reach += std::abs(length.toDouble());
    hand = std::abs(length.toDouble());
  }
  return {{"type", "hello"},
          {"robot", _arm->armStyle().isEmpty() ? QStringLiteral("humanoid") : _arm->armStyle()},
          {"shoulder", QJsonArray{0.0, shoulderY, body["shoulderLateral"].toDouble()}},
          {"reach", reach},
          {"hand", hand}};
}

QString SensorBridgeServer::handleMessage(const QString &text) {
  QJsonParseError error;
  const QJsonDocument document = QJsonDocument::fromJson(text.toUtf8(), &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) {
    qWarning() << "SensorBridgeServer: ignoring malformed message:" << error.errorString();
    return {};
  }
  const QJsonObject message = document.object();
  const QString type = message.value(QStringLiteral("type")).toString();

  if (type == QStringLiteral("reach")) {
    double xyz[3];
    if (!readNumbers(message, {"x", "y", "z"}, xyz)) {
      qWarning() << "SensorBridgeServer: reach needs numbers x, y, z";
      return {};
    }
    double pitch = 0.0;
    if (!message.contains(QStringLiteral("pitch"))) {
      _task->reachTo(xyz[0], xyz[1], xyz[2]);
    } else if (readNumbers(message, {"pitch"}, &pitch)) {
      _task->reachToPointing(xyz[0], xyz[1], xyz[2], pitch);
    } else {
      qWarning() << "SensorBridgeServer: reach's pitch must be a number";
      return {};
    }
    if (message.contains(QStringLiteral("roll"))) {
      double roll = 0.0;
      if (readNumbers(message, {"roll"}, &roll)) {
        _task->setHandRoll(roll);
      } else {
        qWarning() << "SensorBridgeServer: reach's roll must be a number";
      }
    }
  } else if (type == QStringLiteral("grip")) {
    double amount = 0.0;
    if (!readNumbers(message, {"amount"}, &amount)) {
      qWarning() << "SensorBridgeServer: grip needs a number amount";
      return {};
    }
    _task->setGrip(amount);
  } else if (type == QStringLiteral("hold")) {
    _task->cancelReach();
  } else if (type == QStringLiteral("ping")) {
    return toText({{"type", "pong"}});
  } else {
    qWarning() << "SensorBridgeServer: ignoring unknown message type:" << type;
  }
  return {};
}

void SensorBridgeServer::onNewConnection() {
  QWebSocket *socket = _server->nextPendingConnection();
  if (socket == nullptr) {
    return;
  }
  connect(socket, &QWebSocket::textMessageReceived, this, [this, socket](const QString &text) {
    const QString reply = handleMessage(text);
    if (!reply.isEmpty()) {
      socket->sendTextMessage(reply);
    }
  });
  connect(socket, &QWebSocket::disconnected, this, &SensorBridgeServer::onClientDisconnected);
  _clients.append(socket);
  socket->sendTextMessage(toText(hello()));
  qInfo().noquote() << "SensorBridgeServer: client connected from" << socket->peerAddress().toString();
  emit clientCountChanged();
}

void SensorBridgeServer::onClientDisconnected() {
  auto *socket = qobject_cast<QWebSocket *>(sender());
  if (socket == nullptr) {
    return;
  }
  _clients.removeAll(socket);
  socket->deleteLater();
  // Whatever it was steering toward, stop there.
  _task->cancelReach();
  qInfo() << "SensorBridgeServer: client disconnected";
  emit clientCountChanged();
}

} // namespace robotarm
