#include "sensor_bridge_server.hpp"

#include <QDebug>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QWebSocket>
#include <QWebSocketServer>

#include <cmath>

#include "arm_controller.hpp"
#include "session_recorder.hpp"
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

constexpr qsizetype kMaxPreviewSize = 2 * 1024 * 1024; // base64 characters

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
          // Where the left one is, if the robot has a working left arm.
          {"left_shoulder", _task->leftArm() != nullptr
                                ? QJsonValue(QJsonArray{0.0, shoulderY, -body["shoulderLateral"].toDouble()})
                                : QJsonValue()},
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
  // Which hand a reach or grip is for: "right" (the default) or "left".
  const QString hand = message.value(QStringLiteral("hand")).toString(QStringLiteral("right"));
  if (hand != QStringLiteral("right") && hand != QStringLiteral("left")) {
    qWarning() << "SensorBridgeServer: hand must be \"right\" or \"left\", not" << hand;
    return {};
  }
  const bool left = hand == QStringLiteral("left");

  if (type == QStringLiteral("reach")) {
    double xyz[3];
    if (!readNumbers(message, {"x", "y", "z"}, xyz)) {
      qWarning() << "SensorBridgeServer: reach needs numbers x, y, z";
      return {};
    }
    // Live control: the assisted reach (a NaN pitch = the hand left free).
    double pitch = std::nan("");
    if (message.contains(QStringLiteral("pitch")) && !readNumbers(message, {"pitch"}, &pitch)) {
      qWarning() << "SensorBridgeServer: reach's pitch must be a number";
      return {};
    }
    left ? _task->reachLeft(xyz[0], xyz[1], xyz[2], pitch) : _task->guidedReach(xyz[0], xyz[1], xyz[2], pitch);
    if (message.contains(QStringLiteral("roll"))) {
      double roll = 0.0;
      if (readNumbers(message, {"roll"}, &roll)) {
        left ? _task->setLeftHandRoll(roll) : _task->setHandRoll(roll);
      } else {
        qWarning() << "SensorBridgeServer: reach's roll must be a number";
      }
    }
    record(message);
  } else if (type == QStringLiteral("grip")) {
    double amount = 0.0;
    if (!readNumbers(message, {"amount"}, &amount)) {
      qWarning() << "SensorBridgeServer: grip needs a number amount";
      return {};
    }
    left ? _task->setLeftGrip(amount) : _task->setGrip(amount);
    record(message);
  } else if (type == QStringLiteral("assist")) {
    const QJsonValue on = message.value(QStringLiteral("on"));
    if (!on.isBool()) {
      qWarning() << "SensorBridgeServer: assist needs a true / false on";
      return {};
    }
    _task->setAssist(on.toBool());
    record(message);
  } else if (type == QStringLiteral("preview")) {
    // Not a command - a picture for the window - so not recorded either.
    const QJsonValue jpeg = message.value(QStringLiteral("jpeg"));
    if (!jpeg.isString() || jpeg.toString().size() > kMaxPreviewSize) {
      qWarning() << "SensorBridgeServer: preview needs a base64 jpeg (under 2 MB)";
      return {};
    }
    emit previewFrame(jpeg.toString().toLatin1());
  } else if (type == QStringLiteral("hold")) {
    _task->cancelReach();
    _task->cancelLeftReach();
    record(message);
  } else if (type == QStringLiteral("ping")) {
    return toText({{"type", "pong"}});
  } else {
    qWarning() << "SensorBridgeServer: ignoring unknown message type:" << type;
  }
  return {};
}

void SensorBridgeServer::record(const QJsonObject &message) {
  if (_recorder != nullptr) {
    _recorder->record(message);
  }
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
  if (_recorder != nullptr && _clients.size() == 1) {
    _recorder->start(); // a session: from the first client connecting to the last one leaving
  }
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
  _task->cancelLeftReach();
  if (_recorder != nullptr && _clients.isEmpty()) {
    _recorder->stop();
  }
  qInfo() << "SensorBridgeServer: client disconnected";
  emit clientCountChanged();
}

} // namespace robotarm
