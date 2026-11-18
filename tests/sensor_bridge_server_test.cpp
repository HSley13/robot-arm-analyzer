// Verifies SensorBridgeServer: each command reaches the right controller
// call, bad input (malformed JSON, unknown types, missing or non-finite
// numbers) is ignored rather than acted on, the hello describes the
// robot, and the whole path works over a real local WebSocket.
#include <QCoreApplication>
#include <QEventLoop>
#include <QJsonArray>
#include <QJsonDocument>
#include <QDir>
#include <QFile>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QThread>
#include <QTimer>
#include <QWebSocket>

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "arm_controller.hpp"
#include "hand_controller.hpp"
#include "sensor_bridge_server.hpp"
#include "session_recorder.hpp"
#include "task_controller.hpp"

using robotarm::ArmController;
using robotarm::HandController;
using robotarm::SensorBridgeServer;
using robotarm::SessionRecorder;
using robotarm::TaskController;

namespace {

struct Rig {
  ArmController arm{std::string(PROJECT_ROOT_DIR) + "/config/humanoid_arm.json", "/nonexistent/weights.json"};
  HandController hand{std::string(PROJECT_ROOT_DIR) + "/config/humanoid_hand.json"};
  TaskController task{&arm, &hand};
  SensorBridgeServer bridge{&task, &arm, 0};
};

// Runs the event loop until `done()` or ~2 s pass.
template <typename Done> bool waitFor(Done done) {
  QEventLoop loop;
  QTimer poll;
  QObject::connect(&poll, &QTimer::timeout, &loop, [&]() {
    if (done()) {
      loop.quit();
    }
  });
  poll.start(5);
  QTimer::singleShot(2000, &loop, &QEventLoop::quit);
  loop.exec();
  return done();
}

} // namespace

TEST(SensorBridge, ReachSetsAHandGoal) {
  Rig rig;
  EXPECT_TRUE(rig.bridge.handleMessage(R"({"type": "reach", "x": 0.5, "y": 0.9, "z": 0.3})").isEmpty());
  EXPECT_TRUE(rig.task.reaching());
}

TEST(SensorBridge, ReachCanAlsoPointTheHand) {
  Rig rig;
  rig.bridge.handleMessage(R"({"type": "reach", "x": 0.5, "y": 0.9, "z": 0.3, "pitch": -0.4})");
  EXPECT_TRUE(rig.task.reaching());
}

TEST(SensorBridge, AReachWithABadPitchIsIgnored) {
  Rig rig;
  rig.bridge.handleMessage(R"({"type": "reach", "x": 0.5, "y": 0.9, "z": 0.3, "pitch": "up"})");
  EXPECT_FALSE(rig.task.reaching());
}

TEST(SensorBridge, ReachCanAlsoTwistTheHand) {
  Rig rig;
  rig.bridge.handleMessage(R"({"type": "reach", "x": 0.5, "y": 0.9, "z": 0.3, "pitch": 0.0, "roll": 0.8})");
  for (int i = 0; i < 600 && rig.task.reaching(); i++) {
    rig.task.advance(1.0 / 60.0);
  }
  EXPECT_NEAR(rig.task.handRollDegrees(), 0.8 * 180.0 / M_PI, 0.1);
}

TEST(SensorBridge, GripClosesTheHand) {
  Rig rig;
  rig.bridge.handleMessage(R"({"type": "grip", "amount": 0.6})");
  EXPECT_DOUBLE_EQ(rig.task.grip(), 0.6);
}

TEST(SensorBridge, HoldStopsTheReach) {
  Rig rig;
  rig.bridge.handleMessage(R"({"type": "reach", "x": 0.5, "y": 0.9, "z": 0.3})");
  rig.bridge.handleMessage(R"({"type": "hold"})");
  EXPECT_FALSE(rig.task.reaching());
}

TEST(SensorBridge, AssistCanBeSwitchedOffAndOn) {
  Rig rig;
  ASSERT_TRUE(rig.task.assist());
  rig.bridge.handleMessage(R"({"type": "assist", "on": false})");
  EXPECT_FALSE(rig.task.assist());
  rig.bridge.handleMessage(R"({"type": "assist", "on": "yes"})"); // not a bool: ignored
  EXPECT_FALSE(rig.task.assist());
  rig.bridge.handleMessage(R"({"type": "assist", "on": true})");
  EXPECT_TRUE(rig.task.assist());
}

TEST(SensorBridge, ReachIsAssisted) {
  // Heading for a bottle over the wire: the app's guess follows.
  Rig rig;
  const int bottle = rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  for (int i = 0; i <= 30; i++) {
    const double t = i / 30.0;
    rig.bridge.handleMessage(QStringLiteral(R"({"type": "reach", "x": %1, "y": %2, "z": 0.30})")
                                 .arg(0.40 + 0.12 * t)
                                 .arg(0.65 - 0.53 * t));
    rig.task.advance(1.0 / 30.0);
    QThread::msleep(33); // the guess runs on real time between messages
  }
  EXPECT_EQ(rig.task.intentId(), bottle);
}

TEST(SensorBridge, PreviewPicturesArePassedOn) {
  Rig rig;
  QByteArrayList frames;
  QObject::connect(&rig.bridge, &SensorBridgeServer::previewFrame,
                   [&](const QByteArray &jpeg) { frames.append(jpeg); });
  rig.bridge.handleMessage(R"({"type": "preview", "jpeg": "anBlZyE="})");
  rig.bridge.handleMessage(R"({"type": "preview", "jpeg": 42})"); // not a picture: ignored
  EXPECT_EQ(frames, QByteArrayList({"anBlZyE="}));
}

TEST(SensorBridge, LeftHandMessagesDriveTheLeftArm) {
  Rig rig;
  ArmController left{std::string(PROJECT_ROOT_DIR) + "/config/humanoid_arm.json", "/nonexistent/weights.json"};
  HandController leftHand{std::string(PROJECT_ROOT_DIR) + "/config/humanoid_hand.json"};
  EXPECT_TRUE(rig.bridge.hello()["left_shoulder"].isNull()); // none yet
  rig.task.setLeftArm(&left, &leftHand);
  EXPECT_DOUBLE_EQ(rig.bridge.hello()["left_shoulder"].toArray()[2].toDouble(), -0.20);

  const std::vector<double> before = left.jointAnglesRadians();
  rig.bridge.handleMessage(R"({"type": "reach", "hand": "left", "x": 0.4, "y": 1.0, "z": -0.3, "roll": 0.5})");
  for (int i = 0; i < 120; i++) {
    rig.task.advance(1.0 / 60.0);
  }
  EXPECT_NE(left.jointAnglesRadians(), before);
  EXPECT_GT(rig.task.leftHandRollDegrees(), 1.0);
  rig.bridge.handleMessage(R"({"type": "grip", "hand": "left", "amount": 0.4})");
  EXPECT_DOUBLE_EQ(rig.task.leftGrip(), 0.4);
  EXPECT_DOUBLE_EQ(rig.task.grip(), 0.0); // the right hand untouched

  rig.bridge.handleMessage(R"({"type": "reach", "hand": "left", "x": 0.4, "y": 0.9, "z": -0.3})");
  rig.bridge.handleMessage(R"({"type": "hold"})"); // stops both
  EXPECT_FALSE(rig.task.reaching());
  rig.bridge.handleMessage(R"({"type": "grip", "hand": "middle", "amount": 0.9})"); // no such hand
  EXPECT_DOUBLE_EQ(rig.task.leftGrip(), 0.4);
  EXPECT_DOUBLE_EQ(rig.task.grip(), 0.0);
}

TEST(SensorBridge, PingIsAnswered) {
  Rig rig;
  const QJsonObject reply = QJsonDocument::fromJson(rig.bridge.handleMessage(R"({"type": "ping"})").toUtf8()).object();
  EXPECT_EQ(reply["type"].toString(), QString("pong"));
}

TEST(SensorBridge, BadInputDoesNothing) {
  Rig rig;
  for (const char *text : {"not json at all", "[1, 2, 3]", R"({"type": "dance"})", R"({"type": "reach", "x": 0.5})",
                           R"({"type": "reach", "x": "far", "y": 1, "z": 0})", R"({"type": "grip"})",
                           R"({"type": "grip", "amount": "lots"})"}) {
    EXPECT_TRUE(rig.bridge.handleMessage(text).isEmpty()) << text;
  }
  EXPECT_FALSE(rig.task.reaching());
  EXPECT_DOUBLE_EQ(rig.task.grip(), 0.0);
}

TEST(SensorBridge, HelloDescribesTheRobot) {
  Rig rig;
  const QJsonObject hello = rig.bridge.hello();
  EXPECT_EQ(hello["type"].toString(), QString("hello"));
  EXPECT_EQ(hello["robot"].toString(), QString("humanoid"));
  const QJsonArray shoulder = hello["shoulder"].toArray();
  ASSERT_EQ(shoulder.size(), 3);
  EXPECT_NEAR(shoulder[1].toDouble(), 0.80 + 0.45, 1e-12); // standing hip + hip to shoulder
  EXPECT_NEAR(shoulder[2].toDouble(), 0.20, 1e-12);        // the right shoulder's side offset
  // Upper arm + forearm + hand link, measured joint to joint (the links'
  // small offsets make it a few mm more than 0.30 + 0.27 + 0.09).
  EXPECT_NEAR(hello["reach"].toDouble(), 0.66, 0.01);
  EXPECT_NEAR(hello["hand"].toDouble(), 0.095, 0.01); // the hand link alone
}

TEST(SensorBridge, WorksOverARealWebSocket) {
  Rig rig;
  ASSERT_TRUE(rig.bridge.start());
  ASSERT_NE(rig.bridge.port(), 0);

  QWebSocket client;
  QStringList received;
  QObject::connect(&client, &QWebSocket::textMessageReceived, [&](const QString &text) { received.append(text); });
  client.open(QUrl(QStringLiteral("ws://127.0.0.1:%1").arg(rig.bridge.port())));

  // Connecting: the server says hello first.
  ASSERT_TRUE(waitFor([&]() { return !received.isEmpty(); }));
  EXPECT_EQ(QJsonDocument::fromJson(received.first().toUtf8()).object()["type"].toString(), QString("hello"));
  EXPECT_EQ(rig.bridge.clientCount(), 1);

  // A command over the wire reaches the controller.
  client.sendTextMessage(R"({"type": "grip", "amount": 0.4})");
  EXPECT_TRUE(waitFor([&]() { return rig.task.grip() == 0.4; }));

  client.close();
  EXPECT_TRUE(waitFor([&]() { return rig.bridge.clientCount() == 0; }));
}

TEST(SensorBridge, RecordsEachSessionOverARealWebSocket) {
  Rig rig;
  QTemporaryDir directory;
  SessionRecorder recorder(&rig.task, &rig.arm, directory.path());
  rig.bridge.setRecorder(&recorder);
  ASSERT_TRUE(rig.bridge.start());

  QWebSocket client;
  bool hello = false;
  QObject::connect(&client, &QWebSocket::textMessageReceived, [&](const QString &) { hello = true; });
  client.open(QUrl(QStringLiteral("ws://127.0.0.1:%1").arg(rig.bridge.port())));
  ASSERT_TRUE(waitFor([&]() { return hello; }));
  EXPECT_TRUE(recorder.recording());

  client.sendTextMessage(R"({"type": "grip", "amount": 0.4})");
  client.sendTextMessage(R"({"type": "ping"})");         // not a command: not recorded
  client.sendTextMessage(R"({"type": "grip", "amount": "x"})"); // ignored: not recorded
  client.sendTextMessage(R"({"type": "reach", "x": 0.5, "y": 0.9, "z": 0.3})");
  ASSERT_TRUE(waitFor([&]() { return rig.task.reaching(); }));
  client.close();
  ASSERT_TRUE(waitFor([&]() { return !recorder.recording(); }));

  QFile file(recorder.path());
  ASSERT_TRUE(file.open(QIODevice::ReadOnly | QIODevice::Text));
  QStringList types;
  while (!file.atEnd()) {
    const QJsonObject line = QJsonDocument::fromJson(file.readLine()).object();
    types.append(line["type"].toString() + (line.contains("message")
                                                ? ":" + line["message"].toObject()["type"].toString()
                                                : QString()));
  }
  EXPECT_EQ(types, QStringList({"start", "input:grip", "input:reach", "end"}));
}

int main(int argc, char **argv) {
  QCoreApplication app(argc, argv); // sockets and timers need one
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
