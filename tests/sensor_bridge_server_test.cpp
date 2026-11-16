// Verifies SensorBridgeServer: each command reaches the right controller
// call, bad input (malformed JSON, unknown types, missing or non-finite
// numbers) is ignored rather than acted on, the hello describes the
// robot, and the whole path works over a real local WebSocket.
#include <QCoreApplication>
#include <QEventLoop>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTimer>
#include <QWebSocket>

#include <gtest/gtest.h>

#include "arm_controller.hpp"
#include "hand_controller.hpp"
#include "sensor_bridge_server.hpp"
#include "task_controller.hpp"

using robotarm::ArmController;
using robotarm::HandController;
using robotarm::SensorBridgeServer;
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

int main(int argc, char **argv) {
  QCoreApplication app(argc, argv); // sockets and timers need one
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
