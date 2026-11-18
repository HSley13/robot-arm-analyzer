// Verifies SessionRecorder: a session file starts with the scene, logs
// each command with what the robot made of it, notes grabs, and ends.
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "arm_controller.hpp"
#include "hand_controller.hpp"
#include "session_recorder.hpp"
#include "task_controller.hpp"

using robotarm::ArmController;
using robotarm::HandController;
using robotarm::SessionRecorder;
using robotarm::TaskController;

namespace {

struct Rig {
  QTemporaryDir directory;
  ArmController arm{std::string(PROJECT_ROOT_DIR) + "/config/humanoid_arm.json", "/nonexistent/weights.json"};
  HandController hand{std::string(PROJECT_ROOT_DIR) + "/config/humanoid_hand.json"};
  TaskController task{&arm, &hand};
  SessionRecorder recorder{&task, &arm, directory.filePath("sessions")};
};

std::vector<QJsonObject> linesOf(const QString &path) {
  QFile file(path);
  EXPECT_TRUE(file.open(QIODevice::ReadOnly | QIODevice::Text)) << path.toStdString();
  std::vector<QJsonObject> lines;
  while (!file.atEnd()) {
    const QJsonDocument line = QJsonDocument::fromJson(file.readLine());
    EXPECT_TRUE(line.isObject());
    lines.push_back(line.object());
  }
  return lines;
}

} // namespace

TEST(SessionRecorder, NothingIsWrittenUntilStarted) {
  Rig rig;
  EXPECT_FALSE(rig.recorder.recording());
  rig.recorder.record({{"type", "grip"}, {"amount", 1.0}});
  EXPECT_FALSE(QDir(rig.directory.filePath("sessions")).exists());
}

TEST(SessionRecorder, ASessionIsTheSceneThenTheCommandsThenAnEnd) {
  Rig rig;
  const int bottle = rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  ASSERT_TRUE(rig.recorder.start());
  EXPECT_TRUE(rig.recorder.path().startsWith(rig.directory.filePath("sessions/session-")));

  const QJsonObject reach{{"type", "reach"}, {"x", 0.5}, {"y", 0.9}, {"z", 0.3}};
  rig.task.guidedReach(0.5, 0.9, 0.3, std::nan(""), 1.0 / 30.0);
  rig.recorder.record(reach);
  rig.recorder.stop();
  EXPECT_FALSE(rig.recorder.recording());

  const std::vector<QJsonObject> lines = linesOf(rig.recorder.path());
  ASSERT_EQ(lines.size(), 3u);
  const QJsonObject &start = lines[0];
  EXPECT_EQ(start["type"].toString(), QString("start"));
  EXPECT_EQ(start["version"].toInt(), SessionRecorder::kFormatVersion);
  EXPECT_EQ(start["robot"].toString(), QString("humanoid"));
  ASSERT_EQ(start["objects"].toArray().size(), 1);
  const QJsonObject object = start["objects"].toArray()[0].toObject();
  EXPECT_EQ(object["id"].toInt(), bottle);
  EXPECT_EQ(object["type"].toString(), QString("bottle"));
  EXPECT_NEAR(object["x"].toDouble(), 0.62, 1e-12);
  EXPECT_GT(object["grip_height"].toDouble(), 0.0);
  EXPECT_EQ(start["tables"].toArray().size(), 1);

  const QJsonObject &input = lines[1];
  EXPECT_EQ(input["type"].toString(), QString("input"));
  EXPECT_EQ(input["message"].toObject(), reach);
  EXPECT_EQ(input["pose"].toObject()["arm"].toArray().size(), 3);
  EXPECT_TRUE(input["intent"].toObject().contains("confidence"));
  EXPECT_EQ(input["held"].toInt(), -1);
  EXPECT_EQ(input["held_left"].toInt(), -1);
  EXPECT_FALSE(input["pose"].toObject().contains("left_arm")); // no left arm in this rig
  EXPECT_GE(input["t"].toDouble(), start["t"].toDouble());

  EXPECT_EQ(lines[2]["type"].toString(), QString("end"));
}

TEST(SessionRecorder, AGrabIsWrittenDown) {
  Rig rig;
  const int bottle = rig.task.addObject("bottle", 0.0, 0.62, 0.30);
  rig.task.reachTo(0.62, 0.10, 0.30);
  for (int i = 0; i < 60 * 10 && rig.task.reaching(); i++) {
    rig.task.advance(1.0 / 60.0);
  }
  ASSERT_TRUE(rig.recorder.start());
  rig.task.setGrip(1.0);
  ASSERT_EQ(rig.task.heldId(), bottle);
  rig.recorder.record({{"type", "grip"}, {"amount", 1.0}});
  rig.recorder.stop();

  const std::vector<QJsonObject> lines = linesOf(rig.recorder.path());
  ASSERT_EQ(lines.size(), 4u); // start, objects (the grab), input, end
  EXPECT_EQ(lines[1]["type"].toString(), QString("objects"));
  EXPECT_TRUE(lines[1]["objects"].toArray()[0].toObject()["held"].toBool());
  EXPECT_EQ(lines[2]["held"].toInt(), bottle);
}

TEST(SessionRecorder, EachSessionGetsItsOwnFile) {
  Rig rig;
  ASSERT_TRUE(rig.recorder.start());
  const QString first = rig.recorder.path();
  ASSERT_TRUE(rig.recorder.start()); // ends the first
  EXPECT_NE(rig.recorder.path(), first);
  EXPECT_EQ(linesOf(first).back()["type"].toString(), QString("end"));
}
