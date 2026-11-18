// Verifies WebcamController: finding the program, starting and stopping
// it, its output becoming the status, and preview frames - with small
// stand-in python3 programs instead of the real (camera) one.
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QStandardPaths>
#include <QTemporaryDir>
#include <QThread>

#include <functional>

#include <gtest/gtest.h>

#include "webcam_controller.hpp"

using robotarm::WebcamController;

namespace {

bool waitFor(const std::function<bool()> &done, int ms = 5000) {
  QElapsedTimer clock;
  clock.start();
  while (!done() && clock.elapsed() < ms) {
    QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    QThread::msleep(5);
  }
  return done();
}

WebcamController::Launch python(const QString &code) {
  WebcamController::Launch launch;
  launch.program = QStandardPaths::findExecutable(QStringLiteral("python3"));
  launch.arguments = {QStringLiteral("-u"), QStringLiteral("-c"), code};
  return launch;
}

void touch(const QString &path) {
  QDir().mkpath(QFileInfo(path).path());
  QFile file(path);
  ASSERT_TRUE(file.open(QIODevice::WriteOnly));
}

} // namespace

TEST(WebcamController, UnavailableWithoutTheProgram) {
  QTemporaryDir empty;
  WebcamController webcam(WebcamController::defaultLaunch(empty.path(), 8765));
  EXPECT_FALSE(webcam.available());
  EXPECT_TRUE(webcam.status().contains("not found"));
  webcam.start();
  EXPECT_FALSE(webcam.running());
}

TEST(WebcamController, PrefersTheFoldersOwnVirtualEnvironment) {
  QTemporaryDir dir;
  touch(dir.filePath("webcam_teleop.py"));
  touch(dir.filePath(".venv/bin/python3"));
  const WebcamController::Launch launch = WebcamController::defaultLaunch(dir.path(), 9001);
  EXPECT_EQ(launch.program, dir.filePath(".venv/bin/python3"));
  EXPECT_EQ(launch.workingDirectory, dir.path());
  EXPECT_TRUE(launch.arguments.contains("9001"));
  EXPECT_TRUE(launch.arguments.contains("--stream-preview"));
}

TEST(WebcamController, RunsTheProgramShowsWhatItSaysAndStopsIt) {
  if (QStandardPaths::findExecutable(QStringLiteral("python3")).isEmpty()) {
    GTEST_SKIP() << "no python3";
  }
  WebcamController webcam(python("import time\nprint('waiting for the robot app')\nprint('connected')\ntime.sleep(30)"));
  webcam.start();
  ASSERT_TRUE(waitFor([&]() { return webcam.status() == "connected"; })) << webcam.status().toStdString();
  EXPECT_TRUE(webcam.running());
  webcam.stop();
  EXPECT_FALSE(webcam.running());
  EXPECT_TRUE(waitFor([&]() { return webcam.status() == "Webcam off"; })) << webcam.status().toStdString();
}

TEST(WebcamController, SaysWhyTheProgramStopped) {
  if (QStandardPaths::findExecutable(QStringLiteral("python3")).isEmpty()) {
    GTEST_SKIP() << "no python3";
  }
  WebcamController webcam(python("import sys\nprint(\"can't open camera 0\")\nsys.exit(1)"));
  webcam.setRunning(true);
  ASSERT_TRUE(waitFor([&]() { return !webcam.running(); }));
  EXPECT_TRUE(waitFor([&]() { return webcam.status() == "Webcam stopped: can't open camera 0"; }))
      << webcam.status().toStdString();
}

TEST(WebcamController, PreviewFramesOnlyWhileRunning) {
  if (QStandardPaths::findExecutable(QStringLiteral("python3")).isEmpty()) {
    GTEST_SKIP() << "no python3";
  }
  WebcamController webcam(python("import time\ntime.sleep(30)"));
  webcam.showPreview("AAAA");
  EXPECT_TRUE(webcam.preview().isEmpty());
  webcam.start();
  ASSERT_TRUE(waitFor([&]() { return webcam.running(); }));
  webcam.showPreview("AAAA");
  EXPECT_EQ(webcam.preview(), QString("data:image/jpeg;base64,AAAA"));
  webcam.stop();
  EXPECT_TRUE(waitFor([&]() { return webcam.preview().isEmpty(); }));
}

int main(int argc, char **argv) {
  QCoreApplication app(argc, argv); // processes need one
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
