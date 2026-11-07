#include <memory>
#include <string>

#include <QCoreApplication>
#include <QQmlComponent>
#include <QQmlEngine>
#include <QtQml/qqmlextensionplugin.h>

#include <gtest/gtest.h>

#include "arm_controller.hpp"
#include "hand_controller.hpp"
#include "task_controller.hpp"

// The module lives in a static library: pull in its type registrations.
Q_IMPORT_QML_PLUGIN(Humanoid_BackendPlugin)

using robotarm::ArmController;
using robotarm::HandController;
using robotarm::TaskController;

namespace {

struct Controllers {
  ArmController arm{std::string(PROJECT_ROOT_DIR) + "/config/humanoid_arm.json", "/nonexistent/weights.json"};
  HandController hand{std::string(PROJECT_ROOT_DIR) + "/config/humanoid_hand.json"};
  TaskController task{&arm, &hand};

  Controllers() {
    ArmController::setQmlInstance(&arm);
    HandController::setQmlInstance(&hand);
    TaskController::setQmlInstance(&task);
  }
};

// Compiles and creates `qml`; fails the test with the engine's errors.
std::unique_ptr<QObject> create(QQmlEngine &engine, const QByteArray &qml) {
  QQmlComponent component(&engine);
  component.setData(qml, QUrl(QStringLiteral("inline.qml")));
  std::unique_ptr<QObject> object(component.create());
  EXPECT_NE(object, nullptr) << component.errorString().toStdString();
  return object;
}

} // namespace

TEST(QmlModule, ControllersAreTheSingletonsMainMade) {
  Controllers controllers;
  QQmlEngine engine;
  engine.addImportPath(QStringLiteral("qrc:/qt/qml"));

  const auto object = create(engine, R"(
import QtQml
import Humanoid.Backend
QtObject {
    // Properties: read through the type name, kept up to date by NOTIFY.
    property int joints: ArmController.jointCount
    property string gesture: HandController.currentGesture
    property int tables: TaskController.tables.length
    // Methods: called like any JS function.
    Component.onCompleted: TaskController.addTable("coffee table", 0.05, 1.0)
}
)");
  ASSERT_NE(object, nullptr);
  EXPECT_EQ(object->property("joints").toInt(), 3);
  EXPECT_EQ(object->property("gesture").toString(), controllers.hand.currentGesture());
  // The call reached the C++ object main() would have made...
  EXPECT_EQ(controllers.task.tables().size(), 2);
  // ...and the binding followed its signal.
  EXPECT_EQ(object->property("tables").toInt(), 2);
}

TEST(QmlModule, QmlCanListenToControllerSignals) {
  Controllers controllers;
  QQmlEngine engine;
  engine.addImportPath(QStringLiteral("qrc:/qt/qml"));

  const auto object = create(engine, R"(
import QtQml
import Humanoid.Backend
QtObject {
    property int tableChanges: 0
    property Connections watcher: Connections {
        target: TaskController
        function onTablesChanged() { tableChanges++ }
    }
}
)");
  ASSERT_NE(object, nullptr);
  controllers.task.addTable("counter", 0.6, 0.55);
  EXPECT_EQ(object->property("tableChanges").toInt(), 1);
}

int main(int argc, char **argv) {
  QCoreApplication app(argc, argv); // the QML engine needs one
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
