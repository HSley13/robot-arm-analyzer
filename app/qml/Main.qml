import QtQuick
import QtQuick.Window
import QtQuick.Layouts
import Humanoid.Backend

Window {
    width: launchOptions.width !== undefined ? launchOptions.width : Math.min(1600, Screen.desktopAvailableWidth * 0.92)
    height: launchOptions.height !== undefined ? launchOptions.height : Math.min(1050, Screen.desktopAvailableHeight * 0.92)
    visible: true
    color: "#0e1021"
    title: "Humanoid Arm Load Analyzer - " + ArmController.armName
            + (ArmController.aiMode ? " [AI]" : " [Exact]")

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 0

            ArmView3D {
                Layout.fillWidth: true
                Layout.fillHeight: true
            }

            ControlsPanel {
                Layout.preferredWidth: bottomBar.implicitHeight * 1.1
                Layout.fillHeight: true
            }
        }

        BottomBar {
            id: bottomBar
            Layout.fillWidth: true
        }
    }
}
