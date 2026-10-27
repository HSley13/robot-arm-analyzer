import QtQuick
import QtQuick.Window
import QtQuick.Layouts

Window {
    width: 1100
    height: 700
    visible: true
    title: "Robot Arm Load Analyzer - " + armController.armName
            + (armController.aiMode ? " [AI]" : " [Exact]")

    RowLayout {
        anchors.fill: parent
        spacing: 0

        ArmView3D {
            Layout.fillWidth: true
            Layout.fillHeight: true
        }

        ControlsPanel {
            Layout.preferredWidth: 320
            Layout.fillHeight: true
        }
    }
}
