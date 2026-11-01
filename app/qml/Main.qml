import QtQuick
import QtQuick.Window
import QtQuick.Layouts

Window {
    // Sized to the actual screen rather than a fixed guess - the
    // sidebar holds a lot (every joint's dial/gauge/limit/length/mass,
    // payload, AI toggle, payload-capacity scan, config file), so
    // using as much of whatever display this actually runs on is what
    // gets the most of it on screen without scrolling, on any machine.
    width: Math.min(1600, Screen.desktopAvailableWidth * 0.92)
    height: Math.min(1050, Screen.desktopAvailableHeight * 0.92)
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
            Layout.preferredWidth: 400
            Layout.fillHeight: true
        }
    }
}
