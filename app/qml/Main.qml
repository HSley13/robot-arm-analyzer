import QtQuick
import QtQuick.Window
import QtQuick.Layouts

Window {
    // Sized to the actual screen rather than a fixed guess - between
    // the sidebar and the bottom bar there's a lot to show (every
    // joint's dial/gauge/limit/length/mass, payload, AI toggle,
    // Static/Dynamic mode, the payload-capacity scan, config file),
    // so using as much of whatever display this actually runs on is
    // what gets the most of it on screen without scrolling, on any
    // machine.
    width: Math.min(1600, Screen.desktopAvailableWidth * 0.92)
    height: Math.min(1050, Screen.desktopAvailableHeight * 0.92)
    visible: true
    title: "Robot Arm Load Analyzer - " + armController.armName
            + (armController.aiMode ? " [AI]" : " [Exact]")

    // Sidebar (right) + bottom bar (full width, below) - everything
    // that's "about one specific joint" (angle/load/limit/length/
    // mass, and Dynamic mode's velocity/acceleration/breakdown) lives
    // in the bottom bar, laid out one card per joint side by side;
    // everything else (reach targeting, payload, AI/Static-Dynamic
    // mode, payload-capacity scan, config file) lives in the sidebar.
    // Deliberately no tabs anywhere - every setting is visible all at
    // once, split across two panels instead of hidden behind one.
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
                Layout.preferredWidth: 400
                Layout.fillHeight: true
            }
        }

        BottomBar {
            Layout.fillWidth: true
            // No Layout.preferredHeight set - BottomBar sizes itself
            // to its own content (see its implicitHeight binding), so
            // it grows a little when Dynamic mode adds velocity/
            // acceleration/breakdown controls to every joint card,
            // and the 3D view above it gets whatever height remains.
        }
    }
}
