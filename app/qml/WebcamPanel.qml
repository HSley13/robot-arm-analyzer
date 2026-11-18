import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import Humanoid.Backend

// Webcam control, in a corner of the 3D view: on / off, the assist, the
// camera behind the robot, and the tracker's own picture of you (mirrored,
// with your tracked arm drawn on it).
Rectangle {
    id: root

    // The 3D view puts its camera behind the robot when asked.
    signal behindViewRequested()
    signal normalViewRequested()
    property bool behindView: false

    width: 300
    height: column.implicitHeight + 16
    radius: 8
    color: Qt.rgba(0.055, 0.063, 0.13, 0.88)
    border.color: WebcamController.running ? "#17e0ff" : "#3a3f5c"

    ColumnLayout {
        id: column
        anchors.fill: parent
        anchors.margins: 8
        spacing: 6

        RowLayout {
            Layout.fillWidth: true
            Label {
                text: "Webcam"
                color: "white"
                font.bold: true
                Layout.fillWidth: true
            }
            Switch {
                enabled: WebcamController.available
                checked: WebcamController.running
                onToggled: WebcamController.running = checked
            }
        }

        Image {
            visible: WebcamController.running
            Layout.fillWidth: true
            Layout.preferredHeight: width * 0.5625
            fillMode: Image.PreserveAspectFit
            source: WebcamController.preview
            cache: false
            asynchronous: false

            Label {
                visible: WebcamController.preview === ""
                anchors.centerIn: parent
                color: "#9aa0c0"
                text: "Waiting for the camera..."
            }
        }

        Label {
            Layout.fillWidth: true
            color: "#9aa0c0"
            font.pixelSize: 11
            wrapMode: Text.Wrap
            text: WebcamController.status
        }

        RowLayout {
            visible: WebcamController.running
            Layout.fillWidth: true
            CheckBox {
                text: "Assist"
                checked: TaskController.assist
                onToggled: TaskController.assist = checked
                ToolTip.visible: hovered
                ToolTip.text: "Guess what you reach for, pull the hand onto it, finish the grab"
            }
            Item { Layout.fillWidth: true }
            Button {
                text: root.behindView ? "Free view" : "Behind view"
                ToolTip.visible: hovered
                ToolTip.text: "Look over the robot's shoulder: your right is its right"
                onClicked: root.behindView ? root.normalViewRequested() : root.behindViewRequested()
            }
        }
    }
}
