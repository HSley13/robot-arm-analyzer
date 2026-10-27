import QtQuick
import QtQuick.Window

Window {
    width: 640
    height: 480
    visible: true
    title: "Robot Arm Load Analyzer"

    Rectangle {
        anchors.fill: parent
        color: "#202020"

        Text {
            anchors.centerIn: parent
            text: "Qt/QML scaffolding OK"
            color: "white"
            font.pixelSize: 24
        }
    }
}
