import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    color: "#16161e"

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 16

        Label {
            text: armController.armName
            font.pixelSize: 18
            font.bold: true
            color: "white"
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
        }

        Repeater {
            model: armController.jointCount

            ColumnLayout {
                id: jointRow
                required property int index
                Layout.fillWidth: true
                spacing: 4

                Label {
                    color: "white"
                    text: "Joint " + (jointRow.index + 1) + ": "
                        + armController.jointAnglesDegrees[jointRow.index].toFixed(1) + "°"
                }

                Slider {
                    Layout.fillWidth: true
                    from: -180
                    to: 180
                    value: armController.jointAnglesDegrees[jointRow.index]
                    onMoved: armController.setJointAngleDegrees(jointRow.index, value)
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8

                    Rectangle {
                        implicitWidth: 14
                        implicitHeight: 14
                        radius: 3
                        color: armController.jointColors[jointRow.index]
                    }

                    Label {
                        color: "white"
                        Layout.fillWidth: true
                        text: "τ = " + armController.jointTorques[jointRow.index].toFixed(2)
                            + " / " + armController.jointTorqueLimits[jointRow.index].toFixed(1) + " N·m"
                    }
                }
            }
        }

        ColumnLayout {
            Layout.fillWidth: true
            spacing: 4

            Label {
                color: "white"
                text: "Payload: " + armController.payloadKg.toFixed(2) + " kg"
            }

            Slider {
                Layout.fillWidth: true
                from: 0
                to: 5
                value: armController.payloadKg
                onMoved: armController.payloadKg = value
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 8

            Label {
                color: "white"
                opacity: armController.aiMode ? 0.5 : 1.0
                text: "Exact"
            }

            Switch {
                checked: armController.aiMode
                enabled: armController.aiModeAvailable
                onToggled: armController.aiMode = checked
            }

            Label {
                color: "white"
                opacity: armController.aiMode ? 1.0 : 0.5
                text: "AI" + (armController.aiModeAvailable ? "" : " (no trained model)")
            }
        }

        Item { Layout.fillHeight: true }
    }
}
