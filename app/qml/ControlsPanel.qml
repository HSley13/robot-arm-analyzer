import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts

// Sidebar: legend, reach targeting, payload + hand, and config file I/O.
// Per-joint controls, AI toggle, and Static/Dynamic mode live in
// BottomBar.qml instead.
Rectangle {
    id: root
    color: "#0e1021"

    // file:// URL -> local path (loadConfig()/saveConfig() take a path).
    function urlToLocalPath(fileUrl) {
        let path = fileUrl.toString()
        if (path.startsWith("file://")) path = path.substring(7)
        return decodeURIComponent(path)
    }

    ScrollView {
        anchors.fill: parent
        contentWidth: availableWidth

        ColumnLayout {
            width: parent.width
            anchors.margins: 10
            spacing: 8

            Label {
                text: armController.armName
                font.pixelSize: 18
                font.bold: true
                color: "white"
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }

            Rectangle {
                Layout.fillWidth: true
                color: "#171a2b"
                radius: 12
                border.color: "#262a40"
                border.width: 1
                implicitHeight: legendRow.implicitHeight + 16

                RowLayout {
                    id: legendRow
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 10

                    Repeater {
                        model: [
                            { color: "#2ecc71", label: "Safe" },
                            { color: "#ffbf00", label: "Caution" },
                            { color: "#ff4d4f", label: "Overload" }
                        ]

                        RowLayout {
                            required property var modelData
                            spacing: 5

                            Rectangle {
                                implicitWidth: 10
                                implicitHeight: 10
                                radius: 5
                                color: parent.modelData.color
                            }
                            Label {
                                color: "#c7cbd6"
                                font.pixelSize: 11
                                text: parent.modelData.label
                            }
                        }
                    }

                    Item { Layout.fillWidth: true }
                }
            }

            Rectangle {
                Layout.fillWidth: true
                color: "#171a2b"
                radius: 14
                border.color: "#262a40"
                border.width: 1
                implicitHeight: moveToTargetColumn.implicitHeight + 16

                ColumnLayout {
                    id: moveToTargetColumn
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 5

                    Label {
                        color: "white"
                        font.bold: true
                        text: "Move to target"
                    }

                    ReachPad {
                        Layout.alignment: Qt.AlignHCenter
                        Layout.preferredWidth: 150
                        Layout.preferredHeight: 150
                        readonly property real reachDiameterMeters: armController.canvasCenter.x * 2
                        rangeMeters: Math.max(reachDiameterMeters * 1.8, 0.5)
                        maxReachMeters: reachDiameterMeters
                        markerXMeters: armController.jointPositions.length > 0
                            ? armController.jointPositions[armController.jointPositions.length - 1].x : 0
                        markerYMeters: armController.jointPositions.length > 0
                            ? armController.jointPositions[armController.jointPositions.length - 1].y : 0
                        onTargetMoved: (x, y) => armController.moveToTarget(x, y)
                    }
                }
            }

            Rectangle {
                Layout.fillWidth: true
                color: "#171a2b"
                radius: 14
                border.color: "#262a40"
                border.width: 1
                implicitHeight: payloadColumn.implicitHeight + 16

                ColumnLayout {
                    id: payloadColumn
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 5

                    Label {
                        color: "white"
                        font.bold: true
                        text: "Payload"
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 5

                        Slider {
                            Layout.fillWidth: true
                            from: 0
                            to: 30
                            value: armController.payloadKg
                            onMoved: armController.payloadKg = value
                        }

                        TextField {
                            implicitWidth: 70
                            text: armController.payloadKg.toFixed(1)
                            validator: DoubleValidator { bottom: 0 }
                            onEditingFinished: armController.payloadKg = parseFloat(text)
                        }

                        Label {
                            color: "#9aa0ab"
                            text: "kg"
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 5

                        Label {
                            color: "white"
                            opacity: armController.gripperOpen ? 1.0 : 0.5
                            text: "Released"
                        }

                        Switch {
                            checked: !armController.gripperOpen
                            onToggled: armController.gripperOpen = !checked
                        }

                        Label {
                            color: "white"
                            opacity: armController.gripperOpen ? 0.5 : 1.0
                            text: "Gripping"
                        }
                    }
                }
            }

            // Hand gestures. Gripping/releasing the payload above also
            // drives the hand (power grip / open), these override it.
            Rectangle {
                Layout.fillWidth: true
                color: "#171a2b"
                radius: 14
                border.color: "#262a40"
                border.width: 1
                implicitHeight: handColumn.implicitHeight + 16

                ColumnLayout {
                    id: handColumn
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 6

                    RowLayout {
                        Layout.fillWidth: true
                        Label {
                            color: "white"
                            font.bold: true
                            text: "Hand"
                        }
                        Item { Layout.fillWidth: true }
                        Label {
                            color: "#7a8099"
                            font.pixelSize: 11
                            text: handController.currentGesture
                        }
                    }

                    GridLayout {
                        Layout.fillWidth: true
                        columns: 3
                        columnSpacing: 5
                        rowSpacing: 5

                        Repeater {
                            model: [
                                { label: "Grip", gesture: "powerGrip" },
                                { label: "Open", gesture: "openPalm" },
                                { label: "Fist", gesture: "closedFist" },
                                { label: "Relax", gesture: "restPose" },
                                { label: "Wave", gesture: "wave" }
                            ]
                            Button {
                                required property var modelData
                                Layout.fillWidth: true
                                text: modelData.label
                                highlighted: handController.currentGesture === modelData.gesture
                                onClicked: handController.playGesture(modelData.gesture)
                            }
                        }
                    }
                }
            }

            Rectangle {
                Layout.fillWidth: true
                color: "#171a2b"
                radius: 14
                border.color: "#262a40"
                border.width: 1
                implicitHeight: configColumn.implicitHeight + 16

                ColumnLayout {
                    id: configColumn
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 5

                    Label {
                        color: "white"
                        font.bold: true
                        text: "Config file"
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 84
                        radius: 8
                        color: dropArea.containsDrag ? "#1c2a3a" : "#11131f"
                        border.color: dropArea.containsDrag ? "#17e0ff" : "#262a40"
                        border.width: 1

                        Behavior on color { ColorAnimation { duration: 120 } }
                        Behavior on border.color { ColorAnimation { duration: 120 } }

                        ColumnLayout {
                            anchors.centerIn: parent
                            spacing: 5

                            Label {
                                Layout.alignment: Qt.AlignHCenter
                                color: "#7a8099"
                                font.pixelSize: 11
                                text: "Drop a config .json file here"
                            }

                            Button {
                                Layout.alignment: Qt.AlignHCenter
                                text: "Browse…"
                                onClicked: loadFileDialog.open()
                            }
                        }

                        DropArea {
                            id: dropArea
                            anchors.fill: parent
                            onDropped: (drop) => {
                                if (!drop.hasUrls || drop.urls.length === 0) return
                                armController.loadConfig(root.urlToLocalPath(drop.urls[0]))
                            }
                        }
                    }

                    Button {
                        Layout.fillWidth: true
                        text: "Save…"
                        onClicked: saveFileDialog.open()
                    }

                    FileDialog {
                        id: loadFileDialog
                        title: "Choose an arm config file (e.g. config/humanoid_arm.json)"
                        fileMode: FileDialog.OpenFile
                        nameFilters: ["JSON files (*.json)", "All files (*)"]
                        onAccepted: armController.loadConfig(root.urlToLocalPath(selectedFile))
                    }

                    FileDialog {
                        id: saveFileDialog
                        title: "Save arm config as"
                        fileMode: FileDialog.SaveFile
                        nameFilters: ["JSON files (*.json)", "All files (*)"]
                        onAccepted: armController.saveConfig(root.urlToLocalPath(selectedFile))
                    }

                    Label {
                        color: "#9aa0ab"
                        font.pixelSize: 11
                        wrapMode: Text.WordWrap
                        Layout.fillWidth: true
                        visible: armController.configMessage.length > 0
                        text: armController.configMessage
                    }
                }
            }

            Item { Layout.fillHeight: true }
        }
    }
}
