import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts

// Sidebar: legend, the arm (humanoid / warehouse, joints), a mouse
// cheat-sheet + status, the body's joints, payload + hand, and config I/O.
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

            // The arm on the right shoulder: the humanoid arm + hand, or a
            // warehouse arm with a gripper and as many joints as you like.
            Rectangle {
                id: armCard
                Layout.fillWidth: true
                color: "#171a2b"
                radius: 14
                border.color: "#262a40"
                border.width: 1
                implicitHeight: armColumn.implicitHeight + 16

                readonly property bool industrial: armController.armStyle === "industrial"
                // Swapping the arm under a held object (or mid-move) would drop it.
                readonly property bool canChange: !taskController.busy && taskController.heldId < 0

                ColumnLayout {
                    id: armColumn
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 6

                    RowLayout {
                        Layout.fillWidth: true
                        Label { color: "white"; font.bold: true; text: "Arm" }
                        Item { Layout.fillWidth: true }
                        Label {
                            color: "#7a8099"
                            font.pixelSize: 11
                            text: armController.jointCount + (armController.jointCount === 1 ? " joint" : " joints")
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 5
                        Button {
                            Layout.fillWidth: true
                            text: "Humanoid"
                            checkable: true
                            checked: !armCard.industrial
                            enabled: armCard.canChange
                            onClicked: taskController.useHumanoidArm()
                        }
                        Button {
                            Layout.fillWidth: true
                            text: "Warehouse"
                            checkable: true
                            checked: armCard.industrial
                            enabled: armCard.canChange
                            onClicked: taskController.useIndustrialArm()
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 5
                        Button {
                            Layout.fillWidth: true
                            text: "+ Joint"
                            enabled: armCard.canChange
                            onClicked: armController.addJoint()
                        }
                        Button {
                            Layout.fillWidth: true
                            text: "− Joint"
                            enabled: armCard.canChange && armController.jointCount > 1
                            onClicked: armController.removeLastJoint()
                        }
                    }
                    Label {
                        Layout.fillWidth: true
                        visible: !armCard.industrial
                        wrapMode: Text.WordWrap
                        color: "#7a8099"
                        font.pixelSize: 11
                        text: "Adding or removing a joint turns it into a warehouse arm with a gripper."
                    }
                }
            }

            // The scene is driven with the mouse; this card just says how,
            // and shows what's going on.
            Rectangle {
                id: sceneCard
                Layout.fillWidth: true
                color: "#171a2b"
                radius: 14
                border.color: "#262a40"
                border.width: 1
                implicitHeight: sceneColumn.implicitHeight + 16

                ColumnLayout {
                    id: sceneColumn
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 6

                    RowLayout {
                        Layout.fillWidth: true
                        Label { color: "white"; font.bold: true; text: "Mouse" }
                        Item { Layout.fillWidth: true }
                        Button {
                            visible: taskController.busy
                            text: "Stop"
                            onClicked: taskController.stop()
                        }
                    }

                    Repeater {
                        model: [
                            ["Drag the hand", "the body follows"],
                            ["Drag a joint or limb", "swing that joint"],
                            ["Drag an object", "move it"],
                            ["Click an object", "pick up · move · mass · delete"],
                            ["Click the floor / table", "put down · reach · add"],
                            ["Drag empty space", "orbit · right-drag pans · wheel zooms"]
                        ]
                        Label {
                            required property var modelData
                            Layout.fillWidth: true
                            wrapMode: Text.WordWrap
                            font.pixelSize: 11
                            color: "#9aa0ab"
                            textFormat: Text.StyledText
                            text: "<font color='#ffffff'>" + modelData[0] + "</font> - " + modelData[1]
                        }
                    }

                    Label {
                        Layout.fillWidth: true
                        visible: taskController.status !== ""
                        wrapMode: Text.WordWrap
                        color: taskController.status.indexOf("Can't") === 0 || taskController.status.indexOf("No room") === 0
                               ? "#ff4d4f" : "#17e0ff"
                        font.pixelSize: 12
                        text: taskController.status
                    }
                }
            }

            // The body's own joints: waist turn, lift height, hip bend.
            Rectangle {
                Layout.fillWidth: true
                color: "#171a2b"
                radius: 14
                border.color: "#262a40"
                border.width: 1
                implicitHeight: bodyColumn.implicitHeight + 16

                ColumnLayout {
                    id: bodyColumn
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 2
                    enabled: !taskController.busy

                    Label {
                        color: "white"
                        font.bold: true
                        text: "Body"
                    }

                    Repeater {
                        model: [
                            { label: "Waist", unit: "°", from: taskController.body.minYaw, to: taskController.body.maxYaw,
                              key: "waistYawDegrees", decimals: 0 },
                            { label: "Lift", unit: " m", from: taskController.body.minHipHeight,
                              to: taskController.body.maxHipHeight, key: "hipHeight", decimals: 2 },
                            { label: "Lean", unit: "°", from: 0, to: taskController.body.maxHipBend,
                              key: "hipBendDegrees", decimals: 0 }
                        ]
                        RowLayout {
                            required property var modelData
                            Layout.fillWidth: true
                            spacing: 4
                            Label {
                                color: "#9aa0ab"
                                Layout.preferredWidth: 36
                                text: modelData.label
                            }
                            Slider {
                                Layout.fillWidth: true
                                from: modelData.from
                                to: modelData.to
                                value: taskController[modelData.key]
                                onMoved: taskController[modelData.key] = value
                            }
                            Label {
                                color: "white"
                                Layout.preferredWidth: 46
                                horizontalAlignment: Text.AlignRight
                                text: taskController[modelData.key].toFixed(modelData.decimals) + modelData.unit
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
                implicitHeight: payloadColumn.implicitHeight + 16
                // The demo payload is the arm's own; while the hand carries a
                // scene object, that object's mass is the payload instead.
                enabled: taskController.heldId < 0 && !taskController.busy
                opacity: enabled ? 1.0 : 0.5

                ColumnLayout {
                    id: payloadColumn
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 5

                    Label {
                        color: "white"
                        font.bold: true
                        text: "Payload (demo dumbbell)"
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
                visible: armController.armStyle !== "industrial"

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
