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

            // Objects: place things on the floor/table (click or typed
            // coordinates), then have the humanoid pick them up and put
            // them down somewhere else.
            Rectangle {
                id: objectsCard
                Layout.fillWidth: true
                color: "#171a2b"
                radius: 14
                border.color: taskController.clickMode !== "" ? "#17e0ff" : "#262a40"
                border.width: 1
                implicitHeight: objectsColumn.implicitHeight + 16

                readonly property var selected: {
                    const list = taskController.objects
                    for (let i = 0; i < list.length; i++) {
                        if (list[i].id === taskController.selectedId) return list[i]
                    }
                    return null
                }
                readonly property bool holding: taskController.heldId >= 0

                ColumnLayout {
                    id: objectsColumn
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 6

                    RowLayout {
                        Layout.fillWidth: true
                        Label {
                            color: "white"
                            font.bold: true
                            text: "Objects"
                        }
                        Item { Layout.fillWidth: true }
                        Button {
                            visible: taskController.busy
                            text: "Stop"
                            onClicked: taskController.stop()
                        }
                    }

                    // New object: type + mass, then click to drop or type x/z.
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 5
                        ComboBox {
                            Layout.fillWidth: true
                            model: taskController.objectTypes
                            currentIndex: taskController.objectTypes.indexOf(taskController.placeType)
                            onActivated: (index) => taskController.placeType = taskController.objectTypes[index]
                        }
                        TextField {
                            implicitWidth: 52
                            text: taskController.placeMassKg.toFixed(2)
                            validator: DoubleValidator { bottom: 0.01 }
                            onEditingFinished: taskController.placeMassKg = parseFloat(text)
                        }
                        Label { color: "#9aa0ab"; text: "kg" }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 5
                        Button {
                            Layout.fillWidth: true
                            text: taskController.clickMode === "place" ? "Cancel" : "Click to place"
                            highlighted: taskController.clickMode === "place"
                            onClicked: taskController.clickMode = taskController.clickMode === "place" ? "" : "place"
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 4
                        Label { color: "#9aa0ab"; text: "x" }
                        TextField { id: newX; implicitWidth: 50; text: "0.60"; validator: DoubleValidator {} }
                        Label { color: "#9aa0ab"; text: "z" }
                        TextField { id: newZ; implicitWidth: 50; text: "-0.30"; validator: DoubleValidator {} }
                        Button {
                            Layout.fillWidth: true
                            text: "Add"
                            onClicked: taskController.addObject(taskController.placeType, taskController.placeMassKg,
                                                                parseFloat(newX.text), parseFloat(newZ.text))
                        }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#262a40" }

                    // The selected object (click one in the 3D view).
                    Label {
                        Layout.fillWidth: true
                        wrapMode: Text.WordWrap
                        color: objectsCard.selected ? "white" : "#7a8099"
                        font.pixelSize: 12
                        text: objectsCard.selected
                              ? objectsCard.selected.label + " · " + objectsCard.selected.mass.toFixed(2) + " kg · ("
                                + objectsCard.selected.x.toFixed(2) + ", " + objectsCard.selected.z.toFixed(2) + ")"
                                + (objectsCard.selected.y > 0 ? " on the table" : " on the floor")
                              : (objectsCard.holding ? "" : "Click an object in the view to select it")
                    }
                    RowLayout {
                        visible: objectsCard.selected !== null && !objectsCard.selected.held
                        Layout.fillWidth: true
                        spacing: 5
                        Button {
                            Layout.fillWidth: true
                            text: "Pick up"
                            enabled: !taskController.busy && !objectsCard.holding
                            onClicked: taskController.pickUp(taskController.selectedId)
                        }
                        Button {
                            Layout.fillWidth: true
                            text: "Move"
                            highlighted: taskController.clickMode === "move"
                            enabled: !taskController.busy
                            onClicked: taskController.clickMode = taskController.clickMode === "move" ? "" : "move"
                        }
                        Button {
                            text: "✕"
                            enabled: !taskController.busy
                            onClicked: taskController.removeObject(taskController.selectedId)
                        }
                    }
                    RowLayout {
                        visible: objectsCard.selected !== null
                        Layout.fillWidth: true
                        spacing: 5
                        Label { color: "#9aa0ab"; text: "mass" }
                        Slider {
                            Layout.fillWidth: true
                            from: 0.1
                            to: 30
                            value: objectsCard.selected ? objectsCard.selected.mass : 1
                            onMoved: taskController.setObjectMass(taskController.selectedId, value)
                        }
                    }

                    // Putting the held object down.
                    ColumnLayout {
                        visible: objectsCard.holding
                        Layout.fillWidth: true
                        spacing: 5
                        Button {
                            Layout.fillWidth: true
                            text: taskController.clickMode === "putDown" ? "Cancel" : "Put down (click a spot)"
                            highlighted: taskController.clickMode === "putDown"
                            enabled: !taskController.busy
                            onClicked: taskController.clickMode = taskController.clickMode === "putDown" ? "" : "putDown"
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 4
                            Label { color: "#9aa0ab"; text: "x" }
                            TextField { id: putX; implicitWidth: 50; text: "0.60"; validator: DoubleValidator {} }
                            Label { color: "#9aa0ab"; text: "z" }
                            TextField { id: putZ; implicitWidth: 50; text: "0.45"; validator: DoubleValidator {} }
                            Button {
                                Layout.fillWidth: true
                                text: "Put at x,z"
                                enabled: !taskController.busy
                                onClicked: taskController.putDown(parseFloat(putX.text), parseFloat(putZ.text))
                            }
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
