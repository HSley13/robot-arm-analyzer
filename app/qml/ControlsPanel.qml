import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts

// Phase 5: redesigned around direct feedback that the plain-Slider
// sidebar was "ugly" and "ain't the best way" to set a joint angle,
// and that the green/amber/red dot "only turns yellow, red" without
// explaining what that means. Replaces per-joint angle Sliders with
// AngleDial (a circular drag control, matching the arm's own rotary
// joints better than a linear bar) and the color dot with RadialGauge
// (a percentage, a SAFE/CAUTION/OVERLOAD word, and the raw numbers,
// not color alone) - plus a one-time legend so that word/color pairing
// only needs explaining once, not per joint. Everything is grouped
// into labeled cards instead of one long unbroken column, and the
// color palette matches the 3D view's own (dark navy background, cyan
// accent) so the sidebar reads as part of the same instrument panel.
//
// Phase 6 follow-up (round 3): per-joint controls moved out to
// BottomBar.qml entirely - every joint's full card now shows side by
// side there instead of one at a time behind a chip selector here
// (user feedback: didn't want to switch between joints in a panel at
// all). No tabs either - everything here is always visible, all at
// once.
//
// Round 4: narrowed further, on request, to only the four things
// that are explicitly the sidebar's job - reach targeting, payload,
// the payload-capacity scan, and config file I/O. The legend, AI
// toggle, and Static/Dynamic mode moved to BottomBar.qml, which had
// room to spare once every joint's own card moved there (round 3) -
// see that file for why a ROW has room a COLUMN doesn't.
Rectangle {
    id: root
    color: "#0e1021"

    // Converts a file:// URL (from FileDialog.selectedFile or a
    // DropArea's drop.urls) into a plain local path string -
    // armController.loadConfig()/saveConfig() take a filesystem path,
    // not a URL, and QML has no built-in url-to-local-path conversion.
    // Only handles the file:// scheme this app's own file pickers and
    // OS drag-and-drop actually produce, not arbitrary URLs.
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

            // Click-to-reach inverse kinematics (Phase 6): drag the
            // dot on the pad - the two interactions every robot arm
            // demo/simulator researched for this feature supports are
            // dragging and direct 3D manipulation (see ArmView3D.qml's
            // joint-drag rings); a typed x/y entry turned out to be
            // redundant with both and was removed. The pad is a flat
            // top-down view of the arm's own XY motion plane, not a 3D
            // click on the view at right - see ReachPad.qml for why
            // that's deliberate, not a missing feature.
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

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 5

                        Label {
                            color: "white"
                            font.bold: true
                            text: "Move to target"
                        }

                        Item { Layout.fillWidth: true }

                        Label {
                            color: "#9aa0ab"
                            font.pixelSize: 11
                            text: "Show reach"
                        }

                        Switch {
                            checked: armController.showWorkspaceEnvelope
                            onToggled: armController.showWorkspaceEnvelope = checked
                        }
                    }

                    ReachPad {
                        Layout.alignment: Qt.AlignHCenter
                        Layout.preferredWidth: 170
                        Layout.preferredHeight: 170
                        // The pad's own visible range is deliberately
                        // WIDER than the reach circle (not equal to
                        // it) - margin around the circle to drag into
                        // past the boundary, instead of the circle's
                        // edge landing exactly on the pad's own edge.
                        // Dragging right at that edge is exactly where
                        // the arm is forced fully extended (a genuine
                        // kinematic singularity - see
                        // solve_inverse_kinematics()'s own doc comment)
                        // and feels the least responsive, so giving it
                        // room on every side makes it easier to stay
                        // off that exact boundary by accident.
                        readonly property real reachDiameterMeters: armController.canvasCenter.x * 2
                        rangeMeters: Math.max(reachDiameterMeters * 1.4, 0.5)
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

                    // The slider's range (0-50kg) is just a convenient
                    // everyday scale for dragging - it was previously
                    // capped at 5kg with no way to go higher at all.
                    // The field next to it takes any value, so this
                    // arm can carry whatever someone is actually
                    // designing it to lift, not just what fits a
                    // slider.
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 5

                        Slider {
                            Layout.fillWidth: true
                            from: 0
                            to: 50
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

                    // The gripper decides whether the staged payload
                    // above actually counts toward the torque
                    // calculation - open it and drag the payload
                    // slider, nothing changes; close it, and the
                    // torques jump by the weight. A deliberately
                    // visible cause-and-effect, not just a cosmetic
                    // toggle.
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 5

                        Label {
                            color: "white"
                            opacity: armController.gripperOpen ? 1.0 : 0.5
                            text: "Open"
                        }

                        Switch {
                            checked: !armController.gripperOpen
                            onToggled: armController.gripperOpen = !checked
                        }

                        Label {
                            color: "white"
                            opacity: armController.gripperOpen ? 0.5 : 1.0
                            text: "Closed (holding payload)"
                        }
                    }
                }
            }

            // Payload capacity (Phase 6): the live number answers "how
            // much more could this arm safely lift right now", and the
            // scan answers the question the Exact/AI toggle never
            // could on its own - whether the AI surrogate is actually
            // worth using, measured, not just "close on one pose".
            Rectangle {
                Layout.fillWidth: true
                color: "#171a2b"
                radius: 14
                border.color: "#262a40"
                border.width: 1
                implicitHeight: payloadCapacityColumn.implicitHeight + 16

                ColumnLayout {
                    id: payloadCapacityColumn
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 5

                    Label {
                        color: "white"
                        font.bold: true
                        text: "Payload capacity"
                    }

                    Label {
                        color: "#17e0ff"
                        font.bold: true
                        font.pixelSize: 16
                        text: "Safe right now: " + armController.currentMaxSafePayloadKg.toFixed(1) + " kg"
                    }

                    Button {
                        Layout.fillWidth: true
                        text: "Run workspace scan"
                        onClicked: armController.runPayloadScan(40)
                    }

                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 5
                        visible: armController.scanSampleCount > 0

                        Label {
                            color: "#9aa0ab"
                            font.pixelSize: 11
                            wrapMode: Text.WordWrap
                            Layout.fillWidth: true
                            text: armController.scanAiAvailable
                                ? "Exact: " + armController.scanExactMilliseconds.toFixed(2) + " ms  ·  AI: "
                                    + armController.scanAiMilliseconds.toFixed(2) + " ms  ("
                                    + (armController.scanAiMilliseconds > 0
                                        ? (armController.scanExactMilliseconds / armController.scanAiMilliseconds).toFixed(1)
                                        : "?") + "x faster)"
                                : "Exact: " + armController.scanExactMilliseconds.toFixed(2)
                                    + " ms  ·  AI unavailable for this arm"
                        }

                        PayloadScanChart {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 150
                            reachMeters: armController.scanReachMeters
                            exactPayloadKg: armController.scanExactPayloadKg
                            aiPayloadKg: armController.scanAiPayloadKg
                            aiAvailable: armController.scanAiAvailable
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 14

                            RowLayout {
                                spacing: 5
                                Rectangle { implicitWidth: 10; implicitHeight: 10; radius: 5; color: "#17e0ff" }
                                Label { color: "#c7cbd6"; font.pixelSize: 11; text: "Exact" }
                            }

                            RowLayout {
                                spacing: 5
                                visible: armController.scanAiAvailable
                                Rectangle { implicitWidth: 10; implicitHeight: 10; radius: 5; color: "#ff8a3d" }
                                Label { color: "#c7cbd6"; font.pixelSize: 11; text: "AI" }
                            }

                            Item { Layout.fillWidth: true }
                        }

                        Label {
                            color: "#7a8099"
                            font.pixelSize: 10
                            wrapMode: Text.WordWrap
                            Layout.fillWidth: true
                            text: "Sweeps from the shoulder's current angle, folding every other "
                                + "joint together, from fully extended down to folded."
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

                    // One combined load zone, no typed path at all:
                    // drop a .json file from the Finder anywhere on
                    // it, or click the button in its middle to browse
                    // for one instead - both end up calling
                    // loadConfig() the same way. The Button and the
                    // DropArea don't conflict despite overlapping:
                    // a DropArea only reacts to an external OS drag
                    // session, never to an ordinary click, so the
                    // button underneath it still receives clicks
                    // normally.
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
                        title: "Choose an arm config file"
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
