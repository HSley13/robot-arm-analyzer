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
            anchors.margins: 16
            spacing: 14

            Label {
                text: armController.armName
                font.pixelSize: 18
                font.bold: true
                color: "white"
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }

            // Legend: explains the gauge's color zones exactly once,
            // instead of leaving a viewer to guess what "amber" means
            // the first time they see it on a specific joint.
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

            // Drive & reach (Phase 6): one pad showing every point
            // that matters in the arm's shared XY plane - where it's
            // aiming (cyan), where the payload rests (pink), and where
            // the mobile base itself is (violet) - since a real demo
            // is "drive over, then reach the rest of the way", not
            // two unrelated controls. Drag a dot, or type exact
            // coordinates - the two interactions every robot arm demo/
            // simulator researched for this feature supports. The pad
            // is a flat top-down view, not a 3D click on the view at
            // right - see ReachPad.qml for why that's deliberate, not
            // a missing feature.
            Rectangle {
                Layout.fillWidth: true
                color: "#171a2b"
                radius: 14
                border.color: "#262a40"
                border.width: 1
                implicitHeight: moveToTargetColumn.implicitHeight + 28

                ColumnLayout {
                    id: moveToTargetColumn
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 8

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8

                        Label {
                            color: "white"
                            font.bold: true
                            text: "Drive & reach"
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
                        // Wide enough to always fit the base whether
                        // it's parked at the origin or driven out to
                        // meet a distant payload, not just the arm's
                        // own reach around wherever it happens to be.
                        rangeMeters: Math.max(
                            (armController.canvasCenter.x - armController.baseX) * 2,
                            Math.abs(armController.baseX) * 1.3,
                            Math.abs(armController.baseY) * 1.3,
                            Math.abs(armController.payloadX) * 1.3,
                            Math.abs(armController.payloadY) * 1.3,
                            0.5)
                        maxReachMeters: (armController.canvasCenter.x - armController.baseX) * 2
                        reachCenterXMeters: armController.baseX
                        reachCenterYMeters: armController.baseY
                        markerXMeters: armController.jointPositions.length > 0
                            ? armController.jointPositions[armController.jointPositions.length - 1].x : 0
                        markerYMeters: armController.jointPositions.length > 0
                            ? armController.jointPositions[armController.jointPositions.length - 1].y : 0
                        onTargetMoved: (x, y) => armController.moveToTarget(x, y)

                        // The payload's own place in this same plane
                        // (Phase 6) - drag the pink dot to set down
                        // where it rests; it locks (can't be dragged)
                        // while the gripper is actually holding it,
                        // since placePayload() would silently no-op
                        // then anyway.
                        showPayloadMarker: true
                        payloadXMeters: armController.payloadX
                        payloadYMeters: armController.payloadY
                        payloadLocked: armController.payloadAttached
                        onPayloadMoved: (x, y) => armController.placePayload(x, y)

                        // The mobile base's own place (Phase 6) - drag
                        // the violet dot to drive it directly.
                        showBaseMarker: true
                        baseXMeters: armController.baseX
                        baseYMeters: armController.baseY
                        onBaseMoved: (x, y) => armController.driveTo(x, y)
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 10

                        RowLayout {
                            spacing: 5
                            Rectangle { implicitWidth: 9; implicitHeight: 9; radius: 4.5; color: "#17e0ff" }
                            Label { color: "#7a8099"; font.pixelSize: 10; text: "target" }
                        }
                        RowLayout {
                            spacing: 5
                            Rectangle { implicitWidth: 9; implicitHeight: 9; radius: 4.5; color: "#e94560" }
                            Label { color: "#7a8099"; font.pixelSize: 10; text: "payload" }
                        }
                        RowLayout {
                            spacing: 5
                            Rectangle { implicitWidth: 9; implicitHeight: 9; radius: 4.5; color: "#9b6bff" }
                            Label { color: "#7a8099"; font.pixelSize: 10; text: "base" }
                        }
                        Item { Layout.fillWidth: true }
                    }

                    Label {
                        color: "#9aa0ab"
                        font.pixelSize: 11
                        font.bold: true
                        text: "Reach (arm)"
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8

                        TextField {
                            id: targetXField
                            Layout.fillWidth: true
                            placeholderText: "x (m)"
                            validator: DoubleValidator {}
                        }

                        TextField {
                            id: targetYField
                            Layout.fillWidth: true
                            placeholderText: "y (m)"
                            validator: DoubleValidator {}
                        }

                        Button {
                            text: "Go"
                            enabled: targetXField.text.length > 0 && targetYField.text.length > 0
                            onClicked: armController.moveToTarget(parseFloat(targetXField.text), parseFloat(targetYField.text))
                        }
                    }

                    // A precise shortcut for the pick-and-place demo -
                    // dragging the cyan target dot exactly onto the
                    // pink payload dot by eye is imprecise at this
                    // pad's size; this sends the arm to the payload's
                    // exact stored coordinates instead, guaranteeing
                    // it lands within pickup range.
                    Button {
                        Layout.fillWidth: true
                        text: "Go to payload"
                        enabled: !armController.payloadAttached
                        onClicked: armController.moveToTarget(armController.payloadX, armController.payloadY)
                    }

                    Label {
                        color: "#9aa0ab"
                        font.pixelSize: 11
                        font.bold: true
                        text: "Drive (base)"
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8

                        TextField {
                            id: driveXField
                            Layout.fillWidth: true
                            placeholderText: "x (m)"
                            validator: DoubleValidator {}
                        }

                        TextField {
                            id: driveYField
                            Layout.fillWidth: true
                            placeholderText: "y (m)"
                            validator: DoubleValidator {}
                        }

                        Button {
                            text: "Go"
                            enabled: driveXField.text.length > 0 && driveYField.text.length > 0
                            onClicked: armController.driveTo(parseFloat(driveXField.text), parseFloat(driveYField.text))
                        }
                    }

                    // Parks the base within reach of the payload
                    // automatically - see ArmController::driveNearPayload()'s
                    // own doc comment for exactly how close.
                    Button {
                        Layout.fillWidth: true
                        text: "Drive to payload"
                        enabled: !armController.payloadAttached
                        onClicked: armController.driveNearPayload()
                    }
                }
            }

            // Dynamic joint count (Phase 6): add/remove a joint from
            // the END of the chain without replacing the whole arm via
            // Save/Load. New joints default to continuing the taper
            // convention (shorter, lighter, lower torque limit than
            // the one before) and are fully editable afterward via
            // the same per-joint controls below.
            Rectangle {
                Layout.fillWidth: true
                color: "#171a2b"
                radius: 14
                border.color: "#262a40"
                border.width: 1
                implicitHeight: jointCountRow.implicitHeight + 28

                RowLayout {
                    id: jointCountRow
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 8

                    Label {
                        color: "white"
                        font.bold: true
                        text: "Joints: " + armController.jointCount
                    }

                    Item { Layout.fillWidth: true }

                    Button {
                        text: "− Remove"
                        enabled: armController.jointCount > 1
                        onClicked: armController.removeLastJoint()
                    }

                    Button {
                        text: "+ Add joint"
                        onClicked: armController.addJoint()
                    }
                }
            }

            Repeater {
                model: armController.jointCount

                Rectangle {
                    id: jointCard
                    required property int index
                    Layout.fillWidth: true
                    color: "#171a2b"
                    radius: 14
                    border.color: "#262a40"
                    border.width: 1
                    implicitHeight: jointColumn.implicitHeight + 28

                    ColumnLayout {
                        id: jointColumn
                        anchors.fill: parent
                        anchors.margins: 14
                        spacing: 8

                        Label {
                            color: "white"
                            font.bold: true
                            text: "Joint " + (jointCard.index + 1)
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Layout.alignment: Qt.AlignHCenter
                            spacing: 18

                            ColumnLayout {
                                spacing: 2
                                Label {
                                    Layout.alignment: Qt.AlignHCenter
                                    color: "#7a8099"
                                    font.pixelSize: 10
                                    font.bold: true
                                    text: "ANGLE"
                                }
                                // Stays enabled during scrubbing (unlike
                                // the velocity/acceleration sliders
                                // below, which the trajectory fully
                                // determines then): the intended
                                // workflow is scrub to the end of the
                                // recording, drag this to stage the
                                // NEXT waypoint's pose, then "Record
                                // waypoint" further down.
                                AngleDial {
                                    Layout.preferredWidth: 104
                                    Layout.preferredHeight: 104
                                    enabled: !armController.trajectoryScrubMode
                                    value: armController.jointAnglesDegrees[jointCard.index]
                                    onMoved: armController.setJointAngleDegrees(jointCard.index, value)
                                }
                            }

                            ColumnLayout {
                                spacing: 2
                                Label {
                                    Layout.alignment: Qt.AlignHCenter
                                    color: "#7a8099"
                                    font.pixelSize: 10
                                    font.bold: true
                                    text: "LOAD"
                                }
                                RadialGauge {
                                    Layout.preferredWidth: 104
                                    Layout.preferredHeight: 104
                                    value: armController.jointTorques[jointCard.index]
                                    limit: armController.jointTorqueLimits[jointCard.index]
                                }
                            }
                        }

                        // Velocity/acceleration input (Phase 2): only
                        // meaningful in dynamic mode, and only directly
                        // settable when not scrubbing a trajectory (the
                        // trajectory supplies both while scrubbing
                        // instead). Left as sliders, not dials - these
                        // are secondary controls, and a rate rather
                        // than a rotation doesn't carry the same
                        // "this is literally the joint's own angle"
                        // argument for a circular widget.
                        Label {
                            visible: armController.dynamicMode
                            color: "#9aa0ab"
                            font.pixelSize: 12
                            text: "velocity: " + armController.jointVelocitiesDegrees[jointCard.index].toFixed(0) + "°/s"
                        }

                        Slider {
                            Layout.fillWidth: true
                            visible: armController.dynamicMode
                            enabled: !armController.trajectoryScrubMode
                            from: -180
                            to: 180
                            value: armController.jointVelocitiesDegrees[jointCard.index]
                            onMoved: armController.setJointVelocityDegrees(jointCard.index, value)
                        }

                        Label {
                            visible: armController.dynamicMode
                            color: "#9aa0ab"
                            font.pixelSize: 12
                            text: "acceleration: " + armController.jointAccelerationsDegrees[jointCard.index].toFixed(0) + "°/s²"
                        }

                        Slider {
                            Layout.fillWidth: true
                            visible: armController.dynamicMode
                            enabled: !armController.trajectoryScrubMode
                            from: -360
                            to: 360
                            value: armController.jointAccelerationsDegrees[jointCard.index]
                            onMoved: armController.setJointAccelerationDegrees(jointCard.index, value)
                        }

                        // Editable torque limit (Phase 4): staged here,
                        // then written out by "Save" in the config
                        // card below. Only commits on editingFinished
                        // (not every keystroke), since a half-typed
                        // number (e.g. "5." before the next digit)
                        // would otherwise be rejected as non-positive.
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 8

                            Label {
                                color: "#9aa0ab"
                                text: "limit"
                            }

                            TextField {
                                Layout.fillWidth: true
                                text: armController.jointTorqueLimits[jointCard.index].toFixed(1)
                                validator: DoubleValidator { bottom: 0.0001 }
                                onEditingFinished: armController.setJointTorqueLimit(jointCard.index, parseFloat(text))
                            }

                            Label {
                                color: "#9aa0ab"
                                text: "N·m"
                            }
                        }

                        // Editable link length (Phase 6) - a real
                        // geometry change, not just a pose change, so
                        // it's a live-dragging Slider (immediate visual
                        // feedback as the arm's shape changes) rather
                        // than the torque limit's commit-on-enter
                        // TextField above.
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 8

                            Label {
                                color: "#9aa0ab"
                                text: "length"
                            }

                            Slider {
                                Layout.fillWidth: true
                                from: 0.1
                                to: 2.0
                                value: armController.linkLengths[jointCard.index]
                                onMoved: armController.setJointLength(jointCard.index, value)
                            }

                            Label {
                                color: "#9aa0ab"
                                text: armController.linkLengths[jointCard.index].toFixed(2) + " m"
                            }
                        }

                        // Editable link mass (Phase 6) - the arm's own
                        // weight, not the payload it carries, so it
                        // can be scaled to simulate something far
                        // lighter or heavier than the shipped
                        // defaults. The slider covers a convenient
                        // everyday range (0-100kg); the field next to
                        // it takes any value at all, since a fixed
                        // slider range can never cover "whatever scale
                        // someone is actually designing for."
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 8

                            Label {
                                color: "#9aa0ab"
                                text: "mass"
                            }

                            Slider {
                                Layout.fillWidth: true
                                from: 0
                                to: 100
                                value: armController.jointMassesKg[jointCard.index]
                                onMoved: armController.setJointMass(jointCard.index, value)
                            }

                            TextField {
                                implicitWidth: 60
                                text: armController.jointMassesKg[jointCard.index].toFixed(1)
                                validator: DoubleValidator { bottom: 0 }
                                onEditingFinished: armController.setJointMass(jointCard.index, parseFloat(text))
                            }

                            Label {
                                color: "#9aa0ab"
                                text: "kg"
                            }
                        }

                        // Torque breakdown (Phase 2): gravity/inertia/
                        // Coriolis, per the roadmap's NovaSolver lesson
                        // - *why* a joint is loaded, not just how much.
                        Label {
                            visible: armController.dynamicMode
                            color: "#7a8099"
                            font.pixelSize: 11
                            wrapMode: Text.WordWrap
                            Layout.fillWidth: true
                            text: "gravity " + armController.gravityTorques[jointCard.index].toFixed(1)
                                + "  ·  inertia " + armController.inertiaTorques[jointCard.index].toFixed(1)
                                + "  ·  coriolis " + armController.coriolisTorques[jointCard.index].toFixed(1)
                                + " N·m"
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
                implicitHeight: payloadColumn.implicitHeight + 28

                ColumnLayout {
                    id: payloadColumn
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 8

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
                        spacing: 8

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

                    // Phase 6: closing the gripper only picks up the
                    // payload if the end effector is actually near it
                    // (placePayload()/the pink dot on the pad above
                    // decide where "it" is) - this switch alone no
                    // longer guarantees the torques jump by the
                    // weight, so the status line below says plainly
                    // whether anything actually got grabbed.
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8

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
                            text: "Closed"
                        }
                    }

                    Label {
                        Layout.fillWidth: true
                        wrapMode: Text.WordWrap
                        font.pixelSize: 12
                        color: armController.payloadAttached ? "#2ecc71" : "#9aa0ab"
                        font.bold: armController.payloadAttached
                        text: armController.payloadAttached
                            ? "Holding the payload"
                            : (armController.gripperOpen
                                ? "Gripper open — move to the payload, then close to grab it"
                                : "Gripper closed on nothing — not near the payload")
                    }
                }
            }

            Rectangle {
                Layout.fillWidth: true
                color: "#171a2b"
                radius: 14
                border.color: "#262a40"
                border.width: 1
                implicitHeight: modeColumn.implicitHeight + 28

                ColumnLayout {
                    id: modeColumn
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 8

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8

                        Label {
                            color: "white"
                            opacity: armController.dynamicMode ? 0.5 : 1.0
                            text: "Static"
                        }

                        Switch {
                            checked: armController.dynamicMode
                            onToggled: armController.dynamicMode = checked
                        }

                        Label {
                            color: "white"
                            opacity: armController.dynamicMode ? 1.0 : 0.5
                            text: "Dynamic"
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8
                        visible: armController.dynamicMode

                        Label {
                            color: "white"
                            text: "Scrub trajectory"
                        }

                        Switch {
                            checked: armController.trajectoryScrubMode
                            onToggled: armController.trajectoryScrubMode = checked
                        }
                    }

                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 4
                        visible: armController.trajectoryScrubMode

                        Label {
                            color: "#9aa0ab"
                            font.pixelSize: 12
                            text: "Recording: " + armController.trajectoryWaypointCount + " waypoints (drag to scrub)"
                        }

                        Slider {
                            Layout.fillWidth: true
                            from: 0
                            to: 1
                            value: armController.trajectoryProgress
                            onMoved: armController.trajectoryProgress = value
                        }

                        Button {
                            Layout.fillWidth: true
                            text: "Record waypoint at current angles"
                            onClicked: armController.recordWaypoint()
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
                implicitHeight: aiRow.implicitHeight + 28

                RowLayout {
                    id: aiRow
                    anchors.fill: parent
                    anchors.margins: 14
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
            }

            // Payload capacity (Phase 6): the live number answers "how
            // much more could this arm safely lift right now", and the
            // scan answers the question the Exact/AI toggle above never
            // could on its own - whether the AI surrogate is actually
            // worth using, measured, not just "close on one pose".
            Rectangle {
                Layout.fillWidth: true
                color: "#171a2b"
                radius: 14
                border.color: "#262a40"
                border.width: 1
                implicitHeight: payloadCapacityColumn.implicitHeight + 28

                ColumnLayout {
                    id: payloadCapacityColumn
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 8

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
                        spacing: 6
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
                implicitHeight: configColumn.implicitHeight + 28

                ColumnLayout {
                    id: configColumn
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 6

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
                            spacing: 8

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
