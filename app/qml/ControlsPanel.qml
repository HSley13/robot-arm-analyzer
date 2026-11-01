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

    // Which joint the "Controls" tab's single joint card shows - Phase
    // 6 follow-up: showing every joint's card stacked at once was the
    // single biggest contributor to the sidebar needing to scroll, so
    // only one is shown at a time, picked via the small numbered chips
    // above it. Clamped through effectiveJointIndex rather than reset
    // reactively, so removeLastJoint() shrinking the count below the
    // previously-selected index can't leave this pointing past the end.
    property int selectedJointIndex: 0
    readonly property int effectiveJointIndex: Math.min(selectedJointIndex, armController.jointCount - 1)

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
            id: sidebarColumn
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

            // Click-to-reach inverse kinematics (Phase 6): drag the
            // dot on the pad, or type exact coordinates - the two
            // interactions every robot arm demo/simulator researched
            // for this feature supports. The pad is a flat top-down
            // view of the arm's own XY motion plane, not a 3D click on
            // the view at right - see ReachPad.qml for why that's
            // deliberate, not a missing feature.
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
                        rangeMeters: Math.max(armController.canvasCenter.x * 2, 0.5)
                        maxReachMeters: armController.canvasCenter.x * 2
                        markerXMeters: armController.jointPositions.length > 0
                            ? armController.jointPositions[armController.jointPositions.length - 1].x : 0
                        markerYMeters: armController.jointPositions.length > 0
                            ? armController.jointPositions[armController.jointPositions.length - 1].y : 0
                        onTargetMoved: (x, y) => armController.moveToTarget(x, y)
                    }
                }
            }

            // Controls / Advanced split (Phase 6 follow-up): everyday
            // stuff (joint angle/load, payload, AI toggle) on the tab
            // shown by default, specialist stuff (Static/Dynamic mode,
            // per-joint velocity/acceleration/torque-breakdown, the
            // payload-capacity scan, config file) one click away - the
            // combination of this split AND only ever showing ONE
            // joint's card at a time (below) is what actually gets the
            // default tab to fit without scrolling; neither alone was
            // enough (a 3-joint arm's stacked cards alone measured
            // ~1588px of content, far past any real screen).
            TabBar {
                id: sectionTabBar
                Layout.fillWidth: true
                TabButton { text: "Controls" }
                TabButton { text: "Advanced" }
            }

            ColumnLayout {
                Layout.fillWidth: true
                visible: sectionTabBar.currentIndex === 0
                spacing: 8

                // Dynamic joint count (Phase 6): add/remove a joint
                // from the END of the chain without replacing the
                // whole arm via Save/Load. New joints default to
                // continuing the taper convention (shorter, lighter,
                // lower torque limit than the one before) and are
                // fully editable afterward via the card below.
                Rectangle {
                    Layout.fillWidth: true
                    color: "#171a2b"
                    radius: 14
                    border.color: "#262a40"
                    border.width: 1
                    implicitHeight: jointCountRow.implicitHeight + 16

                    RowLayout {
                        id: jointCountRow
                        anchors.fill: parent
                        anchors.margins: 8
                        spacing: 5

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

                // Picks which single joint the card below shows -
                // see root.selectedJointIndex's own doc comment for
                // why only one at a time.
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 5

                    Repeater {
                        model: armController.jointCount

                        Rectangle {
                            id: jointChip
                            required property int index
                            readonly property bool isSelected: root.effectiveJointIndex === index
                            implicitWidth: 34
                            implicitHeight: 28
                            radius: 8
                            color: isSelected ? "#17e0ff" : "#1c2033"
                            border.color: isSelected ? "#17e0ff" : "#262a40"
                            border.width: 1

                            Label {
                                anchors.centerIn: parent
                                text: jointChip.index + 1
                                color: jointChip.isSelected ? "#0e1021" : "#c7cbd6"
                                font.bold: jointChip.isSelected
                            }

                            TapHandler {
                                onTapped: root.selectedJointIndex = jointChip.index
                            }
                        }
                    }

                    Item { Layout.fillWidth: true }
                }

                Rectangle {
                    Layout.fillWidth: true
                    color: "#171a2b"
                    radius: 14
                    border.color: "#262a40"
                    border.width: 1
                    implicitHeight: jointColumn.implicitHeight + 16

                    ColumnLayout {
                        id: jointColumn
                        anchors.fill: parent
                        anchors.margins: 8
                        spacing: 5

                        Label {
                            color: "white"
                            font.bold: true
                            text: "Joint " + (root.effectiveJointIndex + 1)
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
                                AngleDial {
                                    Layout.preferredWidth: 96
                                    Layout.preferredHeight: 96
                                    value: armController.jointAnglesDegrees[root.effectiveJointIndex]
                                    onMoved: armController.setJointAngleDegrees(root.effectiveJointIndex, value)
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
                                    Layout.preferredWidth: 96
                                    Layout.preferredHeight: 96
                                    value: armController.jointTorques[root.effectiveJointIndex]
                                    limit: armController.jointTorqueLimits[root.effectiveJointIndex]
                                }
                            }
                        }

                        // Editable torque limit (Phase 4): staged here,
                        // then written out by "Save" in the Advanced
                        // tab's config card. Only commits on
                        // editingFinished (not every keystroke), since
                        // a half-typed number (e.g. "5." before the
                        // next digit) would otherwise be rejected as
                        // non-positive.
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 5

                            Label {
                                color: "#9aa0ab"
                                text: "limit"
                            }

                            TextField {
                                Layout.fillWidth: true
                                text: armController.jointTorqueLimits[root.effectiveJointIndex].toFixed(1)
                                validator: DoubleValidator { bottom: 0.0001 }
                                onEditingFinished: armController.setJointTorqueLimit(root.effectiveJointIndex, parseFloat(text))
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
                            spacing: 5

                            Label {
                                color: "#9aa0ab"
                                text: "length"
                            }

                            Slider {
                                Layout.fillWidth: true
                                from: 0.1
                                to: 2.0
                                value: armController.linkLengths[root.effectiveJointIndex]
                                onMoved: armController.setJointLength(root.effectiveJointIndex, value)
                            }

                            Label {
                                color: "#9aa0ab"
                                text: armController.linkLengths[root.effectiveJointIndex].toFixed(2) + " m"
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
                            spacing: 5

                            Label {
                                color: "#9aa0ab"
                                text: "mass"
                            }

                            Slider {
                                Layout.fillWidth: true
                                from: 0
                                to: 100
                                value: armController.jointMassesKg[root.effectiveJointIndex]
                                onMoved: armController.setJointMass(root.effectiveJointIndex, value)
                            }

                            TextField {
                                implicitWidth: 60
                                text: armController.jointMassesKg[root.effectiveJointIndex].toFixed(1)
                                validator: DoubleValidator { bottom: 0 }
                                onEditingFinished: armController.setJointMass(root.effectiveJointIndex, parseFloat(text))
                            }

                            Label {
                                color: "#9aa0ab"
                                text: "kg"
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

                Rectangle {
                    Layout.fillWidth: true
                    color: "#171a2b"
                    radius: 14
                    border.color: "#262a40"
                    border.width: 1
                    implicitHeight: aiColumn.implicitHeight + 16

                    ColumnLayout {
                        id: aiColumn
                        anchors.fill: parent
                        anchors.margins: 8
                        spacing: 5

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 5

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

                        // What this switch actually does, spelled out
                        // once - "Exact" and "AI" otherwise read as two
                        // unexplained words.
                        Label {
                            color: "#7a8099"
                            font.pixelSize: 10
                            wrapMode: Text.WordWrap
                            Layout.fillWidth: true
                            text: "Exact runs the real physics engine. AI asks a small neural "
                                + "network trained to approximate it instead - usually very "
                                + "close, and much faster (see Payload capacity below)."
                        }
                    }
                }

                Item { Layout.fillHeight: true }
            }

            ColumnLayout {
                Layout.fillWidth: true
                visible: sectionTabBar.currentIndex === 1
                spacing: 8

                // Static/Dynamic (Phase 2): switches jointVelocitiesDegrees/
                // jointAccelerationsDegrees below into inverse_dynamics()
                // (or the dynamic AI surrogate) instead of treating the
                // arm as at rest. Specialist-only - most demos never
                // need this - which is exactly why it lives here instead
                // of the default Controls tab.
                Rectangle {
                    Layout.fillWidth: true
                    color: "#171a2b"
                    radius: 14
                    border.color: "#262a40"
                    border.width: 1
                    implicitHeight: modeColumn.implicitHeight + 16

                    ColumnLayout {
                        id: modeColumn
                        anchors.fill: parent
                        anchors.margins: 8
                        spacing: 5

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 5

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

                        Label {
                            color: "#7a8099"
                            font.pixelSize: 10
                            wrapMode: Text.WordWrap
                            Layout.fillWidth: true
                            text: "Dynamic adds each joint's velocity/acceleration into the "
                                + "torque calculation (gravity + inertia + Coriolis), instead "
                                + "of treating the arm as momentarily at rest."
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 5
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

                // Per-joint velocity/acceleration + torque breakdown -
                // only meaningful in Dynamic mode, so collapsed to
                // nothing (not just hidden content in a visible card)
                // while Static, rather than showing N empty-looking
                // cards above the cards that actually have something
                // in Static mode's default state.
                Repeater {
                    model: armController.jointCount

                    Rectangle {
                        id: dynamicsCard
                        required property int index
                        Layout.fillWidth: true
                        visible: armController.dynamicMode
                        color: "#171a2b"
                        radius: 14
                        border.color: "#262a40"
                        border.width: 1
                        implicitHeight: dynamicsColumn.implicitHeight + 16

                        ColumnLayout {
                            id: dynamicsColumn
                            anchors.fill: parent
                            anchors.margins: 8
                            spacing: 5

                            Label {
                                color: "white"
                                font.bold: true
                                text: "Joint " + (dynamicsCard.index + 1) + " dynamics"
                            }

                            Label {
                                color: "#9aa0ab"
                                font.pixelSize: 12
                                text: "velocity: " + armController.jointVelocitiesDegrees[dynamicsCard.index].toFixed(0) + "°/s"
                            }

                            Slider {
                                Layout.fillWidth: true
                                enabled: !armController.trajectoryScrubMode
                                from: -180
                                to: 180
                                value: armController.jointVelocitiesDegrees[dynamicsCard.index]
                                onMoved: armController.setJointVelocityDegrees(dynamicsCard.index, value)
                            }

                            Label {
                                color: "#9aa0ab"
                                font.pixelSize: 12
                                text: "acceleration: " + armController.jointAccelerationsDegrees[dynamicsCard.index].toFixed(0) + "°/s²"
                            }

                            Slider {
                                Layout.fillWidth: true
                                enabled: !armController.trajectoryScrubMode
                                from: -360
                                to: 360
                                value: armController.jointAccelerationsDegrees[dynamicsCard.index]
                                onMoved: armController.setJointAccelerationDegrees(dynamicsCard.index, value)
                            }

                            // Torque breakdown (Phase 2): gravity/inertia/
                            // Coriolis, per the roadmap's NovaSolver lesson
                            // - *why* a joint is loaded, not just how much.
                            Label {
                                color: "#7a8099"
                                font.pixelSize: 11
                                wrapMode: Text.WordWrap
                                Layout.fillWidth: true
                                text: "gravity " + armController.gravityTorques[dynamicsCard.index].toFixed(1)
                                    + "  ·  inertia " + armController.inertiaTorques[dynamicsCard.index].toFixed(1)
                                    + "  ·  coriolis " + armController.coriolisTorques[dynamicsCard.index].toFixed(1)
                                    + " N·m"
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
}
