import QtQuick
import QtQuick.Controls
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
    color: "#0e1021"

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
                        text: "Payload: " + armController.payloadKg.toFixed(2) + " kg"
                    }

                    Slider {
                        Layout.fillWidth: true
                        from: 0
                        to: 5
                        value: armController.payloadKg
                        onMoved: armController.payloadKg = value
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

                    TextField {
                        id: configPathField
                        Layout.fillWidth: true
                        placeholderText: "/path/to/arm_config.json"
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8

                        Button {
                            Layout.fillWidth: true
                            text: "Save"
                            enabled: configPathField.text.length > 0
                            onClicked: armController.saveConfig(configPathField.text)
                        }

                        Button {
                            Layout.fillWidth: true
                            text: "Load"
                            enabled: configPathField.text.length > 0
                            onClicked: armController.loadConfig(configPathField.text)
                        }
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
