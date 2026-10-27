import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    color: "#16161e"

    ScrollView {
        anchors.fill: parent
        contentWidth: availableWidth

        ColumnLayout {
            width: parent.width
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
                        enabled: !armController.trajectoryScrubMode
                        from: -180
                        to: 180
                        value: armController.jointAnglesDegrees[jointRow.index]
                        onMoved: armController.setJointAngleDegrees(jointRow.index, value)
                    }

                    // Velocity/acceleration input (Phase 2): only
                    // meaningful in dynamic mode, and only directly
                    // settable when not scrubbing a trajectory (the
                    // trajectory supplies both while scrubbing instead).
                    Label {
                        visible: armController.dynamicMode
                        color: "#aaaaaa"
                        font.pixelSize: 12
                        text: "velocity: " + armController.jointVelocitiesDegrees[jointRow.index].toFixed(0) + "°/s"
                    }

                    Slider {
                        Layout.fillWidth: true
                        visible: armController.dynamicMode
                        enabled: !armController.trajectoryScrubMode
                        from: -180
                        to: 180
                        value: armController.jointVelocitiesDegrees[jointRow.index]
                        onMoved: armController.setJointVelocityDegrees(jointRow.index, value)
                    }

                    Label {
                        visible: armController.dynamicMode
                        color: "#aaaaaa"
                        font.pixelSize: 12
                        text: "acceleration: " + armController.jointAccelerationsDegrees[jointRow.index].toFixed(0) + "°/s²"
                    }

                    Slider {
                        Layout.fillWidth: true
                        visible: armController.dynamicMode
                        enabled: !armController.trajectoryScrubMode
                        from: -360
                        to: 360
                        value: armController.jointAccelerationsDegrees[jointRow.index]
                        onMoved: armController.setJointAccelerationDegrees(jointRow.index, value)
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

                    // Torque breakdown (Phase 2): gravity/inertia/Coriolis,
                    // per the roadmap's NovaSolver lesson - *why* a joint
                    // is loaded, not just how much.
                    Label {
                        visible: armController.dynamicMode
                        color: "#aaaaaa"
                        font.pixelSize: 11
                        text: "  gravity " + armController.gravityTorques[jointRow.index].toFixed(1)
                            + "  ·  inertia " + armController.inertiaTorques[jointRow.index].toFixed(1)
                            + "  ·  coriolis " + armController.coriolisTorques[jointRow.index].toFixed(1)
                            + " N·m"
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
                    opacity: armController.dynamicMode ? 1.0 : 0.5
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
                    color: "#aaaaaa"
                    font.pixelSize: 12
                    text: "Trajectory: rest → captured pose (drag to scrub)"
                }

                Slider {
                    Layout.fillWidth: true
                    from: 0
                    to: 1
                    value: armController.trajectoryProgress
                    onMoved: armController.trajectoryProgress = value
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
}
