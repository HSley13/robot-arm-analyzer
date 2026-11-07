import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import Humanoid.Backend

// Every joint's full card, side by side across the bottom of the
// window, plus AI/Static-Dynamic/joint-count and payload capacity.
Rectangle {
    id: root
    color: "#0e1021"
    implicitHeight: bottomRow.implicitHeight + 20

    // Movable hand joints (fixed placement joints, min == max, excluded).
    readonly property int handJointCount: {
        let count = 0
        const names = HandController.chainNames
        for (let c = 0; c < names.length; c++) {
            for (let j = 0; j < HandController.jointCountForChain(names[c]); j++) {
                if (HandController.jointMinDegrees(names[c], j) !== HandController.jointMaxDegrees(names[c], j)) count++
            }
        }
        return count
    }

    ScrollView {
        anchors.fill: parent
        anchors.margins: 10
        contentWidth: bottomRow.implicitWidth
        ScrollBar.vertical.policy: ScrollBar.AlwaysOff

        RowLayout {
            id: bottomRow
            spacing: 10

            Rectangle {
                id: combinedCard
                Layout.preferredWidth: 190
                // Matches a joint card's own height - any joint card
                // is representative, so this stays correct as that
                // height changes (e.g. Dynamic mode).
                implicitHeight: jointRepeater.count > 0
                    ? jointRepeater.itemAt(0).implicitHeight
                    : combinedColumn.implicitHeight + 20
                color: "#171a2b"
                radius: 14
                border.color: "#262a40"
                border.width: 1

                ColumnLayout {
                    id: combinedColumn
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 10

                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 5

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 5

                            Label {
                                color: "white"
                                opacity: ArmController.aiMode ? 0.5 : 1.0
                                text: "Exact"
                            }

                            Switch {
                                checked: ArmController.aiMode
                                enabled: ArmController.aiModeAvailable
                                onToggled: ArmController.aiMode = checked
                            }

                            Label {
                                color: "white"
                                opacity: ArmController.aiMode ? 1.0 : 0.5
                                text: "AI" + (ArmController.aiModeAvailable ? "" : " (none)")
                            }
                        }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#262a40" }

                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 5

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 5

                            Label {
                                color: "white"
                                opacity: ArmController.dynamicMode ? 0.5 : 1.0
                                text: "Static"
                            }

                            Switch {
                                checked: ArmController.dynamicMode
                                onToggled: ArmController.dynamicMode = checked
                            }

                            Label {
                                color: "white"
                                opacity: ArmController.dynamicMode ? 1.0 : 0.5
                                text: "Dynamic"
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 5
                            visible: ArmController.dynamicMode

                            Label {
                                color: "white"
                                text: "Scrub trajectory"
                            }

                            Switch {
                                checked: ArmController.trajectoryScrubMode
                                onToggled: ArmController.trajectoryScrubMode = checked
                            }
                        }

                        ColumnLayout {
                            Layout.fillWidth: true
                            spacing: 4
                            visible: ArmController.trajectoryScrubMode

                            Label {
                                color: "#9aa0ab"
                                font.pixelSize: 11
                                wrapMode: Text.WordWrap
                                Layout.fillWidth: true
                                text: ArmController.trajectoryWaypointCount + " waypoints (drag to scrub)"
                            }

                            Slider {
                                Layout.fillWidth: true
                                from: 0
                                to: 1
                                value: ArmController.trajectoryProgress
                                onMoved: ArmController.trajectoryProgress = value
                            }

                            Button {
                                Layout.fillWidth: true
                                text: "Record waypoint"
                                onClicked: ArmController.recordWaypoint()
                            }
                        }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#262a40" }

                    // Arm summary (a humanoid arm has a fixed shoulder/elbow/
                    // wrist layout, so there's no add/remove-joint control).
                    ColumnLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        spacing: 4

                        Label {
                            color: "white"
                            font.bold: true
                            text: "Reach " + (ArmController.canvasCenter.x * 2).toFixed(2) + " m"
                        }
                        Label {
                            color: "#9aa0ab"
                            font.pixelSize: 11
                            wrapMode: Text.WordWrap
                            Layout.fillWidth: true
                            text: ArmController.jointCount + " arm joints · " + root.handJointCount + " hand joints"
                        }

                        Item { Layout.fillHeight: true }
                    }
                }
            }

            Repeater {
                id: jointRepeater
                model: ArmController.jointCount

                Rectangle {
                    id: jointCard
                    required property int index
                    Layout.preferredWidth: 230
                    implicitHeight: jointColumn.implicitHeight + 20
                    color: "#171a2b"
                    radius: 14
                    border.color: "#262a40"
                    border.width: 1

                    ColumnLayout {
                        id: jointColumn
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 5

                        Label {
                            color: "white"
                            font.bold: true
                            text: ArmController.jointNames[jointCard.index]
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Layout.alignment: Qt.AlignHCenter
                            spacing: 14

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
                                    Layout.preferredWidth: 88
                                    Layout.preferredHeight: 88
                                    from: ArmController.jointMinDegrees[jointCard.index]
                                    to: ArmController.jointMaxDegrees[jointCard.index]
                                    value: ArmController.jointAnglesDegrees[jointCard.index]
                                    onMoved: ArmController.setJointAngleDegrees(jointCard.index, value)
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
                                    Layout.preferredWidth: 88
                                    Layout.preferredHeight: 88
                                    value: ArmController.jointTorques[jointCard.index]
                                    limit: ArmController.jointTorqueLimits[jointCard.index]
                                }
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 5

                            Label {
                                color: "#9aa0ab"
                                text: "limit"
                            }

                            TextField {
                                Layout.fillWidth: true
                                text: ArmController.jointTorqueLimits[jointCard.index].toFixed(1)
                                validator: DoubleValidator { bottom: 0.0001 }
                                onEditingFinished: ArmController.setJointTorqueLimit(jointCard.index, parseFloat(text))
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 5

                            Label {
                                color: "#9aa0ab"
                                text: "len"
                            }

                            Slider {
                                Layout.fillWidth: true
                                from: 0.05
                                to: 0.6
                                value: ArmController.linkLengths[jointCard.index]
                                onMoved: ArmController.setJointLength(jointCard.index, value)
                            }

                            Label {
                                color: "#9aa0ab"
                                text: ArmController.linkLengths[jointCard.index].toFixed(2) + "m"
                            }
                        }

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
                                to: 10
                                value: ArmController.jointMassesKg[jointCard.index]
                                onMoved: ArmController.setJointMass(jointCard.index, value)
                            }

                            Label {
                                color: "#9aa0ab"
                                text: ArmController.jointMassesKg[jointCard.index].toFixed(1) + "kg"
                            }
                        }

                        ColumnLayout {
                            Layout.fillWidth: true
                            spacing: 5
                            visible: ArmController.dynamicMode

                            Label {
                                color: "#9aa0ab"
                                font.pixelSize: 11
                                text: "vel " + ArmController.jointVelocitiesDegrees[jointCard.index].toFixed(0) + "°/s"
                            }

                            Slider {
                                Layout.fillWidth: true
                                enabled: !ArmController.trajectoryScrubMode
                                from: -180
                                to: 180
                                value: ArmController.jointVelocitiesDegrees[jointCard.index]
                                onMoved: ArmController.setJointVelocityDegrees(jointCard.index, value)
                            }

                            Label {
                                color: "#9aa0ab"
                                font.pixelSize: 11
                                text: "accel " + ArmController.jointAccelerationsDegrees[jointCard.index].toFixed(0) + "°/s²"
                            }

                            Slider {
                                Layout.fillWidth: true
                                enabled: !ArmController.trajectoryScrubMode
                                from: -360
                                to: 360
                                value: ArmController.jointAccelerationsDegrees[jointCard.index]
                                onMoved: ArmController.setJointAccelerationDegrees(jointCard.index, value)
                            }

                            Label {
                                color: "#7a8099"
                                font.pixelSize: 10
                                wrapMode: Text.WordWrap
                                Layout.fillWidth: true
                                text: "g " + ArmController.gravityTorques[jointCard.index].toFixed(1)
                                    + " · i " + ArmController.inertiaTorques[jointCard.index].toFixed(1)
                                    + " · c " + ArmController.coriolisTorques[jointCard.index].toFixed(1)
                            }
                        }
                    }
                }
            }

            Rectangle {
                id: payloadCapacityCard
                Layout.preferredWidth: 150
                implicitHeight: jointRepeater.count > 0
                    ? jointRepeater.itemAt(0).implicitHeight
                    : payloadCapacityColumn.implicitHeight + 20
                color: "#171a2b"
                radius: 14
                border.color: "#262a40"
                border.width: 1

                ColumnLayout {
                    id: payloadCapacityColumn
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 5

                    Label {
                        color: "white"
                        font.bold: true
                        wrapMode: Text.WordWrap
                        Layout.fillWidth: true
                        text: "Payload capacity"
                    }

                    Label {
                        color: "#17e0ff"
                        font.bold: true
                        font.pixelSize: 15
                        wrapMode: Text.WordWrap
                        Layout.fillWidth: true
                        text: ArmController.currentMaxSafePayloadKg.toFixed(1) + " kg safe now"
                    }

                    Button {
                        Layout.fillWidth: true
                        text: "Run scan"
                        onClicked: ArmController.runPayloadScan(40)
                    }

                    Item { Layout.fillHeight: true }
                }
            }

            Rectangle {
                id: scanResultsCard
                visible: ArmController.scanSampleCount > 0
                Layout.preferredWidth: 340
                implicitHeight: jointRepeater.count > 0
                    ? jointRepeater.itemAt(0).implicitHeight
                    : scanResultsColumn.implicitHeight + 20
                color: "#171a2b"
                radius: 14
                border.color: "#262a40"
                border.width: 1

                ColumnLayout {
                    id: scanResultsColumn
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 5

                    Label {
                        color: "#9aa0ab"
                        font.pixelSize: 11
                        wrapMode: Text.WordWrap
                        Layout.fillWidth: true
                        text: ArmController.scanAiAvailable
                            ? "Exact: " + ArmController.scanExactMilliseconds.toFixed(2) + " ms  ·  AI: "
                                + ArmController.scanAiMilliseconds.toFixed(2) + " ms  ("
                                + (ArmController.scanAiMilliseconds > 0
                                    ? (ArmController.scanExactMilliseconds / ArmController.scanAiMilliseconds).toFixed(1)
                                    : "?") + "x faster)"
                            : "Exact: " + ArmController.scanExactMilliseconds.toFixed(2)
                                + " ms  ·  AI unavailable for this arm"
                    }

                    PayloadScanChart {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        reachMeters: ArmController.scanReachMeters
                        exactPayloadKg: ArmController.scanExactPayloadKg
                        aiPayloadKg: ArmController.scanAiPayloadKg
                        aiAvailable: ArmController.scanAiAvailable
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
                            visible: ArmController.scanAiAvailable
                            Rectangle { implicitWidth: 10; implicitHeight: 10; radius: 5; color: "#ff8a3d" }
                            Label { color: "#c7cbd6"; font.pixelSize: 11; text: "AI" }
                        }

                        Item { Layout.fillWidth: true }
                    }
                }
            }

            Item { Layout.fillWidth: true }
        }
    }
}
