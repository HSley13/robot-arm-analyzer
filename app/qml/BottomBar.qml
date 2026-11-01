import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

// Phase 6 follow-up (round 3): every joint's full card (angle/load,
// torque limit, length, mass, and - in Dynamic mode - velocity/
// acceleration/torque breakdown) side by side across the bottom of
// the window, instead of stacked in the sidebar or hidden behind a
// one-joint-at-a-time chip selector (both tried and both rejected by
// the user - the first for needing to scroll, the second for needing
// to switch at all). Laid out horizontally specifically because a
// ROW of N cards only ever needs the height of ONE card, however many
// joints there are, where a COLUMN of N cards needs N times that -
// exactly the dimension this project has plenty of once the window
// is sized to the actual screen (Main.qml) rather than a fixed guess.
Rectangle {
    id: root
    color: "#0e1021"
    // Self-sizes to the row's own natural content height (one
    // direction only: row -> root, never the reverse), the same
    // pattern every card elsewhere in this project uses
    // (implicitHeight: someColumn.implicitHeight + margins) - Main.qml
    // leaves this item's Layout.preferredHeight unset so Qt Quick
    // Layouts falls back to its implicitHeight automatically.
    implicitHeight: bottomRow.implicitHeight + 20

    // Wrapped in a horizontal ScrollView, not a bare Row, so adding
    // joints past whatever fits the window's width degrades into a
    // scrollbar instead of overflowing or squeezing every card
    // unreadably thin. Only ever scrolls horizontally - root's own
    // height already matches the row's natural height exactly, so
    // there's nothing to scroll vertically.
    ScrollView {
        anchors.fill: parent
        anchors.margins: 10
        contentWidth: bottomRow.implicitWidth
        ScrollBar.vertical.policy: ScrollBar.AlwaysOff

        RowLayout {
            id: bottomRow
            spacing: 10

            // AI mode, Static/Dynamic mode, and joint count: three
            // settings that are each small on their own, combined
            // into ONE card instead of three, stacked vertically so
            // all three are automatically the same width (one shared
            // column) - order between them doesn't matter, so
            // Exact/AI sits on top here simply because it was first
            // in the sidebar before. The card's OVERALL height is
            // pinned to match a joint card's height exactly (see
            // implicitHeight below, and the trailing spacer at the
            // end of combinedColumn that absorbs the difference),
            // not just each section individually.
            Rectangle {
                id: combinedCard
                Layout.preferredWidth: 190
                // Matches a joint card's own height exactly, not an
                // independent guess - jointRepeater.itemAt(0) is
                // representative of every joint card (same layout,
                // just different numbers), and this stays correct
                // automatically as that height changes (e.g. Dynamic
                // mode adding the velocity/acceleration/breakdown
                // section to every joint card).
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

                    // Exact/AI (Phase 1/5): which engine computes
                    // joint torque.
                    ColumnLayout {
                        Layout.fillWidth: true
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
                                text: "AI" + (armController.aiModeAvailable ? "" : " (none)")
                            }
                        }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#262a40" }

                    // Static/Dynamic (Phase 2): switches each joint
                    // card's velocity/acceleration/torque-breakdown
                    // section into inverse_dynamics() (or the dynamic
                    // AI surrogate) instead of treating the arm as at
                    // rest.
                    ColumnLayout {
                        Layout.fillWidth: true
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
                                font.pixelSize: 11
                                wrapMode: Text.WordWrap
                                Layout.fillWidth: true
                                text: armController.trajectoryWaypointCount + " waypoints (drag to scrub)"
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
                                text: "Record waypoint"
                                onClicked: armController.recordWaypoint()
                            }
                        }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#262a40" }

                    // Joint count (Phase 6): add/remove a joint from
                    // the END of the chain without replacing the whole
                    // arm via Save/Load. New joints default to
                    // continuing the taper convention (shorter,
                    // lighter, lower torque limit than the one before)
                    // and are fully editable afterward via the
                    // per-joint cards to the right.
                    ColumnLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        spacing: 6

                        Label {
                            color: "white"
                            font.bold: true
                            wrapMode: Text.WordWrap
                            Layout.fillWidth: true
                            text: "Joints: " + armController.jointCount
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 6

                            Button {
                                Layout.fillWidth: true
                                text: "− Remove"
                                enabled: armController.jointCount > 1
                                onClicked: armController.removeLastJoint()
                            }

                            Button {
                                Layout.fillWidth: true
                                text: "+ Add"
                                onClicked: armController.addJoint()
                            }
                        }

                        Item { Layout.fillHeight: true }
                    }
                }
            }

            Repeater {
                id: jointRepeater
                model: armController.jointCount

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
                            text: "Joint " + (jointCard.index + 1)
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
                                    Layout.preferredWidth: 88
                                    Layout.preferredHeight: 88
                                    value: armController.jointTorques[jointCard.index]
                                    limit: armController.jointTorqueLimits[jointCard.index]
                                }
                            }
                        }

                        // Editable torque limit (Phase 4): staged
                        // here, then written out by "Save" in the
                        // sidebar's config card. Only commits on
                        // editingFinished (not every keystroke),
                        // since a half-typed number (e.g. "5." before
                        // the next digit) would otherwise be rejected
                        // as non-positive.
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 5

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
                        }

                        // Editable link length (Phase 6) - a real
                        // geometry change, not just a pose change, so
                        // it's a live-dragging Slider (immediate
                        // visual feedback as the arm's shape changes)
                        // rather than the torque limit's commit-on-
                        // enter TextField above.
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 5

                            Label {
                                color: "#9aa0ab"
                                text: "len"
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
                                text: armController.linkLengths[jointCard.index].toFixed(2) + "m"
                            }
                        }

                        // Editable link mass (Phase 6) - the arm's own
                        // weight, not the payload it carries, so it
                        // can be scaled to simulate something far
                        // lighter or heavier than the shipped
                        // defaults.
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
                                value: armController.jointMassesKg[jointCard.index]
                                onMoved: armController.setJointMass(jointCard.index, value)
                            }

                            Label {
                                color: "#9aa0ab"
                                text: armController.jointMassesKg[jointCard.index].toFixed(1) + "kg"
                            }
                        }

                        // Velocity/acceleration + torque breakdown -
                        // only meaningful in Dynamic mode (see the
                        // sidebar's Static/Dynamic card), so collapsed
                        // to nothing while Static rather than taking
                        // up space with controls that don't do
                        // anything yet.
                        ColumnLayout {
                            Layout.fillWidth: true
                            spacing: 5
                            visible: armController.dynamicMode

                            Label {
                                color: "#9aa0ab"
                                font.pixelSize: 11
                                text: "vel " + armController.jointVelocitiesDegrees[jointCard.index].toFixed(0) + "°/s"
                            }

                            Slider {
                                Layout.fillWidth: true
                                enabled: !armController.trajectoryScrubMode
                                from: -180
                                to: 180
                                value: armController.jointVelocitiesDegrees[jointCard.index]
                                onMoved: armController.setJointVelocityDegrees(jointCard.index, value)
                            }

                            Label {
                                color: "#9aa0ab"
                                font.pixelSize: 11
                                text: "accel " + armController.jointAccelerationsDegrees[jointCard.index].toFixed(0) + "°/s²"
                            }

                            Slider {
                                Layout.fillWidth: true
                                enabled: !armController.trajectoryScrubMode
                                from: -360
                                to: 360
                                value: armController.jointAccelerationsDegrees[jointCard.index]
                                onMoved: armController.setJointAccelerationDegrees(jointCard.index, value)
                            }

                            // Torque breakdown (Phase 2): gravity/
                            // inertia/Coriolis, per the roadmap's
                            // NovaSolver lesson - *why* a joint is
                            // loaded, not just how much.
                            Label {
                                color: "#7a8099"
                                font.pixelSize: 10
                                wrapMode: Text.WordWrap
                                Layout.fillWidth: true
                                text: "g " + armController.gravityTorques[jointCard.index].toFixed(1)
                                    + " · i " + armController.inertiaTorques[jointCard.index].toFixed(1)
                                    + " · c " + armController.coriolisTorques[jointCard.index].toFixed(1)
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
                        text: armController.currentMaxSafePayloadKg.toFixed(1) + " kg safe now"
                    }

                    Button {
                        Layout.fillWidth: true
                        text: "Run scan"
                        onClicked: armController.runPayloadScan(40)
                    }

                    Item { Layout.fillHeight: true }
                }
            }

            Rectangle {
                id: scanResultsCard
                visible: armController.scanSampleCount > 0
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
                        Layout.fillHeight: true
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
                }
            }

            Item { Layout.fillWidth: true }
        }
    }
}
