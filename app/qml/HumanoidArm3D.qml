pragma ComponentBehavior: Bound
import QtQuick
import QtQuick3D
import Humanoid.Geometry

// The humanoid arm from shoulder to wrist: actuator housings with load-
// status LED rings, a molded upper-arm shell, and a forearm built like
// the reference robot - an exposed chassis carrying stacked control
// boards and cable bundles, a white side cover, and a steel wrist clevis.
//
// Placement comes straight from armController's forward kinematics.
// Each limb is a Node at its joint, rotated by that joint's full frame
// (frameRotations), so inside it:
//   +X runs along the limb toward the next joint,
//   +Y is the in-plane side (the thumb side, for the neutral grip),
//   +Z is the out-of-plane side facing the default camera (the back of
//      the hand / the board side of the forearm).
// Limb lengths are read live, so dragging a "len" slider stretches the
// shells instead of leaving gaps.
//
// The hand (Hand3D) hangs off the wrist pivot, turned so its own frame
// (+X fingers, +Y palm, +Z thumb) holds the neutral, thumb-up grip a
// person uses to carry a dumbbell or a bag.
Node {
    id: root

    readonly property real meshUnit: 100.0
    readonly property int jointCount: armController.jointPositions.length - 1

    function linkLength(i) { return i < armController.linkLengths.length ? armController.linkLengths[i] : 0.0 }
    function jointColor(i) { return i < armController.jointColors.length ? armController.jointColors[i] : "#2ecc71" }

    readonly property real upperArmLength: linkLength(0)
    readonly property real forearmLength: linkLength(1)
    readonly property real wristRadians: armController.jointAnglesDegrees.length > 2
                                         ? armController.jointAnglesDegrees[2] * Math.PI / 180 : 0

    // ---- materials -------------------------------------------------------

    PrincipledMaterial { id: shellMaterial; baseColor: "#7f91a3"; roughness: 0.48; specularAmount: 0.45 }
    PrincipledMaterial { id: shellDarkMaterial; baseColor: "#5f7184"; roughness: 0.55 }
    PrincipledMaterial { id: whiteCoverMaterial; baseColor: "#e9ebee"; roughness: 0.42 }
    PrincipledMaterial { id: navyMaterial; baseColor: "#1d2c4f"; roughness: 0.5 }
    PrincipledMaterial { id: anodizedMaterial; baseColor: "#2f353d"; metalness: 0.75; roughness: 0.42 }
    PrincipledMaterial { id: steelMaterial; baseColor: "#c9ced4"; metalness: 1.0; roughness: 0.26 }
    PrincipledMaterial { id: darkSteelMaterial; baseColor: "#4a5059"; metalness: 0.9; roughness: 0.35 }
    PrincipledMaterial { id: holeMaterial; baseColor: "#161b21"; roughness: 0.9 }
    PrincipledMaterial { id: pcbMaterial; baseColor: "#1e6a3b"; roughness: 0.55; metalness: 0.05 }
    PrincipledMaterial { id: chipMaterial; baseColor: "#121417"; roughness: 0.4 }
    PrincipledMaterial { id: headerMaterial; baseColor: "#ece6d6"; roughness: 0.6 }
    PrincipledMaterial { id: capMaterial; baseColor: "#2b2f6e"; roughness: 0.35; metalness: 0.2 }

    // ---- reusable pieces -------------------------------------------------

    // A round actuator housing whose axis is local Z, with a steel face
    // plate and an LED ring that shows the joint's load status.
    component Actuator: Node {
        id: actuator
        property real radius: 0.05
        property real depth: 0.09
        property color statusColor: "#2ecc71"

        Model {
            source: "#Cylinder"
            eulerRotation.x: 90
            scale: Qt.vector3d(2 * actuator.radius / root.meshUnit, actuator.depth / root.meshUnit,
                               2 * actuator.radius / root.meshUnit)
            materials: shellDarkMaterial
        }
        // Face plates on both sides.
        Repeater3D {
            model: 2
            Node {
                required property int index
                readonly property real side: index === 0 ? 1 : -1
                Model {
                    source: "#Cylinder"
                    eulerRotation.x: 90
                    position: Qt.vector3d(0, 0, parent.side * (actuator.depth / 2 + 0.002))
                    scale: Qt.vector3d(1.78 * actuator.radius / root.meshUnit, 0.004 / root.meshUnit,
                                       1.78 * actuator.radius / root.meshUnit)
                    materials: anodizedMaterial
                }
                Model {
                    source: "#Cylinder"
                    eulerRotation.x: 90
                    position: Qt.vector3d(0, 0, parent.side * (actuator.depth / 2 + 0.005))
                    scale: Qt.vector3d(0.55 * actuator.radius / root.meshUnit, 0.004 / root.meshUnit,
                                       0.55 * actuator.radius / root.meshUnit)
                    materials: steelMaterial
                }
                // LED status ring.
                Model {
                    position: Qt.vector3d(0, 0, parent.side * (actuator.depth / 2 + 0.0005))
                    geometry: TubeGeometry {
                        closed: true
                        radius: 0.0013
                        radialSegments: 8
                        samplesPerSpan: 3
                        points: {
                            const pts = []
                            const r = actuator.radius * 0.93
                            for (let k = 0; k < 24; k++) {
                                const a = 2 * Math.PI * k / 24
                                pts.push(Qt.vector3d(r * Math.cos(a), r * Math.sin(a), 0))
                            }
                            return pts
                        }
                    }
                    materials: PrincipledMaterial {
                        baseColor: actuator.statusColor
                        emissiveFactor: Qt.vector3d(actuator.statusColor.r, actuator.statusColor.g,
                                                    actuator.statusColor.b)
                        lighting: PrincipledMaterial.NoLighting
                    }
                }
            }
        }
    }

    // A control board: green PCB with a processor, header connectors and a
    // couple of capacitors, standing off its mount on four steel posts.
    // Lies in its local XY plane, components on +Z.
    component Board: Node {
        id: board
        property real w: 0.050
        property real h: 0.024
        property real chipSize: 0.011

        Model {
            geometry: RoundedBoxGeometry { size: Qt.vector3d(board.w, board.h, 0.0016); radius: 0.0006 }
            materials: pcbMaterial
        }
        Model {
            position: Qt.vector3d(-board.w * 0.12, 0, 0.0016)
            geometry: RoundedBoxGeometry { size: Qt.vector3d(board.chipSize, board.chipSize, 0.0018); radius: 0.0004 }
            materials: chipMaterial
        }
        Model {
            position: Qt.vector3d(board.w * 0.32, board.h * 0.18, 0.0028)
            geometry: RoundedBoxGeometry { size: Qt.vector3d(0.010, 0.005, 0.0045); radius: 0.0006 }
            materials: headerMaterial
        }
        Model {
            position: Qt.vector3d(board.w * 0.32, -board.h * 0.24, 0.0022)
            geometry: RoundedBoxGeometry { size: Qt.vector3d(0.008, 0.004, 0.003); radius: 0.0005 }
            materials: headerMaterial
        }
        Repeater3D {
            model: 3
            Model {
                required property int index
                source: "#Cylinder"
                eulerRotation.x: 90
                position: Qt.vector3d(board.w * (0.08 + 0.07 * index), -board.h * 0.3, 0.0028)
                scale: Qt.vector3d(0.003 / root.meshUnit, 0.004 / root.meshUnit, 0.003 / root.meshUnit)
                materials: capMaterial
            }
        }
        Repeater3D {
            model: 4
            Model {
                required property int index
                source: "#Cylinder"
                eulerRotation.x: 90
                position: Qt.vector3d((index % 2 === 0 ? -1 : 1) * board.w * 0.44,
                                      (index < 2 ? -1 : 1) * board.h * 0.36, -0.0025)
                scale: Qt.vector3d(0.0022 / root.meshUnit, 0.005 / root.meshUnit, 0.0022 / root.meshUnit)
                materials: steelMaterial
            }
        }
    }

    // A wire following a smooth curve through `points`.
    component Wire: Model {
        id: wire
        property alias points: wireGeometry.points
        property alias thickness: wireGeometry.radius
        property color color: "#151515"
        geometry: TubeGeometry { id: wireGeometry; radius: 0.0016; radialSegments: 8; samplesPerSpan: 6 }
        materials: PrincipledMaterial { baseColor: wire.color; roughness: 0.5 }
    }

    // ---- shoulder + upper arm ------------------------------------------------

    Node {
        id: upperArm
        visible: root.jointCount >= 1
        position: armController.jointPositions.length > 0 ? armController.jointPositions[0] : Qt.vector3d(0, 0, 0)
        rotation: armController.frameRotations.length > 1 ? armController.frameRotations[1] : Qt.quaternion(1, 0, 0, 0)

        Actuator {
            radius: 0.060
            depth: 0.112
            position: Qt.vector3d(0, 0, -0.012)
            statusColor: root.jointColor(0)
        }

        // Molded upper-arm shell, tapering toward the elbow.
        Model {
            readonly property real start: 0.050
            readonly property real end: Math.max(start + 0.02, root.upperArmLength - 0.030)
            position: Qt.vector3d((start + end) / 2, 0, 0)
            eulerRotation.z: -90 // geometry +Y -> limb +X
            geometry: RoundedBoxGeometry {
                size: Qt.vector3d(0.080, Math.max(0.02, root.upperArmLength - 0.080), 0.074)
                radius: 0.024
                taper: 0.84
            }
            materials: shellMaterial
        }
        // Inset side panel with screws, facing the camera.
        Node {
            position: Qt.vector3d(root.upperArmLength * 0.55, 0, 0.0352)
            Model {
                geometry: RoundedBoxGeometry { size: Qt.vector3d(Math.max(0.02, root.upperArmLength * 0.36), 0.046, 0.004); radius: 0.0018 }
                materials: shellDarkMaterial
            }
            Repeater3D {
                model: 4
                Model {
                    required property int index
                    source: "#Cylinder"
                    eulerRotation.x: 90
                    position: Qt.vector3d((index % 2 === 0 ? -1 : 1) * Math.max(0.006, root.upperArmLength * 0.16),
                                          (index < 2 ? -1 : 1) * 0.017, 0.0022)
                    scale: Qt.vector3d(0.0035 / root.meshUnit, 0.001 / root.meshUnit, 0.0035 / root.meshUnit)
                    materials: steelMaterial
                }
            }
        }
    }

    // ---- elbow + forearm -------------------------------------------------

    Node {
        id: forearm
        visible: root.jointCount >= 2
        position: armController.jointPositions.length > 1 ? armController.jointPositions[1] : Qt.vector3d(0, 0, 0)
        rotation: armController.frameRotations.length > 2 ? armController.frameRotations[2] : Qt.quaternion(1, 0, 0, 0)

        readonly property real length: root.forearmLength
        readonly property real chassisStart: 0.034
        readonly property real chassisEnd: Math.max(chassisStart + 0.03, length - 0.050)
        readonly property real chassisLength: chassisEnd - chassisStart

        Actuator {
            radius: 0.045
            depth: 0.090
            statusColor: root.jointColor(1)
        }

        // Dark aluminum chassis the electronics bolt to.
        Model {
            position: Qt.vector3d((forearm.chassisStart + forearm.chassisEnd) / 2, 0, 0)
            eulerRotation.z: -90
            geometry: RoundedBoxGeometry {
                size: Qt.vector3d(0.056, forearm.chassisLength, 0.048)
                radius: 0.009
                taper: 0.86
            }
            materials: anodizedMaterial
        }

        // White molded cover along the thumb side (+Y).
        Model {
            position: Qt.vector3d((forearm.chassisStart + forearm.chassisEnd) / 2 - 0.004, 0.031, 0)
            eulerRotation.z: -90
            geometry: RoundedBoxGeometry {
                size: Qt.vector3d(0.012, forearm.chassisLength * 0.96, 0.060)
                radius: 0.0055
                taper: 0.9
            }
            materials: whiteCoverMaterial
        }
        // Navy strip along the opposite (little-finger) side.
        Model {
            position: Qt.vector3d((forearm.chassisStart + forearm.chassisEnd) / 2, -0.029, 0)
            eulerRotation.z: -90
            geometry: RoundedBoxGeometry {
                size: Qt.vector3d(0.008, forearm.chassisLength * 0.9, 0.042)
                radius: 0.0035
                taper: 0.9
            }
            materials: navyMaterial
        }

        // Stacked control boards on both faces: two rows, as many columns
        // as fit the forearm's current length, a second layer staggered on
        // top of the first like the reference arm.
        Repeater3D {
            model: 2 // +Z face (toward the camera), -Z face
            Node {
                id: boardFace
                required property int index
                readonly property real side: index === 0 ? 1 : -1
                readonly property int columns: Math.max(1, Math.floor(forearm.chassisLength / 0.054))
                readonly property real pitch: forearm.chassisLength / columns
                eulerRotation.y: side > 0 ? 0 : 180
                position: Qt.vector3d(side > 0 ? 0 : 2 * ((forearm.chassisStart + forearm.chassisEnd) / 2), 0, 0)

                Repeater3D {
                    model: boardFace.columns * 2
                    Board {
                        required property int index
                        readonly property int column: Math.floor(index / 2)
                        readonly property int row: index % 2
                        w: boardFace.pitch * 0.9
                        position: Qt.vector3d(forearm.chassisStart + boardFace.pitch * (column + 0.5),
                                              (row === 0 ? 1 : -1) * 0.0128, 0.0285)
                    }
                }
                Repeater3D {
                    model: Math.max(1, boardFace.columns - 1)
                    Board {
                        required property int index
                        w: boardFace.pitch * 0.82
                        h: 0.030
                        chipSize: 0.009
                        position: Qt.vector3d(forearm.chassisStart + boardFace.pitch * (index + 1), 0.001, 0.0355)
                    }
                }
            }
        }

        // Cable bundles from the elbow, over the boards, into the wrist.
        // The far end of every wire is computed in the HAND's rotated frame
        // so the cables bend with the wrist instead of tearing off it.
        function wristPoint(dx, dy, dz) {
            const c = Math.cos(root.wristRadians)
            const s = Math.sin(root.wristRadians)
            return Qt.vector3d(forearm.length + c * dx - s * dy, s * dx + c * dy, dz)
        }

        Repeater3D {
            model: [
                { y: 0.010, z: 0.045, color: "#3c2457", r: 0.0013, phase: 0.0 },
                { y: 0.010, z: 0.045, color: "#141414", r: 0.0013, phase: Math.PI },
                { y: -0.004, z: 0.044, color: "#3c2457", r: 0.0013, phase: 1.0 },
                { y: -0.004, z: 0.044, color: "#141414", r: 0.0013, phase: 1.0 + Math.PI },
                { y: -0.013, z: 0.043, color: "#a3231b", r: 0.0012, phase: 0.6 },
                { y: -0.017, z: 0.042, color: "#e8e8e8", r: 0.0012, phase: 2.2 },
                { y: 0.002, z: 0.047, color: "#141414", r: 0.0018, phase: 1.3 },
            ]
            Wire {
                required property var modelData
                color: modelData.color
                thickness: modelData.r
                points: {
                    const d = modelData
                    const pts = [Qt.vector3d(0.020, d.y * 0.5, 0.040)]
                    // Twisted pairs wind around a shared centerline.
                    const twist = 0.0016
                    const steps = 7
                    for (let k = 1; k <= steps; k++) {
                        const x = forearm.chassisStart + forearm.chassisLength * k / (steps + 1)
                        const a = d.phase + k * 1.9
                        pts.push(Qt.vector3d(x, d.y + twist * Math.cos(a), d.z + twist * Math.sin(a) + 0.002 * Math.sin(k)))
                    }
                    pts.push(Qt.vector3d(forearm.length - 0.030, d.y * 0.6, d.z - 0.010))
                    pts.push(forearm.wristPoint(-0.004, d.y * 0.5, 0.017))
                    pts.push(forearm.wristPoint(0.008, d.y * 0.4, 0.009))
                    return pts
                }
            }
        }

        // Rainbow ribbon cable into the back of the hand.
        Repeater3D {
            model: ["#3a3a3a", "#7a4b2a", "#c62828", "#ef6c00", "#f9c80e", "#2e7d32", "#1565c0", "#6a1b9a", "#9e9e9e", "#eeeeee"]
            Wire {
                required property string modelData
                required property int index
                color: modelData
                thickness: 0.0008
                readonly property real offset: (index - 4.5) * 0.0017
                points: [
                    Qt.vector3d(forearm.chassisEnd - 0.05, 0.022 + offset * 0.2, 0.030 + offset),
                    Qt.vector3d(forearm.chassisEnd - 0.01, 0.024, 0.034 + offset),
                    Qt.vector3d(forearm.length - 0.012, 0.020, 0.030 + offset),
                    forearm.wristPoint(-0.002, 0.018, 0.016 + offset * 0.6),
                    forearm.wristPoint(0.010, 0.014, 0.006 + offset * 0.5)
                ]
            }
        }

        // Steel wrist clevis: two side plates carrying the wrist pin.
        Repeater3D {
            model: 2
            Node {
                required property int index
                readonly property real side: index === 0 ? 1 : -1
                Model {
                    position: Qt.vector3d(forearm.length - 0.024, 0, side * 0.0265)
                    eulerRotation.z: -90
                    geometry: RoundedBoxGeometry { size: Qt.vector3d(0.030, 0.062, 0.0045); radius: 0.002; taper: 0.75 }
                    materials: steelMaterial
                }
                // Bent support tube from the chassis to the clevis.
                Model {
                    geometry: TubeGeometry {
                        radius: 0.0032
                        points: [
                            Qt.vector3d(forearm.chassisEnd - 0.06, side * 0.024, side * 0.020),
                            Qt.vector3d(forearm.chassisEnd - 0.02, side * 0.030, side * 0.024),
                            Qt.vector3d(forearm.length - 0.030, side * 0.020, side * 0.026),
                            Qt.vector3d(forearm.length - 0.010, side * 0.008, side * 0.026)
                        ]
                    }
                    materials: steelMaterial
                }
            }
        }
        // Wrist pin + small pitch actuator.
        Model {
            position: Qt.vector3d(forearm.length, 0, 0)
            source: "#Cylinder"
            eulerRotation.x: 90
            scale: Qt.vector3d(0.012 / root.meshUnit, 0.066 / root.meshUnit, 0.012 / root.meshUnit)
            materials: darkSteelMaterial
        }
        Actuator {
            position: Qt.vector3d(forearm.length - 0.026, 0, 0)
            radius: 0.016
            depth: 0.044
            statusColor: root.jointColor(2)
        }
    }

    // ---- wrist mount for the hand ------------------------------------------

    Node {
        id: handMountNode
        position: armController.jointPositions.length > 2 ? armController.jointPositions[2] : Qt.vector3d(0, 0, 0)
        rotation: armController.frameRotations.length > 3 ? armController.frameRotations[3] : Qt.quaternion(1, 0, 0, 0)

        // Arm frame -> hand frame: fingers along the hand link (+X), palm
        // facing away from the camera (-Z), thumb up (+Y). That's a -90
        // degree turn about X.
        Hand3D {
            eulerRotation.x: -90
            holdingPayload: !armController.gripperOpen
            payloadKg: armController.payloadKg
        }
    }
}
