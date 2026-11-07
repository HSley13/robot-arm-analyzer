pragma ComponentBehavior: Bound
import QtQuick
import QtQuick3D
import Humanoid.Geometry

// The humanoid's right arm from shoulder to wrist, as a clean simulation
// model: molded shells over every segment (no exposed electronics or
// wiring - none of that exists in a simulation), round actuator housings
// at the shoulder, elbow and wrist with load-status rings, a rounded
// deltoid cap, rubber seal bands at the joints, inset service panels and
// screw heads.
//
// Placement comes straight from armController's forward kinematics.
// Each limb is a Node at its joint, rotated by that joint's full frame
// (frameRotations), so inside it:
//   +X runs along the limb toward the next joint,
//   +Y is the in-plane side (the thumb side, for the neutral grip),
//   +Z is out of the arm's plane, away from the body (the back of the
//      hand faces this way; the palm faces -Z, toward the body).
// Limb lengths are read live, so dragging a "len" slider stretches the
// shells instead of leaving gaps.
//
// The hand (Hand3D) hangs off the wrist pivot, turned so its own frame
// (+X fingers, +Y palm, +Z thumb) holds the neutral, thumb-up grip a
// person uses to carry a bottle or a bag.
Node {
    id: root

    readonly property real meshUnit: 100.0
    readonly property int jointCount: armController.jointPositions.length - 1

    function linkLength(i) { return i < armController.linkLengths.length ? armController.linkLengths[i] : 0.0 }
    function jointColor(i) { return i < armController.jointColors.length ? String(armController.jointColors[i]) : "#2ecc71" }

    // Load status on the shells: untouched while a joint is safe, washed
    // amber at caution and red at overload, so a struggling joint shows
    // from across the room - not just on its LED ring.
    function isSafe(i) { return jointColor(i).toLowerCase() === "#2ecc71" }
    function tinted(base, i, amount) {
        if (isSafe(i)) return base
        const alpha = Math.round(amount * 255).toString(16).padStart(2, "0")
        return Qt.tint(base, "#" + alpha + jointColor(i).substring(1))
    }

    readonly property real upperArmLength: linkLength(0)
    readonly property real forearmLength: linkLength(1)

    // ---- materials -------------------------------------------------------

    PrincipledMaterial { id: shellMaterial; baseColor: "#7f91a3"; roughness: 0.46; specularAmount: 0.45 }
    PrincipledMaterial { id: shellDarkMaterial; baseColor: "#5d6f82"; roughness: 0.52 }
    PrincipledMaterial { id: jointMaterial; baseColor: "#3a4552"; roughness: 0.5; metalness: 0.15 }
    PrincipledMaterial { id: rubberMaterial; baseColor: "#262d35"; roughness: 0.9 }
    PrincipledMaterial { id: steelMaterial; baseColor: "#c9ced4"; metalness: 1.0; roughness: 0.26 }
    PrincipledMaterial { id: holeMaterial; baseColor: "#161b21"; roughness: 0.9 }

    // ---- reusable pieces -------------------------------------------------

    // A round actuator housing whose axis is local Z: a dark body, a
    // stepped face on each side with a steel hub, and an LED ring on the
    // outer (+Z) face that glows with the joint's load status.
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
            materials: PrincipledMaterial {
                baseColor: Qt.tint("#3a4552", Qt.rgba(actuator.statusColor.r, actuator.statusColor.g,
                                                      actuator.statusColor.b, 0.35))
                roughness: 0.5
                metalness: 0.15
            }
        }
        Repeater3D {
            model: 2
            Node {
                required property int index
                readonly property real side: index === 0 ? 1 : -1
                // Stepped face plate.
                Model {
                    source: "#Cylinder"
                    eulerRotation.x: 90
                    position: Qt.vector3d(0, 0, parent.side * (actuator.depth / 2 + 0.003))
                    scale: Qt.vector3d(1.8 * actuator.radius / root.meshUnit, 0.006 / root.meshUnit,
                                       1.8 * actuator.radius / root.meshUnit)
                    materials: shellDarkMaterial
                }
                // Steel hub.
                Model {
                    source: "#Cylinder"
                    eulerRotation.x: 90
                    position: Qt.vector3d(0, 0, parent.side * (actuator.depth / 2 + 0.007))
                    scale: Qt.vector3d(0.6 * actuator.radius / root.meshUnit, 0.004 / root.meshUnit,
                                       0.6 * actuator.radius / root.meshUnit)
                    materials: steelMaterial
                }
                // Six bolt heads around the hub.
                Repeater3D {
                    model: 6
                    Model {
                        required property int index
                        readonly property real angle: index * Math.PI / 3
                        source: "#Cylinder"
                        eulerRotation.x: 90
                        position: Qt.vector3d(Math.cos(angle) * actuator.radius * 0.5,
                                              Math.sin(angle) * actuator.radius * 0.5,
                                              parent.side * (actuator.depth / 2 + 0.0068))
                        scale: Qt.vector3d(actuator.radius * 0.11 / root.meshUnit, 0.002 / root.meshUnit,
                                           actuator.radius * 0.11 / root.meshUnit)
                        materials: holeMaterial
                    }
                }
            }
        }
        // LED status ring on the outer face.
        Model {
            position: Qt.vector3d(0, 0, actuator.depth / 2 + 0.0062)
            geometry: TubeGeometry {
                closed: true
                radius: Math.max(0.0028, actuator.radius * 0.06)
                radialSegments: 8
                samplesPerSpan: 3
                points: {
                    const pts = []
                    const r = actuator.radius * 0.76
                    for (let k = 0; k < 24; k++) {
                        const a = 2 * Math.PI * k / 24
                        pts.push(Qt.vector3d(r * Math.cos(a), r * Math.sin(a), 0))
                    }
                    return pts
                }
            }
            materials: PrincipledMaterial {
                baseColor: actuator.statusColor
                emissiveFactor: Qt.vector3d(actuator.statusColor.r, actuator.statusColor.g, actuator.statusColor.b)
                lighting: PrincipledMaterial.NoLighting
            }
        }
    }

    // A tapered limb shell lying along +X from `start` to `end`.
    // RoundedBoxGeometry tapers along its own +Y, so it's built with +Y as
    // the limb's length and turned -90 degrees about Z.
    component LimbShell: Model {
        id: shell
        property real start: 0.0
        property real end: 0.1
        property real width: 0.08     // in-plane (Y) at the start
        property real thickness: 0.07 // out-of-plane (Z) at the start
        property real taper: 0.85
        property real roundness: 0.3
        property color color: "#7f91a3"
        position: Qt.vector3d((start + end) / 2, 0, 0)
        eulerRotation.z: -90
        geometry: RoundedBoxGeometry {
            size: Qt.vector3d(shell.width, Math.max(0.01, shell.end - shell.start), shell.thickness)
            radius: Math.min(shell.width, shell.thickness) * shell.roundness
            taper: shell.taper
        }
        materials: PrincipledMaterial { baseColor: shell.color; roughness: 0.46; specularAmount: 0.45 }
    }

    // A rubber seal band around a limb at `x`.
    component SealBand: Model {
        id: band
        property real x: 0
        property real width: 0.07
        property real thickness: 0.065
        position: Qt.vector3d(band.x, 0, 0)
        eulerRotation.z: -90
        geometry: RoundedBoxGeometry {
            size: Qt.vector3d(band.width, 0.012, band.thickness)
            radius: Math.min(band.width, band.thickness) * 0.32
        }
        materials: rubberMaterial
    }

    // An inset service panel with four screws on a limb's outer (+Z)
    // face, centered at (x, 0, z).
    component ServicePanel: Node {
        id: panel
        property real length: 0.08
        property real height: 0.04
        Model {
            geometry: RoundedBoxGeometry { size: Qt.vector3d(panel.length, panel.height, 0.004); radius: 0.0017 }
            materials: shellDarkMaterial
        }
        Repeater3D {
            model: 4
            Model {
                required property int index
                source: "#Cylinder"
                eulerRotation.x: 90
                position: Qt.vector3d((index % 2 === 0 ? -1 : 1) * (panel.length / 2 - 0.007),
                                      (index < 2 ? -1 : 1) * (panel.height / 2 - 0.006), 0.0022)
                scale: Qt.vector3d(0.0034 / root.meshUnit, 0.001 / root.meshUnit, 0.0034 / root.meshUnit)
                materials: steelMaterial
            }
        }
    }

    // ---- shoulder + upper arm ------------------------------------------------

    Node {
        id: upperArm
        visible: root.jointCount >= 1
        position: armController.jointPositions.length > 0 ? armController.jointPositions[0] : Qt.vector3d(0, 0, 0)
        rotation: armController.frameRotations.length > 1 ? armController.frameRotations[1] : Qt.quaternion(1, 0, 0, 0)

        Actuator {
            radius: 0.056
            depth: 0.118
            position: Qt.vector3d(0, 0, -0.004)
            statusColor: root.jointColor(0)
        }

        // Deltoid cap: a big rounded shell over the top of the shoulder that
        // turns with the upper arm, like a shoulder muscle.
        LimbShell {
            start: -0.055
            end: 0.115
            width: 0.128
            thickness: 0.112
            taper: 0.78
            roundness: 0.46
            position: Qt.vector3d((start + end) / 2, 0.006, -0.004)
            color: root.tinted("#7f91a3", 0, 0.45)
        }

        LimbShell {
            start: 0.07
            end: Math.max(start + 0.02, root.upperArmLength - 0.040)
            width: 0.084
            thickness: 0.078
            taper: 0.84
            roundness: 0.34
            color: root.tinted("#7f91a3", 0, 0.45)
        }
        // Back-of-arm (tricep) panel for some shape.
        LimbShell {
            start: 0.10
            end: Math.max(start + 0.02, root.upperArmLength * 0.72)
            width: 0.05
            thickness: 0.07
            taper: 0.8
            roundness: 0.4
            position: Qt.vector3d((start + end) / 2, -0.024, 0)
            color: root.tinted("#5d6f82", 0, 0.45)
        }

        ServicePanel {
            length: Math.max(0.03, root.upperArmLength * 0.28)
            height: 0.044
            position: Qt.vector3d(root.upperArmLength * 0.52, 0.004, 0.0372 - root.upperArmLength * 0.004)
        }

        SealBand {
            x: root.upperArmLength - 0.052
            width: 0.074
            thickness: 0.069
        }
    }

    // ---- elbow + forearm -------------------------------------------------

    Node {
        id: forearm
        visible: root.jointCount >= 2
        position: armController.jointPositions.length > 1 ? armController.jointPositions[1] : Qt.vector3d(0, 0, 0)
        rotation: armController.frameRotations.length > 2 ? armController.frameRotations[2] : Qt.quaternion(1, 0, 0, 0)

        readonly property real length: root.forearmLength

        Actuator {
            radius: 0.046
            depth: 0.096
            statusColor: root.jointColor(1)
        }

        // Upper forearm: fuller near the elbow, like the forearm muscles.
        LimbShell {
            start: 0.030
            end: Math.min(forearm.length * 0.55, 0.15)
            width: 0.080
            thickness: 0.074
            taper: 0.86
            roundness: 0.42
            color: root.tinted("#7f91a3", 1, 0.45)
        }
        // Main forearm shell, narrowing to the wrist.
        LimbShell {
            start: 0.045
            end: Math.max(start + 0.02, forearm.length - 0.034)
            width: 0.068
            thickness: 0.062
            taper: 0.72
            roundness: 0.36
            color: root.tinted("#7f91a3", 1, 0.45)
        }

        ServicePanel {
            length: Math.max(0.03, forearm.length * 0.26)
            height: 0.034
            position: Qt.vector3d(forearm.length * 0.52, 0.002, 0.0285)
        }

        SealBand {
            x: forearm.length - 0.044
            width: 0.054
            thickness: 0.050
        }

        // Wrist housing: covers the wrist pitch actuator and bridges into the
        // hand's cuff.
        Model {
            position: Qt.vector3d(forearm.length - 0.012, 0, 0)
            eulerRotation.z: -90
            geometry: RoundedBoxGeometry { size: Qt.vector3d(0.050, 0.034, 0.046); radius: 0.013 }
            materials: PrincipledMaterial { baseColor: root.tinted("#5d6f82", 2, 0.5); roughness: 0.52 }
        }
        Actuator {
            position: Qt.vector3d(forearm.length, 0, 0)
            radius: 0.020
            depth: 0.052
            statusColor: root.jointColor(2)
        }
    }

    // ---- wrist mount for the hand ------------------------------------------

    Node {
        id: handMountNode
        position: armController.jointPositions.length > 2 ? armController.jointPositions[2] : Qt.vector3d(0, 0, 0)
        rotation: armController.frameRotations.length > 3 ? armController.frameRotations[3] : Qt.quaternion(1, 0, 0, 0)

        // Arm frame -> hand frame: fingers along the hand link (+X), palm
        // facing the body (-Z), thumb up (+Y). That's a -90 degree turn
        // about X.
        Hand3D {
            eulerRotation.x: -90
            // The demo dumbbell from the Payload card - hidden while the hand
            // is carrying one of the scene's objects instead.
            holdingPayload: !armController.gripperOpen && taskController.heldId < 0
            payloadKg: armController.payloadKg
            heldObject: taskController.heldObject
        }
    }
}
