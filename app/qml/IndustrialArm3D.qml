pragma ComponentBehavior: Bound
import QtQuick
import QtQuick3D
import Humanoid.Geometry

// The warehouse version of the right arm: any number of joints, each a
// round servo drum with a load-status band, joined by safety-yellow box
// links, ending in a flange and a parallel two-finger gripper.
//
// Same frames as HumanoidArm3D: link i is a Node at joint i, turned by
// frameRotations[i + 1], so inside it +X runs along the link, +Y is the
// in-plane side and +Z points out of the arm's plane. The gripper's
// fingers close along Z, across the plane, and their center sits
// `gripperReach` past the flange - where core/pick_planner puts the grip
// point.
Node {
    id: root

    readonly property real meshUnit: 100.0
    readonly property int jointCount: Math.max(0, armController.jointPositions.length - 1)
    readonly property var held: taskController.heldObject
    readonly property real gripperReach: held.gripperReach !== undefined ? held.gripperReach : 0.075

    function linkLength(i) { return i < armController.linkLengths.length ? armController.linkLengths[i] : 0.0 }
    function jointColor(i) { return i < armController.jointColors.length ? String(armController.jointColors[i]) : "#2ecc71" }
    function isSafe(i) { return jointColor(i).toLowerCase() === "#2ecc71" }
    // Same load wash as the humanoid arm: none when safe.
    function tinted(base, i, amount) {
        if (isSafe(i)) return base
        const alpha = Math.round(amount * 255).toString(16).padStart(2, "0")
        return Qt.tint(base, "#" + alpha + jointColor(i).substring(1))
    }
    // Drums get smaller toward the gripper, like a real arm's motors.
    function drumRadius(i) { return Math.max(0.032, 0.066 - i * 0.008) }

    PrincipledMaterial { id: darkMaterial; baseColor: "#2b3138"; roughness: 0.55; metalness: 0.2 }
    PrincipledMaterial { id: steelMaterial; baseColor: "#c9ced4"; metalness: 1.0; roughness: 0.28 }
    PrincipledMaterial { id: padMaterial; baseColor: "#1c2025"; roughness: 0.95 }

    component Drum: Model {
        // A cylinder lying along local Z.
        property real radius: 0.05
        property real depth: 0.05
        source: "#Cylinder"
        eulerRotation.x: 90
        scale: Qt.vector3d(2 * radius / root.meshUnit, depth / root.meshUnit, 2 * radius / root.meshUnit)
        materials: darkMaterial
    }

    // ---- joints + links ---------------------------------------------------

    Repeater3D {
        model: root.jointCount

        Node {
            id: link
            required property int index
            readonly property real length: root.linkLength(index)
            readonly property real radius: root.drumRadius(index)
            readonly property real nextRadius: root.drumRadius(index + 1)
            readonly property bool last: index === root.jointCount - 1
            readonly property color status: root.jointColor(index)

            position: index < armController.jointPositions.length ? armController.jointPositions[index]
                                                                  : Qt.vector3d(0, 0, 0)
            rotation: index + 1 < armController.frameRotations.length ? armController.frameRotations[index + 1]
                                                                      : Qt.quaternion(1, 0, 0, 0)

            // Servo drum with steel end caps.
            Drum { radius: link.radius; depth: 2.0 * link.radius + 0.02 }
            Repeater3D {
                model: 2
                Drum {
                    required property int index
                    radius: link.radius * 0.55
                    depth: 0.008
                    position: Qt.vector3d(0, 0, (index === 0 ? 1 : -1) * (link.radius + 0.012))
                    materials: steelMaterial
                }
            }
            // Load-status band around the drum - glows its joint's color.
            Drum {
                radius: link.radius + 0.003
                depth: 0.016
                materials: PrincipledMaterial {
                    baseColor: link.status
                    emissiveFactor: Qt.vector3d(link.status.r * 0.8, link.status.g * 0.8, link.status.b * 0.8)
                    lighting: PrincipledMaterial.NoLighting
                }
            }

            // The link: a box beam from this drum to the next (the last
            // one stops at the flange), tapering toward the next joint.
            Model {
                id: beam
                readonly property real start: link.radius * 0.6
                readonly property real end: link.last ? link.length - 0.012 : link.length - link.nextRadius * 0.6
                visible: end - start > 0.01
                position: Qt.vector3d((start + end) / 2, 0, 0)
                eulerRotation.z: -90
                geometry: RoundedBoxGeometry {
                    size: Qt.vector3d(1.5 * link.radius, Math.max(0.01, beam.end - beam.start), 1.3 * link.radius)
                    radius: link.radius * 0.35
                    taper: Math.min(1.0, link.last ? 0.8 : link.nextRadius / link.radius + 0.1)
                }
                materials: PrincipledMaterial {
                    baseColor: root.tinted("#f2b233", link.index, 0.55)
                    roughness: 0.4
                    specularAmount: 0.5
                }
            }
            // Dark cable-guard strip along the top of the beam.
            Model {
                visible: link.length > 3 * link.radius
                position: Qt.vector3d(link.length / 2, 0.66 * link.radius, 0)
                geometry: RoundedBoxGeometry {
                    size: Qt.vector3d(Math.max(0.01, link.length - 2.6 * link.radius), 0.012, 0.5 * link.radius)
                    radius: 0.004
                }
                materials: darkMaterial
            }

            // ---- tool flange + gripper on the last link --------------------
            Node {
                visible: link.last
                position: Qt.vector3d(link.length, 0, 0)

                // Flange (a disk facing +X).
                Model {
                    source: "#Cylinder"
                    eulerRotation.z: 90
                    position: Qt.vector3d(-0.006, 0, 0)
                    scale: Qt.vector3d(0.07 / root.meshUnit, 0.012 / root.meshUnit, 0.07 / root.meshUnit)
                    materials: steelMaterial
                }
                // Gripper body: the jaw rail runs along Z.
                Model {
                    position: Qt.vector3d(0.02, 0, 0)
                    geometry: RoundedBoxGeometry { size: Qt.vector3d(0.04, 0.05, 0.13); radius: 0.008 }
                    materials: darkMaterial
                }
                // Two fingers, sliding along Z. Each is a plate whose inner
                // face is half the gripper width from the center.
                Repeater3D {
                    model: 2
                    Node {
                        required property int index
                        readonly property real side: index === 0 ? 1 : -1
                        readonly property real gap: Math.max(0.004, taskController.gripperWidth)
                        position: Qt.vector3d(0, 0, side * (gap / 2 + 0.008))
                        Behavior on position { Vector3dAnimation { duration: 220 } }
                        Model {
                            position: Qt.vector3d(0.04 + root.gripperReach / 2, 0, 0)
                            geometry: RoundedBoxGeometry {
                                size: Qt.vector3d(root.gripperReach + 0.035, 0.036, 0.016)
                                radius: 0.004
                            }
                            materials: steelMaterial
                        }
                        // Rubber pad on the inner face.
                        Model {
                            position: Qt.vector3d(root.gripperReach + 0.004, 0, -parent.side * 0.0095)
                            geometry: RoundedBoxGeometry { size: Qt.vector3d(0.04, 0.03, 0.004); radius: 0.0015 }
                            materials: padMaterial
                        }
                    }
                }

                // The held object, its grip point between the finger pads.
                // From above ("top"): the gripper points down, so the
                // object's up axis is -X (a +90 degree turn about Z);
                // level ("side"): its up axis is the in-plane +Y.
                SceneObject3D {
                    readonly property bool top: root.held.graspMode === "top"
                    readonly property real gripHeight: root.held.graspHeight !== undefined ? root.held.graspHeight : 0
                    visible: root.held.id !== undefined
                    type: root.held.type !== undefined ? root.held.type : "box"
                    mass: root.held.mass !== undefined ? root.held.mass : 1
                    gripRadius: root.held.gripRadius !== undefined ? root.held.gripRadius : 0.03
                    footprint: root.held.footprint !== undefined ? root.held.footprint : 0.03
                    objectHeight: root.held.height !== undefined ? root.held.height : 0.1
                    showSelection: false
                    eulerRotation.z: top ? 90 : 0
                    position: top ? Qt.vector3d(root.gripperReach + gripHeight, 0, 0)
                                  : Qt.vector3d(root.gripperReach, -gripHeight, 0)
                }
            }
        }
    }
}
