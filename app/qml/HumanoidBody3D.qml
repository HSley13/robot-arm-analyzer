pragma ComponentBehavior: Bound
import QtQuick
import QtQuick3D
import Humanoid.Geometry

// The humanoid's upper body: pelvis, a hip joint that lets the torso lean
// forward, segmented abdomen, chest, shoulder yoke, neck and head, a
// passive left arm, and the analyzed right arm (HumanoidArm3D) at the
// right shoulder.
//
// Frames (all from taskController):
//   * this node: at the hip joint, turned by the waist yaw - the pelvis
//     lives here;
//   * `torso`: leans forward by the hip bend (a turn of -bend about Z);
//     the shoulders are hipToShoulder above the hip, +/- shoulderLateral
//     to the sides.
// The right arm's plane is the torso's x-y plane at z = +shoulderLateral,
// which is exactly how core/pick_planner places it - so a planned grasp
// lines up with what's drawn.
Node {
    id: root

    readonly property real meshUnit: 100.0
    readonly property real hipToShoulder: taskController.body.hipToShoulder
    readonly property real shoulderLateral: taskController.body.shoulderLateral
    readonly property real bend: taskController.hipBendDegrees

    // Exposed so ArmView3D can project the arm's joints to the screen.
    property alias armNode: rightArmMount

    position: Qt.vector3d(0, taskController.hipHeight, 0)
    eulerRotation.y: taskController.waistYawDegrees

    PrincipledMaterial { id: shellMaterial; baseColor: "#7f91a3"; roughness: 0.46; specularAmount: 0.45 }
    PrincipledMaterial { id: shellDarkMaterial; baseColor: "#5d6f82"; roughness: 0.52 }
    PrincipledMaterial { id: jointMaterial; baseColor: "#3a4552"; roughness: 0.5; metalness: 0.15 }
    PrincipledMaterial { id: rubberMaterial; baseColor: "#262d35"; roughness: 0.9 }
    PrincipledMaterial { id: steelMaterial; baseColor: "#c9ced4"; metalness: 1.0; roughness: 0.26 }
    PrincipledMaterial { id: visorMaterial; baseColor: "#07090d"; roughness: 0.08; specularAmount: 1.0; metalness: 0.2 }
    PrincipledMaterial {
        id: glowMaterial
        baseColor: "#17e0ff"
        emissiveFactor: Qt.vector3d(0.09, 0.88, 1.0)
        lighting: PrincipledMaterial.NoLighting
    }

    component Box: Model {
        id: box
        property vector3d size: Qt.vector3d(0.1, 0.1, 0.1)
        property real roundness: 0.25
        property real taper: 1.0
        geometry: RoundedBoxGeometry {
            size: box.size
            radius: Math.min(box.size.x, box.size.y, box.size.z) * box.roundness
            taper: box.taper
        }
        materials: shellMaterial
    }

    component Drum: Model {
        // A cylinder lying along local Z.
        property real radius: 0.05
        property real depth: 0.05
        source: "#Cylinder"
        eulerRotation.x: 90
        scale: Qt.vector3d(2 * radius / root.meshUnit, depth / root.meshUnit, 2 * radius / root.meshUnit)
        materials: jointMaterial
    }

    // ---- pelvis + hip joint (turns with the waist, doesn't lean) ----------

    Box {
        size: Qt.vector3d(0.17, 0.11, 0.30)
        roundness: 0.35
        position: Qt.vector3d(0, -0.045, 0)
        materials: shellDarkMaterial
    }
    // Hip actuator: the axis the torso leans about.
    Drum { radius: 0.058; depth: 0.33 }
    Repeater3D {
        model: 2
        Drum {
            required property int index
            radius: 0.044
            depth: 0.008
            position: Qt.vector3d(0, 0, (index === 0 ? 1 : -1) * 0.169)
            materials: steelMaterial
        }
    }
    // Waist turntable on top of the lift column.
    Model {
        source: "#Cylinder"
        position: Qt.vector3d(0, -0.105, 0)
        scale: Qt.vector3d(0.15 / root.meshUnit, 0.02 / root.meshUnit, 0.15 / root.meshUnit)
        materials: steelMaterial
    }

    // ---- torso (leans at the hip) -----------------------------------------

    Node {
        id: torso
        eulerRotation.z: -root.bend

        // Segmented abdomen - reads as a flexible spine.
        Repeater3D {
            model: 3
            Box {
                required property int index
                size: Qt.vector3d(0.145 - index * 0.006, 0.044, 0.235 - index * 0.012)
                roundness: 0.4
                position: Qt.vector3d(0.004, 0.075 + index * 0.05, 0)
                materials: index === 1 ? rubberMaterial : shellDarkMaterial
            }
        }

        // Chest, wider at the shoulders.
        Box {
            size: Qt.vector3d(0.20, 0.25, 0.30)
            roundness: 0.28
            taper: 1.16
            position: Qt.vector3d(0.004, 0.31, 0)
        }
        // Chest plate with a status light strip.
        Box {
            size: Qt.vector3d(0.016, 0.15, 0.20)
            roundness: 0.3
            position: Qt.vector3d(0.104, 0.32, 0)
            materials: shellDarkMaterial
        }
        Box {
            size: Qt.vector3d(0.004, 0.007, 0.12)
            roundness: 0.4
            position: Qt.vector3d(0.113, 0.36, 0)
            materials: glowMaterial
        }
        // Back pack (battery).
        Box {
            size: Qt.vector3d(0.07, 0.22, 0.22)
            roundness: 0.3
            position: Qt.vector3d(-0.115, 0.30, 0)
            materials: shellDarkMaterial
        }
        // Shoulder yoke between the two shoulder actuators.
        Box {
            size: Qt.vector3d(0.13, 0.085, 2 * root.shoulderLateral - 0.08)
            roundness: 0.38
            position: Qt.vector3d(0, root.hipToShoulder - 0.005, 0)
        }

        // Neck + head.
        Model {
            source: "#Cylinder"
            position: Qt.vector3d(0, root.hipToShoulder + 0.065, 0)
            scale: Qt.vector3d(0.07 / root.meshUnit, 0.07 / root.meshUnit, 0.07 / root.meshUnit)
            materials: jointMaterial
        }
        Node {
            position: Qt.vector3d(0.005, root.hipToShoulder + 0.19, 0)
            Box {
                size: Qt.vector3d(0.17, 0.19, 0.16)
                roundness: 0.42
            }
            // Visor.
            Box {
                size: Qt.vector3d(0.03, 0.075, 0.135)
                roundness: 0.4
                position: Qt.vector3d(0.072, 0.006, 0)
                materials: visorMaterial
            }
            // Eye light behind the visor glass.
            Box {
                size: Qt.vector3d(0.003, 0.009, 0.085)
                roundness: 0.4
                position: Qt.vector3d(0.087, 0.008, 0)
                materials: glowMaterial
            }
            // Ear pods.
            Repeater3D {
                model: 2
                Drum {
                    required property int index
                    radius: 0.032
                    depth: 0.02
                    position: Qt.vector3d(-0.005, 0, (index === 0 ? 1 : -1) * 0.083)
                    materials: shellDarkMaterial
                }
            }
        }

        // ---- left arm: passive, hanging under gravity ----------------------
        // Counter-turned by half the bend (it swings forward a little as the
        // torso leans), and the elbow folds up as the shoulder gets close to
        // the floor so the hand never sinks into the base.
        Node {
            readonly property real shoulderHeight: taskController.hipHeight
                                                   + root.hipToShoulder * Math.cos(root.bend * Math.PI / 180)
            readonly property real elbowFold: Math.min(110, 18 + Math.max(0, 1.0 - shoulderHeight) * 180)
            position: Qt.vector3d(0, root.hipToShoulder, -root.shoulderLateral)
            eulerRotation.z: root.bend * 0.5

            Drum { radius: 0.056; depth: 0.118; position: Qt.vector3d(0, 0, 0.004) }
            Drum { radius: 0.05; depth: 0.006; position: Qt.vector3d(0, 0, -0.066); materials: shellDarkMaterial }
            // Deltoid.
            Box {
                size: Qt.vector3d(0.128, 0.17, 0.112)
                roundness: 0.46
                taper: 0.78
                eulerRotation.z: 180
                position: Qt.vector3d(0.006, -0.03, 0.004)
            }
            // Upper arm.
            Box {
                size: Qt.vector3d(0.084, 0.19, 0.078)
                roundness: 0.34
                taper: 0.84
                eulerRotation.z: 180
                position: Qt.vector3d(0, -0.165, 0)
            }
            // Elbow, then the forearm angled slightly forward.
            Node {
                position: Qt.vector3d(0, -0.30, 0)
                eulerRotation.z: parent.elbowFold
                Drum { radius: 0.046; depth: 0.096 }
                Box {
                    size: Qt.vector3d(0.072, 0.21, 0.066)
                    roundness: 0.36
                    taper: 0.74
                    eulerRotation.z: 180
                    position: Qt.vector3d(0, -0.14, 0)
                }
                // Relaxed hand, palm toward the body (+Z), thumb forward.
                Node {
                    position: Qt.vector3d(0, -0.27, 0)
                    Box {
                        size: Qt.vector3d(0.078, 0.09, 0.032)
                        roundness: 0.32
                        position: Qt.vector3d(0, -0.045, 0)
                    }
                    Box {
                        size: Qt.vector3d(0.072, 0.06, 0.028)
                        roundness: 0.42
                        eulerRotation.x: -28
                        position: Qt.vector3d(-0.002, -0.108, 0.01)
                    }
                    Box {
                        size: Qt.vector3d(0.024, 0.055, 0.024)
                        roundness: 0.42
                        eulerRotation.z: 22
                        position: Qt.vector3d(0.046, -0.05, 0.012)
                    }
                }
            }
        }

        // ---- right arm (the analyzed one) -----------------------------------
        Node {
            id: rightArmMount
            position: Qt.vector3d(0, root.hipToShoulder, root.shoulderLateral)

            // The humanoid arm + hand, or - once joints are added or removed -
            // the warehouse arm with a gripper.
            HumanoidArm3D { visible: armController.armStyle !== "industrial" }
            IndustrialArm3D { visible: armController.armStyle === "industrial" }
        }
    }
}
