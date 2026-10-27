import QtQuick
import QtQuick3D
import QtQuick3D.Helpers

// Basic geometry only (cylinders for links, spheres for joints) - the
// "looks like a real hand" visual polish is Phase 3, deliberately
// deferred. All positions/rotations come from ArmController, computed
// from the exact DH forward kinematics (or the AI surrogate, depending
// on armController.aiMode) - this view just places primitives.
Item {
    View3D {
        anchors.fill: parent

        environment: SceneEnvironment {
            clearColor: "#1a1a2e"
            backgroundMode: SceneEnvironment.Color
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
        }

        // Camera orbits around the arm's reach midpoint, not the world
        // origin (0,0,0) - per the roadmap's prior-art finding that
        // world-origin orbiting is frustrating once the arm extends away
        // from it. See ArmController::canvasCenter().
        Node {
            id: cameraOrigin
            position: armController.canvasCenter
            eulerRotation: Qt.vector3d(-25, 25, 0)

            PerspectiveCamera {
                id: camera
                z: 4
                clipNear: 0.01
            }
        }

        OrbitCameraController {
            anchors.fill: parent
            origin: cameraOrigin
            camera: camera
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(-45, -45, 0)
            brightness: 1.2
        }

        // A thin disc marking the base/world origin, for visual reference.
        Model {
            source: "#Cylinder"
            position: Qt.vector3d(0, -0.01, 0)
            // Built-in primitives are 100 units in local space; dividing by
            // 100 maps local units to scene meters.
            scale: Qt.vector3d(2.0, 0.002, 2.0)
            materials: PrincipledMaterial {
                baseColor: "#444455"
                roughness: 0.8
            }
        }

        Repeater3D {
            model: armController.linkLengths.length

            Model {
                required property int index
                source: "#Cylinder"
                position: armController.linkMidpoints[index]
                rotation: armController.linkRotations[index]
                scale: Qt.vector3d(0.06, armController.linkLengths[index] / 100.0, 0.06)

                materials: PrincipledMaterial {
                    baseColor: armController.jointColors[index]
                    metalness: 0.3
                    roughness: 0.4
                }
            }
        }

        Repeater3D {
            model: armController.jointPositions.length

            Model {
                required property int index
                source: "#Sphere"
                position: armController.jointPositions[index]
                scale: Qt.vector3d(0.08, 0.08, 0.08)

                materials: PrincipledMaterial {
                    baseColor: "#dddddd"
                    metalness: 0.5
                    roughness: 0.3
                }
            }
        }

        // Payload marker at the end-effector, sized (loosely) by mass.
        Model {
            readonly property int lastIndex: armController.jointPositions.length - 1
            source: "#Cube"
            visible: armController.payloadKg > 0 && lastIndex >= 0
            position: lastIndex >= 0 ? armController.jointPositions[lastIndex] : Qt.vector3d(0, 0, 0)
            scale: {
                const s = 0.08 + armController.payloadKg * 0.01
                return Qt.vector3d(s, s, s)
            }

            materials: PrincipledMaterial {
                baseColor: "#e94560"
                metalness: 0.1
                roughness: 0.6
            }
        }
    }
}
