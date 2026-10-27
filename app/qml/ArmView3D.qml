pragma ComponentBehavior: Bound
import QtQuick
import QtQuick3D
import QtQuick3D.Helpers

// Phase 3, Path A: procedural geometry stays exactly as Phase 1 built it
// (cylinders for links, spheres for joints) - per the roadmap, the actual
// rigged-mesh "looks like a real hand" work (Path B) is a genuinely
// different skill set and a stretch goal, not required here. What
// changes in this module is materials and lighting: PBR material
// parameters tuned per part, a two-light key/fill rig with shadows, and
// ambient occlusion/tonemapping on the environment - no textures or 3D
// modeling software needed for any of it.
Item {
    id: root

    // Qt Quick 3D's built-in primitives (#Cube, #Sphere, #Cylinder, ...)
    // are modeled at a fixed 100-unit native size, regardless of the
    // scene's own unit choice (here, 1 scene unit = 1 meter, matching the
    // physics engine's positions directly). Every Model.scale below is
    // (desired meters) / meshUnit - unlike `position`, which is already
    // in meters and needs no conversion. Forgetting this on a radius/size
    // value (while remembering it for a link's length) is exactly the bug
    // that first made the joints/payload render many meters wide next to
    // a ~1.5m arm.
    readonly property real meshUnit: 100.0

    View3D {
        anchors.fill: parent

        environment: SceneEnvironment {
            clearColor: "#1a1a2e"
            backgroundMode: SceneEnvironment.Color
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
            specularAAEnabled: true
            tonemapMode: SceneEnvironment.TonemapModeFilmic

            // Ambient occlusion: a cheap, texture-free way to add
            // perceived depth at the joints/corners where surfaces meet -
            // exactly the kind of "materials and lighting do more visual
            // work than people expect" investment Path A calls for.
            aoStrength: 60
            aoDistance: 0.2
            aoSoftness: 20
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
                z: 3
                clipNear: 0.01
            }
        }

        OrbitCameraController {
            anchors.fill: parent
            origin: cameraOrigin
            camera: camera
        }

        // Key light: the main directional light, casting shadows onto
        // the base disc for a sense of grounding.
        DirectionalLight {
            eulerRotation: Qt.vector3d(-45, -45, 0)
            brightness: 1.3
            castsShadow: true
            shadowMapQuality: Light.ShadowMapQualityHigh
            shadowFactor: 75
        }

        // Fill light: dimmer, from the opposite side, no shadow - softens
        // the key light's shadow side instead of leaving it flat black.
        // The classic two-light rig, needing no environment texture.
        DirectionalLight {
            eulerRotation: Qt.vector3d(-20, 135, 0)
            brightness: 0.4
            castsShadow: false
        }

        // A thin disc marking the base/world origin, for visual reference
        // - matte, non-metal, so it reads as a platform, not a link.
        Model {
            source: "#Cylinder"
            position: Qt.vector3d(0, -0.01, 0)
            scale: Qt.vector3d(0.3 / root.meshUnit, 0.01 / root.meshUnit, 0.3 / root.meshUnit)
            materials: PrincipledMaterial {
                baseColor: "#33333d"
                metalness: 0.0
                roughness: 0.9
            }
        }

        Repeater3D {
            model: armController.linkLengths.length

            Model {
                required property int index
                source: "#Cylinder"
                position: armController.linkMidpoints[index]
                rotation: armController.linkRotations[index]
                scale: Qt.vector3d(0.05 / root.meshUnit, armController.linkLengths[index] / root.meshUnit,
                                    0.05 / root.meshUnit)

                // Brushed-metal look. baseColor stays the torque-limit
                // green/amber/red readout carried over from Phase 1 - the
                // "is this joint overloaded" signal is functional, not
                // just decorative, so Phase 3's polish tunes the material
                // response around it rather than replacing it.
                materials: PrincipledMaterial {
                    baseColor: armController.jointColors[index]
                    metalness: 0.75
                    roughness: 0.35
                }
            }
        }

        Repeater3D {
            model: armController.jointPositions.length

            Model {
                required property int index
                source: "#Sphere"
                position: armController.jointPositions[index]
                scale: Qt.vector3d(0.09 / root.meshUnit, 0.09 / root.meshUnit, 0.09 / root.meshUnit)

                // Dark matte-plastic "joint housing" look, deliberately
                // distinct from the links' color-coded metal so a viewer
                // can tell joints from links at a glance, structurally.
                materials: PrincipledMaterial {
                    baseColor: "#2a2a33"
                    metalness: 0.2
                    roughness: 0.55
                }
            }
        }

        // Payload marker at the end-effector, sized (loosely) by mass -
        // standing in for "something attached to the hand" until a real
        // gripper/hand mesh exists (Phase 3, Path B - a stretch goal).
        Model {
            readonly property int lastIndex: armController.jointPositions.length - 1
            source: "#Cube"
            visible: armController.payloadKg > 0 && lastIndex >= 0
            position: lastIndex >= 0 ? armController.jointPositions[lastIndex] : Qt.vector3d(0, 0, 0)
            scale: {
                const meters = 0.1 + armController.payloadKg * 0.015
                const s = meters / root.meshUnit
                return Qt.vector3d(s, s, s)
            }

            materials: PrincipledMaterial {
                baseColor: "#e94560"
                metalness: 0.1
                roughness: 0.5
            }
        }
    }
}
