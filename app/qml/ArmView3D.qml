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
        id: view3D
        anchors.fill: parent
        // Explicit, not just "whichever camera node happens to render" -
        // view3D.mapFrom3DScene() (used by the joint-drag overlay below)
        // needs this set explicitly to resolve screen positions; without
        // it, rendering still worked but mapFrom3DScene() warned at
        // runtime and returned a meaningless position.
        camera: camera

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

    // Exposed at the root level so the overlay below (each a Repeater
    // delegate - a separate component boundary under
    // `pragma ComponentBehavior: Bound` above) can force a dependency on
    // the camera explicitly. view3D.mapFrom3DScene() below depends on
    // the camera's current transform, but nothing in that call reads a
    // QML property directly, so without this, QML's binding tracker has
    // no way to know to re-run it while the user orbits - the overlay
    // would silently drift out of sync with the joints it's supposed to
    // sit on.
    readonly property vector3d cameraEulerRotation: cameraOrigin.eulerRotation
    readonly property real cameraDistance: camera.z

    // Direct manipulation: a small draggable ring per joint (not per
    // jointPositions entry - the last entry is the end-effector tip, not
    // a pivot), projected onto its screen position. Dragging it left/
    // right changes that joint's angle directly in the main view, not
    // just via the sidebar sliders. These are plain 2D Items stacked on
    // top of the View3D, not 3D ray-picking - standard Qt Quick 2D event
    // delivery means a press that lands on a ring is consumed there and
    // never reaches the View3D's OrbitCameraController underneath, so
    // this can't conflict with camera-orbit dragging the way two
    // competing 3D pointer handlers could.
    Repeater {
        model: armController.jointCount

        Rectangle {
            id: jointHandle
            required property int index

            readonly property vector3d screenPosition: {
                const _cameraRotationDependency = root.cameraEulerRotation
                const _cameraDistanceDependency = root.cameraDistance
                return view3D.mapFrom3DScene(armController.jointPositions[index])
            }

            enabled: !armController.trajectoryScrubMode
            x: screenPosition.x - width / 2
            y: screenPosition.y - height / 2
            width: 28
            height: 28
            radius: width / 2
            color: "transparent"
            opacity: enabled ? 1.0 : 0.3
            border.width: dragHandler.active ? 3 : 2
            border.color: dragHandler.active ? "#ffffff" : "#cccccc"

            DragHandler {
                id: dragHandler
                target: null
                property double baseAngleDegrees: 0

                onActiveChanged: {
                    if (active) baseAngleDegrees = armController.jointAnglesDegrees[jointHandle.index]
                }

                onCentroidChanged: {
                    if (!active) return
                    // Horizontal drag maps to degrees - a simple, generic
                    // convention (like dragging a numeric field), not a
                    // literal rotation gesture, since the joint's actual
                    // rotation axis isn't generally aligned with the
                    // screen from any given camera angle.
                    const deltaPixels = centroid.position.x - centroid.pressPosition.x
                    const sensitivity = 0.5 // degrees per pixel
                    armController.setJointAngleDegrees(jointHandle.index, baseAngleDegrees + deltaPixels * sensitivity)
                }
            }
        }
    }
}
