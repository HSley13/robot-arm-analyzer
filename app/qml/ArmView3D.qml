pragma ComponentBehavior: Bound
import QtQuick
import QtQuick3D
import QtQuick3D.Helpers

// Phase 5: a from-scratch look pass driven by direct feedback that the
// Phase 1/3 arm ("two cylinders and two spheres") didn't read as a robot
// arm at all. Still entirely procedural geometry - no textures, no 3D
// modeling software - but assembled to suggest real hardware: a grounded
// two-piece pedestal the shoulder rises out of, limbs that taper from
// shoulder to wrist like a real arm's bicep/forearm/wrist, and joints
// built from a cylindrical "motor drum" (its flat face is the joint's own
// rotation axis, since every joint here rotates about Z) instead of a
// bare sphere. The torque-limit color readout moved from tinting the
// whole limb to a slim "status band" wrapped around each link's middle,
// so the chassis itself can read as a single consistent material (brushed
// steel) with its own cyan accent lighting, independent of load state.
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

    // Link/joint sizing tapers from shoulder (index 0) to wrist (the
    // highest index) so the arm reads as bicep -> forearm -> wrist, not
    // a uniform tube - and degrades gracefully for any joint count (2, 3,
    // or whatever a loaded config has), not just the 3-DOF arm this was
    // designed against.
    function linkDiameterMeters(index) { return Math.max(0.05, 0.11 - index * 0.03) }
    function jointDrumDiameterMeters(index) { return Math.max(0.09, 0.16 - index * 0.025) }

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
            clearColor: "#0e1021"
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
        // the pedestal for a sense of grounding.
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

        // Pedestal: a wide, flat foot bolted to "the floor" and a
        // narrower column rising to meet the shoulder joint at the
        // origin - a real base, not just a reference disc. Matte,
        // non-metal, so it visually grounds the arm instead of competing
        // with its brushed-metal limbs.
        Model {
            source: "#Cylinder"
            position: Qt.vector3d(0, -0.34, 0)
            scale: Qt.vector3d(0.55 / root.meshUnit, 0.08 / root.meshUnit, 0.55 / root.meshUnit)
            materials: PrincipledMaterial {
                baseColor: "#24262e"
                metalness: 0.1
                roughness: 0.85
            }
        }

        Model {
            source: "#Cylinder"
            position: Qt.vector3d(0, -0.15, 0)
            scale: Qt.vector3d(0.28 / root.meshUnit, 0.3 / root.meshUnit, 0.28 / root.meshUnit)
            materials: PrincipledMaterial {
                baseColor: "#2a2d36"
                metalness: 0.15
                roughness: 0.8
            }
        }

        // A thin glowing collar where the column meets the shoulder -
        // the one deliberately "futuristic" accent on the base, done
        // with an emissive PrincipledMaterial rather than a texture or
        // a light of its own.
        Model {
            source: "#Cylinder"
            position: Qt.vector3d(0, -0.01, 0)
            scale: Qt.vector3d(0.3 / root.meshUnit, 0.015 / root.meshUnit, 0.3 / root.meshUnit)
            materials: PrincipledMaterial {
                baseColor: "#113a42"
                emissiveFactor: Qt.vector3d(0.08, 0.85, 1.0)
                metalness: 0.0
                roughness: 0.3
            }
        }

        // Workspace envelope (Phase 6, toggled off by default): a
        // translucent disc in the arm's own XY motion plane showing
        // how far it can reach - radius = the sum of every link's
        // length (canvasCenter.x is already exactly half that, reused
        // here rather than adding a second property for the same
        // number). Rotated 90 about X for the same reason the joint
        // motor drums are: the built-in cylinder's flat face is
        // normally perpendicular to Y, and this needs it perpendicular
        // to Z instead, flat in the plane the arm actually moves in.
        // A single disc, not a true annulus with a cut-out center for
        // arms whose first link is much longer than the rest can't
        // quite reach - a deliberate simplification (Qt Quick 3D has
        // no ring primitive to subtract one disc from another), not
        // an attempt at an exact reachable-workspace boundary.
        Model {
            readonly property real maxReachMeters: armController.canvasCenter.x * 2
            source: "#Cylinder"
            visible: armController.showWorkspaceEnvelope
            rotation: Qt.vector3d(90, 0, 0)
            opacity: 0.12
            scale: {
                const diameter = (maxReachMeters * 2) / root.meshUnit
                return Qt.vector3d(diameter, 0.005 / root.meshUnit, diameter)
            }
            materials: PrincipledMaterial {
                baseColor: "#17e0ff"
                metalness: 0.0
                roughness: 0.6
            }
        }

        Repeater3D {
            model: armController.linkLengths.length

            Model {
                required property int index
                source: "#Cylinder"
                position: armController.linkMidpoints[index]
                rotation: armController.linkRotations[index]
                scale: {
                    const diameter = root.linkDiameterMeters(index) / root.meshUnit
                    return Qt.vector3d(diameter, armController.linkLengths[index] / root.meshUnit, diameter)
                }

                // A single consistent brushed-steel chassis color for
                // every limb - the torque-limit readout moved to the
                // status band below, so this material no longer has to
                // double as both "what the limb is made of" and "is it
                // overloaded".
                materials: PrincipledMaterial {
                    baseColor: "#d7dbe2"
                    metalness: 0.8
                    roughness: 0.3
                }
            }
        }

        // Status band: a short, slightly-wider collar wrapped around
        // each link's middle, carrying the green/amber/red torque-limit
        // color that used to tint the whole limb. Keeping the signal but
        // moving it here reads more like a real safety-rated machine
        // (a status light band) than a limb that changes color outright.
        Repeater3D {
            model: armController.linkLengths.length

            Model {
                required property int index
                source: "#Cylinder"
                position: armController.linkMidpoints[index]
                rotation: armController.linkRotations[index]
                scale: {
                    const diameter = (root.linkDiameterMeters(index) + 0.03) / root.meshUnit
                    return Qt.vector3d(diameter, 0.05 / root.meshUnit, diameter)
                }

                materials: PrincipledMaterial {
                    baseColor: armController.jointColors[index]
                    metalness: 0.2
                    roughness: 0.4
                }
            }
        }

        // Joint "motor drum": a cylinder rotated so its flat circular
        // face points along Z - every joint in every arm config this
        // project loads rotates about Z (alpha_deg is always 0, so DH's
        // z-axis never tips out of the arm's plane of motion), so this
        // face is literally the joint's own rotation axis, not just a
        // decorative choice. Reads as a motor housing bolted across the
        // joint, the way a real servo/motor drum looks on an actual arm.
        Repeater3D {
            model: armController.jointPositions.length

            Model {
                required property int index
                source: "#Cylinder"
                position: armController.jointPositions[index]
                rotation: Qt.vector3d(90, 0, 0)
                scale: {
                    const diameter = root.jointDrumDiameterMeters(index) / root.meshUnit
                    return Qt.vector3d(diameter, 0.09 / root.meshUnit, diameter)
                }

                // Dark matte-plastic housing, deliberately distinct from
                // the limbs' brushed-metal chassis so a viewer can tell
                // joints from links at a glance, structurally.
                materials: PrincipledMaterial {
                    baseColor: "#2a2a33"
                    metalness: 0.3
                    roughness: 0.5
                }
            }
        }

        // Center cap on each motor drum - a small glossy cyan "indicator
        // lens", the same accent color as the pedestal collar, tying the
        // whole arm's lighting language together.
        Repeater3D {
            model: armController.jointPositions.length

            Model {
                required property int index
                source: "#Sphere"
                position: armController.jointPositions[index]
                scale: {
                    const diameter = (root.jointDrumDiameterMeters(index) * 0.45) / root.meshUnit
                    return Qt.vector3d(diameter, diameter, diameter)
                }

                materials: PrincipledMaterial {
                    baseColor: "#0d2126"
                    emissiveFactor: Qt.vector3d(0.08, 0.85, 1.0)
                    metalness: 0.0
                    roughness: 0.25
                }
            }
        }

        // Gripper: a pivot Node sitting at the end effector, oriented
        // like the arm's final link (so "forward" continues the arm's
        // own direction), holding two fingers that splay open/closed.
        // The existing motor-drum + indicator-lens Repeater3Ds above
        // already draw a housing at this exact position (jointPositions'
        // last entry is the wrist tip, included in that loop), so it
        // doubles as this gripper's palm - no separate palm mesh needed.
        Node {
            id: gripperBase
            readonly property int lastLinkIndex: armController.linkRotations.length - 1
            visible: lastLinkIndex >= 0
            position: armController.jointPositions.length > 0
                      ? armController.jointPositions[armController.jointPositions.length - 1]
                      : Qt.vector3d(0, 0, 0)
            rotation: lastLinkIndex >= 0 ? armController.linkRotations[lastLinkIndex] : Qt.quaternion(1, 0, 0, 0)

            // Both joints' angle lives in degrees here for a smooth
            // open/close animation independent of how fast armController
            // itself updates - splayed wide open, nearly parallel when
            // gripping (never fully 0, so the fingers stay visually
            // distinct even closed).
            property real splayDegrees: armController.gripperOpen ? 24 : 5
            Behavior on splayDegrees { NumberAnimation { duration: 300; easing.type: Easing.InOutQuad } }

            Repeater3D {
                model: 2

                Node {
                    required property int index
                    readonly property real side: index === 0 ? 1 : -1
                    rotation: Qt.vector3d(0, 0, gripperBase.splayDegrees * side)

                    Model {
                        source: "#Cube"
                        position: Qt.vector3d(0, 0.07 / root.meshUnit, 0)
                        scale: Qt.vector3d(0.025 / root.meshUnit, 0.14 / root.meshUnit, 0.02 / root.meshUnit)
                        materials: PrincipledMaterial {
                            baseColor: "#d7dbe2"
                            metalness: 0.7
                            roughness: 0.3
                        }
                    }
                }
            }
        }

        // Payload marker - a weight hanging on a string, not a crate,
        // so a sphere reads better than a cube here - sized (loosely)
        // by mass, hanging below the gripper on a thin string - only
        // shown while the gripper is closed (nothing is open and still
        // holding) and there's actually a payload staged. Hangs
        // straight down (-Y, the same "down" every torque calculation's
        // gravity vector already uses) rather than along the arm's own
        // direction, the way a real cable under tension would.
        Model {
            readonly property int lastIndex: armController.jointPositions.length - 1
            readonly property vector3d tipPosition: lastIndex >= 0 ? armController.jointPositions[lastIndex]
                                                                    : Qt.vector3d(0, 0, 0)
            readonly property real hangMeters: 0.12 + armController.payloadKg * 0.01

            source: "#Sphere"
            visible: !armController.gripperOpen && armController.payloadKg > 0 && lastIndex >= 0
            position: Qt.vector3d(tipPosition.x, tipPosition.y - hangMeters, tipPosition.z)
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

        // The string itself: a thin vertical cylinder from the gripper
        // tip down to the payload - always straight down by
        // construction (both endpoints share x/z), so no rotation math
        // is needed the way the tilted links need linkRotations.
        Model {
            readonly property int lastIndex: armController.jointPositions.length - 1
            readonly property vector3d tipPosition: lastIndex >= 0 ? armController.jointPositions[lastIndex]
                                                                    : Qt.vector3d(0, 0, 0)
            readonly property real hangMeters: 0.12 + armController.payloadKg * 0.01

            source: "#Cylinder"
            visible: !armController.gripperOpen && armController.payloadKg > 0 && lastIndex >= 0
            position: Qt.vector3d(tipPosition.x, tipPosition.y - hangMeters / 2, tipPosition.z)
            scale: Qt.vector3d(0.006 / root.meshUnit, hangMeters / root.meshUnit, 0.006 / root.meshUnit)

            materials: PrincipledMaterial {
                baseColor: "#9aa0ab"
                metalness: 0.2
                roughness: 0.6
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
