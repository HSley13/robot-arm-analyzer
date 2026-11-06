pragma ComponentBehavior: Bound
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick3D
import QtQuick3D.Helpers
import Humanoid.Geometry

// The 3D scene: the humanoid arm + hand (HumanoidArm3D / Hand3D) mounted
// on a lab test stand, its reach envelope, and a draggable ring over every
// arm joint.
Item {
    id: root

    readonly property real meshUnit: 100.0
    // Height of the shoulder pivot above the floor (the arm hangs from a
    // stand, like a humanoid's shoulder).
    readonly property real shoulderHeight: 1.0
    readonly property real totalReach: armController.canvasCenter.x * 2

    // The hand closes around the payload while holding it and opens when
    // released - the same switch that decides whether the payload counts
    // toward the joint torques.
    readonly property bool holding: !armController.gripperOpen
    onHoldingChanged: handController.playGesture(holding ? "powerGrip" : "openPalm")
    Component.onCompleted: handController.playGesture(holding ? "powerGrip" : "openPalm")

    View3D {
        id: view3D
        anchors.fill: parent
        camera: camera

        environment: SceneEnvironment {
            clearColor: "#0e1021"
            backgroundMode: SceneEnvironment.Color
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
            tonemapMode: SceneEnvironment.TonemapModeFilmic
            // Image-based lighting from a generated studio environment
            // (tools/make_studio_hdr.py): gives the metal parts something
            // to reflect and wraps the shells in soft light.
            lightProbe: Texture { source: "../assets/studio.hdr" }
            probeExposure: 0.9
            aoStrength: 45
            aoDistance: 0.05
            aoSoftness: 30
        }

        Node {
            id: cameraOrigin
            // Orbit around the middle of the arm's reach, a little below the
            // shoulder so the hand stays in frame when the arm hangs down.
            position: launchOptions.focus !== undefined ? launchOptions.focus
                                                        : Qt.vector3d(root.totalReach * 0.52, -0.04, 0)
            eulerRotation: Qt.vector3d(launchOptions.cameraPitch !== undefined ? launchOptions.cameraPitch : -10,
                                       launchOptions.cameraYaw !== undefined ? launchOptions.cameraYaw : 26, 0)

            PerspectiveCamera {
                id: camera
                z: launchOptions.cameraDistance !== undefined ? launchOptions.cameraDistance
                                                               : Math.max(0.6, root.totalReach * 1.32)
                fieldOfView: 40
                clipNear: 0.01
                clipFar: 50
            }
        }

        OrbitCameraController {
            anchors.fill: parent
            origin: cameraOrigin
            camera: camera
        }

        // Key light (casts the shadows), cool fill from the other side, and
        // a rim light from behind that picks out the shell edges.
        DirectionalLight {
            eulerRotation: Qt.vector3d(-50, 35, 0)
            brightness: 0.9
            color: "#fff6ec"
            castsShadow: true
            shadowMapQuality: Light.ShadowMapQualityHigh
            shadowFactor: 60
            shadowBias: 0.002
        }
        DirectionalLight {
            eulerRotation: Qt.vector3d(-15, -60, 0)
            brightness: 0.2
            color: "#cfe0ff"
        }
        DirectionalLight {
            eulerRotation: Qt.vector3d(-20, 160, 0)
            brightness: 0.45
            color: "#d8e6ff"
        }

        // ---- lab floor + test stand ---------------------------------------

        Model {
            source: "#Rectangle"
            eulerRotation.x: -90
            position: Qt.vector3d(0, -root.shoulderHeight, 0)
            scale: Qt.vector3d(8 / root.meshUnit, 8 / root.meshUnit, 1)
            receivesShadows: true
            materials: PrincipledMaterial { baseColor: "#161925"; roughness: 0.9 }
        }
        Model {
            position: Qt.vector3d(0, -root.shoulderHeight + 0.001, 0)
            scale: Qt.vector3d(0.01, 0.01, 0.01)
            geometry: GridGeometry {
                horizontalLines: 40
                verticalLines: 40
                horizontalStep: 10
                verticalStep: 10
            }
            eulerRotation.x: -90
            materials: PrincipledMaterial {
                baseColor: "#23283a"
                lighting: PrincipledMaterial.NoLighting
            }
        }

        // Aluminum extrusion post behind the shoulder, on a weighted base,
        // with a bracket out to the shoulder actuator.
        Node {
            id: stand
            readonly property real postZ: -0.16

            Model {
                position: Qt.vector3d(0, -root.shoulderHeight + 0.012, stand.postZ)
                source: "#Cylinder"
                scale: Qt.vector3d(0.42 / root.meshUnit, 0.024 / root.meshUnit, 0.42 / root.meshUnit)
                materials: PrincipledMaterial { baseColor: "#262a33"; metalness: 0.6; roughness: 0.5 }
            }
            Model {
                position: Qt.vector3d(0, -root.shoulderHeight / 2 + 0.02, stand.postZ)
                geometry: RoundedBoxGeometry {
                    size: Qt.vector3d(0.045, root.shoulderHeight + 0.04, 0.045)
                    radius: 0.004
                }
                materials: PrincipledMaterial { baseColor: "#9aa1aa"; metalness: 0.85; roughness: 0.38 }
            }
            // Extrusion slots.
            Repeater3D {
                model: 4
                Model {
                    required property int index
                    readonly property real angle: index * 90
                    position: Qt.vector3d(0.0226 * Math.cos(angle * Math.PI / 180), -root.shoulderHeight / 2 + 0.02,
                                          stand.postZ + 0.0226 * Math.sin(angle * Math.PI / 180))
                    eulerRotation.y: -angle
                    geometry: RoundedBoxGeometry { size: Qt.vector3d(0.002, root.shoulderHeight + 0.03, 0.008); radius: 0.0008 }
                    materials: PrincipledMaterial { baseColor: "#2a2d33"; roughness: 0.7 }
                }
            }
            // Round mounting flange from the post to the back of the shoulder.
            Model {
                source: "#Cylinder"
                eulerRotation.x: 90
                position: Qt.vector3d(0, 0, (stand.postZ - 0.06) / 2)
                scale: Qt.vector3d(0.07 / root.meshUnit, (Math.abs(stand.postZ) - 0.06) / root.meshUnit, 0.07 / root.meshUnit)
                materials: PrincipledMaterial { baseColor: "#9aa1aa"; metalness: 0.85; roughness: 0.38 }
            }
        }

        // ---- the humanoid arm + hand ---------------------------------------

        HumanoidArm3D {}

        // Reach envelope: a faint ring in the arm's plane at full reach.
        Model {
            visible: root.totalReach > 0
            geometry: TubeGeometry {
                closed: true
                radius: 0.0018
                radialSegments: 6
                samplesPerSpan: 2
                points: {
                    const pts = []
                    for (let k = 0; k < 72; k++) {
                        const a = 2 * Math.PI * k / 72
                        pts.push(Qt.vector3d(root.totalReach * Math.cos(a), root.totalReach * Math.sin(a), 0))
                    }
                    return pts
                }
            }
            opacity: 0.14
            materials: PrincipledMaterial {
                baseColor: "#17e0ff"
                emissiveFactor: Qt.vector3d(0.05, 0.6, 0.75)
                lighting: PrincipledMaterial.NoLighting
            }
        }
    }

    // Forces the overlay below to depend on the camera explicitly -
    // view3D.mapFrom3DScene() reads the camera's transform without
    // QML's binding tracker seeing it, so without this the overlay
    // wouldn't re-run while orbiting.
    readonly property vector3d cameraEulerRotation: cameraOrigin.eulerRotation
    readonly property real cameraDistance: camera.z
    // mapFrom3DScene() also only gives a real answer once the view has
    // been laid out and rendered; re-run the projection a few times after
    // startup and on every resize, or the rings sit at (0, 0) until the
    // first joint moves.
    property int layoutTick: 0
    Timer {
        interval: 120
        repeat: true
        running: root.layoutTick < 10
        onTriggered: root.layoutTick++
    }
    Connections {
        target: view3D
        function onWidthChanged() { root.layoutTick++ }
        function onHeightChanged() { root.layoutTick++ }
    }

    // Draggable ring per joint, projected onto its screen position via
    // mapFrom3DScene() - plain 2D Items on top of the View3D, not 3D
    // ray-picking, so a press on a ring never reaches the orbit
    // controller underneath.
    Repeater {
        model: armController.jointCount

        Item {
            id: jointHandle
            required property int index

            readonly property vector3d screenPosition: {
                const _cameraRotationDependency = root.cameraEulerRotation
                const _cameraDistanceDependency = root.cameraDistance
                const _layoutDependency = root.layoutTick
                return view3D.mapFrom3DScene(armController.jointPositions[index])
            }

            enabled: !armController.trajectoryScrubMode
            x: screenPosition.x - width / 2
            y: screenPosition.y - height / 2
            width: 34
            height: 34
            opacity: enabled ? 1.0 : 0.3

            Rectangle {
                anchors.fill: parent
                radius: width / 2
                color: dragHandler.active ? Qt.rgba(0.09, 0.88, 1.0, 0.3)
                     : (hoverHandler.hovered ? Qt.rgba(0.09, 0.88, 1.0, 0.15) : Qt.rgba(0.09, 0.88, 1.0, 0.04))
                border.width: dragHandler.active ? 3 : 2
                border.color: dragHandler.active ? "#ffffff" : (hoverHandler.hovered ? "#17e0ff" : Qt.rgba(0.6, 0.66, 0.78, 0.55))

                Behavior on color { ColorAnimation { duration: 120 } }
                Behavior on border.color { ColorAnimation { duration: 120 } }
            }

            Rectangle {
                visible: dragHandler.active || hoverHandler.hovered
                anchors.bottom: parent.top
                anchors.bottomMargin: 6
                anchors.horizontalCenter: parent.horizontalCenter
                width: angleReadout.implicitWidth + 12
                height: angleReadout.implicitHeight + 6
                radius: 4
                color: "#0e1021"
                border.color: "#17e0ff"
                border.width: 1

                Text {
                    id: angleReadout
                    anchors.centerIn: parent
                    color: "white"
                    font.bold: true
                    font.pixelSize: 12
                    text: armController.jointNames[jointHandle.index] + "  "
                          + Math.round(armController.jointAnglesDegrees[jointHandle.index]) + "°"
                }
            }

            HoverHandler {
                id: hoverHandler
                cursorShape: dragHandler.active ? Qt.ClosedHandCursor : Qt.OpenHandCursor
            }

            DragHandler {
                id: dragHandler
                target: null
                property double baseAngleDegrees: 0

                onActiveChanged: {
                    if (active) baseAngleDegrees = armController.jointAnglesDegrees[jointHandle.index]
                }

                onCentroidChanged: {
                    if (!active) return
                    // Drag up/right to raise (positive angle).
                    const delta = (centroid.position.x - centroid.pressPosition.x)
                                - (centroid.position.y - centroid.pressPosition.y)
                    const sensitivity = 0.4
                    armController.setJointAngleDegrees(jointHandle.index, baseAngleDegrees + delta * sensitivity)
                }
            }
        }
    }
}
