pragma ComponentBehavior: Bound
import QtQuick
import QtQuick3D
import QtQuick3D.Helpers

Item {
    id: root

    // Model.scale is (desired meters) / meshUnit; position is already
    // in meters and needs no conversion.
    readonly property real meshUnit: 100.0

    function linkDiameterMeters(index) { return Math.max(0.05, 0.11 - index * 0.03) }
    function jointDrumDiameterMeters(index) { return Math.max(0.09, 0.16 - index * 0.025) }

    View3D {
        id: view3D
        anchors.fill: parent
        camera: camera

        environment: SceneEnvironment {
            clearColor: "#0e1021"
            backgroundMode: SceneEnvironment.Color
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
            specularAAEnabled: true
            tonemapMode: SceneEnvironment.TonemapModeFilmic

            aoStrength: 60
            aoDistance: 0.2
            aoSoftness: 20
        }

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

        DirectionalLight {
            eulerRotation: Qt.vector3d(-45, -45, 0)
            brightness: 1.3
            castsShadow: true
            shadowMapQuality: Light.ShadowMapQualityHigh
            shadowFactor: 75
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(-20, 135, 0)
            brightness: 0.4
            castsShadow: false
        }

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

        // Workspace envelope: radius tracks canvasCenter.x live.
        Model {
            readonly property real maxReachMeters: armController.canvasCenter.x * 2
            source: "#Cylinder"
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

                materials: PrincipledMaterial {
                    baseColor: "#d7dbe2"
                    metalness: 0.8
                    roughness: 0.3
                }
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

                materials: PrincipledMaterial {
                    baseColor: "#2a2a33"
                    metalness: 0.3
                    roughness: 0.5
                }
            }
        }

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

        Node {
            id: gripperBase
            readonly property int lastLinkIndex: armController.linkRotations.length - 1
            visible: lastLinkIndex >= 0
            position: armController.jointPositions.length > 0
                      ? armController.jointPositions[armController.jointPositions.length - 1]
                      : Qt.vector3d(0, 0, 0)
            rotation: lastLinkIndex >= 0 ? armController.linkRotations[lastLinkIndex] : Qt.quaternion(1, 0, 0, 0)

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
                        position: Qt.vector3d(0, 0.07, 0)
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

    // Forces the overlay below to depend on the camera explicitly -
    // view3D.mapFrom3DScene() reads the camera's transform without
    // QML's binding tracker seeing it, so without this the overlay
    // wouldn't re-run while orbiting.
    readonly property vector3d cameraEulerRotation: cameraOrigin.eulerRotation
    readonly property real cameraDistance: camera.z

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
                return view3D.mapFrom3DScene(armController.jointPositions[index])
            }

            enabled: !armController.trajectoryScrubMode
            x: screenPosition.x - width / 2
            y: screenPosition.y - height / 2
            width: 32
            height: 32
            opacity: enabled ? 1.0 : 0.3

            Rectangle {
                anchors.fill: parent
                radius: width / 2
                color: dragHandler.active ? Qt.rgba(0.09, 0.88, 1.0, 0.3)
                     : (hoverHandler.hovered ? Qt.rgba(0.09, 0.88, 1.0, 0.15) : Qt.rgba(0.09, 0.88, 1.0, 0.05))
                border.width: dragHandler.active ? 3 : 2
                border.color: dragHandler.active ? "#ffffff" : (hoverHandler.hovered ? "#17e0ff" : "#4a5468")

                Behavior on color { ColorAnimation { duration: 120 } }
                Behavior on border.color { ColorAnimation { duration: 120 } }
            }

            Rectangle {
                visible: dragHandler.active
                anchors.bottom: parent.top
                anchors.bottomMargin: 6
                anchors.horizontalCenter: parent.horizontalCenter
                width: angleReadout.implicitWidth + 10
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
                    text: Math.round(armController.jointAnglesDegrees[jointHandle.index]) + "°"
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
                    const deltaPixels = centroid.position.x - centroid.pressPosition.x
                    const sensitivity = 0.5
                    armController.setJointAngleDegrees(jointHandle.index, baseAngleDegrees + deltaPixels * sensitivity)
                }
            }
        }
    }
}
