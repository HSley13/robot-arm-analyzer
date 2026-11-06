pragma ComponentBehavior: Bound
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick3D
import QtQuick3D.Helpers
import Humanoid.Geometry

// The 3D scene: the humanoid (HumanoidBody3D, which carries the analyzed
// right arm and hand) on its mobile lift base, a table, the objects lying
// around, and a draggable ring over every arm joint.
//
// Clicking the floor or the table is handed to taskController.handleClick()
// - what it does depends on the Objects panel (place a new object, move
// one, put the held one down, or just select).
Item {
    id: root

    readonly property real meshUnit: 100.0
    readonly property var table: taskController.table

    // The hand closes around the Payload card's demo dumbbell and opens
    // when it's released. While a pick/put-down sequence runs, the
    // TaskController drives the hand itself.
    readonly property bool holding: !armController.gripperOpen
    onHoldingChanged: {
        if (!taskController.busy && taskController.heldId < 0) {
            handController.playGesture(holding ? "powerGrip" : "openPalm")
        }
    }
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
            position: launchOptions.focus !== undefined ? launchOptions.focus : Qt.vector3d(0.30, 0.72, 0.0)
            eulerRotation: Qt.vector3d(launchOptions.cameraPitch !== undefined ? launchOptions.cameraPitch : -14,
                                       launchOptions.cameraYaw !== undefined ? launchOptions.cameraYaw : 42, 0)

            PerspectiveCamera {
                id: camera
                z: launchOptions.cameraDistance !== undefined ? launchOptions.cameraDistance : 2.9
                fieldOfView: 40
                clipNear: 0.02
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
            eulerRotation: Qt.vector3d(-55, 30, 0)
            brightness: 0.9
            color: "#fff6ec"
            castsShadow: true
            shadowMapQuality: Light.ShadowMapQualityHigh
            shadowFactor: 60
            shadowBias: 0.002
            shadowMapFar: 8
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

        // ---- floor ----------------------------------------------------------

        Model {
            objectName: "floor"
            source: "#Rectangle"
            eulerRotation.x: -90
            scale: Qt.vector3d(8 / root.meshUnit, 8 / root.meshUnit, 1)
            receivesShadows: true
            pickable: true
            materials: PrincipledMaterial { baseColor: "#161925"; roughness: 0.9 }
        }
        Model {
            position: Qt.vector3d(0, 0.001, 0)
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

        // ---- table ----------------------------------------------------------

        Node {
            id: tableNode
            readonly property real x0: root.table.x0
            readonly property real x1: root.table.x1
            readonly property real z0: root.table.z0
            readonly property real z1: root.table.z1
            readonly property real h: root.table.height

            Model {
                objectName: "table"
                position: Qt.vector3d((tableNode.x0 + tableNode.x1) / 2, tableNode.h - 0.015, (tableNode.z0 + tableNode.z1) / 2)
                geometry: RoundedBoxGeometry {
                    size: Qt.vector3d(tableNode.x1 - tableNode.x0, 0.03, tableNode.z1 - tableNode.z0)
                    radius: 0.006
                }
                pickable: true
                materials: PrincipledMaterial { baseColor: "#6b5643"; roughness: 0.6 }
            }
            Repeater3D {
                model: 4
                Model {
                    required property int index
                    readonly property real lx: index % 2 === 0 ? tableNode.x0 + 0.04 : tableNode.x1 - 0.04
                    readonly property real lz: index < 2 ? tableNode.z0 + 0.04 : tableNode.z1 - 0.04
                    position: Qt.vector3d(lx, (tableNode.h - 0.03) / 2, lz)
                    geometry: RoundedBoxGeometry { size: Qt.vector3d(0.04, tableNode.h - 0.03, 0.04); radius: 0.006 }
                    materials: PrincipledMaterial { baseColor: "#2b2f37"; metalness: 0.6; roughness: 0.45 }
                }
            }
        }

        // ---- the robot ------------------------------------------------------

        LiftBase3D {}
        HumanoidBody3D { id: body }

        // ---- objects lying around -------------------------------------------

        Repeater3D {
            model: taskController.objects
            SceneObject3D {
                required property var modelData
                visible: !modelData.held
                position: Qt.vector3d(modelData.x, modelData.y, modelData.z)
                type: modelData.type
                mass: modelData.mass
                gripRadius: modelData.gripRadius
                footprint: modelData.footprint
                objectHeight: modelData.height
                selected: modelData.selected
            }
        }
    }

    // A click (not a drag - dragging orbits the camera) on the floor, the
    // table or an object: find the surface point under the cursor and let
    // the task controller decide what it means.
    TapHandler {
        acceptedButtons: Qt.LeftButton
        onTapped: (eventPoint) => {
            const result = view3D.pick(eventPoint.position.x, eventPoint.position.y)
            if (result.objectHit === null) {
                return
            }
            const p = result.scenePosition
            taskController.handleClick(p.x, p.z)
        }
    }

    // Hint for the armed click mode.
    Rectangle {
        visible: taskController.clickMode !== ""
        anchors.top: parent.top
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.topMargin: 14
        radius: 8
        color: "#17e0ff"
        width: hintText.implicitWidth + 24
        height: hintText.implicitHeight + 12
        Text {
            id: hintText
            anchors.centerIn: parent
            color: "#0e1021"
            font.bold: true
            text: taskController.clickMode === "place" ? "Click the floor or the table to place a " + taskController.placeType
                : taskController.clickMode === "move" ? "Click where the selected object should go"
                : "Click where to put the held object down"
        }
    }

    // Forces the overlay below to depend on everything that moves the arm
    // on screen - mapFrom3DScene() reads transforms without QML's binding
    // tracker seeing them.
    readonly property vector3d cameraEulerRotation: cameraOrigin.eulerRotation
    readonly property real cameraDistance: camera.z
    readonly property real bodyDependency: taskController.waistYawDegrees + taskController.hipHeight
                                           + taskController.hipBendDegrees
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

    // Draggable ring per arm joint, projected onto its screen position -
    // plain 2D Items on top of the View3D, so a press on a ring never
    // reaches the orbit controller underneath.
    Repeater {
        model: armController.jointCount

        Item {
            id: jointHandle
            required property int index

            readonly property vector3d screenPosition: {
                const _cameraRotationDependency = root.cameraEulerRotation
                const _cameraDistanceDependency = root.cameraDistance
                const _layoutDependency = root.layoutTick
                const _bodyDependency = root.bodyDependency
                const scenePoint = body.armNode.mapPositionToScene(armController.jointPositions[index])
                return view3D.mapFrom3DScene(scenePoint)
            }

            enabled: !armController.trajectoryScrubMode && !taskController.busy
            x: screenPosition.x - width / 2
            y: screenPosition.y - height / 2
            width: 30
            height: 30
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
