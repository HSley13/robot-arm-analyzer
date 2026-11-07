pragma ComponentBehavior: Bound
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick3D
import QtQuick3D.Helpers
import Humanoid.Geometry

// The 3D scene: the humanoid (HumanoidBody3D, which carries the analyzed
// right arm and hand) on its mobile lift base, a table and the objects
// lying around - and everything you do with the mouse:
//
//   * drag the hand (or gripper)  -> the whole body follows it: waist
//     turns, lift rises/sinks, hip leans, arm reaches (taskController.reachTo)
//   * drag a joint or a limb      -> swings that joint, like grabbing the arm
//   * drag an object              -> slides it across the floor / table
//   * click an object             -> menu: pick up, move, mass, delete
//   * click the floor / table     -> menu: put down here, reach here, add
//   * drag empty space            -> orbit; right-drag pans; wheel zooms
Item {
    id: root

    readonly property real meshUnit: 100.0

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

        // ---- tables ---------------------------------------------------------

        Repeater3D {
            model: taskController.tables
            Table3D {
                required property var modelData
                tableId: modelData.id
                kind: modelData.kind
                position: Qt.vector3d(modelData.x, 0, modelData.z)
                tableWidth: modelData.width
                tableDepth: modelData.depth
                tableHeight: modelData.height
                highlighted: mouse.grab.kind === "table" && mouse.grab.id === modelData.id
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
                objectId: modelData.id
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

    // ---- screen <-> scene helpers ------------------------------------------

    // Arm-frame point (armController.jointPositions) -> scene -> screen.
    function armToScene(p) { return body.armNode.mapPositionToScene(p) }
    function toScreen(scenePoint) { return view3D.mapFrom3DScene(scenePoint) }
    function jointScreen(i) {
        const _dependencies = [root.cameraEulerRotation, root.cameraDistance, root.cameraOriginPosition,
                               root.layoutTick, root.bodyDependency, armController.jointAnglesDegrees]
        if (!view3D.camera) return Qt.vector3d(-1000, -1000, 0)
        return i < armController.jointPositions.length ? toScreen(armToScene(armController.jointPositions[i]))
                                                       : Qt.vector3d(-1000, -1000, 0)
    }
    // Where the mouse ray crosses the plane through `point` facing the
    // camera - how far "in" a drag happens is wherever it started.
    function rayOnCameraPlane(x, y, point) {
        const a = view3D.mapTo3DScene(Qt.vector3d(x, y, 0))
        const b = view3D.mapTo3DScene(Qt.vector3d(x, y, 1))
        const dir = b.minus(a).normalized()
        const n = camera.forward
        const denom = dir.dotProduct(n)
        if (Math.abs(denom) < 1e-6) return point
        const t = point.minus(a).dotProduct(n) / denom
        return a.plus(dir.times(t))
    }
    function distanceToSegment(p, a, b) {
        const abx = b.x - a.x, aby = b.y - a.y
        const len2 = abx * abx + aby * aby
        const t = len2 > 0 ? Math.max(0, Math.min(1, ((p.x - a.x) * abx + (p.y - a.y) * aby) / len2)) : 0
        return { distance: Math.hypot(p.x - (a.x + t * abx), p.y - (a.y + t * aby)), t: t }
    }
    // The floor or table point under the mouse, ignoring objects (so an
    // object can be dragged across the surface under it), or null.
    function surfaceAt(x, y) {
        const hits = view3D.pickAll(x, y)
        for (let i = 0; i < hits.length; i++) {
            const name = hits[i].objectHit ? hits[i].objectHit.objectName : ""
            if (name === "floor" || name === "table") return hits[i].scenePosition
        }
        return null
    }
    // The id of the scene object under the mouse (one standing around, not
    // the one being carried), or -1.
    function objectAt(x, y) {
        const hits = view3D.pickAll(x, y)
        for (let i = 0; i < hits.length; i++) {
            for (let node = hits[i].objectHit; node; node = node.parent) {
                if (node.objectId !== undefined) {
                    if (node.objectId >= 0 && node.visible) return node.objectId
                    break
                }
            }
        }
        return -1
    }

    // What's under the mouse, most specific first:
    //   {kind: "hand"} / {kind: "joint", index} / {kind: "object", id} /
    //   {kind: "surface", point} / {kind: "none"}
    function hitTest(x, y) {
        const p = Qt.point(x, y)
        const n = armController.jointCount
        if (!taskController.busy && !armController.trajectoryScrubMode && n > 0) {
            // The hand: around the arm's tip or along the hand link.
            const tip = jointScreen(n)
            const wrist = jointScreen(n - 1)
            if (Math.hypot(x - tip.x, y - tip.y) < 34) return { kind: "hand" }
            const last = distanceToSegment(p, wrist, tip)
            if (last.distance < 22 && last.t > 0.35) return { kind: "hand" }
            for (let i = 0; i < n; i++) {
                const c = jointScreen(i)
                if (Math.hypot(x - c.x, y - c.y) < 20) return { kind: "joint", index: i }
            }
        }
        const object = objectAt(x, y)
        if (object >= 0) return { kind: "object", id: object }
        if (!taskController.busy && !armController.trajectoryScrubMode) {
            // Grabbing a limb swings the joint it hangs from.
            for (let i = 0; i < n; i++) {
                if (distanceToSegment(p, jointScreen(i), jointScreen(i + 1)).distance < 16) {
                    return { kind: "joint", index: i }
                }
            }
        }
        const surface = surfaceAt(x, y)
        return surface !== null ? { kind: "surface", point: surface } : { kind: "none" }
    }

    // Forces the overlay below to depend on everything that moves the arm
    // on screen - mapFrom3DScene() reads transforms without QML's binding
    // tracker seeing them.
    readonly property vector3d cameraEulerRotation: cameraOrigin.eulerRotation
    readonly property vector3d cameraOriginPosition: cameraOrigin.position
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

    // ---- the mouse ----------------------------------------------------------

    MouseArea {
        id: mouse
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton | Qt.RightButton
        hoverEnabled: true

        property var grab: ({ kind: "none" })   // what the press landed on
        property var hover: ({ kind: "none" })
        property bool dragging: false
        property point pressPoint
        property point lastPoint
        property int pressButton: Qt.NoButton
        // Hand drag: the plane it moves in (planning a reach takes a few
        // milliseconds, so every mouse move goes straight to the planner).
        property vector3d handPlanePoint
        // Joint drag: screen angle around the joint at the last move.
        property real lastScreenAngle: 0
        property real jointSign: 1

        cursorShape: dragging && grab.kind !== "none" && grab.kind !== "surface" ? Qt.ClosedHandCursor
                   : hover.kind === "hand" || hover.kind === "joint" ? Qt.OpenHandCursor
                   : hover.kind === "object" ? Qt.PointingHandCursor
                   : Qt.ArrowCursor

        onPressed: (event) => {
            pressPoint = Qt.point(event.x, event.y)
            lastPoint = pressPoint
            pressButton = event.button
            dragging = false
            grab = event.button === Qt.LeftButton ? root.hitTest(event.x, event.y) : { kind: "none" }
            if (grab.kind === "hand") {
                handPlanePoint = root.armToScene(armController.jointPositions[armController.jointCount])
            } else if (grab.kind === "joint") {
                taskController.cancelReach() // the joint is yours now, not the hand goal's
                const c = root.jointScreen(grab.index)
                lastScreenAngle = Math.atan2(event.y - c.y, event.x - c.x)
                // Positive joint angles turn counter-clockwise about the arm
                // plane's normal; on screen that's counter-clockwise only when
                // the normal faces the camera.
                const normal = body.armNode.mapDirectionToScene(Qt.vector3d(0, 0, 1))
                jointSign = normal.dotProduct(camera.forward) < 0 ? -1 : 1
            }
        }

        onPositionChanged: (event) => {
            if (!pressed) {
                hover = root.hitTest(event.x, event.y)
                return
            }
            if (!dragging && Math.hypot(event.x - pressPoint.x, event.y - pressPoint.y) < 4) return
            dragging = true
            const dx = event.x - lastPoint.x
            const dy = event.y - lastPoint.y
            lastPoint = Qt.point(event.x, event.y)

            if (grab.kind === "hand") {
                const target = root.rayOnCameraPlane(event.x, event.y, handPlanePoint)
                taskController.reachTo(target.x, target.y, target.z)
            } else if (grab.kind === "joint") {
                taskController.cancelReach() // the joint is yours now, not the hand goal's
                const c = root.jointScreen(grab.index)
                const angle = Math.atan2(event.y - c.y, event.x - c.x)
                let delta = angle - lastScreenAngle
                if (delta > Math.PI) delta -= 2 * Math.PI
                if (delta < -Math.PI) delta += 2 * Math.PI
                lastScreenAngle = angle
                const current = armController.jointAnglesDegrees[grab.index]
                armController.setJointAngleDegrees(grab.index, current + jointSign * delta * 180 / Math.PI)
            } else if (grab.kind === "object") {
                const spot = root.surfaceAt(event.x, event.y)
                if (spot !== null) taskController.moveObject(grab.id, spot.x, spot.z)
            } else if (pressButton === Qt.RightButton) {
                // Pan: slide the orbit center in the camera's plane.
                const scale = camera.z * 0.0013
                const right = camera.right
                const up = camera.up
                cameraOrigin.position = cameraOrigin.position.minus(right.times(dx * scale)).plus(up.times(dy * scale))
            } else {
                // Orbit.
                const pitch = Math.max(-85, Math.min(5, cameraOrigin.eulerRotation.x - dy * 0.3))
                cameraOrigin.eulerRotation = Qt.vector3d(pitch, cameraOrigin.eulerRotation.y - dx * 0.3, 0)
            }
        }

        onReleased: (event) => {
            if (!dragging) root.clicked(event.x, event.y, event.button)
            dragging = false
            grab = { kind: "none" }
        }

        onWheel: (wheel) => {
            camera.z = Math.max(0.5, Math.min(8.0, camera.z * Math.pow(0.9, wheel.angleDelta.y / 120)))
        }
        onExited: hover = { kind: "none" }
    }

    // A plain click (no drag): menus.
    signal clicked(real x, real y, int button)
    onClicked: (x, y, button) => {
        const hit = hitTest(x, y)
        if (taskController.clickMode !== "" && hit.kind === "surface") {
            // An armed click mode from the panel still works.
            taskController.handleClick(hit.point.x, hit.point.z)
        } else if (hit.kind === "object") {
            taskController.selectedId = hit.id
            objectMenu.openFor(hit.id, x, y)
        } else if (hit.kind === "surface") {
            spotMenu.openAt(hit.point, x, y)
        } else {
            taskController.selectedId = -1
        }
    }

    ObjectMenu { id: objectMenu }
    SpotMenu { id: spotMenu }

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

    // ---- joint markers + the hand handle (drawn only; the MouseArea
    // above does the grabbing) ----------------------------------------------

    Repeater {
        model: armController.jointCount

        Item {
            id: jointMarker
            required property int index
            readonly property vector3d screenPosition: root.jointScreen(index)
            readonly property bool active: (mouse.grab.kind === "joint" && mouse.grab.index === index && mouse.dragging)
            readonly property bool hovered: mouse.hover.kind === "joint" && mouse.hover.index === index
            readonly property color status: index < armController.jointColors.length ? armController.jointColors[index] : "#2ecc71"

            visible: !taskController.busy
            x: screenPosition.x - width / 2
            y: screenPosition.y - height / 2
            width: 26
            height: 26

            Rectangle {
                anchors.fill: parent
                radius: width / 2
                color: jointMarker.active ? Qt.rgba(1, 1, 1, 0.25) : jointMarker.hovered ? Qt.rgba(1, 1, 1, 0.12) : "transparent"
                border.width: jointMarker.active || jointMarker.hovered ? 3 : 2
                border.color: jointMarker.active ? "#ffffff" : jointMarker.status
                opacity: jointMarker.active || jointMarker.hovered ? 1.0 : 0.75
            }

            Rectangle {
                visible: jointMarker.active || jointMarker.hovered
                anchors.bottom: parent.top
                anchors.bottomMargin: 6
                anchors.horizontalCenter: parent.horizontalCenter
                width: angleReadout.implicitWidth + 12
                height: angleReadout.implicitHeight + 6
                radius: 4
                color: "#0e1021"
                border.color: jointMarker.status
                border.width: 1

                Text {
                    id: angleReadout
                    anchors.centerIn: parent
                    color: "white"
                    font.bold: true
                    font.pixelSize: 12
                    text: (armController.jointNames[jointMarker.index] || ("Joint " + (jointMarker.index + 1))) + "  "
                          + Math.round(armController.jointAnglesDegrees[jointMarker.index]) + "°  ·  "
                          + Math.round(Math.abs(armController.jointTorques[jointMarker.index] || 0)) + " N·m"
                }
            }
        }
    }

    // Hand handle: a dashed cyan circle at the tip that says "drag me".
    Item {
        id: handHandle
        readonly property vector3d screenPosition: root.jointScreen(armController.jointCount)
        readonly property bool active: mouse.grab.kind === "hand" && mouse.dragging
        readonly property bool hovered: mouse.hover.kind === "hand"
        visible: !taskController.busy && armController.jointCount > 0
        x: screenPosition.x - width / 2
        y: screenPosition.y - height / 2
        width: active || hovered ? 46 : 38
        height: width

        Rectangle {
            anchors.fill: parent
            radius: width / 2
            color: handHandle.active ? Qt.rgba(0.09, 0.88, 1.0, 0.22) : handHandle.hovered ? Qt.rgba(0.09, 0.88, 1.0, 0.12) : "transparent"
            border.width: 2
            border.color: "#17e0ff"
            opacity: handHandle.active || handHandle.hovered ? 1.0 : 0.6
        }
        Rectangle {
            visible: handHandle.hovered && !handHandle.active
            anchors.top: parent.bottom
            anchors.topMargin: 6
            anchors.horizontalCenter: parent.horizontalCenter
            width: handHint.implicitWidth + 12
            height: handHint.implicitHeight + 6
            radius: 4
            color: "#0e1021"
            border.color: "#17e0ff"
            Text {
                id: handHint
                anchors.centerIn: parent
                color: "white"
                font.pixelSize: 12
                text: "Drag the hand - the body follows"
            }
        }
    }
}
