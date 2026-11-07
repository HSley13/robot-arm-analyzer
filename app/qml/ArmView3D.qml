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
//   * drag the torso / the column / the round base
//                                 -> lean at the hip / raise or lower the
//                                    lift / turn the waist
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
    // Where the mouse ray crosses the horizontal plane at height h, or null.
    function rayOnPlaneY(x, y, h) {
        const a = view3D.mapTo3DScene(Qt.vector3d(x, y, 0))
        const b = view3D.mapTo3DScene(Qt.vector3d(x, y, 1))
        const dir = b.minus(a)
        if (Math.abs(dir.y) < 1e-9) return null
        const t = (h - a.y) / dir.y
        return t > 0 ? a.plus(dir.times(t)) : null
    }
    function tableById(id) {
        const list = taskController.tables
        for (let i = 0; i < list.length; i++) {
            if (list[i].id === id) return list[i]
        }
        return null
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
    // The table whose top is under the mouse: {id, point} or null.
    function tableAt(x, y) {
        const hits = view3D.pickAll(x, y)
        for (let i = 0; i < hits.length; i++) {
            for (let node = hits[i].objectHit; node; node = node.parent) {
                if (node.tableId !== undefined) return { id: node.tableId, point: hits[i].scenePosition }
                if (node.objectId !== undefined) break // an object standing on it is in front
            }
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

    // ---- the body's own joints ----------------------------------------------

    readonly property real torsoLength: taskController.body.hipToShoulder + 0.25 // hip to the top of the head
    readonly property real baseTop: 0.11
    readonly property real baseRadius: taskController.body.baseRadius

    // Scene point on the torso centerline `length` above the hip at lean
    // `bend` (radians) - the body node is the hip, turned by the waist.
    function torsoPoint(length, bend) {
        return body.mapPositionToScene(Qt.vector3d(length * Math.sin(bend), length * Math.cos(bend), 0))
    }
    // How fast (px per unit) scene point p moves on screen along scene
    // direction `dir`, where dir is the point's velocity per unit of some
    // joint value.
    function screenVelocity(p, dir) {
        const a = toScreen(p)
        const b = toScreen(p.plus(dir.times(0.01)))
        return Qt.vector2d((b.x - a.x) / 0.01, (b.y - a.y) / 0.01)
    }
    // The joint change that moves a point with screen velocity v along
    // with the mouse moving by (dx, dy): least squares, damped where the
    // point barely moves on screen (looking straight along its motion).
    function followMouse(v, dx, dy) {
        return (dx * v.x + dy * v.y) / Math.max(v.x * v.x + v.y * v.y, 400)
    }
    function hipScreen() {
        const _dependencies = [root.cameraEulerRotation, root.cameraDistance, root.cameraOriginPosition,
                               root.layoutTick, root.bodyDependency]
        return view3D.camera ? toScreen(body.mapPositionToScene(Qt.vector3d(0, 0, 0))) : Qt.vector3d(-1000, -1000, 0)
    }
    function headScreen() {
        const _dependencies = [root.cameraEulerRotation, root.cameraDistance, root.cameraOriginPosition,
                               root.layoutTick, root.bodyDependency]
        return view3D.camera ? toScreen(torsoPoint(torsoLength, taskController.hipBendDegrees * Math.PI / 180))
                             : Qt.vector3d(-1000, -1000, 0)
    }
    function columnScreen(top) {
        const _dependencies = [root.cameraEulerRotation, root.cameraDistance, root.cameraOriginPosition,
                               root.layoutTick, root.bodyDependency]
        const y = top ? taskController.hipHeight - 0.13 : baseTop + 0.05
        return view3D.camera ? toScreen(Qt.vector3d(0, y, 0)) : Qt.vector3d(-1000, -1000, 0)
    }
    // The point of the base's rim nearest the camera - where the waist
    // handle is drawn.
    function waistHandleScreen() {
        const _dependencies = [root.cameraEulerRotation, root.cameraDistance, root.cameraOriginPosition,
                               root.layoutTick]
        if (!view3D.camera) return Qt.vector3d(-1000, -1000, 0)
        const c = camera.scenePosition
        const d = Math.max(1e-6, Math.hypot(c.x, c.z))
        return toScreen(Qt.vector3d(baseRadius * c.x / d, baseTop, baseRadius * c.z / d))
    }
    // A point on the base under the mouse (pushed out to the rim, so the
    // waist turns at a sensible rate wherever the base was grabbed), or null.
    function baseGrabPoint(x, y) {
        const p = rayOnPlaneY(x, y, baseTop)
        if (p === null) return null
        const r = Math.hypot(p.x, p.z)
        if (r > baseRadius + 0.02) return null
        const scale = r > 1e-6 ? baseRadius / r : 0
        return Qt.vector3d(p.x * scale, baseTop, r > 1e-6 ? p.z * scale : baseRadius)
    }

    // What's under the mouse, most specific first:
    //   {kind: "hand"} / {kind: "joint", index} / {kind: "object", id} /
    //   {kind: "lean"} / {kind: "lift"} / {kind: "waist", point} /
    //   {kind: "table", id, point} / {kind: "surface", point} / {kind: "none"}
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
        if (!taskController.busy) {
            // The body: torso (lean at the hip), column (lift), base (waist).
            const hip = hipScreen()
            if (distanceToSegment(p, hip, headScreen()).distance < 30) return { kind: "lean" }
            if (distanceToSegment(p, columnScreen(false), columnScreen(true)).distance < 16) return { kind: "lift" }
            const handle = waistHandleScreen()
            const onBase = baseGrabPoint(x, y)
            if (Math.hypot(x - handle.x, y - handle.y) < 20) {
                const c = camera.scenePosition
                const d = Math.max(1e-6, Math.hypot(c.x, c.z))
                return { kind: "waist", point: Qt.vector3d(baseRadius * c.x / d, baseTop, baseRadius * c.z / d) }
            }
            if (onBase !== null) return { kind: "waist", point: onBase }
        }
        const table = tableAt(x, y)
        if (table !== null) return { kind: "table", id: table.id, point: table.point }
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
        // Waist drag: the grabbed rim point and the yaw when it was grabbed.
        property vector3d waistGrab
        property real waistYawAtGrab: 0
        // Table drag: grab offset from its center, in its top's plane.
        property vector2d tableOffset
        // Joint drag: screen angle around the joint at the last move.
        property real lastScreenAngle: 0
        property real jointSign: 1

        cursorShape: dragging && grab.kind !== "none" && grab.kind !== "surface" ? Qt.ClosedHandCursor
                   : hover.kind === "lift" ? Qt.SizeVerCursor
                   : hover.kind === "waist" ? Qt.SizeHorCursor
                   : ["hand", "joint", "table", "lean"].indexOf(hover.kind) >= 0 ? Qt.OpenHandCursor
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
            } else if (grab.kind === "lean" || grab.kind === "lift" || grab.kind === "waist") {
                taskController.cancelReach()
                if (grab.kind === "waist") {
                    waistGrab = grab.point
                    waistYawAtGrab = taskController.waistYawDegrees
                }
            } else if (grab.kind === "table") {
                const table = root.tableById(grab.id)
                tableOffset = Qt.vector2d(table.x - grab.point.x, table.z - grab.point.z)
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
            } else if (grab.kind === "lean") {
                // The head swings about the hip: d(point)/d(bend).
                const bend = taskController.hipBendDegrees * Math.PI / 180
                const L = root.torsoLength
                const v = root.screenVelocity(root.torsoPoint(L, bend),
                                              body.mapDirectionToScene(Qt.vector3d(L * Math.cos(bend), -L * Math.sin(bend), 0)))
                taskController.hipBendDegrees = taskController.hipBendDegrees + root.followMouse(v, dx, dy) * 180 / Math.PI
            } else if (grab.kind === "lift") {
                const v = root.screenVelocity(Qt.vector3d(0, taskController.hipHeight, 0), Qt.vector3d(0, 1, 0))
                taskController.hipHeight = taskController.hipHeight + root.followMouse(v, dx, dy)
            } else if (grab.kind === "waist") {
                // The grabbed rim point turns with the waist: d(point)/d(yaw)
                // is (z, 0, -x) for a turn about +Y.
                const turned = (taskController.waistYawDegrees - waistYawAtGrab) * Math.PI / 180
                const g = waistGrab
                const p = Qt.vector3d(g.x * Math.cos(turned) + g.z * Math.sin(turned), g.y,
                                      -g.x * Math.sin(turned) + g.z * Math.cos(turned))
                const v = root.screenVelocity(p, Qt.vector3d(p.z, 0, -p.x))
                taskController.waistYawDegrees = taskController.waistYawDegrees + root.followMouse(v, dx, dy) * 180 / Math.PI
            } else if (grab.kind === "table") {
                const table = root.tableById(grab.id)
                const p = table !== null ? root.rayOnPlaneY(event.x, event.y, table.height) : null
                if (p !== null) taskController.moveTable(grab.id, p.x + tableOffset.x, p.z + tableOffset.y)
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
        if (taskController.clickMode !== "" && (hit.kind === "surface" || hit.kind === "table")) {
            // An armed click mode from the panel still works.
            taskController.handleClick(hit.point.x, hit.point.z)
        } else if (hit.kind === "object") {
            taskController.selectedId = hit.id
            objectMenu.openFor(hit.id, x, y)
        } else if (hit.kind === "table") {
            spotMenu.openAt(hit.point, x, y, hit.id)
        } else if (hit.kind === "surface") {
            spotMenu.openAt(hit.point, x, y, -1)
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

    // Handles for the body's own joints: lean (at the hip), lift (on the
    // column), waist (on the base's rim, facing you).
    component BodyHandle: Item {
        id: handle
        property vector3d screenPosition
        property string glyph
        property string label
        property string kind
        readonly property bool active: mouse.grab.kind === kind && mouse.dragging
        readonly property bool hovered: mouse.hover.kind === kind
        visible: !taskController.busy
        x: screenPosition.x - width / 2
        y: screenPosition.y - height / 2
        width: 26
        height: 26

        Rectangle {
            anchors.fill: parent
            radius: width / 2
            color: handle.active ? Qt.rgba(1, 0.75, 0.2, 0.35) : Qt.rgba(0.06, 0.07, 0.13, handle.hovered ? 0.85 : 0.6)
            border.width: handle.active || handle.hovered ? 2 : 1
            border.color: handle.active || handle.hovered ? "#ffbf3a" : Qt.rgba(1, 0.75, 0.23, 0.6)
            Text {
                anchors.centerIn: parent
                color: "#ffd27a"
                font.pixelSize: 14
                font.bold: true
                text: handle.glyph
            }
        }
        Rectangle {
            visible: handle.active || handle.hovered
            anchors.left: parent.right
            anchors.leftMargin: 6
            anchors.verticalCenter: parent.verticalCenter
            width: handleLabel.implicitWidth + 12
            height: handleLabel.implicitHeight + 6
            radius: 4
            color: "#0e1021"
            border.color: "#ffbf3a"
            Text {
                id: handleLabel
                anchors.centerIn: parent
                color: "white"
                font.pixelSize: 12
                text: handle.label
            }
        }
    }

    BodyHandle {
        kind: "lean"
        glyph: "↷"
        screenPosition: root.hipScreen()
        label: "Lean " + Math.round(taskController.hipBendDegrees) + "°  -  drag the torso"
    }
    BodyHandle {
        kind: "lift"
        glyph: "↕"
        screenPosition: {
            const a = root.columnScreen(false)
            const b = root.columnScreen(true)
            return Qt.vector3d((a.x + b.x) / 2, (a.y + b.y) / 2, 0)
        }
        label: "Lift " + taskController.hipHeight.toFixed(2) + " m  -  drag up / down"
    }
    BodyHandle {
        kind: "waist"
        glyph: "⟲"
        screenPosition: root.waistHandleScreen()
        label: "Waist " + Math.round(taskController.waistYawDegrees) + "°  -  drag the base sideways"
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
