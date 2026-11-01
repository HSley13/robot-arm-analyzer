import QtQuick

// A 2D "click the workspace" pad for inverse-kinematics targets - the
// top-down view of the arm's own XY motion plane, not a 3D click on
// the View3D itself. Deliberately not 3D ray-picking: turning a 2D
// screen click into a 3D world point needs intersecting the camera's
// ray with the arm's motion plane, which depends on wherever the
// camera currently is orbited to - exactly the kind of camera-math
// that's genuinely hard to get right with no way to see it running.
// A flat pad sidesteps that entirely: dragging within it maps
// linearly to meters, independent of the 3D view's camera at all.
Item {
    id: root

    property real rangeMeters: 2.0 // the pad shows -rangeMeters..+rangeMeters on both axes
    property real markerXMeters: 0
    property real markerYMeters: 0
    // Drawn as a faint reference circle - the arm's own total reach,
    // so a target placed near or past the edge visibly explains why
    // moveToTarget() might not land exactly on it.
    property real maxReachMeters: -1

    signal targetMoved(real xMeters, real yMeters)

    implicitWidth: 180
    implicitHeight: 180

    function pxFromMeters(x) { return width / 2 + (x / rangeMeters) * (width / 2) }
    function pyFromMeters(y) { return height / 2 - (y / rangeMeters) * (height / 2) }
    function metersFromPx(px) { return (px - width / 2) / (width / 2) * rangeMeters }
    function metersFromPy(py) { return -(py - height / 2) / (height / 2) * rangeMeters }

    // A true circle, not a rounded square - this pad represents the
    // arm's reach, which is itself circular, so the backdrop reads as
    // the same shape as the reference reach ring drawn on top of it
    // rather than a square window it happens to sit inside.
    Rectangle {
        anchors.fill: parent
        color: "#11131f"
        radius: width / 2
        border.color: "#262a40"
        border.width: 1
    }

    // The base, at the pad's own center - every arm this project
    // models starts its reach from the origin.
    Rectangle {
        anchors.centerIn: parent
        width: 8
        height: 8
        radius: 4
        color: "#2a2a33"
    }

    Rectangle {
        visible: root.maxReachMeters > 0
        anchors.centerIn: parent
        width: Math.min(root.width, root.height) * Math.min(root.maxReachMeters / root.rangeMeters, 1.0)
        height: width
        radius: width / 2
        color: "transparent"
        border.color: "#30343f"
        border.width: 1
    }

    Rectangle {
        id: marker
        width: 16
        height: 16
        radius: 8
        color: dragHandler.active ? "#ffffff" : "#17e0ff"
        border.color: "#0e1021"
        border.width: 2
        x: root.pxFromMeters(root.markerXMeters) - width / 2
        y: root.pyFromMeters(root.markerYMeters) - height / 2
    }

    // Dragging anywhere on the pad (not just the marker) moves the
    // target, clamped to the pad's own visible range.
    DragHandler {
        id: dragHandler
        target: null

        onCentroidChanged: {
            if (!active) return
            const clampedX = Math.max(0, Math.min(root.width, centroid.position.x))
            const clampedY = Math.max(0, Math.min(root.height, centroid.position.y))
            root.targetMoved(root.metersFromPx(clampedX), root.metersFromPy(clampedY))
        }
    }
}
