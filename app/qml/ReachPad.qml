import QtQuick

// A flat 2D pad for inverse-kinematics targets, not a 3D click on the
// View3D itself - dragging maps linearly to meters, independent of
// the 3D view's camera.
Item {
    id: root

    property real rangeMeters: 2.0
    property real markerXMeters: 0
    property real markerYMeters: 0
    property real maxReachMeters: -1

    signal targetMoved(real xMeters, real yMeters)

    implicitWidth: 180
    implicitHeight: 180

    function pxFromMeters(x) { return width / 2 + (x / rangeMeters) * (width / 2) }
    function pyFromMeters(y) { return height / 2 - (y / rangeMeters) * (height / 2) }
    function metersFromPx(px) { return (px - width / 2) / (width / 2) * rangeMeters }
    function metersFromPy(py) { return -(py - height / 2) / (height / 2) * rangeMeters }

    Rectangle {
        anchors.fill: parent
        color: "#11131f"
        radius: width / 2
        border.color: "#262a40"
        border.width: 1
    }

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
