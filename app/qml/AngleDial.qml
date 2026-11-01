import QtQuick
import QtQuick.Shapes
import QtQuick.Controls

// A circular drag control for a joint angle. Same value/moved contract
// a Slider has.
Item {
    id: root

    property real value: 0
    property real from: -180
    property real to: 180

    signal moved(real value)

    implicitWidth: 110
    implicitHeight: 110
    opacity: enabled ? 1.0 : 0.4

    readonly property real normalized: to > from ? (value - from) / (to - from) : 0
    readonly property real trackWidth: Math.max(6, width * 0.09)
    readonly property real arcRadius: width / 2 - trackWidth / 2 - 2

    readonly property real startAngle: 135
    readonly property real sweepTotal: 270
    readonly property real handleAngleDeg: root.startAngle + root.sweepTotal * root.normalized
    readonly property real handleAngleRad: handleAngleDeg * Math.PI / 180

    Shape {
        id: dialShape
        anchors.fill: parent
        antialiasing: true

        ShapePath {
            strokeColor: "#30343f"
            strokeWidth: root.trackWidth
            fillColor: "transparent"
            capStyle: ShapePath.RoundCap
            PathAngleArc {
                centerX: dialShape.width / 2
                centerY: dialShape.height / 2
                radiusX: root.arcRadius
                radiusY: root.arcRadius
                startAngle: root.startAngle
                sweepAngle: root.sweepTotal
            }
        }

        ShapePath {
            strokeColor: "#17e0ff"
            strokeWidth: root.trackWidth
            fillColor: "transparent"
            capStyle: ShapePath.RoundCap
            PathAngleArc {
                centerX: dialShape.width / 2
                centerY: dialShape.height / 2
                radiusX: root.arcRadius
                radiusY: root.arcRadius
                startAngle: root.startAngle
                sweepAngle: root.sweepTotal * root.normalized
            }
        }
    }

    Label {
        anchors.centerIn: parent
        color: "white"
        font.pixelSize: root.width * 0.2
        font.bold: true
        text: Math.round(root.value) + "°"
    }

    Rectangle {
        id: handle
        width: root.trackWidth + 6
        height: width
        radius: width / 2
        color: dragHandler.active ? "#ffffff" : "#17e0ff"
        border.color: "#0e1021"
        border.width: 2
        x: root.width / 2 + root.arcRadius * Math.cos(root.handleAngleRad) - width / 2
        y: root.height / 2 + root.arcRadius * Math.sin(root.handleAngleRad) - height / 2
    }

    DragHandler {
        id: dragHandler
        target: null
        enabled: root.enabled

        onCentroidChanged: {
            if (!active) return
            const dx = centroid.position.x - root.width / 2
            const dy = centroid.position.y - root.height / 2
            let angleDeg = Math.atan2(dy, dx) * 180 / Math.PI
            if (angleDeg < 0) angleDeg += 360

            let rel = ((angleDeg - root.startAngle) % 360 + 360) % 360
            if (rel > root.sweepTotal) {
                // In the gap: snap to the nearer end of the arc.
                const distToEnd = rel - root.sweepTotal
                const distToStart = 360 - rel
                rel = (distToEnd < distToStart) ? root.sweepTotal : 0
            }

            const t = rel / root.sweepTotal
            root.moved(root.from + t * (root.to - root.from))
        }
    }
}
