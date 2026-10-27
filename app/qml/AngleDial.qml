import QtQuick
import QtQuick.Shapes
import QtQuick.Controls

// A circular drag control for setting a joint angle, replacing a plain
// Slider - user feedback was that sliders "ain't the best way to do
// it" for something that's actually a rotation. Same `value`/`moved`
// contract a Slider has (bind `value`, react to `moved`), so swapping
// it in doesn't change how the parent persists the result.
Item {
    id: root

    property real value: 0
    property real from: -180
    property real to: 180

    signal moved(real value)

    implicitWidth: 110
    implicitHeight: 110
    // Item.enabled disables input handling but doesn't dim anything by
    // itself - bind opacity explicitly so a disabled dial actually
    // looks disabled, matching how Slider/Switch already behave.
    opacity: enabled ? 1.0 : 0.4

    readonly property real normalized: to > from ? (value - from) / (to - from) : 0
    readonly property real trackWidth: Math.max(6, width * 0.09)
    readonly property real arcRadius: width / 2 - trackWidth / 2 - 2

    // Same 270° sweep/gap as RadialGauge - see that component for why.
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

    // Dragging anywhere in the control maps the pointer's angle from
    // center back into [from, to], clamped to the dial's own arc - not
    // just tracking the small handle, so the whole ring is grabbable.
    DragHandler {
        id: dragHandler
        target: null
        // Input handlers aren't automatically gated by the Item's own
        // `enabled` - without this, a "disabled" dial would still
        // respond to dragging even though it looks dimmed and inert.
        enabled: root.enabled

        onCentroidChanged: {
            if (!active) return
            const dx = centroid.position.x - root.width / 2
            const dy = centroid.position.y - root.height / 2
            let angleDeg = Math.atan2(dy, dx) * 180 / Math.PI
            if (angleDeg < 0) angleDeg += 360

            // Relative to the arc's own start, wrapped into [0, 360).
            let rel = ((angleDeg - root.startAngle) % 360 + 360) % 360
            if (rel > root.sweepTotal) {
                // In the gap at the bottom - snap to whichever end of
                // the arc the pointer is actually closer to, rather
                // than letting it jump to a fixed end regardless of
                // which side was dragged past.
                const distToEnd = rel - root.sweepTotal
                const distToStart = 360 - rel
                rel = (distToEnd < distToStart) ? root.sweepTotal : 0
            }

            const t = rel / root.sweepTotal
            root.moved(root.from + t * (root.to - root.from))
        }
    }
}
