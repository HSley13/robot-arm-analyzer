import QtQuick
import QtQuick.Shapes
import QtQuick.Controls

// A per-joint load gauge: how close `value` is to `limit`, as a 270°
// arc matching AngleDial's sweep.
Item {
    id: root

    property real value: 0
    property real limit: 1
    property string unit: "N·m"

    implicitWidth: 110
    implicitHeight: 110

    readonly property real ratio: limit > 0 ? Math.abs(value) / limit : 0
    readonly property real ratioForArc: Math.min(ratio, 1.0)
    readonly property string zone: ratio < 0.6 ? "safe" : (ratio < 0.9 ? "caution" : "overload")
    readonly property color zoneColor: zone === "safe" ? "#2ecc71" : (zone === "caution" ? "#ffbf00" : "#ff4d4f")
    readonly property string zoneLabel: zone === "safe" ? "SAFE" : (zone === "caution" ? "CAUTION" : "OVERLOAD")
    readonly property real trackWidth: Math.max(6, width * 0.09)
    readonly property real arcRadius: width / 2 - trackWidth / 2 - 2

    readonly property real startAngle: 135
    readonly property real sweepTotal: 270

    Shape {
        id: gaugeShape
        anchors.fill: parent
        antialiasing: true

        ShapePath {
            strokeColor: "#30343f"
            strokeWidth: root.trackWidth
            fillColor: "transparent"
            capStyle: ShapePath.RoundCap
            PathAngleArc {
                centerX: gaugeShape.width / 2
                centerY: gaugeShape.height / 2
                radiusX: root.arcRadius
                radiusY: root.arcRadius
                startAngle: root.startAngle
                sweepAngle: root.sweepTotal
            }
        }

        ShapePath {
            strokeColor: root.zoneColor
            strokeWidth: root.trackWidth
            fillColor: "transparent"
            capStyle: ShapePath.RoundCap
            PathAngleArc {
                centerX: gaugeShape.width / 2
                centerY: gaugeShape.height / 2
                radiusX: root.arcRadius
                radiusY: root.arcRadius
                startAngle: root.startAngle
                sweepAngle: root.sweepTotal * root.ratioForArc
            }
        }
    }

    Column {
        anchors.centerIn: parent
        spacing: 1

        Label {
            anchors.horizontalCenter: parent.horizontalCenter
            color: "white"
            font.pixelSize: root.width * 0.2
            font.bold: true
            text: Math.round(root.ratio * 100) + "%"
        }
        Label {
            anchors.horizontalCenter: parent.horizontalCenter
            color: root.zoneColor
            font.pixelSize: root.width * 0.1
            font.bold: true
            text: root.zoneLabel
        }
        Label {
            anchors.horizontalCenter: parent.horizontalCenter
            color: "#9aa0ab"
            font.pixelSize: root.width * 0.085
            text: root.value.toFixed(1) + "/" + root.limit.toFixed(0) + " " + root.unit
        }
    }
}
