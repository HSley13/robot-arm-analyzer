import QtQuick

// A minimal line chart for runPayloadScan(): payload capacity (Y)
// against reach (X), one line per engine.
Item {
    id: root

    property var reachMeters: []
    property var exactPayloadKg: []
    property var aiPayloadKg: []
    property bool aiAvailable: false

    implicitWidth: 260
    implicitHeight: 160

    Canvas {
        id: canvas
        anchors.fill: parent

        property var reach: root.reachMeters
        property var exact: root.exactPayloadKg
        property var ai: root.aiPayloadKg
        property bool aiOn: root.aiAvailable

        // Qt.callLater(), not a direct requestPaint(): these can fire
        // during the initial layout pass, before the window is
        // attached to the render thread - calling requestPaint()
        // synchronously then caused an intermittent "Updates can only
        // be scheduled from GUI thread" warning.
        onReachChanged: Qt.callLater(requestPaint)
        onExactChanged: Qt.callLater(requestPaint)
        onAiChanged: Qt.callLater(requestPaint)
        onAiOnChanged: Qt.callLater(requestPaint)
        onWidthChanged: Qt.callLater(requestPaint)
        onHeightChanged: Qt.callLater(requestPaint)

        onPaint: {
            const ctx = getContext("2d")
            ctx.clearRect(0, 0, width, height)

            const n = reach.length
            if (n < 2 || exact.length !== n) return

            let minReach = Math.min.apply(Math, reach)
            let maxReach = Math.max.apply(Math, reach)
            const hasAi = aiOn && ai.length === n
            const allPayloads = hasAi ? exact.concat(ai) : exact.slice()
            let minPayload = Math.min.apply(Math, allPayloads)
            let maxPayload = Math.max.apply(Math, allPayloads)
            if (maxReach <= minReach) maxReach = minReach + 1
            if (maxPayload <= minPayload) maxPayload = minPayload + 1

            const marginLeft = 40
            const marginRight = 8
            const marginTop = 10
            const marginBottom = 18
            const plotWidth = width - marginLeft - marginRight
            const plotHeight = height - marginTop - marginBottom

            function xAt(r) { return marginLeft + (r - minReach) / (maxReach - minReach) * plotWidth }
            function yAt(p) { return marginTop + plotHeight - (p - minPayload) / (maxPayload - minPayload) * plotHeight }

            ctx.strokeStyle = "#30343f"
            ctx.lineWidth = 1
            ctx.strokeRect(marginLeft, marginTop, plotWidth, plotHeight)

            function drawLine(values, color) {
                ctx.strokeStyle = color
                ctx.lineWidth = 2
                ctx.beginPath()
                for (let i = 0; i < n; i++) {
                    const x = xAt(reach[i])
                    const y = yAt(values[i])
                    if (i === 0) ctx.moveTo(x, y)
                    else ctx.lineTo(x, y)
                }
                ctx.stroke()
            }

            drawLine(exact, "#17e0ff")
            if (hasAi) drawLine(ai, "#ff8a3d")

            ctx.fillStyle = "#9aa0ab"
            ctx.font = "10px sans-serif"
            ctx.fillText(minReach.toFixed(2) + "m", marginLeft, height - 4)
            ctx.fillText(maxReach.toFixed(2) + "m", width - marginRight - 34, height - 4)
            ctx.fillText(maxPayload.toFixed(1) + "kg", 2, marginTop + 9)
            ctx.fillText(minPayload.toFixed(1) + "kg", 2, marginTop + plotHeight)
        }
    }
}
