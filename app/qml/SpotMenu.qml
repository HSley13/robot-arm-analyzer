import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

// The menu that opens when you click the floor or a table: put the held
// object down there, send the hand there, drop a new object - and add
// furniture (on the floor) or remove the table you clicked.
Popup {
    id: menu

    property vector3d point
    property int tableId: -1
    readonly property var held: taskController.heldObject
    readonly property var table: {
        const list = taskController.tables
        for (let i = 0; i < list.length; i++) {
            if (list[i].id === tableId) return list[i]
        }
        return null
    }

    function openAt(scenePoint, x, y, clickedTableId) {
        point = scenePoint
        tableId = clickedTableId
        menu.x = Math.max(8, Math.min(x + 12, parent.width - implicitWidth - 8))
        menu.y = Math.max(8, Math.min(y - 20, parent.height - implicitHeight - 8))
        open()
    }

    padding: 10
    width: 230
    background: Rectangle {
        color: "#171a2b"
        radius: 12
        border.color: "#262a40"
        border.width: 1
    }

    contentItem: ColumnLayout {
        spacing: 6

        Label {
            color: "white"
            font.bold: true
            text: (menu.table !== null ? menu.table.label : "Floor") + "  ·  x " + menu.point.x.toFixed(2)
                  + ", z " + menu.point.z.toFixed(2)
        }

        Button {
            Layout.fillWidth: true
            visible: menu.held.id !== undefined
            enabled: !taskController.busy
            highlighted: true
            text: "Put the " + (menu.held.type || "object") + " down here"
            onClicked: {
                menu.close()
                taskController.putDown(menu.point.x, menu.point.z)
            }
        }
        Button {
            Layout.fillWidth: true
            enabled: !taskController.busy
            text: "Reach here"
            onClicked: {
                menu.close()
                // A hand's width above the surface, not into it.
                taskController.reachTo(menu.point.x, menu.point.y + 0.10, menu.point.z)
            }
        }

        Label { color: "#9aa0ab"; font.pixelSize: 11; text: "Add an object here:" }
        Flow {
            Layout.fillWidth: true
            spacing: 4
            Repeater {
                model: taskController.objectTypes
                Button {
                    required property string modelData
                    text: modelData
                    onClicked: {
                        menu.close()
                        taskController.addObject(modelData, 0, menu.point.x, menu.point.z)
                    }
                }
            }
        }

        Label {
            visible: menu.table === null
            color: "#9aa0ab"
            font.pixelSize: 11
            text: "Add furniture here:"
        }
        Flow {
            visible: menu.table === null
            Layout.fillWidth: true
            spacing: 4
            Repeater {
                model: taskController.tableKinds
                Button {
                    required property string modelData
                    text: modelData
                    onClicked: {
                        menu.close()
                        taskController.addTable(modelData, menu.point.x, menu.point.z)
                    }
                }
            }
        }

        Button {
            Layout.fillWidth: true
            visible: menu.table !== null
            enabled: !taskController.busy
            text: menu.table !== null ? "Remove this " + menu.table.kind : ""
            palette.buttonText: "#ff4d4f"
            onClicked: {
                const id = menu.tableId
                menu.close()
                taskController.removeTable(id)
            }
        }
    }
}
