import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

// The menu that opens when you click an object in the 3D view: pick it
// up, move it to exact coordinates, change its mass, or delete it.
// (Dragging the object moves it too - this is for when you want numbers.)
Popup {
    id: menu

    property int objectId: -1
    readonly property var object: {
        const list = taskController.objects
        for (let i = 0; i < list.length; i++) {
            if (list[i].id === objectId) return list[i]
        }
        return null
    }

    function openFor(id, x, y) {
        objectId = id
        if (object === null) return
        xField.text = object.x.toFixed(2)
        zField.text = object.z.toFixed(2)
        massField.text = object.mass.toFixed(2)
        menu.x = Math.max(8, Math.min(x + 12, parent.width - implicitWidth - 8))
        menu.y = Math.max(8, Math.min(y - 20, parent.height - implicitHeight - 8))
        open()
    }

    onClosed: taskController.selectedId = -1
    // It's gone (deleted, or picked up) - nothing left to edit.
    onObjectChanged: if (opened && (object === null || object.held)) close()

    padding: 10
    width: 230
    background: Rectangle {
        color: "#171a2b"
        radius: 12
        border.color: "#17e0ff"
        border.width: 1
    }

    component Field: TextField {
        implicitWidth: 56
        validator: DoubleValidator {}
        selectByMouse: true
    }

    contentItem: ColumnLayout {
        spacing: 6

        Label {
            color: "white"
            font.bold: true
            font.pixelSize: 14
            text: menu.object === null ? "" : menu.object.type.charAt(0).toUpperCase() + menu.object.type.slice(1)
                                              + "  ·  " + menu.object.mass.toFixed(2) + " kg"
        }
        Label {
            color: "#9aa0ab"
            font.pixelSize: 11
            text: menu.object === null ? "" : (menu.object.y > 0 ? "On the table" : "On the floor")
                                              + " - drag it to move it"
        }

        Button {
            Layout.fillWidth: true
            text: taskController.heldId >= 0 ? "Pick up (hands full)" : "Pick up"
            enabled: !taskController.busy && taskController.heldId < 0
            highlighted: true
            onClicked: {
                const id = menu.objectId
                menu.close()
                taskController.pickUp(id)
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 4
            Label { color: "#9aa0ab"; text: "x" }
            Field { id: xField }
            Label { color: "#9aa0ab"; text: "z" }
            Field { id: zField }
            Button {
                Layout.fillWidth: true
                text: "Move"
                onClicked: taskController.moveObject(menu.objectId, parseFloat(xField.text), parseFloat(zField.text))
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 4
            Label { color: "#9aa0ab"; text: "mass" }
            Field { id: massField; validator: DoubleValidator { bottom: 0.01 } }
            Label { color: "#9aa0ab"; text: "kg" }
            Button {
                Layout.fillWidth: true
                text: "Set"
                onClicked: taskController.setObjectMass(menu.objectId, parseFloat(massField.text))
            }
        }

        Button {
            Layout.fillWidth: true
            text: "Delete"
            palette.buttonText: "#ff4d4f"
            onClicked: {
                const id = menu.objectId
                menu.close()
                taskController.removeObject(id)
            }
        }
    }
}
