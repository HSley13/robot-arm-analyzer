pragma ComponentBehavior: Bound
import QtQuick
import QtQuick3D
import Humanoid.Geometry
import Humanoid.Backend

// One table from TaskController.tables, drawn by kind:
//   * "table"        - wooden top on four steel legs;
//   * "coffee table" - low, thick walnut top with a shelf underneath;
//   * "counter"      - a cabinet with a stone top and a door + handle on
//                      the side facing the robot.
// Its origin is the center of the floor footprint. The top surface is
// pickable and named "table", and `tableId` says which one it is, so the
// 3D view can put things on it and drag it around.
Node {
    id: root

    property int tableId: -1
    property string kind: "table"
    property real tableWidth: 0.5   // along X
    property real tableDepth: 0.45  // along Z
    property real tableHeight: 0.72
    property bool highlighted: false

    readonly property real top: kind === "coffee table" ? 0.045 : kind === "counter" ? 0.04 : 0.03
    // Which way is the robot (at the origin)? The counter's door faces it.
    readonly property bool faceAlongX: Math.abs(position.x) >= Math.abs(position.z)
    readonly property real faceSign: faceAlongX ? (position.x > 0 ? -1 : 1) : (position.z > 0 ? -1 : 1)

    // The top: the surface things stand on.
    Model {
        objectName: "table"
        pickable: true
        position: Qt.vector3d(0, root.tableHeight - root.top / 2, 0)
        geometry: RoundedBoxGeometry {
            size: Qt.vector3d(root.tableWidth, root.top, root.tableDepth)
            radius: root.kind === "coffee table" ? 0.014 : 0.006
        }
        materials: PrincipledMaterial {
            baseColor: root.kind === "coffee table" ? "#4a3326" : root.kind === "counter" ? "#d9d6cf" : "#6b5643"
            roughness: root.kind === "counter" ? 0.35 : 0.6
            emissiveFactor: root.highlighted ? Qt.vector3d(0.03, 0.18, 0.22) : Qt.vector3d(0, 0, 0)
        }
    }

    // ---- table + coffee table: legs ----------------------------------------
    Repeater3D {
        model: root.kind === "counter" ? 0 : 4
        Model {
            id: leg
            required property int index
            readonly property real inset: root.kind === "coffee table" ? 0.05 : 0.04
            readonly property real legHeight: root.tableHeight - root.top
            readonly property real legSize: root.kind === "coffee table" ? 0.05 : 0.04
            position: Qt.vector3d((index % 2 === 0 ? -1 : 1) * (root.tableWidth / 2 - inset), legHeight / 2,
                                  (index < 2 ? -1 : 1) * (root.tableDepth / 2 - inset))
            geometry: RoundedBoxGeometry {
                size: Qt.vector3d(leg.legSize, leg.legHeight, leg.legSize)
                radius: root.kind === "coffee table" ? 0.012 : 0.006
                taper: root.kind === "coffee table" ? 0.8 : 1.0
            }
            materials: PrincipledMaterial {
                baseColor: root.kind === "coffee table" ? "#3b291f" : "#2b2f37"
                metalness: root.kind === "coffee table" ? 0.0 : 0.6
                roughness: root.kind === "coffee table" ? 0.6 : 0.45
            }
        }
    }
    // Coffee table shelf.
    Model {
        visible: root.kind === "coffee table"
        position: Qt.vector3d(0, 0.11, 0)
        geometry: RoundedBoxGeometry {
            size: Qt.vector3d(root.tableWidth - 0.10, 0.02, root.tableDepth - 0.10)
            radius: 0.006
        }
        materials: PrincipledMaterial { baseColor: "#3b291f"; roughness: 0.65 }
    }

    // ---- counter: cabinet, plinth, door, handle -----------------------------
    Node {
        id: cabinet
        visible: root.kind === "counter"
        readonly property real bodyHeight: root.tableHeight - root.top

        // Recessed kick plinth.
        Model {
            position: Qt.vector3d(0, 0.04, 0)
            geometry: RoundedBoxGeometry {
                size: Qt.vector3d(root.tableWidth - 0.06, 0.08, root.tableDepth - 0.06)
                radius: 0.004
            }
            materials: PrincipledMaterial { baseColor: "#1c1f26"; roughness: 0.8 }
        }
        // Cabinet body.
        Model {
            position: Qt.vector3d(0, 0.08 + (parent.bodyHeight - 0.08) / 2, 0)
            geometry: RoundedBoxGeometry {
                size: Qt.vector3d(root.tableWidth - 0.02, cabinet.bodyHeight - 0.08, root.tableDepth - 0.02)
                radius: 0.006
            }
            materials: PrincipledMaterial { baseColor: "#3d4a5c"; roughness: 0.55 }
        }
        // Door on the side facing the robot, with a steel bar handle.
        Node {
            id: door
            readonly property real doorHeight: parent.bodyHeight - 0.14
            readonly property real faceWidth: root.faceAlongX ? root.tableDepth : root.tableWidth
            readonly property real faceOffset: (root.faceAlongX ? root.tableWidth : root.tableDepth) / 2 - 0.006
            position: root.faceAlongX ? Qt.vector3d(root.faceSign * faceOffset, 0.08 + 0.03 + doorHeight / 2, 0)
                                      : Qt.vector3d(0, 0.08 + 0.03 + doorHeight / 2, root.faceSign * faceOffset)
            eulerRotation.y: root.faceAlongX ? 90 : 0

            Model {
                geometry: RoundedBoxGeometry {
                    size: Qt.vector3d(door.faceWidth - 0.06, door.doorHeight, 0.012)
                    radius: 0.004
                }
                materials: PrincipledMaterial { baseColor: "#4a596d"; roughness: 0.5 }
            }
            Model {
                position: Qt.vector3d(parent.faceWidth / 2 - 0.07, parent.doorHeight / 2 - 0.12, root.faceSign * 0.016)
                geometry: RoundedBoxGeometry { size: Qt.vector3d(0.014, 0.16, 0.014); radius: 0.006 }
                materials: PrincipledMaterial { baseColor: "#c9ced4"; metalness: 1.0; roughness: 0.25 }
            }
        }
    }
}
