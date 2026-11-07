pragma ComponentBehavior: Bound
import QtQuick
import QtQuick3D
import Humanoid.Geometry

// One pickable object: a bottle, a can, a box, a ball or a standing
// dumbbell. Its local origin is the center of its base and +Y is up, so
// it can be dropped straight onto a surface - or, when held, turned so
// its up axis follows the hand's thumb (Hand3D does that).
//
// Sizes come from TaskController (gripRadius, footprint, height) so what
// you see is exactly what the planner reasons about.
Node {
    id: root

    // TaskController's id for it (-1 when it's the one being carried), so
    // a pick in the 3D view can tell which object was hit.
    property int objectId: -1
    property string type: "bottle"
    property real mass: 1.0
    property real gripRadius: 0.033
    property real footprint: 0.034
    property real objectHeight: 0.23
    property bool selected: false
    property bool showSelection: true

    readonly property real meshUnit: 100.0

    component Cyl: Model {
        property real radius: 0.01
        property real length: 0.01
        property real y: 0
        source: "#Cylinder"
        position: Qt.vector3d(0, y + length / 2, 0)
        scale: Qt.vector3d(2 * radius / root.meshUnit, length / root.meshUnit, 2 * radius / root.meshUnit)
        pickable: true
    }

    // ---- bottle: translucent-looking blue plastic, label, white cap ----
    Node {
        visible: root.type === "bottle"
        readonly property real r: root.gripRadius
        Cyl {
            radius: parent.r; length: root.objectHeight * 0.66
            materials: PrincipledMaterial { baseColor: "#4f9bd8"; roughness: 0.18; specularAmount: 0.8 }
        }
        Cyl {
            radius: parent.r * 1.005; length: root.objectHeight * 0.22; y: root.objectHeight * 0.2
            materials: PrincipledMaterial { baseColor: "#f2f2ee"; roughness: 0.6 }
        }
        Model {
            source: "#Cone"
            position: Qt.vector3d(0, root.objectHeight * 0.66, 0)
            scale: Qt.vector3d(2 * parent.r / root.meshUnit, root.objectHeight * 0.17 / root.meshUnit,
                               2 * parent.r / root.meshUnit)
            pickable: true
            materials: PrincipledMaterial { baseColor: "#4f9bd8"; roughness: 0.18; specularAmount: 0.8 }
        }
        Cyl {
            radius: parent.r * 0.42; length: root.objectHeight * 0.1; y: root.objectHeight * 0.8
            materials: PrincipledMaterial { baseColor: "#4f9bd8"; roughness: 0.2 }
        }
        Cyl {
            radius: parent.r * 0.48; length: root.objectHeight * 0.07; y: root.objectHeight * 0.9
            materials: PrincipledMaterial { baseColor: "#f5f5f5"; roughness: 0.5 }
        }
    }

    // ---- can: brushed aluminum with a red wrap ----
    Node {
        visible: root.type === "can"
        Cyl {
            radius: root.gripRadius; length: root.objectHeight
            materials: PrincipledMaterial { baseColor: "#c2c6cc"; metalness: 1.0; roughness: 0.3 }
        }
        Cyl {
            radius: root.gripRadius * 1.004; length: root.objectHeight * 0.7; y: root.objectHeight * 0.13
            materials: PrincipledMaterial { baseColor: "#c62828"; roughness: 0.35; specularAmount: 0.7 }
        }
    }

    // ---- box: cardboard with a tape stripe ----
    Node {
        visible: root.type === "box"
        Model {
            position: Qt.vector3d(0, root.objectHeight / 2, 0)
            geometry: RoundedBoxGeometry {
                size: Qt.vector3d(2 * root.gripRadius, root.objectHeight, 2 * root.gripRadius)
                radius: 0.003
            }
            pickable: true
            materials: PrincipledMaterial { baseColor: "#b98a55"; roughness: 0.85 }
        }
        Model {
            position: Qt.vector3d(0, root.objectHeight + 0.0004, 0)
            geometry: RoundedBoxGeometry { size: Qt.vector3d(2 * root.gripRadius * 1.01, 0.0012, 0.018); radius: 0.0004 }
            materials: PrincipledMaterial { baseColor: "#d9c69a"; roughness: 0.4 }
        }
    }

    // ---- ball: orange with dark seams ----
    Node {
        visible: root.type === "ball"
        Model {
            source: "#Sphere"
            position: Qt.vector3d(0, root.gripRadius, 0)
            scale: Qt.vector3d(2 * root.gripRadius / root.meshUnit, 2 * root.gripRadius / root.meshUnit,
                               2 * root.gripRadius / root.meshUnit)
            pickable: true
            materials: PrincipledMaterial { baseColor: "#e8742a"; roughness: 0.75 }
        }
        Repeater3D {
            model: 2
            Model {
                required property int index
                position: Qt.vector3d(0, root.gripRadius, 0)
                eulerRotation: index === 0 ? Qt.vector3d(0, 0, 0) : Qt.vector3d(90, 0, 0)
                geometry: TubeGeometry {
                    closed: true
                    radius: 0.0012
                    radialSegments: 6
                    samplesPerSpan: 2
                    points: {
                        const pts = []
                        const r = root.gripRadius * 1.002
                        for (let k = 0; k < 36; k++) {
                            const a = 2 * Math.PI * k / 36
                            pts.push(Qt.vector3d(r * Math.cos(a), 0, r * Math.sin(a)))
                        }
                        return pts
                    }
                }
                materials: PrincipledMaterial { baseColor: "#2b1a12"; roughness: 0.8 }
            }
        }
    }

    // ---- dumbbell standing on one end ----
    Node {
        visible: root.type === "dumbbell"
        readonly property real plateThickness: Math.min(0.07, 0.010 + 0.003 * root.mass)
        readonly property real plateRadius: root.footprint
        Cyl {
            radius: parent.plateRadius; length: parent.plateThickness
            materials: PrincipledMaterial { baseColor: "#1b1d21"; roughness: 0.8 }
        }
        Cyl {
            radius: 0.020; length: 0.010; y: parent.plateThickness
            materials: PrincipledMaterial { baseColor: "#c9ced4"; metalness: 1.0; roughness: 0.28 }
        }
        Cyl {
            radius: root.gripRadius; length: 0.132; y: parent.plateThickness
            materials: PrincipledMaterial { baseColor: "#5b6168"; metalness: 1.0; roughness: 0.45 }
        }
        Cyl {
            radius: 0.020; length: 0.010; y: parent.plateThickness + 0.122
            materials: PrincipledMaterial { baseColor: "#c9ced4"; metalness: 1.0; roughness: 0.28 }
        }
        Cyl {
            radius: parent.plateRadius; length: parent.plateThickness; y: parent.plateThickness + 0.132
            materials: PrincipledMaterial { baseColor: "#1b1d21"; roughness: 0.8 }
        }
    }

    // Selection ring on the surface around the object.
    Model {
        visible: root.selected && root.showSelection
        position: Qt.vector3d(0, 0.002, 0)
        geometry: TubeGeometry {
            closed: true
            radius: 0.0022
            radialSegments: 6
            samplesPerSpan: 2
            points: {
                const pts = []
                const r = root.footprint + 0.018
                for (let k = 0; k < 40; k++) {
                    const a = 2 * Math.PI * k / 40
                    pts.push(Qt.vector3d(r * Math.cos(a), 0, r * Math.sin(a)))
                }
                return pts
            }
        }
        materials: PrincipledMaterial {
            baseColor: "#17e0ff"
            emissiveFactor: Qt.vector3d(0.09, 0.88, 1.0)
            lighting: PrincipledMaterial.NoLighting
        }
    }
}
