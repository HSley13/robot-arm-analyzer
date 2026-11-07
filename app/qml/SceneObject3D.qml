pragma ComponentBehavior: Bound
import QtQuick
import QtQuick3D
import Humanoid.Geometry

// One pickable object: a bottle, can, mug, box, book, ball, traffic
// cone, potted plant, standing dumbbell or kettlebell. Its local origin is the center of its base and +Y is up, so
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

    // A ring of `count` points of radius r in the XZ plane at height y.
    function ring(r, y, count) {
        const pts = []
        for (let k = 0; k < count; k++) {
            const a = 2 * Math.PI * k / count
            pts.push(Qt.vector3d(r * Math.cos(a), y, r * Math.sin(a)))
        }
        return pts
    }

    // ---- mug: glazed ceramic, coffee inside, handle on +X ----
    Node {
        visible: root.type === "mug"
        Cyl {
            radius: root.gripRadius; length: root.objectHeight
            materials: PrincipledMaterial { baseColor: "#e8e2d6"; roughness: 0.22; specularAmount: 0.7 }
        }
        Cyl {
            radius: root.gripRadius * 0.86; length: 0.004; y: root.objectHeight - 0.014
            materials: PrincipledMaterial { baseColor: "#3a2416"; roughness: 0.15 }
        }
        Cyl {
            radius: root.gripRadius * 1.006; length: root.objectHeight * 0.18; y: root.objectHeight * 0.55
            materials: PrincipledMaterial { baseColor: "#2f6f8f"; roughness: 0.3 }
        }
        Model {
            geometry: TubeGeometry {
                radius: 0.006
                radialSegments: 8
                samplesPerSpan: 6
                points: [Qt.vector3d(root.gripRadius - 0.004, root.objectHeight * 0.78, 0),
                         Qt.vector3d(root.gripRadius + 0.022, root.objectHeight * 0.72, 0),
                         Qt.vector3d(root.gripRadius + 0.026, root.objectHeight * 0.45, 0),
                         Qt.vector3d(root.gripRadius + 0.012, root.objectHeight * 0.26, 0),
                         Qt.vector3d(root.gripRadius - 0.004, root.objectHeight * 0.22, 0)]
            }
            pickable: true
            materials: PrincipledMaterial { baseColor: "#e8e2d6"; roughness: 0.22; specularAmount: 0.7 }
        }
    }

    // ---- book: standing up, hardcover - thickness along Z (how it's gripped) ----
    Node {
        visible: root.type === "book"
        readonly property real thickness: 2 * root.gripRadius
        readonly property real bookWidth: 0.15
        // Covers.
        Repeater3D {
            model: 2
            Model {
                required property int index
                position: Qt.vector3d(0, root.objectHeight / 2, (index === 0 ? 1 : -1) * (parent.thickness / 2 - 0.0015))
                geometry: RoundedBoxGeometry { size: Qt.vector3d(0.15, root.objectHeight, 0.003); radius: 0.001 }
                pickable: true
                materials: PrincipledMaterial { baseColor: "#7a1f2b"; roughness: 0.55 }
            }
        }
        // Spine.
        Model {
            position: Qt.vector3d(-0.075 + 0.002, root.objectHeight / 2, 0)
            geometry: RoundedBoxGeometry { size: Qt.vector3d(0.004, root.objectHeight, 2 * root.gripRadius); radius: 0.0015 }
            pickable: true
            materials: PrincipledMaterial { baseColor: "#6a1a25"; roughness: 0.55 }
        }
        // Gold title band on the spine.
        Model {
            position: Qt.vector3d(-0.075, root.objectHeight * 0.72, 0)
            geometry: RoundedBoxGeometry { size: Qt.vector3d(0.002, 0.03, 2 * root.gripRadius - 0.006); radius: 0.0008 }
            materials: PrincipledMaterial { baseColor: "#d4af37"; metalness: 1.0; roughness: 0.3 }
        }
        // Page block.
        Model {
            position: Qt.vector3d(0.002, root.objectHeight / 2, 0)
            geometry: RoundedBoxGeometry {
                size: Qt.vector3d(0.144, root.objectHeight - 0.008, 2 * root.gripRadius - 0.006)
                radius: 0.0008
            }
            pickable: true
            materials: PrincipledMaterial { baseColor: "#f1ead8"; roughness: 0.9 }
        }
    }

    // ---- traffic cone: square rubber base, orange body, reflective bands ----
    Node {
        id: coneNode
        visible: root.type === "cone"
        readonly property real baseHeight: 0.02
        readonly property real bodyRadius: 0.075
        function radiusAt(y) {
            return bodyRadius * (1 - (y - baseHeight) / (root.objectHeight - baseHeight))
        }
        Model {
            position: Qt.vector3d(0, parent.baseHeight / 2, 0)
            geometry: RoundedBoxGeometry { size: Qt.vector3d(0.18, 0.02, 0.18); radius: 0.008 }
            pickable: true
            materials: PrincipledMaterial { baseColor: "#1d1f22"; roughness: 0.9 }
        }
        Model {
            source: "#Cone"
            position: Qt.vector3d(0, parent.baseHeight, 0)
            scale: Qt.vector3d(2 * parent.bodyRadius / root.meshUnit, (root.objectHeight - parent.baseHeight) / root.meshUnit,
                               2 * parent.bodyRadius / root.meshUnit)
            pickable: true
            materials: PrincipledMaterial { baseColor: "#ff6a13"; roughness: 0.5 }
        }
        Repeater3D {
            model: [0.13, 0.21]
            Model {
                id: band
                required property real modelData
                readonly property real r: coneNode.radiusAt(modelData) + 0.003
                geometry: TubeGeometry {
                    closed: true
                    radius: 0.007
                    radialSegments: 6
                    samplesPerSpan: 2
                    points: root.ring(band.r, band.modelData, 32)
                }
                materials: PrincipledMaterial { baseColor: "#f4f4f4"; roughness: 0.25; specularAmount: 1.0 }
            }
        }
    }

    // ---- potted plant: terracotta pot, soil, a crown of leaves ----
    Node {
        visible: root.type === "plant"
        readonly property real potHeight: 0.14
        Cyl {
            radius: root.gripRadius; length: parent.potHeight
            materials: PrincipledMaterial { baseColor: "#b5562f"; roughness: 0.85 }
        }
        Cyl {
            radius: root.gripRadius * 1.12; length: 0.025; y: parent.potHeight - 0.025
            materials: PrincipledMaterial { baseColor: "#a44c28"; roughness: 0.85 }
        }
        Cyl {
            radius: root.gripRadius * 0.95; length: 0.004; y: parent.potHeight - 0.012
            materials: PrincipledMaterial { baseColor: "#3a2a1f"; roughness: 1.0 }
        }
        // Leaves: flattened, tilted ellipsoids fanning out and up.
        Repeater3D {
            model: 9
            Node {
                id: leaf
                required property int index
                readonly property real turn: index * 40 + (index % 2) * 17
                readonly property real lean: 18 + (index % 3) * 16
                readonly property real leafLength: 0.13 + (index % 4) * 0.025
                position: Qt.vector3d(0, 0.13, 0)
                eulerRotation: Qt.vector3d(0, turn, -lean)
                Model {
                    source: "#Sphere"
                    position: Qt.vector3d(0, parent.leafLength / 2, 0)
                    scale: Qt.vector3d(0.045 / root.meshUnit, parent.leafLength / root.meshUnit, 0.008 / root.meshUnit)
                    pickable: true
                    materials: PrincipledMaterial {
                        baseColor: leaf.index % 3 === 0 ? "#2f7d3a" : leaf.index % 3 === 1 ? "#3f9a48" : "#27682f"
                        roughness: 0.55
                    }
                }
            }
        }
    }

    // ---- kettlebell: cast-iron ball + handle, a color band coded by
    // weight like competition kettlebells ----
    Node {
        id: kettlebell
        visible: root.type === "kettlebell"
        readonly property real r: root.footprint
        readonly property color band: root.mass < 10 ? "#e8559b" : root.mass < 14 ? "#2f7fd8"
                                    : root.mass < 18 ? "#f2c230" : root.mass < 22 ? "#8a4fd1" : "#3fae5a"
        // Flat bottom ring, then the ball.
        Cyl {
            radius: parent.r * 0.62; length: 0.012
            materials: PrincipledMaterial { baseColor: "#17191c"; roughness: 0.7 }
        }
        Model {
            source: "#Sphere"
            position: Qt.vector3d(0, parent.r, 0)
            scale: Qt.vector3d(2 * parent.r / root.meshUnit, 2 * parent.r / root.meshUnit, 2 * parent.r / root.meshUnit)
            pickable: true
            materials: PrincipledMaterial { baseColor: "#24272b"; roughness: 0.55; metalness: 0.4 }
        }
        Model {
            geometry: TubeGeometry {
                closed: true
                radius: 0.006
                radialSegments: 6
                samplesPerSpan: 2
                points: root.ring(root.footprint * 1.002, root.footprint, 40)
            }
            materials: PrincipledMaterial { baseColor: kettlebell.band; roughness: 0.4 }
        }
        // Handle: two horns rising off the shoulders, joined across the top.
        Model {
            geometry: TubeGeometry {
                radius: root.gripRadius
                radialSegments: 10
                samplesPerSpan: 6
                points: [Qt.vector3d(-root.footprint * 0.62, root.footprint * 1.62, 0),
                         Qt.vector3d(-root.footprint * 0.62, root.objectHeight - 0.03, 0),
                         Qt.vector3d(-root.footprint * 0.35, root.objectHeight - root.gripRadius, 0),
                         Qt.vector3d(root.footprint * 0.35, root.objectHeight - root.gripRadius, 0),
                         Qt.vector3d(root.footprint * 0.62, root.objectHeight - 0.03, 0),
                         Qt.vector3d(root.footprint * 0.62, root.footprint * 1.62, 0)]
            }
            pickable: true
            materials: PrincipledMaterial { baseColor: "#2c3035"; roughness: 0.45; metalness: 0.5 }
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
