pragma ComponentBehavior: Bound
import QtQuick
import QtQuick3D
import Humanoid.Geometry

// The humanoid's mobile base and lift column. It doesn't turn with the
// waist; the column stretches from the base up to the hip joint
// (taskController.hipHeight), so lowering the lift visibly telescopes it.
Node {
    id: root

    readonly property real meshUnit: 100.0
    readonly property real baseRadius: taskController.body.baseRadius
    readonly property real baseTop: 0.11
    readonly property real columnTop: taskController.hipHeight - 0.115 // just under the waist turntable

    // Base body.
    Model {
        source: "#Cylinder"
        position: Qt.vector3d(0, 0.065, 0)
        scale: Qt.vector3d(2 * root.baseRadius * 0.97 / root.meshUnit, 0.09 / root.meshUnit,
                           2 * root.baseRadius * 0.97 / root.meshUnit)
        materials: PrincipledMaterial { baseColor: "#3a4552"; roughness: 0.5; metalness: 0.15 }
    }
    // Top cover.
    Model {
        source: "#Cylinder"
        position: Qt.vector3d(0, root.baseTop - 0.002, 0)
        scale: Qt.vector3d(2 * root.baseRadius * 0.9 / root.meshUnit, 0.008 / root.meshUnit,
                           2 * root.baseRadius * 0.9 / root.meshUnit)
        materials: PrincipledMaterial { baseColor: "#7f91a3"; roughness: 0.46 }
    }
    // Rubber bumper.
    Model {
        position: Qt.vector3d(0, 0.05, 0)
        geometry: TubeGeometry {
            closed: true
            radius: 0.014
            radialSegments: 10
            samplesPerSpan: 2
            points: {
                const pts = []
                for (let k = 0; k < 48; k++) {
                    const a = 2 * Math.PI * k / 48
                    pts.push(Qt.vector3d(root.baseRadius * Math.cos(a), 0, root.baseRadius * Math.sin(a)))
                }
                return pts
            }
        }
        materials: PrincipledMaterial { baseColor: "#1d2127"; roughness: 0.9 }
    }
    // Status light strip around the base.
    Model {
        position: Qt.vector3d(0, 0.088, 0)
        geometry: TubeGeometry {
            closed: true
            radius: 0.0025
            radialSegments: 6
            samplesPerSpan: 2
            points: {
                const pts = []
                for (let k = 0; k < 48; k++) {
                    const a = 2 * Math.PI * k / 48
                    const r = root.baseRadius * 0.972
                    pts.push(Qt.vector3d(r * Math.cos(a), 0, r * Math.sin(a)))
                }
                return pts
            }
        }
        materials: PrincipledMaterial {
            baseColor: "#17e0ff"
            emissiveFactor: Qt.vector3d(0.05, 0.5, 0.6)
            lighting: PrincipledMaterial.NoLighting
        }
    }
    // Three caster wheels.
    Repeater3D {
        model: 3
        Model {
            required property int index
            readonly property real angle: index * 2 * Math.PI / 3 + Math.PI / 6
            source: "#Cylinder"
            position: Qt.vector3d(0.2 * Math.cos(angle), 0.025, 0.2 * Math.sin(angle))
            eulerRotation: Qt.vector3d(90, -angle * 180 / Math.PI, 0)
            scale: Qt.vector3d(0.05 / root.meshUnit, 0.025 / root.meshUnit, 0.05 / root.meshUnit)
            materials: PrincipledMaterial { baseColor: "#15181c"; roughness: 0.85 }
        }
    }

    // Column collar on the base, then the telescoping column.
    Model {
        position: Qt.vector3d(0, root.baseTop + 0.02, 0)
        geometry: RoundedBoxGeometry { size: Qt.vector3d(0.15, 0.04, 0.15); radius: 0.012 }
        materials: PrincipledMaterial { baseColor: "#5d6f82"; roughness: 0.52 }
    }
    Model {
        id: column
        readonly property real bottom: root.baseTop + 0.03
        readonly property real top: Math.max(bottom + 0.01, root.columnTop)
        position: Qt.vector3d(0, (bottom + top) / 2, 0)
        // (Inner objects can't see their parent's properties unqualified -
        // hence the id.)
        geometry: RoundedBoxGeometry { size: Qt.vector3d(0.10, column.top - column.bottom, 0.10); radius: 0.01 }
        materials: PrincipledMaterial { baseColor: "#c9ced4"; metalness: 1.0; roughness: 0.3 }
    }
}
