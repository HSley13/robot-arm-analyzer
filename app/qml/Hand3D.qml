pragma ComponentBehavior: Bound
import QtQuick
import QtQuick3D

// Procedural multi-fingered hand: one small sphere per joint, one
// oriented tapered cylinder per link - the same "cylinders + spheres,
// built in code, no imported mesh" philosophy ArmView3D.qml itself
// uses, generic over however many chains/joints handController's
// loaded config actually has (nothing here hardcodes "five fingers").
//
// handController.jointPositionsForChain()/linkMidpointsForChain()/etc.
// are Q_INVOKABLE methods, not real Q_PROPERTYs (they take a chain-name
// argument, which Q_PROPERTY can't do) - calling them from a binding
// does not, by itself, make QML re-run that binding when the
// underlying pose changes. _data's binding deliberately reads
// handController.revision (a plain counter bumped on every recompute)
// purely to give it a real dependency to react to; without that read,
// this would render only the very first pose and never update again.
Node {
    id: root

    readonly property real meshUnit: 100.0
    readonly property real jointDiameterMeters: 0.016
    readonly property real linkDiameterMeters: 0.010

    function _flatten() {
        const names = handController.chainNames
        const joints = []
        const linkMid = []
        const linkLen = []
        const linkRot = []
        const linkIsWrist = []

        for (let c = 0; c < names.length; c++) {
            const name = names[c]
            const isWrist = name === "wrist"

            const positions = handController.jointPositionsForChain(name)
            for (let i = 0; i < positions.length; i++) joints.push(positions[i])

            const mids = handController.linkMidpointsForChain(name)
            const lens = handController.linkLengthsForChain(name)
            const rots = handController.linkRotationsForChain(name)
            for (let i = 0; i < mids.length; i++) {
                linkMid.push(mids[i])
                linkLen.push(lens[i])
                linkRot.push(rots[i])
                linkIsWrist.push(isWrist)
            }
        }

        return { joints: joints, linkMid: linkMid, linkLen: linkLen, linkRot: linkRot, linkIsWrist: linkIsWrist }
    }

    readonly property var _data: {
        handController.revision // forces re-evaluation on every recompute, see note above
        return root._flatten()
    }

    Repeater3D {
        model: root._data.linkMid.length

        Model {
            required property int index
            source: "#Cylinder"
            position: root._data.linkMid[index]
            rotation: root._data.linkRot[index]
            scale: {
                const diameter = root.linkDiameterMeters / root.meshUnit
                return Qt.vector3d(diameter, root._data.linkLen[index] / root.meshUnit, diameter)
            }
            materials: PrincipledMaterial {
                baseColor: root._data.linkIsWrist[index] ? "#2a2d36" : "#d7dbe2"
                metalness: root._data.linkIsWrist[index] ? 0.2 : 0.75
                roughness: 0.35
            }
        }
    }

    Repeater3D {
        model: root._data.joints.length

        Model {
            required property int index
            source: "#Sphere"
            position: root._data.joints[index]
            scale: {
                const diameter = root.jointDiameterMeters / root.meshUnit
                return Qt.vector3d(diameter, diameter, diameter)
            }
            materials: PrincipledMaterial {
                baseColor: "#17e0ff"
                emissiveFactor: Qt.vector3d(0.05, 0.5, 0.6)
                metalness: 0.0
                roughness: 0.3
            }
        }
    }
}
