pragma ComponentBehavior: Bound
import QtQuick
import QtQuick3D

// Procedural multi-fingered hand: one tapered cylinder per link, one
// rounded sphere per joint, and a single palm "mound" where every
// finger's own metacarpal segment converges - generic over however
// many chains/joints handController's loaded config actually has
// (nothing here hardcodes "five fingers").
//
// Two things a first version of this file got wrong, both fixed here:
// every link was the same uniform thin diameter (reads as a skeleton of
// sticks, not a finger - real phalanges are relatively chunky and taper
// from base to tip), and each finger's own metacarpal segment - the
// piece connecting it to the palm at all - was never drawn, since it
// falls outside any single chain's own position list (see
// HandController::rootSegmentFor*Chain). Both together made fingers
// look like they were floating, disconnected from any hand shape.
//
// handController.jointPositionsForChain()/etc. are Q_INVOKABLE methods,
// not real Q_PROPERTYs (they take a chain-name argument, which
// Q_PROPERTY can't do) - calling them from a binding does not, by
// itself, make QML re-run that binding when the underlying pose
// changes. _data's binding deliberately reads handController.revision
// (a plain counter bumped on every recompute) purely to give it a real
// dependency to react to; without that read, this would render only
// the very first pose and never update again.
Node {
    id: root

    readonly property real meshUnit: 100.0

    // Tapers from a chunky base (nearest the palm) to a slimmer tip,
    // but never thinner than about half the base - real finger
    // segments stay relatively thick all the way to the fingertip,
    // unlike a sharply-pointed cone.
    function _linkDiameterMeters(localIndexInFinger) {
        const base = 0.020
        const taperPerStep = 0.0028
        return Math.max(0.010, base - localIndexInFinger * taperPerStep)
    }

    function _flatten() {
        const names = handController.chainNames
        const joints = []
        const jointTaper = []
        const linkMid = []
        const linkLen = []
        const linkRot = []
        const linkTaper = []
        let palmPosition = null

        for (let c = 0; c < names.length; c++) {
            const name = names[c]

            // The wrist chain's own joints are all pure rotations at
            // the same point (every DH 'a' is zero) - nothing to draw
            // for it directly, but its own tip IS where every finger's
            // metacarpal segment converges, so capture that as the
            // palm anchor instead of rendering it as a chain.
            if (name === "wrist") {
                const wristPositions = handController.jointPositionsForChain(name)
                if (wristPositions.length > 0) palmPosition = wristPositions[wristPositions.length - 1]
                continue
            }

            const positions = handController.jointPositionsForChain(name)
            for (let i = 0; i < positions.length; i++) {
                joints.push(positions[i])
                jointTaper.push(i)
            }

            // The metacarpal segment, connecting this finger's base to
            // the palm - outside this chain's own position list, so
            // HandController exposes it separately (see
            // rootSegmentFor*Chain's own doc comment for why).
            const rootMid = handController.rootSegmentMidpointForChain(name)
            const rootLen = handController.rootSegmentLengthForChain(name)
            const rootRot = handController.rootSegmentRotationForChain(name)
            let localIndex = 0
            if (rootMid.length > 0) {
                linkMid.push(rootMid[0])
                linkLen.push(rootLen[0])
                linkRot.push(rootRot[0])
                linkTaper.push(localIndex)
                localIndex++
            }

            const mids = handController.linkMidpointsForChain(name)
            const lens = handController.linkLengthsForChain(name)
            const rots = handController.linkRotationsForChain(name)
            for (let i = 0; i < mids.length; i++) {
                linkMid.push(mids[i])
                linkLen.push(lens[i])
                linkRot.push(rots[i])
                linkTaper.push(localIndex)
                localIndex++
            }
        }

        return {
            joints: joints, jointTaper: jointTaper,
            linkMid: linkMid, linkLen: linkLen, linkRot: linkRot, linkTaper: linkTaper,
            palmPosition: palmPosition,
        }
    }

    readonly property var _data: {
        handController.revision // forces re-evaluation on every recompute, see note above
        return root._flatten()
    }

    // One shared material for links, joints, and the palm, so the whole
    // hand reads as one continuous matte surface - a glove, not a
    // skeleton of shiny metal rods with glowing ball joints.
    PrincipledMaterial {
        id: handMaterial
        baseColor: "#dcbfa3"
        metalness: 0.08
        roughness: 0.55
    }

    Repeater3D {
        model: root._data.linkMid.length

        Model {
            required property int index
            source: "#Cylinder"
            position: root._data.linkMid[index]
            rotation: root._data.linkRot[index]
            scale: {
                const diameter = root._linkDiameterMeters(root._data.linkTaper[index]) / root.meshUnit
                return Qt.vector3d(diameter, root._data.linkLen[index] / root.meshUnit, diameter)
            }
            materials: handMaterial
        }
    }

    Repeater3D {
        model: root._data.joints.length

        Model {
            required property int index
            source: "#Sphere"
            position: root._data.joints[index]
            scale: {
                // Slightly larger than the adjoining link so the joint
                // reads as a rounded knuckle bulge, not a visible seam.
                const diameter = (root._linkDiameterMeters(root._data.jointTaper[index]) * 1.12) / root.meshUnit
                return Qt.vector3d(diameter, diameter, diameter)
            }
            materials: handMaterial
        }
    }

    Model {
        visible: root._data.palmPosition !== null
        source: "#Sphere"
        position: root._data.palmPosition ? root._data.palmPosition : Qt.vector3d(0, 0, 0)
        scale: {
            const diameter = 0.052 / root.meshUnit
            return Qt.vector3d(diameter, diameter, diameter)
        }
        materials: handMaterial
    }
}
