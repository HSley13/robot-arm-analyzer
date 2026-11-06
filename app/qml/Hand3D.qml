pragma ComponentBehavior: Bound
import QtQuick
import QtQuick3D
import Humanoid.Geometry

// The humanoid hand, modeled after a real dexterous robot hand: matte
// blue-grey molded shells for the palm and every phalanx, dark hinge
// pins showing in the gaps between segments, small screw holes on the
// back of each segment, a rubber pad on the fingertips and palm, and a
// cuff where the hand meets the wrist mechanism.
//
// Everything here is in the HAND's own local frame - the frame
// handController's forward kinematics works in:
//   +X toward the fingertips, +Y out of the palm (palmar), +Z thumb side.
// ArmView3D parents this node to the arm's wrist, so the hand follows the
// arm without this file knowing anything about it.
//
// Shapes come from forward kinematics, not hand-placed: every phalanx is
// placed at its link's midpoint with linkFramesForChain()'s orientation
// (+Y along the bone, +X along the hinge pin, +Z toward the palm), so a
// differently-proportioned hand config renders correctly with no QML
// changes.
//
// handController.jointPositionsForChain()/etc. are Q_INVOKABLE methods
// (they take a chain name, which a Q_PROPERTY can't), so a binding that
// calls them doesn't re-run on its own when the pose changes. _data
// reads handController.revision (bumped on every recompute) to give it a
// real dependency.
Node {
    id: root

    // Whether the payload dumbbell is in the hand, and how heavy it is.
    property bool holdingPayload: false
    property real payloadKg: 0

    // Where the handle of a held object sits: inside the curled fingers
    // of the power-grip pose, along the hand's Z (thumb) axis.
    readonly property vector3d gripCenter: Qt.vector3d(0.097, 0.040, 0.0)

    readonly property real meshUnit: 100.0 // built-in #Cylinder etc. are 100 units across

    // Per-finger shell sizes (meters): width across the knuckle and
    // thickness palm-to-back. Real fingers are a little wider than thick.
    readonly property var fingerWidths: ({ "thumb": 0.0255, "index": 0.0185, "middle": 0.0192,
                                           "ring": 0.0182, "pinky": 0.0164 })
    function widthOf(chain) { return fingerWidths[chain] !== undefined ? fingerWidths[chain] : 0.018 }

    function _flatten() {
        const names = handController.chainNames
        const links = []
        const pins = []
        const roots = []

        for (let c = 0; c < names.length; c++) {
            const name = names[c]
            if (name === "wrist") continue // pure rotations at one point - the palm stands in for it
            const width = widthOf(name)

            const positions = handController.jointPositionsForChain(name)
            const mids = handController.linkMidpointsForChain(name)
            const lens = handController.linkLengthsForChain(name)
            const frames = handController.linkFramesForChain(name)
            for (let i = 0; i < mids.length; i++) {
                // Fingers taper a little toward the tip.
                const w = width * (1.0 - 0.07 * i)
                links.push({ position: mids[i], rotation: frames[i], length: lens[i], width: w,
                             thickness: w * 0.9, distal: i === mids.length - 1 })
                // A hinge pin at the base of every phalanx.
                pins.push({ position: positions[i], rotation: frames[i], width: w, thickness: w * 0.9 })
            }

            const rootMid = handController.rootSegmentMidpointForChain(name)
            if (rootMid.length > 0) {
                roots.push({ name: name, position: rootMid[0],
                             rotation: handController.rootSegmentFrameForChain(name)[0],
                             length: handController.rootSegmentLengthForChain(name)[0],
                             width: width, base: handController.chainBasePosition(name) })
            }
        }
        return { links: links, pins: pins, roots: roots }
    }

    readonly property var _data: {
        handController.revision // forces re-evaluation on every recompute, see note above
        return root._flatten()
    }

    // ---- materials -------------------------------------------------------

    PrincipledMaterial {
        id: shellMaterial
        baseColor: "#7f91a3"
        metalness: 0.0
        roughness: 0.48
        specularAmount: 0.45
    }
    PrincipledMaterial {
        id: shellDarkMaterial
        baseColor: "#6c7e90"
        metalness: 0.0
        roughness: 0.55
    }
    PrincipledMaterial {
        id: holeMaterial
        baseColor: "#161b21"
        roughness: 0.9
    }
    PrincipledMaterial {
        id: hingeMaterial
        baseColor: "#4b545e"
        metalness: 0.7
        roughness: 0.35
    }
    PrincipledMaterial {
        id: rubberMaterial
        baseColor: "#2c333b"
        roughness: 0.92
    }
    PrincipledMaterial {
        id: padMaterial
        baseColor: "#55636f"
        roughness: 0.9
    }
    PrincipledMaterial {
        id: steelMaterial
        baseColor: "#c8cdd3"
        metalness: 1.0
        roughness: 0.28
    }

    // ---- reusable pieces -------------------------------------------------

    // A small blind screw hole on a surface whose outward normal is the
    // node's local -Z (the back of a finger segment).
    component ScrewHole: Model {
        source: "#Cylinder"
        eulerRotation.x: 90
        scale: Qt.vector3d(0.0024 / root.meshUnit, 0.0012 / root.meshUnit, 0.0024 / root.meshUnit)
        materials: holeMaterial
    }

    // One finger segment. Local frame: +Y along the bone, +X along the
    // hinge, +Z toward the palm. The shell is a little shorter than the
    // joint-to-joint length so a dark gap with the hinge pin shows at
    // every knuckle.
    component Phalanx: Node {
        id: phalanx
        property real length: 0.04
        property real width: 0.018
        property real thickness: 0.016
        property bool distal: false
        readonly property real gap: 0.0032
        readonly property real shellLength: Math.max(0.004, length - gap)

        Model {
            geometry: RoundedBoxGeometry {
                size: Qt.vector3d(phalanx.width, phalanx.shellLength, phalanx.thickness)
                radius: phalanx.distal ? phalanx.thickness * 0.46 : phalanx.thickness * 0.3
                taper: phalanx.distal ? 0.86 : 0.95
            }
            materials: shellMaterial
        }

        // Fingertip pad (palm side of the last segment).
        Model {
            visible: phalanx.distal
            position: Qt.vector3d(0, phalanx.shellLength * 0.06, phalanx.thickness * 0.36)
            geometry: RoundedBoxGeometry {
                size: Qt.vector3d(phalanx.width * 0.74, phalanx.shellLength * 0.72, phalanx.thickness * 0.34)
                radius: phalanx.thickness * 0.15
                taper: 0.85
            }
            materials: rubberMaterial
        }

        // Two screw holes on the back of the segment, like the reference
        // hand's molded shells.
        ScrewHole { position: Qt.vector3d(-phalanx.width * 0.2, phalanx.distal ? -phalanx.shellLength * 0.05 : 0, -phalanx.thickness * 0.5) }
        ScrewHole { position: Qt.vector3d(phalanx.width * 0.2, phalanx.distal ? -phalanx.shellLength * 0.05 : 0, -phalanx.thickness * 0.5) }
    }

    // ---- palm ------------------------------------------------------------

    // The palm follows the wrist chain's own roll/deviation.
    Node {
        id: palm
        position: { handController.revision; return handController.palmPosition }
        rotation: { handController.revision; return handController.palmRotation }

        // Main palm shell: narrower at the wrist, widening toward the
        // knuckles, and turned a few degrees about the palm normal so its
        // far edge follows the knuckle line (the pinky knuckle sits further
        // back than the middle finger's). RoundedBoxGeometry tapers along
        // its own +Y, so it's built with +Y as the palm's length and turned
        // -90 degrees about Z to point that along the palm's +X.
        Node {
            position: Qt.vector3d(0.049, -0.0005, -0.001)
            eulerRotation: Qt.vector3d(0, 7, 0)

            Model {
                eulerRotation.z: -90
                geometry: RoundedBoxGeometry {
                    size: Qt.vector3d(0.031, 0.088, 0.075)
                    radius: 0.0135
                    taper: 1.12
                }
                materials: shellMaterial
            }

            // Knuckle ridge: the shell steps up slightly across the back of
            // the knuckles, like the molded cover on the reference hand.
            Model {
                position: Qt.vector3d(0.034, -0.0128, 0)
                eulerRotation.x: 90
                geometry: RoundedBoxGeometry {
                    size: Qt.vector3d(0.016, 0.081, 0.010)
                    radius: 0.0048
                }
                materials: shellMaterial
            }
            // Screw holes on the back of the hand.
            Repeater3D {
                model: [Qt.vector3d(-0.024, 0, -0.022), Qt.vector3d(-0.024, 0, 0.022),
                        Qt.vector3d(0.010, 0, -0.026), Qt.vector3d(0.010, 0, 0.026),
                        Qt.vector3d(0.034, -0.0024, -0.012), Qt.vector3d(0.034, -0.0024, 0.012)]
                Model {
                    required property vector3d modelData
                    position: Qt.vector3d(modelData.x, -0.0157 + modelData.y, modelData.z)
                    source: "#Cylinder"
                    scale: Qt.vector3d(0.0026 / root.meshUnit, 0.0012 / root.meshUnit, 0.0026 / root.meshUnit)
                    materials: holeMaterial
                }
            }
        }

        // Rubber grip pad on the palm side.
        Model {
            position: Qt.vector3d(0.054, 0.0165, -0.004)
            eulerRotation: Qt.vector3d(0, 7, 0)
            geometry: RoundedBoxGeometry {
                size: Qt.vector3d(0.050, 0.003, 0.054)
                radius: 0.0014
            }
            materials: padMaterial
        }

        // Wrist cuff: the flared lower edge of the palm shell that the
        // wrist clevis disappears into.
        Model {
            position: Qt.vector3d(0.0, -0.0005, 0.0)
            eulerRotation: Qt.vector3d(0, 0, -90) // geometry's +Y -> palm's +X (toward the fingers)
            geometry: RoundedBoxGeometry {
                size: Qt.vector3d(0.040, 0.030, 0.070)
                radius: 0.010
                taper: 1.12
            }
            materials: shellDarkMaterial
        }
    }

    // ---- fingers ---------------------------------------------------------

    Repeater3D {
        model: root._data.links.length

        Phalanx {
            required property int index
            readonly property var link: root._data.links[index]
            position: link.position
            rotation: link.rotation
            length: link.length
            width: link.width
            thickness: link.thickness
            distal: link.distal
        }
    }

    // Hinge pins - dark axles across every knuckle gap.
    Repeater3D {
        model: root._data.pins.length

        Node {
            required property int index
            readonly property var pin: root._data.pins[index]
            position: pin.position
            rotation: pin.rotation

            Model {
                source: "#Cylinder"
                eulerRotation.z: 90
                scale: Qt.vector3d(pin.thickness * 0.42 / root.meshUnit, pin.width * 0.86 / root.meshUnit,
                                   pin.thickness * 0.42 / root.meshUnit)
                materials: hingeMaterial
            }
        }
    }

    // Metacarpals (each chain's root segment). For the four fingers these
    // stay hidden inside the palm shell; for the thumb this is the big
    // thenar segment that swings out when the thumb opposes.
    Repeater3D {
        model: root._data.roots.length

        Node {
            required property int index
            readonly property var seg: root._data.roots[index]
            readonly property bool isThumb: seg.name === "thumb"
            position: seg.position
            rotation: seg.rotation

            Model {
                geometry: RoundedBoxGeometry {
                    size: Qt.vector3d(seg.width * (isThumb ? 1.25 : 0.9), seg.length * (isThumb ? 0.95 : 0.8),
                                      seg.width * (isThumb ? 1.1 : 0.8))
                    radius: seg.width * 0.32
                    taper: isThumb ? 0.92 : 1.0
                }
                materials: shellMaterial
            }
        }
    }

    // Thumb base (CMC) housing where the thumb pivots out of the palm.
    Model {
        readonly property vector3d base: { handController.revision; return handController.chainBasePosition("thumb") }
        position: base
        geometry: RoundedBoxGeometry {
            size: Qt.vector3d(0.030, 0.028, 0.026)
            radius: 0.010
        }
        materials: shellDarkMaterial
    }

    // ---- held payload: a dumbbell in the power grip ------------------------

    Node {
        id: dumbbell
        visible: root.holdingPayload && root.payloadKg > 0
        position: root.gripCenter

        readonly property real plateRadius: Math.min(0.10, 0.030 + 0.0095 * Math.sqrt(root.payloadKg))
        readonly property real plateThickness: Math.min(0.07, 0.010 + 0.003 * root.payloadKg)
        readonly property real innerZ: 0.066

        // Handle along the hand's Z (thumb) axis.
        Model {
            source: "#Cylinder"
            eulerRotation.x: 90
            scale: Qt.vector3d(0.025 / root.meshUnit, (2 * dumbbell.innerZ + 0.004) / root.meshUnit, 0.025 / root.meshUnit)
            materials: PrincipledMaterial { baseColor: "#5b6168"; metalness: 1.0; roughness: 0.45 }
        }

        Repeater3D {
            model: 2
            Node {
                required property int index
                readonly property real side: index === 0 ? 1 : -1
                // Collar.
                Model {
                    source: "#Cylinder"
                    eulerRotation.x: 90
                    position: Qt.vector3d(0, 0, parent.side * (dumbbell.innerZ - 0.004))
                    scale: Qt.vector3d(0.040 / root.meshUnit, 0.010 / root.meshUnit, 0.040 / root.meshUnit)
                    materials: steelMaterial
                }
                // Rubber-coated plate.
                Model {
                    source: "#Cylinder"
                    eulerRotation.x: 90
                    position: Qt.vector3d(0, 0, parent.side * (dumbbell.innerZ + dumbbell.plateThickness / 2))
                    scale: Qt.vector3d(2 * dumbbell.plateRadius / root.meshUnit, dumbbell.plateThickness / root.meshUnit,
                                       2 * dumbbell.plateRadius / root.meshUnit)
                    materials: PrincipledMaterial { baseColor: "#1b1d21"; roughness: 0.8 }
                }
                // Steel end cap.
                Model {
                    source: "#Cylinder"
                    eulerRotation.x: 90
                    position: Qt.vector3d(0, 0, parent.side * (dumbbell.innerZ + dumbbell.plateThickness + 0.004))
                    scale: Qt.vector3d(0.034 / root.meshUnit, 0.008 / root.meshUnit, 0.034 / root.meshUnit)
                    materials: steelMaterial
                }
            }
        }
    }
}
