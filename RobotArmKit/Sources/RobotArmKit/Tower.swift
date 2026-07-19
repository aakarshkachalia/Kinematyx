//
//  Tower.swift
//  RobotArmKit
//
//  A second AssemblyDefinition: a stacked TOWER. A heavy base plate sits fixed on
//  the table; a run of identical blocks stack onto it one at a time — each block's
//  bottom SOCKET dropping onto the STUD of the piece below — and a wider cap/roof
//  finishes the top. Every step depends on the one beneath it, so the build is a
//  strict bottom-up sequence.
//
//  Like ModelCar this is pure data + geometry constants (no RealityKit); the app
//  target reads `TowerGeometry` to build matching meshes. The crucial difference
//  from the car is that each step's TARGET is the previously-placed piece — not a
//  single fixed base — so the assembly layer tracks every placed part's world pose.
//

import Foundation
import simd

/// Dimensions (meters) shared by the tower definition and the app's meshes. Kept
/// modest so the finished tower stays inside the arm's reach.
public enum TowerGeometry {
    // Base plate: a wide, heavy slab the tower stands on.
    public static let baseLength: Double = 0.30   // x
    public static let baseHeight: Double = 0.05   // y
    public static let baseWidth:  Double = 0.30   // z

    // Stacking blocks: square posts, narrower than the base.
    public static let blockLength: Double = 0.18  // x
    public static let blockHeight: Double = 0.06  // y
    public static let blockWidth:  Double = 0.18  // z

    // Cap / roof: a slightly overhanging lid.
    public static let capLength: Double = 0.24    // x
    public static let capHeight: Double = 0.05    // y
    public static let capWidth:  Double = 0.24    // z

    // Studs that locate each piece on the one below, and the sockets that receive
    // them. One centered stud per level keeps the mate unambiguous.
    public static let studRadius: Double = 0.022
    public static let studHeight: Double = 0.04

    // How many stacking blocks between the base and the cap.
    public static let blockCount: Int = 6

    // Masses (kg), well within the UR5 payload.
    public static let baseMass: Double = 3.0
    public static let blockMass: Double = 0.7
    public static let capMass: Double = 1.0
}

public enum Tower {

    public static let baseID = "base"
    public static let capID = "cap"

    /// Block instance ids and their human-readable names, bottom-to-top.
    public static var blocks: [(id: String, name: String, stepID: String)] {
        (1...TowerGeometry.blockCount).map { i in
            ("block\(i)", "Block \(i)", "step.block\(i)")
        }
    }

    // Feature-frame helpers (local +Z = insertion axis, per Assembly.swift).
    private static func rotX(_ a: Double) -> simd_double3x3 {
        simd_double3x3(simd_quatd(angle: a, axis: SIMD3<Double>(1, 0, 0)))
    }

    /// A STUD on the top face of a part, pointing up: feature +Z → part +Y.
    private static func topStud(id: String, partHalfHeight: Double) -> MatingFeature {
        MatingFeature(
            id: id, kind: .stud,
            localPose: Pose(position: SIMD3<Double>(0, partHalfHeight, 0), orientation: rotX(-.pi / 2)),
            clearance: 0.014
        )
    }

    /// A SOCKET on the bottom face of a part, opening down: feature +Z → part −Y.
    private static func bottomSocket(id: String, partHalfHeight: Double) -> MatingFeature {
        MatingFeature(
            id: id, kind: .socket,
            localPose: Pose(position: SIMD3<Double>(0, -partHalfHeight, 0), orientation: rotX(.pi / 2)),
            clearance: 0.014
        )
    }

    /// The full tower assembly definition.
    public static let definition: AssemblyDefinition = {
        let g = TowerGeometry.self

        // --- Base plate (fixed): one stud on top. Never grasped. ---
        let base = AssemblyPart(
            id: baseID, name: "Base plate", mass: g.baseMass,
            graspPoses: [],
            features: [topStud(id: "stud", partHalfHeight: g.baseHeight / 2)]
        )

        // --- Stacking blocks: socket underneath, stud on top. Grasped on a side
        //     face, clear of both the top stud and the bottom socket. ---
        func makeBlock(id: String, name: String) -> AssemblyPart {
            let halfH = g.blockHeight / 2
            let grasps = [
                Pose(position: SIMD3<Double>(g.blockLength / 2, 0, 0)),
                Pose(position: SIMD3<Double>(-g.blockLength / 2, 0, 0)),
            ]
            return AssemblyPart(
                id: id, name: name, mass: g.blockMass, graspPoses: grasps,
                features: [
                    bottomSocket(id: "socket", partHalfHeight: halfH),
                    topStud(id: "stud", partHalfHeight: halfH),
                ]
            )
        }
        let blockParts = blocks.map { makeBlock(id: $0.id, name: $0.name) }

        // --- Cap: socket underneath only (nothing stacks on the roof). ---
        let cap = AssemblyPart(
            id: capID, name: "Cap", mass: g.capMass,
            graspPoses: [Pose(position: SIMD3<Double>(g.capLength / 2, 0, 0))],
            features: [bottomSocket(id: "socket", partHalfHeight: g.capHeight / 2)]
        )

        // --- Steps: each block onto the piece below, then the cap. Strict order:
        //     every step requires the one beneath it. ---
        var steps: [AssemblyStep] = []
        var previousStud = (partID: baseID, stepID: String?.none)
        for b in blocks {
            steps.append(AssemblyStep(
                id: b.stepID,
                movingPartID: b.id, movingFeatureID: "socket",
                targetPartID: previousStud.partID, targetFeatureID: "stud",
                prerequisites: previousStud.stepID.map { [$0] } ?? []
            ))
            previousStud = (b.id, b.stepID)
        }
        steps.append(AssemblyStep(
            id: "step.cap",
            movingPartID: capID, movingFeatureID: "socket",
            targetPartID: previousStud.partID, targetFeatureID: "stud",
            prerequisites: previousStud.stepID.map { [$0] } ?? []
        ))

        return AssemblyDefinition(
            name: "Tower",
            parts: [base] + blockParts + [cap],
            basePartID: baseID,
            steps: steps
        )
    }()
}
