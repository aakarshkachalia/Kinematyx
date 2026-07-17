//
//  ModelCar.swift
//  RobotArmKit
//
//  A concrete AssemblyDefinition for a small model car: a chassis (the base part,
//  fixed to the table), four wheels that press onto axle pegs, and a body shell
//  that drops onto two studs. Pure data + geometry constants — the app target
//  reads `CarGeometry` to build matching meshes, but no RealityKit lives here.
//
//  SEQUENCING: the body covers the axles once fitted, so all four wheels must go
//  on first. That's encoded as the body step's `prerequisites`.
//

import Foundation
import simd

/// Dimensions (meters) shared by the assembly definition and the app's meshes,
/// so the visible geometry and the mating math can never drift apart. Sized as
/// chunky industrial castings, but kept within the gripper's ~0.24 m open span so
/// the parts are still grabbable.
public enum CarGeometry {
    // Chassis: a heavy flat slab.
    public static let chassisLength: Double = 0.42   // x
    public static let chassisHeight: Double = 0.10   // y
    public static let chassisWidth:  Double = 0.26   // z

    // Axle pegs on the chassis sides.
    public static let axleRadius: Double = 0.03
    public static let axleLength: Double = 0.10
    public static let axleInsetX: Double = 0.15      // front/back offset from center

    // Wheels: short cylinders (mesh axis = local +Y) with a center hub bore.
    public static let wheelRadius: Double = 0.11
    public static let wheelThickness: Double = 0.08
    public static let hubClearance: Double = 0.035

    // Studs on the chassis top that locate the body.
    public static let studRadius: Double = 0.022
    public static let studHeight: Double = 0.06
    public static let studInsetX: Double = 0.14

    // Body shell.
    public static let bodyLength: Double = 0.36  // x
    public static let bodyHeight: Double = 0.14  // y
    public static let bodyWidth:  Double = 0.22  // z

    // Masses (kg) — chunky but within the UR5's payload.
    public static let chassisMass: Double = 3.0
    public static let wheelMass: Double = 0.8
    public static let bodyMass: Double = 2.0
}

public enum ModelCar {

    /// Wheel instance ids and their human-readable names, in assembly order.
    public static let wheels: [(id: String, name: String, pegID: String, stepID: String)] = [
        ("wheelFL", "Front-left wheel",  "pegFL", "step.wheelFL"),
        ("wheelFR", "Front-right wheel", "pegFR", "step.wheelFR"),
        ("wheelRL", "Rear-left wheel",   "pegRL", "step.wheelRL"),
        ("wheelRR", "Rear-right wheel",  "pegRR", "step.wheelRR"),
    ]

    public static let chassisID = "chassis"
    public static let bodyID = "body"

    // Small rotation helpers (feature-frame convention: local +Z = insertion axis).
    private static func rotX(_ a: Double) -> simd_double3x3 {
        simd_double3x3(simd_quatd(angle: a, axis: SIMD3<Double>(1, 0, 0)))
    }
    private static func rotY(_ a: Double) -> simd_double3x3 {
        simd_double3x3(simd_quatd(angle: a, axis: SIMD3<Double>(0, 1, 0)))
    }

    /// The full model-car assembly definition.
    public static let definition: AssemblyDefinition = {
        let g = CarGeometry.self

        // --- Chassis (base part): four axle pegs + two roof studs. ---
        let halfW = g.chassisWidth / 2
        let halfH = g.chassisHeight / 2

        func peg(_ id: String, x: Double, leftSide: Bool) -> MatingFeature {
            // Left pegs point out along +Z (identity); right pegs along −Z (flip).
            let z = leftSide ? halfW : -halfW
            let orient = leftSide ? matrix_identity_double3x3 : rotY(.pi)
            return MatingFeature(
                id: id, kind: .peg,
                localPose: Pose(position: SIMD3<Double>(x, 0, z), orientation: orient),
                clearance: 0.015
            )
        }
        func stud(_ id: String, x: Double) -> MatingFeature {
            // Studs point straight up: feature +Z → part +Y.
            MatingFeature(
                id: id, kind: .stud,
                localPose: Pose(position: SIMD3<Double>(x, halfH, 0), orientation: rotX(-.pi / 2)),
                clearance: 0.012
            )
        }

        let chassis = AssemblyPart(
            id: chassisID, name: "Chassis", mass: g.chassisMass,
            graspPoses: [],   // the base is fixed to the table, never grasped
            features: [
                peg("pegFL", x:  g.axleInsetX, leftSide: true),
                peg("pegFR", x:  g.axleInsetX, leftSide: false),
                peg("pegRL", x: -g.axleInsetX, leftSide: true),
                peg("pegRR", x: -g.axleInsetX, leftSide: false),
                stud("studL", x:  g.studInsetX),
                stud("studR", x: -g.studInsetX),
            ]
        )

        // --- Wheels: hub hole on the axis (+Z), grasped at the rim (hub clear). ---
        func makeWheel(id: String, name: String) -> AssemblyPart {
            // The wheel mesh is a cylinder about its local +Y (it lies flat in the
            // tray), so the hub's insertion axis is +Y: feature +Z → part +Y.
            let hub = MatingFeature(
                id: "hub", kind: .hole,
                localPose: Pose(position: .zero, orientation: rotX(-.pi / 2)),
                clearance: g.hubClearance
            )
            // Rim grasp points in the wheel's disc plane (XZ), well clear of the
            // hub axis. Orientation is approximate (a radial pinch); it's refined
            // for auto-grasp in Phase 5.
            let grasps = [
                Pose(position: SIMD3<Double>(g.wheelRadius, 0, 0)),
                Pose(position: SIMD3<Double>(0, 0, g.wheelRadius)),
            ]
            return AssemblyPart(id: id, name: name, mass: g.wheelMass, graspPoses: grasps, features: [hub])
        }
        let wheelParts = wheels.map { makeWheel(id: $0.id, name: $0.name) }

        // --- Body shell: two downward sockets; grasped on the roof. ---
        func socket(_ id: String, x: Double) -> MatingFeature {
            // Socket opening faces down: feature +Z → part −Y.
            MatingFeature(
                id: id, kind: .socket,
                localPose: Pose(position: SIMD3<Double>(x, -g.bodyHeight / 2, 0), orientation: rotX(.pi / 2)),
                clearance: 0.012
            )
        }
        let body = AssemblyPart(
            id: bodyID, name: "Body shell", mass: g.bodyMass,
            graspPoses: [Pose(position: SIMD3<Double>(0, g.bodyHeight / 2, 0))],
            features: [socket("socketL", x: g.studInsetX), socket("socketR", x: -g.studInsetX)]
        )

        // --- Steps: four wheels (any order), then the body (needs all wheels). ---
        var steps: [AssemblyStep] = wheels.map { w in
            AssemblyStep(
                id: w.stepID,
                movingPartID: w.id, movingFeatureID: "hub",
                targetPartID: chassisID, targetFeatureID: w.pegID
            )
        }
        steps.append(AssemblyStep(
            id: "step.body",
            movingPartID: bodyID, movingFeatureID: "socketL",
            targetPartID: chassisID, targetFeatureID: "studL",
            prerequisites: wheels.map { $0.stepID }
        ))

        return AssemblyDefinition(
            name: "Model car",
            parts: [chassis] + wheelParts + [body],
            basePartID: chassisID,
            steps: steps
        )
    }()
}
