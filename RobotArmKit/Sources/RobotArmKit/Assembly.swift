//
//  Assembly.swift
//  RobotArmKit
//
//  The assembly graph: the pure-data description of a multi-part product (parts,
//  their mating features, and the ordered steps that join them) plus the
//  transform math that turns "feature A mates with feature B" into "the moving
//  part must reach THIS world pose."
//
//  This is deliberately UI-framework-agnostic — no RealityKit, no SwiftUI. The
//  app target owns the meshes, physics, sounds, and panels; this file owns the
//  geometry of WHERE things go and WHETHER a plan is well-formed.
//
//  SNAP-FIT, NOT PHYSICS INSERTION: assembly succeeds when a held part's mating
//  feature comes within tolerance of its target socket, at which point the app
//  snaps the part to the exact mated pose (Phase 3). Nothing here simulates
//  contact forces; this file just computes the exact poses and validates plans.
//

import Foundation
import simd

// MARK: - Mating features

/// The kind of a mating feature. Complementary kinds can join: a peg enters a
/// hole, a stud enters a socket. `surface` mates flush against another surface.
public enum MatingKind: String, Sendable, CaseIterable {
    case peg, hole, stud, socket, surface

    /// The kind this one mates with.
    public var complement: MatingKind {
        switch self {
        case .peg:     return .hole
        case .hole:    return .peg
        case .stud:    return .socket
        case .socket:  return .stud
        case .surface: return .surface
        }
    }

    /// Whether two kinds can join.
    public func canMate(with other: MatingKind) -> Bool { other == complement }

    /// The "male" side (peg/stud) is the one that inserts, and by convention
    /// defines the insertion direction. Holes/sockets receive it.
    public var isMale: Bool { self == .peg || self == .stud }
}

/// A place on a part where it joins another part.
///
/// FRAME CONVENTION: `localPose` is the feature's frame expressed in its parent
/// part's local frame, and the feature's local **+Z is the insertion axis** — a
/// peg slides out of its part along +Z; a hole is entered by approaching along
/// its own +Z. The frame origin is the mating datum (peg base / hole mouth
/// center). Keeping a single, explicit convention is what lets the mate math
/// below stay simple and general.
public struct MatingFeature: Identifiable, Sendable, Equatable {
    public let id: String
    public let kind: MatingKind
    public let localPose: Pose
    /// Radius (m) of clear space the feature needs around its insertion axis. A
    /// grasp inside this zone would physically block insertion; used by the
    /// grasp-vs-feature validation below.
    public let clearance: Double

    public init(id: String, kind: MatingKind, localPose: Pose, clearance: Double) {
        self.id = id
        self.kind = kind
        self.localPose = localPose
        self.clearance = clearance
    }

    /// The insertion axis (feature local +Z) expressed in the parent part frame.
    public var localInsertionAxis: SIMD3<Double> {
        simd_normalize(localPose.orientation.columns.2)
    }

    /// The default relative transform that seats a MALE feature frame inside its
    /// mating FEMALE feature frame: coincident origins, insertion axes OPPOSED
    /// (the male +Z points into the female, i.e. antiparallel to the female +Z).
    /// A 180° rotation about the feature X axis flips Y and Z to achieve this.
    ///
    /// Round pegs/holes are symmetric about Z, so which perpendicular axis we
    /// flip about doesn't matter. A keyed (non-round) feature would override this
    /// by supplying an explicit `mateTransform` on its `AssemblyStep`.
    public static func opposedMateTransform() -> Pose {
        let r = simd_double3x3(columns: (
            SIMD3<Double>(1,  0,  0),
            SIMD3<Double>(0, -1,  0),
            SIMD3<Double>(0,  0, -1)
        ))
        return Pose(position: .zero, orientation: r)
    }
}

// MARK: - Parts

/// One rigid part of an assembly: its identity, mass, where it can be grasped,
/// and the features by which it joins other parts.
public struct AssemblyPart: Identifiable, Sendable {
    public let id: String
    public let name: String
    public let mass: Double
    /// Candidate grasp poses (the GRIPPER TOOL frame relative to the part's
    /// origin). Several so a planner can fall back to a reachable one; every one
    /// should leave the part's mating features clear (see the validation below).
    public let graspPoses: [Pose]
    public let features: [MatingFeature]

    public init(
        id: String, name: String, mass: Double,
        graspPoses: [Pose], features: [MatingFeature]
    ) {
        self.id = id
        self.name = name
        self.mass = mass
        self.graspPoses = graspPoses
        self.features = features
    }

    public func feature(id: String) -> MatingFeature? {
        features.first { $0.id == id }
    }
}

public extension AssemblyPart {
    /// Approximate half-width (m) of the gripper's occupied region at the grasp
    /// point, used only for the grasp-vs-feature overlap check. A real gripper's
    /// fingers span a bit; this keeps a small buffer around the grasp point.
    static let graspClearanceRadius: Double = 0.02

    /// Whether a grasp pose overlaps a given feature's clearance zone — i.e. the
    /// grasp point sits close enough to the feature that the fingers would block
    /// insertion (the classic "a wheel grabbed across its hub can't go on the
    /// axle" mistake).
    func grasp(_ grasp: Pose, blocks feature: MatingFeature) -> Bool {
        simd_length(grasp.position - feature.localPose.position)
            < feature.clearance + Self.graspClearanceRadius
    }

    /// Every (grasp, feature) pair where the grasp would block that feature.
    /// Empty means all grasp poses are clear of all features.
    func graspFeatureConflicts() -> [(grasp: Pose, feature: MatingFeature)] {
        var conflicts: [(Pose, MatingFeature)] = []
        for g in graspPoses {
            for f in features where grasp(g, blocks: f) {
                conflicts.append((g, f))
            }
        }
        return conflicts
    }

    /// True when no grasp pose overlaps any of the part's own features.
    var graspPosesAreValid: Bool { graspFeatureConflicts().isEmpty }

    /// Only the grasp poses that leave every feature clear.
    var clearGraspPoses: [Pose] {
        graspPoses.filter { g in !features.contains { grasp(g, blocks: $0) } }
    }
}

// MARK: - Steps and definition

/// One join in the assembly sequence: `movingPart`'s `movingFeature` mates to
/// the already-placed `targetPart`'s `targetFeature`. `mateTransform` is the
/// resulting relative transform of the moving feature frame WITHIN the target
/// feature frame (defaults to the opposed peg-in-hole seating).
public struct AssemblyStep: Identifiable, Sendable {
    public let id: String
    public let movingPartID: String
    public let targetPartID: String
    public let movingFeatureID: String
    public let targetFeatureID: String
    public let mateTransform: Pose
    /// IDs of steps that must be completed before this one may run. This is how
    /// sequencing constraints are encoded — e.g. the body shell lists all four
    /// wheel steps here, because it covers the axles once fitted.
    public let prerequisites: [String]

    public init(
        id: String,
        movingPartID: String, movingFeatureID: String,
        targetPartID: String, targetFeatureID: String,
        mateTransform: Pose = MatingFeature.opposedMateTransform(),
        prerequisites: [String] = []
    ) {
        self.id = id
        self.movingPartID = movingPartID
        self.movingFeatureID = movingFeatureID
        self.targetPartID = targetPartID
        self.targetFeatureID = targetFeatureID
        self.mateTransform = mateTransform
        self.prerequisites = prerequisites
    }
}

/// A full product: its parts, the base part that starts fixed to the table, and
/// the ORDERED steps that build it. Step order is the required sequence (e.g.
/// wheels before the body shell that covers the axles).
public struct AssemblyDefinition: Sendable {
    public let name: String
    public let parts: [AssemblyPart]
    public let basePartID: String
    public let steps: [AssemblyStep]

    public init(name: String, parts: [AssemblyPart], basePartID: String, steps: [AssemblyStep]) {
        self.name = name
        self.parts = parts
        self.basePartID = basePartID
        self.steps = steps
    }

    public func part(id: String) -> AssemblyPart? { parts.first { $0.id == id } }

    public func step(id: String) -> AssemblyStep? { steps.first { $0.id == id } }
}

// MARK: - Sequencing

public extension AssemblyDefinition {
    /// Whether `step` may run given the set of already-completed step IDs: it must
    /// not be done already, and all its prerequisites must be complete.
    func isStepAvailable(_ step: AssemblyStep, completed: Set<String>) -> Bool {
        !completed.contains(step.id) && step.prerequisites.allSatisfy { completed.contains($0) }
    }

    /// Every step that may run right now, in definition order.
    func availableSteps(completed: Set<String>) -> [AssemblyStep] {
        steps.filter { isStepAvailable($0, completed: completed) }
    }

    /// The next step to perform (first available in order), or nil if none remain
    /// available (either all done, or blocked by incomplete prerequisites).
    func nextStep(completed: Set<String>) -> AssemblyStep? {
        availableSteps(completed: completed).first
    }

    /// Whether every step has been completed.
    func isComplete(completed: Set<String>) -> Bool {
        completed.count >= steps.count && steps.allSatisfy { completed.contains($0.id) }
    }
}

// MARK: - Mate pose math

public extension AssemblyDefinition {
    /// The world pose the MOVING part must reach so its feature mates with the
    /// target feature, given the target part's current world pose.
    ///
    /// TRANSFORM CHAIN (every term is a rigid transform mapping LOCAL → PARENT):
    ///
    ///     W_M = W_T · L_Tf · Mate · L_Mf⁻¹
    ///
    ///   W_T   = target part world pose            (target-local  → world)
    ///   L_Tf  = target feature in target part     (targetFeat    → target-local)
    ///   Mate  = moving feature in target feature  (movingFeat    → targetFeat)
    ///   L_Mf  = moving feature in moving part      (movingFeat    → moving-local)
    ///
    /// Read left-to-right: hop from world down into the target part, then to its
    /// feature, then across the mate to where the moving feature must sit, then
    /// back OUT to the moving part's origin via the inverse of the moving
    /// feature's local pose. The result is where to command the grasped part.
    ///
    /// - Returns: `nil` only if the step references an unknown part/feature.
    func mateWorldPose(for step: AssemblyStep, targetPartWorld: Pose) -> Pose? {
        guard
            let movingPart = part(id: step.movingPartID),
            let targetPart = part(id: step.targetPartID),
            let movingFeature = movingPart.feature(id: step.movingFeatureID),
            let targetFeature = targetPart.feature(id: step.targetFeatureID)
        else { return nil }

        let wT = targetPartWorld.matrix
        let lTf = targetFeature.localPose.matrix
        let mate = step.mateTransform.matrix
        let lMfInverse = movingFeature.localPose.matrix.inverse

        return Pose(wT * lTf * mate * lMfInverse)
    }

    /// The world pose of the TARGET feature (the socket) given the target part's
    /// world pose. Phase 3 uses this + the insertion axis to build the straight
    /// line the moving part travels along.
    func targetFeatureWorldPose(for step: AssemblyStep, targetPartWorld: Pose) -> Pose? {
        guard
            let targetPart = part(id: step.targetPartID),
            let targetFeature = targetPart.feature(id: step.targetFeatureID)
        else { return nil }
        return Pose(targetPartWorld.matrix * targetFeature.localPose.matrix)
    }

    /// The PRE-INSERT standoff pose and the final MATE pose for the moving part.
    ///
    /// The pre-insert pose is the mated pose backed off along the socket's
    /// insertion axis by `backoff` meters. The arm reaches pre-insert FIRST, then
    /// travels in a straight Cartesian line into the mate pose — so the part
    /// approaches squarely down the axle instead of arriving from an angle and
    /// sweeping through the target.
    ///
    /// - Returns: `(preInsert, mate)` world poses, or `nil` for an invalid step.
    func insertionPoses(
        for step: AssemblyStep, targetPartWorld: Pose, backoff: Double = 0.05
    ) -> (preInsert: Pose, mate: Pose)? {
        guard
            let mate = mateWorldPose(for: step, targetPartWorld: targetPartWorld),
            let socket = targetFeatureWorldPose(for: step, targetPartWorld: targetPartWorld)
        else { return nil }
        // The socket's +Z points OUT of the hole — the direction to stand off in.
        let approach = simd_normalize(socket.orientation.columns.2)
        let preInsert = Pose(
            position: mate.position + approach * backoff,
            orientation: mate.orientation
        )
        return (preInsert, mate)
    }
}

// MARK: - Validation

public extension AssemblyDefinition {
    /// Structural problems with the definition, as human-readable strings. Empty
    /// means the plan is well-formed. Checks: the base part exists; every step
    /// names known parts and features; each step's feature pair is actually
    /// complementary; and no part's grasp poses collide with its own features.
    func validationIssues() -> [String] {
        var issues: [String] = []

        if part(id: basePartID) == nil {
            issues.append("Base part '\(basePartID)' is not in the parts list.")
        }

        for step in steps {
            guard let movingPart = part(id: step.movingPartID) else {
                issues.append("Step '\(step.id)': unknown moving part '\(step.movingPartID)'.")
                continue
            }
            guard let targetPart = part(id: step.targetPartID) else {
                issues.append("Step '\(step.id)': unknown target part '\(step.targetPartID)'.")
                continue
            }
            guard let movingFeature = movingPart.feature(id: step.movingFeatureID) else {
                issues.append("Step '\(step.id)': part '\(movingPart.id)' has no feature '\(step.movingFeatureID)'.")
                continue
            }
            guard let targetFeature = targetPart.feature(id: step.targetFeatureID) else {
                issues.append("Step '\(step.id)': part '\(targetPart.id)' has no feature '\(step.targetFeatureID)'.")
                continue
            }
            if !movingFeature.kind.canMate(with: targetFeature.kind) {
                issues.append(
                    "Step '\(step.id)': \(movingFeature.kind.rawValue) can't mate with \(targetFeature.kind.rawValue)."
                )
            }
        }

        for p in parts {
            for conflict in p.graspFeatureConflicts() {
                issues.append(
                    "Part '\(p.id)': a grasp pose overlaps feature '\(conflict.feature.id)' — it would block insertion."
                )
            }
        }

        return issues
    }

    /// Convenience: whether the definition passes all validation checks.
    var isValid: Bool { validationIssues().isEmpty }
}
