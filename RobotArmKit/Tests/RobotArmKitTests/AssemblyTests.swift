//
//  AssemblyTests.swift
//  RobotArmKitTests
//
//  Exercises the assembly graph's pure transform math and plan validation.
//

import Testing
import simd
@testable import RobotArmKit

struct AssemblyTests {

    // MARK: Helpers

    /// A rotation matrix for `angle` radians about a unit `axis`.
    private func rot(_ angle: Double, _ axis: SIMD3<Double>) -> simd_double3x3 {
        simd_double3x3(simd_quatd(angle: angle, axis: simd_normalize(axis)))
    }

    /// Max absolute element-wise difference between two 4×4 matrices.
    private func maxDiff(_ a: simd_double4x4, _ b: simd_double4x4) -> Double {
        var m = 0.0
        for c in 0..<4 {
            let d = a[c] - b[c]
            m = max(m, abs(d.x), abs(d.y), abs(d.z), abs(d.w))
        }
        return m
    }

    /// A minimal two-part definition: a moving part with a peg, a target part
    /// with a hole, joined by one default (opposed) step.
    private func pegHoleDefinition() -> AssemblyDefinition {
        let moving = AssemblyPart(
            id: "m", name: "Mover", mass: 0.1,
            graspPoses: [Pose(position: SIMD3<Double>(0, 0.08, 0))],   // off to the side, clear
            features: [MatingFeature(
                id: "peg", kind: .peg,
                localPose: Pose(position: SIMD3<Double>(0, 0, 0.05)),  // peg tip 5cm out along +Z
                clearance: 0.01
            )]
        )
        let target = AssemblyPart(
            id: "t", name: "Target", mass: 1.0,
            graspPoses: [],
            features: [MatingFeature(
                id: "hole", kind: .hole,
                localPose: Pose(position: SIMD3<Double>(0.1, 0, 0)),   // hole offset in the part
                clearance: 0.01
            )]
        )
        let step = AssemblyStep(
            id: "s1",
            movingPartID: "m", movingFeatureID: "peg",
            targetPartID: "t", targetFeatureID: "hole"
        )
        return AssemblyDefinition(name: "Test", parts: [moving, target], basePartID: "t", steps: [step])
    }

    // MARK: Complementarity

    @Test func complementaryKindsMate() {
        #expect(MatingKind.peg.canMate(with: .hole))
        #expect(MatingKind.hole.canMate(with: .peg))
        #expect(MatingKind.stud.canMate(with: .socket))
        #expect(!MatingKind.peg.canMate(with: .socket))
        #expect(!MatingKind.peg.canMate(with: .peg))
    }

    // MARK: Mate-pose transform math (the key property)

    /// After placing the moving part at the computed world pose, its feature
    /// frame must coincide with the target feature frame composed with the mate
    /// transform — for an arbitrary (translated AND rotated) target world pose.
    @Test func mateWorldPosePlacesFeaturesTogether() {
        let def = pegHoleDefinition()
        let step = def.steps[0]

        let targetWorld = Pose(
            position: SIMD3<Double>(0.3, 0.2, 0.1),
            orientation: rot(.pi / 6, SIMD3<Double>(0.2, 1, 0.3))
        )

        let movingWorld = def.mateWorldPose(for: step, targetPartWorld: targetWorld)
        #expect(movingWorld != nil)

        let movingPart = def.part(id: "m")!
        let targetPart = def.part(id: "t")!
        let lMf = movingPart.feature(id: "peg")!.localPose.matrix
        let lTf = targetPart.feature(id: "hole")!.localPose.matrix

        // Where the moving feature actually ends up in the world.
        let movingFeatureWorld = movingWorld!.matrix * lMf
        // Where it SHOULD be: target feature world, then the mate transform.
        let expected = (targetWorld.matrix * lTf) * step.mateTransform.matrix

        #expect(maxDiff(movingFeatureWorld, expected) < 1e-9)
    }

    /// The seated peg's insertion axis must point OPPOSITE the hole's (it points
    /// into the hole), which is what the default mate transform encodes.
    @Test func seatedInsertionAxesAreOpposed() {
        let def = pegHoleDefinition()
        let step = def.steps[0]
        let targetWorld = Pose(position: SIMD3<Double>(0.3, 0, 0.1))

        let movingWorld = def.mateWorldPose(for: step, targetPartWorld: targetWorld)!
        let targetFeatWorld = def.targetFeatureWorldPose(for: step, targetPartWorld: targetWorld)!

        let movingPart = def.part(id: "m")!
        let pegWorld = Pose(movingWorld.matrix * movingPart.feature(id: "peg")!.localPose.matrix)

        let pegAxis = simd_normalize(pegWorld.orientation.columns.2)
        let holeAxis = simd_normalize(targetFeatWorld.orientation.columns.2)
        #expect(simd_dot(pegAxis, holeAxis) < -0.999)   // antiparallel

        // And the feature ORIGINS coincide (peg seated at the hole mouth).
        #expect(simd_length(pegWorld.position - targetFeatWorld.position) < 1e-9)
    }

    // MARK: Grasp-vs-feature validation

    @Test func graspAcrossFeatureIsFlagged() {
        // A wheel whose hub hole is at the center; grabbing at the center blocks it.
        let badGrasp = Pose(position: SIMD3<Double>(0, 0, 0))
        let goodGrasp = Pose(position: SIMD3<Double>(0.06, 0, 0))   // out at the rim
        let wheel = AssemblyPart(
            id: "w", name: "Wheel", mass: 0.05,
            graspPoses: [badGrasp, goodGrasp],
            features: [MatingFeature(
                id: "hub", kind: .hole,
                localPose: Pose(position: SIMD3<Double>(0, 0, 0)),
                clearance: 0.02
            )]
        )
        #expect(!wheel.graspPosesAreValid)
        #expect(wheel.graspFeatureConflicts().count == 1)
        #expect(wheel.clearGraspPoses.count == 1)
        #expect(wheel.clearGraspPoses.first?.position == goodGrasp.position)
    }

    // MARK: Definition validation

    @Test func wellFormedDefinitionHasNoIssues() {
        #expect(pegHoleDefinition().isValid)
    }

    // MARK: Pre-insert waypoint

    @Test func preInsertStandsOffAlongInsertionAxis() {
        let def = pegHoleDefinition()
        let step = def.steps[0]
        let targetWorld = Pose(
            position: SIMD3<Double>(0.3, 0.2, 0.1),
            orientation: rot(.pi / 5, SIMD3<Double>(0, 1, 0.4))
        )
        let backoff = 0.05
        let poses = def.insertionPoses(for: step, targetPartWorld: targetWorld, backoff: backoff)
        #expect(poses != nil)
        let (preInsert, mate) = poses!

        // Pre-insert is exactly `backoff` from the mate pose...
        #expect(abs(preInsert.positionDistance(to: mate) - backoff) < 1e-9)
        // ...along the socket's insertion axis (+Z of the target feature world)...
        let socket = def.targetFeatureWorldPose(for: step, targetPartWorld: targetWorld)!
        let approach = simd_normalize(socket.orientation.columns.2)
        let offset = preInsert.position - mate.position
        #expect(simd_length(offset - approach * backoff) < 1e-9)
        // ...and with the SAME orientation as the mate pose (pure translation).
        // (acos near 1 is numerically noisy, so allow a µrad of slack.)
        #expect(preInsert.angularDistance(to: mate) < 1e-6)
    }

    // MARK: Cartesian interpolation

    @Test func interpolationHitsEndpointsAndMidpoint() {
        let a = Pose(position: SIMD3<Double>(0, 0, 0), orientation: rot(0, SIMD3<Double>(0, 0, 1)))
        let b = Pose(position: SIMD3<Double>(1, 2, 3), orientation: rot(.pi / 2, SIMD3<Double>(0, 0, 1)))

        #expect(a.interpolated(to: b, t: 0).positionDistance(to: a) < 1e-12)
        #expect(a.interpolated(to: b, t: 1).positionDistance(to: b) < 1e-12)

        let mid = a.interpolated(to: b, t: 0.5)
        #expect(simd_length(mid.position - SIMD3<Double>(0.5, 1, 1.5)) < 1e-12)
        // Halfway through a 90° turn is 45°.
        #expect(abs(mid.angularDistance(to: a) - .pi / 4) < 1e-9)
    }

    @Test func mismatchedFeatureKindsAreReported() {
        // Point the step at a hole→hole pairing, which cannot mate.
        let moving = AssemblyPart(
            id: "m", name: "M", mass: 0.1, graspPoses: [],
            features: [MatingFeature(id: "h1", kind: .hole, localPose: Pose(position: .zero), clearance: 0.01)]
        )
        let target = AssemblyPart(
            id: "t", name: "T", mass: 1, graspPoses: [],
            features: [MatingFeature(id: "h2", kind: .hole, localPose: Pose(position: .zero), clearance: 0.01)]
        )
        let step = AssemblyStep(
            id: "s", movingPartID: "m", movingFeatureID: "h1",
            targetPartID: "t", targetFeatureID: "h2"
        )
        let def = AssemblyDefinition(name: "Bad", parts: [moving, target], basePartID: "t", steps: [step])
        #expect(!def.isValid)
        #expect(def.validationIssues().contains { $0.contains("can't mate") })
    }

    // MARK: Model car

    @Test func modelCarIsValid() {
        let issues = ModelCar.definition.validationIssues()
        #expect(issues.isEmpty, "Model car should be well-formed: \(issues)")
    }

    @Test func modelCarSequencesWheelsBeforeBody() {
        let def = ModelCar.definition
        var completed = Set<String>()

        // With nothing done, the four wheel steps are available but the body is not.
        let firstAvailable = def.availableSteps(completed: completed).map(\.id)
        #expect(firstAvailable.contains("step.wheelFL"))
        #expect(!firstAvailable.contains("step.body"))

        // Completing three wheels still leaves the body blocked.
        completed = ["step.wheelFL", "step.wheelFR", "step.wheelRL"]
        #expect(!def.isStepAvailable(def.step(id: "step.body")!, completed: completed))

        // All four wheels done → the body unlocks.
        completed.insert("step.wheelRR")
        #expect(def.isStepAvailable(def.step(id: "step.body")!, completed: completed))
        #expect(def.nextStep(completed: completed)?.id == "step.body")

        // And with the body done, the build is complete.
        completed.insert("step.body")
        #expect(def.isComplete(completed: completed))
    }

    @Test func modelCarWheelSeatsOnAxle() {
        // Sanity: computing the wheel's mate pose and pushing its hub through its
        // local pose lands exactly on the chassis peg (feature origins coincide).
        let def = ModelCar.definition
        let step = def.step(id: "step.wheelFL")!
        let chassisWorld = Pose(position: SIMD3<Double>(0.4, 0.75, 0.1))
        let wheelWorld = def.mateWorldPose(for: step, targetPartWorld: chassisWorld)!
        let wheel = def.part(id: "wheelFL")!
        let hubWorld = Pose(wheelWorld.matrix * wheel.feature(id: "hub")!.localPose.matrix)
        let pegWorld = def.targetFeatureWorldPose(for: step, targetPartWorld: chassisWorld)!
        #expect(hubWorld.positionDistance(to: pegWorld) < 1e-9)
    }
}
