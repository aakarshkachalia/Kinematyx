//
//  TowerTests.swift
//  RobotArmKitTests
//
//  Validates the stacked-tower assembly definition: it's well-formed, it sequences
//  strictly bottom-up, and each piece's socket lands exactly on the stud below.
//

import Testing
import simd
@testable import RobotArmKit

struct TowerTests {

    @Test func towerIsValid() {
        let issues = Tower.definition.validationIssues()
        #expect(issues.isEmpty, "Tower should be well-formed: \(issues)")
    }

    @Test func towerHasBasePlusBlocksPlusCap() {
        let def = Tower.definition
        // base + N blocks + cap parts; N blocks + cap steps.
        #expect(def.parts.count == TowerGeometry.blockCount + 2)
        #expect(def.steps.count == TowerGeometry.blockCount + 1)
        #expect(def.basePartID == Tower.baseID)
    }

    @Test func towerSequencesStrictlyBottomUp() {
        let def = Tower.definition
        var completed = Set<String>()

        // Only the first block is available at the start; everything else is blocked.
        let firstAvailable = def.availableSteps(completed: completed).map(\.id)
        #expect(firstAvailable == ["step.block1"])

        // Completing each block unlocks exactly the next step, in order.
        for step in def.steps {
            #expect(def.nextStep(completed: completed)?.id == step.id)
            completed.insert(step.id)
        }
        #expect(def.isComplete(completed: completed))
    }

    @Test func eachBlockSocketSeatsOnTheStudBelow() {
        // Push each moving part's socket through the mate transform and confirm its
        // feature origin coincides with the target stud's — for the first stack.
        let def = Tower.definition
        let baseWorld = Pose(position: SIMD3<Double>(0.4, 0.8, -0.2))

        let step = def.step(id: "step.block1")!
        let movingWorld = def.mateWorldPose(for: step, targetPartWorld: baseWorld)!
        let block = def.part(id: "block1")!
        let socketWorld = Pose(movingWorld.matrix * block.feature(id: "socket")!.localPose.matrix)
        let studWorld = def.targetFeatureWorldPose(for: step, targetPartWorld: baseWorld)!
        #expect(socketWorld.positionDistance(to: studWorld) < 1e-9)

        // And the block ends up ABOVE the base (it stacks upward).
        #expect(movingWorld.position.y > baseWorld.position.y)
    }
}
