//
//  InverseKinematicsTests.swift
//  RobotArmKitTests
//
//  Verifies the IK solver by the "round trip" property: pick joint angles, run
//  FORWARD kinematics to get a definitely-reachable target, then ask IK to solve
//  for that target and confirm the solution — run back through FK — lands on it.
//  This avoids hand-deriving IK answers (there are many valid ones) while still
//  proving the solver is correct.
//

import Testing
import simd
@testable import RobotArmKit

struct InverseKinematicsTests {

    /// Helper: the FK tool position for a set of angles.
    private func toolPosition(_ arm: RobotArm, _ angles: [Double]) -> SIMD3<Double> {
        let frames = arm.forwardKinematics(jointAngles: angles)
        let p = frames.last!.columns.3
        return SIMD3<Double>(p.x, p.y, p.z)
    }

    // MARK: Reachable target #1

    @Test
    func solvesReachableTargetA() {
        let arm = RobotArm.ur5
        // A clearly reachable pose (moderate bends).
        let truthAngles = [0.3, -0.7, 0.9, -0.2, 0.4, 0.0]
        let target = toolPosition(arm, truthAngles)

        let result = arm.inverseKinematics(targetPosition: target)

        #expect(result.reached)
        // The solution's OWN forward kinematics should hit the target.
        let achieved = toolPosition(arm, result.jointAngles)
        #expect(simd_length(achieved - target) < 1e-3)
    }

    // MARK: Reachable target #2

    @Test
    func solvesReachableTargetB() {
        let arm = RobotArm.ur5
        let truthAngles = [-0.8, -0.4, 0.6, 0.5, -0.3, 0.1]
        let target = toolPosition(arm, truthAngles)

        let result = arm.inverseKinematics(targetPosition: target)

        #expect(result.reached)
        let achieved = toolPosition(arm, result.jointAngles)
        #expect(simd_length(achieved - target) < 1e-3)
    }

    // MARK: Solutions respect joint limits

    @Test
    func solutionStaysWithinLimits() {
        let arm = RobotArm.ur5
        let target = toolPosition(arm, [0.5, -0.5, 0.5, 0.0, 0.3, 0.0])
        let result = arm.inverseKinematics(targetPosition: target)
        #expect(arm.isWithinLimits(result.jointAngles))
    }

    // MARK: Unreachable target fails gracefully

    @Test
    func unreachableTargetReportsFailureWithoutNaNs() {
        let arm = RobotArm.ur5
        // Far outside the UR5's ~0.85 m reach.
        let target = SIMD3<Double>(10, 10, 10)

        let result = arm.inverseKinematics(targetPosition: target)

        #expect(!result.reached)                    // honestly reports failure
        #expect(result.positionError > 1.0)         // still far away
        // No NaNs, and still a usable (closest) configuration within limits.
        for angle in result.jointAngles {
            #expect(angle.isFinite)
        }
        #expect(arm.isWithinLimits(result.jointAngles))
    }
}
