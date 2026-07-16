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

    // MARK: Approach-direction IK points the tool down

    /// With a downward approach axis requested, the solution's tool frame should
    /// both reach the target AND have its approach axis (+Z) pointing down.
    @Test
    func solvesWithDownwardApproach() {
        let arm = RobotArm.ur5
        // A reachable point in front of and below the shoulder.
        let target = SIMD3<Double>(0.35, 0.1, 0.2)
        let down = SIMD3<Double>(0, 0, -1)   // base −Z is straight down

        let result = arm.inverseKinematics(targetPosition: target, approachAxis: down)
        #expect(result.reached)

        let frames = arm.forwardKinematics(jointAngles: result.jointAngles)
        let tool = frames.last!
        // Position close to target.
        let pos = SIMD3<Double>(tool.columns.3.x, tool.columns.3.y, tool.columns.3.z)
        #expect(simd_length(pos - target) < 5e-3)
        // Approach axis (tool +Z) aligned with down, within ~6°.
        let approach = simd_normalize(SIMD3<Double>(tool.columns.2.x, tool.columns.2.y, tool.columns.2.z))
        #expect(simd_dot(approach, down) > cos(6 * .pi / 180))
    }

    // MARK: Full-orientation IK matches a target frame

    /// With a full target orientation, the solution's tool frame should match it
    /// (all three axes), not just the approach direction.
    @Test
    func solvesFullOrientation() {
        let arm = RobotArm.ur5
        let target = SIMD3<Double>(0.3, 0.15, 0.25)
        // Tool +Z straight down, +X along base +X, +Y = Z × X.
        let x = SIMD3<Double>(1, 0, 0)
        let z = SIMD3<Double>(0, 0, -1)
        let y = simd_cross(z, x)
        let orientation = simd_double3x3(columns: (x, y, z))

        let result = arm.inverseKinematics(targetPosition: target, targetOrientation: orientation)
        #expect(result.reached)

        let tool = arm.forwardKinematics(jointAngles: result.jointAngles).last!
        let colX = simd_normalize(SIMD3<Double>(tool.columns.0.x, tool.columns.0.y, tool.columns.0.z))
        let colZ = simd_normalize(SIMD3<Double>(tool.columns.2.x, tool.columns.2.y, tool.columns.2.z))
        #expect(simd_dot(colX, x) > cos(5 * .pi / 180))   // finger axis aligned
        #expect(simd_dot(colZ, z) > cos(5 * .pi / 180))   // approach axis down
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
