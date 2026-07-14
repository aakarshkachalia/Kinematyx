//
//  DynamicsTests.swift
//  RobotArmKitTests
//
//  Sanity checks for the Jacobian-based singularity measure and the static
//  torque estimate. These assert relationships (which pose is more singular /
//  higher torque) rather than magic numbers, so they stay robust.
//

import Testing
import simd
@testable import RobotArmKit

struct DynamicsTests {

    // MARK: Singularity

    @Test
    func extendedPoseIsMoreSingularThanBentPose() {
        let arm = RobotArm.ur5
        // All zeros = arm fully stretched out (elbow "locked") → near singular.
        let extended = [Double](repeating: 0, count: 6)
        // A comfortably bent pose → well-conditioned.
        let bent = [0.2, -0.9, 1.2, -0.4, 0.6, 0.0]

        let wExtended = arm.manipulability(jointAngles: extended)
        let wBent = arm.manipulability(jointAngles: bent)

        // Print so the threshold constant can be tuned to sit between them.
        print("manipulability extended=\(wExtended)  bent=\(wBent)  threshold=\(RobotArm.singularityThreshold)")

        #expect(wExtended < wBent)
        #expect(arm.isNearSingularity(jointAngles: extended))
        #expect(!arm.isNearSingularity(jointAngles: bent))
    }

    // MARK: Torque

    @Test
    func torqueIsHigherWhenArmIsExtended() {
        let arm = RobotArm.ur5
        // Shoulder out horizontal = long moment arm on the shoulder joint.
        let extended = [0.0, -Double.pi / 2, 0.0, 0.0, 0.0, 0.0]
        // Arm folded up = short moment arm.
        let folded = [0.0, -0.1, 2.6, 0.0, 0.0, 0.0]

        let tExtended = arm.jointTorques(jointAngles: extended, payloadMass: 1.0)
        let tFolded = arm.jointTorques(jointAngles: folded, payloadMass: 1.0)

        // Shoulder joint (index 1) should bear more torque extended than folded.
        #expect(abs(tExtended[1]) > abs(tFolded[1]))
        // Adding a payload increases shoulder torque.
        let tNoPayload = arm.jointTorques(jointAngles: extended, payloadMass: 0)
        #expect(abs(tExtended[1]) > abs(tNoPayload[1]))
    }

    @Test
    func torqueCountMatchesJoints() {
        let arm = RobotArm.ur5
        #expect(arm.jointTorques(jointAngles: [Double](repeating: 0, count: 6), payloadMass: 0).count == 6)
    }
}
