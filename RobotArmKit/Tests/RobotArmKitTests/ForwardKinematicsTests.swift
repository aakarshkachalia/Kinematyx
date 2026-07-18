//
//  ForwardKinematicsTests.swift
//  RobotArmKitTests
//
//  Verifies RobotArm.forwardKinematics against poses we can work out BY HAND,
//  so the math is checkable independently of any code. Every expected number
//  below is derived in the comments — follow along with a UR5 datasheet.

import Testing
import simd
@testable import RobotArmKit

struct ForwardKinematicsTests {

    /// How close two floating-point numbers must be to count as "equal".
    /// FK involves lots of sin/cos, so we allow a tiny numerical slack.
    let tolerance = 1e-6

    // MARK: - Case 1: all joints at zero

    /// With every joint angle = 0, the UR5 is fully extended along its axes and
    /// the end-effector position collapses to simple sums of the DH offsets.
    ///
    /// DERIVATION (standard DH, UR5 numbers)
    /// -------------------------------------
    /// At θ = 0 each joint's transform A_i simplifies (cosθ = 1, sinθ = 0) to:
    ///
    ///   [ 1     0       0      a ]
    ///   [ 0   cosα   -sinα     0 ]
    ///   [ 0   sinα    cosα     d ]
    ///   [ 0     0       0      1 ]
    ///
    /// Multiplying A₁·A₂·…·A₆ and tracking only the translation, the alternating
    /// ±90° twists route each offset onto a specific base axis. Working it out:
    ///
    ///   x = a₂ + a₃            = -0.425 - 0.39225      = -0.81725
    ///   y = -(d₄ + d₆)         = -(0.10915 + 0.0823)   = -0.19145
    ///   z =  d₁ - d₅           =  0.089159 - 0.09465   = -0.005491
    ///
    /// (The d₄ and d₆ offsets get rotated onto the base's -Y axis by the α=+π/2
    /// twist at joint 4; d₅ lands on -Z via the α=-π/2 twist at joint 5.)
    @Test
    func allJointsZero() {
        let arm = RobotArm.ur5
        let angles = [Double](repeating: 0, count: 6)

        let position = arm.endEffectorPosition(jointAngles: angles)

        #expect(abs(position.x - (-0.81725))  < tolerance)
        #expect(abs(position.y - (-0.19145))  < tolerance)
        #expect(abs(position.z - (-0.005491)) < tolerance)
    }

    // MARK: - Case 2: base joint rotated 90°

    /// Rotating ONLY joint 1 (the base) by +90° spins the entire arm about the
    /// base Z axis. Since joint 1's rotation Rz(θ₁) is the very first factor in
    /// the product A₁·A₂·…·A₆, the whole result is just the zero-pose position
    /// pre-multiplied by Rz(π/2).
    ///
    /// A +90° rotation about Z maps (x, y, z) → (-y, x, z). Applying that to the
    /// zero-pose position p₀ = (-0.81725, -0.19145, -0.005491):
    ///
    ///   x' = -y₀ = -(-0.19145) =  0.19145
    ///   y' =  x₀ =              = -0.81725
    ///   z' =  z₀ =              = -0.005491
    @Test
    func baseRotatedNinetyDegrees() {
        let arm = RobotArm.ur5
        var angles = [Double](repeating: 0, count: 6)
        angles[0] = Double.pi / 2   // +90° on the base joint only

        let position = arm.endEffectorPosition(jointAngles: angles)

        #expect(abs(position.x - 0.19145)     < tolerance)
        #expect(abs(position.y - (-0.81725))  < tolerance)
        #expect(abs(position.z - (-0.005491)) < tolerance)
    }

    // MARK: - Forward kinematics returns every joint frame

    /// FK should return one transform per joint (base → each frame), not just
    /// the final tool pose, because rendering will need each link's frame.
    @Test
    func returnsOneFramePerJoint() {
        let arm = RobotArm.ur5
        let frames = arm.forwardKinematics(jointAngles: [Double](repeating: 0, count: 6))
        #expect(frames.count == arm.degreesOfFreedom)
    }

    // MARK: - Joint-limit checks

    @Test
    func anglesWithinLimitsPass() {
        let arm = RobotArm.ur5
        // Well inside the ±2π range of every UR5 joint.
        let angles = [0.0, 0.5, -0.5, 1.0, -1.0, 0.2]
        #expect(arm.isWithinLimits(angles))
    }

    @Test
    func angleBeyondLimitFails() {
        let arm = RobotArm.ur5
        var angles = [Double](repeating: 0, count: 6)
        angles[2] = 3 * Double.pi   // 3π > 2π → out of range
        #expect(!arm.isWithinLimits(angles))
    }

    @Test
    func wrongAngleCountFails() {
        let arm = RobotArm.ur5
        #expect(!arm.isWithinLimits([0, 0, 0]))   // too few angles
    }
}
