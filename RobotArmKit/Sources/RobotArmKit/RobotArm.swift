//
//  RobotArm.swift
//  RobotArmKit
//
//  A robot arm modeled as an ordered chain of joints, plus forward kinematics:
//  given the angle of every joint, where does each part of the arm end up?

import Foundation
import simd

/// A serial (chain) robot arm: an ordered list of joints from base to tool.
public struct RobotArm: Sendable {
    /// Display name (e.g. "UR5").
    public let name: String
    /// The joints, ordered base → end-effector. `joints[0]` is the base joint.
    public let joints: [Joint]
    /// Mass of each link in kilograms (one per joint), for torque estimation.
    public let linkMasses: [Double]
    /// Published maximum torque of each joint's motor in N·m, for the warning.
    public let jointTorqueLimits: [Double]

    public init(
        name: String = "Arm",
        joints: [Joint],
        linkMasses: [Double] = [],
        jointTorqueLimits: [Double] = []
    ) {
        self.name = name
        self.joints = joints
        self.linkMasses = linkMasses
        self.jointTorqueLimits = jointTorqueLimits
    }

    /// Number of degrees of freedom (one per revolute joint).
    public var degreesOfFreedom: Int { joints.count }

    // MARK: - Limit checking

    /// Whether every angle in `jointAngles` is within its joint's limits.
    ///
    /// - Parameter jointAngles: one angle per joint, in radians, base first.
    /// - Returns: `false` if the count is wrong or any angle is out of range.
    public func isWithinLimits(_ jointAngles: [Double]) -> Bool {
        guard jointAngles.count == joints.count else { return false }
        for (joint, angle) in zip(joints, jointAngles) where !joint.allows(angle) {
            return false
        }
        return true
    }

    // MARK: - Forward kinematics

    /// Computes the transform of EVERY joint frame, expressed in the base frame.
    ///
    /// - Parameter jointAngles: one commanded angle per joint, in radians,
    ///   ordered base → tool.
    /// - Returns: an array of `degreesOfFreedom` matrices. Element `i` is the
    ///   cumulative transform `A₁·A₂·…·A_(i+1)` — i.e. the pose of joint `i`'s
    ///   frame relative to the robot's base. The LAST element is the pose of the
    ///   end-effector (the tool tip).
    ///
    /// WHY RETURN EVERY JOINT, NOT JUST THE TOOL?
    /// ------------------------------------------
    /// Each link of the arm has to be drawn somewhere. To place link `i` in 3D
    /// later, we need to know where joint `i`'s frame sits in space — not just
    /// where the final tool ends up. Returning the full list now means the
    /// rendering code (added later) can position each link without recomputing.
    ///
    /// HOW FORWARD KINEMATICS WORKS
    /// ----------------------------
    /// Each joint contributes one transform `A_i` (from its DH row + angle) that
    /// says "how the next frame sits relative to this one." Chaining them by
    /// multiplication walks us outward from the base:
    ///
    ///     base → joint1 : A₁
    ///     base → joint2 : A₁·A₂
    ///     base → tool   : A₁·A₂·…·A₆
    ///
    /// We accumulate the running product as we loop so each joint's world pose
    /// is just "the previous world pose times this joint's local transform."
    ///
    /// - Precondition: `jointAngles.count == joints.count`.
    public func forwardKinematics(jointAngles: [Double]) -> [simd_double4x4] {
        precondition(
            jointAngles.count == joints.count,
            "forwardKinematics expected \(joints.count) angles, got \(jointAngles.count)"
        )

        // The running product, starting at the identity (the base frame itself).
        var cumulative = matrix_identity_double4x4
        var frames: [simd_double4x4] = []
        frames.reserveCapacity(joints.count)

        for (joint, angle) in zip(joints, jointAngles) {
            // Local transform from this joint's frame to the previous frame.
            let local = joint.dh.transform(theta: angle)
            // Fold it into the running product to get this frame in base space.
            // Order matters: previous-world (cumulative) THEN this local step.
            cumulative = cumulative * local
            frames.append(cumulative)
        }

        return frames
    }

    /// Convenience: the end-effector's XYZ position in the base frame (meters).
    ///
    /// This is the translation column of the final forward-kinematics matrix.
    public func endEffectorPosition(jointAngles: [Double]) -> simd_double3 {
        let frames = forwardKinematics(jointAngles: jointAngles)
        guard let tool = frames.last else { return .zero }
        // Column 3 of a homogeneous transform holds its (x, y, z, 1) translation.
        return simd_double3(tool.columns.3.x, tool.columns.3.y, tool.columns.3.z)
    }
}

// MARK: - The Universal Robots UR5

public extension RobotArm {
    /// The default arm: a Universal Robots **UR5**, using real published DH
    /// parameters (standard/distal DH convention).
    ///
    /// SOURCE OF THESE NUMBERS
    /// -----------------------
    /// Universal Robots publishes the official DH parameters for their arms at:
    ///   https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
    ///
    /// The UR5 values from that page are:
    ///
    ///   Joint |   a (m)   |   d (m)   |  α (rad)
    ///   ------+-----------+-----------+---------
    ///     1   |  0        | 0.089159  |  +π/2
    ///     2   | -0.425    | 0         |   0
    ///     3   | -0.39225  | 0         |   0
    ///     4   |  0        | 0.10915   |  +π/2
    ///     5   |  0        | 0.09465   |  -π/2
    ///     6   |  0        | 0.0823    |   0
    ///
    /// All `thetaOffset`s are 0: with every θ = 0 the arm is in the fully
    /// extended configuration used as the reference pose on that page.
    ///
    /// JOINT LIMITS
    /// ------------
    /// Every UR5 joint can rotate ±360° (±2π radians), so we use ±2π for each.
    static let ur5: RobotArm = {
        let halfPi = Double.pi / 2
        return RobotArm(
            name: "UR5",
            joints: [
                makeRevolute(alpha:  halfPi, a:  0,        d: 0.089159),
                makeRevolute(alpha:  0,      a: -0.425,    d: 0),
                makeRevolute(alpha:  0,      a: -0.39225,  d: 0),
                makeRevolute(alpha:  halfPi, a:  0,        d: 0.10915),
                makeRevolute(alpha: -halfPi, a:  0,        d: 0.09465),
                makeRevolute(alpha:  0,      a:  0,        d: 0.0823),
            ],
            // Published UR5 link masses (kg).
            linkMasses: [3.7, 8.393, 2.275, 1.219, 1.219, 0.1879],
            // Published UR5 joint torque limits (N·m): 150 for the three big
            // base joints, 28 for the three wrist joints.
            jointTorqueLimits: [150, 150, 150, 28, 28, 28]
        )
    }()

    /// A Universal Robots **UR3**: same family, smaller reach (~0.5 m). DH
    /// parameters from Universal Robots' published table (see `ur5` for the URL).
    ///
    ///   Joint |   a (m)    |   d (m)   |  α (rad)
    ///   ------+------------+-----------+---------
    ///     1   |  0         | 0.1519    |  +π/2
    ///     2   | -0.24365   | 0         |   0
    ///     3   | -0.21325   | 0         |   0
    ///     4   |  0         | 0.11235   |  +π/2
    ///     5   |  0         | 0.08535   |  -π/2
    ///     6   |  0         | 0.0819    |   0
    static let ur3: RobotArm = {
        let halfPi = Double.pi / 2
        return RobotArm(
            name: "UR3",
            joints: [
                makeRevolute(alpha:  halfPi, a:  0,        d: 0.1519),
                makeRevolute(alpha:  0,      a: -0.24365,  d: 0),
                makeRevolute(alpha:  0,      a: -0.21325,  d: 0),
                makeRevolute(alpha:  halfPi, a:  0,        d: 0.11235),
                makeRevolute(alpha: -halfPi, a:  0,        d: 0.08535),
                makeRevolute(alpha:  0,      a:  0,        d: 0.0819),
            ],
            // Approximate published UR3 link masses (kg).
            linkMasses: [2.0, 3.42, 1.26, 0.8, 0.8, 0.35],
            // Approximate UR3 joint torque limits (N·m): 56 for base joints,
            // 12 for the wrist joints.
            jointTorqueLimits: [56, 56, 28, 12, 12, 12]
        )
    }()

    /// A revolute joint with the standard UR ±360° limit.
    private static func makeRevolute(alpha: Double, a: Double, d: Double) -> Joint {
        let twoPi = 2 * Double.pi
        return Joint(dh: DHParameter(alpha: alpha, a: a, d: d), minAngle: -twoPi, maxAngle: twoPi)
    }

    /// All arm profiles available for selection, in menu order.
    static let allProfiles: [RobotArm] = [.ur5, .ur3]
}
