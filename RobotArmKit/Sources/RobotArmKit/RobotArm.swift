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
    /// The joints, ordered base → end-effector. `joints[0]` is the base joint.
    public let joints: [Joint]

    public init(joints: [Joint]) {
        self.joints = joints
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
        let twoPi = 2 * Double.pi

        // Build each joint from one DH row + the ±2π limit.
        func makeJoint(alpha: Double, a: Double, d: Double) -> Joint {
            Joint(
                dh: DHParameter(alpha: alpha, a: a, d: d),
                minAngle: -twoPi,
                maxAngle: twoPi
            )
        }

        let halfPi = Double.pi / 2

        return RobotArm(joints: [
            makeJoint(alpha:  halfPi, a:  0,        d: 0.089159),
            makeJoint(alpha:  0,      a: -0.425,    d: 0),
            makeJoint(alpha:  0,      a: -0.39225,  d: 0),
            makeJoint(alpha:  halfPi, a:  0,        d: 0.10915),
            makeJoint(alpha: -halfPi, a:  0,        d: 0.09465),
            makeJoint(alpha:  0,      a:  0,        d: 0.0823),
        ])
    }()
}
