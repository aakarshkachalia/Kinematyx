//
//  Pose.swift
//  RobotArmKit
//
//  A rigid-body pose: a position plus an orientation. This is the currency of
//  full 6-DOF inverse kinematics (Phase 1) and of the assembly graph (Phase 2):
//  a wheel isn't just placed at a POINT, it's placed at a POINT pointing a
//  particular WAY.
//

import Foundation
import simd

/// A full 6-DOF pose in some reference frame: where something is AND which way
/// it points.
///
/// Orientation is stored as a 3×3 ROTATION MATRIX whose columns are the local
/// X, Y, Z axes expressed in the reference frame. We deliberately avoid Euler
/// angles here: near certain orientations two Euler axes line up ("gimbal
/// lock") and the angle triple becomes ambiguous and discontinuous, which
/// wrecks an iterative solver and makes error math jumpy. A rotation matrix
/// (equivalently a quaternion) has no such coordinate singularity.
public struct Pose: Sendable, Equatable {
    /// Position in the reference frame (meters).
    public var position: SIMD3<Double>
    /// Orientation as a rotation matrix (columns = local axes in the ref frame).
    public var orientation: simd_double3x3

    public init(
        position: SIMD3<Double>,
        orientation: simd_double3x3 = matrix_identity_double3x3
    ) {
        self.position = position
        self.orientation = orientation
    }

    /// Builds a pose from a homogeneous 4×4 transform (rotation + translation).
    public init(_ m: simd_double4x4) {
        position = SIMD3<Double>(m.columns.3.x, m.columns.3.y, m.columns.3.z)
        orientation = simd_double3x3(columns: (
            SIMD3<Double>(m.columns.0.x, m.columns.0.y, m.columns.0.z),
            SIMD3<Double>(m.columns.1.x, m.columns.1.y, m.columns.1.z),
            SIMD3<Double>(m.columns.2.x, m.columns.2.y, m.columns.2.z)
        ))
    }

    /// The equivalent homogeneous 4×4 transform.
    public var matrix: simd_double4x4 {
        let r = orientation
        return simd_double4x4(columns: (
            SIMD4<Double>(r.columns.0.x, r.columns.0.y, r.columns.0.z, 0),
            SIMD4<Double>(r.columns.1.x, r.columns.1.y, r.columns.1.z, 0),
            SIMD4<Double>(r.columns.2.x, r.columns.2.y, r.columns.2.z, 0),
            SIMD4<Double>(position.x, position.y, position.z, 1)
        ))
    }

    public static func == (lhs: Pose, rhs: Pose) -> Bool {
        lhs.position == rhs.position && lhs.orientation == rhs.orientation
    }
}

public extension RobotArm {
    /// The end-effector's full pose (position + orientation) in the base frame —
    /// the 6-DOF counterpart of `endEffectorPosition`.
    func endEffectorPose(jointAngles: [Double]) -> Pose {
        Pose(forwardKinematics(jointAngles: jointAngles).last ?? matrix_identity_double4x4)
    }
}

// MARK: - Pose interpolation and error (Cartesian motion + snap checks)

public extension Pose {
    /// CARTESIAN (straight-line) interpolation toward `other`: position is
    /// linearly interpolated and orientation is SLERPed (shortest-arc blend of
    /// the two rotations). `t = 0` → self, `t = 1` → `other`.
    ///
    /// WHY THIS MATTERS (Cartesian vs joint-space interpolation):
    /// If you instead interpolate the six JOINT ANGLES linearly between the start
    /// and end poses, each joint moves at a constant rate — but because the tool's
    /// position is a nonlinear function of the angles, the tool traces a CURVED
    /// arc through space. For assembly that arc can sweep the held part sideways
    /// THROUGH the target before arriving. Interpolating the tool POSE here and
    /// solving IK at each sample instead keeps the tool on a straight line, so a
    /// peg travels dead straight down its insertion axis into the hole.
    func interpolated(to other: Pose, t: Double) -> Pose {
        let p = position + (other.position - position) * t
        let qA = simd_quatd(orientation)
        let qB = simd_quatd(other.orientation)
        let q = simd_slerp(qA, qB, t)
        return Pose(position: p, orientation: simd_double3x3(q))
    }

    /// Positional distance (meters) to another pose.
    func positionDistance(to other: Pose) -> Double {
        simd_length(other.position - position)
    }

    /// Smallest rotation angle (radians) between this orientation and another's,
    /// via the angle of the relative rotation `Rᵀ·R'` (`acos((trace−1)/2)`). This
    /// is exact for any angle, unlike the small-angle rotation-vector magnitude.
    func angularDistance(to other: Pose) -> Double {
        let r = orientation.transpose * other.orientation
        let trace = r.columns.0.x + r.columns.1.y + r.columns.2.z
        let cosAngle = min(1, max(-1, (trace - 1) / 2))
        return acos(cosAngle)
    }
}
