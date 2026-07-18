//
//  RobotDynamics.swift
//  RobotArmKit
//
//  The velocity Jacobian, a singularity measure built from it, and a static
//  gravity-torque estimate. All pure math, no UI.
//

import Foundation
import simd

public extension RobotArm {

    // MARK: - Jacobian

    /// The 6×N geometric Jacobian at a given configuration: how a tiny change in
    /// each joint angle moves the end effector. Rows 0–2 are the LINEAR velocity
    /// (how the tip translates), rows 3–5 are the ANGULAR velocity (how the tool
    /// spins). Column i is joint i's contribution.
    ///
    /// For a revolute joint with axis `z` at origin `o`, the tip (at `p`) moves
    /// along `z × (p − o)` and the tool spins about `z`.
    func jacobian(jointAngles: [Double]) -> [[Double]] {
        let frames = forwardKinematics(jointAngles: jointAngles)
        let toolPos = translation(frames.last!)

        // Frame before each joint (its rotation axis + origin live here).
        var prior: [simd_double4x4] = [matrix_identity_double4x4]
        prior.append(contentsOf: frames.dropLast())

        return prior.map { frame in
            let axis = zAxisND(frame)
            let origin = translation(frame)
            let linear = simd_cross(axis, toolPos - origin)
            return [linear.x, linear.y, linear.z, axis.x, axis.y, axis.z]  // one 6-vector column
        }
    }

    // MARK: - Singularity detection

    /// Yoshikawa's **manipulability measure**, `w = √det(J·Jᵀ)`.
    ///
    /// WHY THIS DETECTS SINGULARITIES:
    /// The Jacobian maps joint speeds to tool speeds. At a singularity the arm
    /// loses the ability to move the tool in some direction — geometrically, the
    /// Jacobian's rows become linearly dependent, its "volume" collapses, and
    /// `det(J·Jᵀ)` (hence `w`) drops toward zero. A large `w` means the arm is in
    /// a dexterous, well-conditioned pose; a small `w` means it's near a
    /// singularity where small tool motions can demand huge joint speeds.
    func manipulability(jointAngles: [Double]) -> Double {
        let j = jacobian(jointAngles: jointAngles)   // 6×N, as N columns
        // Build J·Jᵀ (6×6): entry (a,b) = Σ_columns col[a]·col[b].
        var jjt = [[Double]](repeating: [Double](repeating: 0, count: 6), count: 6)
        for col in j {
            for a in 0..<6 {
                for b in 0..<6 {
                    jjt[a][b] += col[a] * col[b]
                }
            }
        }
        return sqrt(max(0, Self.determinant(jjt)))
    }

    /// Whether the pose is near a kinematic singularity. The threshold is tuned
    /// so the fully-extended ("elbow-locked") pose and wrist-aligned poses trip
    /// it, while normal working poses don't.
    func isNearSingularity(jointAngles: [Double]) -> Bool {
        manipulability(jointAngles: jointAngles) < Self.singularityThreshold
    }

    /// Manipulability below this counts as "near singular" (see `isNearSingularity`).
    static let singularityThreshold: Double = 0.015

    // MARK: - Static torque estimation

    /// Estimates the gravity torque each joint's motor must hold at a static pose.
    ///
    /// THE PHYSICS (static approximation, no acceleration/inertia):
    /// Each link's weight, and any grasped payload, pulls straight down. A joint
    /// must resist the twisting those weights produce about its own axis. That
    /// twist (torque) from a weight `W` acting at point `p` about a joint at
    /// origin `o` with axis `ẑ` is `ẑ · ((p − o) × W)`. We sum the contribution of
    /// every mass OUTBOARD of the joint (links farther out plus the payload),
    /// because only those are being held up by that joint. This ignores dynamics
    /// (a moving arm needs more), but captures the key intuition: torque grows
    /// with how far out the mass is — the moment arm.
    ///
    /// - Parameters:
    ///   - jointAngles: current pose (radians).
    ///   - payloadMass: mass held at the tool (kg); 0 if empty-handed.
    ///   - gravity: gravitational acceleration (m/s²).
    /// - Returns: signed torque per joint (N·m). Magnitude is what matters for
    ///   comparing against the motor limit.
    func jointTorques(jointAngles: [Double], payloadMass: Double, gravity: Double = 9.81) -> [Double] {
        let n = joints.count
        let frames = forwardKinematics(jointAngles: jointAngles)

        // Joint origins: index 0 is the base, then each frame's origin.
        var origins: [SIMD3<Double>] = [SIMD3<Double>(0, 0, 0)]
        origins.append(contentsOf: frames.map { translation($0) })

        // Each joint's rotation axis (z of the frame before it).
        var prior: [simd_double4x4] = [matrix_identity_double4x4]
        prior.append(contentsOf: frames.dropLast())
        let axes = prior.map { zAxisND($0) }

        // A "point mass" for each link (placed at the link's midpoint) plus the
        // payload at the tool. Gravity pulls each straight down (−Z in the base
        // frame, which is up).
        struct PointMass { let position: SIMD3<Double>; let weight: SIMD3<Double>; let link: Int }
        var masses: [PointMass] = []
        for k in 0..<n {
            let mass = k < linkMasses.count ? linkMasses[k] : 0
            guard mass > 0 else { continue }
            let com = (origins[k] + origins[k + 1]) / 2
            masses.append(PointMass(position: com, weight: SIMD3<Double>(0, 0, -mass * gravity), link: k))
        }
        if payloadMass > 0 {
            masses.append(PointMass(position: origins[n], weight: SIMD3<Double>(0, 0, -payloadMass * gravity), link: n))
        }

        // For each joint, sum the torque from every mass outboard of it.
        return (0..<n).map { i in
            var torque = 0.0
            for m in masses where m.link >= i {           // outboard of joint i
                let moment = simd_cross(m.position - origins[i], m.weight)
                torque += simd_dot(axes[i], moment)
            }
            return torque
        }
    }

    // MARK: - Private helpers

    private func translation(_ m: simd_double4x4) -> SIMD3<Double> {
        SIMD3<Double>(m.columns.3.x, m.columns.3.y, m.columns.3.z)
    }

    private func zAxisND(_ m: simd_double4x4) -> SIMD3<Double> {
        simd_normalize(SIMD3<Double>(m.columns.2.x, m.columns.2.y, m.columns.2.z))
    }

    /// Determinant of a small dense matrix via Gaussian elimination (product of
    /// the pivots, sign-flipped for each row swap).
    private static func determinant(_ matrix: [[Double]]) -> Double {
        var a = matrix
        let n = a.count
        var det = 1.0
        for pivot in 0..<n {
            var maxRow = pivot
            for r in (pivot + 1)..<n where abs(a[r][pivot]) > abs(a[maxRow][pivot]) {
                maxRow = r
            }
            if abs(a[maxRow][pivot]) < 1e-15 { return 0 }
            if maxRow != pivot { a.swapAt(pivot, maxRow); det = -det }
            det *= a[pivot][pivot]
            for r in (pivot + 1)..<n {
                let factor = a[r][pivot] / a[pivot][pivot]
                for c in pivot..<n { a[r][c] -= factor * a[pivot][c] }
            }
        }
        return det
    }
}
