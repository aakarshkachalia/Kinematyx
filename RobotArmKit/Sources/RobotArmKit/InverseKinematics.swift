//
//  InverseKinematics.swift
//  RobotArmKit
//
//  Inverse kinematics (IK): the reverse of forward kinematics. Forward asks
//  "given the joint angles, where is the tool?" — IK asks "given where I want
//  the tool (and optionally which way it should point), what joint angles get it
//  there?"
//
//  WHY AN ITERATIVE SOLVER (and which one)
//  ---------------------------------------
//  A 6-joint arm generally has no simple formula for the joint angles. We use
//  DAMPED LEAST SQUARES (Levenberg–Marquardt): repeatedly nudge the joints to
//  shrink the error, with a small damping term λ² that keeps the math stable
//  near "singularities" (where the arm briefly loses a degree of freedom) so it
//  never flails or returns NaNs.
//
//  Two modes:
//   * position only — 3 error terms (x, y, z).
//   * position + approach direction — 6 error terms (position + which way the
//     tool's approach axis points). This is what lets the gripper come straight
//     down onto objects from above.
//

import Foundation
import simd

/// The outcome of an IK solve.
public struct IKResult: Sendable {
    /// The best joint angles found (radians), always within the joint limits.
    public let jointAngles: [Double]
    /// Whether the solver got within tolerance of the target.
    public let reached: Bool
    /// Final distance from the achieved tool position to the target (meters).
    public let positionError: Double
}

public extension RobotArm {

    /// Solves for joint angles that place the end effector at `targetPosition`,
    /// and — if `approachAxis` is given — also orient the tool's approach axis
    /// (its local +Z, the direction the gripper "points") along that base-frame
    /// direction. Pass `[0, 0, -1]` for a straight-down, from-above approach.
    ///
    /// - Returns: an `IKResult`. Never throws or returns NaNs; unreachable
    ///   targets yield the closest solution with `reached == false`.
    func inverseKinematics(
        targetPosition: SIMD3<Double>,
        approachAxis: SIMD3<Double>? = nil,
        targetOrientation: simd_double3x3? = nil,
        initialGuess: [Double]? = nil,
        maxIterations: Int = 400,
        tolerance: Double = 1e-3
    ) -> IKResult {
        let n = joints.count
        var theta = clampToLimits(initialGuess ?? defaultIKSeed(count: n))

        let lambdaSq = 0.06 * 0.06
        // Weight on the orientation error so it's comparable to the position
        // error (which is in meters, usually small). Too high and position
        // suffers; too low and the tool won't point where we asked.
        let orientationWeight = 0.5
        let desiredApproach = approachAxis.map { simd_normalize($0) }
        // Any orientation target (a full frame, or just the approach axis) adds
        // the 3 angular rows to the Jacobian.
        let useOrientation = (targetOrientation != nil) || (desiredApproach != nil)

        var bestAngles = theta
        var bestError = Double.greatestFiniteMagnitude

        for _ in 0..<maxIterations {
            let frames = forwardKinematics(jointAngles: theta)
            let toolPos = position(of: frames.last!)
            let posError = targetPosition - toolPos
            let distance = simd_length(posError)

            // Orientation error, as a base-frame rotation vector (axis · sin θ).
            var rotError = SIMD3<Double>(0, 0, 0)
            var orientationAligned = true
            if let target = targetOrientation {
                // FULL orientation: the small rotation from the current tool
                // frame to the target. R_err = R_target · R_currentᵀ; the vee of
                // its skew-symmetric part is that rotation vector. This pins all
                // three rotational DOF — including the wrist ROLL, so the gripper
                // fingers line up with the object instead of hitting a corner.
                let current = rotation(of: frames.last!)
                rotError = Self.rotationVector(target * current.transpose)
                orientationAligned = simd_length(rotError) < sin(4 * .pi / 180)
            } else if let desired = desiredApproach {
                // APPROACH ONLY: align the tool's +Z with `desired`; roll is free.
                let current = zAxis(of: frames.last!)
                rotError = simd_cross(current, desired)
                orientationAligned = simd_dot(current, desired) > cos(6 * .pi / 180)
            }

            if distance < bestError {
                bestError = distance
                bestAngles = theta
            }
            if distance < tolerance && orientationAligned {
                return IKResult(jointAngles: theta, reached: true, positionError: distance)
            }

            // Build the Jacobian columns. Each joint contributes a 6-vector:
            //   linear part  Jv = axis × (toolPos − jointOrigin)   (how the tip moves)
            //   angular part Jw = axis                              (how the tool spins)
            var priorFrames: [simd_double4x4] = [matrix_identity_double4x4]
            priorFrames.append(contentsOf: frames.dropLast())

            var columns: [[Double]] = []
            columns.reserveCapacity(n)
            for frame in priorFrames {
                let axis = zAxis(of: frame)
                let origin = position(of: frame)
                let jv = simd_cross(axis, toolPos - origin)
                if useOrientation {
                    let jw = axis * orientationWeight
                    columns.append([jv.x, jv.y, jv.z, jw.x, jw.y, jw.z])
                } else {
                    columns.append([jv.x, jv.y, jv.z])
                }
            }

            // Stacked error vector (weighted to match the Jacobian's angular rows).
            let error: [Double] = useOrientation
                ? [posError.x, posError.y, posError.z,
                   rotError.x * orientationWeight, rotError.y * orientationWeight, rotError.z * orientationWeight]
                : [posError.x, posError.y, posError.z]

            // Damped least squares:  Δθ = Jᵀ (J Jᵀ + λ²I)⁻¹ · error.
            // Build the small (3×3 or 6×6) system M = J Jᵀ + λ²I and solve M·y = error.
            let m = error.count
            var mMatrix = [[Double]](repeating: [Double](repeating: 0, count: m), count: m)
            for col in columns {
                for a in 0..<m {
                    for b in 0..<m {
                        mMatrix[a][b] += col[a] * col[b]
                    }
                }
            }
            for a in 0..<m { mMatrix[a][a] += lambdaSq }

            guard let y = Self.solveLinearSystem(mMatrix, error) else { break }

            // Δθ_i = columnᵢ · y, clamped so no single step is wild.
            let maxStep = 0.25
            for i in 0..<n {
                var delta = 0.0
                for a in 0..<m { delta += columns[i][a] * y[a] }
                delta = min(max(delta, -maxStep), maxStep)
                theta[i] += delta
            }
            theta = clampToLimits(theta)
        }

        return IKResult(jointAngles: bestAngles, reached: false, positionError: bestError)
    }

    // MARK: - Small helpers

    private func clampToLimits(_ angles: [Double]) -> [Double] {
        zip(joints, angles).map { $0.clamp($1) }
    }

    private func defaultIKSeed(count n: Int) -> [Double] {
        var seed = [Double](repeating: 0, count: n)
        if n > 1 { seed[1] = -0.6 }
        if n > 2 { seed[2] = 0.8 }
        return seed
    }

    private func position(of m: simd_double4x4) -> SIMD3<Double> {
        SIMD3<Double>(m.columns.3.x, m.columns.3.y, m.columns.3.z)
    }

    private func zAxis(of m: simd_double4x4) -> SIMD3<Double> {
        simd_normalize(SIMD3<Double>(m.columns.2.x, m.columns.2.y, m.columns.2.z))
    }

    /// The 3×3 rotation part of a homogeneous transform.
    private func rotation(of m: simd_double4x4) -> simd_double3x3 {
        simd_double3x3(columns: (
            SIMD3<Double>(m.columns.0.x, m.columns.0.y, m.columns.0.z),
            SIMD3<Double>(m.columns.1.x, m.columns.1.y, m.columns.1.z),
            SIMD3<Double>(m.columns.2.x, m.columns.2.y, m.columns.2.z)
        ))
    }

    /// Extracts the rotation vector (axis · sin θ) from a small rotation matrix,
    /// i.e. the "vee" of its skew-symmetric part. `R[row][col] = R.columns.col[row]`.
    static func rotationVector(_ r: simd_double3x3) -> SIMD3<Double> {
        SIMD3<Double>(
            0.5 * (r.columns.1[2] - r.columns.2[1]),
            0.5 * (r.columns.2[0] - r.columns.0[2]),
            0.5 * (r.columns.0[1] - r.columns.1[0])
        )
    }

    /// Solves the dense linear system `A · x = b` by Gaussian elimination with
    /// partial pivoting. Used for the small (3×3 or 6×6) damped-least-squares
    /// system; simd only offers fixed inverses up to 4×4, so we roll our own.
    private static func solveLinearSystem(_ A: [[Double]], _ b: [Double]) -> [Double]? {
        let n = b.count
        var a = A
        var x = b
        for pivot in 0..<n {
            // Find the largest pivot in this column for numerical stability.
            var maxRow = pivot
            for r in (pivot + 1)..<n where abs(a[r][pivot]) > abs(a[maxRow][pivot]) {
                maxRow = r
            }
            guard abs(a[maxRow][pivot]) > 1e-12 else { return nil }
            a.swapAt(pivot, maxRow)
            x.swapAt(pivot, maxRow)

            // Eliminate below.
            for r in (pivot + 1)..<n {
                let factor = a[r][pivot] / a[pivot][pivot]
                for c in pivot..<n { a[r][c] -= factor * a[pivot][c] }
                x[r] -= factor * x[pivot]
            }
        }
        // Back-substitution.
        var result = [Double](repeating: 0, count: n)
        for row in stride(from: n - 1, through: 0, by: -1) {
            var sum = x[row]
            for c in (row + 1)..<n { sum -= a[row][c] * result[c] }
            result[row] = sum / a[row][row]
        }
        return result
    }
}
