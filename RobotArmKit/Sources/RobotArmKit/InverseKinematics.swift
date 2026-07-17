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
//  never flails or returns NaNs. We use DLS rather than the simpler Jacobian
//  TRANSPOSE method because transpose converges poorly on orientation error —
//  it can crawl or oscillate on the rotational rows — whereas DLS solves the
//  small normal-equations system directly and converges quickly on both.
//
//  Three modes, sharing one core:
//   * position only — 3 error terms (x, y, z); orientation left free.
//   * position + approach direction — 6 error terms (position + which way the
//     tool's approach axis points), wrist roll left free. Used by click-to-move.
//   * position + FULL orientation (a target Pose) — 6 error terms pinning all
//     three rotational DOF, so a grasped part lines up exactly. Used by assembly.
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
    /// Final orientation error at the solution (radians). Zero in position-only
    /// mode (no orientation was requested).
    public let orientationError: Double
    /// Whether the returned configuration is near a kinematic singularity
    /// (small manipulability). A `reached == true` result can still be flagged
    /// here — it means the solution is valid but poorly conditioned.
    public let nearSingularity: Bool
}

public extension RobotArm {

    // MARK: - Public entry points

    /// Position (and optionally approach-direction / full-orientation) IK.
    ///
    /// This is the long-standing entry point used by click-to-move and grasp
    /// alignment. It is a thin wrapper over the shared `solveIK` core:
    ///   * pass only `targetPosition` → position-only (orientation free);
    ///   * add `approachAxis` → also aim the tool's +Z along it (roll free);
    ///   * add `targetOrientation` → pin the whole tool frame.
    ///
    /// - Parameter orientationTolerance: how close (radians) the orientation must
    ///   be to count as reached. `nil` keeps the historical defaults (4° for a
    ///   full frame, 6° for approach-only) so existing behavior is unchanged;
    ///   callers that need tight pose control pass an explicit value.
    /// - Returns: an `IKResult`. Never throws or returns NaNs; unreachable
    ///   targets yield the closest solution with `reached == false`.
    func inverseKinematics(
        targetPosition: SIMD3<Double>,
        approachAxis: SIMD3<Double>? = nil,
        targetOrientation: simd_double3x3? = nil,
        initialGuess: [Double]? = nil,
        maxIterations: Int = 400,
        tolerance: Double = 1e-3,
        orientationTolerance: Double? = nil
    ) -> IKResult {
        // Preserve the historical, mode-specific default tolerances.
        let defaultTol = (targetOrientation != nil)
            ? 4 * Double.pi / 180
            : 6 * Double.pi / 180
        return solveIK(
            targetPosition: targetPosition,
            desiredApproach: approachAxis,
            targetOrientation: targetOrientation,
            initialGuess: initialGuess,
            maxIterations: maxIterations,
            positionTolerance: tolerance,
            orientationTolerance: orientationTolerance ?? defaultTol
        )
    }

    /// Full 6-DOF IK toward a target `Pose` (position AND orientation). This is
    /// the entry point assembly uses: a wheel must arrive at the right point AND
    /// pointing the right way for its hole to line up with the axle.
    ///
    /// Defaults to a tight 2° orientation tolerance because assembly cares about
    /// exact alignment; tests tighten it further to prove sub-degree accuracy.
    func inverseKinematics(
        targetPose: Pose,
        initialGuess: [Double]? = nil,
        maxIterations: Int = 400,
        tolerance: Double = 1e-3,
        orientationTolerance: Double = 2 * Double.pi / 180
    ) -> IKResult {
        solveIK(
            targetPosition: targetPose.position,
            desiredApproach: nil,
            targetOrientation: targetPose.orientation,
            initialGuess: initialGuess,
            maxIterations: maxIterations,
            positionTolerance: tolerance,
            orientationTolerance: orientationTolerance
        )
    }

    // MARK: - Core solver

    /// The shared damped-least-squares core. All public IK methods funnel here.
    private func solveIK(
        targetPosition: SIMD3<Double>,
        desiredApproach approachAxisIn: SIMD3<Double>?,
        targetOrientation: simd_double3x3?,
        initialGuess: [Double]?,
        maxIterations: Int,
        positionTolerance: Double,
        orientationTolerance: Double
    ) -> IKResult {
        let n = joints.count
        var theta = clampToLimits(initialGuess ?? defaultIKSeed(count: n))

        // DAMPING FACTOR CHOICE (λ²).
        // The DLS step is  Δθ = Jᵀ (J Jᵀ + λ²I)⁻¹ e.  The λ²I term bounds how big
        // (J Jᵀ)⁻¹ can get: without it, a near-singular J Jᵀ inverts to huge
        // numbers and the step explodes. We pick a BASE λ₀ ≈ 0.06 (λ₀² ≈ 0.0036):
        // small enough that in well-conditioned poses the solver still converges
        // to sub-millimetre / sub-degree accuracy, yet large enough to keep every
        // step tame. Empirically 0.06 is the sweet spot for the UR5's scale
        // (links ~0.1–0.4 m); much smaller lets the wrist jitter near
        // singularities, much larger makes convergence sluggish.
        let baseLambdaSq = 0.06 * 0.06
        // SINGULARITY-ROBUST DAMPING (Nakamura & Hanafusa): near a singularity we
        // ADD damping so the solver degrades gracefully instead of demanding
        // enormous joint speeds. We ramp λ² up as the manipulability `w` falls
        // below the same threshold the app already uses to WARN about
        // singularities, reaching +maxExtraLambdaSq at a full singularity (w→0).
        let maxExtraLambdaSq = 0.08 * 0.08
        let singularityKnee = Self.singularityThreshold

        // Weight on the orientation error so it's comparable to the position
        // error (which is in meters, usually small). Too high and position
        // suffers; too low and the tool won't point where we asked.
        let orientationWeight = 0.5
        let desiredApproach = approachAxisIn.map { simd_normalize($0) }
        // Any orientation target (a full frame, or just the approach axis) adds
        // the 3 angular rows to the Jacobian, making it 6×N.
        let useOrientation = (targetOrientation != nil) || (desiredApproach != nil)

        var bestAngles = theta
        var bestError = Double.greatestFiniteMagnitude
        var bestRotError = 0.0

        for _ in 0..<maxIterations {
            let frames = forwardKinematics(jointAngles: theta)
            let toolPos = position(of: frames.last!)
            let posError = targetPosition - toolPos
            let distance = simd_length(posError)

            // Orientation error, as a base-frame ROTATION VECTOR (axis · angle).
            // Using axis-angle (not Euler) keeps the error well-defined and
            // continuous even near singular orientations.
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
                orientationAligned = simd_length(rotError) < orientationTolerance
            } else if let desired = desiredApproach {
                // APPROACH ONLY: align the tool's +Z with `desired`; roll is free.
                let current = zAxis(of: frames.last!)
                rotError = simd_cross(current, desired)
                orientationAligned = simd_dot(current, desired) > cos(orientationTolerance)
            }

            if distance < bestError {
                bestError = distance
                bestAngles = theta
                bestRotError = simd_length(rotError)
            }
            if distance < positionTolerance && orientationAligned {
                return IKResult(
                    jointAngles: theta, reached: true, positionError: distance,
                    orientationError: simd_length(rotError),
                    nearSingularity: isNearSingularity(jointAngles: theta)
                )
            }

            // SINGULARITY-ROBUST λ²: raise damping as the pose approaches a
            // singularity (manipulability `w` drops below the knee).
            let w = manipulability(jointAngles: theta)
            let lambdaSq: Double
            if w < singularityKnee {
                let t = 1 - w / singularityKnee     // 0 at the knee → 1 at w = 0
                lambdaSq = baseLambdaSq + maxExtraLambdaSq * t * t
            } else {
                lambdaSq = baseLambdaSq
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

        return IKResult(
            jointAngles: bestAngles, reached: false, positionError: bestError,
            orientationError: bestRotError,
            nearSingularity: isNearSingularity(jointAngles: bestAngles)
        )
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
