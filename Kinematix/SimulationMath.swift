//
//  SimulationMath.swift
//  Kinematix
//
//  Small, focused helpers for bridging RobotArmKit's math into RealityKit, plus
//  procedural geometry helpers. These are the "nontrivial math" bits, so they're
//  heavily commented and kept out of the view code.
//

import Foundation
import RealityKit
import simd

// MARK: - Precision bridging (Double → Float)

extension simd_float4x4 {
    /// Converts a double-precision 4x4 matrix (what RobotArmKit produces) into
    /// the single-precision matrix RealityKit uses for entity transforms.
    ///
    /// WHY: robotics math is conventionally done in Double for accuracy, but
    /// RealityKit (and GPUs) work in Float. We do the kinematics in Double and
    /// only narrow to Float at the very last step, right before rendering.
    init(_ m: simd_double4x4) {
        self.init(
            SIMD4<Float>(m.columns.0),
            SIMD4<Float>(m.columns.1),
            SIMD4<Float>(m.columns.2),
            SIMD4<Float>(m.columns.3)
        )
    }
}

extension simd_double4x4 {
    /// The translation (position) stored in a homogeneous transform's 4th column.
    var translation: SIMD3<Float> {
        SIMD3<Float>(Float(columns.3.x), Float(columns.3.y), Float(columns.3.z))
    }
}

// MARK: - Coordinate-system conversion (robotics Z-up → RealityKit Y-up)

enum CoordinateSpace {
    /// Rotation that converts the robot's coordinate frame into RealityKit's.
    ///
    /// WHY THIS EXISTS:
    /// Robotics (and the UR5 DH parameters) use a **Z-up** world: the arm's
    /// first joint offset `d1` lifts the arm straight up along +Z. RealityKit
    /// instead uses a **Y-up** world: +Y is up, and the ground lies in the XZ
    /// plane.
    ///
    /// Rotating -90° about the X axis maps robot axes to RealityKit axes:
    ///     robot +Z (up)   → RealityKit +Y (up)
    ///     robot +X        → RealityKit +X
    ///     robot +Y        → RealityKit -Z
    ///
    /// We apply this ONCE, to the arm's root entity. Every child of that root is
    /// then authored in plain robot coordinates, and the root rotation makes it
    /// stand up correctly on the floor. This keeps the per-joint math simple.
    static let robotToRealityKit = simd_quatf(angle: -.pi / 2, axis: SIMD3<Float>(1, 0, 0))
}

// MARK: - Connecting two points with a cylinder ("link")

extension ModelEntity {
    /// Positions and orients a **unit-height** cylinder so it spans from point
    /// `a` to point `b`. The cylinder must have been created with `height == 1`;
    /// we stretch it along its local Y axis using `scale.y`.
    ///
    /// THE MATH (why each step):
    ///   * A RealityKit cylinder is created centered at the origin, running along
    ///     its local **Y** axis. So a unit cylinder spans y ∈ [-0.5, 0.5].
    ///   * `midpoint = (a + b) / 2` — the cylinder is centered, so its center
    ///     must sit halfway between the two endpoints.
    ///   * `length = |b - a|` — how far apart the points are; we scale the unit
    ///     cylinder's Y by this to make it reach exactly from a to b.
    ///   * `orientation` rotates the local Y axis to point along `(b - a)`.
    ///     `simd_quatf(from:to:)` finds the shortest rotation between two unit
    ///     vectors, which is exactly what we want.
    /// Positions and orients a FIXED-length cylinder so it spans from `a` to `b`,
    /// WITHOUT scaling it. The caller must have built the cylinder with a height
    /// equal to `|b - a|` already. We avoid scaling because non-uniform scale
    /// distorts cylinder/capsule collision shapes (RealityKit only supports
    /// non-uniform scale on box/mesh colliders), and our link lengths are
    /// constant anyway (the distance between two joint origins is √(a²+d²)).
    func alignBetween(_ a: SIMD3<Float>, _ b: SIMD3<Float>) {
        position = (a + b) / 2
        let delta = b - a
        let length = simd_length(delta)
        guard length > 1e-6 else { return }
        let direction = delta / length
        let up = SIMD3<Float>(0, 1, 0)
        if simd_dot(up, direction) < -0.9999 {
            orientation = simd_quatf(angle: .pi, axis: SIMD3<Float>(1, 0, 0))
        } else {
            orientation = simd_quatf(from: up, to: direction)
        }
    }

    func stretchBetween(_ a: SIMD3<Float>, _ b: SIMD3<Float>) {
        let delta = b - a
        let length = simd_length(delta)

        position = (a + b) / 2
        scale = SIMD3<Float>(1, max(length, 0.0001), 1)

        guard length > 1e-6 else {
            orientation = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1) // identity
            return
        }

        let direction = delta / length
        let up = SIMD3<Float>(0, 1, 0)

        // Edge case: if the direction is nearly opposite to +Y, `from:to:` can
        // be numerically unstable, so rotate a clean 180° about X instead.
        if simd_dot(up, direction) < -0.9999 {
            orientation = simd_quatf(angle: .pi, axis: SIMD3<Float>(1, 0, 0))
        } else {
            orientation = simd_quatf(from: up, to: direction)
        }
    }
}
