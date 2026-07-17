//
//  Collision.swift
//  RobotArmKit
//
//  A lightweight collision model for the arm: each link is treated as a CAPSULE
//  (a line segment between two joint origins, given some radius), and obstacles
//  are bounding SPHERES. Collision is then just "is any link-segment closer to an
//  obstacle centre than linkRadius + obstacleRadius?" — cheap and good enough to
//  plan safe paths (Phase: obstacle-aware motion). Pure math, no rendering.
//

import Foundation
import simd

/// A bounding sphere obstacle in the arm's base (robot) frame.
public struct CollisionSphere: Sendable, Equatable {
    public var center: SIMD3<Double>
    public var radius: Double
    public init(center: SIMD3<Double>, radius: Double) {
        self.center = center
        self.radius = radius
    }
}

public extension RobotArm {

    /// The arm as a chain of line segments: base→joint1→…→tool, in the base frame.
    /// These are the centre-lines of the link capsules used for collision tests.
    func linkSegments(jointAngles: [Double]) -> [(SIMD3<Double>, SIMD3<Double>)] {
        let frames = forwardKinematics(jointAngles: jointAngles)
        var points: [SIMD3<Double>] = [SIMD3<Double>(0, 0, 0)]
        points.append(contentsOf: frames.map {
            SIMD3<Double>($0.columns.3.x, $0.columns.3.y, $0.columns.3.z)
        })
        var segments: [(SIMD3<Double>, SIMD3<Double>)] = []
        for i in 0..<(points.count - 1) { segments.append((points[i], points[i + 1])) }
        return segments
    }

    /// Whether the arm at `jointAngles` collides with any obstacle (each link
    /// capsule vs each obstacle sphere), and optionally with ITSELF (non-adjacent
    /// link capsules getting too close).
    ///
    /// - Parameters:
    ///   - linkRadius: the capsule radius modelling each link's thickness.
    ///   - checkSelf: also reject configurations where the arm folds into itself.
    ///   - selfRadius: capsule radius used for the self-collision test.
    func isColliding(
        jointAngles: [Double],
        obstacles: [CollisionSphere],
        linkRadius: Double = 0.05,
        checkSelf: Bool = false,
        selfRadius: Double = 0.045
    ) -> Bool {
        let segments = linkSegments(jointAngles: jointAngles)

        for segment in segments {
            for obstacle in obstacles {
                let d = Self.segmentPointDistance(segment.0, segment.1, obstacle.center)
                if d < linkRadius + obstacle.radius { return true }
            }
        }

        if checkSelf {
            // Only test non-adjacent links: neighbours always touch at the shared
            // joint, so comparing i with i+1 would always "collide".
            for i in 0..<segments.count {
                for j in (i + 2)..<segments.count {
                    let d = Self.segmentSegmentDistance(
                        segments[i].0, segments[i].1, segments[j].0, segments[j].1
                    )
                    if d < 2 * selfRadius { return true }
                }
            }
        }
        return false
    }

    // MARK: - Distance primitives

    /// Shortest distance from point `p` to segment `a`–`b`.
    static func segmentPointDistance(
        _ a: SIMD3<Double>, _ b: SIMD3<Double>, _ p: SIMD3<Double>
    ) -> Double {
        let ab = b - a
        let denom = simd_dot(ab, ab)
        guard denom > 1e-12 else { return simd_length(p - a) }   // degenerate segment
        let t = min(1, max(0, simd_dot(p - a, ab) / denom))
        let projection = a + ab * t
        return simd_length(p - projection)
    }

    /// Shortest distance between segment `p1`–`q1` and segment `p2`–`q2`
    /// (Ericson, *Real-Time Collision Detection*). Used for self-collision.
    static func segmentSegmentDistance(
        _ p1: SIMD3<Double>, _ q1: SIMD3<Double>,
        _ p2: SIMD3<Double>, _ q2: SIMD3<Double>
    ) -> Double {
        let d1 = q1 - p1          // direction of segment 1
        let d2 = q2 - p2          // direction of segment 2
        let r = p1 - p2
        let a = simd_dot(d1, d1)
        let e = simd_dot(d2, d2)
        let f = simd_dot(d2, r)

        var s = 0.0, t = 0.0
        let eps = 1e-12

        if a <= eps && e <= eps {
            return simd_length(p1 - p2)   // both degenerate
        }
        if a <= eps {
            s = 0
            t = min(1, max(0, f / e))
        } else {
            let c = simd_dot(d1, r)
            if e <= eps {
                t = 0
                s = min(1, max(0, -c / a))
            } else {
                let b = simd_dot(d1, d2)
                let denom = a * e - b * b
                if denom > eps {
                    s = min(1, max(0, (b * f - c * e) / denom))
                } else {
                    s = 0
                }
                t = (b * s + f) / e
                if t < 0 {
                    t = 0
                    s = min(1, max(0, -c / a))
                } else if t > 1 {
                    t = 1
                    s = min(1, max(0, (b - c) / a))
                }
            }
        }
        let c1 = p1 + d1 * s
        let c2 = p2 + d2 * t
        return simd_length(c1 - c2)
    }
}
