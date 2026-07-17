//
//  MotionPlanner.swift
//  RobotArmKit
//
//  Collision-free motion planning in JOINT space with RRT-Connect.
//
//  WHY PLAN IN JOINT SPACE (and why RRT):
//  A straight line in tool space still has to be realised by the joints, and a
//  straight line in JOINT space can sweep the arm's links through an obstacle even
//  when the endpoints are clear. There's no closed-form "safe path," so we search:
//  RRT-Connect grows two random trees (one from the start, one from the goal) until
//  they meet, checking every edge for collision. It's probabilistically complete —
//  given enough samples it finds a path if one exists — and fast for the handful of
//  sphere obstacles in this scene. The raw path is then SHORTCUT-smoothed so the
//  arm doesn't wander.
//

import Foundation
import simd

/// A small, deterministic RNG (SplitMix64) so planning is reproducible in tests.
public struct SeededGenerator: RandomNumberGenerator {
    private var state: UInt64
    public init(seed: UInt64) { state = seed }
    public mutating func next() -> UInt64 {
        state = state &+ 0x9E3779B97F4A7C15
        var z = state
        z = (z ^ (z >> 30)) &* 0xBF58476D1CE4E5B9
        z = (z ^ (z >> 27)) &* 0x94D049BB133111EB
        return z ^ (z >> 31)
    }
}

public struct MotionPlanner {
    /// Capsule radius for each link during collision checks.
    public var linkRadius: Double
    /// Max joint-space distance a single tree extension advances (radians).
    public var stepSize: Double
    /// Spacing (radians of joint-space travel) at which an edge is sampled for
    /// collision — smaller is safer but slower.
    public var edgeResolution: Double
    /// How far outside the start/goal box (radians) sampling is allowed to roam.
    public var sampleMargin: Double
    public var maxIterations: Int
    public var checkSelf: Bool
    public var seed: UInt64

    public init(
        linkRadius: Double = 0.05,
        stepSize: Double = 0.25,
        edgeResolution: Double = 0.08,
        sampleMargin: Double = 1.2,
        maxIterations: Int = 4000,
        checkSelf: Bool = false,
        seed: UInt64 = 0xC0FFEE
    ) {
        self.linkRadius = linkRadius
        self.stepSize = stepSize
        self.edgeResolution = edgeResolution
        self.sampleMargin = sampleMargin
        self.maxIterations = maxIterations
        self.checkSelf = checkSelf
        self.seed = seed
    }

    /// Plans a collision-free joint path from `start` to `goal` around `obstacles`.
    /// Returns the waypoint list (including both endpoints) or nil if it can't find
    /// one (or an endpoint is itself in collision).
    public func plan(
        arm: RobotArm, from start: [Double], to goal: [Double],
        obstacles: [CollisionSphere]
    ) -> [[Double]]? {
        // An endpoint already in collision is hopeless — let the caller decide.
        if colliding(arm, start, obstacles) || colliding(arm, goal, obstacles) { return nil }

        // Fast path: if the straight joint-space move is already clear, take it.
        if edgeClear(arm, start, goal, obstacles) { return [start, goal] }

        var rng = SeededGenerator(seed: seed)
        let bounds = sampleBounds(arm: arm, start: start, goal: goal)

        var treeStart: [Node] = [Node(q: start, parent: -1)]
        var treeGoal: [Node] = [Node(q: goal, parent: -1)]
        var extendStartFirst = true

        for _ in 0..<maxIterations {
            let qRand = sample(bounds: bounds, rng: &rng)

            if extendStartFirst {
                let (statusA, idxA) = extend(arm, &treeStart, toward: qRand, obstacles: obstacles)
                if statusA != .trapped {
                    let (statusB, idxB) = connect(arm, &treeGoal, toward: treeStart[idxA].q, obstacles: obstacles)
                    if statusB == .reached {
                        return finish(arm, treeStart, idxA, treeGoal, idxB, obstacles, &rng)
                    }
                }
            } else {
                let (statusA, idxA) = extend(arm, &treeGoal, toward: qRand, obstacles: obstacles)
                if statusA != .trapped {
                    let (statusB, idxB) = connect(arm, &treeStart, toward: treeGoal[idxA].q, obstacles: obstacles)
                    if statusB == .reached {
                        return finish(arm, treeStart, idxB, treeGoal, idxA, obstacles, &rng)
                    }
                }
            }
            extendStartFirst.toggle()
        }
        return nil
    }

    // MARK: - RRT internals

    private struct Node { var q: [Double]; var parent: Int }
    private enum Status { case reached, advanced, trapped }

    private func extend(
        _ arm: RobotArm, _ tree: inout [Node], toward target: [Double],
        obstacles: [CollisionSphere]
    ) -> (Status, Int) {
        let nearestIdx = nearest(tree, target)
        let qNear = tree[nearestIdx].q
        let qNew = steer(from: qNear, to: target, arm: arm)
        guard edgeClear(arm, qNear, qNew, obstacles) else { return (.trapped, nearestIdx) }
        tree.append(Node(q: qNew, parent: nearestIdx))
        let newIdx = tree.count - 1
        return (distance(qNew, target) < 1e-6 ? .reached : .advanced, newIdx)
    }

    /// Repeatedly extends `tree` toward `target` until it arrives or gets stuck.
    private func connect(
        _ arm: RobotArm, _ tree: inout [Node], toward target: [Double],
        obstacles: [CollisionSphere]
    ) -> (Status, Int) {
        var status = Status.advanced
        var idx = -1
        while status == .advanced {
            (status, idx) = extend(arm, &tree, toward: target, obstacles: obstacles)
        }
        return (status, idx)
    }

    /// Reconstructs both half-paths and joins them into start→goal, then smooths.
    private func finish(
        _ arm: RobotArm,
        _ treeStart: [Node], _ idxStart: Int,
        _ treeGoal: [Node], _ idxGoal: Int,
        _ obstacles: [CollisionSphere], _ rng: inout SeededGenerator
    ) -> [[Double]] {
        let fromStart = reconstruct(treeStart, idxStart)          // start … conn
        let fromGoal = reconstruct(treeGoal, idxGoal)             // goal … conn
        let path = fromStart + fromGoal.reversed().dropFirst()    // start … conn … goal
        return shortcut(arm, Array(path), obstacles, &rng)
    }

    private func reconstruct(_ tree: [Node], _ index: Int) -> [[Double]] {
        var path: [[Double]] = []
        var i = index
        while i >= 0 {
            path.append(tree[i].q)
            i = tree[i].parent
        }
        return path.reversed()   // root → node
    }

    // MARK: - Sampling / steering / distance

    private func sampleBounds(arm: RobotArm, start: [Double], goal: [Double]) -> [(Double, Double)] {
        (0..<arm.joints.count).map { j in
            let lo = min(start[j], goal[j]) - sampleMargin
            let hi = max(start[j], goal[j]) + sampleMargin
            return (max(lo, arm.joints[j].minAngle), min(hi, arm.joints[j].maxAngle))
        }
    }

    private func sample(bounds: [(Double, Double)], rng: inout SeededGenerator) -> [Double] {
        bounds.map { lo, hi in Double.random(in: lo...max(lo, hi), using: &rng) }
    }

    /// Moves from `from` toward `to` by at most `stepSize`, clamped to joint limits.
    private func steer(from: [Double], to: [Double], arm: RobotArm) -> [Double] {
        let delta = zip(to, from).map(-)
        let dist = sqrt(delta.reduce(0) { $0 + $1 * $1 })
        let scale = dist <= stepSize ? 1.0 : stepSize / dist
        return (0..<from.count).map { i in
            let v = from[i] + delta[i] * scale
            return min(max(v, arm.joints[i].minAngle), arm.joints[i].maxAngle)
        }
    }

    private func distance(_ a: [Double], _ b: [Double]) -> Double {
        sqrt(zip(a, b).reduce(0) { $0 + ($1.0 - $1.1) * ($1.0 - $1.1) })
    }

    private func nearest(_ tree: [Node], _ q: [Double]) -> Int {
        var best = 0
        var bestD = Double.greatestFiniteMagnitude
        for (i, node) in tree.enumerated() {
            let d = distance(node.q, q)
            if d < bestD { bestD = d; best = i }
        }
        return best
    }

    // MARK: - Collision checks

    private func colliding(_ arm: RobotArm, _ q: [Double], _ obstacles: [CollisionSphere]) -> Bool {
        arm.isColliding(jointAngles: q, obstacles: obstacles, linkRadius: linkRadius, checkSelf: checkSelf)
    }

    /// True if the straight joint-space edge a→b is collision-free at every sample.
    private func edgeClear(
        _ arm: RobotArm, _ a: [Double], _ b: [Double], _ obstacles: [CollisionSphere]
    ) -> Bool {
        let dist = distance(a, b)
        let steps = max(1, Int((dist / edgeResolution).rounded(.up)))
        for k in 0...steps {
            let t = Double(k) / Double(steps)
            let q = (0..<a.count).map { a[$0] + (b[$0] - a[$0]) * t }
            if colliding(arm, q, obstacles) { return false }
        }
        return true
    }

    /// Shortcut smoothing: repeatedly try to replace a sub-path between two random
    /// waypoints with a direct edge when that edge is collision-free.
    private func shortcut(
        _ arm: RobotArm, _ path: [[Double]], _ obstacles: [CollisionSphere],
        _ rng: inout SeededGenerator
    ) -> [[Double]] {
        guard path.count > 2 else { return path }
        var result = path
        for _ in 0..<100 {
            guard result.count > 2 else { break }
            let i = Int.random(in: 0..<(result.count - 1), using: &rng)
            let j = Int.random(in: (i + 1)..<result.count, using: &rng)
            if j - i >= 2, edgeClear(arm, result[i], result[j], obstacles) {
                result.removeSubrange((i + 1)..<j)
            }
        }
        return result
    }
}
