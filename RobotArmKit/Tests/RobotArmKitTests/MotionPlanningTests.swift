//
//  MotionPlanningTests.swift
//  RobotArmKitTests
//
//  Collision-model distance checks + RRT-Connect planning behaviour.
//

import Testing
import simd
@testable import RobotArmKit

struct MotionPlanningTests {

    // MARK: Distance primitives

    @Test func segmentPointDistanceBasics() {
        let a = SIMD3<Double>(0, 0, 0)
        let b = SIMD3<Double>(1, 0, 0)
        // Perpendicular from above the midpoint.
        #expect(abs(RobotArm.segmentPointDistance(a, b, SIMD3<Double>(0.5, 1, 0)) - 1) < 1e-9)
        // Past the end clamps to the endpoint.
        #expect(abs(RobotArm.segmentPointDistance(a, b, SIMD3<Double>(2, 0, 0)) - 1) < 1e-9)
        // On the segment → zero.
        #expect(RobotArm.segmentPointDistance(a, b, SIMD3<Double>(0.3, 0, 0)) < 1e-9)
    }

    @Test func segmentSegmentDistanceBasics() {
        // Two parallel unit segments 1 apart in Y.
        let d = RobotArm.segmentSegmentDistance(
            SIMD3<Double>(0, 0, 0), SIMD3<Double>(1, 0, 0),
            SIMD3<Double>(0, 1, 0), SIMD3<Double>(1, 1, 0)
        )
        #expect(abs(d - 1) < 1e-9)
        // Crossing segments (perpendicular, touching at origin) → zero.
        let d2 = RobotArm.segmentSegmentDistance(
            SIMD3<Double>(-1, 0, 0), SIMD3<Double>(1, 0, 0),
            SIMD3<Double>(0, -1, 0), SIMD3<Double>(0, 1, 0)
        )
        #expect(d2 < 1e-9)
    }

    // MARK: isColliding

    @Test func detectsAndClearsObstacle() {
        let arm = RobotArm.ur5
        let q = [0.2, -0.6, 0.8, 0.0, 0.5, 0.0]
        // A sphere right on the tool point must register as a collision...
        let tool = arm.endEffectorPosition(jointAngles: q)
        let onTool = [CollisionSphere(center: tool, radius: 0.05)]
        #expect(arm.isColliding(jointAngles: q, obstacles: onTool, linkRadius: 0.05))
        // ...and a sphere far away must not.
        let far = [CollisionSphere(center: SIMD3<Double>(5, 5, 5), radius: 0.1)]
        #expect(!arm.isColliding(jointAngles: q, obstacles: far, linkRadius: 0.05))
    }

    // MARK: Planning

    /// Validates a returned path: endpoints match and every edge is collision-free
    /// at a fine interpolation (independent re-check, not trusting the planner).
    private func validate(
        _ path: [[Double]], arm: RobotArm, start: [Double], goal: [Double],
        obstacles: [CollisionSphere], linkRadius: Double
    ) {
        #expect(path.first == start)
        #expect(path.last == goal)
        for k in 1..<path.count {
            let a = path[k - 1], b = path[k]
            let dist = sqrt(zip(a, b).reduce(0) { $0 + ($1.0 - $1.1) * ($1.0 - $1.1) })
            let steps = max(1, Int((dist / 0.03).rounded(.up)))
            for s in 0...steps {
                let t = Double(s) / Double(steps)
                let q = (0..<a.count).map { a[$0] + (b[$0] - a[$0]) * t }
                #expect(!arm.isColliding(jointAngles: q, obstacles: obstacles, linkRadius: linkRadius))
            }
        }
    }

    @Test func trivialPathWhenClear() {
        let arm = RobotArm.ur5
        let start = [0.0, -0.6, 0.8, 0.0, 0.4, 0.0]
        let goal = [0.4, -0.5, 0.7, 0.1, 0.5, 0.0]
        let planner = MotionPlanner()
        let path = planner.plan(arm: arm, from: start, to: goal, obstacles: [])
        #expect(path != nil)
        // No obstacles → the direct edge is clear, so the path is just the endpoints.
        #expect(path?.count == 2)
        if let path { validate(path, arm: arm, start: start, goal: goal, obstacles: [], linkRadius: 0.05) }
    }

    @Test func routesAroundBlockingObstacle() {
        let arm = RobotArm.ur5
        // A base rotation that sweeps the tool through a wide arc.
        let start = [0.6, -0.6, 0.9, 0.0, 0.5, 0.0]
        let goal = [-0.6, -0.6, 0.9, 0.0, 0.5, 0.0]

        // Put a sphere on the tool position at the straight-line MIDPOINT config, so
        // the direct joint-space move is blocked but detours exist.
        let mid = (0..<start.count).map { (start[$0] + goal[$0]) / 2 }
        let block = CollisionSphere(center: arm.endEffectorPosition(jointAngles: mid), radius: 0.06)
        let obstacles = [block]
        let linkRadius = 0.05

        // Sanity: the straight-line midpoint really is in collision.
        #expect(arm.isColliding(jointAngles: mid, obstacles: obstacles, linkRadius: linkRadius))

        var planner = MotionPlanner(linkRadius: linkRadius, maxIterations: 8000, seed: 42)
        planner.stepSize = 0.2
        let path = planner.plan(arm: arm, from: start, to: goal, obstacles: obstacles)
        #expect(path != nil, "planner should route around the obstacle")
        if let path {
            #expect(path.count > 2, "a detour needs intermediate waypoints")
            validate(path, arm: arm, start: start, goal: goal, obstacles: obstacles, linkRadius: linkRadius)
        }
    }
}
