import Testing
import simd
@testable import RobotArmKit

/// Regression: replicates the app's world→robot transform chain (SandboxScene +
/// AssemblyController) and asserts that every model-car wheel — the rear-right
/// wheel especially — is IK-reachable for pickup, pre-insert, and mate, from the
/// arm base position the app actually uses.
struct WheelRRReach {

    static let armScale = 2.0
    static let benchTop = 0.75
    static let armBase = SIMD3<Double>(0, benchTop, -0.15)

    static func rkToRobotVec(_ v: SIMD3<Double>) -> SIMD3<Double> { SIMD3(v.x, -v.z, v.y) }
    static func rkToRobotRot(_ r: simd_double3x3) -> simd_double3x3 {
        simd_double3x3(columns: (rkToRobotVec(r.columns.0), rkToRobotVec(r.columns.1), rkToRobotVec(r.columns.2)))
    }

    static func worldToRobot(_ p: Pose, base: SIMD3<Double>) -> Pose {
        Pose(position: rkToRobotVec(p.position - base) / armScale, orientation: rkToRobotRot(p.orientation))
    }

    static var partInTool: simd_double4x4 {
        let rt = simd_double3x3(columns: (
            SIMD3<Double>(1, 0, 0), SIMD3<Double>(0, 0, 1), SIMD3<Double>(0, -1, 0)
        ))
        let toolWorld = Pose(position: SIMD3<Double>(0, 0.02, 0), orientation: rt)
        return toolWorld.matrix.inverse * Pose(position: .zero, orientation: matrix_identity_double3x3).matrix
    }
    static func toolWorld(part: Pose) -> Pose { Pose(part.matrix * partInTool.inverse) }

    static func downTool(at p: SIMD3<Double>) -> Pose {
        Pose(position: p, orientation: simd_double3x3(columns: (
            SIMD3<Double>(1, 0, 0), SIMD3<Double>(0, 0, 1), SIMD3<Double>(0, -1, 0)
        )))
    }

    static func reaches(_ toolWorldPose: Pose, base: SIMD3<Double>, arm: RobotArm) -> Bool {
        let target = worldToRobot(toolWorldPose, base: base)
        let seeds: [[Double]] = [
            [0.0, -1.2, 1.4, -1.6, -1.57, 0.0],
            [0.6, -1.0, 1.2, -1.4, -1.57, 0.0],
            [-0.6, -1.0, 1.2, -1.4, 1.57, 0.0],
        ]
        return seeds.contains { seed in
            arm.inverseKinematics(targetPose: target, initialGuess: seed,
                                  tolerance: 4e-3, orientationTolerance: 6 * .pi / 180).reached
        }
    }

    @Test func allWheelsReachableFromAppBase() {
        let arm = RobotArm.ur5
        let def = ModelCar.definition
        let chassisY = Self.benchTop + CarGeometry.chassisHeight / 2
        let chassis = Pose(position: SIMD3<Double>(0.46, chassisY, -0.28), orientation: matrix_identity_double3x3)
        let wheelY = Self.benchTop + CarGeometry.wheelThickness / 2
        let spots: [SIMD3<Double>] = [
            SIMD3(0.30, wheelY, 0.40), SIMD3(0.66, wheelY, 0.40),
            SIMD3(0.30, wheelY, 0.76), SIMD3(0.66, wheelY, 0.76),
        ]

        for (i, w) in ModelCar.wheels.enumerated() {
            let step = def.step(id: w.stepID)!
            let ins = def.insertionPoses(for: step, targetPartWorld: chassis, backoff: 0.06)!
            #expect(Self.reaches(Self.downTool(at: spots[i] + SIMD3(0, 0.22, 0)), base: Self.armBase, arm: arm),
                    "\(w.id): tray approach unreachable")
            #expect(Self.reaches(Self.downTool(at: spots[i] + SIMD3(0, 0.02, 0)), base: Self.armBase, arm: arm),
                    "\(w.id): tray descend unreachable")
            #expect(Self.reaches(Self.toolWorld(part: ins.preInsert), base: Self.armBase, arm: arm),
                    "\(w.id): pre-insert unreachable")
            #expect(Self.reaches(Self.toolWorld(part: ins.mate), base: Self.armBase, arm: arm),
                    "\(w.id): mate unreachable")
        }
    }
}
