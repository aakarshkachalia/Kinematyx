import Testing
import simd
@testable import RobotArmKit

struct WheelRRDiag {
    let q = simd_quatd(angle: -.pi/2, axis: SIMD3<Double>(1,0,0))   // robotToRealityKit
    func w2r(_ p: SIMD3<Double>) -> SIMD3<Double> { q.inverse.act(p - SIMD3<Double>(0,0.75,0)) / 2.0 }
    func oriW2R(_ r: simd_double3x3) -> simd_double3x3 { simd_double3x3(q.inverse * simd_quatd(r)) }
    func downTool() -> simd_double3x3 { simd_double3x3(columns:(SIMD3<Double>(1,0,0),SIMD3<Double>(0,0,1),SIMD3<Double>(0,-1,0))) }

    @Test func wheelRRPreInsert() {
        let arm = RobotArm.ur5
        let def = ModelCar.definition
        let step = def.step(id: "step.wheelRR")!
        let chassis = Pose(position: SIMD3<Double>(0.46, 0.80, -0.28))
        let tray = SIMD3<Double>(0.66, 0.79, 0.76)                    // wheelRR spawn
        // grab from above
        let toolGrab = Pose(position: tray + SIMD3<Double>(0,0.02,0), orientation: downTool())
        let wheelSpawn = Pose(position: tray)                        // identity orient
        let partInTool = toolGrab.matrix.inverse * wheelSpawn.matrix
        let ins = def.insertionPoses(for: step, targetPartWorld: chassis, backoff: 0.06)!
        let toolPre = Pose(ins.preInsert.matrix * partInTool.inverse)
        let robotPos = w2r(toolPre.position)
        let robotOri = oriW2R(toolPre.orientation)
        let target = Pose(position: robotPos, orientation: robotOri)
        print("RR preInsert tool robot pos:", robotPos, "dist:", simd_length(robotPos))
        let az = atan2(robotPos.y, robotPos.x)
        print("azimuth:", az)
        let oldSeeds: [[Double]] = [[0,-1.2,1.4,-1.6,-1.57,0],[0.6,-1.0,1.2,-1.4,-1.57,0],[-0.6,-1.0,1.2,-1.4,1.57,0]]
        let azSeeds: [[Double]] = [[az,-1.2,1.4,-1.6,-1.57,0],[az,-1.0,1.2,-1.4,1.57,0],[az,-1.5,1.8,-1.8,-1.57,0]]
        func tryset(_ name: String, _ seeds: [[Double]]) {
            var ok=false; var best=9.9
            for s in seeds { let r=arm.inverseKinematics(targetPose: target, initialGuess: s, tolerance:4e-3, orientationTolerance:6 * .pi/180); best=min(best,r.positionError); if r.reached {ok=true} }
            print(name, "reached:", ok, "bestPosErr:", best)
        }
        tryset("OLD seeds", oldSeeds)
        tryset("AZ seeds", azSeeds)
        #expect(true)
    }
}
