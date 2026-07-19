//
//  RobotViewModel.swift
//  Kinematix
//
//  Shared state between the SwiftUI controls (sliders, reset button) and the
//  RealityKit viewport. The sliders write joint angles here; the viewport reads
//  them back to drive the arm via forward kinematics.
//
//  IMPORTANT: this type holds ONLY state, never kinematics math. All of the
//  actual math lives in RobotArmKit — we just call into it. That keeps the
//  "brains" reusable and testable, exactly as the project requires.
//

import Foundation
import Observation
import RobotArmKit

/// Observable state for the robot arm. SwiftUI views that read `jointAngles`
/// automatically re-render (and the RealityView's update closure re-runs) when
/// the angles change.
@MainActor
@Observable
final class RobotViewModel {
    /// The arm being simulated. Swappable between profiles (Phase 10). The app
    /// opens on the UR3 (index 1 in `allProfiles`).
    private(set) var arm: RobotArm = .ur3
    /// Index into `RobotArm.allProfiles` for the current arm.
    private(set) var profileIndex = 1

    /// On-screen scale for the current arm, chosen so different-reach arms look a
    /// similar size. (UR5 reaches ~0.85 m, UR3 ~0.5 m.)
    private let profileScales: [Float] = [2.0, 3.4]
    var displayScale: Float { profileScales[min(profileIndex, profileScales.count - 1)] }

    /// One commanded angle per joint, in RADIANS, base first.
    var jointAngles: [Double]

    init() {
        jointAngles = [Double](repeating: 0, count: RobotArm.ur3.degreesOfFreedom)
    }

    /// Sends the arm back to its home position (all joints at 0 radians).
    func resetToHome() {
        jointAngles = [Double](repeating: 0, count: arm.degreesOfFreedom)
    }

    /// Switches to a different arm profile and resets to its home pose.
    func selectProfile(_ index: Int) {
        guard index >= 0, index < RobotArm.allProfiles.count, index != profileIndex else { return }
        profileIndex = index
        arm = RobotArm.allProfiles[index]
        jointAngles = [Double](repeating: 0, count: arm.degreesOfFreedom)
    }

    /// The end-effector position in the arm's base frame (meters), for display.
    var endEffectorPosition: SIMD3<Double> {
        arm.endEffectorPosition(jointAngles: jointAngles)
    }

    // MARK: - Live readouts for the side panels

    /// DH parameters + the live joint angle (θ) for each joint (Phase 2).
    var dhRows: [(index: Int, alpha: Double, a: Double, d: Double, theta: Double)] {
        arm.joints.enumerated().map { i, joint in
            (i + 1, joint.dh.alpha, joint.dh.a, joint.dh.d, jointAngles[i])
        }
    }

    /// Whether the current pose is near a kinematic singularity (Phase 3).
    var isNearSingularity: Bool { arm.isNearSingularity(jointAngles: jointAngles) }

    /// Estimated static torque per joint (N·m) for a given held payload (Phase 4).
    func jointTorques(payloadMass: Double) -> [Double] {
        arm.jointTorques(jointAngles: jointAngles, payloadMass: payloadMass)
    }

    var torqueLimits: [Double] { arm.jointTorqueLimits }
}
