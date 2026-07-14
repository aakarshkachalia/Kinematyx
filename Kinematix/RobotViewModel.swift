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
    /// The arm being simulated. `.ur5` carries the real DH parameters + limits.
    let arm = RobotArm.ur5

    /// One commanded angle per joint, in RADIANS, base first.
    /// Radians is the unit RobotArmKit expects; the sliders convert to/from
    /// degrees for display only.
    var jointAngles: [Double]

    init() {
        // Start every joint at 0 (the UR5's documented reference pose).
        jointAngles = [Double](repeating: 0, count: arm.degreesOfFreedom)
    }

    /// Sends the arm back to its home position (all joints at 0 radians).
    func resetToHome() {
        jointAngles = [Double](repeating: 0, count: arm.degreesOfFreedom)
    }

    /// The end-effector position in the arm's base frame (meters), for display.
    var endEffectorPosition: SIMD3<Double> {
        arm.endEffectorPosition(jointAngles: jointAngles)
    }
}
