//
//  URScript.swift
//  Kinematix
//
//  Converts a recorded pose sequence into simplified URScript — Universal
//  Robots' real robot programming language. This is educational output modeled
//  on real UR syntax; it's intentionally minimal (just `movej` calls) and is not
//  guaranteed to run on hardware without review.
//

import Foundation

enum URScript {
    /// Builds a URScript program that moves the arm joint-by-joint (`movej`)
    /// through each recorded pose. Joint angles are in radians, which is what
    /// URScript's `movej` expects.
    ///
    /// - Parameters:
    ///   - poses: recorded joint angles (radians), one array of 6 per pose.
    ///   - acceleration: joint acceleration (rad/s²) for the `a=` argument.
    ///   - velocity: joint speed (rad/s) for the `v=` argument.
    static func program(poses: [[Double]], acceleration: Double = 1.4, velocity: Double = 1.05) -> String {
        var lines: [String] = []
        lines.append("# Kinematix export — simplified URScript (educational).")
        lines.append("# Review before running on real hardware.")
        lines.append("def kinematix_program():")

        if poses.isEmpty {
            lines.append("  # (no poses recorded)")
        }
        for (i, pose) in poses.enumerated() {
            let joints = pose.map { String(format: "%.5f", $0) }.joined(separator: ", ")
            lines.append("  # pose \(i + 1)")
            lines.append("  movej([\(joints)], a=\(fmt(acceleration)), v=\(fmt(velocity)))")
        }

        lines.append("end")
        lines.append("kinematix_program()")
        return lines.joined(separator: "\n")
    }

    private static func fmt(_ v: Double) -> String { String(format: "%.2f", v) }
}
