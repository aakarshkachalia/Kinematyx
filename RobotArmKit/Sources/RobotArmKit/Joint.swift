//
//  Joint.swift
//  RobotArmKit
//
//  A single revolute joint: its fixed DH geometry plus how far it is allowed
//  to rotate. Real robots can't spin their joints infinitely — cables, motor
//  ranges, and mechanical stops limit each joint to a min/max angle. Tracking
//  those limits here lets us stop the UI (and later, motion planning) from
//  commanding a pose the real arm could never reach.

import Foundation

/// A revolute joint = its DH geometry + its rotation limits (in radians).
public struct Joint: Equatable, Sendable {
    /// The fixed Denavit–Hartenberg geometry for this joint.
    public let dh: DHParameter
    /// Smallest allowed joint angle, in radians (inclusive).
    public let minAngle: Double
    /// Largest allowed joint angle, in radians (inclusive).
    public let maxAngle: Double

    public init(dh: DHParameter, minAngle: Double, maxAngle: Double) {
        self.dh = dh
        self.minAngle = minAngle
        self.maxAngle = maxAngle
    }

    /// Whether a commanded angle (radians) is inside this joint's limits.
    public func allows(_ angle: Double) -> Bool {
        angle >= minAngle && angle <= maxAngle
    }

    /// Clamps a commanded angle (radians) to this joint's limits.
    ///
    /// Handy for the UI: a slider can pin a value to the legal range instead of
    /// letting the user drag past a mechanical stop.
    public func clamp(_ angle: Double) -> Double {
        min(max(angle, minAngle), maxAngle)
    }
}
