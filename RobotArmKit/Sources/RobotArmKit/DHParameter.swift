//
//  DHParameter.swift
//  RobotArmKit
//
//  One row of a Denavit–Hartenberg (DH) table, plus the math that turns that
//  row into a 4x4 transformation matrix.
//
//  WHAT IS A DH PARAMETER?
//  -----------------------
//  A robot arm is a chain of rigid links connected by joints. To do any math
//  with it, we need a precise way to describe how each link is positioned and
//  oriented relative to the previous one. The Denavit–Hartenberg convention is
//  the standard, agreed-upon way to do this using just FOUR numbers per joint:
//
//    * alpha (α) – "link twist":  rotation about the X axis
//    * a           – "link length": translation along the X axis
//    * d           – "link offset": translation along the Z axis
//    * thetaOffset – a constant added to the joint's variable angle θ
//
//  Three of these (alpha, a, d) are fixed by the robot's physical geometry.
//  The fourth, θ, is the thing that actually MOVES for a rotary (revolute)
//  joint — it's the value you change when you drive a motor. `thetaOffset`
//  lets us bake in a constant offset so that "θ = 0" lines up with the robot's
//  real-world home position.

import Foundation
import simd

/// One row of a Denavit–Hartenberg table describing a single joint.
///
/// All angles are in **radians** and all lengths are in **meters** (SI units),
/// which is what every robotics reference and the UR5 datasheet uses.
public struct DHParameter: Equatable, Sendable {
    /// Link twist α: rotation about the (new) X axis, in radians.
    public let alpha: Double
    /// Link length a: translation along the (new) X axis, in meters.
    public let a: Double
    /// Link offset d: translation along the Z axis, in meters.
    public let d: Double
    /// A constant added to the variable joint angle θ, in radians.
    /// This aligns "θ = 0" with the robot's documented home pose.
    public let thetaOffset: Double

    public init(alpha: Double, a: Double, d: Double, thetaOffset: Double = 0) {
        self.alpha = alpha
        self.a = a
        self.d = d
        self.thetaOffset = thetaOffset
    }

    /// Builds the 4x4 homogeneous transform for this joint at a given angle.
    ///
    /// - Parameter theta: the *variable* joint angle in radians (the motor
    ///   command). The stored `thetaOffset` is added on top of it.
    /// - Returns: a `simd_double4x4` that converts a point expressed in this
    ///   joint's frame into the *previous* joint's frame.
    ///
    /// WHY THIS SPECIFIC SEQUENCE OF OPERATIONS?
    /// -----------------------------------------
    /// The standard ("distal") DH convention defines the transform between two
    /// consecutive joint frames as exactly four elementary motions, applied in
    /// this fixed order:
    ///
    ///     A_i = Rz(θ) · Tz(d) · Tx(a) · Rx(α)
    ///
    ///   1. Rz(θ) – rotate about Z so this joint's X axis lines up correctly.
    ///              (This is the part that changes as the joint moves.)
    ///   2. Tz(d) – slide along Z to reach the shared normal between the axes.
    ///   3. Tx(a) – slide along X by the link length to the next joint's axis.
    ///   4. Rx(α) – rotate about X so Z lines up with the next joint's axis.
    ///
    /// The ORDER matters: matrix multiplication is not commutative, and this is
    /// the order that the DH convention (and the UR5 datasheet) assumes. If you
    /// multiply these four out by hand you get the closed-form matrix below,
    /// which is what we build directly to avoid four separate multiplications.
    public func transform(theta: Double) -> simd_double4x4 {
        // Total rotation about Z for this joint = commanded angle + fixed offset.
        let t = theta + thetaOffset

        // Precompute the trig terms once for readability and speed.
        let ct = cos(t)
        let st = sin(t)
        let ca = cos(alpha)
        let sa = sin(alpha)

        // Closed-form result of Rz(θ)·Tz(d)·Tx(a)·Rx(α).
        //
        // Column-major note: simd stores matrices by COLUMN, so
        // `simd_double4x4(columns:)` takes four *columns*, not four rows.
        // The math below is written as columns; read across the four SIMD4s
        // to reconstruct each row if you're checking against a textbook, which
        // lays the same matrix out as rows:
        //
        //   [ cosθ   -sinθ·cosα    sinθ·sinα    a·cosθ ]
        //   [ sinθ    cosθ·cosα   -cosθ·sinα    a·sinθ ]
        //   [ 0       sinα         cosα         d      ]
        //   [ 0       0            0            1      ]
        //
        let column0 = simd_double4(ct, st, 0, 0)
        let column1 = simd_double4(-st * ca, ct * ca, sa, 0)
        let column2 = simd_double4(st * sa, -ct * sa, ca, 0)
        let column3 = simd_double4(a * ct, a * st, d, 1)

        return simd_double4x4(columns: (column0, column1, column2, column3))
    }
}
