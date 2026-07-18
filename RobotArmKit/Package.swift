// swift-tools-version: 6.0
//
//  Package.swift
//  RobotArmKit
//
//  Pure-Swift kinematics core for the Kinematix robot-arm simulator.
//
//  This package deliberately has NO dependency on SwiftUI, RealityKit, or
//  AppKit. Everything here is plain Swift + simd, which means:
//    * it can be unit-tested from the command line with `swift test`
//    * it can be shared unchanged by a future iOS (or visionOS) target
//
//  Keeping the math separate from any UI is the whole point of a package:
//  the "brains" of the robot don't care how (or whether) they are drawn.

import PackageDescription

let package = Package(
    name: "RobotArmKit",
    // Platforms are listed so the package can be reused by an iOS app later.
    // These are only *minimums*; the macOS app can deploy to a newer OS.
    // Must cover every platform the app target supports; otherwise, building for
    // a platform the package doesn't declare (e.g. visionOS) fails with
    // "Missing package product 'RobotArmKit'".
    platforms: [
        .macOS(.v13),
        .iOS(.v16),
        .visionOS(.v1),
    ],
    products: [
        // A single library other targets (the Kinematix app) can import.
        .library(
            name: "RobotArmKit",
            targets: ["RobotArmKit"]
        ),
    ],
    targets: [
        // The library itself. No external dependencies.
        .target(
            name: "RobotArmKit"
        ),
        // Unit tests, run via `swift test`.
        .testTarget(
            name: "RobotArmKitTests",
            dependencies: ["RobotArmKit"]
        ),
    ]
)
