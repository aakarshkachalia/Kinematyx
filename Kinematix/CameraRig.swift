//
//  CameraRig.swift
//  Kinematix
//
//  A hand-rolled orbit camera. Instead of relying on RealityKit's built-in
//  camera controls (which manage their own camera and gave us no control over
//  the starting position or zoom), we track the camera in spherical coordinates
//  around a focus point and compute its position ourselves each frame.
//
//  SPHERICAL CAMERA MODEL (why this is the standard way to orbit):
//  A point on a sphere is fully described by two angles and a distance:
//    * azimuth   – how far around the vertical axis we've spun (left/right drag)
//    * elevation – how high above the horizon we are (up/down drag)
//    * radius    – how far from the focus point we are (zoom)
//  Converting those to an (x, y, z) position lets the camera "orbit" the arm
//  while always looking at the same focus point.
//

import Foundation
import Observation
import simd

@MainActor
@Observable
final class CameraRig {
    /// Angle around the vertical (Y) axis, in radians. Changed by horizontal drag.
    var azimuth: Float = 0.9
    /// Angle above the horizon, in radians. Changed by vertical drag.
    var elevation: Float = 0.55
    /// Distance from the focus point, in meters. Changed by scroll / pinch.
    var radius: Float = 3.4

    /// The point the camera looks at and orbits around. Panning shifts this.
    var focus: SIMD3<Float> = SIMD3<Float>(0, 0.35, 0)

    // Keep the camera comfortably above ground and never flipped over the poles.
    private let minElevation: Float = 0.05
    private let maxElevation: Float = 1.45   // ~83°
    private let minRadius: Float = 0.8
    private let maxRadius: Float = 15

    /// The camera's world-space position, derived from the spherical coordinates.
    var position: SIMD3<Float> {
        let ce = cos(elevation)
        let se = sin(elevation)
        let sa = sin(azimuth)
        let ca = cos(azimuth)
        // Direction from focus out to the camera.
        let direction = SIMD3<Float>(ce * sa, se, ce * ca)
        return focus + direction * radius
    }

    // MARK: - Input handlers (called by the event-catching view)

    /// Orbit by a drag delta in points. Horizontal spins, vertical tilts.
    func orbit(deltaX: Float, deltaY: Float) {
        let sensitivity: Float = 0.008
        azimuth -= deltaX * sensitivity
        elevation += deltaY * sensitivity
        elevation = min(max(elevation, minElevation), maxElevation)
    }

    /// Zoom by a scroll/pinch amount. Positive = zoom in (closer).
    func zoom(by amount: Float) {
        // Scale multiplicatively so zooming feels even at any distance.
        radius *= (1 - amount)
        radius = min(max(radius, minRadius), maxRadius)
    }

    /// Pan the focus point in the camera's own left/right and up plane.
    func pan(deltaX: Float, deltaY: Float) {
        // Screen-space right and up vectors for the current azimuth. Right lies
        // in the ground plane; up is world up. Pan speed scales with distance so
        // it feels consistent whether zoomed in or out.
        let ca = cos(azimuth)
        let sa = sin(azimuth)
        let right = SIMD3<Float>(ca, 0, -sa)
        let up = SIMD3<Float>(0, 1, 0)
        let speed = radius * 0.0015
        focus += (-deltaX * right + deltaY * up) * speed
    }
}
