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
    /// The camera's vertical field of view, in degrees. Must match the value we
    /// set on the RealityKit PerspectiveCamera so our ray math lines up with what
    /// the user actually sees.
    static let verticalFOVDegrees: Float = 60

    /// Angle around the vertical (Y) axis, in radians. Changed by horizontal drag.
    var azimuth: Float = 0.85
    /// Angle above the horizon, in radians. Changed by vertical drag.
    var elevation: Float = 0.42
    /// Distance from the focus point, in meters. Changed by scroll / pinch.
    /// Frames the bench-mounted arm with table, floor grid, and the human
    /// reference visible for scale.
    var radius: Float = 4.0

    /// The point the camera looks at and orbits around. Panning shifts this.
    /// Sits around the arm's mid-height above the ~0.75 m bench.
    var focus: SIMD3<Float> = SIMD3<Float>(0, 1.3, 0)

    /// When true, the viewport ignores the orbit position and mounts the camera
    /// on the gripper (first-person). Any orbit input turns this back off.
    var followGripper = false

    // Keep the camera comfortably above ground and never flipped over the poles.
    private let minElevation: Float = 0.05
    private let maxElevation: Float = 1.45   // ~83°
    private let minRadius: Float = 0.6
    private let maxRadius: Float = 30

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
        followGripper = false   // taking manual control leaves the gripper cam
        let sensitivity: Float = 0.008
        azimuth -= deltaX * sensitivity
        elevation += deltaY * sensitivity
        elevation = min(max(elevation, minElevation), maxElevation)
    }

    // MARK: - Presets

    enum Preset { case orbit, top, side, front, gripper }

    func apply(_ preset: Preset) {
        switch preset {
        case .orbit:   set(0.85, 0.42, 4.0, [0, 1.3, 0])
        case .top:     set(0.0, maxElevation, 4.2, [0, benchHeightGuess, 0])
        case .side:    set(.pi / 2, 0.15, 4.0, [0, 1.3, 0])
        case .front:   set(0.0, 0.15, 4.0, [0, 1.3, 0])
        case .gripper: followGripper = true; return
        }
        followGripper = false
    }

    private func set(_ az: Float, _ el: Float, _ r: Float, _ f: SIMD3<Float>) {
        azimuth = az; elevation = el; radius = r; focus = f
    }

    /// Approximate bench-top height used for the top-down focus point.
    private let benchHeightGuess: Float = 0.75

    /// Zoom by a scroll/pinch amount. Positive = zoom in (closer).
    func zoom(by amount: Float) {
        // Scale multiplicatively so zooming feels even at any distance.
        radius *= (1 - amount)
        radius = min(max(radius, minRadius), maxRadius)
    }

    // MARK: - Picking (screen point → world position on the ground)

    /// Converts a point in the viewport (SwiftUI coords, origin top-left) into
    /// the world position where the ray through that pixel hits the ground plane
    /// (y = 0). Used to decide where a dragged object should be dropped.
    ///
    /// HOW (pinhole camera unprojection):
    ///   1. Turn the pixel into normalized device coordinates in [-1, 1].
    ///   2. Build the camera's basis: forward (toward focus), right, and up.
    ///   3. Form the ray direction by nudging `forward` by the pixel's offset,
    ///      scaled by tan(fov/2) — that's how far the frustum spreads per unit.
    ///   4. Intersect that ray with the ground plane y = 0.
    func groundHitPoint(viewSize: CGSize, point: CGPoint) -> SIMD3<Float>? {
        guard let (origin, dir) = ray(viewSize: viewSize, point: point) else { return nil }
        // Only hits the floor if the ray points downward.
        guard dir.y < -1e-4 else { return nil }
        let t = -origin.y / dir.y
        guard t > 0 else { return nil }
        return origin + dir * t
    }

    /// Builds the world-space ray (origin + direction) through a viewport pixel.
    /// This is the shared unprojection used both for dropping objects on the
    /// floor and for click-picking objects in the scene.
    func ray(viewSize: CGSize, point: CGPoint) -> (origin: SIMD3<Float>, direction: SIMD3<Float>)? {
        let w = Float(viewSize.width)
        let h = Float(viewSize.height)
        guard w > 0, h > 0 else { return nil }

        // Pixel → normalized device coords. Flip Y because screen Y grows down.
        let ndcX = Float(point.x) / w * 2 - 1
        let ndcY = 1 - Float(point.y) / h * 2

        let origin = position
        let forward = simd_normalize(focus - origin)
        let worldUp = SIMD3<Float>(0, 1, 0)
        let right = simd_normalize(simd_cross(forward, worldUp))
        let up = simd_cross(right, forward)

        let tanY = tan(Self.verticalFOVDegrees * .pi / 180 / 2)
        let aspect = w / h
        let dir = simd_normalize(
            forward + ndcX * tanY * aspect * right + ndcY * tanY * up
        )
        return (origin, dir)
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
