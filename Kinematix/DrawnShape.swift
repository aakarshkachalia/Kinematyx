//
//  DrawnShape.swift
//  Kinematix
//
//  Turns a 2D sketch into a 3D object. The user draws an outline; we take the
//  convex hull of the strokes and extrude it into a prism, which becomes a
//  reusable shelf item that spawns physics objects on drag-and-drop.
//
//  NOTE: we use the CONVEX HULL of the drawing (not the exact outline). That
//  guarantees a valid, physics-friendly solid from any scribble and gives a
//  clean convex collider; the trade-off is that concave details are filled in.
//

import Foundation
import Observation
import RealityKit
import AppKit
import simd

struct DrawnShape: Identifiable, Sendable {
    let id: String
    /// Outline points, normalized and centered into roughly [-0.5, 0.5].
    let points: [SIMD2<Float>]
    /// Color hue [0, 1] for the spawned object.
    let hue: Double
}

@MainActor
@Observable
final class DrawnShapeStore {
    private(set) var shapes: [DrawnShape] = []

    /// Creates a shape from raw canvas points and appends it to the shelf.
    @discardableResult
    func add(canvasPoints: [CGPoint], canvasSize: CGSize) -> DrawnShape? {
        guard canvasPoints.count >= 3, canvasSize.width > 0, canvasSize.height > 0 else { return nil }
        // Normalize to a centered, square-ish space; flip Y (screen Y grows down).
        let scale = Float(max(canvasSize.width, canvasSize.height))
        let cx = Float(canvasSize.width) / 2, cy = Float(canvasSize.height) / 2
        let normalized = canvasPoints.map {
            SIMD2<Float>((Float($0.x) - cx) / scale, (cy - Float($0.y)) / scale)
        }
        let shape = DrawnShape(id: UUID().uuidString, points: normalized, hue: Double.random(in: 0...1))
        shapes.append(shape)
        return shape
    }

    func shape(id: String) -> DrawnShape? { shapes.first { $0.id == id } }
}

/// Geometry helpers: convex hull + extrusion into a 3D prism mesh + collider.
enum DrawnGeometry {
    /// Extrudes the (hull of the) 2D outline into a prism lying on the XZ plane,
    /// `size` meters wide and `depth` meters tall. Returns the visual mesh and
    /// the corner points for a convex collider.
    static func extrude(_ outline: [SIMD2<Float>], size: Float, depth: Float)
        -> (MeshResource, [SIMD3<Float>]) {
        // Scale into meters and take the convex hull.
        let scaled = outline.map { $0 * size }
        var hull = convexHull(scaled)
        if hull.count < 3 { hull = defaultSquare(size: size) }

        let halfD = depth / 2
        // 3D corners: hull in XZ, duplicated at bottom (−Y) and top (+Y).
        let bottom = hull.map { SIMD3<Float>($0.x, -halfD, $0.y) }
        let top = hull.map { SIMD3<Float>($0.x, halfD, $0.y) }

        var positions: [SIMD3<Float>] = []
        var normals: [SIMD3<Float>] = []
        var indices: [UInt32] = []

        func addTri(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ c: SIMD3<Float>, _ n: SIMD3<Float>) {
            let base = UInt32(positions.count)
            positions += [a, b, c]
            normals += [n, n, n]
            indices += [base, base + 1, base + 2]
        }

        let m = hull.count
        // Top and bottom caps: triangle fans (valid because the hull is convex).
        for i in 1..<(m - 1) {
            addTri(top[0], top[i], top[i + 1], SIMD3<Float>(0, 1, 0))
            addTri(bottom[0], bottom[i + 1], bottom[i], SIMD3<Float>(0, -1, 0))
        }
        // Side walls: one quad per hull edge, with an outward normal.
        for i in 0..<m {
            let j = (i + 1) % m
            let edge = hull[j] - hull[i]
            let outward = simd_normalize(SIMD3<Float>(edge.y, 0, -edge.x))   // perp, in XZ
            addTri(bottom[i], bottom[j], top[j], outward)
            addTri(bottom[i], top[j], top[i], outward)
        }

        var descriptor = MeshDescriptor(name: "drawn")
        descriptor.positions = MeshBuffers.Positions(positions)
        descriptor.normals = MeshBuffers.Normals(normals)
        descriptor.primitives = .triangles(indices)

        let mesh = (try? MeshResource.generate(from: [descriptor]))
            ?? MeshResource.generateBox(size: SIMD3<Float>(size, depth, size))
        return (mesh, bottom + top)
    }

    /// 2D convex hull (Andrew's monotone chain), returned counter-clockwise.
    static func convexHull(_ input: [SIMD2<Float>]) -> [SIMD2<Float>] {
        // De-duplicate near-identical points, then sort by x (then y).
        let unique: [Point] = Array(Set(input.map { Point($0) }))
        var points: [SIMD2<Float>] = unique.map { $0.v }
        points.sort { a, b in
            a.x == b.x ? a.y < b.y : a.x < b.x
        }
        guard points.count >= 3 else { return points }

        func cross(_ o: SIMD2<Float>, _ a: SIMD2<Float>, _ b: SIMD2<Float>) -> Float {
            (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
        }

        var lower: [SIMD2<Float>] = []
        for p in points {
            while lower.count >= 2, cross(lower[lower.count - 2], lower[lower.count - 1], p) <= 0 {
                lower.removeLast()
            }
            lower.append(p)
        }
        var upper: [SIMD2<Float>] = []
        for p in points.reversed() {
            while upper.count >= 2, cross(upper[upper.count - 2], upper[upper.count - 1], p) <= 0 {
                upper.removeLast()
            }
            upper.append(p)
        }
        lower.removeLast(); upper.removeLast()
        return lower + upper
    }

    private static func defaultSquare(size: Float) -> [SIMD2<Float>] {
        let h = size / 2
        return [SIMD2(-h, -h), SIMD2(h, -h), SIMD2(h, h), SIMD2(-h, h)]
    }

    /// Hashable wrapper so we can de-duplicate near-identical points.
    private struct Point: Hashable {
        let x: Int, y: Int
        let v: SIMD2<Float>
        init(_ v: SIMD2<Float>) { self.v = v; x = Int(v.x * 10000); y = Int(v.y * 10000) }
        static func == (a: Point, b: Point) -> Bool { a.x == b.x && a.y == b.y }
        func hash(into h: inout Hasher) { h.combine(x); h.combine(y) }
    }
}
