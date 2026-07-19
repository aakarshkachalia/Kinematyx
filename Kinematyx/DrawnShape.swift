//
//  DrawnShape.swift
//  Kinematix
//
//  Turns a 2D sketch into a 3D object. The user draws an outline; we extrude that
//  ACTUAL outline into a prism, which becomes a reusable shelf item that spawns
//  physics objects on drag-and-drop.
//
//  The VISUAL mesh follows the real (possibly concave) outline — triangulated by
//  ear clipping — so the 3D object matches what the user drew. Only the COLLIDER
//  is a convex hull (RealityKit convex shapes are physics-friendly); if a scribble
//  is degenerate / self-intersecting and can't be triangulated, we fall back to
//  the hull for the visual too so we always produce a valid solid.
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
    /// Extrudes the 2D outline into a prism lying on the XZ plane, `size` meters
    /// wide and `depth` meters tall. The visual mesh follows the ACTUAL outline
    /// (concave-safe, via ear clipping); the returned corner points feed a convex
    /// collider. Falls back to the convex hull if the outline can't be triangulated.
    static func extrude(_ outline: [SIMD2<Float>], size: Float, depth: Float)
        -> (MeshResource, [SIMD3<Float>]) {
        // Scale into meters and clean up the raw stroke (drop near-duplicate points
        // and a closing point that repeats the start).
        var poly = cleanedPolygon(outline.map { $0 * size })
        if poly.count < 3 { poly = defaultSquare(size: size) }
        // Orient CCW so cap normals and the ear test are consistent.
        if signedArea(poly) < 0 { poly.reverse() }

        // Triangulate the real outline; only fall back to the hull if that fails
        // (e.g. a self-intersecting scribble).
        var capTriangles = earClip(poly)
        if capTriangles == nil {
            var hull = convexHull(poly)
            if hull.count < 3 { hull = defaultSquare(size: size) }
            if signedArea(hull) < 0 { hull.reverse() }
            poly = hull
            capTriangles = fanIndices(hull.count)
        }
        let tris = capTriangles!

        let halfD = depth / 2
        // 3D corners: outline in XZ, duplicated at bottom (−Y) and top (+Y).
        let bottom = poly.map { SIMD3<Float>($0.x, -halfD, $0.y) }
        let top = poly.map { SIMD3<Float>($0.x, halfD, $0.y) }

        var positions: [SIMD3<Float>] = []
        var normals: [SIMD3<Float>] = []
        var indices: [UInt32] = []

        func addTri(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ c: SIMD3<Float>, _ n: SIMD3<Float>) {
            let base = UInt32(positions.count)
            positions += [a, b, c]
            normals += [n, n, n]
            indices += [base, base + 1, base + 2]
        }

        // Top and bottom caps: the ear-clipped triangles (bottom wound the other way).
        var t = 0
        while t < tris.count {
            let a = tris[t], b = tris[t + 1], c = tris[t + 2]
            addTri(top[a], top[b], top[c], SIMD3<Float>(0, 1, 0))
            addTri(bottom[a], bottom[c], bottom[b], SIMD3<Float>(0, -1, 0))
            t += 3
        }
        // Side walls: one quad per outline edge, with an outward normal.
        let m = poly.count
        for i in 0..<m {
            let j = (i + 1) % m
            let edge = poly[j] - poly[i]
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

    // MARK: Polygon helpers (outline cleanup + concave triangulation)

    /// Drops consecutive near-duplicate points and a closing point that repeats the
    /// start, so the ear clipper sees a clean simple polygon.
    static func cleanedPolygon(_ pts: [SIMD2<Float>]) -> [SIMD2<Float>] {
        var out: [SIMD2<Float>] = []
        for p in pts {
            if let last = out.last, simd_length(p - last) < 1e-4 { continue }
            out.append(p)
        }
        if out.count > 1, simd_length(out[0] - out[out.count - 1]) < 1e-4 { out.removeLast() }
        return out
    }

    /// Signed area of a polygon (positive = counter-clockwise).
    static func signedArea(_ p: [SIMD2<Float>]) -> Float {
        var a: Float = 0
        for i in 0..<p.count {
            let j = (i + 1) % p.count
            a += p[i].x * p[j].y - p[j].x * p[i].y
        }
        return a / 2
    }

    /// Ear-clipping triangulation of a simple (CCW) polygon. Returns triangle
    /// vertex indices into `poly`, or nil if it isn't a triangulable simple polygon.
    static func earClip(_ poly: [SIMD2<Float>]) -> [Int]? {
        let n = poly.count
        guard n >= 3 else { return nil }
        if n == 3 { return [0, 1, 2] }

        var remaining = Array(0..<n)
        var result: [Int] = []
        var guardIterations = 0
        let maxIterations = n * n

        while remaining.count > 3 {
            let count = remaining.count
            var clippedEar = false
            for i in 0..<count {
                let ia = remaining[(i + count - 1) % count]
                let ib = remaining[i]
                let ic = remaining[(i + 1) % count]
                let a = poly[ia], b = poly[ib], c = poly[ic]
                // Convex corner? (left turn for a CCW polygon.)
                if triCross(a, b, c) <= 1e-9 { continue }
                // No other vertex inside this candidate ear?
                var containsOther = false
                for j in remaining where j != ia && j != ib && j != ic {
                    if pointInTriangle(poly[j], a, b, c) { containsOther = true; break }
                }
                if containsOther { continue }
                result += [ia, ib, ic]
                remaining.remove(at: i)
                clippedEar = true
                break
            }
            if !clippedEar { return nil }
            guardIterations += 1
            if guardIterations > maxIterations { return nil }
        }
        result += remaining   // final triangle
        return result
    }

    /// Triangle-fan indices for a convex polygon (the hull fallback cap).
    static func fanIndices(_ m: Int) -> [Int] {
        var out: [Int] = []
        for i in 1..<(m - 1) { out += [0, i, i + 1] }
        return out
    }

    /// Twice the signed area of triangle abc (>0 = counter-clockwise / left turn).
    private static func triCross(_ a: SIMD2<Float>, _ b: SIMD2<Float>, _ c: SIMD2<Float>) -> Float {
        (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    }

    /// Whether point `p` lies inside triangle abc (edges inclusive).
    private static func pointInTriangle(_ p: SIMD2<Float>, _ a: SIMD2<Float>, _ b: SIMD2<Float>, _ c: SIMD2<Float>) -> Bool {
        let d1 = triCross(p, a, b)
        let d2 = triCross(p, b, c)
        let d3 = triCross(p, c, a)
        let hasNeg = d1 < 0 || d2 < 0 || d3 < 0
        let hasPos = d1 > 0 || d2 > 0 || d3 > 0
        return !(hasNeg && hasPos)
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
