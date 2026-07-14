//
//  SandboxScene.swift
//  Kinematix
//
//  Owns and builds the RealityKit world: floor + grid, lighting + shadows, the
//  robot arm (rendered from RobotArmKit's forward-kinematics transforms), and
//  the physics objects the user drops in. It never does kinematics itself.
//
//  Plain reference type (not @Observable): it holds entity references so the
//  RealityView's `update` closure can reposition things every frame.
//

import Foundation
import SwiftUI
import RealityKit
import RobotArmKit
import AppKit
import simd

@MainActor
final class SandboxScene {

    // MARK: Tunable constants (everything in REAL meters — no fake scaling)

    /// Side length of the plain floor, big enough to feel open.
    private let floorSize: Float = 200
    /// How far out (meters) the crisp 1 m grid extends from the origin.
    private let gridExtent: Int = 12
    /// Radius of the cylinders representing the links.
    private let linkRadius: Float = 0.028
    /// Radius / half-height of the compact joint housings.
    private let housingRadius: Float = 0.045
    private let housingHalfHeight: Float = 0.055

    // MARK: Entities kept for per-frame updates

    /// Root the arm hangs off of. Rotated once to convert robot Z-up → RK Y-up.
    private let armRoot = Entity()
    private let camera = PerspectiveCamera()

    private var linkEntities: [ModelEntity] = []
    private var housingEntities: [ModelEntity] = []

    /// The end-effector gripper. Off-axis prongs make joint-6 roll visible.
    private let toolEntity = Entity()

    /// Parent for every dropped object, so "clear all" is trivial and we can add
    /// objects to the live scene without the RealityView `content` handle.
    private let objectsRoot = Entity()

    private let arm = RobotArm.ur5
    private var didBuild = false

    // MARK: - Build

    func build(into content: inout RealityViewCameraContent) async {
        guard !didBuild else { return }
        didBuild = true

        content.camera = .virtual

        addFloorAndGrid(to: &content)
        addLighting(to: &content)
        addArm(to: &content)
        addCamera(to: &content)
        content.add(objectsRoot)

        updateArm(with: [Double](repeating: 0, count: arm.degreesOfFreedom))
    }

    // MARK: Floor + grid

    private func addFloorAndGrid(to content: inout RealityViewCameraContent) {
        // Warm off-white matte floor. A thin box (not a plane) so it has depth
        // for the static physics body and casts/receives shadow cleanly.
        let thickness: Float = 0.04
        var floorMat = PhysicallyBasedMaterial()
        floorMat.baseColor = .init(tint: NSColor(red: 0.93, green: 0.92, blue: 0.89, alpha: 1))
        floorMat.roughness = 0.95
        floorMat.metallic = 0.0

        let floor = ModelEntity(
            mesh: .generateBox(width: floorSize, height: thickness, depth: floorSize),
            materials: [floorMat]
        )
        floor.position = SIMD3<Float>(0, -thickness / 2, 0)

        // Static physics body so dropped objects land instead of falling through.
        let floorShape = ShapeResource.generateBox(
            width: floorSize, height: thickness, depth: floorSize
        )
        floor.components.set(CollisionComponent(shapes: [floorShape]))
        floor.components.set(PhysicsBodyComponent(
            shapes: [floorShape], mass: 0,
            material: Self.objectPhysicsMaterial, mode: .static
        ))
        content.add(floor)

        content.add(makeGrid())
    }

    /// Builds the grid out of REAL thin-box geometry (one box per line) rather
    /// than a texture. Geometry stays pixel-crisp at every camera distance — the
    /// old texture blurred into mush because it was minified across a huge plane.
    private func makeGrid() -> Entity {
        let grid = Entity()
        let extent = Float(gridExtent)
        let lineColor = NSColor(red: 0.72, green: 0.71, blue: 0.68, alpha: 1)
        let axisColor = NSColor(red: 0.55, green: 0.54, blue: 0.50, alpha: 1)

        var lineMat = PhysicallyBasedMaterial()
        lineMat.baseColor = .init(tint: lineColor)
        lineMat.roughness = 1.0
        var axisMat = PhysicallyBasedMaterial()
        axisMat.baseColor = .init(tint: axisColor)
        axisMat.roughness = 1.0

        let thin: Float = 0.008     // line width
        let y: Float = 0.002        // sit just above the floor to avoid z-fighting
        let length = extent * 2

        for i in -gridExtent...gridExtent {
            let coord = Float(i)
            let mat = (i == 0) ? axisMat : lineMat   // axes a touch darker

            // Line running along X (varies in x, fixed z).
            let xLine = ModelEntity(
                mesh: .generateBox(width: length, height: 0.004, depth: thin),
                materials: [mat]
            )
            xLine.position = SIMD3<Float>(0, y, coord)
            grid.addChild(xLine)

            // Line running along Z (fixed x, varies in z).
            let zLine = ModelEntity(
                mesh: .generateBox(width: thin, height: 0.004, depth: length),
                materials: [mat]
            )
            zLine.position = SIMD3<Float>(coord, y, 0)
            grid.addChild(zLine)
        }
        return grid
    }

    // MARK: Lighting (three-point + shadows)

    private func addLighting(to content: inout RealityViewCameraContent) {
        // KEY: the main light, bright, casts the contact shadow onto the floor.
        let key = Entity()
        var keyLight = DirectionalLightComponent(color: .white, intensity: 3200)
        keyLight.isRealWorldProxy = false
        key.components.set(keyLight)
        key.components.set(DirectionalLightComponent.Shadow())
        key.look(at: .zero, from: SIMD3<Float>(1.5, 2.5, 1.8), relativeTo: nil)
        content.add(key)

        // FILL: softens shadows from the opposite side (fakes bounce light).
        let fill = Entity()
        fill.components.set(DirectionalLightComponent(color: .white, intensity: 900))
        fill.look(at: .zero, from: SIMD3<Float>(-2.5, 1.5, -1.5), relativeTo: nil)
        content.add(fill)

        // RIM: from behind, to separate the arm's silhouette from the floor.
        let rim = Entity()
        rim.components.set(DirectionalLightComponent(color: .white, intensity: 700))
        rim.look(at: .zero, from: SIMD3<Float>(0, 1.2, -3), relativeTo: nil)
        content.add(rim)
    }

    // MARK: Arm

    private func addArm(to content: inout RealityViewCameraContent) {
        armRoot.orientation = CoordinateSpace.robotToRealityKit
        content.add(armRoot)

        // Charcoal base plinth.
        let base = ModelEntity(
            mesh: .generateCylinder(height: 0.05, radius: 0.09),
            materials: [Self.charcoalMaterial]
        )
        base.position = SIMD3<Float>(0, 0.025, 0)
        content.add(base)

        // Links (brushed aluminium) + compact joint housings (charcoal).
        for _ in 0..<arm.degreesOfFreedom {
            let link = ModelEntity(
                mesh: .generateCylinder(height: 1, radius: linkRadius),   // unit, stretched later
                materials: [Self.aluminiumMaterial]
            )
            linkEntities.append(link)
            armRoot.addChild(link)

            let housing = ModelEntity(
                mesh: .generateCylinder(height: housingHalfHeight * 2, radius: housingRadius),
                materials: [Self.charcoalMaterial]
            )
            housingEntities.append(housing)
            armRoot.addChild(housing)
        }

        addTool()
    }

    private func addTool() {
        // Wrist flange (steel) + two off-axis prongs so joint-6 roll is visible.
        let flange = ModelEntity(
            mesh: .generateCylinder(height: 0.03, radius: 0.05),
            materials: [Self.steelMaterial]
        )
        flange.orientation = simd_quatf(angle: .pi / 2, axis: SIMD3<Float>(1, 0, 0)) // face +Z
        toolEntity.addChild(flange)

        for side in [Float(1), Float(-1)] {
            let prong = ModelEntity(
                mesh: .generateBox(width: 0.018, height: 0.04, depth: 0.09),
                materials: [Self.steelMaterial]
            )
            prong.position = SIMD3<Float>(0.035 * side, 0, 0.06)
            toolEntity.addChild(prong)
        }

        // Kinematic collision body (gripper-sized): driven by FK, pushes dynamic
        // objects it overlaps, but is itself unaffected by gravity/impacts.
        let gripShape = ShapeResource.generateSphere(radius: 0.08)
        toolEntity.components.set(CollisionComponent(shapes: [gripShape]))
        toolEntity.components.set(PhysicsBodyComponent(
            shapes: [gripShape], mass: 1,
            material: Self.objectPhysicsMaterial, mode: .kinematic
        ))

        armRoot.addChild(toolEntity)
    }

    // MARK: Camera

    private func addCamera(to content: inout RealityViewCameraContent) {
        camera.camera.fieldOfViewInDegrees = CameraRig.verticalFOVDegrees
        content.add(camera)
    }

    func updateCamera(position: SIMD3<Float>, focus: SIMD3<Float>) {
        camera.look(at: focus, from: position, relativeTo: nil)
    }

    // MARK: - Per-frame arm update

    func updateArm(with jointAngles: [Double]) {
        guard jointAngles.count == arm.degreesOfFreedom else { return }

        let frames = arm.forwardKinematics(jointAngles: jointAngles)

        // Joint origins in robot space: index 0 = base, rest = each frame origin.
        var points: [SIMD3<Float>] = [SIMD3<Float>(0, 0, 0)]
        points.append(contentsOf: frames.map { $0.translation })

        // The frame BEFORE each joint gives that joint's rotation axis (its z).
        var priorFrames: [simd_double4x4] = [matrix_identity_double4x4]
        priorFrames.append(contentsOf: frames.dropLast())

        for i in 0..<arm.degreesOfFreedom {
            // Link spans this joint origin to the next.
            linkEntities[i].stretchBetween(points[i], points[i + 1])

            // Compact housing sits at the joint, aligned with its rotation axis.
            let axis = SIMD3<Float>(
                Float(priorFrames[i].columns.2.x),
                Float(priorFrames[i].columns.2.y),
                Float(priorFrames[i].columns.2.z)
            )
            housingEntities[i].position = points[i]
            housingEntities[i].orientation = Self.rotation(from: SIMD3<Float>(0, 1, 0), to: axis)
        }

        // Gripper follows the full final transform (position + orientation).
        if let toolFrame = frames.last {
            toolEntity.transform = Transform(matrix: simd_float4x4(toolFrame))
        }
    }

    // MARK: - Dropped objects (physics)

    private static let objectPhysicsMaterial = PhysicsMaterialResource.generate(
        friction: 0.6, restitution: 0.1
    )

    @discardableResult
    func spawnObject(_ kind: ObjectKind, at position: SIMD3<Float>) -> ModelEntity {
        let entity = makeObject(kind)
        entity.position = position
        objectsRoot.addChild(entity)
        return entity
    }

    func clearObjects() {
        for child in objectsRoot.children.map({ $0 }) {
            child.removeFromParent()
        }
    }

    /// Every dropped object currently in the world.
    var droppedObjects: [Entity] { objectsRoot.children.map { $0 } }

    private func makeObject(_ kind: ObjectKind) -> ModelEntity {
        var material = PhysicallyBasedMaterial()
        material.baseColor = .init(tint: Self.objectColor(kind))
        material.roughness = 0.5
        material.metallic = 0.0

        let mesh: MeshResource
        let shape: ShapeResource
        let mass: Float

        switch kind {
        case .cube:
            let side: Float = 0.12
            mesh = .generateBox(size: side, cornerRadius: 0.006)
            shape = .generateBox(size: SIMD3<Float>(repeating: side))
            mass = 0.5
        case .sphere:
            let r: Float = 0.07
            mesh = .generateSphere(radius: r)
            shape = .generateSphere(radius: r)
            mass = 0.4
        case .ramp:
            let (wedgeMesh, corners) = Self.makeWedgeMesh(width: 0.18, depth: 0.18, height: 0.1)
            mesh = wedgeMesh
            shape = .generateConvex(from: corners)
            mass = 0.7
        }

        let entity = ModelEntity(mesh: mesh, materials: [material])
        entity.name = "object.\(kind.rawValue)"
        entity.components.set(CollisionComponent(shapes: [shape]))
        entity.components.set(PhysicsBodyComponent(
            shapes: [shape], mass: mass,
            material: Self.objectPhysicsMaterial, mode: .dynamic
        ))
        return entity
    }

    private static func objectColor(_ kind: ObjectKind) -> NSColor {
        switch kind {
        case .cube:   return NSColor(red: 0.86, green: 0.42, blue: 0.34, alpha: 1)
        case .sphere: return NSColor(red: 0.30, green: 0.55, blue: 0.80, alpha: 1)
        case .ramp:   return NSColor(red: 0.52, green: 0.62, blue: 0.38, alpha: 1)
        }
    }

    // MARK: - Materials (PBR)

    private static let aluminiumMaterial: PhysicallyBasedMaterial = {
        var m = PhysicallyBasedMaterial()
        m.baseColor = .init(tint: NSColor(white: 0.82, alpha: 1))
        m.roughness = 0.35
        m.metallic = 0.9
        return m
    }()

    private static let charcoalMaterial: PhysicallyBasedMaterial = {
        var m = PhysicallyBasedMaterial()
        m.baseColor = .init(tint: NSColor(white: 0.13, alpha: 1))
        m.roughness = 0.55
        m.metallic = 0.4
        return m
    }()

    private static let steelMaterial: PhysicallyBasedMaterial = {
        var m = PhysicallyBasedMaterial()
        m.baseColor = .init(tint: NSColor(white: 0.55, alpha: 1))
        m.roughness = 0.3
        m.metallic = 0.85
        return m
    }()

    // MARK: - Geometry helpers

    /// Shortest rotation that turns unit vector `from` into unit vector `to`,
    /// with a safe fallback when they're nearly antiparallel.
    private static func rotation(from: SIMD3<Float>, to: SIMD3<Float>) -> simd_quatf {
        let dot = simd_dot(from, to)
        if dot < -0.9999 {
            return simd_quatf(angle: .pi, axis: SIMD3<Float>(1, 0, 0))
        }
        return simd_quatf(from: from, to: to)
    }

    /// Triangular-prism "wedge" mesh (a ramp) + its 6 corners (for the collider).
    /// Base is a width×depth rectangle on y=0; full height `h` along the back edge
    /// (z=-depth/2), sloping to zero at the front (z=+depth/2). Each face gets its
    /// own vertices and a flat outward normal so the ramp shades crisply.
    private static func makeWedgeMesh(
        width w: Float, depth d: Float, height h: Float
    ) -> (MeshResource, [SIMD3<Float>]) {
        let hw = w / 2, hd = d / 2
        let p0 = SIMD3<Float>(-hw, 0, -hd)
        let p1 = SIMD3<Float>( hw, 0, -hd)
        let p2 = SIMD3<Float>( hw, 0,  hd)
        let p3 = SIMD3<Float>(-hw, 0,  hd)
        let p4 = SIMD3<Float>(-hw, h, -hd)
        let p5 = SIMD3<Float>( hw, h, -hd)

        var positions: [SIMD3<Float>] = []
        var normals: [SIMD3<Float>] = []
        var indices: [UInt32] = []

        func addTri(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ c: SIMD3<Float>, _ n: SIMD3<Float>) {
            let base = UInt32(positions.count)
            positions.append(contentsOf: [a, b, c])
            normals.append(contentsOf: [n, n, n])
            indices.append(contentsOf: [base, base + 1, base + 2])
        }
        func addQuad(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ c: SIMD3<Float>, _ e: SIMD3<Float>, _ n: SIMD3<Float>) {
            addTri(a, b, c, n); addTri(a, c, e, n)
        }

        let slopeN = simd_normalize(SIMD3<Float>(0, d, h))
        addQuad(p0, p1, p2, p3, SIMD3<Float>(0, -1, 0))   // bottom
        addQuad(p0, p4, p5, p1, SIMD3<Float>(0, 0, -1))   // back
        addQuad(p3, p2, p5, p4, slopeN)                   // slope
        addTri(p0, p3, p4, SIMD3<Float>(-1, 0, 0))        // left
        addTri(p1, p5, p2, SIMD3<Float>(1, 0, 0))         // right

        var descriptor = MeshDescriptor(name: "wedge")
        descriptor.positions = MeshBuffers.Positions(positions)
        descriptor.normals = MeshBuffers.Normals(normals)
        descriptor.primitives = .triangles(indices)

        let mesh = (try? MeshResource.generate(from: [descriptor]))
            ?? MeshResource.generateBox(size: SIMD3<Float>(w, h, d))
        return (mesh, [p0, p1, p2, p3, p4, p5])
    }
}
