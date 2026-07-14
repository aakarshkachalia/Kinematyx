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

    /// Height of the workbench the UR5 is bolted to. Mounting the arm on a bench
    /// (like a real robot cell) keeps its whole reach ABOVE the floor, so the arm
    /// no longer clips through the ground in its normal working poses.
    private let benchHeight: Float = 0.75
    /// World-space offset of the arm's base (top of the bench).
    private var armBaseOffset: SIMD3<Float> { SIMD3<Float>(0, benchHeight, 0) }

    /// Uniform scale applied to the whole arm, so different-reach arms (UR5/UR3)
    /// appear a similar on-screen size. The IK coordinate conversion divides by
    /// it, so the solver still works in the arm's own (real-meter) units.
    private var armScale: Float = 2.0

    // MARK: Entities kept for per-frame updates

    /// Root the arm hangs off of. Rotated once to convert robot Z-up → RK Y-up.
    private let armRoot = Entity()
    private let camera = PerspectiveCamera()

    private var linkEntities: [ModelEntity] = []
    private var housingEntities: [ModelEntity] = []

    /// The end-effector gripper root (follows the FK tool frame).
    private let toolEntity = Entity()
    /// The two gripper fingers, which translate apart/together in X.
    private let leftFinger = ModelEntity()
    private let rightFinger = ModelEntity()
    /// Finger X-offset when fully open / fully closed (meters from center).
    private let fingerOpenX: Float = 0.06
    private let fingerClosedX: Float = 0.026

    /// Parent for every dropped object, so "clear all" is trivial and we can add
    /// objects to the live scene without the RealityView `content` handle.
    private let objectsRoot = Entity()

    /// Parent for static obstacles. Kept separate from `objectsRoot` so the arm
    /// can't try to grasp them and so raycasts classify them as surfaces.
    private let obstaclesRoot = Entity()

    /// The physics simulation lives on this root. Every physics body (floor, arm
    /// colliders, dropped objects) is a descendant of it, which guarantees the
    /// simulation actually runs with gravity — the default scene doesn't always
    /// step physics in a macOS RealityView until a simulation root exists.
    private let simulationRoot = Entity()

    private var arm = RobotArm.ur5
    private let basePlinth = ModelEntity()
    private var didBuild = false

    // Phase 1: per-joint RGB coordinate-frame gizmos.
    private var frameGizmos: [Entity] = []
    private var jointFramesVisible = true

    // Phase 5 extras.
    private let reachSphere = ModelEntity()          // translucent reach volume
    private var trailDots: [ModelEntity] = []        // fading end-effector trail
    private var trailIndex = 0
    private var trailEnabled = false
    private let trailCount = 48
    private var currentFriction: Float = 0.6         // adjustable via settings

    /// Called once per rendered frame with the elapsed time. RobotViewport sets
    /// this to advance the arm animation and keep the arm/camera in sync.
    var onFrame: ((TimeInterval) -> Void)?
    private var frameSubscription: EventSubscription?

    /// Called (throttled) when a dropped object starts colliding — used for the
    /// landing "thud" sound.
    var onObjectLanded: (() -> Void)?
    private var collisionSubscription: EventSubscription?
    private var lastLandingTime: TimeInterval = 0

    // MARK: - Build

    func build(into content: inout RealityViewCameraContent) async {
        guard !didBuild else { return }
        didBuild = true

        content.camera = .virtual

        // Everything that participates in physics hangs off this simulation root.
        simulationRoot.components.set(PhysicsSimulationComponent())
        content.add(simulationRoot)

        addFloorAndGrid(to: &content)
        addWorkbench()
        addScaleReference()
        addLighting(to: &content)
        addArm(to: &content)
        addCamera(to: &content)
        simulationRoot.addChild(objectsRoot)
        simulationRoot.addChild(obstaclesRoot)

        // One callback per rendered frame drives animation + arm/camera sync.
        frameSubscription = content.subscribe(to: SceneEvents.Update.self) { [weak self] event in
            self?.onFrame?(event.deltaTime)
        }

        // A dropped object beginning a collision = it just landed / hit something.
        // Throttled so a settling stack doesn't machine-gun the thud sound.
        collisionSubscription = content.subscribe(to: CollisionEvents.Began.self) { [weak self] event in
            guard let self else { return }
            let involvesObject = self.objectsRoot.children.contains { $0 == event.entityA || $0 == event.entityB }
            guard involvesObject else { return }
            let now = Date().timeIntervalSinceReferenceDate
            guard now - self.lastLandingTime > 0.12 else { return }
            self.lastLandingTime = now
            self.onObjectLanded?()
        }

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
        floor.name = "Floor"
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
        simulationRoot.addChild(floor)

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

    // MARK: Workbench (mounts the arm above the floor)

    private func addWorkbench() {
        let topW: Float = 3.0, topD: Float = 3.0, topT: Float = 0.06

        // Tabletop slab: its top surface sits exactly at benchHeight, and it
        // carries the static collider objects rest on.
        let top = ModelEntity(
            mesh: .generateBox(width: topW, height: topT, depth: topD),
            materials: [Self.benchTopMaterial]
        )
        top.name = "Bench"
        top.position = SIMD3<Float>(0, benchHeight - topT / 2, 0)
        let topShape = ShapeResource.generateBox(width: topW, height: topT, depth: topD)
        top.components.set(CollisionComponent(shapes: [topShape]))
        top.components.set(PhysicsBodyComponent(
            shapes: [topShape], mass: 0, material: Self.objectPhysicsMaterial, mode: .static
        ))
        simulationRoot.addChild(top)

        // Four legs (visual only).
        let legInset: Float = 0.2
        let legR: Float = 0.05
        for sx in [Float(-1), 1] {
            for sz in [Float(-1), 1] {
                let leg = ModelEntity(
                    mesh: .generateCylinder(height: benchHeight - topT, radius: legR),
                    materials: [Self.benchFrameMaterial]
                )
                leg.position = SIMD3<Float>(
                    sx * (topW / 2 - legInset),
                    (benchHeight - topT) / 2,
                    sz * (topD / 2 - legInset)
                )
                simulationRoot.addChild(leg)
            }
        }
    }

    // MARK: Scale reference (a simple human silhouette)

    /// One familiar-sized object calibrates the whole scene instantly. A simple
    /// but proportioned ~1.7 m human figure (head, shoulders, torso, arms, legs)
    /// stands flush ON the bench top, off to one side.
    private func addScaleReference() {
        let person = Entity()
        // Feet at local y = 0; place the whole figure on the bench surface so it
        // sits flush with no gap.
        person.position = SIMD3<Float>(1.0, benchHeight, 1.0)

        var mat = PhysicallyBasedMaterial()
        mat.baseColor = .init(tint: NSColor(white: 0.5, alpha: 1))
        mat.roughness = 0.85

        func part(_ mesh: MeshResource, _ pos: SIMD3<Float>) -> ModelEntity {
            let e = ModelEntity(mesh: mesh, materials: [mat])
            e.position = pos
            return e
        }

        // Legs (two), pelvis→shoulders torso, two arms at the sides, neck, head.
        person.addChild(part(.generateCylinder(height: 0.8, radius: 0.08), [-0.1, 0.4, 0]))
        person.addChild(part(.generateCylinder(height: 0.8, radius: 0.08), [ 0.1, 0.4, 0]))
        person.addChild(part(.generateBox(width: 0.42, height: 0.55, depth: 0.22, cornerRadius: 0.08), [0, 1.1, 0]))
        person.addChild(part(.generateCylinder(height: 0.6, radius: 0.055), [-0.26, 1.05, 0]))
        person.addChild(part(.generateCylinder(height: 0.6, radius: 0.055), [ 0.26, 1.05, 0]))
        person.addChild(part(.generateCylinder(height: 0.1, radius: 0.05), [0, 1.43, 0]))   // neck
        person.addChild(part(.generateSphere(radius: 0.12), [0, 1.58, 0]))                  // head

        simulationRoot.addChild(person)
    }

    // MARK: Lighting (three-point + shadows)

    private let keyLight = Entity()
    private let fillLight = Entity()
    private let rimLight = Entity()
    private let warningStripes = Entity()

    private func addLighting(to content: inout RealityViewCameraContent) {
        // KEY casts the contact shadow; FILL softens the far side; RIM separates
        // the silhouette. Intensities/colors are set by `setEnvironment`.
        keyLight.components.set(DirectionalLightComponent.Shadow())
        keyLight.look(at: .zero, from: SIMD3<Float>(1.5, 2.5, 1.8), relativeTo: nil)
        content.add(keyLight)

        fillLight.look(at: .zero, from: SIMD3<Float>(-2.5, 1.5, -1.5), relativeTo: nil)
        content.add(fillLight)

        rimLight.look(at: .zero, from: SIMD3<Float>(0, 1.2, -3), relativeTo: nil)
        content.add(rimLight)

        // Hazard stripes around the bench edge, shown only in the Factory preset.
        buildWarningStripes()
        content.add(warningStripes)

        setEnvironment(.classroom)
    }

    /// Applies a lighting/environment preset (Phase 7). Only lights + the hazard
    /// stripes change — never physics or the arm.
    func setEnvironment(_ preset: EnvironmentPreset) {
        let warm = NSColor(red: 1.0, green: 0.94, blue: 0.82, alpha: 1)
        let cool = NSColor(red: 0.80, green: 0.86, blue: 1.0, alpha: 1)

        switch preset {
        case .classroom:
            setLight(keyLight, .white, 3200)
            setLight(fillLight, .white, 900)
            setLight(rimLight, .white, 700)
            warningStripes.isEnabled = false
        case .factory:
            setLight(keyLight, warm, 5000)   // harsher, warmer key
            setLight(fillLight, warm, 500)
            setLight(rimLight, warm, 600)
            warningStripes.isEnabled = true
        case .nightShift:
            setLight(keyLight, cool, 2600)   // single dramatic source
            setLight(fillLight, cool, 80)    // very dim ambient
            setLight(rimLight, cool, 300)
            warningStripes.isEnabled = false
        }
    }

    private func setLight(_ entity: Entity, _ color: NSColor, _ intensity: Float) {
        // Re-setting the light component leaves any Shadow component intact.
        entity.components.set(DirectionalLightComponent(color: color, intensity: intensity))
    }

    /// A yellow hazard border just inside the bench edge (Factory preset only).
    private func buildWarningStripes() {
        let mat = UnlitMaterial(color: NSColor(red: 0.95, green: 0.8, blue: 0.1, alpha: 1))
        let edge: Float = 1.45, thick: Float = 0.06, y = benchHeight + 0.002
        let spans: [(SIMD3<Float>, SIMD3<Float>)] = [
            (SIMD3(0, y, edge),  SIMD3(2 * edge, 0.004, thick)),
            (SIMD3(0, y, -edge), SIMD3(2 * edge, 0.004, thick)),
            (SIMD3(edge, y, 0),  SIMD3(thick, 0.004, 2 * edge)),
            (SIMD3(-edge, y, 0), SIMD3(thick, 0.004, 2 * edge)),
        ]
        for (pos, size) in spans {
            let bar = ModelEntity(mesh: .generateBox(size: size), materials: [mat])
            bar.position = pos
            warningStripes.addChild(bar)
        }
        warningStripes.isEnabled = false
    }

    // MARK: Arm

    private func addArm(to content: inout RealityViewCameraContent) {
        armRoot.orientation = CoordinateSpace.robotToRealityKit
        armRoot.position = armBaseOffset          // sit the arm on the bench top
        armRoot.scale = SIMD3<Float>(repeating: armScale)
        simulationRoot.addChild(armRoot)

        basePlinth.components.set(ModelComponent(mesh: .generateCylinder(height: 0.05, radius: 0.09),
                                                 materials: [Self.charcoalMaterial]))
        simulationRoot.addChild(basePlinth)

        rebuildArmGeometry()
        addTool()
    }

    /// (Re)builds the arm's links, joint housings, base plinth, and frame gizmos
    /// for the current `arm` and `armScale`. Called on first build and whenever
    /// the selected arm profile changes (Phase 10).
    private func rebuildArmGeometry() {
        // Remove any existing arm geometry.
        (linkEntities + housingEntities + frameGizmos).forEach { $0.removeFromParent() }
        linkEntities.removeAll()
        housingEntities.removeAll()
        frameGizmos.removeAll()

        // Base plinth scaled to match the arm.
        basePlinth.scale = SIMD3<Float>(repeating: armScale)
        basePlinth.position = SIMD3<Float>(0, benchHeight + 0.025 * armScale, 0)

        for i in 0..<arm.degreesOfFreedom {
            // The distance between consecutive joint origins is CONSTANT = √(a²+d²)
            // (independent of joint angle), so we build fixed-length links and only
            // rotate/translate them — non-uniform scaling would distort colliders.
            let dh = arm.joints[i].dh
            let length = max(Float((dh.a * dh.a + dh.d * dh.d).squareRoot()), 0.02)

            let linkMesh = MeshResource.generateCylinder(height: length, radius: linkRadius)
            let link = ModelEntity(mesh: linkMesh, materials: [Self.aluminiumMaterial])
            let linkShape = ShapeResource.generateConvex(from: linkMesh)
            link.components.set(CollisionComponent(shapes: [linkShape]))
            var body = PhysicsBodyComponent(
                shapes: [linkShape], mass: 1,
                material: Self.objectPhysicsMaterial, mode: .kinematic
            )
            body.isContinuousCollisionDetectionEnabled = true   // no tunneling
            link.components.set(body)
            linkEntities.append(link)
            armRoot.addChild(link)

            let housing = ModelEntity(
                mesh: .generateCylinder(height: housingHalfHeight * 2, radius: housingRadius),
                materials: [Self.charcoalMaterial]
            )
            housingEntities.append(housing)
            armRoot.addChild(housing)
        }

        addJointFrames()
    }

    /// Swaps in a different arm profile (Phase 10): rebuilds geometry at the new
    /// scale, resizes the reach volume, and redraws at the home pose.
    func setArm(_ newArm: RobotArm, scale: Float) {
        arm = newArm
        armScale = scale
        armRoot.scale = SIMD3<Float>(repeating: scale)
        rebuildArmGeometry()
        reachSphere.model = ModelComponent(
            mesh: .generateSphere(radius: armReach() * scale),
            materials: [Self.reachMaterial]
        )
        updateArm(with: [Double](repeating: 0, count: arm.degreesOfFreedom))
    }

    /// Approximate max reach = tool distance at the fully-stretched (all-zero) pose.
    private func armReach() -> Float {
        let zeros = [Double](repeating: 0, count: arm.degreesOfFreedom)
        return Float(simd_length(arm.endEffectorPosition(jointAngles: zeros)))
    }

    // MARK: Joint coordinate frames (Phase 1)

    /// Builds a small RGB axis gizmo (X=red, Y=green, Z=blue) for each joint. The
    /// gizmos are re-oriented every frame from the FK frames, so they show each
    /// joint's real, current orientation — not a static overlay.
    private func addJointFrames() {
        frameGizmos.removeAll()
        for _ in 0..<arm.degreesOfFreedom {
            let gizmo = Entity()
            gizmo.addChild(axisBar(color: .systemRed,   along: SIMD3<Float>(1, 0, 0)))
            gizmo.addChild(axisBar(color: .systemGreen, along: SIMD3<Float>(0, 1, 0)))
            gizmo.addChild(axisBar(color: .systemBlue,  along: SIMD3<Float>(0, 0, 1)))
            gizmo.isEnabled = jointFramesVisible
            frameGizmos.append(gizmo)
            armRoot.addChild(gizmo)
        }
    }

    /// One colored axis bar, extending from the origin along `along`.
    private func axisBar(color: NSColor, along dir: SIMD3<Float>) -> ModelEntity {
        let length: Float = 0.1, thin: Float = 0.008
        let size = SIMD3<Float>(
            dir.x != 0 ? length : thin,
            dir.y != 0 ? length : thin,
            dir.z != 0 ? length : thin
        )
        // Unlit so the axis colors stay pure regardless of scene lighting.
        let bar = ModelEntity(mesh: .generateBox(size: size), materials: [UnlitMaterial(color: color)])
        bar.position = dir * (length / 2)   // start at the joint origin
        return bar
    }

    func setJointFramesVisible(_ visible: Bool) {
        jointFramesVisible = visible
        frameGizmos.forEach { $0.isEnabled = visible }
    }

    private func addTool() {
        // Wrist flange behind the fingers.
        let flange = ModelEntity(
            mesh: .generateCylinder(height: 0.03, radius: 0.05),
            materials: [Self.steelMaterial]
        )
        flange.orientation = simd_quatf(angle: .pi / 2, axis: SIMD3<Float>(1, 0, 0)) // face +Z
        flange.position = SIMD3<Float>(0, 0, -0.02)
        toolEntity.addChild(flange)

        // Two fingers that STRADDLE the tool origin (so a target we IK-to sits
        // between them). Each finger carries its own collider so we can detect
        // two-sided contact when closing, and so they physically push objects.
        let fingerMesh = MeshResource.generateBox(size: SIMD3<Float>(0.02, 0.055, 0.09))
        let fingerShape = ShapeResource.generateBox(size: SIMD3<Float>(0.02, 0.055, 0.09))
        for finger in [leftFinger, rightFinger] {
            finger.model = ModelComponent(mesh: fingerMesh, materials: [Self.steelMaterial])
            finger.position = SIMD3<Float>(0, 0, 0.03)
            finger.components.set(CollisionComponent(shapes: [fingerShape]))
            var body = PhysicsBodyComponent(
                shapes: [fingerShape], mass: 1,
                material: Self.objectPhysicsMaterial, mode: .kinematic
            )
            body.isContinuousCollisionDetectionEnabled = true
            finger.components.set(body)
            toolEntity.addChild(finger)
        }

        setFingerOpening(1)   // start open
        armRoot.addChild(toolEntity)

        addReachVolume()
        addTrail()
    }

    // MARK: Reach visualization (Phase 5)

    /// A translucent sphere showing the UR5's ~0.85 m maximum reach around its
    /// mounting point. Hidden until toggled on.
    private func addReachVolume() {
        reachSphere.model = ModelComponent(
            mesh: .generateSphere(radius: armReach() * armScale),
            materials: [Self.reachMaterial]
        )
        reachSphere.position = armBaseOffset
        reachSphere.isEnabled = false
        simulationRoot.addChild(reachSphere)
    }

    private static let reachMaterial: UnlitMaterial = {
        var mat = UnlitMaterial(color: NSColor(red: 0.36, green: 0.36, blue: 0.9, alpha: 1))
        mat.blending = .transparent(opacity: 0.10)
        return mat
    }()

    func setReachVisible(_ visible: Bool) { reachSphere.isEnabled = visible }

    // MARK: Motion trail (Phase 5)

    private func addTrail() {
        let mat = UnlitMaterial(color: NSColor(red: 0.36, green: 0.36, blue: 0.9, alpha: 1))
        for _ in 0..<trailCount {
            let dot = ModelEntity(mesh: .generateSphere(radius: 0.008), materials: [mat])
            dot.isEnabled = false
            trailDots.append(dot)
            simulationRoot.addChild(dot)
        }
    }

    func setTrailEnabled(_ enabled: Bool) {
        trailEnabled = enabled
        if !enabled {
            trailDots.forEach { $0.isEnabled = false }
            trailIndex = 0
        }
    }

    /// Records the current end-effector position into the trail ring buffer and
    /// fades older dots down by shrinking them.
    func updateTrail() {
        guard trailEnabled else { return }
        let toolPos = toolEntity.position(relativeTo: nil)
        trailDots[trailIndex].position = toolPos
        trailDots[trailIndex].isEnabled = true
        trailIndex = (trailIndex + 1) % trailCount
        for k in 0..<trailCount {
            let age = (trailIndex + trailCount - 1 - k) % trailCount   // 0 = newest
            let f = 1 - Float(age) / Float(trailCount)
            trailDots[k].scale = SIMD3<Float>(repeating: max(0.15, f))
        }
    }

    // MARK: Physics parameters (Phase 5)

    func setGravity(_ g: Float) {
        guard var sim = simulationRoot.components[PhysicsSimulationComponent.self] else { return }
        sim.gravity = SIMD3<Float>(0, -g, 0)
        simulationRoot.components.set(sim)
    }

    func setFriction(_ f: Float) {
        currentFriction = f
        let mat = PhysicsMaterialResource.generate(friction: f, restitution: 0.1)
        for object in objectsRoot.children.compactMap({ $0 as? ModelEntity }) {
            if var body = object.components[PhysicsBodyComponent.self] {
                body.material = mat
                object.components.set(body)
            }
        }
    }

    // MARK: Gripper-mounted camera (Phase 5 preset)

    /// Positions the camera just behind the gripper, looking down its approach
    /// axis — a first-person "from the gripper" view.
    func updateCameraToGripper() {
        let m = toolEntity.transformMatrix(relativeTo: nil)
        let pos = SIMD3<Float>(m.columns.3.x, m.columns.3.y, m.columns.3.z)
        let forward = simd_normalize(SIMD3<Float>(m.columns.2.x, m.columns.2.y, m.columns.2.z))
        let up = simd_normalize(SIMD3<Float>(m.columns.1.x, m.columns.1.y, m.columns.1.z))
        let camPos = pos - forward * 0.14 + up * 0.05
        camera.look(at: pos + forward * 0.3, from: camPos, relativeTo: nil)
    }

    // MARK: Gripper fingers

    /// Positions the fingers. `t` = 1 fully open, 0 fully closed.
    func setFingerOpening(_ t: Float) {
        let x = fingerClosedX + (fingerOpenX - fingerClosedX) * max(0, min(1, t))
        leftFinger.position.x = -x
        rightFinger.position.x = x
    }

    /// Collision-based grasp test, run at the moment the gripper finishes closing.
    /// A grasp only succeeds if BOTH fingers are in contact with the SAME object,
    /// one on each side — closing on empty air, or touching only one side, returns
    /// nil so nothing attaches. We cast a short ray inward from each finger toward
    /// the other; if both rays strike the same dropped object, it's a valid grasp.
    func attemptGrasp() -> ModelEntity? {
        guard let realityScene = simulationRoot.scene else { return nil }
        let leftPos = leftFinger.position(relativeTo: nil)
        let rightPos = rightFinger.position(relativeTo: nil)
        let span = simd_length(rightPos - leftPos)
        guard span > 1e-4 else { return nil }
        let inward = (rightPos - leftPos) / span

        let leftObj = firstObjectHit(realityScene, from: leftPos, direction: inward, length: span)
        let rightObj = firstObjectHit(realityScene, from: rightPos, direction: -inward, length: span)

        if let l = leftObj, let r = rightObj, l == r { return l }
        return nil
    }

    /// First DROPPED OBJECT a ray hits (skipping the arm's own colliders).
    private func firstObjectHit(
        _ scene: RealityKit.Scene, from origin: SIMD3<Float>,
        direction: SIMD3<Float>, length: Float
    ) -> ModelEntity? {
        let hits = scene.raycast(origin: origin, direction: direction, length: length, query: .all)
        for hit in hits {
            if let obj = objectsRoot.children.first(where: { $0 == hit.entity }) as? ModelEntity {
                return obj
            }
        }
        return nil
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
            // Link spans this joint origin to the next (fixed length; no scaling).
            linkEntities[i].alignBetween(points[i], points[i + 1])

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

        // Joint frame gizmos take each joint frame's full pose.
        if jointFramesVisible {
            for i in 0..<min(frameGizmos.count, frames.count) {
                frameGizmos[i].transform = Transform(matrix: simd_float4x4(frames[i]))
            }
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

    /// Removes all dropped objects AND obstacles.
    func clearAll() {
        clearObjects()
        for child in obstaclesRoot.children.map({ $0 }) {
            child.removeFromParent()
        }
    }

    // MARK: - Obstacles (Phase 4: static bodies)

    /// Spawns a STATIC obstacle resting on the surface the ray hit. Static bodies
    /// don't fall; dynamic objects and the arm collide against them.
    func spawnObstacle(_ kind: ObstacleKind, at rawSurface: SIMD3<Float>) {
        // Never place an obstacle below the floor plane.
        let surface = SIMD3<Float>(rawSurface.x, max(rawSurface.y, 0), rawSurface.z)

        let material = Self.obstacleMaterial(for: kind)

        let entity = ModelEntity()
        entity.name = "obstacle.\(kind.rawValue)"

        let shape: ShapeResource
        switch kind {
        case .platform:
            let s = SIMD3<Float>(0.3, 0.12, 0.3)
            entity.model = ModelComponent(mesh: .generateBox(size: s), materials: [material])
            shape = .generateBox(size: s)
            entity.position = surface + SIMD3<Float>(0, s.y / 2, 0)   // rest on surface
        case .wall:
            let s = SIMD3<Float>(0.4, 0.28, 0.05)
            entity.model = ModelComponent(mesh: .generateBox(size: s), materials: [material])
            shape = .generateBox(size: s)
            entity.position = surface + SIMD3<Float>(0, s.y / 2, 0)
        case .ramp:
            let (mesh, corners) = Self.makeWedgeMesh(width: 0.3, depth: 0.3, height: 0.16)
            entity.model = ModelComponent(mesh: mesh, materials: [material])
            shape = .generateConvex(from: corners)
            entity.position = surface   // wedge base already sits at local y = 0
        }

        entity.components.set(CollisionComponent(shapes: [shape]))
        entity.components.set(PhysicsBodyComponent(
            shapes: [shape], mass: 0, material: Self.objectPhysicsMaterial, mode: .static
        ))
        obstaclesRoot.addChild(entity)
    }

    /// Every dropped object currently in the world.
    var droppedObjects: [Entity] { objectsRoot.children.map { $0 } }

    /// The bench-top height, so challenge checks can reason about "elevated".
    var benchTopHeight: Float { benchHeight }

    /// A lightweight snapshot of each dropped object's kind + world position,
    /// used by the challenge checker (Phase 9).
    func objectSnapshots() -> [(kind: ObjectKind, position: SIMD3<Float>)] {
        objectsRoot.children.compactMap { entity in
            let raw = entity.name.replacingOccurrences(of: "object.", with: "")
            guard let kind = ObjectKind(rawValue: raw) else { return nil }
            return (kind, entity.position(relativeTo: nil))
        }
    }

    /// World positions of the static obstacles by kind (Phase 9).
    func obstacleSnapshots() -> [(kind: ObstacleKind, position: SIMD3<Float>)] {
        obstaclesRoot.children.compactMap { entity in
            let raw = entity.name.replacingOccurrences(of: "obstacle.", with: "")
            guard let kind = ObstacleKind(rawValue: raw) else { return nil }
            return (kind, entity.position(relativeTo: nil))
        }
    }

    private func makeObject(_ kind: ObjectKind) -> ModelEntity {
        var material = PhysicallyBasedMaterial()
        material.baseColor = .init(tint: Self.objectColor(kind))
        material.roughness = 0.5
        material.metallic = 0.0

        let mesh: MeshResource
        let shape: ShapeResource
        let mass: Float

        // Sized to fit the gripper (which at this arm scale opens to ~24 cm and
        // closes to ~10 cm), so objects are ~12 cm.
        switch kind {
        case .cube:
            let side: Float = 0.12
            mesh = .generateBox(size: side, cornerRadius: 0.008)
            shape = .generateBox(size: SIMD3<Float>(repeating: side))
            mass = 0.4
        case .sphere:
            let r: Float = 0.06
            mesh = .generateSphere(radius: r)
            shape = .generateSphere(radius: r)
            mass = 0.3
        case .ramp:
            let (wedgeMesh, corners) = Self.makeWedgeMesh(width: 0.14, depth: 0.14, height: 0.09)
            mesh = wedgeMesh
            shape = .generateConvex(from: corners)
            mass = 0.5
        }

        let entity = ModelEntity(mesh: mesh, materials: [material])
        entity.name = "object.\(kind.rawValue)"
        entity.components.set(CollisionComponent(shapes: [shape]))
        var body = PhysicsBodyComponent(
            shapes: [shape], mass: mass,
            material: PhysicsMaterialResource.generate(friction: currentFriction, restitution: 0.1),
            mode: .dynamic
        )
        // CCD so a small object dropped from a height can't fall THROUGH the thin
        // floor between two frames (the classic "tunneling" bug).
        body.isContinuousCollisionDetectionEnabled = true
        entity.components.set(body)
        return entity
    }

    /// Spawns a shelf object a little ABOVE the drop point and lets gravity carry
    /// it down over several frames — so you see it fall and land, not teleport.
    @discardableResult
    func dropObject(_ kind: ObjectKind, onto groundPoint: SIMD3<Float>) -> ModelEntity {
        var spawn = groundPoint
        spawn.y += 0.35   // small height above the surface
        return spawnObject(kind, at: spawn)
    }

    /// Spawns a user-drawn extruded shape as a dynamic physics object, dropped
    /// from just above the surface. Reusable — every drag spawns a fresh copy.
    func dropDrawnObject(_ shape: DrawnShape, onto groundPoint: SIMD3<Float>) {
        let (mesh, corners) = DrawnGeometry.extrude(shape.points, size: 0.14, depth: 0.06)
        var material = PhysicallyBasedMaterial()
        material.baseColor = .init(tint: NSColor(hue: CGFloat(shape.hue), saturation: 0.6, brightness: 0.85, alpha: 1))
        material.roughness = 0.5

        let entity = ModelEntity(mesh: mesh, materials: [material])
        entity.name = "object.drawn"
        let collider = ShapeResource.generateConvex(from: corners)
        entity.components.set(CollisionComponent(shapes: [collider]))
        var body = PhysicsBodyComponent(
            shapes: [collider], mass: 0.3,
            material: PhysicsMaterialResource.generate(friction: currentFriction, restitution: 0.1),
            mode: .dynamic
        )
        body.isContinuousCollisionDetectionEnabled = true
        entity.components.set(body)
        entity.position = groundPoint + SIMD3<Float>(0, 0.35, 0)
        objectsRoot.addChild(entity)
    }

    private static func objectColor(_ kind: ObjectKind) -> NSColor {
        switch kind {
        case .cube:   return NSColor(red: 0.86, green: 0.42, blue: 0.34, alpha: 1)
        case .sphere: return NSColor(red: 0.30, green: 0.55, blue: 0.80, alpha: 1)
        case .ramp:   return NSColor(red: 0.52, green: 0.62, blue: 0.38, alpha: 1)
        }
    }

    // MARK: - Interaction (Phase 3: pick / place)

    /// Raycasts into the live scene and classifies what the ray hit first.
    func pick(origin: SIMD3<Float>, direction: SIMD3<Float>) -> PickHit {
        guard let realityScene = objectsRoot.scene else { return .none }
        let hits = realityScene.raycast(
            origin: origin, direction: direction, length: 100, query: .nearest
        )
        guard let hit = hits.first else { return .none }

        // A dropped object?
        if let obj = objectsRoot.children.first(where: { $0 == hit.entity }) as? ModelEntity {
            return .object(obj, hit.position)
        }
        // Clicking the arm itself does nothing.
        if isDescendant(hit.entity, of: armRoot) { return .none }
        // Anything else solid (floor, bench, obstacle) is a valid place surface.
        return .ground(hit.position)
    }

    private func isDescendant(_ entity: Entity, of ancestor: Entity) -> Bool {
        var e: Entity? = entity
        while let current = e {
            if current == ancestor { return true }
            e = current.parent
        }
        return false
    }

    /// Converts a point from RealityKit world space into the arm's base frame
    /// (robot Z-up meters), which is what the IK solver expects. The arm root has
    /// only the Z-up→Y-up rotation and no translation/scale, so inverting that
    /// rotation is the whole conversion.
    func worldToRobot(_ p: SIMD3<Float>) -> SIMD3<Float> {
        // Undo the base offset (bench height), then the Z-up→Y-up rotation, then
        // the arm's visual scale — so the IK solver works in the arm's own units.
        let local = CoordinateSpace.robotToRealityKit.inverse.act(p - armBaseOffset)
        return local / armScale
    }

    /// First surface the ray strikes (bench top, floor, obstacle, or object),
    /// used to decide where a dragged shelf item lands.
    func surfaceHit(origin: SIMD3<Float>, direction: SIMD3<Float>) -> SIMD3<Float>? {
        guard let realityScene = simulationRoot.scene else { return nil }
        return realityScene.raycast(
            origin: origin, direction: direction, length: 100, query: .nearest
        ).first?.position
    }

    /// Attaches a grasped object to the gripper.
    ///
    /// We PARENT the object to the gripper and make its body kinematic, rather
    /// than using a physics fixed-joint. Parenting + kinematic is far more stable
    /// here: the object rigidly tracks the fast-moving gripper with zero solver
    /// jitter, whereas RealityKit physics joints need a joints/simulation setup
    /// and tend to stretch or vibrate under quick kinematic motion. We preserve
    /// the world transform so the object stays exactly where it was grasped —
    /// between the fingers — instead of snapping to a fixed offset.
    func grab(_ object: ModelEntity) {
        object.setParent(toolEntity, preservingWorldTransform: true)
        if var body = object.components[PhysicsBodyComponent.self] {
            body.mode = .kinematic
            object.components.set(body)
        }
        setHighlight(object, on: true)
    }

    /// Releases an object: switches it back to a DYNAMIC body FIRST, then reparents
    /// it to the world keeping its exact current position. Order matters — making
    /// it dynamic before any reparenting means the physics engine takes ownership
    /// in place, so it starts falling from wherever it was let go instead of
    /// teleporting. We never set an explicit resting position; gravity does that.
    func release(_ object: ModelEntity) {
        setHighlight(object, on: false)
        if var body = object.components[PhysicsBodyComponent.self] {
            body.mode = .dynamic
            object.components.set(body)
        }
        object.setParent(objectsRoot, preservingWorldTransform: true)
    }

    /// Toggles a soft emissive glow to show which object is currently held.
    private func setHighlight(_ object: ModelEntity, on: Bool) {
        guard var model = object.model,
              var material = model.materials.first as? PhysicallyBasedMaterial else { return }
        if on {
            material.emissiveColor = .init(color: NSColor(red: 0.36, green: 0.36, blue: 0.9, alpha: 1))
            material.emissiveIntensity = 0.9
        } else {
            material.emissiveIntensity = 0
        }
        model.materials = [material]
        object.model = model
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

    /// Matte top surface of the bench (things rest on it).
    private static let benchTopMaterial: PhysicallyBasedMaterial = {
        var m = PhysicallyBasedMaterial()
        m.baseColor = .init(tint: NSColor(red: 0.30, green: 0.32, blue: 0.36, alpha: 1))
        m.roughness = 0.55
        m.metallic = 0.3
        return m
    }()

    /// Brushed-metal look for the bench edge/legs (Phase 8): lower roughness and
    /// high metallic so the frame reads as machined steel rather than flat gray.
    private static let benchFrameMaterial: PhysicallyBasedMaterial = {
        var m = PhysicallyBasedMaterial()
        m.baseColor = .init(tint: NSColor(red: 0.42, green: 0.44, blue: 0.48, alpha: 1))
        m.roughness = 0.25
        m.metallic = 0.95
        return m
    }()

    /// Obstacle materials with a little per-type variation (Phase 8) so they
    /// don't all look like the same flat primitive.
    private static func obstacleMaterial(for kind: ObstacleKind) -> PhysicallyBasedMaterial {
        var m = PhysicallyBasedMaterial()
        switch kind {
        case .platform:
            m.baseColor = .init(tint: NSColor(white: 0.62, alpha: 1))
            m.roughness = 0.7;  m.metallic = 0.15
        case .wall:
            m.baseColor = .init(tint: NSColor(red: 0.6, green: 0.58, blue: 0.55, alpha: 1))
            m.roughness = 0.9;  m.metallic = 0.0
        case .ramp:
            m.baseColor = .init(tint: NSColor(white: 0.58, alpha: 1))
            m.roughness = 0.55; m.metallic = 0.2
        }
        return m
    }

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
