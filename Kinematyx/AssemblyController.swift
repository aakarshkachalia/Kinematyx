//
//  AssemblyController.swift
//  Kinematyx
//
//  Created by Aakarsh Kachalia on 7/18/26.
//

//
//  AssemblyController.swift
//  Kinematix
//
//  Drives multi-part SNAP-FIT assembly. It owns the model-car plan (from
//  RobotArmKit), spawns the parts into the scene, and every frame — while a part
//  is held — measures that part's active mating feature against its target
//  socket. When both the position AND orientation errors fall under tolerance it
//  SNAPS: the part jumps to the exact mated pose and welds into the assembly.
//
//  Snap-fit, not physics insertion: we never simulate a peg sliding into a hole
//  via contact forces (an open research problem). We just watch the alignment and
//  weld when it's close enough. Failure stays possible — open the gripper out of
//  tolerance and the part simply falls.
//

import Foundation
import Observation
import RealityKit
import RobotArmKit
import simd

@MainActor
@Observable
final class AssemblyController {

    // MARK: Tunable snap tolerances (NOT magic numbers — surfaced and reused)

    /// Position tolerance for a snap: the mating feature must be within this of
    /// its target socket (meters). Default 15 mm.
    static let snapPositionTolerance: Double = 0.015
    /// Orientation tolerance for a snap (radians), measured as the tilt of the
    /// insertion axis (rotation ABOUT it is free — the features are round). Default 15°.
    static let snapOrientationTolerance: Double = 15 * .pi / 180
    /// Auto-assembly seat-fallback position tolerance (meters): if the live snap
    /// didn't fire but the part is this close to its exact mate spot, seat it anyway
    /// (the weld fixes orientation). Default 25 mm.
    static let snapFallbackPositionTolerance: Double = 0.025

    // MARK: State

    /// The models the arm can build. Each maps to an `AssemblyDefinition` in
    /// RobotArmKit and its own scene-spawning layout.
    enum Model: String, CaseIterable, Identifiable {
        case car = "Model car"
        case tower = "Tower"
        var id: String { rawValue }

        var definition: AssemblyDefinition {
            switch self {
            case .car:   return ModelCar.definition
            case .tower: return Tower.definition
            }
        }
    }

    /// The model currently loaded (or last chosen). Drives `definition`.
    private(set) var model: Model = .car
    var definition: AssemblyDefinition { model.definition }

    private unowned var scene: SandboxScene!
    private unowned var arm: ArmController!

    /// partID → its scene entity.
    private var entities: [String: ModelEntity] = [:]
    /// partID → its known WORLD pose: the fixed base part plus every part welded so
    /// far. A step's target may be ANY already-placed part (a tower block stacks on
    /// the block below), so we track them all rather than a single base pose.
    private var placedWorld: [String: Pose] = [:]

    private(set) var completedSteps: Set<String> = []
    private(set) var loaded = false

    /// Live alignment readout for the held part (nil when nothing snappable held).
    struct Indicator {
        let partName: String
        let distanceMillimeters: Double
        let angleDegrees: Double
        let withinTolerance: Bool
    }
    private(set) var indicator: Indicator?
    /// Last snap event, e.g. "Front-left wheel attached" — shown as a status line.
    private(set) var lastEvent: String?

    // MARK: Auto-assemble state + metrics

    private(set) var isAutoRunning = false
    private(set) var isPaused = false
    /// Set when a step stops the run (IK unreachable or a snap missed); the scene
    /// is left inspectable rather than crashing or silently continuing.
    private(set) var failureMessage: String?
    private var autoTask: Task<Void, Never>?

    /// Cycle-time / effort metrics for the build.
    struct Metrics {
        var totalTime: TimeInterval = 0          // summed step time (excludes pauses)
        var perStep: [(name: String, time: TimeInterval)] = []
        var travelMeters: Double = 0             // total tool travel in world space
        var failedSnaps: Int = 0
    }
    private(set) var metrics = Metrics()
    /// Last planned tool position, for accumulating travel distance.
    private var lastToolPlanned: SIMD3<Double>?

    func attach(scene: SandboxScene, arm: ArmController) {
        self.scene = scene
        self.arm = arm
    }

    // MARK: Loading / reset

    /// Horizontal shift of the WHOLE model preset out along +X, away from the arm
    /// base at the origin. This pushes every mating feature out of the arm's inner
    /// dead zone and into its dexterous workspace, while staying within reach.
    static let siteOffset = SIMD3<Float>(0.22, 0, 0)

    /// Loads the given model: spawns its fixed base at the build site and its loose
    /// pieces staged in the supply tray, and resets progress. Safe to call while a
    /// build (or auto-run) is in progress — it stops and clears first.
    func load(_ model: Model) {
        stopAuto()
        self.model = model
        clear()
        switch model {
        case .car:   spawnCar()
        case .tower: spawnTower()
        }
        completedSteps = []
        lastEvent = nil
        indicator = nil
        failureMessage = nil
        metrics = Metrics()
        loaded = true
    }

    /// Convenience retained for existing callers.
    func loadCar() { load(.car) }

    /// The chassis (fixed) plus the wheels and body staged in the tray.
    private func spawnCar() {
        let benchTop = scene.benchTopHeight
        let site = Self.siteOffset

        // Parts tray: a mat under the loose parts (wheels + body).
        scene.spawnTray(center: SIMD3<Float>(0.48, benchTop + 0.003, 0.42) + site,
                        size: SIMD2<Float>(0.64, 1.00))

        // Chassis: fixed build site, out along +X but clear of the base.
        let chassisY = benchTop + Float(CarGeometry.chassisHeight) / 2
        let chassisPose = Pose(position: SIMD3<Double>(SIMD3<Float>(0.46, chassisY, -0.28) + site))
        entities[ModelCar.chassisID] = scene.spawnChassis(at: chassisPose)
        placedWorld[ModelCar.chassisID] = chassisPose

        // Wheels: laid flat in a widely-spaced 2×2 grid on the tray.
        let wheelY = benchTop + Float(CarGeometry.wheelThickness) / 2
        let wheelSpots: [SIMD3<Float>] = [
            SIMD3<Float>(0.30, wheelY, 0.40) + site,
            SIMD3<Float>(0.66, wheelY, 0.40) + site,
            SIMD3<Float>(0.30, wheelY, 0.76) + site,
            SIMD3<Float>(0.66, wheelY, 0.76) + site,
        ]
        for (i, w) in ModelCar.wheels.enumerated() {
            let wheel = scene.spawnWheel(id: w.id, at: wheelSpots[i])
            scene.setFrozen(wheel, true)          // sit still until grasped
            entities[w.id] = wheel
        }

        // Body shell: on the tray between the wheels and the chassis.
        let bodyY = benchTop + Float(CarGeometry.bodyHeight) / 2
        let body = scene.spawnBody(at: SIMD3<Float>(0.46, bodyY, 0.06) + site)
        scene.setFrozen(body, true)
        entities[ModelCar.bodyID] = body
    }

    /// The tower base (fixed, at the build site) plus the loose blocks and cap laid
    /// out in a supply tray OFF TO THE SIDE, so the arm fetches each and carries it
    /// over to stack it.
    private func spawnTower() {
        let g = TowerGeometry.self
        let benchTop = scene.benchTopHeight
        let site = Self.siteOffset

        // Build site: the base plate, nearest the arm along +X. The tower rises here.
        let baseY = benchTop + Float(g.baseHeight) / 2
        let basePose = Pose(position: SIMD3<Double>(SIMD3<Float>(0.42, baseY, -0.20) + site))
        entities[Tower.baseID] = scene.spawnTowerBase(at: basePose)
        placedWorld[Tower.baseID] = basePose

        // Supply tray: a mat holding the loose blocks + cap, off to the +Z side and
        // clear of the build column.
        scene.spawnTray(center: SIMD3<Float>(0.50, benchTop + 0.003, 0.52) + site,
                        size: SIMD2<Float>(0.72, 0.86))

        // Lay the loose pieces in a widely-spaced grid on the tray (7 spots: 6
        // blocks + cap), all reachable for a straight-down pick.
        let blockY = benchTop + Float(g.blockHeight) / 2
        let capY = benchTop + Float(g.capHeight) / 2
        let cols: [Float] = [0.26, 0.50, 0.74]
        let rows: [Float] = [0.34, 0.58, 0.82]
        var spots: [SIMD3<Float>] = []
        for r in rows { for c in cols { spots.append(SIMD3<Float>(c, blockY, r) + site) } }

        let count = Tower.blocks.count
        for (i, b) in Tower.blocks.enumerated() {
            let block = scene.spawnTowerBlock(id: b.id, at: spots[i], hue: Double(i) / Double(count))
            scene.setFrozen(block, true)
            entities[b.id] = block
        }
        // Cap goes in the next free spot, sitting a touch lower (thinner piece).
        var capSpot = spots[Tower.blocks.count]
        capSpot.y = capY
        let cap = scene.spawnTowerCap(at: capSpot)
        scene.setFrozen(cap, true)
        entities[Tower.capID] = cap
    }

    /// Returns everything to the tray and clears progress (same model).
    func reset() {
        stopAuto()
        guard loaded else { return }
        load(model)
    }

    private func clear() {
        for entity in entities.values { entity.removeFromParent() }
        scene.clearAssembly()
        entities = [:]
        placedWorld = [:]
        loaded = false
    }

    // MARK: Per-frame snap check

    func tick() {
        guard loaded,
              let held = arm.heldEntity,
              let partID = partID(of: held),
              let step = candidateStep(forMovingPart: partID),
              let targetWorld = placedWorld[step.targetPartID],
              let mate = definition.mateWorldPose(for: step, targetPartWorld: targetWorld),
              let movingPart = definition.part(id: partID)
        else {
            indicator = nil
            return
        }

        // Compare the held part's CURRENT world pose against the exact MATED pose
        // it should reach. (Comparing the feature to the raw socket would read a
        // constant 180° error, because a peg-in-hole mate is deliberately opposed —
        // the part's axis points INTO the socket, antiparallel to it.)
        let heldPose = scene.worldPose(of: held)
        let distance = heldPose.positionDistance(to: mate)

        // The mating features here (round pegs/holes, studs/sockets) are symmetric
        // about their insertion axis, so ROTATION ABOUT THAT AXIS (yaw) is physically
        // irrelevant — a wheel spun on its hub, or a square block yawed on a round
        // stud, still seats. We therefore gate the snap on the AXIS TILT (how far the
        // held feature's insertion axis is from the mated one), not the full frame
        // angle, which would otherwise reject a perfectly-seated but yawed part.
        let angle = heldPose.angularDistance(to: mate)
        let axisTilt: Double
        if let feature = movingPart.feature(id: step.movingFeatureID) {
            let heldAxis = simd_normalize(heldPose.orientation * feature.localInsertionAxis)
            let mateAxis = simd_normalize(mate.orientation * feature.localInsertionAxis)
            axisTilt = acos(max(-1, min(1, simd_dot(heldAxis, mateAxis))))
        } else {
            axisTilt = angle
        }
        let within = distance <= Self.snapPositionTolerance && axisTilt <= Self.snapOrientationTolerance

        indicator = Indicator(
            partName: movingPart.name,
            distanceMillimeters: distance * 1000,
            angleDegrees: axisTilt * 180 / .pi,
            withinTolerance: within
        )

        if within {
            snap(step: step, entity: held, mate: mate, partName: movingPart.name)
        }
    }

    private func snap(step: AssemblyStep, entity: ModelEntity, mate: Pose, partName: String) {
        scene.weldPart(entity, toWorld: mate)      // exact pose + kinematic + welded
        arm.handOffHeldObject()                    // gripper no longer owns it
        placedWorld[step.movingPartID] = mate      // later pieces may stack on this one
        completedSteps.insert(step.id)
        scene.pulseHighlight(entity)               // brief glow
        arm.audio.playClick()                      // soft click
        lastEvent = "\(partName) attached"
        indicator = nil
    }

    // MARK: - Auto-assemble (Phase 5)

    /// Runs the whole remaining sequence unattended.
    func startAuto() {
        guard !isAutoRunning, loaded else { return }
        metrics = Metrics()
        failureMessage = nil
        isAutoRunning = true
        isPaused = false
        autoTask = Task { await runLoop(singleStep: false) }
    }

    /// Runs exactly one step (the next available), then stops — for watching a
    /// single part go on closely.
    func stepOne() {
        guard !isAutoRunning, loaded else { return }
        failureMessage = nil
        isAutoRunning = true
        isPaused = false
        autoTask = Task { await runLoop(singleStep: true) }
    }

    func togglePause() { if isAutoRunning { isPaused.toggle() } }

    func stopAuto() {
        autoTask?.cancel()
        autoTask = nil
        isAutoRunning = false
        isPaused = false
    }

    private func runLoop(singleStep: Bool) async {
        // Leave the singular home pose before any IK, or the first grasp won't solve.
        if arm.isNearSingularity { await arm.easeToReady() }
        lastToolPlanned = arm.toolWorldPose().position
        while let step = definition.nextStep(completed: completedSteps) {
            if Task.isCancelled { break }
            await waitWhilePaused()
            if Task.isCancelled { break }

            let start = Date()
            let ok = await performStep(step)
            let elapsed = Date().timeIntervalSince(start)
            metrics.perStep.append((definition.part(id: step.movingPartID)?.name ?? step.id, elapsed))
            metrics.totalTime += elapsed

            if !ok { break }          // failure recorded; leave the scene as-is
            if singleStep { break }
        }
        isAutoRunning = false
        isPaused = false
    }

    private func waitWhilePaused() async {
        while isPaused && !Task.isCancelled {
            try? await Task.sleep(for: .milliseconds(80))
        }
    }

    /// One full pick-place-insert-snap cycle. Returns false (and sets
    /// `failureMessage`) if IK is unreachable or the part fails to snap.
    private func performStep(_ step: AssemblyStep) async -> Bool {
        guard let entity = entities[step.movingPartID],
              let targetWorld = placedWorld[step.targetPartID],
              let insertion = definition.insertionPoses(for: step, targetPartWorld: targetWorld, backoff: 0.06)
        else { failureMessage = "Step \(step.id): missing part or target."; return false }

        let center = scene.worldPose(of: entity).position

        // 1. Approach directly above the part (tool down), routing around the others.
        let approachPose = downTool(at: center + SIMD3<Double>(0, 0.22, 0))
        guard await planMove(to: approachPose, excluding: step.movingPartID, status: .movingToObject)
        else { return failIK(step, "approach") }

        // 2. Descend onto it.
        let descendPose = downTool(at: center + SIMD3<Double>(0, 0.02, 0))
        guard await move(to: descendPose)
        else { return failIK(step, "descend") }

        // 3. Close to grab the KNOWN part directly, then capture how it sits in the
        //    tool frame so we can command the PART to any world pose from here on.
        await arm.closeGripperOnPart(entity)
        guard arm.isHolding else {
            metrics.failedSnaps += 1
            failureMessage = "Step \(step.id): failed to grasp the part."
            return false
        }
        let partInTool = arm.toolWorldPose().matrix.inverse * scene.worldPose(of: entity).matrix

        // 4. Lift clear of the tray.
        _ = await move(to: downTool(at: center + SIMD3<Double>(0, 0.24, 0)), status: .movingToDrop)

        // 5. Go to the pre-insert standoff, routing the held part around the others.
        let preInsertPose = toolPose(part: insertion.preInsert, partInTool: partInTool)
        guard await planMove(to: preInsertPose, excluding: step.movingPartID)
        else { return failIK(step, "pre-insert") }

        // 6. CARTESIAN straight-line insert: step the PART pose from pre-insert to
        //    mate, solving IK for the matching tool pose each increment (a joint-
        //    space move would arc the part sideways through the axle). The snap
        //    check welds it the instant it's within tolerance.
        let substeps = 24
        for i in 1...substeps {
            if completedSteps.contains(step.id) { break }
            if Task.isCancelled { return false }
            let t = Double(i) / Double(substeps)
            let partPose = insertion.preInsert.interpolated(to: insertion.mate, t: t)
            let toolTarget = toolPose(part: partPose, partInTool: partInTool)
            // Skip a transiently-unreachable interpolation point rather than
            // aborting — the snap check at the end is the real success gate.
            if let sol = arm.solveWorldPose(toolTarget) {
                addTravel(toolTarget.position)
                arm.setJointsImmediate(sol)
            }
            try? await Task.sleep(for: .milliseconds(28))
        }

        // 7. Confirm it seated. SNAP-FIT FALLBACK: if the live snap check didn't
        //    fire but the part reached its target SPOT, seat it exactly anyway —
        //    the weld sets the precise mated pose, so a slightly-off approach
        //    orientation shouldn't reject a part that's physically there. Only fail
        //    if it genuinely never arrived.
        if !completedSteps.contains(step.id), arm.isHolding,
           scene.worldPose(of: entity).positionDistance(to: insertion.mate) <= Self.snapFallbackPositionTolerance {
            snap(step: step, entity: entity, mate: insertion.mate,
                 partName: definition.part(id: step.movingPartID)?.name ?? step.id)
        }
        guard completedSteps.contains(step.id) else {
            metrics.failedSnaps += 1
            failureMessage = "Step \(step.id): part never reached its target."
            return false
        }

        // 8. Open (already handed off) and retract straight up.
        await arm.openGripperAsync()
        _ = await move(to: downTool(at: insertion.mate.position + SIMD3<Double>(0, 0.22, 0)), status: .idle)
        return true
    }

    // MARK: Auto-assemble helpers

    /// Eases the tool to a world pose, accumulating travel. Returns false if the
    /// pose is unreachable.
    private func move(to toolWorld: Pose, status: ArmStatus? = nil) async -> Bool {
        guard let sol = arm.solveWorldPose(toolWorld) else { return false }
        addTravel(toolWorld.position)
        await arm.easeTo(sol, status: status)
        return true
    }

    /// Like `move`, but plans a COLLISION-FREE path around the other parts so the
    /// arm routes around them instead of sweeping through. `excludedID` is the part
    /// we're grabbing/holding (never an obstacle to itself).
    private func planMove(to toolWorld: Pose, excluding excludedID: String?, status: ArmStatus? = nil) async -> Bool {
        addTravel(toolWorld.position)
        return await arm.planAndMove(toWorldPose: toolWorld,
                                     obstacles: obstacleSpheres(excluding: excludedID),
                                     status: status)
    }

    /// Bounding-sphere obstacles for the planner: every part except the one being
    /// handled and the fixed base (the build target the arm must reach into).
    /// Already-placed parts are included, so later moves route around them too.
    private func obstacleSpheres(excluding excludedID: String?) -> [CollisionSphere] {
        var spheres: [CollisionSphere] = []
        for (id, entity) in entities where id != excludedID && id != definition.basePartID {
            let bounds = entity.visualBounds(relativeTo: nil)
            let radius = simd_length(bounds.extents) / 2
            spheres.append(scene.robotFrameSphere(worldCenter: bounds.center, worldRadius: radius))
        }
        return spheres
    }

    private func failIK(_ step: AssemblyStep, _ phase: String) -> Bool {
        failureMessage = "Step \(step.id): unreachable during \(phase)."
        return false
    }

    /// A tool pose at `position` pointing straight down (approach = −Y world).
    private func downTool(at position: SIMD3<Double>) -> Pose {
        Pose(position: position, orientation: simd_double3x3(columns: (
            SIMD3<Double>(1, 0, 0),   // tool X
            SIMD3<Double>(0, 0, 1),   // tool Y
            SIMD3<Double>(0, -1, 0)   // tool Z (approach, down)
        )))
    }

    /// The tool world pose that places the held PART at `part`, using the grasp
    /// offset captured at pickup: T_tool = T_part · (part-in-tool)⁻¹.
    private func toolPose(part: Pose, partInTool: simd_double4x4) -> Pose {
        Pose(part.matrix * partInTool.inverse)
    }

    private func addTravel(_ p: SIMD3<Double>) {
        if let last = lastToolPlanned { metrics.travelMeters += simd_length(p - last) }
        lastToolPlanned = p
    }

    // MARK: Panel state

    enum StepState { case done, inProgress, ready, blocked }

    /// Per-step display state for the assembly panel, in plan order.
    func stepStates() -> [(id: String, name: String, state: StepState)] {
        let heldPart = arm.heldEntity.flatMap { partID(of: $0) }
        return definition.steps.map { step in
            let name = definition.part(id: step.movingPartID)?.name ?? step.movingPartID
            let state: StepState
            if completedSteps.contains(step.id) {
                state = .done
            } else if heldPart == step.movingPartID
                        && definition.isStepAvailable(step, completed: completedSteps) {
                state = .inProgress
            } else if definition.isStepAvailable(step, completed: completedSteps) {
                state = .ready
            } else {
                state = .blocked
            }
            return (step.id, name, state)
        }
    }

    var isComplete: Bool { definition.isComplete(completed: completedSteps) }
    var completedCount: Int { completedSteps.count }
    var totalSteps: Int { definition.steps.count }

    // MARK: Helpers

    /// The next available step whose moving part matches `partID` (each wheel/body
    /// instance maps to exactly one step, so this is unambiguous).
    private func candidateStep(forMovingPart partID: String) -> AssemblyStep? {
        definition.availableSteps(completed: completedSteps).first { $0.movingPartID == partID }
    }

    /// The assembly part id encoded in an entity's name ("part.<id>"), or nil.
    private func partID(of entity: Entity) -> String? {
        let prefix = "part."
        guard entity.name.hasPrefix(prefix) else { return nil }
        return String(entity.name.dropFirst(prefix.count))
    }
}
