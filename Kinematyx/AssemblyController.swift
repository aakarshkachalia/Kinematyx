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
    /// Orientation tolerance for a snap (radians). Default 15°.
    static let snapOrientationTolerance: Double = 15 * .pi / 180

    // MARK: State

    let definition = ModelCar.definition
    private unowned var scene: SandboxScene!
    private unowned var arm: ArmController!

    /// partID → its scene entity (chassis, wheels, body).
    private var entities: [String: ModelEntity] = [:]
    /// World pose of the fixed base part (chassis).
    private var basePartWorld: Pose?

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

    /// Spawns the chassis (fixed) plus the wheels and body in the tray.
    func loadCar() {
        clear()
        let benchTop = scene.benchTopHeight

        // Everything is grouped in the front-right work area, spaced generously and
        // well clear of the arm's base column (~0.18 m radius) so no part spawns on
        // top of the arm. Loose parts are FROZEN (kinematic) so the arm can't knock
        // them off or bump them out of place — each becomes movable only once it's
        // grasped, and falls under gravity if a placement is released short.

        // Parts tray: a mat under the loose parts (wheels + body).
        scene.spawnTray(center: SIMD3<Float>(0.48, benchTop + 0.003, 0.42),
                        size: SIMD2<Float>(0.64, 1.00))

        // Chassis: fixed build site, nearest the arm but still clear of the base.
        let chassisY = benchTop + Float(CarGeometry.chassisHeight) / 2
        let chassisPose = Pose(position: SIMD3<Double>(0.46, Double(chassisY), -0.28))
        entities[ModelCar.chassisID] = scene.spawnChassis(at: chassisPose)
        basePartWorld = chassisPose

        // Wheels: laid flat in a widely-spaced 2×2 grid on the tray.
        let wheelY = benchTop + Float(CarGeometry.wheelThickness) / 2
        let wheelSpots: [SIMD3<Float>] = [
            SIMD3<Float>(0.30, wheelY, 0.40),
            SIMD3<Float>(0.66, wheelY, 0.40),
            SIMD3<Float>(0.30, wheelY, 0.76),
            SIMD3<Float>(0.66, wheelY, 0.76),
        ]
        for (i, w) in ModelCar.wheels.enumerated() {
            let wheel = scene.spawnWheel(id: w.id, at: wheelSpots[i])
            scene.setFrozen(wheel, true)          // sit still until grasped
            entities[w.id] = wheel
        }

        // Body shell: on the tray between the wheels and the chassis, clear of both.
        let bodyY = benchTop + Float(CarGeometry.bodyHeight) / 2
        let body = scene.spawnBody(at: SIMD3<Float>(0.46, bodyY, 0.06))
        scene.setFrozen(body, true)
        entities[ModelCar.bodyID] = body

        completedSteps = []
        lastEvent = nil
        indicator = nil
        failureMessage = nil
        metrics = Metrics()
        loaded = true
    }

    /// Returns everything to the tray and clears progress.
    func reset() {
        stopAuto()
        guard loaded else { return }
        loadCar()
    }

    private func clear() {
        for entity in entities.values { entity.removeFromParent() }
        scene.clearAssembly()
        entities = [:]
        basePartWorld = nil
        loaded = false
    }

    // MARK: Per-frame snap check

    func tick() {
        guard loaded,
              let held = arm.heldEntity,
              let partID = partID(of: held),
              let step = candidateStep(forMovingPart: partID),
              let base = basePartWorld,
              let mate = definition.mateWorldPose(for: step, targetPartWorld: base),
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
        let angle = heldPose.angularDistance(to: mate)
        let within = distance <= Self.snapPositionTolerance && angle <= Self.snapOrientationTolerance

        indicator = Indicator(
            partName: movingPart.name,
            distanceMillimeters: distance * 1000,
            angleDegrees: angle * 180 / .pi,
            withinTolerance: within
        )

        if within {
            snap(step: step, entity: held, mate: mate, partName: movingPart.name)
        }
    }

    private func snap(step: AssemblyStep, entity: ModelEntity, mate: Pose, partName: String) {
        scene.weldPart(entity, toWorld: mate)      // exact pose + kinematic + welded
        arm.handOffHeldObject()                    // gripper no longer owns it
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
        guard let entity = entities[step.movingPartID], let base = basePartWorld,
              let insertion = definition.insertionPoses(for: step, targetPartWorld: base, backoff: 0.06)
        else { failureMessage = "Step \(step.id): missing part or base."; return false }

        let center = scene.worldPose(of: entity).position

        // 1. Approach directly above the part (tool down), routing around the others.
        guard await planMove(to: downTool(at: center + SIMD3<Double>(0, 0.22, 0)),
                             excluding: step.movingPartID, status: .movingToObject)
        else { return failIK(step, "approach") }

        // 2. Descend onto it.
        guard await move(to: downTool(at: center + SIMD3<Double>(0, 0.02, 0)))
        else { return failIK(step, "descend") }

        // 3. Close to grab, then capture how the part sits in the tool frame so we
        //    can command the PART to any world pose from here on.
        await arm.closeGripperAsync()
        guard arm.isHolding else {
            metrics.failedSnaps += 1
            failureMessage = "Step \(step.id): failed to grasp the part."
            return false
        }
        let partInTool = arm.toolWorldPose().matrix.inverse * scene.worldPose(of: entity).matrix

        // 4. Lift clear of the tray.
        _ = await move(to: downTool(at: center + SIMD3<Double>(0, 0.24, 0)), status: .movingToDrop)

        // 5. Go to the pre-insert standoff, routing the held part around the others.
        guard await planMove(to: toolPose(part: insertion.preInsert, partInTool: partInTool),
                             excluding: step.movingPartID)
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

        // 7. Confirm it snapped.
        guard completedSteps.contains(step.id) else {
            metrics.failedSnaps += 1
            failureMessage = "Step \(step.id): part never came within snap tolerance."
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
    /// handled and the chassis (the build target the arm must reach into). Already-
    /// placed wheels are included, so later moves avoid them too.
    private func obstacleSpheres(excluding excludedID: String?) -> [CollisionSphere] {
        var spheres: [CollisionSphere] = []
        for (id, entity) in entities where id != excludedID && id != ModelCar.chassisID {
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
