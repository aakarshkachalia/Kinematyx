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

    func attach(scene: SandboxScene, arm: ArmController) {
        self.scene = scene
        self.arm = arm
    }

    // MARK: Loading / reset

    /// Spawns the chassis (fixed) plus the wheels and body in the tray.
    func loadCar() {
        clear()
        let benchTop = scene.benchTopHeight

        // Chassis: fixed, in front of the arm.
        let chassisY = benchTop + Float(CarGeometry.chassisHeight) / 2
        let chassisPose = Pose(position: SIMD3<Double>(0.34, Double(chassisY), 0.0))
        entities[ModelCar.chassisID] = scene.spawnChassis(at: chassisPose)
        basePartWorld = chassisPose

        // Wheels: laid flat in a tray row, reachable and not stacked.
        let wheelY = benchTop + Float(CarGeometry.wheelThickness) / 2
        let zs: [Float] = [-0.18, -0.06, 0.06, 0.18]
        for (i, w) in ModelCar.wheels.enumerated() {
            entities[w.id] = scene.spawnWheel(id: w.id, at: SIMD3<Float>(0.16, wheelY, zs[i]))
        }

        // Body shell: off to the side.
        let bodyY = benchTop + Float(CarGeometry.bodyHeight) / 2
        entities[ModelCar.bodyID] = scene.spawnBody(at: SIMD3<Float>(0.16, bodyY, 0.30))

        completedSteps = []
        lastEvent = nil
        indicator = nil
        loaded = true
    }

    /// Returns everything to the tray and clears progress.
    func reset() {
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
              let socket = definition.targetFeatureWorldPose(for: step, targetPartWorld: base),
              let movingPart = definition.part(id: partID),
              let feature = movingPart.feature(id: step.movingFeatureID)
        else {
            indicator = nil
            return
        }

        // Where the held part's mating feature currently sits, from its live world
        // transform (the part rigidly tracks the gripper).
        let featureWorld = Pose(held.transformMatrix(relativeTo: nil).assemblyMatrix) * feature.localPose

        let distance = featureWorld.positionDistance(to: socket)
        let angle = featureWorld.angularDistance(to: socket)
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
