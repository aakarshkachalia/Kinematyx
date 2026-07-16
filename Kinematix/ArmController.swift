//
//  ArmController.swift
//  Kinematix
//
//  Drives interaction: click-to-move (IK) plus an EXPLICIT gripper the user
//  opens/closes. It owns the state machine, runs the IK solver, eases the arm's
//  joints toward solved targets, animates the gripper fingers, and — only when
//  the closing gripper actually contacts an object on both sides — grasps it.
//  It writes joint angles into RobotViewModel (shared with the sliders).
//

import Foundation
import Observation
import RealityKit
import RobotArmKit
import simd

/// Plain-language state shown to the user.
enum ArmStatus: Equatable {
    case idle
    case movingToObject
    case movingToDrop
    case holding(ObjectKind)
    case unreachable
    case replaying

    var label: String {
        switch self {
        case .idle:           return "Idle"
        case .movingToObject: return "Moving to target"
        case .movingToDrop:   return "Moving to drop-off"
        case .holding(let k): return "Holding \(k.label.lowercased())"
        case .unreachable:    return "Target unreachable"
        case .replaying:      return "Replaying sequence"
        }
    }
}

@MainActor
@Observable
final class ArmController {
    private(set) var status: ArmStatus = .idle
    private(set) var toast: String?
    /// Whether the gripper is currently open (drives the button label).
    private(set) var isGripperOpen = true

    // Non-owning references (both live as long as the view).
    private unowned var model: RobotViewModel!
    private unowned var scene: SandboxScene!
    /// The active arm profile lives on the model (so it can be switched, Phase 10).
    private var arm: RobotArm { model.arm }
    let audio = AudioManager()

    /// Mass (kg) of whatever is currently held; 0 when empty-handed (Phase 4).
    private(set) var heldPayloadMass: Double = 0
    /// Whether the gripper currently holds an object (for challenge checks).
    var isHolding: Bool { heldObject != nil }

    // Arm joint animation.
    private var isAnimating = false
    private var startAngles: [Double] = []
    private var endAngles: [Double] = []
    private var elapsed: TimeInterval = 0
    private let moveDuration: TimeInterval = 1.1
    private var onArrival: (() -> Void)?
    private var servoOn = false

    // Gripper animation (finger opening: 1 = open, 0 = closed).
    private var gripperAnimating = false
    private var gripperOpening: Float = 1
    private var gripperFrom: Float = 1
    private var gripperTo: Float = 1
    private var gripperElapsed: TimeInterval = 0
    private let gripperDuration: TimeInterval = 0.35

    private var heldObject: ModelEntity?
    private var heldKind: ObjectKind?
    /// Object frozen in place while the gripper descends onto it, so the descent
    /// can't shove it out of reach before we close. Cleared when grabbed/aborted.
    private var pendingGrasp: ModelEntity?
    private var gripperCompletion: (() -> Void)?
    private var toastTask: Task<Void, Never>?

    // Teach & replay: an ordered list of recorded joint poses.
    private var poses: [[Double]] = []
    private(set) var recordedPoseCount = 0
    /// The recorded poses (radians), for URScript export (Phase 6).
    var recordedPoses: [[Double]] { poses }

    func attach(model: RobotViewModel, scene: SandboxScene) {
        self.model = model
        self.scene = scene
        // A dropped object landing plays a soft thud.
        scene.onObjectLanded = { [weak self] in self?.audio.playThud() }
    }

    // MARK: - Per-frame animation

    func tick(_ dt: TimeInterval) {
        advanceArm(dt)
        advanceGripper(dt)
    }

    private func advanceArm(_ dt: TimeInterval) {
        guard isAnimating else { return }
        if !servoOn { audio.startServo(); servoOn = true }

        elapsed += dt
        let t = min(elapsed / moveDuration, 1)
        let e = Self.easeInOut(Float(t))

        var angles = startAngles
        for i in angles.indices {
            angles[i] = startAngles[i] + (endAngles[i] - startAngles[i]) * Double(e)
        }
        model.jointAngles = angles

        if t >= 1 {
            isAnimating = false
            audio.stopServo(); servoOn = false
            let arrived = onArrival
            onArrival = nil
            arrived?()
        }
    }

    private func advanceGripper(_ dt: TimeInterval) {
        guard gripperAnimating else { return }
        gripperElapsed += dt
        let t = min(gripperElapsed / gripperDuration, 1)
        let e = Self.easeInOut(Float(t))
        gripperOpening = gripperFrom + (gripperTo - gripperFrom) * e
        scene.setFingerOpening(gripperOpening)

        if t >= 1 {
            gripperAnimating = false
            isGripperOpen = gripperTo >= 0.5
            onGripperSettled()
            let completion = gripperCompletion
            gripperCompletion = nil
            completion?()
        }
    }

    // MARK: - Gripper (explicit open/close)

    /// Toggles the gripper (side-panel button / G key).
    func toggleGripper() { setGripper(closed: isGripperOpen) }
    /// Close the gripper (may grasp on settle).
    func closeGripper(_ completion: (() -> Void)? = nil) { setGripper(closed: true, completion: completion) }
    /// Open the gripper (releases anything held on settle).
    func openGripper(_ completion: (() -> Void)? = nil) { setGripper(closed: false, completion: completion) }

    private func setGripper(closed: Bool, completion: (() -> Void)? = nil) {
        guard !gripperAnimating else { completion?(); return }
        var target: Float = closed ? 0 : 1

        // Grab the instant we START to close: freeze the object (kinematic) BEFORE
        // the fingers move, so closing can never shove a dynamic object off the
        // table. Then stop the fingers at the object's width instead of overclosing.
        if closed, heldObject == nil, let object = scene.graspCandidate() {
            performGrab(object)
            target = scene.gripOpening(for: object)
        }

        guard abs(target - gripperOpening) > 0.001 else { completion?(); return }
        audio.playClick()
        gripperCompletion = completion
        gripperFrom = gripperOpening
        gripperTo = target
        gripperElapsed = 0
        gripperAnimating = true
    }

    private func performGrab(_ object: ModelEntity) {
        scene.grab(object)
        heldObject = object
        pendingGrasp = nil   // it's held now, not merely frozen-in-place
        heldKind = ObjectKind(rawValue: object.name.replacingOccurrences(of: "object.", with: "")) ?? .cube
        heldPayloadMass = heldKind?.mass ?? 0
        status = .holding(heldKind ?? .cube)
    }

    private func onGripperSettled() {
        if isGripperOpen {
            // Finished opening: release whatever we held (it falls per Phase 1).
            if let held = heldObject {
                scene.release(held)
                heldObject = nil
                heldKind = nil
                heldPayloadMass = 0
                status = .idle
            }
        } else if heldObject == nil {
            // Closed on empty air (the grab, if any, happened at close-start).
            flashToast("Nothing to grasp")
            abortPendingGrasp()
        }
    }

    /// Un-freezes an object we were about to grasp but didn't (target unreachable,
    /// or the close found nothing), so it behaves as a dynamic body again.
    private func abortPendingGrasp() {
        guard let object = pendingGrasp else { return }
        scene.setFrozen(object, false)
        pendingGrasp = nil
    }

    // MARK: - Click to pick / place (staged: above → down → grip)

    /// Clearance the gripper hovers at directly above the target before descending.
    private let approachHeight: Float = 0.18

    /// A click runs a real motion:
    ///  • Click an OBJECT (empty-handed): open the gripper, move directly above
    ///    it with the fingers ALIGNED to the object, descend, then close to grab.
    ///  • Click anywhere else (empty-handed): move the gripper there.
    ///  • Click while HOLDING: move above the spot, descend, open to release, then
    ///    raise the arm back up (leaving the object behind).
    func handleClick(point: CGPoint, viewSize: CGSize, rig: CameraRig) {
        guard !isAnimating, !gripperAnimating else { return }
        guard let (origin, direction) = rig.ray(viewSize: viewSize, point: point) else { return }
        let hit = scene.pick(origin: origin, direction: direction)

        if isHolding {
            let surface: SIMD3<Float>
            switch hit {
            case .ground(let p), .object(_, let p): surface = p
            case .none: return
            }
            approach(target: surface + SIMD3<Float>(0, 0.06, 0), orientation: nil, status: .movingToDrop) {
                [weak self] above in
                guard let self else { return }
                // Release FIRST, then lift the arm away from the dropped object.
                self.openGripper { [weak self] in self?.raiseTo(above) }
            }
            return
        }

        switch hit {
        case .object(let object, _):
            openGripper()   // ready the gripper the moment you pick an object
            // Freeze the object in place so the descending gripper can't knock it
            // away before closing, and aim at its VISUAL center (drawn prisms are
            // offset from their entity origin).
            scene.setFrozen(object, true)
            pendingGrasp = object
            let target = scene.graspCenter(of: object) + SIMD3<Float>(0, 0.02, 0)
            approach(target: target, orientation: object.orientation(relativeTo: nil), status: .movingToObject) {
                [weak self] _ in self?.closeGripper()
            }
        case .ground(let p):
            // Click anywhere → just take the gripper there.
            approach(target: p + SIMD3<Float>(0, 0.10, 0), orientation: nil, status: .movingToObject) {
                [weak self] _ in self?.status = self?.heldKind.map { .holding($0) } ?? .idle
            }
        case .none:
            return
        }
    }

    /// Two-stage move: to a hover point directly above `target`, then straight
    /// down to it. `orientation` (if given) aligns the whole tool frame — used
    /// for grasps so the fingers line up with the object. `onArrival` receives
    /// the hover point so a caller can raise back up to it.
    private func approach(
        target: SIMD3<Float>, orientation: simd_quatf?,
        status newStatus: ArmStatus, onArrival: @escaping (SIMD3<Float>) -> Void
    ) {
        let above = target + SIMD3<Float>(0, approachHeight, 0)
        let solveFor: (SIMD3<Float>) -> [Double]? = { [weak self] worldPoint in
            guard let self else { return nil }
            if let orientation { return self.solveAligned(worldTarget: worldPoint, objectOrientation: orientation) }
            return self.solve(worldTarget: worldPoint)
        }
        guard let solutionAbove = solveFor(above), let solutionAt = solveFor(target) else {
            status = .unreachable
            flashToast("Target out of reach")
            abortPendingGrasp()   // let a frozen would-be target fall/behave again
            return
        }
        status = newStatus
        beginMove(to: solutionAbove) { [weak self] in
            self?.beginMove(to: solutionAt) { onArrival(above) }
        }
    }

    /// Moves the arm up to a (previously computed) hover point after a release.
    private func raiseTo(_ worldTarget: SIMD3<Float>) {
        guard let solution = solve(worldTarget: worldTarget) else { status = .idle; return }
        beginMove(to: solution) { [weak self] in self?.status = .idle }
    }

    // MARK: - Teach & replay

    /// Records the arm's current joint pose as the next step in the sequence.
    func recordPose() {
        poses.append(model.jointAngles)
        recordedPoseCount = poses.count
    }

    func clearRecording() {
        poses.removeAll()
        recordedPoseCount = 0
    }

    /// Plays the recorded poses back in order, easing between each.
    func replay() {
        guard !isAnimating, !poses.isEmpty else { return }
        status = .replaying
        playPose(0)
    }

    private func playPose(_ index: Int) {
        guard index < poses.count else { status = .idle; return }
        beginMove(to: poses[index]) { [weak self] in
            self?.playPose(index + 1)
        }
    }

    // MARK: - Sequence metrics (Phase 5)

    /// Total end-effector travel distance across the recorded sequence (meters),
    /// summed over each move by running the poses through forward kinematics.
    var sequenceTravelDistance: Double {
        guard poses.count > 1 else { return 0 }
        var total = 0.0
        for i in 1..<poses.count {
            let a = arm.endEffectorPosition(jointAngles: poses[i - 1])
            let b = arm.endEffectorPosition(jointAngles: poses[i])
            total += Double(simd_length(b - a))
        }
        return total
    }

    /// Total playback time (seconds): one eased move per gap between poses.
    var sequencePlaybackTime: Double {
        Double(max(0, poses.count - 1)) * moveDuration
    }

    // MARK: - Helpers

    private func solve(worldTarget: SIMD3<Float>) -> [Double]? {
        let robotTarget = scene.worldToRobot(worldTarget)
        // Approach straight down (tool +Z = base −Z); wrist roll is left free.
        let result = arm.inverseKinematics(
            targetPosition: SIMD3<Double>(robotTarget),
            approachAxis: SIMD3<Double>(0, 0, -1),
            initialGuess: model.jointAngles
        )
        return result.reached ? result.jointAngles : nil
    }

    /// Like `solve`, but pins the FULL tool orientation so the gripper fingers
    /// line up with the object (rather than hitting it at an angle). The finger
    /// axis (tool +X) is aligned with the object's horizontal X direction, the
    /// approach (+Z) points straight down, and Y completes the right-handed frame.
    private func solveAligned(worldTarget: SIMD3<Float>, objectOrientation: simd_quatf) -> [Double]? {
        let robotTarget = scene.worldToRobot(worldTarget)

        // The object's local X in the arm's base frame, flattened to horizontal.
        let objectXWorld = objectOrientation.act(SIMD3<Float>(1, 0, 0))
        let objectXRobot = CoordinateSpace.robotToRealityKit.inverse.act(objectXWorld)
        var x = SIMD3<Double>(Double(objectXRobot.x), Double(objectXRobot.y), 0)
        if simd_length(x) < 1e-4 { x = SIMD3<Double>(1, 0, 0) }
        x = simd_normalize(x)

        let z = SIMD3<Double>(0, 0, -1)               // straight down
        let y = simd_normalize(simd_cross(z, x))
        x = simd_cross(y, z)                          // re-orthonormalize
        let orientation = simd_double3x3(columns: (x, y, z))

        let result = arm.inverseKinematics(
            targetPosition: SIMD3<Double>(robotTarget),
            targetOrientation: orientation,
            initialGuess: model.jointAngles
        )
        return result.reached ? result.jointAngles : nil
    }

    private func beginMove(to target: [Double], then arrival: @escaping () -> Void) {
        startAngles = model.jointAngles
        endAngles = target
        elapsed = 0
        onArrival = arrival
        isAnimating = true
    }

    private func flashToast(_ message: String) {
        toast = message
        toastTask?.cancel()
        toastTask = Task { [weak self] in
            try? await Task.sleep(for: .seconds(1.8))
            guard let self, !Task.isCancelled else { return }
            self.toast = nil
            if case .unreachable = self.status {
                self.status = self.heldKind.map { .holding($0) } ?? .idle
            }
        }
    }

    /// Smoothstep easing (slow-in, slow-out) so moves look intentional.
    private static func easeInOut(_ t: Float) -> Float { t * t * (3 - 2 * t) }
}
