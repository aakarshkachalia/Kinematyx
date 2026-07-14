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
        }
    }

    // MARK: - Gripper (explicit open/close)

    /// Toggles the gripper (side-panel button / G key).
    func toggleGripper() { setGripper(closed: isGripperOpen) }
    /// Close the gripper (may grasp on settle).
    func closeGripper() { setGripper(closed: true) }
    /// Open the gripper (releases anything held on settle).
    func openGripper() { setGripper(closed: false) }

    private func setGripper(closed: Bool) {
        guard !gripperAnimating else { return }
        let target: Float = closed ? 0 : 1
        guard abs(target - gripperOpening) > 0.001 else { return }   // already there
        audio.playClick()
        gripperFrom = gripperOpening
        gripperTo = target
        gripperElapsed = 0
        gripperAnimating = true
    }

    private func onGripperSettled() {
        if !isGripperOpen {
            // Just finished closing: grasp only if both fingers touched an object.
            guard heldObject == nil else { return }
            if let object = scene.attemptGrasp() {
                scene.grab(object)
                heldObject = object
                heldKind = ObjectKind(rawValue: object.name.replacingOccurrences(of: "object.", with: "")) ?? .cube
                heldPayloadMass = heldKind?.mass ?? 0
                status = .holding(heldKind!)
            } else {
                flashToast("Nothing to grasp")
            }
        } else {
            // Just finished opening: release whatever we held (it falls per Phase 1).
            if let held = heldObject {
                scene.release(held)
                heldObject = nil
                heldKind = nil
                heldPayloadMass = 0
                status = .idle
            }
        }
    }

    // MARK: - Click to pick / place (staged: above → down → grip)

    /// Clearance the gripper hovers at directly above the target before descending.
    private let approachHeight: Float = 0.18

    /// A click runs a real pick/place motion: move DIRECTLY ABOVE the target,
    /// descend straight down onto it, then close (to grab) or open (to release)
    /// the gripper — mirroring how a real arm approaches from the top.
    func handleClick(point: CGPoint, viewSize: CGSize, rig: CameraRig) {
        guard !isAnimating, !gripperAnimating else { return }
        guard let (origin, direction) = rig.ray(viewSize: viewSize, point: point) else { return }
        let hit = scene.pick(origin: origin, direction: direction)

        if isHolding {
            // Place: descend over the clicked surface, then open to drop.
            let surface: SIMD3<Float>
            switch hit {
            case .ground(let p), .object(_, let p): surface = p
            case .none: return
            }
            approach(target: surface + SIMD3<Float>(0, 0.06, 0), status: .movingToDrop) {
                [weak self] in self?.openGripper()
            }
        } else {
            // Pick: only clicking an object starts a pick.
            guard case let .object(object, _) = hit else { return }
            let center = object.position(relativeTo: nil) + SIMD3<Float>(0, 0.02, 0)
            approach(target: center, status: .movingToObject) {
                [weak self] in self?.closeGripper()
            }
        }
    }

    /// Two-stage move: to a hover point directly above `target`, then straight
    /// down to `target`; runs `onArrival` (grip/release) once landed. Fails
    /// cleanly if either stage is unreachable.
    private func approach(target: SIMD3<Float>, status newStatus: ArmStatus, onArrival: @escaping () -> Void) {
        let above = target + SIMD3<Float>(0, approachHeight, 0)
        guard let solutionAbove = solve(worldTarget: above),
              let solutionAt = solve(worldTarget: target) else {
            status = .unreachable
            flashToast("Target out of reach")
            return
        }
        status = newStatus
        beginMove(to: solutionAbove) { [weak self] in
            guard let self else { return }
            self.beginMove(to: solutionAt) { onArrival() }
        }
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
        // Always approach from directly above: the tool's approach axis (+Z)
        // points along the base frame's −Z, which is straight down.
        let result = arm.inverseKinematics(
            targetPosition: SIMD3<Double>(robotTarget),
            approachAxis: SIMD3<Double>(0, 0, -1),
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
