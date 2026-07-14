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

    var label: String {
        switch self {
        case .idle:           return "Idle"
        case .movingToObject: return "Moving to target"
        case .movingToDrop:   return "Moving to drop-off"
        case .holding(let k): return "Holding \(k.label.lowercased())"
        case .unreachable:    return "Target unreachable"
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
    private let arm = RobotArm.ur5
    let audio = AudioManager()

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

    /// Toggles the gripper. Closing may grasp; opening releases. This is the ONLY
    /// way to grab — proximity never auto-grabs.
    func toggleGripper() {
        guard !gripperAnimating else { return }
        audio.playClick()
        gripperFrom = gripperOpening
        gripperTo = isGripperOpen ? 0 : 1   // open → close, or closed → open
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
                status = .idle
            }
        }
    }

    // MARK: - Click to move (no grabbing here)

    /// Clicking moves the gripper NEAR the target and stops. It never grabs or
    /// releases — that's the gripper's job.
    func handleClick(point: CGPoint, viewSize: CGSize, rig: CameraRig) {
        guard !isAnimating else { return }
        guard let (origin, direction) = rig.ray(viewSize: viewSize, point: point) else { return }

        let target: SIMD3<Float>
        switch scene.pick(origin: origin, direction: direction) {
        case .object(let object, _):
            target = object.position(relativeTo: nil)
        case .ground(let p):
            target = p + SIMD3<Float>(0, 0.08, 0)   // hover just above the surface
        case .none:
            return
        }

        guard let solution = solve(worldTarget: target) else {
            status = .unreachable
            flashToast("Target out of reach")
            return
        }

        status = heldObject != nil ? .movingToDrop : .movingToObject
        beginMove(to: solution) { [weak self] in
            guard let self else { return }
            self.status = self.heldKind.map { .holding($0) } ?? .idle
        }
    }

    // MARK: - Helpers

    private func solve(worldTarget: SIMD3<Float>) -> [Double]? {
        let robotTarget = scene.worldToRobot(worldTarget)
        let result = arm.inverseKinematics(
            targetPosition: SIMD3<Double>(robotTarget),
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
