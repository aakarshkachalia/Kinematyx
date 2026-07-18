//
//  ChallengeManager.swift
//  Kinematyx
//
//  Created by Aakarsh Kachalia on 7/18/26.
//

//
//  ChallengeManager.swift
//  Kinematix
//
//  A lightweight guided-challenge system (Phase 9): a short ordered list of
//  tasks, each with a completion check evaluated from the scene's existing
//  object/obstacle positions and the gripper state. Latching — once done, stays
//  done until reset. No scoring or timers yet.
//

import Foundation
import Observation
import simd

@MainActor
@Observable
final class ChallengeManager {
    struct Challenge: Identifiable {
        let id = UUID()
        let title: String
        var done = false
    }

    private(set) var challenges: [Challenge] = [
        Challenge(title: "Grab any object with the gripper"),
        Challenge(title: "Place a cube on a platform"),
        Challenge(title: "Stack one object on another"),
    ]

    func reset() {
        for i in challenges.indices { challenges[i].done = false }
    }

    /// Re-checks each not-yet-done challenge against the current world. Called
    /// periodically from the frame loop.
    func evaluate(scene: SandboxScene, controller: ArmController) {
        let objects = scene.objectSnapshots()
        let platforms = scene.obstacleSnapshots().filter { $0.kind == .platform }
        let benchTop = scene.benchTopHeight

        markDone(0) { controller.isHolding }

        // A cube sits clearly above the bench, roughly over a platform.
        markDone(1) {
            objects.contains { obj in
                guard obj.kind == .cube, obj.position.y > benchTop + 0.09 else { return false }
                return platforms.contains { horizontalDistance($0.position, obj.position) < 0.22 }
            }
        }

        // Two objects stacked: one resting a little above another, aligned in XZ.
        markDone(2) {
            for i in objects.indices {
                for j in objects.indices where j != i {
                    let dy = objects[i].position.y - objects[j].position.y
                    if dy > 0.03, dy < 0.2,
                       horizontalDistance(objects[i].position, objects[j].position) < 0.09 {
                        return true
                    }
                }
            }
            return false
        }
    }

    private func markDone(_ index: Int, _ check: () -> Bool) {
        guard index < challenges.count, !challenges[index].done else { return }
        if check() { challenges[index].done = true }
    }
}

private func horizontalDistance(_ a: SIMD3<Float>, _ b: SIMD3<Float>) -> Float {
    let dx = a.x - b.x, dz = a.z - b.z
    return (dx * dx + dz * dz).squareRoot()
}
