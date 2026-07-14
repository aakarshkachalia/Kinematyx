//
//  ObjectKind.swift
//  Kinematix
//
//  The kinds of physics objects the user can drag from the shelf into the world.
//  This type is shared by the shelf UI (which drags it) and SandboxScene (which
//  spawns the matching entity). It's Transferable so SwiftUI drag-and-drop can
//  carry it from the shelf to the viewport.
//

import Foundation
import CoreTransferable
import RealityKit
import simd

/// What a click-raycast hit in the 3D scene.
enum PickHit {
    /// A dropped object (and the world point where the ray struck it).
    case object(ModelEntity, SIMD3<Float>)
    /// The floor (and the world point where the ray struck it).
    case ground(SIMD3<Float>)
    /// Nothing relevant.
    case none
}

/// Dynamic pickup objects.
enum ObjectKind: String, CaseIterable, Codable, Sendable {
    case cube
    case sphere
    case ramp

    var label: String {
        switch self {
        case .cube:   return "Cube"
        case .sphere: return "Sphere"
        case .ramp:   return "Ramp"
        }
    }

    /// Approximate mass in kilograms, used for payload/torque estimation (Phase 4).
    var mass: Double {
        switch self {
        case .cube:   return 0.5
        case .sphere: return 0.3
        case .ramp:   return 0.4
        }
    }
}

/// Lighting/environment presets (Phase 7).
enum EnvironmentPreset: String, CaseIterable, Sendable {
    case classroom = "Classroom"
    case factory = "Factory"
    case nightShift = "Night shift"
}

/// Static obstacles / platforms (they don't fall; things collide against them).
enum ObstacleKind: String, CaseIterable, Codable, Sendable {
    case platform
    case wall
    case ramp

    var label: String {
        switch self {
        case .platform: return "Platform"
        case .wall:     return "Wall"
        case .ramp:     return "Ramp"
        }
    }
}

/// Which shelf a dragged item came from — carried across the drag-and-drop.
enum SpawnPayload: Codable, Sendable {
    case object(ObjectKind)
    case obstacle(ObstacleKind)
    case drawn(String)   // id of a DrawnShape in the store

    /// Compact string codec for the drag pasteboard.
    var code: String {
        switch self {
        case .object(let k):   return "object:\(k.rawValue)"
        case .obstacle(let k): return "obstacle:\(k.rawValue)"
        case .drawn(let id):   return "drawn:\(id)"
        }
    }

    init?(code: String) {
        let parts = code.split(separator: ":", maxSplits: 1).map(String.init)
        guard parts.count == 2 else { return nil }
        switch parts[0] {
        case "object":   if let k = ObjectKind(rawValue: parts[1])   { self = .object(k);   return }
        case "obstacle": if let k = ObstacleKind(rawValue: parts[1]) { self = .obstacle(k); return }
        case "drawn":    self = .drawn(parts[1]); return
        default: break
        }
        return nil
    }
}

extension SpawnPayload: Transferable {
    static var transferRepresentation: some TransferRepresentation {
        ProxyRepresentation(
            exporting: { $0.code },
            importing: { SpawnPayload(code: $0) ?? .object(.cube) }
        )
    }
}
