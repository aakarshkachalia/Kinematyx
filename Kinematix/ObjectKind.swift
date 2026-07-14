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

enum ObjectKind: String, CaseIterable, Codable, Sendable {
    case cube
    case sphere
    case ramp

    /// Human-readable name shown under each shelf item.
    var label: String {
        switch self {
        case .cube:   return "Cube"
        case .sphere: return "Sphere"
        case .ramp:   return "Ramp"
        }
    }
}

extension ObjectKind: Transferable {
    // Carry the object kind as its raw string. `exporting`/`importing` make the
    // value survive the round-trip from the shelf's drag to the viewport's drop.
    static var transferRepresentation: some TransferRepresentation {
        ProxyRepresentation(
            exporting: { $0.rawValue },
            importing: { ObjectKind(rawValue: $0) ?? .cube }
        )
    }
}
