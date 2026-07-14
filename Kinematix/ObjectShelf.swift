//
//  ObjectShelf.swift
//  Kinematix
//
//  A floating shelf of draggable templates at the bottom of the viewport, with a
//  toggle between dynamic "Objects" (things to pick up) and static "Obstacles"
//  (platforms/walls/ramps). Each item is a simple 2D glyph the user drags into
//  the world to spawn the matching entity.
//

import SwiftUI

struct ObjectShelf: View {
    /// Removes everything the user has placed.
    var onClear: () -> Void

    private enum Mode: String, CaseIterable { case objects = "Objects", obstacles = "Obstacles" }
    @State private var mode: Mode = .objects

    var body: some View {
        VStack(spacing: 8) {
            Picker("", selection: $mode) {
                ForEach(Mode.allCases, id: \.self) { Text($0.rawValue).tag($0) }
            }
            .pickerStyle(.segmented)
            .labelsHidden()
            .frame(width: 220)

            HStack(spacing: 14) {
                if mode == .objects {
                    ForEach(ObjectKind.allCases, id: \.self) { kind in
                        ShelfItem(label: kind.label, glyph: glyph(for: kind))
                            .draggable(SpawnPayload.object(kind))
                    }
                } else {
                    ForEach(ObstacleKind.allCases, id: \.self) { kind in
                        ShelfItem(label: kind.label, glyph: obstacleGlyph(for: kind))
                            .draggable(SpawnPayload.obstacle(kind))
                    }
                }

                Divider().frame(height: 40)

                Button(role: .destructive, action: onClear) {
                    Label("Clear", systemImage: "trash").font(.callout.weight(.medium))
                }
                .buttonStyle(.borderless)
            }
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 10)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 18))
        .overlay(RoundedRectangle(cornerRadius: 18).stroke(.separator, lineWidth: 1))
        .shadow(radius: 8, y: 2)
        .animation(.snappy, value: mode)
    }

    @ViewBuilder private func glyph(for kind: ObjectKind) -> some View {
        let fill = LinearGradient(colors: [Color(white: 0.85), Color(white: 0.6)],
                                  startPoint: .top, endPoint: .bottom)
        switch kind {
        case .cube:   RoundedRectangle(cornerRadius: 6).fill(fill)
        case .sphere: Circle().fill(fill)
        case .ramp:   RampGlyph().fill(fill)
        }
    }

    @ViewBuilder private func obstacleGlyph(for kind: ObstacleKind) -> some View {
        let fill = LinearGradient(colors: [Color(white: 0.7), Color(white: 0.5)],
                                  startPoint: .top, endPoint: .bottom)
        switch kind {
        case .platform: RoundedRectangle(cornerRadius: 3).fill(fill).frame(height: 16)
        case .wall:     RoundedRectangle(cornerRadius: 3).fill(fill).frame(width: 14)
        case .ramp:     RampGlyph().fill(fill)
        }
    }
}

/// One draggable template: a thumbnail glyph plus its name.
private struct ShelfItem<Glyph: View>: View {
    let label: String
    let glyph: Glyph

    var body: some View {
        VStack(spacing: 4) {
            glyph.frame(width: 40, height: 40)
            Text(label).font(.caption2).foregroundStyle(.secondary)
        }
        .padding(6)
        .contentShape(Rectangle())
        .help("Drag \(label) into the world")
    }
}

/// A right-triangle glyph used for ramp thumbnails.
private struct RampGlyph: Shape {
    func path(in rect: CGRect) -> Path {
        var p = Path()
        p.move(to: CGPoint(x: rect.minX, y: rect.maxY))
        p.addLine(to: CGPoint(x: rect.maxX, y: rect.maxY))
        p.addLine(to: CGPoint(x: rect.maxX, y: rect.minY))
        p.closeSubpath()
        return p
    }
}
