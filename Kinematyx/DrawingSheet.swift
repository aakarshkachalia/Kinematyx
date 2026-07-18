//
//  DrawingSheet.swift
//  Kinematyx
//
//  Created by Aakarsh Kachalia on 7/18/26.
//

//
//  DrawingSheet.swift
//  Kinematix
//
//  A simple freeform drawing canvas. The user sketches an outline with the
//  mouse/trackpad; "Add to Shelf" stores it, after which it can be dragged into
//  the world as a 3D extruded object as many times as they like.
//

import SwiftUI

struct DrawingSheet: View {
    var store: DrawnShapeStore
    @Environment(\.dismiss) private var dismiss

    @State private var points: [CGPoint] = []
    @State private var canvasSize: CGSize = .zero

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Draw a Shape").font(.title2.bold())
            Text("Sketch an outline. It becomes a 3D object (the filled convex outline) you can drag into the world.")
                .font(.caption).foregroundStyle(.secondary)

            canvas

            HStack {
                Button("Clear") { points.removeAll() }
                    .disabled(points.isEmpty)
                Spacer()
                Button("Cancel") { dismiss() }
                Button("Add to Shelf") {
                    store.add(canvasPoints: points, canvasSize: canvasSize)
                    dismiss()
                }
                .buttonStyle(.borderedProminent)
                .tint(Theme.accent)
                .disabled(points.count < 3)
                .keyboardShortcut(.defaultAction)
            }
        }
        .padding(20)
        .frame(width: 420, height: 460)
        .fontDesign(.rounded)
    }

    private var canvas: some View {
        Canvas { context, _ in
            guard let first = points.first else { return }
            var path = Path()
            path.move(to: first)
            for p in points.dropFirst() { path.addLine(to: p) }
            context.stroke(path, with: .color(Theme.accent), style: StrokeStyle(lineWidth: 3, lineCap: .round, lineJoin: .round))
        }
        .background {
            GeometryReader { geo in
                RoundedRectangle(cornerRadius: 10)
                    .fill(.background.opacity(0.5))
                    .onAppear { canvasSize = geo.size }
                    .onChange(of: geo.size) { _, s in canvasSize = s }
            }
        }
        .overlay(RoundedRectangle(cornerRadius: 10).stroke(.separator, lineWidth: 1))
        .contentShape(Rectangle())
        .gesture(
            DragGesture(minimumDistance: 0)
                .onChanged { points.append($0.location) }
        )
    }
}
