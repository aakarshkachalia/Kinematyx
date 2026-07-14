//
//  RobotViewport.swift
//  Kinematix
//
//  The 3D viewport: a RealityView showing the world and arm, a custom orbit
//  camera, click-to-pick-and-place interaction, a drag-and-drop object shelf,
//  and floating status / toast overlays.
//

import SwiftUI
import RealityKit
import simd

struct RobotViewport: View {
    var model: RobotViewModel
    var rig: CameraRig
    var controller: ArmController

    /// Owns the RealityKit entities. Held as @State so it survives redraws.
    @State private var scene = SandboxScene()

    /// Accent used sparingly for active states.
    private static let accent = Color(red: 0.36, green: 0.36, blue: 0.9)
    /// Warm off-white world background (matches the floor for a seamless horizon).
    private static let worldBackground = Color(red: 0.93, green: 0.92, blue: 0.89)

    var body: some View {
        GeometryReader { geo in
            ZStack {
                // 3D content. `make` builds the world and wires the per-frame loop.
                RealityView { content in
                    await scene.build(into: &content)
                    controller.attach(model: model, scene: scene)

                    // One place drives everything each frame: advance the arm
                    // animation, then sync the rendered arm + camera.
                    scene.onFrame = { [weak scene] dt in
                        guard let scene else { return }
                        controller.tick(dt)
                        scene.updateArm(with: model.jointAngles)
                        scene.updateCamera(position: rig.position, focus: rig.focus)
                    }
                }

                // Transparent overlay that turns mouse/trackpad input into camera
                // moves and clicks into pick/place actions.
                ViewportInput(rig: rig) { point, size in
                    controller.handleClick(point: point, viewSize: size, rig: rig)
                }

                overlays
            }
            // Drop a shelf item: raycast the drop point to the floor and spawn
            // a physics object there (it falls from a small height and settles).
            .dropDestination(for: ObjectKind.self) { items, location in
                guard let kind = items.first,
                      let hit = rig.groundHitPoint(viewSize: geo.size, point: location)
                else { return false }
                scene.spawnObject(kind, at: SIMD3<Float>(hit.x, 0.4, hit.z))
                return true
            }
            .background(Self.worldBackground)
        }
    }

    // MARK: Overlays (status badge, toast, shelf)

    private var overlays: some View {
        ZStack {
            // Status badge, top-left.
            VStack {
                HStack {
                    StatusBadge(status: controller.status, accent: Self.accent)
                    Spacer()
                }
                Spacer()
            }

            // Toast, top-center, only when there's a message.
            if let toast = controller.toast {
                VStack {
                    ToastView(text: toast)
                    Spacer()
                }
                .padding(.top, 8)
                .transition(.move(edge: .top).combined(with: .opacity))
            }

            // Object shelf, bottom-center.
            VStack {
                Spacer()
                ObjectShelf(onClear: { scene.clearObjects() })
            }
        }
        .padding(16)
        .animation(.snappy, value: controller.status)
        .animation(.snappy, value: controller.toast)
    }
}

// MARK: - Status badge

private struct StatusBadge: View {
    let status: ArmStatus
    let accent: Color

    var body: some View {
        HStack(spacing: 8) {
            Circle()
                .fill(dotColor)
                .frame(width: 8, height: 8)
            Text(status.label)
                .font(.callout.weight(.medium))
                .contentTransition(.numericText())
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 7)
        .background(.regularMaterial, in: Capsule())
        .overlay(Capsule().stroke(.separator, lineWidth: 1))
        .shadow(color: .black.opacity(0.08), radius: 4, y: 1)
    }

    private var dotColor: Color {
        switch status {
        case .idle:                       return .secondary
        case .movingToObject, .movingToDrop, .holding: return accent
        case .unreachable:                return .orange
        }
    }
}

// MARK: - Toast

private struct ToastView: View {
    let text: String

    var body: some View {
        Label(text, systemImage: "exclamationmark.triangle.fill")
            .font(.callout.weight(.medium))
            .foregroundStyle(.orange)
            .padding(.horizontal, 14)
            .padding(.vertical, 9)
            .background(.regularMaterial, in: Capsule())
            .overlay(Capsule().stroke(.separator, lineWidth: 1))
            .shadow(color: .black.opacity(0.12), radius: 6, y: 2)
    }
}
