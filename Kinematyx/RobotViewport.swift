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
    var scene: SandboxScene
    var challenges: ChallengeManager
    var drawStore: DrawnShapeStore
    var assembly: AssemblyController

    /// TEMP: guards the headless auto-test so it fires only once.
    nonisolated(unsafe) static var autotestStarted = false

    /// Current lighting/environment preset (Phase 7).
    @State private var environment: EnvironmentPreset = .classroom
    /// Whether the draw-a-shape sheet is showing.
    @State private var showingDraw = false

    private static let accent = Color(red: 0.36, green: 0.36, blue: 0.9)

    /// The SwiftUI background shows through the (transparent) RealityView; each
    /// preset tints the "sky" to match its mood.
    private var worldBackground: Color {
        switch environment {
        case .classroom: return Color(red: 0.93, green: 0.92, blue: 0.89)
        case .factory:   return Color(red: 0.82, green: 0.80, blue: 0.74)
        case .nightShift: return Color(red: 0.09, green: 0.10, blue: 0.14)
        }
    }

    var body: some View {
        GeometryReader { geo in
            ZStack {
                // 3D content. `make` builds the world and wires the per-frame loop.
                RealityView { content in
                    await scene.build(into: &content)
                    controller.attach(model: model, scene: scene)
                    assembly.attach(scene: scene, arm: controller)

                    // One place drives everything each frame.
                    scene.onFrame = { [weak scene] dt in
                        guard let scene else { return }
                        controller.tick(dt)
                        scene.updateArm(with: model.jointAngles)
                        // Snap check runs AFTER updateArm so the held part's world
                        // transform for this frame is current.
                        assembly.tick()
                        scene.updateTrail()
                        challenges.evaluate(scene: scene, controller: controller)
                        if rig.followGripper {
                            scene.updateCameraToGripper()
                        } else {
                            scene.updateCamera(position: rig.position, focus: rig.focus)
                        }
                    }

                    // TEMP DIAGNOSTIC: auto-run the assembly to capture logs headlessly.
                    if ProcessInfo.processInfo.environment["KINEMATYX_AUTOTEST"] != nil, !Self.autotestStarted {
                        Self.autotestStarted = true
                        Task { @MainActor in
                            try? await Task.sleep(for: .seconds(2))
                            assembly.loadCar()
                            try? await Task.sleep(for: .seconds(1))
                            assembly.startAuto()
                        }
                    }
                }

                ViewportInput(
                    rig: rig,
                    onClick: { point, size in
                        controller.handleClick(point: point, viewSize: size, rig: rig)
                    },
                    probeObject: { point, size in
                        guard let (origin, direction) = rig.ray(viewSize: size, point: point) else { return false }
                        return scene.objectUnderRay(origin: origin, direction: direction) != nil
                    },
                    beginObjectDrag: { point, size in
                        guard let (origin, direction) = rig.ray(viewSize: size, point: point) else { return }
                        scene.beginObjectDrag(origin: origin, direction: direction)
                    },
                    updateObjectDrag: { point, size, vertical in
                        guard let (origin, direction) = rig.ray(viewSize: size, point: point) else { return }
                        scene.updateObjectDrag(origin: origin, direction: direction, vertical: vertical)
                    },
                    endObjectDrag: { scene.endObjectDrag() }
                )

                overlays
            }
            .dropDestination(for: SpawnPayload.self) { items, location in
                guard let payload = items.first,
                      let (origin, direction) = rig.ray(viewSize: geo.size, point: location)
                else { return false }
                let hit = scene.surfaceHit(origin: origin, direction: direction)
                    ?? rig.groundHitPoint(viewSize: geo.size, point: location)
                guard let hit else { return false }
                switch payload {
                case .object(let kind):   scene.dropObject(kind, onto: hit)
                case .obstacle(let kind): scene.spawnObstacle(kind, at: hit)
                case .drawn(let id):
                    if let shape = drawStore.shape(id: id) { scene.dropDrawnObject(shape, onto: hit) }
                }
                return true
            }
            .background(worldBackground.animation(.easeInOut(duration: 0.6)))
            .sheet(isPresented: $showingDraw) { DrawingSheet(store: drawStore) }
            // Swap arm profile (Phase 10).
            .onChange(of: model.profileIndex) { _, _ in
                scene.setArm(model.arm, scale: model.displayScale)
            }
            // Apply the lighting preset (Phase 7).
            .onChange(of: environment) { _, preset in
                scene.setEnvironment(preset)
            }
        }
    }

    // MARK: Overlays (status badge, toast, shelf)

    private var overlays: some View {
        ZStack {
            // Status badge + teach/replay bar, top-left.
            VStack(alignment: .leading, spacing: 10) {
                StatusBadge(status: controller.status, accent: Self.accent)
                TeachBar(controller: controller, accent: Self.accent)
                Spacer()
            }
            .frame(maxWidth: .infinity, alignment: .leading)

            // Camera presets + settings, top-right.
            VStack(alignment: .trailing, spacing: 10) {
                CameraPresetsBar { rig.apply($0) }
                SettingsPanel(scene: scene, audio: controller.audio,
                              environment: $environment, accent: Self.accent)
                Spacer()
            }
            .frame(maxWidth: .infinity, alignment: .trailing)

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
                ObjectShelf(store: drawStore,
                            onDraw: { showingDraw = true },
                            onClear: { scene.clearAll() })
            }

            // Live snap-alignment indicator, bottom-right, only while lining up.
            if let indicator = assembly.indicator {
                VStack {
                    Spacer()
                    HStack {
                        Spacer()
                        SnapIndicator(indicator: indicator)
                            .padding(.bottom, 96)
                    }
                }
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
                .font(.callout.weight(.semibold))
                .foregroundStyle(.primary)
                .contentTransition(.numericText())
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 7)
        .background(.thickMaterial, in: Capsule())
        .overlay(Capsule().stroke(.separator, lineWidth: 1))
        .shadow(color: .black.opacity(0.18), radius: 5, y: 1)
    }

    private var dotColor: Color {
        switch status {
        case .idle:                       return .secondary
        case .movingToObject, .movingToDrop, .holding, .replaying: return accent
        case .unreachable:                return .orange
        }
    }
}

// MARK: - Teach & replay

private struct TeachBar: View {
    var controller: ArmController
    let accent: Color

    var body: some View {
        HStack(spacing: 8) {
            Button { controller.recordPose() } label: {
                Label("Record", systemImage: "plus.circle.fill")
            }
            Button { controller.replay() } label: {
                Label("Play", systemImage: "play.fill")
            }
            .disabled(controller.recordedPoseCount == 0)
            Button { controller.clearRecording() } label: {
                Image(systemName: "trash")
            }
            .disabled(controller.recordedPoseCount == 0)

            Text("\(controller.recordedPoseCount) pose\(controller.recordedPoseCount == 1 ? "" : "s")")
                .font(.caption.monospacedDigit().weight(.medium))
                .foregroundStyle(.primary)
        }
        .font(.callout.weight(.semibold))
        .tint(accent)
        .buttonStyle(.borderless)
        .padding(.horizontal, 12)
        .padding(.vertical, 7)
        .background(.thickMaterial, in: Capsule())
        .overlay(Capsule().stroke(.separator, lineWidth: 1))
        .shadow(color: .black.opacity(0.18), radius: 5, y: 1)
    }
}

// MARK: - Camera presets

private struct CameraPresetsBar: View {
    var onSelect: (CameraRig.Preset) -> Void

    private let presets: [(CameraRig.Preset, String, String)] = [
        (.orbit, "Orbit", "arrow.clockwise"),
        (.top, "Top", "arrow.down.to.line"),
        (.front, "Front", "square"),
        (.side, "Side", "square.righthalf.filled"),
        (.gripper, "Gripper", "camera.viewfinder"),
    ]

    var body: some View {
        HStack(spacing: 6) {
            ForEach(presets, id: \.1) { preset, name, icon in
                Button { onSelect(preset) } label: {
                    VStack(spacing: 2) {
                        Image(systemName: icon).font(.body.weight(.semibold))
                        Text(name).font(.caption2.weight(.medium))
                    }
                    .foregroundStyle(.primary)
                    .frame(width: 46, height: 40)
                    .contentShape(Rectangle())
                    .help("\(name) view")
                }
                .buttonStyle(.borderless)
            }
        }
        .padding(.horizontal, 8)
        .padding(.vertical, 5)
        .background(.thickMaterial, in: RoundedRectangle(cornerRadius: 14))
        .overlay(RoundedRectangle(cornerRadius: 14).stroke(.separator, lineWidth: 1))
        .shadow(color: .black.opacity(0.18), radius: 5, y: 1)
    }
}

// MARK: - Physics / settings (collapsible)

private struct SettingsPanel: View {
    let scene: SandboxScene
    let audio: AudioManager
    @Binding var environment: EnvironmentPreset
    let accent: Color

    @State private var expanded = false
    @State private var gravity: Double = 9.81
    @State private var friction: Double = 0.6
    @State private var soundOn = true
    @State private var reachOn = false
    @State private var trailOn = false

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Button {
                withAnimation(.snappy) { expanded.toggle() }
            } label: {
                Label("Settings", systemImage: expanded ? "chevron.down" : "slider.horizontal.3")
                    .font(.callout.weight(.medium))
            }
            .buttonStyle(.borderless)

            if expanded {
                // Phase 7: lighting/environment preset.
                Picker("Environment", selection: $environment) {
                    ForEach(EnvironmentPreset.allCases, id: \.self) { Text($0.rawValue).tag($0) }
                }
                .pickerStyle(.menu)

                Toggle("Reach volume", isOn: $reachOn)
                    .onChange(of: reachOn) { _, v in scene.setReachVisible(v) }
                Toggle("Motion trail", isOn: $trailOn)
                    .onChange(of: trailOn) { _, v in scene.setTrailEnabled(v) }
                Toggle("Sound", isOn: $soundOn)
                    .onChange(of: soundOn) { _, v in audio.isEnabled = v }

                labeledSlider("Gravity", value: $gravity, range: 0...20, unit: "m/s²") {
                    scene.setGravity(Float(gravity))
                }
                labeledSlider("Friction", value: $friction, range: 0...1.5, unit: "") {
                    scene.setFriction(Float(friction))
                }
            }
        }
        .tint(accent)
        .padding(12)
        .frame(width: expanded ? 210 : nil, alignment: .leading)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 12))
        .overlay(RoundedRectangle(cornerRadius: 12).stroke(.separator, lineWidth: 1))
        .shadow(color: .black.opacity(0.08), radius: 4, y: 1)
    }

    private func labeledSlider(
        _ title: String, value: Binding<Double>, range: ClosedRange<Double>,
        unit: String, onChange: @escaping () -> Void
    ) -> some View {
        VStack(alignment: .leading, spacing: 2) {
            HStack {
                Text(title).font(.caption)
                Spacer()
                Text("\(value.wrappedValue, specifier: "%.1f")\(unit)")
                    .font(.caption.monospacedDigit()).foregroundStyle(.secondary)
            }
            Slider(value: value, in: range).onChange(of: value.wrappedValue) { _, _ in onChange() }
        }
    }
}

// MARK: - Snap alignment indicator (Phase 3)

/// Shows how far the held part's mating feature is from its target socket, in
/// both distance and angle, turning green when both are within snap tolerance.
private struct SnapIndicator: View {
    let indicator: AssemblyController.Indicator

    private var tint: Color { indicator.withinTolerance ? .green : Color(red: 0.36, green: 0.36, blue: 0.9) }

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack(spacing: 6) {
                Image(systemName: indicator.withinTolerance ? "checkmark.circle.fill" : "scope")
                    .foregroundStyle(tint)
                Text(indicator.withinTolerance ? "Ready to snap" : "Aligning \(indicator.partName)")
                    .font(.caption.weight(.semibold))
            }
            HStack(spacing: 12) {
                readout("Distance", String(format: "%.0f mm", indicator.distanceMillimeters),
                        ok: indicator.distanceMillimeters <= AssemblyController.snapPositionTolerance * 1000)
                readout("Angle", String(format: "%.0f°", indicator.angleDegrees),
                        ok: indicator.angleDegrees <= AssemblyController.snapOrientationTolerance * 180 / .pi)
            }
        }
        .padding(.horizontal, 14).padding(.vertical, 10)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 12))
        .overlay(RoundedRectangle(cornerRadius: 12).stroke(tint.opacity(0.6), lineWidth: 1.5))
        .shadow(color: .black.opacity(0.12), radius: 6, y: 2)
    }

    private func readout(_ label: String, _ value: String, ok: Bool) -> some View {
        VStack(spacing: 2) {
            Text(value)
                .font(.callout.monospacedDigit().weight(.semibold))
                .foregroundStyle(ok ? .green : .primary)
            Text(label).font(.caption2).foregroundStyle(.secondary)
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
