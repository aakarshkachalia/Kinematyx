//
//  ContentView.swift
//  Kinematix
//
//  Top-level layout: the 3D viewport fills the window; a fixed-width side panel
//  holds the joint sliders plus the educational readouts (DH table, torque,
//  singularity, sequence metrics, URScript export) and the challenge list.
//

import SwiftUI
import AppKit
import UniformTypeIdentifiers
import RobotArmKit

/// Shared visual theme. One accent color, everything else neutral.
enum Theme {
    static let accent = Color(red: 0.36, green: 0.36, blue: 0.9)     // indigo
    static let accentSoft = accent.opacity(0.14)
    static let cardCorner: CGFloat = 12
}

struct ContentView: View {
    @State private var model = RobotViewModel()
    @State private var rig = CameraRig()
    @State private var controller = ArmController()
    @State private var scene = SandboxScene()
    @State private var challenges = ChallengeManager()
    @State private var drawStore = DrawnShapeStore()

    var body: some View {
        HStack(spacing: 0) {
            RobotViewport(model: model, rig: rig, controller: controller,
                          scene: scene, challenges: challenges, drawStore: drawStore)
                .frame(maxWidth: .infinity, maxHeight: .infinity)

            ControlPanel(model: model, controller: controller,
                         scene: scene, challenges: challenges)
                .frame(width: 320)
        }
        .frame(minWidth: 1040, minHeight: 720)
        .fontDesign(.rounded)
    }
}

// MARK: - Control panel

private struct ControlPanel: View {
    var model: RobotViewModel
    var controller: ArmController
    var scene: SandboxScene
    var challenges: ChallengeManager

    @State private var showJointFrames = true
    @State private var showExport = false

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 18) {
                header
                armPicker

                VStack(spacing: 10) {
                    ForEach(model.arm.joints.indices, id: \.self) { index in
                        JointRow(model: model, index: index)
                    }
                }

                gripperButton
                resetButton
                jointFramesToggle

                if model.isNearSingularity { singularityBadge }

                endEffectorCard
                TorqueSection(model: model, controller: controller)
                DHTableSection(model: model)
                SequenceSection(controller: controller, showExport: $showExport)
                ChallengeSection(challenges: challenges)
            }
            .padding(20)
        }
        .background(.regularMaterial)
        .overlay(alignment: .leading) { Divider() }
        .shadow(color: .black.opacity(0.10), radius: 10, x: -2, y: 0)
        .sheet(isPresented: $showExport) {
            URScriptExportView(script: URScript.program(poses: controller.recordedPoses))
        }
    }

    private var header: some View {
        VStack(alignment: .leading, spacing: 2) {
            Text("Kinematix").font(.largeTitle.bold())
            Text("\(model.arm.name) · 6-DOF kinematics")
                .font(.caption).foregroundStyle(.secondary)
        }
    }

    // Phase 10: choose the arm profile.
    private var armPicker: some View {
        Picker("Arm", selection: Binding(
            get: { model.profileIndex },
            set: { model.selectProfile($0) }
        )) {
            ForEach(RobotArm.allProfiles.indices, id: \.self) { i in
                Text(RobotArm.allProfiles[i].name).tag(i)
            }
        }
        .pickerStyle(.segmented)
    }

    private var gripperButton: some View {
        Button { controller.toggleGripper() } label: {
            Label(controller.isGripperOpen ? "Close Gripper" : "Open Gripper",
                  systemImage: controller.isGripperOpen ? "circle.grid.cross.fill" : "circle.grid.cross")
                .font(.body.weight(.semibold))
                .frame(maxWidth: .infinity).padding(.vertical, 4)
        }
        .buttonStyle(.borderedProminent).tint(Theme.accent).controlSize(.large)
        .keyboardShortcut("g", modifiers: [])
        .animation(.snappy, value: controller.isGripperOpen)
    }

    private var resetButton: some View {
        Button { withAnimation(.snappy) { model.resetToHome() } } label: {
            Label("Reset to Home", systemImage: "house.fill")
                .font(.body.weight(.semibold))
                .frame(maxWidth: .infinity).padding(.vertical, 4)
        }
        .buttonStyle(.bordered).controlSize(.large)
    }

    // Phase 1
    private var jointFramesToggle: some View {
        Toggle("Show joint frames", isOn: $showJointFrames)
            .tint(Theme.accent)
            .onChange(of: showJointFrames) { _, v in scene.setJointFramesVisible(v) }
    }

    // Phase 3
    private var singularityBadge: some View {
        HStack(alignment: .top, spacing: 8) {
            Image(systemName: "exclamationmark.triangle.fill").foregroundStyle(.orange)
            Text("Near singularity: some joints are aligned, so motion here can be unpredictable.")
                .font(.caption)
        }
        .padding(10)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color.orange.opacity(0.12), in: RoundedRectangle(cornerRadius: Theme.cardCorner))
        .transition(.opacity)
    }

    private var endEffectorCard: some View {
        let p = model.endEffectorPosition
        return VStack(alignment: .leading, spacing: 10) {
            Label("End Effector", systemImage: "scope")
                .font(.subheadline.weight(.semibold)).foregroundStyle(.secondary)
            HStack(spacing: 8) {
                AxisReadout(name: "X", value: p.x)
                AxisReadout(name: "Y", value: p.y)
                AxisReadout(name: "Z", value: p.z)
            }
        }
        .padding(14).frame(maxWidth: .infinity, alignment: .leading)
        .background(Theme.accentSoft, in: RoundedRectangle(cornerRadius: Theme.cardCorner))
    }
}

// MARK: - Joint row

private struct JointRow: View {
    var model: RobotViewModel
    let index: Int

    var body: some View {
        let joint = model.arm.joints[index]
        let minDeg = joint.minAngle * 180 / .pi
        let maxDeg = joint.maxAngle * 180 / .pi
        let degrees = Binding<Double>(
            get: { model.jointAngles[index] * 180 / .pi },
            set: { model.jointAngles[index] = $0 * .pi / 180 }
        )

        return VStack(spacing: 6) {
            HStack(spacing: 10) {
                Text("\(index + 1)").font(.footnote.weight(.bold)).foregroundStyle(.white)
                    .frame(width: 22, height: 22).background(Theme.accent, in: Circle())
                Text("Joint \(index + 1)").font(.subheadline.weight(.medium))
                Spacer()
                Text("\(degrees.wrappedValue, specifier: "%.0f")°")
                    .font(.caption.monospacedDigit().weight(.semibold)).foregroundStyle(Theme.accent)
                    .padding(.horizontal, 8).padding(.vertical, 3)
                    .background(Theme.accentSoft, in: Capsule())
            }
            Slider(value: degrees, in: minDeg...maxDeg).tint(Theme.accent)
        }
        .padding(12)
        .background(.background.opacity(0.4), in: RoundedRectangle(cornerRadius: Theme.cardCorner))
        .overlay(RoundedRectangle(cornerRadius: Theme.cardCorner).stroke(.separator.opacity(0.5), lineWidth: 1))
    }
}

private struct AxisReadout: View {
    let name: String
    let value: Double
    var body: some View {
        VStack(spacing: 3) {
            Text(name).font(.caption2.weight(.bold)).foregroundStyle(.secondary)
            Text("\(value, specifier: "%.3f")").font(.caption.monospacedDigit())
        }
        .frame(maxWidth: .infinity).padding(.vertical, 8)
        .background(.background.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }
}

// MARK: - Phase 4: torque

private struct TorqueSection: View {
    var model: RobotViewModel
    var controller: ArmController

    var body: some View {
        let torques = model.jointTorques(payloadMass: controller.heldPayloadMass)
        let limits = model.torqueLimits

        return VStack(alignment: .leading, spacing: 8) {
            Label("Estimated Torque", systemImage: "gauge.with.needle")
                .font(.subheadline.weight(.semibold)).foregroundStyle(.secondary)

            ForEach(torques.indices, id: \.self) { i in
                let limit = i < limits.count ? limits[i] : 0
                TorqueBar(joint: i + 1, torque: abs(torques[i]), limit: limit)
            }

            Text("Torque is higher when the arm is extended — that's why real robots often work with the elbow bent.")
                .font(.caption2).foregroundStyle(.secondary)
        }
        .padding(14).frame(maxWidth: .infinity, alignment: .leading)
        .background(.background.opacity(0.4), in: RoundedRectangle(cornerRadius: Theme.cardCorner))
    }
}

private struct TorqueBar: View {
    let joint: Int
    let torque: Double
    let limit: Double

    private var fraction: Double { limit > 0 ? min(torque / limit, 1.2) : 0 }
    private var over: Bool { limit > 0 && torque > limit }

    var body: some View {
        VStack(spacing: 2) {
            HStack {
                Text("J\(joint)").font(.caption2.monospacedDigit()).foregroundStyle(.secondary)
                Spacer()
                Text("\(torque, specifier: "%.0f") / \(limit, specifier: "%.0f") N·m")
                    .font(.caption2.monospacedDigit())
                    .foregroundStyle(over ? .orange : .secondary)
                if over { Image(systemName: "exclamationmark.triangle.fill").font(.caption2).foregroundStyle(.orange) }
            }
            GeometryReader { geo in
                ZStack(alignment: .leading) {
                    Capsule().fill(.quaternary)
                    Capsule().fill(over ? Color.orange : Theme.accent)
                        .frame(width: geo.size.width * CGFloat(min(fraction, 1)))
                }
            }
            .frame(height: 5)
        }
    }
}

// MARK: - Phase 2: DH table

private struct DHTableSection: View {
    var model: RobotViewModel
    @State private var expanded = false

    var body: some View {
        DisclosureGroup(isExpanded: $expanded) {
            VStack(alignment: .leading, spacing: 4) {
                Text("These parameters define how each joint connects to the next, using the Denavit–Hartenberg convention.")
                    .font(.caption2).foregroundStyle(.secondary).padding(.bottom, 4)

                HStack {
                    Text("J").frame(width: 18, alignment: .leading)
                    Text("α°").frame(maxWidth: .infinity)
                    Text("a").frame(maxWidth: .infinity)
                    Text("d").frame(maxWidth: .infinity)
                    Text("θ°").frame(maxWidth: .infinity)
                }
                .font(.caption2.weight(.bold)).foregroundStyle(.secondary)

                ForEach(model.dhRows, id: \.index) { row in
                    HStack {
                        Text("\(row.index)").frame(width: 18, alignment: .leading)
                        Text("\(row.alpha * 180 / .pi, specifier: "%.0f")").frame(maxWidth: .infinity)
                        Text("\(row.a, specifier: "%.3f")").frame(maxWidth: .infinity)
                        Text("\(row.d, specifier: "%.3f")").frame(maxWidth: .infinity)
                        Text("\(row.theta * 180 / .pi, specifier: "%.0f")").frame(maxWidth: .infinity)
                    }
                    .font(.caption2.monospacedDigit())
                }
            }
            .padding(.top, 6)
        } label: {
            Label("DH Parameters", systemImage: "tablecells")
                .font(.subheadline.weight(.semibold))
        }
        .tint(Theme.accent)
        .padding(14)
        .background(.background.opacity(0.4), in: RoundedRectangle(cornerRadius: Theme.cardCorner))
    }
}

// MARK: - Phase 5/6: sequence metrics + export

private struct SequenceSection: View {
    var controller: ArmController
    @Binding var showExport: Bool

    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            Label("Sequence", systemImage: "waveform.path.ecg")
                .font(.subheadline.weight(.semibold)).foregroundStyle(.secondary)

            HStack(spacing: 8) {
                metric("Poses", "\(controller.recordedPoseCount)")
                metric("Time", String(format: "%.1fs", controller.sequencePlaybackTime))
                metric("Travel", String(format: "%.2fm", controller.sequenceTravelDistance))
            }

            Button { showExport = true } label: {
                Label("Export URScript", systemImage: "square.and.arrow.up")
                    .font(.callout.weight(.medium)).frame(maxWidth: .infinity)
            }
            .buttonStyle(.bordered)
            .disabled(controller.recordedPoseCount == 0)

            Text("Real industrial robots are optimized on exactly these numbers — cycle time and travel distance.")
                .font(.caption2).foregroundStyle(.secondary)
        }
        .padding(14).frame(maxWidth: .infinity, alignment: .leading)
        .background(.background.opacity(0.4), in: RoundedRectangle(cornerRadius: Theme.cardCorner))
    }

    private func metric(_ label: String, _ value: String) -> some View {
        VStack(spacing: 2) {
            Text(value).font(.callout.monospacedDigit().weight(.semibold))
            Text(label).font(.caption2).foregroundStyle(.secondary)
        }
        .frame(maxWidth: .infinity).padding(.vertical, 6)
        .background(.background.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }
}

// MARK: - Phase 9: challenges

private struct ChallengeSection: View {
    var challenges: ChallengeManager

    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Label("Challenges", systemImage: "checklist")
                    .font(.subheadline.weight(.semibold)).foregroundStyle(.secondary)
                Spacer()
                Button("Reset") { challenges.reset() }.font(.caption).buttonStyle(.borderless)
            }
            ForEach(challenges.challenges) { challenge in
                HStack(spacing: 8) {
                    Image(systemName: challenge.done ? "checkmark.circle.fill" : "circle")
                        .foregroundStyle(challenge.done ? Color.green : .secondary)
                    Text(challenge.title)
                        .font(.caption)
                        .strikethrough(challenge.done)
                        .foregroundStyle(challenge.done ? .secondary : .primary)
                }
                .animation(.snappy, value: challenge.done)
            }
        }
        .padding(14).frame(maxWidth: .infinity, alignment: .leading)
        .background(.background.opacity(0.4), in: RoundedRectangle(cornerRadius: Theme.cardCorner))
    }
}

// MARK: - Phase 6: URScript export sheet

private struct URScriptExportView: View {
    let script: String
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("URScript Export").font(.title2.bold())
            Text("Simplified, educational output modeled on real UR syntax. Review before running on physical hardware — direct compatibility isn't guaranteed.")
                .font(.caption).foregroundStyle(.secondary)

            ScrollView {
                Text(script)
                    .font(.system(.caption, design: .monospaced))
                    .textSelection(.enabled)
                    .frame(maxWidth: .infinity, alignment: .leading)
                    .padding(10)
            }
            .background(.background.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))

            HStack {
                Button("Copy") {
                    NSPasteboard.general.clearContents()
                    NSPasteboard.general.setString(script, forType: .string)
                }
                Button("Save to File…") { save() }
                Spacer()
                Button("Done") { dismiss() }.keyboardShortcut(.defaultAction)
            }
        }
        .padding(20)
        .frame(width: 460, height: 420)
        .fontDesign(.rounded)
    }

    private func save() {
        let panel = NSSavePanel()
        panel.nameFieldStringValue = "kinematix.script"
        panel.allowedContentTypes = [.plainText]
        if panel.runModal() == .OK, let url = panel.url {
            try? script.write(to: url, atomically: true, encoding: .utf8)
        }
    }
}

#Preview {
    ContentView()
}
