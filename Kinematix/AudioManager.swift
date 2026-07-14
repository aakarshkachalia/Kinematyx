//
//  AudioManager.swift
//  Kinematix
//
//  Subtle procedural sound design: a soft servo hum while joints move, a click
//  for gripper open/close, and a thud when an object lands. All tones are
//  synthesized at runtime with AVAudioEngine, so there are no audio asset files
//  to ship. Kept quiet on purpose — it should feel tactile, not noisy.
//

import Foundation
import AVFoundation

@MainActor
final class AudioManager {
    /// Master on/off (wired to the physics/settings panel later).
    var isEnabled = true {
        didSet { if !isEnabled { stopServo() } }
    }

    private let engine = AVAudioEngine()
    private let servoPlayer = AVAudioPlayerNode()
    private let sfxPlayer = AVAudioPlayerNode()
    private let sampleRate: Double = 44_100

    private var servoBuffer: AVAudioPCMBuffer?
    private var clickBuffer: AVAudioPCMBuffer?
    private var thudBuffer: AVAudioPCMBuffer?

    private var servoRunning = false

    init() {
        let format = AVAudioFormat(standardFormatWithSampleRate: sampleRate, channels: 1)!
        engine.attach(servoPlayer)
        engine.attach(sfxPlayer)
        engine.connect(servoPlayer, to: engine.mainMixerNode, format: format)
        engine.connect(sfxPlayer, to: engine.mainMixerNode, format: format)

        // A low, steady hum for the servo; short enveloped blips for click/thud.
        servoBuffer = makeTone(frequency: 120, duration: 0.5, amplitude: 0.05, sustained: true, format: format)
        clickBuffer = makeTone(frequency: 850, duration: 0.05, amplitude: 0.18, sustained: false, format: format)
        thudBuffer  = makeTone(frequency: 90,  duration: 0.16, amplitude: 0.30, sustained: false, format: format)

        try? engine.start()
    }

    // MARK: Public triggers

    /// Start the looping servo hum (no-op if already running).
    func startServo() {
        guard isEnabled, !servoRunning, let buffer = servoBuffer else { return }
        servoRunning = true
        servoPlayer.scheduleBuffer(buffer, at: nil, options: .loops)
        servoPlayer.play()
    }

    func stopServo() {
        guard servoRunning else { return }
        servoRunning = false
        servoPlayer.stop()
    }

    func playClick() { playOneShot(clickBuffer) }
    func playThud()  { playOneShot(thudBuffer) }

    // MARK: Synthesis

    private func playOneShot(_ buffer: AVAudioPCMBuffer?) {
        guard isEnabled, let buffer else { return }
        sfxPlayer.scheduleBuffer(buffer, at: nil)
        sfxPlayer.play()
    }

    /// Builds a single-channel PCM buffer holding a sine wave.
    /// - `sustained`: true = constant amplitude (loopable hum); false = a quick
    ///   exponential decay (a percussive click/thud).
    private func makeTone(
        frequency: Double, duration: Double, amplitude: Float,
        sustained: Bool, format: AVAudioFormat
    ) -> AVAudioPCMBuffer? {
        let frames = AVAudioFrameCount(duration * sampleRate)
        guard let buffer = AVAudioPCMBuffer(pcmFormat: format, frameCapacity: frames),
              let channel = buffer.floatChannelData?[0] else { return nil }
        buffer.frameLength = frames

        for i in 0..<Int(frames) {
            let t = Double(i) / sampleRate
            let sample = sin(2 * .pi * frequency * t)
            // Constant for a loop; fast decay for a one-shot.
            let envelope: Double = sustained ? 1.0 : exp(-Double(i) / (sampleRate * duration * 0.25))
            channel[i] = Float(sample * envelope) * amplitude
        }
        return buffer
    }
}
