//
//  ViewportInput.swift
//  Kinematix
//
//  A transparent AppKit view laid over the 3D viewport that captures raw mouse
//  and trackpad events and forwards them to the CameraRig (orbit/pan/zoom) and,
//  for simple clicks, to a pick-and-place handler. SwiftUI has no direct gesture
//  for the scroll wheel, so we drop down to NSView to handle it properly.
//

import SwiftUI
import AppKit

/// Bridges an AppKit event-catching view into SwiftUI.
struct ViewportInput: NSViewRepresentable {
    var rig: CameraRig
    /// Called for a click (a mouse press with negligible drag). Gives the point
    /// in the view's coordinate space (top-left origin) and the view size.
    var onClick: (CGPoint, CGSize) -> Void

    func makeNSView(context: Context) -> EventCatchingView {
        let view = EventCatchingView()
        view.rig = rig
        view.onClick = onClick
        return view
    }

    func updateNSView(_ nsView: EventCatchingView, context: Context) {
        nsView.rig = rig
        nsView.onClick = onClick
    }
}

/// An NSView that translates user input into camera moves and clicks:
///   * left drag        → orbit
///   * left click       → pick / place (drag under a few px counts as a click)
///   * right drag       → pan
///   * scroll wheel     → zoom
///   * pinch (magnify)  → zoom
final class EventCatchingView: NSView {
    var rig: CameraRig?
    var onClick: ((CGPoint, CGSize) -> Void)?

    // Use a top-left origin so our coordinates match SwiftUI / the camera math.
    override var isFlipped: Bool { true }
    override var acceptsFirstResponder: Bool { true }
    override func acceptsFirstMouse(for event: NSEvent?) -> Bool { true }

    // Track how far the mouse moved during a press to tell clicks from drags.
    private var pressLocation: CGPoint = .zero
    private var dragDistance: CGFloat = 0

    // MARK: Left button: orbit, or click if it barely moved

    override func mouseDown(with event: NSEvent) {
        pressLocation = convert(event.locationInWindow, from: nil)
        dragDistance = 0
    }

    override func mouseDragged(with event: NSEvent) {
        dragDistance += abs(event.deltaX) + abs(event.deltaY)
        rig?.orbit(deltaX: Float(event.deltaX), deltaY: Float(event.deltaY))
    }

    override func mouseUp(with event: NSEvent) {
        // A near-stationary press is a click, not an orbit.
        if dragDistance < 4 {
            let point = convert(event.locationInWindow, from: nil)
            onClick?(point, bounds.size)
        }
    }

    // MARK: Pan (right button drag)

    override func rightMouseDragged(with event: NSEvent) {
        rig?.pan(deltaX: Float(event.deltaX), deltaY: Float(event.deltaY))
    }

    // MARK: Zoom (scroll wheel / two-finger scroll)

    override func scrollWheel(with event: NSEvent) {
        let raw = Float(event.scrollingDeltaY)
        let factor: Float = event.hasPreciseScrollingDeltas ? 0.004 : 0.02
        rig?.zoom(by: raw * factor)
    }

    // MARK: Zoom (trackpad pinch)

    override func magnify(with event: NSEvent) {
        rig?.zoom(by: Float(event.magnification))
    }
}
