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
    /// Whether a draggable object sits under the given point (decides drag vs orbit).
    var probeObject: (CGPoint, CGSize) -> Bool
    /// Start hand-dragging the object under the point.
    var beginObjectDrag: (CGPoint, CGSize) -> Void
    /// Continue the hand-drag; `vertical` is true while ⇧ is held (move in Y).
    var updateObjectDrag: (CGPoint, CGSize, Bool) -> Void
    /// Finish the hand-drag.
    var endObjectDrag: () -> Void

    func makeNSView(context: Context) -> EventCatchingView {
        let view = EventCatchingView()
        configure(view)
        return view
    }

    func updateNSView(_ nsView: EventCatchingView, context: Context) {
        configure(nsView)
    }

    private func configure(_ view: EventCatchingView) {
        view.rig = rig
        view.onClick = onClick
        view.probeObject = probeObject
        view.beginObjectDrag = beginObjectDrag
        view.updateObjectDrag = updateObjectDrag
        view.endObjectDrag = endObjectDrag
    }
}

/// An NSView that translates user input into camera moves, clicks, and object drags:
///   * left drag on empty  → orbit
///   * left drag on object → hand-move it across X/Z (hold ⇧ to move in Y)
///   * left click          → pick / place with the arm (barely-moved press)
///   * right drag          → pan
///   * scroll wheel        → zoom
///   * pinch (magnify)     → zoom
final class EventCatchingView: NSView {
    var rig: CameraRig?
    var onClick: ((CGPoint, CGSize) -> Void)?
    var probeObject: ((CGPoint, CGSize) -> Bool)?
    var beginObjectDrag: ((CGPoint, CGSize) -> Void)?
    var updateObjectDrag: ((CGPoint, CGSize, Bool) -> Void)?
    var endObjectDrag: (() -> Void)?

    // Use a top-left origin so our coordinates match SwiftUI / the camera math.
    override var isFlipped: Bool { true }
    override var acceptsFirstResponder: Bool { true }
    override func acceptsFirstMouse(for event: NSEvent?) -> Bool { true }

    // Track how far the mouse moved during a press to tell clicks from drags.
    private var pressLocation: CGPoint = .zero
    private var dragDistance: CGFloat = 0
    /// Whether the press landed on a draggable object (so drags move it, not orbit).
    private var pressedOnObject = false
    /// Whether a hand-drag of an object is actually underway.
    private var isDraggingObject = false

    // MARK: Left button: orbit, hand-drag an object, or click if it barely moved

    override func mouseDown(with event: NSEvent) {
        pressLocation = convert(event.locationInWindow, from: nil)
        dragDistance = 0
        isDraggingObject = false
        pressedOnObject = probeObject?(pressLocation, bounds.size) ?? false
    }

    override func mouseDragged(with event: NSEvent) {
        dragDistance += abs(event.deltaX) + abs(event.deltaY)
        let point = convert(event.locationInWindow, from: nil)

        if pressedOnObject {
            // Only begin the drag once past the click threshold, so a quick click
            // still falls through to the arm's pick-and-place.
            if !isDraggingObject && dragDistance >= 4 {
                isDraggingObject = true
                beginObjectDrag?(point, bounds.size)
            }
            if isDraggingObject {
                updateObjectDrag?(point, bounds.size, event.modifierFlags.contains(.shift))
            }
        } else {
            rig?.orbit(deltaX: Float(event.deltaX), deltaY: Float(event.deltaY))
        }
    }

    override func mouseUp(with event: NSEvent) {
        let point = convert(event.locationInWindow, from: nil)
        if isDraggingObject {
            endObjectDrag?()
        } else if dragDistance < 4 {
            // A near-stationary press is a click (arm pick/place), not an orbit.
            onClick?(point, bounds.size)
        }
        pressedOnObject = false
        isDraggingObject = false
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
