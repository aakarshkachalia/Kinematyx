# Kinematyx

A macOS app that simulates a real 6-axis industrial robotic arm — the kind that costs upwards of $10,000 — with physically accurate kinematics, so anyone can experiment with one for free.

## Why I built this

I've spent seven years on FTC robotics, and every season comes back to the same limitation: the arms worth learning on are the ones none of us can afford. A real UR5 costs more than most families are going to spend on a robotics hobby, and the handful of simulators that do exist are built for engineers, not for someone trying to build intuition for the first time.

I wanted to close that gap. Not a toy that gestures at what a robotic arm does, but something that gets the actual math right — real DH parameters pulled from a real UR5, real inverse kinematics, real joint limits and torque estimation — wrapped in something approachable enough that a kid could open it and just start dragging the arm around.

## What it is

Kinematyx is a macOS app built in Swift and RealityKit, with the kinematics engine (`RobotArmKit`) written as a standalone, UI-agnostic Swift package — every bit of forward kinematics, inverse kinematics, joint limits, and collision math is unit tested independently of the 3D rendering.

Core features:

- **A real UR5 model**, kinematically accurate to the published spec, not an invented approximation
- **Full 6DOF inverse kinematics** with a damped least-squares solver, so the arm can be driven to a target position *and* orientation, not just a point in space
- **A physics-correct sandbox** — drop objects, stack them, place obstacles, and watch the arm interact with them under real gravity and collision, not scripted animation
- **A working two-finger gripper** with real contact-based grasping, not proximity-based auto-pickup
- **Multi-part assembly** — the arm can build a model car from separate parts (chassis, wheels, body), using snap-fit mating tolerances and a proper motion sequence: approach, grasp, lift, pre-insert, linear insertion, mate
- **Singularity detection and torque estimation**, so the app teaches the *reasons* real arms behave the way they do, not just the motion
- **Record and replay**, with cycle-time and distance metrics, mirroring how real industrial arms get optimized on a production line

## Who it's for

Students learning robotics for the first time, FTC/FRC teams who want to prototype motion before touching hardware, and anyone curious what a $10,000 machine actually does — without needing to own one.

---

*Built by Aakarsh Kachalia.*
