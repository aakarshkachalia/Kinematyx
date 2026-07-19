//
//  KinematyxApp.swift
//  Kinematyx
//
//  Created by Aakarsh Kachalia on 7/18/26.
//

import SwiftUI

@main
struct KinematyxApp: App {
    init() {
        // TEMP DIAGNOSTIC: unbuffered stdout so headless auto-test logs flush live.
        if ProcessInfo.processInfo.environment["KINEMATYX_AUTOTEST"] != nil {
            setvbuf(stdout, nil, _IONBF, 0)
        }
    }

    var body: some Scene {
        WindowGroup {
            ContentView()
        }
    }
}
