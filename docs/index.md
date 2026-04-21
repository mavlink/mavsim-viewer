---
layout: home

hero:
  name: Hawkeye
  text: Real-time 3D flight visualizer for PX4
  tagline: Watch live SITL simulations, replay ULog flights, and analyze multi-drone swarms — up to 16 vehicles with ghost overlays, correlation tracking, and takeoff alignment.
  image:
    src: /screenshot.png
    alt: Hawkeye screenshot
  actions:
    - theme: brand
      text: Get Started
      link: /first-sitl
    - theme: alt
      text: Install
      link: /installation
    - theme: alt
      text: View on GitHub
      link: https://github.com/PX4/Hawkeye

features:
  - title: Live SITL
    details: Connect directly to PX4 SIH and visualize vehicle state from MAVLink HIL_STATE_QUATERNION. Single vehicle or full swarms on sequential UDP ports.
    link: /sitl
    linkText: SITL guide
  - title: ULog Replay
    details: Load .ulg flight logs for interactive 3D playback with transport controls, markers, and dead-reckoning interpolation between samples.
    link: /replay
    linkText: Replay guide
  - title: Multi-Drone Swarms
    details: Up to 16 vehicles with ghost mode overlays, automatic deconfliction, CUSUM takeoff alignment, and real-time Pearson correlation between pinned drones.
    link: /multi_drone
    linkText: Multi-drone guide
  - title: Two HUD Modes
    details: Console HUD for analysis with full telemetry and annunciators, or Tactical HUD with radar panel and gimbal rings for recording demos.
    link: /hud
    linkText: HUD guide
  - title: Cameras, Views & Themes
    details: Chase, FPV, and free-fly cameras. Orthographic side panel and fullscreen ortho views. Grid, Rez, and Snow themes for any lighting condition.
    link: /views
    linkText: Views guide
  - title: Zero Dependencies
    details: Built on Raylib and MAVLink. Ships as a single binary. macOS, Linux, and Windows supported out of the box.
    link: /installation
    linkText: Install
---
