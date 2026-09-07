# Privacy Policy

Dronecode Project, Inc. publishes Hawkeye (`com.px4.hawkeye.android`) as an open source project. It is provided at no cost and is intended for use as is.

This policy covers the Hawkeye Android app, the desktop builds for macOS, Linux, and Windows, and the browser replay.

**Effective date: 2026-09-06**

## The short version

Hawkeye collects no personal data. It has no user accounts, no analytics, no advertising, and no tracking of any kind. Nothing you open in Hawkeye is sent to Dronecode, to PX4, or to any third party, because there is no Hawkeye server to send it to.

The one exception is crash reporting, and it applies to the Android app only. When the app crashes or hits an unexpected error, it sends a diagnostic report so the fault can be fixed. **Your flight logs and their location data are never part of a report**, and you can turn crash reporting off. The next section says exactly what a report contains.

## Crash reporting

The Android app uses [Firebase Crashlytics](https://firebase.google.com/docs/crashlytics), a Google service, to report crashes and unexpected errors. This is the only feature that sends anything off your device, and it exists so that faults which would otherwise be invisible get fixed.

**What a report contains:**

- The failure itself: the exception or fatal signal, and the sequence of function calls that led to it. For a crash in the 3D renderer this includes a memory dump of the crashed threads.
- The state of the device and the app: your device model, CPU architecture, and Android version; how much memory and storage were free; whether the device is rooted; whether the app was in the foreground; and which version of Hawkeye you are running.
- Two randomly generated installation identifiers, which let separate crashes from the same install be recognized as related. They are not tied to you, your Google account, or your device's advertising ID, and they are reset if you reinstall the app.

Google publishes the exhaustive list of what Crashlytics records on its [Firebase privacy page](https://firebase.google.com/support/privacy).

**What a report never contains:**

- Your flight logs, or any part of them. Nothing from a `.ulg` file is read into a report.
- Location data of any kind, whether the vehicle's or the device's.
- The addresses of vehicles or simulators you connect to.
- Your name, email address, or any account identifier. Hawkeye has no accounts.

Reports go to a Firebase project held by the Dronecode Foundation. Google processes them on Dronecode's behalf, under [Google's privacy policy](https://policies.google.com/privacy). Google states that it keeps crash traces, dump data, and the associated identifiers for 90 days before beginning to remove them from live and backup systems.

**Turning it off.** Crash reporting is on when you first install the app. To turn it off, open **Settings** and switch off **Send crash reports**. The choice is remembered, applies to the whole app including the 3D renderer, and survives updates. With it off, nothing is sent.

Crash reporting exists only in the Android app. The desktop builds and the browser replay have none.

## What stays on your device

Everything Hawkeye works with stays on the device you run it on:

- **Flight logs.** ULog files you import are copied into the app's private storage so they can be replayed. **Flight logs contain location data**, including the vehicle's GPS coordinates and its home position, along with the times the flight took place.
- **Your log library.** A local database records each imported log's original file name, its size, and when you imported it.
- **Settings.** Your theme, unit preference, and the network port Hawkeye listens on.

This data is read and rendered on the device and is never uploaded. Hawkeye has no share, upload, or export feature.

Android's automatic backup is **disabled** for Hawkeye, so this data is not copied to Google Drive or transferred to a new device. That is a deliberate choice: flight logs contain location traces, and they should not leave your device without you moving them yourself.

You can delete any imported log from within the app, and uninstalling Hawkeye removes all of it.

## Network activity

Hawkeye connects to a vehicle or simulator **that you choose**, over MAVLink on your local network. Two things are worth being precise about:

- Traffic flows in both directions. Hawkeye listens for telemetry, and when a vehicle first responds it sends a small number of telemetry-request commands back to it, asking for the position and attitude messages it needs to draw the scene. It sends no flight or arming commands.
- That traffic goes only to the address the telemetry came from, meaning your own vehicle or simulator. It never reaches a Dronecode or PX4 server.

While a live session is running, Hawkeye accepts telemetry on all of the device's network interfaces, not only the loopback interface, so that a vehicle elsewhere on your network can reach it. It listens only while you have started a live session.

Apart from crash reports, Hawkeye does not check for updates, download maps, or contact any remote service.

## Android permissions

The Android app declares these permissions:

| Permission | Why it is there |
| --- | --- |
| `INTERNET` | Sending and receiving MAVLink telemetry on your local network. |
| `ACCESS_NETWORK_STATE` | Letting crash reporting wait for a working connection before sending a report. Also required by the AndroidX media library that plays the background video on the home screen. |
| `WAKE_LOCK` | Not used by Hawkeye. Added by the media library above. |
| `DYNAMIC_RECEIVER_NOT_EXPORTED_PERMISSION` | Not used by Hawkeye. Added by the AndroidX core library for its own internal messaging. It is signature level, so no other app can hold it. |

Hawkeye requests no permission that Android classes as dangerous. It does not ask for your location, your files, your contacts, your camera, or your microphone. The location data it displays comes from the flight logs you open and from the vehicle you connect to, not from the device's location services.

## Desktop and browser builds

The desktop builds behave the same way: logs are read from the disk you point them at, telemetry stays on your local network, and nothing is uploaded. They have no crash reporting.

The browser replay processes the file you select inside the page itself. The file is not uploaded to a server.

## This website

These docs are published with GitHub Pages, and some pages load images and video from `artifacts.px4.io`. As with any website, those hosts receive standard request information such as your IP address and browser user agent in their server logs. The site sets no analytics or advertising cookies, and its search runs entirely in your browser.

## Children's privacy

Hawkeye is not directed at children under 13, and it collects no personal information from anyone, including children.

## Changes to this policy

If Hawkeye's behavior changes in a way that affects this policy, the policy will be updated here and the effective date above will change.

## Contact

Questions about this policy can go to [info@dronecode.org](mailto:info@dronecode.org), or to an issue on [GitHub](https://github.com/PX4/Hawkeye/issues).
