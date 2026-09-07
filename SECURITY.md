# Security policy

## Reporting a vulnerability

**Please do not report security vulnerabilities through public GitHub issues.**

Hawkeye follows the PX4 project-wide process. Security reports for any PX4 repository, including this one, go through GitHub Security Advisories on [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot), so the project has a single private disclosure channel.

If AI assistance contributed to your finding, do not file a private advisory. Read [If you used AI assistance to find the issue](#if-you-used-ai-assistance-to-find-the-issue) below instead.

To report a vulnerability:

1. Go to [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot).
2. Click the **Security** tab (or **...** then **Security** on mobile).
3. Click **Report a Vulnerability** and fill in the advisory form.

Mention that the report concerns Hawkeye so it reaches the right maintainers. Please include enough detail to reproduce and verify the issue.

Expect acknowledgment within 7 days. If you do not hear back in that time, follow up with the PX4 [release managers](https://github.com/PX4/PX4-Autopilot/blob/main/MAINTAINERS.md). Disclosure is coordinated with you, and you will be credited in the advisory unless you ask to stay anonymous.

The [PX4 security policy](https://github.com/PX4/.github/blob/main/SECURITY.md) and the [PX4-Autopilot security policy](https://github.com/PX4/PX4-Autopilot/blob/main/SECURITY.md) describe the response process and the project's secure development practices in full. Where they and this page differ, they are authoritative.

### If you used AI assistance to find the issue

PX4 asks that AI-assisted findings be treated as public rather than filed as a private advisory, because the same bug tends to surface across multiple researchers at once. Open a pull request with a fix against [PX4/Hawkeye](https://github.com/PX4/Hawkeye) instead, or an issue if you cannot write one; check the [open pull requests](https://github.com/PX4/Hawkeye/pulls) first in case a fix already exists. Do not post the reproducer publicly. Say that it exists and share it privately on request. Read the [full guidance](https://github.com/PX4/PX4-Autopilot/blob/main/SECURITY.md#ai-assisted-discovery) before you report.

## Supported builds

Only official builds are supported:

- **GitHub Releases** on [PX4/Hawkeye](https://github.com/PX4/Hawkeye/releases)
- **Google Play**, published under the Dronecode Foundation account, currently the internal test track only
- **Homebrew**, via the [PX4/homebrew-px4](https://github.com/PX4/homebrew-px4) tap

Hawkeye is pre-1.0. Fixes ship in the next release and are not back-ported to earlier ones.

Anything obtained elsewhere is an unofficial build. This project did not produce it and cannot vouch for it, so please report problems with such a build to whoever published it. See [FORKS.md](FORKS.md) for what makes a build official and what a fork is asked to rename.

## Scope

Hawkeye visualizes and analyzes flight data. It sends no flight or arming commands and is not a flight-safety device, so its attack surface is narrow. The main places it handles untrusted input are where a security report is most likely to be actionable:

- **ULog parsing.** Entry point `src/ulog_parser.c`, with replay handling in `src/ulog_replay.c` and `src/ulog_replay_apply.c`. A `.ulg` file is untrusted input from an arbitrary source, and on Android it arrives through the system file picker. Memory-safety bugs reachable from a malformed or hostile log are in scope.
- **The MAVLink listener.** `src/mavlink_receiver.c` binds a UDP port on all interfaces and reads unauthenticated datagrams, so crashes or memory-safety bugs triggered by malformed MAVLink input are in scope. Hawkeye also learns its peer from the first datagram it receives, does not re-authenticate it, and sends telemetry-request commands (`MAV_CMD_REQUEST_MESSAGE`, `MAV_CMD_SET_MESSAGE_INTERVAL`) back to that peer.
- **Theme files.** `src/theme.c` parses `.mvt` theme files from a caller-supplied path, including files dropped onto the desktop window.
- **The browser build.** `wasm/` uses its own ULog implementation in `src/wasm/ulog_extractor.c` rather than the native parser, so a finding there should cite that file specifically.

Generally out of scope: issues that require an attacker to already control the device, and the absence of authentication in the MAVLink protocol itself, which is a property of the protocol rather than a defect in Hawkeye.

Hawkeye bundles no analytics. The Android app reports crashes to Firebase Crashlytics, in a Firebase project held by the Dronecode Foundation, and a report carries the failure, device state, and an installation identifier but nothing from a flight log; it can be turned off in Settings, and the desktop and browser builds have no crash reporting at all. See the [privacy policy](https://px4.github.io/Hawkeye/privacy) for what a report contains. Apart from that, Hawkeye's network traffic goes only to the vehicle or simulator you connect it to.

For bugs that are not security issues, open an issue on [PX4/Hawkeye](https://github.com/PX4/Hawkeye/issues).
