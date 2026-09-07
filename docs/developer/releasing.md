# Releasing

Releases are cut by pushing a `v*` tag.
`.github/workflows/release.yml` is the only thing that reacts to that tag, and it produces every published artifact.
No version number is stored anywhere in the repository; the tag is the single source of truth.

::: info
The release workflow has no trigger other than the tag push, so it cannot be dry-run.
Before tagging, exercise the build with the pre-tag check described in [Checking a release build before tagging](#checking-a-release-build-before-tagging).
:::

## Cutting a release

Tag the commit and push the tag:

```sh
git tag -a v0.4.0 -m "v0.4.0"
git push origin v0.4.0
```

The workflow creates the GitHub release immediately, then each platform job uploads its artifact as it finishes.
The release is published rather than drafted, because the macOS bottle build needs the source tarball URL to be publicly fetchable.

A tag carrying a suffix, such as `v0.4.0-rc2`, is published as a prerelease; a bare `vMAJOR.MINOR.PATCH` tag is published as a full release.
`gh release create` does not infer this from the tag, so the workflow derives it from the version string and passes `--prerelease` explicitly.
This is what keeps an rc off the repository's "Latest release" slot, which is where the README download links and anyone landing on the releases page are pointed.

## Release artifacts

A tag produces six assets:

| Artifact                                       | Platform    | Job              |
| ---------------------------------------------- | ----------- | ---------------- |
| `hawkeye-<version>.tar.gz`                     | Source      | `source-tarball` |
| `hawkeye-<version>.arm64_sonoma.bottle.tar.gz` | macOS arm64 | `bottle-arm64`   |
| `hawkeye_<version>_amd64.deb`                  | Linux amd64 | `deb-amd64`      |
| `hawkeye_<version>_arm64.deb`                  | Linux arm64 | `deb-arm64`      |
| `hawkeye-<version>-windows-x64.zip`            | Windows x64 | `windows-x64`    |
| `hawkeye-<version>-android.apk`                | Android     | `android`        |

The `.deb` files use Debian policy naming with underscores, which is why the install command in [Installation](../installation.md) globs `hawkeye_*.deb` and not `hawkeye-*.deb`.

Every platform job depends only on `source-tarball`, so one platform failing does not block the release or the other artifacts.
A failed job leaves the release published with that asset missing; re-upload it by hand with `gh release upload <tag> <file>` once the cause is fixed.

A seventh job, `update-tap`, is the exception.
It depends on `bottle-arm64` as well as `source-tarball`, and it pushes the updated formula to the [PX4/homebrew-px4](https://github.com/PX4/homebrew-px4) tap.
If the bottle build fails, the tap keeps pointing at the previous version while the GitHub release advertises the new one, so `brew install hawkeye` silently serves the old build until the job is re-run.
That is the only failure in this workflow with a user-visible consequence beyond a missing asset.

## How the version is derived

The `source-tarball` job strips the leading `v` from the tag and exports the result, and every other job reads it from there.
The two build systems receive it differently:

| Build system | How it receives the version      |
| ------------ | -------------------------------- |
| CMake        | `-DHAWKEYE_VERSION=<version>`    |
| Gradle       | `-PhawkeyeVersionName=<version>` |

`android/app/build.gradle.kts` computes the Android `versionCode` from that string as `major * 100000000 + minor * 100000 + patch * 100 + rc`, so CI passes one value and Gradle derives the other:

| Tag          | versionName | versionCode |
| ------------ | ----------- | ----------- |
| `v0.4.0-rc1` | `0.4.0-rc1` | 400001      |
| `v0.4.0`     | `0.4.0`     | 400099      |
| `v1.2.3`     | `1.2.3`     | 100200399   |
| no tag       | `0.0.0-dev` | 1           |

The rc component is what makes prerelease tags safe: a final release takes 99, an `rcN` suffix takes N (1 through 98), and the `dev` and `ci` fallbacks take 0, so every rc sorts below its final release, above the previous release, and each code can be uploaded to Google Play exactly once.
A local build with no `-PhawkeyeVersionName` falls back to `0.0.0-dev`, so debug builds need no extra flags.
The version is parsed strictly: a tag that is not `MAJOR.MINOR.PATCH` with an optional `rcN` suffix, that has a component outside `0..999`, or whose major exceeds 20 (past which the derivation overflows Google Play's version code cap) fails the Android build rather than producing a misleading version code.

## The Android APK

The `android` job builds a single universal APK containing `arm64-v8a` and `x86_64`.
There is no `armeabi-v7a` build, so 32-bit ARM devices are not supported.
The minimum supported platform is Android 10 (API 29).

The APK is signed with the project's upload key, which the job decodes from the `ANDROID_UPLOAD_KEYSTORE_BASE64`, `ANDROID_UPLOAD_KEYSTORE_PASSWORD`, and `ANDROID_UPLOAD_KEY_ALIAS` repository secrets, and the asset installs as downloaded.
A fork without those secrets falls back to the pre-signing behavior: the artifact is built unsigned, named `hawkeye-<version>-android-unsigned.apk` to make that obvious, and has to be signed before it will install; see [Signing an APK yourself](https://github.com/PX4/Hawkeye/blob/main/android/README.md#signing-an-apk-yourself) in the Android README.

Because the APK cannot be launched on a CI runner without an emulator, `android/scripts/verify-release-apk.sh` asserts on its contents instead:

- Both `lib/arm64-v8a/libhawkeye.so` and `lib/x86_64/libhawkeye.so` are present.
- All four asset trees are packaged: `assets/models/`, `assets/shaders/`, `assets/fonts/`, and `assets/themes/`, along with `assets/NOTICE.md`, which the in-app About screen renders.
  These are symlinks into the repository root, so the check catches a runner that failed to materialize them.
- The `versionName` AGP recorded matches the tag, and the `versionCode` is one Android will accept.
- With `--signed`, which the job passes whenever the keystore secrets are present, `apksigner verify` confirms the signature.

That script takes the APK output directory and the expected version, so you can run the same check locally against your own build.

## Google Play internal testing

The same `android` job also runs `bundleRelease`, checks the resulting AAB with `android/scripts/verify-release-bundle.sh`, and uploads it to the Google Play internal test track under the Dronecode Foundation account.
The upload step runs only when the `PLAY_SERVICE_ACCOUNT_JSON` secret is present; it authenticates as a Google Cloud service account granted release-to-testing permission on the app in the Play Console.
Prerelease tags upload like any other tag; rc builds are what the internal track is for.
Promotion beyond internal testing is manual in the Play Console.

The AAB is not attached to the GitHub release.
Google Play is its only destination, and Play App Signing re-signs it with the app signing key Google holds, so a Play install and a sideloaded APK carry different signatures and cannot upgrade over each other.

Anyone who sideloaded a self-signed APK from a release cut before signing landed (v0.3.0 and earlier) has to uninstall it once before an official signed build will install; release notes should carry that reminder until it stops being relevant.

### Store listing wording

No Play listing text lives in this repository; the workflow uploads the AAB and nothing else, so the listing is maintained by hand in the Play Console.
Because the project is open source and anyone may publish a fork, the listing has to make it obvious which app this is.
Whoever edits it should keep three things in the description:

- That this is the official Hawkeye app, published by the Dronecode Foundation.
- A link to <https://github.com/PX4/Hawkeye>.
- That Hawkeye is a visualization and analysis tool rather than a flight-safety device.

Forks are asked to change their app name, icon, and `applicationId` before publishing, and to carry a non-affiliation notice; the policy they are pointed at is [FORKS.md](https://github.com/PX4/Hawkeye/blob/main/FORKS.md) in the repository root.
If a listing turns up that misrepresents itself as this app, report it through Google Play's [impersonation report](https://support.google.com/googleplay/android-developer/answer/16341334).

### Privacy policy and Data safety

Google Play requires both a privacy policy URL and a completed Data safety form for every app, including apps that collect nothing, and both must be in place before the app is promoted beyond the internal test track.
Neither can be scripted; they are filled in by hand in the Play Console.

- The privacy policy URL is <https://px4.github.io/Hawkeye/privacy>, published from [`docs/privacy.md`](../privacy.md).
- The Data safety form declares three data types, all *collected*, none *shared*, and all marked **optional** because crash reporting can be turned off in Settings:
  - **Crash logs** and **Diagnostics**, under *App info and performance*.
  - **Device or other IDs**, because Crashlytics sends a Crashlytics installation UUID and a Firebase installation ID with each report. This one is easy to miss: it comes from the SDK rather than from anything Hawkeye asks for, and Firebase's own [Play data disclosure](https://firebase.google.com/docs/android/play-data-disclosure) is the authority on what has to be declared.
  - Every type is answered *not processed ephemerally* (Google retains reports for 90 days) with purpose **Analytics**, whose Play definition covers diagnosing and fixing crashes. Security practices declare data encrypted in transit; data deletion is declared as not offered, since reports key to an anonymous install ID that no user can identify as theirs.

That declaration holds because of what the app actually does, and it is worth knowing why rather than taking it on faith.
Google defines collection as transmitting data off the device, and explicitly exempts data that is only processed locally.
Hawkeye's flight logs, log library, and settings never leave the device: there is no Hawkeye server and no analytics, and Android backup is disabled so the platform does not copy them either.
Live telemetry travels only between the app and the vehicle or simulator the user connected to.
The single outbound path is Firebase Crashlytics, which sends a crash trace, device state, and an installation identifier when the app fails. It carries nothing from a flight log, which is why the location categories stay unchecked.

The form and the policy have to agree, since a mismatch is a common cause of listing rejection.
Anything that changes the answer, in particular adding an analytics or advertising dependency, widening what Crashlytics reports, or re-enabling `android:allowBackup`, means updating `docs/privacy.md` and the Data safety form together.

Crashlytics also needs two repository secrets, and a release built without them is not broken, just unreported: `FIREBASE_GOOGLE_SERVICES_JSON` (base64 of `google-services.json`) and `FIREBASE_SERVICE_ACCOUNT_JSON` (a service account key for the Firebase project, holding `roles/firebasecrashlytics.admin`; the Gradle upload task reads it through `GOOGLE_APPLICATION_CREDENTIALS`).
The two are gated independently. Without `FIREBASE_GOOGLE_SERVICES_JSON` the build ships with crash reporting dormant, exactly as a fork's CI does. Without `FIREBASE_SERVICE_ACCOUNT_JSON` the build still reports crashes, but the symbol upload is skipped and native stack traces arrive as raw addresses.
The symbol upload runs before the APK is published, so a failure there stops the release rather than shipping a build whose native crashes cannot be read.

## Checking a release build before tagging

`.github/workflows/android.yml` has a `release-build` job that runs the same `assembleRelease` and `bundleRelease` tasks and the same verification scripts the release uses, including the signed path when the keystore secrets are present.
It runs on pushes to `main` and on manual dispatch, and is skipped on pull requests to keep review turnaround fast.

Trigger it from a branch before tagging:

```sh
gh workflow run android.yml --ref my-branch
```

The push-to-`main` run of that job also warms the native build cache the release job reads, because a tag run restores caches from the default branch.
A `--ref my-branch` dispatch does not warm it, since caches written on a branch are not visible to a later tag run.

The workflow's path filter covers the repository root `src/`, `lib/`, `fonts/`, `models/`, `shaders/`, and `themes/` directories in addition to `android/`.
The Android native library compiles source files out of the root `src/` tree and its assets are symlinks to the root asset directories, so a desktop-side change can break the APK and has to trigger Android CI.
