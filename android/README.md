# Hawkeye Android

Android port of the Hawkeye 3D renderer, running the desktop C/Raylib scene natively on Android via NativeActivity and OpenGL ES 3.0.

## Architecture

The app is a `NativeActivity` subclass (`HawkeyeActivity`) that loads `libhawkeye.so` and runs a standard C `main()` via Raylib's Android platform backend. All 3D rendering and replay logic is C — the Kotlin layer only handles platform-side concerns: `VIEW`/`SEND` intents, file ingestion into the inbox, and Android-SDK chrome (dialogs, toasts). The Kotlin side follows MVI; see [Kotlin Architecture (MVI)](#kotlin-architecture-mvi) below.

```
android/
├── build-logic/                  — Gradle convention plugins (hawkeye.android.*, hawkeye.jvm.library)
├── core/
│   ├── domain/                   — pure Kotlin: Result, Error, DataError
│   ├── presentation/             — Compose helpers: UiText, ObserveAsEvents, app-provided seams
│   ├── design-system/            — HawkeyeTheme (Compose Material 3)
│   └── navigation/               — NavKey destinations, TopLevelDestination, NavBackStacks
├── feature/home/                 — hero screen: mode cards + recent flights
├── feature/replay/               — .ulg ingestion + confirm flow
│   ├── domain/                   — UlogInboxDataSource interface, UlogFile, ReplayError
│   ├── data/                     — AndroidUlogInboxDataSource (does the file copy + sentinel write)
│   └── presentation/             — ReplayViewModel, State/Action/Event, ReplayRoot + dialogs
├── feature/live/                 — live MAVLink session setup
├── feature/settings/             — theme, units, listen port
├── feature/about/                — version, official-build info, bundled license notices
└── app/src/main/
    ├── AndroidManifest.xml       — declares HawkeyeActivity, requires OpenGL ES 3.0
    ├── assets/                   — all symlinks into the parent Hawkeye project
    │   ├── fonts  -> ../../../../../fonts
    │   ├── models -> ../../../../../models
    │   ├── shaders -> ../../../../../shaders
    │   ├── themes -> ../../../../../themes
    │   └── NOTICE.md -> ../../../../../NOTICE.md
    ├── java/com/px4/hawkeye/android/
    │   ├── HawkeyeActivity.kt    — NativeActivity host; owner plumbing + Compose overlay
    │   └── HawkeyeApp.kt         — Application class, startKoin
    └── cpp/
        ├── CMakeLists.txt        — fetches Raylib 5.5, compiles rendering subset
        └── android_main.c        — entry point: extracts assets, polls inbox sentinel, render loop
```

The Hawkeye source files compiled in are: `scene.c`, `vehicle.c`, `theme.c`, `ortho_panel.c`, plus the ULog replay subset (`data_source_ulog.c`, `ulog_parser.c`, `ulog_replay.c`). MAVLink, HUD, and the desktop multi-drone replay machinery are excluded.

## Kotlin Architecture (MVI)

The Android-SDK-using code follows MVI (Model–View–Intent). State, actions, and one-time side effects are owned by a `ViewModel` and rendered by Compose — not scattered across the Activity. If you're only touching the C/Raylib rendering engine, you can ignore this section: that code lives under `app/src/main/cpp/` and the Kotlin layer doesn't touch it.

### Module layers

Clean Architecture, split feature-first:

- **`domain`** — pure Kotlin. Domain models, data-source/repository **interfaces**, feature-specific error types. Never imports Android.
- **`data`** — Android library. Concrete implementations of the domain interfaces (file I/O, ContentResolver, networking). DTOs and mappers stay here.
- **`presentation`** — Android library + Compose. `ViewModel`, `State`/`Action`/`Event`, `Root` + `Screen` composables.

Dependency direction: `presentation → domain ← data`. Domain depends on nothing else (other than `core:domain`). Features never depend on each other — shared code moves into `core/`. `:app` wires everything via Koin.

### The four MVI pieces

Every screen is built from:

1. **`<Screen>State`** — a single data class holding all UI state. Update with `_state.update { it.copy(...) }`, never replace the flow.
2. **`<Screen>Action`** — a sealed interface of every user-triggered intent (clicks, text input, dialog dismissal).
3. **`<Screen>Event`** — a sealed interface of one-time side effects (toasts, navigation, snackbars). Delivered through a `Channel`, not state.
4. **`<Screen>ViewModel`** — exposes `state: StateFlow<State>`, `events: Flow<Event>`, and `fun onAction(action: Action)`.

```kotlin
data class ReplayState(val dialog: ReplayDialog? = null, val isIngesting: Boolean = false)

sealed interface ReplayAction {
    data class OnIntentReceived(val uri: String?, val isColdLaunch: Boolean) : ReplayAction
    data object OnConfirmOpen : ReplayAction
    data object OnDismissDialog : ReplayAction
}

sealed interface ReplayEvent {
    data class ShowToast(val text: UiText) : ReplayEvent
}

class ReplayViewModel(private val ulogInbox: UlogInboxDataSource) : ViewModel() {
    private val _state = MutableStateFlow(ReplayState())
    val state = _state.asStateFlow()

    private val _events = Channel<ReplayEvent>()
    val events = _events.receiveAsFlow()

    fun onAction(action: ReplayAction) { /* update state, emit events */ }
}
```

### Composables: Root vs Screen

Two composables per screen, in the same file:

- **`<Screen>Root`** — gets the ViewModel via `koinViewModel()`, observes state and events, passes them down. The only composable that touches the VM.
- **`<Screen>Screen`** — receives only `state` + `onAction`. Pure, previewable, no ViewModel reference.

```kotlin
@Composable
fun ReplayRoot(viewModel: ReplayViewModel = koinViewModel()) {
    val state by viewModel.state.collectAsStateWithLifecycle()
    val context = LocalContext.current
    ObserveAsEvents(viewModel.events) { event ->
        when (event) {
            is ReplayEvent.ShowToast ->
                Toast.makeText(context, event.text.asString(context), Toast.LENGTH_SHORT).show()
        }
    }
    ReplayScreen(state = state, onAction = viewModel::onAction)
}

@Composable
fun ReplayScreen(state: ReplayState, onAction: (ReplayAction) -> Unit) { /* render */ }
```

### Errors and `UiText`

Every data source / repository returns `Result<T, E>` where `E` is a typed error (`ReplayError`, `DataError.Local`, etc.). Never throw exceptions for expected failures — catch them at the layer that owns the cause and return `Result.Error(...)`.

Failures that reach the `UNKNOWN` branch of an error mapper — and only those — are reported through `CrashReporter` (`core:domain`), injected into the data class that owns the mapping. Expected outcomes the user already sees, such as `NOT_FOUND` or `DISK_FULL`, are never reported; a dashboard full of full disks hides the defects worth fixing. `CrashReporter.None` is the dormant implementation used by forks and tests, and it is deliberately not a constructor default, so a dropped injection fails to compile instead of silently going quiet.

User-facing strings that come from resources are wrapped in `UiText` so the ViewModel can carry them without holding a `Context`. Each feature defines a `<FeatureError>.toUiText()` extension in its presentation module — `ReplayError.toUiText()` lives next to `ReplayViewModel`.

### Dependency injection (Koin)

Each feature has two Koin modules — one in `data`, one in `presentation`:

```kotlin
// feature:replay:data
val replayDataModule = module {
    single<UlogInboxDataSource> {
        AndroidUlogInboxDataSource(androidContext().contentResolver, androidContext().filesDir)
    }
}

// feature:replay:presentation
val replayPresentationModule = module {
    viewModelOf(::ReplayViewModel)
}
```

Both are registered in `HawkeyeApp.onCreate { startKoin { modules(...) } }`. Root composables get their VM via `koinViewModel()`. **Never** instantiate a ViewModel directly outside Koin/tests.

### Testing

Unit-test every ViewModel using JUnit5 + Turbine + AssertK + a `Fake<Whatever>` data source:

```kotlin
@OptIn(ExperimentalCoroutinesApi::class)
class ReplayViewModelTest {
    private val dispatcher = UnconfinedTestDispatcher()
    @BeforeEach fun setUp() { Dispatchers.setMain(dispatcher) }
    @AfterEach fun tearDown() { Dispatchers.resetMain() }

    @Test fun `cold launch with null uri shows NoFileLoaded`() = runTest { /* ... */ }
}
```

Fakes (in-memory implementations of the domain interface) catch more real bugs than mocks. See `feature/replay/presentation/src/test/.../FakeUlogInboxDataSource.kt` for the pattern.

### Adding a new feature

1. Create three modules under `feature/<name>/`: `domain` (`hawkeye.jvm.library`), `data` (`hawkeye.android.library`), `presentation` (`hawkeye.android.feature`).
2. Add each to `settings.gradle.kts`.
3. In **domain**: define the data-source/repository interface, models, and a `<Feature>Error : Error` enum. Return `Result<T, <Feature>Error>` everywhere a call can fail.
4. In **data**: implement the interface. Name the class for what makes it unique (`AndroidFooDataSource`, `KtorBarDataSource`) — never suffix with `Impl`. Add `<feature>DataModule`.
5. In **presentation**: write `<Feature>State`, `<Feature>Action`, `<Feature>Event`, `<Feature>ViewModel`, plus `<Feature>Root` + `<Feature>Screen` composables. Map errors to `UiText` via `<FeatureError>.toUiText()`. Add `<feature>PresentationModule` with `viewModelOf(::<Feature>ViewModel)`.
6. Write VM unit tests against a `Fake<DataSource>`. Cover every Action and both success/failure branches.
7. Register both Koin modules in `HawkeyeApp.startKoin { modules(...) }`.
8. Mount the new `<Feature>Root` composable somewhere — typically inside `HawkeyeActivity`'s Compose overlay next to `ReplayRoot`.

### Why HawkeyeActivity has owner boilerplate

`NativeActivity` extends `android.app.Activity` — not `ComponentActivity` — so the AndroidX lifecycle/owner machinery isn't there for free. `HawkeyeActivity` implements `LifecycleOwner`, `ViewModelStoreOwner`, and `SavedStateRegistryOwner` by hand. The dispatch ordering matches what `ComponentActivity` does internally so Compose, ViewModels, and Koin all see a well-formed owner tree. Compose chrome renders into a transparent `ComposeView` overlaid via `window.addContentView`; the native engine keeps its own SurfaceView underneath. **Do not** swap `NativeActivity` for another base class — `android.app.lib_name=hawkeye` in the manifest is what triggers `ANativeActivity_onCreate` in the engine.

## Loading flight logs

Logs are imported through the replay library screen, which opens the system document picker via `ActivityResultContracts.OpenDocument()`. ULog has no registered MIME type, so the picker accepts `*/*`. Import is a raw byte copy with no format check, so a file that isn't a valid ULog is rejected when playback starts, not when it is picked. Imported payloads live in the app's library directory with their metadata in Room (`feature/replay/data/`), so a log only has to be picked once.

Mechanism: starting playback stages the selected library payloads into `filesDir/inbox/`. The first drone keeps the name `current.ulg` and the rest become `swarm_<i>.ulg`, then a `<millis>` token (or `<millis> <count>` for a swarm) is written to `filesDir/inbox/.ready`. The sentinel is bumped only after the whole batch has copied, so any I/O failure leaves the inbox exactly as the still-unchanged token describes it. The native render loop polls that sentinel at roughly 1 Hz and reloads the replay when the token changes.

Live mode uses the same directory: `RendererLauncher` writes an `inbox/.live` marker containing `<millis> <port>`, and native startup compares it against the replay `.ready` token, newest wins.

The manifest declares no `VIEW` or `SEND` intent filters, so logs cannot currently be opened by sharing them into Hawkeye from Drive, Files, or Gmail.

## Shader Compatibility

The original shaders use `#version 330` (desktop OpenGL). On Android they are patched at load time via Raylib's `SetLoadFileTextCallback`:

- `#version 330` → `#version 300 es`
- `precision mediump float;` is injected into fragment shaders

No shader copies are kept in the Android project — the `shaders/` asset dir is a symlink to the originals.

## Environment Setup

If you haven't done Android development before, here's what you need:

### 1. Install Android Studio

Download and install [Android Studio](https://developer.android.com/studio) for your platform (macOS, Linux, or Windows). The installer includes the Android SDK and the SDK Manager.

### 2. Install the NDK and CMake

Open Android Studio, go to **Settings → Languages & Frameworks → Android SDK → SDK Tools**, check:

- **NDK (Side by side)** — install version `30.0.14904198`
- **CMake** — install version `3.22.1`

Click **Apply**.

Alternatively, from the command line (replace `$ANDROID_SDK_ROOT` with your SDK path — typically `~/Library/Android/sdk` on macOS or `~/Android/Sdk` on Linux):

```bash
$ANDROID_SDK_ROOT/cmdline-tools/latest/bin/sdkmanager "ndk;30.0.14904198" "cmake;3.22.1"
```

### 3. Open the project

Open the `android/` directory in Android Studio. Gradle will sync automatically and download any remaining dependencies.

### Note on symlinks

The `assets/` directory uses symlinks into the parent repo (fonts, models, shaders, themes, and `NOTICE.md`). These work on macOS and Linux out of the box. On Windows, either enable [Developer Mode](https://learn.microsoft.com/en-us/windows/apps/get-started/enable-your-device-for-development) before cloning or use WSL.

## Requirements

To build:

- JDK 21
- Android SDK Platform 36.1, set by `compileSdk` in `app/build.gradle.kts`
- NDK 30.0.14904198, exactly; `ndkVersion` pins it and AGP will not substitute another
- CMake 3.22.1, exactly; `externalNativeBuild` pins it the same way

Gradle comes from the wrapper, so there is nothing to install for it.

To run:

- Android 10 (API 29) or newer, set by `minSdk`
- An `arm64-v8a` or `x86_64` device or emulator, since those are the only ABIs built
- OpenGL ES 3.0

## Building

```bash
./gradlew assembleDebug
```

Raylib 5.5 is fetched automatically by CMake on the first build. Built ABIs: `arm64-v8a`, `x86_64`.

### Release builds

```bash
./gradlew assembleRelease -PhawkeyeVersionName=0.4.0
```

`hawkeyeVersionName` sets the APK `versionName`, and `versionCode` is derived from it as `major * 100000000 + minor * 100000 + patch * 100 + rc`, where the rc component is `99` for a final version, `N` for an `rcN` suffix, and `0` for `dev` and `ci`, so `0.4.0-rc1` becomes `400001` and `0.4.0` becomes `400099`. The value is parsed strictly: anything that isn't `MAJOR.MINOR.PATCH` with an optional `rcN` suffix, that has a component outside `0..999`, or whose major exceeds `20` fails the build rather than producing a misleading version code. Omit the property and the build falls back to `0.0.0-dev` with version code `1`, which is why `assembleDebug` needs no flags. The release workflow passes the value from the git tag; see [Releasing](https://px4.github.io/Hawkeye/developer/releasing) for the full picture.

The output is `app/build/outputs/apk/release/app-release-unsigned.apk`, unless the upload keystore environment variables are set (see [Release signing](#release-signing)), in which case it is a signed `app-release.apk`. An unsigned APK cannot be installed until you sign it yourself (see below).

To run the same checks CI runs against a release APK (both ABIs present, all four asset trees and `NOTICE.md` packaged, version as expected):

```bash
scripts/verify-release-apk.sh app/build/outputs/apk/release 0.4.0
```

AGP maps the `release` build type to CMake `RelWithDebInfo`, not `Release`. Native objects therefore land in a `.cxx/RelWithDebInfo/` tree separate from the debug one, and the first release build recompiles raylib from scratch even if a debug build is already warm.

## Deploying

```bash
adb install -r app/build/outputs/apk/debug/app-debug.apk
adb shell am start -n com.px4.hawkeye.android/.MainActivity
```

`MainActivity` is the launcher (the Compose shell). `HawkeyeActivity` is `exported="false"` and is launched from within the app, so it can't be started directly from `adb`.

### Release signing

CI signs release builds with the project upload key when the environment provides it. `app/build.gradle.kts` activates the `upload` signing config only when `HAWKEYE_UPLOAD_KEYSTORE` (a path to a decoded keystore), `HAWKEYE_UPLOAD_KEYSTORE_PASSWORD`, and `HAWKEYE_UPLOAD_KEY_ALIAS` are set; the workflows decode the keystore from the `ANDROID_UPLOAD_KEYSTORE_BASE64` repository secret. Without those variables, `assembleRelease` produces an unsigned APK, which is what local builds and forks get.

The release workflow also uploads the signed AAB from `bundleRelease` to the Google Play internal test track. Play App Signing re-signs it with a key Google holds, so a Play install and a sideloaded APK cannot upgrade over each other.

### Signing an APK yourself

A locally built release APK is unsigned, so `adb install` rejects it. Generate a key once, then align and sign before installing. `zipalign` and `apksigner` ship with the SDK build tools; substitute your installed version for `36.0.0`.

```bash
keytool -genkeypair -v -keystore hawkeye.jks -alias hawkeye \
    -keyalg RSA -keysize 2048 -validity 10000

$ANDROID_SDK_ROOT/build-tools/36.0.0/zipalign -p -f 4 \
    app/build/outputs/apk/release/app-release-unsigned.apk hawkeye-aligned.apk

$ANDROID_SDK_ROOT/build-tools/36.0.0/apksigner sign \
    --ks hawkeye.jks --out hawkeye.apk hawkeye-aligned.apk

adb install -r hawkeye.apk
```

Keep the keystore out of the repo. Reusing the same key across builds is what lets you install over a previous version instead of having to uninstall first.

## Continuous Integration

Three CI jobs build this app:

| Workflow      | Job             | Variant | Runs on                                        |
| ------------- | --------------- | ------- | ---------------------------------------------- |
| `android.yml` | `build`         | Debug   | Pull requests, pushes to main, manual dispatch |
| `android.yml` | `release-build` | Release | Pushes to main, manual dispatch                |
| `release.yml` | `android`       | Release | `v*` tag pushes only                           |

`release-build` runs the same `assembleRelease` and `bundleRelease` tasks and the same `scripts/verify-release-apk.sh` and `scripts/verify-release-bundle.sh` checks the release uses, including the signed path when the keystore secrets are present, so a break shows up on a normal merge rather than on a live tag. It is skipped on pull requests to keep review turnaround fast, and can be triggered from a branch with `gh workflow run android.yml --ref <branch>`.

All three jobs share `.github/actions/setup-android-build` for the toolchain install and caching, so the NDK and CMake versions are pinned in one place.

`android.yml` triggers on the repo-root `src/`, `lib/`, `fonts/`, `models/`, `shaders/`, and `themes/` directories as well as `android/`. The native library compiles source files out of the root `src/` tree and `app/src/main/assets/` is symlinks into the root asset directories, so a desktop-side change can break this app and has to run Android CI.

