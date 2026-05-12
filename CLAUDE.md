# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Layout

This is DJI's Mobile SDK V5 for Android — a sample Android app that demonstrates how to integrate `dji-sdk-v5-aircraft` (Maven `com.dji:dji-sdk-v5-aircraft:5.17.0`) to control DJI drones. It is **not the SDK source itself**; the SDK ships as AARs pulled from Maven. Two source modules live under `SampleCode-V5/`:

- `android-sdk-v5-as/` — the **Gradle root**. All builds run from here. Contains `settings.gradle`, `dependencies.gradle`, `gradle.properties`, the wrapper, and the shared keystore (`msdkkeystore.jks`).
- `android-sdk-v5-sample/` (module `:sample`) — the demo app (`applicationId com.dji.sampleV5.aircraft`). Per-feature fragments in `pages/` paired with ViewModels in `models/`.
- `android-sdk-v5-uxsdk/` (module `:uxsdk`) — a library module (`dji.v5.ux`) of reusable widgets, panels, and `DefaultLayoutActivity` / `WidgetsActivity` showcase screens. `:sample` depends on `:uxsdk`.

This fork has custom additions beyond the upstream DJI sample — notably `ArucoFollowFragment` (OpenCV-based ArUco marker tracking via *basic* VirtualStick), a **LAN Dashboard Server** (embedded NanoHTTPD/WebSocket server in `dji.sampleV5.aircraft.dashboard` + `DashboardServerVM` that broadcasts MJPEG-over-WebSocket video and JSON telemetry to a browser on the same Wi-Fi and accepts JSON command frames back for remote control — takeoff/land/enableVS/startTrack/setGain/setArucoMode/setAutoLand; the same VM also runs an ArUco tracking pipeline using DJI's **advanced VirtualStick** API — `sendVirtualStickAdvancedParam` with BODY-frame ANGLE-mode roll/pitch (degrees of tilt), pumped at 20 Hz; the mobile side ships unannotated NV21→JPEG frames (no OpenCV drawing) and publishes marker corners + frame dimensions in the telemetry JSON, and the browser draws all HUD overlays (reticle, deadzone bounds, marker bbox, offset vector, roll command bar) as SVG over the `<img>` — so visual styles can be iterated without rebuilding the APK; basic-stick mode and VELOCITY mode were both abandoned because the FC's noise filter (basic) and velocity-tracker filter (VELOCITY) swallow small corrections after a hover settles, causing the drone to ignore tracking inputs — the "stuck after centering" bug that still afflicts `ArucoFollowFragment`. Tradeoff: ANGLE mode bypasses obstacle-avoidance; the dashboard exposes a **Safety / Obstacle Avoidance** section that wraps `PerceptionManager.setObstacleAvoidanceType`/`setObstacleAvoidanceEnabled` and surfaces live `PerceptionInfo` + `ObstacleData` (per-direction enable state, sensor working flags, nearest-obstacle distance per H/U/D) so the operator can verify OA is actually braking before flying; static dashboard lives at `assets/dashboard/index.html`), and the "agroz autonomous drone" work referenced in recent commits.

## Build & Run

All Gradle commands must run from `SampleCode-V5/android-sdk-v5-as/`:

```powershell
cd SampleCode-V5\android-sdk-v5-as
.\gradlew.bat :sample:assembleDebug          # Build debug APK
.\gradlew.bat :sample:installDebug           # Install to connected device
.\gradlew.bat :sample:assembleRelease        # Build release (signed with bundled keystore)
.\gradlew.bat :uxsdk:assembleDebug           # Build the UX library only
.\gradlew.bat clean
```

Tests use JUnit4 + AndroidX Test / Espresso (declared in `dependencies.gradle`); run with `:sample:testDebugUnitTest` / `:sample:connectedDebugAndroidTest`. There are no dedicated lint tasks — `lintOptions { checkReleaseBuilds false; abortOnError false }` in `:sample`.

### Required configuration before first build

Edit `android-sdk-v5-as/gradle.properties`:

- `AIRCRAFT_API_KEY` — DJI Mobile SDK app key registered for `applicationId com.dji.sampleV5.aircraft` at developer.dji.com. The current value in the file is committed and must match the applicationId, so renaming the app requires a new key.
- `GMAP_API_KEY` — Google Maps key (placeholder `ENTER YOUR Google Map API KEY`).
- `MAPLIBRE_TOKEN` — MapLibre token (placeholder `ENTER YOUR MapLibre TOKEN`).

These are injected as manifest placeholders (`${API_KEY}`, `${GMAP_API_KEY}`, `${MAPLIBRE_TOKEN}`) — they're not read from code, so a missing value silently fails registration at runtime rather than at build time.

### Build constraints to know

- **arm64-v8a only**: `:sample` sets `ndk { abiFilters 'arm64-v8a' }`. Emulators and 32-bit devices will not run this build.
- **minSdk 24, target/compile 35, Kotlin 2.1.0, AGP 8.7.0, JVM target 1.8, NDK 21.4.7075529**. All centralized in `gradle.properties` / `dependencies.gradle` — don't override per-module.
- **Debug builds use the release keystore** (`signingConfigs.release` is wired into both `debug` and `release` block in `:sample/build.gradle`). The keystore password (`123456`) is committed; the key only signs the demo APK.
- Many native libs are listed in `packagingOptions.doNotStrip` (DJI core, FFmpeg, mrtc_*, agora). Don't add ProGuard/R8 rules that strip these.
- `compileOnly deps.aircraftProvided` + `implementation deps.aircraft` is the required dependency pattern for any module touching the MSDK — `aircraft-provided` is interfaces only and must not leak into the runtime classpath.

## High-Level Architecture

### SDK lifecycle (entry point)

`DJIAircraftApplication` → `DJIApplication.onCreate()` → `MSDKManagerVM.initMobileSDK(context)`.

`MSDKManagerVM` (a `ViewModel` held in an app-scoped store via `globalViewModels()`) calls `SDKManager.getInstance().init(...)` with a `SDKManagerCallback`. The callback drives five `MutableLiveData` streams that the rest of the app observes:

- `lvRegisterState` — registration success/failure
- `lvProductConnectionState` / `lvProductChanges` — physical drone connect/disconnect
- `lvInitProcess` — init progress (when it hits `INITIALIZE_COMPLETE`, `MSDKManagerVM` calls `SDKManager.registerApp()`)
- `lvDBDownloadProgress` — fly-zone DB download

Anything that needs to wait for a registered SDK should observe these LiveDatas (typically via `BaseMainActivityVm` / `MSDKInfoVm`), not poll. `DJIAircraftApplication.attachBaseContext` also calls `com.cySdkyc.clx.Helper.install(this)` — that's a DJI-provided init shim and must run before `super.attachBaseContext`.

### Activity / navigation structure

`DJIAircraftMainActivity` extends abstract `DJIMainActivity`. Its `prepareUxActivity()` hook wires up the UX SDK showcases (`DefaultLayoutActivity`, `WidgetsActivity`) and initializes `UxSharedPreferencesUtil`, `GlobalPreferencesManager`, and `GeoidManager` — these three init calls must happen before any UX widget is inflated.

Navigation uses three AndroidX nav graphs:

- `nav_main.xml` — top-level menu screens
- `nav_aircraft.xml` — aircraft-specific feature pages
- `nav_common.xml` — pages shared across product types

`AircraftFragmentPageInfoFactory` (in `data/`) is the single source of truth for what appears in the main menu list — adding a new demo screen means: create the fragment in `pages/`, register its destination in `nav_aircraft.xml`, then add a `FragmentPageItem(...)` entry here.

### MVVM convention

Each feature is one fragment in `pages/` + one ViewModel in `models/` (e.g. `VirtualStickFragment` ↔ `VirtualStickVM`, `RTKCenterFragment` ↔ `RTKCenterVM`). ViewModels expose `MutableLiveData` / `LiveData` for state and call DJI manager singletons (`SDKManager`, `KeyManager`, `VirtualStickManager`, `MediaDataCenter`, `FlightControllerKey`, etc.) directly — there is no repository layer. `globalViewModels()` is a small helper for app-scoped sharing of `MSDKManagerVM`-style state across activities.

### KeyValue subsystem

`keyvalue/` (split between Java and Kotlin) is a generic key inspection / get / set / action / listen UI driven by `KeyItem` + `KeyItemAdapter`. Most diagnostic pages (`KeyValueFragment`, `DiagnosticFragment`) plug into this — when adding a new key category, extend the relevant `KeyItem*` and register it via `KeyItemDataUtil`.

### UX SDK package map (`dji.v5.ux.*`)

Widgets are grouped by hardware concern: `flight/`, `gimbal/`, `remotecontroller/`, `cameracore/`, `visualcamera/`, `obstacle/`, `accessory/`, `warning/`, `map/`, `mapkit/`, `training/`. The `core/` package holds shared infrastructure: `base/` (lifecycle-aware widget base classes), `communication/` (global pref + observable buses, including `GlobalPreferencesManager`), `panel/`, `widget/`, `ui/`, `util/`. The `sample/showcase/` package is what gets launched from `DJIAircraftMainActivity` — `defaultlayout/` is the production-style layout; `widgetlist/` is a flat catalog for manual QA.

## Conventions and gotchas specific to this codebase

- The DJI SDK is registered against `applicationId` — changing the applicationId in `:sample/build.gradle` invalidates the API key and the SDK will fail to register with an `INVALID_APP_KEY` error at runtime.
- `:uxsdk` sets `resourcePrefix "uxsdk_"` — all resources in that module must start with `uxsdk_` or the build fails.
- LeakCanary is shipped in `:sample`'s implementation deps (not `debugImplementation`), so it runs in release builds too. Be intentional before removing it.
- OpenCV (`org.opencv:opencv:4.10.0`) is pulled in by `:sample` and used by `ArucoFollowFragment` — it loads via `OpenCVLoader.initLocal()` / `initDebug()` patterns. Don't add `OpenCVLoaderCallback` callbacks; this version supports synchronous load.
- The repo has both an English (`README.md`) and a Chinese (`README_CN.md`) README, plus generated API docs under `Docs/Android_API/{en,cn}/`. When updating user-facing docs, update both languages or call out that one is intentionally lagging.
