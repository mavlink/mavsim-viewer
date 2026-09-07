import com.google.firebase.crashlytics.buildtools.gradle.CrashlyticsExtension

plugins {
    // Must stay first: the Crashlytics plugin applied below refuses to load unless
    // com.android.application is already on the project, and this convention plugin is
    // what applies it.
    id("hawkeye.android.application")
}

// Release builds take their version from the git tag, passed as
// -PhawkeyeVersionName=<x.y.z> by .github/workflows/release.yml. Local and CI debug
// builds fall back to a dev version so no extra flags are needed.
val hawkeyeVersionName: String =
    providers.gradleProperty("hawkeyeVersionName").getOrElse("0.0.0-dev")

// Monotonic code derived from the semver: 0.4.0-rc1 -> 400001, 0.4.0 -> 400099. The
// derivation and its rules live in build-logic (VersionCode.kt) behind a unit test,
// because a silently wrong code would brick upgrades and ship in a release.
val hawkeyeVersionCode: Int = com.px4.hawkeye.buildlogic.hawkeyeVersionCode(hawkeyeVersionName)

// Release signing activates only when the environment provides an upload keystore
// (HAWKEYE_UPLOAD_KEYSTORE is a path to a decoded .jks). CI injects it from repository
// secrets; local and PR builds have none and keep producing an unsigned release APK.
// Blank counts as absent because a workflow env mapping of an unset secret yields an
// empty string, and that case has to stay on the unsigned path.
val uploadKeystorePath: String? = providers.environmentVariable("HAWKEYE_UPLOAD_KEYSTORE")
    .orNull?.takeIf { it.isNotBlank() }

// Crashlytics activates only when google-services.json is present. CI decodes it from a
// repository secret; local checkouts and forks have none and build with Firebase dormant,
// exactly like the unsigned-release path above. The google-services plugin fails the build
// outright when the file is missing, so it has to be applied conditionally rather than
// declared in the plugins block. Both plugins are on the classpath via `apply false` in
// android/build.gradle.kts.
val firebaseEnabled: Boolean = file("google-services.json").exists()
if (firebaseEnabled) {
    apply(plugin = "com.google.gms.google-services")
    apply(plugin = "com.google.firebase.crashlytics")
}

android {
    namespace = "com.px4.hawkeye.android"
    compileSdk {
        version = release(36) {
            minorApiLevel = 1
        }
    }

    ndkVersion = "30.0.14904198"

    externalNativeBuild {
        cmake {
            path = file("src/main/cpp/CMakeLists.txt")
            version = "3.22.1"
        }
    }

    defaultConfig {
        applicationId = "com.px4.hawkeye.android"
        targetSdk = 36
        versionCode = hawkeyeVersionCode
        versionName = hawkeyeVersionName

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"

        externalNativeBuild {
            cmake {
                // Changing this list means updating android/scripts/verify-release-apk.sh,
                // android/scripts/verify-release-bundle.sh, and the ABI list documented
                // in docs/installation.md, docs/developer/releasing.md,
                // docs/troubleshooting.md, and README.md.
                abiFilters += listOf("arm64-v8a", "x86_64")
            }
        }
    }

    signingConfigs {
        if (uploadKeystorePath != null) {
            val password = providers.environmentVariable("HAWKEYE_UPLOAD_KEYSTORE_PASSWORD").orNull
            val alias = providers.environmentVariable("HAWKEYE_UPLOAD_KEY_ALIAS").orNull
            // Checked here rather than left to .get() so a half-configured environment
            // fails at configuration time with the trio contract spelled out, not deep
            // in :app:packageRelease. isNullOrEmpty, not isPresent: an env mapping of an
            // unset secret arrives as an empty string.
            require(!password.isNullOrEmpty() && !alias.isNullOrEmpty()) {
                "HAWKEYE_UPLOAD_KEYSTORE is set, so HAWKEYE_UPLOAD_KEYSTORE_PASSWORD and " +
                    "HAWKEYE_UPLOAD_KEY_ALIAS must be set too"
            }
            create("upload") {
                storeFile = file(uploadKeystorePath)
                storePassword = password
                keyAlias = alias
                // The upload keystore is PKCS12, which has a single password.
                keyPassword = password
            }
        }
    }

    buildTypes {
        release {
            if (uploadKeystorePath != null) {
                signingConfig = signingConfigs.getByName("upload")
            }
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
            // Upload libhawkeye.so's DWARF to Crashlytics so native crashes in the
            // renderer resolve to a function and line instead of a hex address. AGP builds
            // release native code as RelWithDebInfo (-O2 -g) and the unstripped objects
            // survive in merged_native_libs, so the symbols already exist; this only sends
            // them. Debug is left at the default (off) — uploading on every local build
            // would cost minutes and tell us nothing.
            if (firebaseEnabled) {
                configure<CrashlyticsExtension> {
                    nativeSymbolUploadEnabled = true
                }
            }
        }
    }

    testOptions { unitTests.all { it.useJUnitPlatform() } }
}

dependencies {
    implementation(project(":core:domain"))
    implementation(project(":core:presentation"))
    implementation(project(":core:design-system"))
    implementation(project(":feature:replay:data"))
    implementation(project(":feature:replay:presentation"))
    implementation(project(":core:navigation"))
    implementation(project(":feature:home:presentation"))
    implementation(project(":feature:settings:domain"))
    implementation(project(":feature:settings:data"))
    implementation(project(":feature:settings:presentation"))
    implementation(project(":feature:about:presentation"))
    implementation(project(":feature:live:domain"))
    implementation(project(":feature:live:data"))
    implementation(project(":feature:live:presentation"))

    implementation(libs.androidx.core.ktx)
    implementation(libs.kotlinx.coroutines.android)

    implementation(libs.androidx.lifecycle.runtime.ktx)
    implementation(libs.androidx.lifecycle.viewmodel)
    implementation(libs.androidx.lifecycle.viewmodel.compose)
    implementation(libs.androidx.savedstate.ktx)

    implementation(libs.koin.android)
    implementation(libs.koin.androidx.compose)

    // Unconditional on purpose: :app references these types, so making them conditional
    // would break compilation for forks. Without google-services.json the SDK simply never
    // initializes and AppModule falls back to CrashReporter.None.
    implementation(platform(libs.firebase.bom))
    implementation(libs.firebase.crashlytics)
    implementation(libs.firebase.crashlytics.ndk)

    implementation(libs.androidx.navigation3.runtime)
    implementation(libs.androidx.navigation3.ui)
    implementation(libs.androidx.compose.material3.adaptive.navigation.suite)

    testImplementation(libs.junit.jupiter)
    testRuntimeOnly(libs.junit.jupiter.engine)
    testRuntimeOnly(libs.junit.platform.launcher)
    testImplementation(libs.assertk)
    testImplementation(libs.kotlinx.coroutines.test)

    androidTestImplementation(platform(libs.androidx.compose.bom))
    androidTestImplementation(libs.androidx.junit)
    androidTestImplementation(libs.androidx.espresso.core)
    androidTestImplementation(libs.androidx.compose.ui.test.junit4)
    // Pin window-core to the version the app runs on so the layout test can use the
    // modern WindowSizeClass(Int, Int) constructor (the adaptive lib only pulls 1.3.0).
    androidTestImplementation(libs.androidx.window)
    debugImplementation(libs.androidx.compose.ui.test.manifest)
}
