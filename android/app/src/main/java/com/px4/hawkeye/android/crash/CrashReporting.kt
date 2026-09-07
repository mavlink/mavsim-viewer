package com.px4.hawkeye.android.crash

import android.content.Context
import com.google.firebase.FirebaseApp
import com.google.firebase.crashlytics.FirebaseCrashlytics
import com.px4.hawkeye.core.domain.CrashReporter

/**
 * Starts Crashlytics in the calling process and labels which process that is.
 *
 * Firebase normally bootstraps itself from `FirebaseInitProvider`, so most apps never call
 * this. Hawkeye has to: that provider is declared with no `android:process`, and Android
 * instantiates a provider only in the process its declaration names — the default one.
 * ":renderer", where every line of native code runs, is therefore never initialized by it.
 * Skipping this call means no NDK signal handler in the one process that can segfault.
 *
 * [FirebaseApp.initializeApp] is idempotent, so calling it in the main process too is
 * harmless. It returns null when no `google_app_id` resource was baked in, which is every
 * fork and every local checkout: nothing to start, and nothing to report.
 */
fun initCrashReporting(context: Context, processName: String) {
    FirebaseApp.initializeApp(context) ?: return
    // Distinguishes a Compose/Kotlin failure in the main process from a native one in the
    // renderer, which otherwise arrive looking alike.
    FirebaseCrashlytics.getInstance().setCustomKey(PROCESS_KEY, processName)
}

/**
 * Picks the live reporter or the no-op, depending on whether [initCrashReporting] managed
 * to start Firebase in this process. Must be called after it.
 */
fun createCrashReporter(context: Context): CrashReporter =
    if (FirebaseApp.getApps(context).isNotEmpty()) FirebaseCrashReporter() else CrashReporter.None

private const val PROCESS_KEY = "process"
