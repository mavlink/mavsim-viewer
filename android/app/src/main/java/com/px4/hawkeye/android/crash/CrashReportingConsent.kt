package com.px4.hawkeye.android.crash

import com.px4.hawkeye.core.domain.CrashReporter
import com.px4.hawkeye.feature.settings.domain.SettingsRepository
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.launch

/**
 * Mirrors the user's Settings choice into the crash reporter.
 *
 * The reporter persists the flag itself, and every process reads it at startup, so the
 * renderer picks up a change on its next launch — which is always, because HawkeyeActivity
 * halts its process on the way out. Only the main process runs this: DataStore is
 * single-process here, the same reason Koin is.
 *
 * Extracted from [com.px4.hawkeye.android.HawkeyeApp] so the one privacy-critical path in
 * the app can be unit-tested with a fake repository instead of needing a device.
 */
class CrashReportingConsent(
    private val settings: SettingsRepository,
    private val reporter: CrashReporter,
    private val scope: CoroutineScope,
) {
    /** Collects for the life of [scope], which is the life of the process. */
    fun start(): Job = scope.launch {
        settings.settings
            .map { it.crashReportingEnabled }
            .distinctUntilChanged()
            .collect(reporter::setEnabled)
    }
}
