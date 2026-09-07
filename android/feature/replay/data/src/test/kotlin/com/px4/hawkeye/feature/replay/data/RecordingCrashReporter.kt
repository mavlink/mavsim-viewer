package com.px4.hawkeye.feature.replay.data

import com.px4.hawkeye.core.domain.CrashReporter

/** Captures what would have been reported, so tests can assert on it. */
class RecordingCrashReporter : CrashReporter {
    val recorded = mutableListOf<Pair<Throwable, String>>()
    override fun recordException(throwable: Throwable, origin: String) {
        recorded += throwable to origin
    }
    override fun setEnabled(enabled: Boolean) = Unit
}
