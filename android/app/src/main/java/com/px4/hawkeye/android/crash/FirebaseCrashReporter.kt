package com.px4.hawkeye.android.crash

import com.google.firebase.crashlytics.CustomKeysAndValues
import com.google.firebase.crashlytics.FirebaseCrashlytics
import com.px4.hawkeye.core.domain.CrashReporter

/**
 * Crashlytics-backed reporter. Only constructed once Firebase is confirmed initialized
 * (see AppModule), so [crashlytics] is safe to resolve.
 */
class FirebaseCrashReporter(
    private val crashlytics: FirebaseCrashlytics = FirebaseCrashlytics.getInstance(),
) : CrashReporter {

    override fun recordException(throwable: Throwable, origin: String) {
        // Scoped keys rather than setCustomKey: the latter is sticky and would attach this
        // origin to every later report from the same session.
        crashlytics.recordException(
            throwable,
            CustomKeysAndValues.Builder().putString(ORIGIN_KEY, origin).build(),
        )
    }

    override fun setEnabled(enabled: Boolean) {
        crashlytics.setCrashlyticsCollectionEnabled(enabled)
    }

    private companion object {
        const val ORIGIN_KEY = "origin"
    }
}
