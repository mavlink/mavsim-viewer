package com.px4.hawkeye.core.domain

/**
 * Records unexpected failures for later diagnosis.
 *
 * Deliberately free of any Firebase or Android type so the modules that report through it
 * stay testable on the JVM and gain no dependency on the reporting backend. The only real
 * implementation is the Crashlytics-backed one in `:app`; [None] stands in whenever
 * Firebase is dormant, which is every fork and every local checkout.
 *
 * This is for failures nobody predicted. Expected outcomes that already have a
 * user-visible result — a missing file, a full disk — are modelled as [DataError] and must
 * not be reported, or real defects drown in noise.
 */
interface CrashReporter {

    /**
     * Records [throwable] as a non-fatal event. [origin] names the operation that failed,
     * so the report is attributable without a stack trace that ends in a shared helper.
     */
    fun recordException(throwable: Throwable, origin: String)

    /**
     * Turns collection on or off in response to the user's Settings choice. The backend
     * persists this, so it survives restarts and applies to every process.
     */
    fun setEnabled(enabled: Boolean)

    /**
     * Discards everything. Used where Firebase never initializes (every fork and local
     * checkout) and by tests that do not care about reporting.
     *
     * Deliberately not a default argument on production constructors: a reporter that
     * silently does nothing is a degraded value, and this whole seam exists to stop
     * failures being invisible. Injection sites must name it.
     */
    object None : CrashReporter {
        override fun recordException(throwable: Throwable, origin: String) = Unit
        override fun setEnabled(enabled: Boolean) = Unit
    }
}
