package com.px4.hawkeye.android.crash

import assertk.assertThat
import assertk.assertions.containsExactly
import com.px4.hawkeye.core.domain.CrashReporter
import com.px4.hawkeye.feature.settings.domain.AppSettings
import com.px4.hawkeye.feature.settings.domain.DistanceUnit
import com.px4.hawkeye.feature.settings.domain.SettingsRepository
import com.px4.hawkeye.feature.settings.domain.ThemeMode
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.test.UnconfinedTestDispatcher
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Test

@OptIn(ExperimentalCoroutinesApi::class)
class CrashReportingConsentTest {

    private class FakeSettingsRepository : SettingsRepository {
        private val state = MutableStateFlow(AppSettings())
        override val settings = state
        override suspend fun setThemeMode(mode: ThemeMode) = Unit
        override suspend fun setDistanceUnit(unit: DistanceUnit) = Unit
        override suspend fun setListenPort(port: Int) = Unit
        override suspend fun setCrashReportingEnabled(enabled: Boolean) {
            state.value = state.value.copy(crashReportingEnabled = enabled)
        }
    }

    private class RecordingCrashReporter : CrashReporter {
        val enabledCalls = mutableListOf<Boolean>()
        override fun recordException(throwable: Throwable, origin: String) = Unit
        override fun setEnabled(enabled: Boolean) { enabledCalls += enabled }
    }

    @Test
    fun `pushes the persisted choice through on start`() = runTest(UnconfinedTestDispatcher()) {
        val reporter = RecordingCrashReporter()
        val job = CrashReportingConsent(FakeSettingsRepository(), reporter, this).start()

        // Collection defaults on, so a fresh install must arrive as enabled rather than
        // leaving the reporter on whatever the SDK last persisted.
        assertThat(reporter.enabledCalls).containsExactly(true)

        job.cancel()
    }

    @Test
    fun `opting out and back in reaches the reporter`() = runTest(UnconfinedTestDispatcher()) {
        val settings = FakeSettingsRepository()
        val reporter = RecordingCrashReporter()
        val job = CrashReportingConsent(settings, reporter, this).start()

        settings.setCrashReportingEnabled(false)
        settings.setCrashReportingEnabled(true)

        assertThat(reporter.enabledCalls).containsExactly(true, false, true)

        job.cancel()
    }

    @Test
    fun `an unchanged setting does not re-notify`() = runTest(UnconfinedTestDispatcher()) {
        val settings = FakeSettingsRepository()
        val reporter = RecordingCrashReporter()
        val job = CrashReportingConsent(settings, reporter, this).start()

        // An unrelated settings write re-emits AppSettings; the reporter must not be
        // touched again, or every theme change would rewrite the SDK's persisted flag.
        settings.setCrashReportingEnabled(true)
        settings.setThemeMode(ThemeMode.DARK)

        assertThat(reporter.enabledCalls).containsExactly(true)

        job.cancel()
    }
}
