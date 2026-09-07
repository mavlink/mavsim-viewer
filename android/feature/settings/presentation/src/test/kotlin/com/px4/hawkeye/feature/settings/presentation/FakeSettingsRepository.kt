package com.px4.hawkeye.feature.settings.presentation

import com.px4.hawkeye.feature.settings.domain.AppSettings
import com.px4.hawkeye.feature.settings.domain.DistanceUnit
import com.px4.hawkeye.feature.settings.domain.SettingsRepository
import com.px4.hawkeye.feature.settings.domain.ThemeMode
import kotlinx.coroutines.flow.MutableStateFlow

class FakeSettingsRepository : SettingsRepository {
    private val state = MutableStateFlow(AppSettings())
    override val settings = state
    override suspend fun setThemeMode(mode: ThemeMode) { state.value = state.value.copy(themeMode = mode) }
    override suspend fun setDistanceUnit(unit: DistanceUnit) { state.value = state.value.copy(distanceUnit = unit) }
    override suspend fun setListenPort(port: Int) { state.value = state.value.copy(listenPort = port) }
    override suspend fun setCrashReportingEnabled(enabled: Boolean) {
        state.value = state.value.copy(crashReportingEnabled = enabled)
    }
}
