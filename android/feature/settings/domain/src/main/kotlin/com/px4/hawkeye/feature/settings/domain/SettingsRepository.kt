package com.px4.hawkeye.feature.settings.domain

import kotlinx.coroutines.flow.Flow

interface SettingsRepository {
    val settings: Flow<AppSettings>
    suspend fun setThemeMode(mode: ThemeMode)
    suspend fun setDistanceUnit(unit: DistanceUnit)
    suspend fun setListenPort(port: Int)
    suspend fun setCrashReportingEnabled(enabled: Boolean)
}
