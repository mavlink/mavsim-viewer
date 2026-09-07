package com.px4.hawkeye.feature.settings.data

import android.content.Context
import androidx.datastore.preferences.core.booleanPreferencesKey
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.intPreferencesKey
import androidx.datastore.preferences.core.stringPreferencesKey
import androidx.datastore.preferences.preferencesDataStore
import com.px4.hawkeye.core.domain.DEFAULT_LIVE_PORT
import com.px4.hawkeye.feature.settings.domain.AppSettings
import com.px4.hawkeye.feature.settings.domain.DistanceUnit
import com.px4.hawkeye.feature.settings.domain.SettingsRepository
import com.px4.hawkeye.feature.settings.domain.ThemeMode
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map

private val Context.dataStore by preferencesDataStore(name = "hawkeye_settings")
private val THEME = stringPreferencesKey("theme_mode")
private val UNIT = stringPreferencesKey("distance_unit")
private val LISTEN_PORT = intPreferencesKey("listen_port")
private val CRASH_REPORTING = booleanPreferencesKey("crash_reporting_enabled")

internal fun parseThemeMode(name: String?): ThemeMode =
    ThemeMode.entries.firstOrNull { it.name == name } ?: ThemeMode.SYSTEM

internal fun parseDistanceUnit(name: String?): DistanceUnit =
    DistanceUnit.entries.firstOrNull { it.name == name } ?: DistanceUnit.METRIC

class DataStoreSettingsRepository(private val context: Context) : SettingsRepository {

    override val settings: Flow<AppSettings> = context.dataStore.data.map { prefs ->
        AppSettings(
            themeMode = parseThemeMode(prefs[THEME]),
            distanceUnit = parseDistanceUnit(prefs[UNIT]),
            listenPort = prefs[LISTEN_PORT] ?: DEFAULT_LIVE_PORT,
            // Absent means the user has never touched the switch, which is consent on.
            crashReportingEnabled = prefs[CRASH_REPORTING] ?: true,
        )
    }

    override suspend fun setThemeMode(mode: ThemeMode) {
        context.dataStore.edit { it[THEME] = mode.name }
    }

    override suspend fun setDistanceUnit(unit: DistanceUnit) {
        context.dataStore.edit { it[UNIT] = unit.name }
    }

    override suspend fun setListenPort(port: Int) {
        context.dataStore.edit { it[LISTEN_PORT] = port }
    }

    override suspend fun setCrashReportingEnabled(enabled: Boolean) {
        context.dataStore.edit { it[CRASH_REPORTING] = enabled }
    }
}
