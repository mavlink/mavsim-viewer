package com.px4.hawkeye.feature.settings.presentation

import com.px4.hawkeye.feature.settings.domain.DistanceUnit
import com.px4.hawkeye.feature.settings.domain.ThemeMode

sealed interface SettingsAction {
    data class OnThemeModeSelected(val mode: ThemeMode) : SettingsAction
    data class OnDistanceUnitSelected(val unit: DistanceUnit) : SettingsAction
    data class OnListenPortChanged(val raw: String) : SettingsAction
    data class OnCrashReportingToggled(val enabled: Boolean) : SettingsAction
}
