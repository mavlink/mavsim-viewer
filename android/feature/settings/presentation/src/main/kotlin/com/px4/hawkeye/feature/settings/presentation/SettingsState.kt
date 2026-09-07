package com.px4.hawkeye.feature.settings.presentation

import com.px4.hawkeye.core.domain.DEFAULT_LIVE_PORT
import com.px4.hawkeye.core.presentation.UiText
import com.px4.hawkeye.feature.settings.domain.DistanceUnit
import com.px4.hawkeye.feature.settings.domain.ThemeMode

data class SettingsState(
    val themeMode: ThemeMode = ThemeMode.SYSTEM,
    val distanceUnit: DistanceUnit = DistanceUnit.METRIC,
    // Raw text in the port field, so an in-progress/invalid entry can be shown without
    // overwriting the persisted port. Seeded from the saved value, updated on each keystroke.
    val portInput: String = DEFAULT_LIVE_PORT.toString(),
    val portError: UiText? = null,
    val crashReportingEnabled: Boolean = true,
)
