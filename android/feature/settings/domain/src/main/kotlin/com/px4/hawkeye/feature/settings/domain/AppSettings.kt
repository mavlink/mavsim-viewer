package com.px4.hawkeye.feature.settings.domain

import com.px4.hawkeye.core.domain.DEFAULT_LIVE_PORT

data class AppSettings(
    val themeMode: ThemeMode = ThemeMode.SYSTEM,
    val distanceUnit: DistanceUnit = DistanceUnit.METRIC,
    val listenPort: Int = DEFAULT_LIVE_PORT,
    // Defaults on, matching the manifest's firebase_crashlytics_collection_enabled. The
    // user can turn it off in Settings; see docs/privacy.md.
    val crashReportingEnabled: Boolean = true,
)
