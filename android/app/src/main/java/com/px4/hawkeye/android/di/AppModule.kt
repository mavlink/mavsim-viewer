package com.px4.hawkeye.android.di

import com.px4.hawkeye.android.about.AndroidAboutInfoProvider
import com.px4.hawkeye.android.crash.CrashReportingConsent
import com.px4.hawkeye.android.crash.createCrashReporter
import com.px4.hawkeye.android.live.AndroidLivePlaybackLauncher
import com.px4.hawkeye.android.replay.AndroidReplayPlaybackLauncher
import com.px4.hawkeye.android.shell.ShellViewModel
import com.px4.hawkeye.core.domain.CrashReporter
import com.px4.hawkeye.core.presentation.AboutInfoProvider
import com.px4.hawkeye.core.presentation.LivePlaybackLauncher
import com.px4.hawkeye.core.presentation.ReplayPlaybackLauncher
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import org.koin.android.ext.koin.androidContext
import org.koin.core.module.dsl.viewModelOf
import org.koin.dsl.module

/** App-level DI: shell-scoped ViewModels and seams that wire features to the renderer. */
val appModule = module {
    viewModelOf(::ShellViewModel)
    single<ReplayPlaybackLauncher> { AndroidReplayPlaybackLauncher() }
    single<LivePlaybackLauncher> { AndroidLivePlaybackLauncher(get()) }
    single<AboutInfoProvider> { AndroidAboutInfoProvider(get()) }
    // Resolved once: whether Firebase is up is fixed for the life of the process, so this
    // is a build-shape decision rather than something to re-check per call site.
    single<CrashReporter> { createCrashReporter(androidContext()) }
    // The scope is built here rather than bound as a type of its own: nothing else should
    // be able to inject a bare application-lifetime CoroutineScope by accident.
    single {
        CrashReportingConsent(
            settings = get(),
            reporter = get(),
            scope = CoroutineScope(SupervisorJob() + Dispatchers.Default),
        )
    }
}
