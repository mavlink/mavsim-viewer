package com.px4.hawkeye.android

import android.app.Application
import com.px4.hawkeye.android.crash.CrashReportingConsent
import com.px4.hawkeye.android.crash.initCrashReporting
import com.px4.hawkeye.android.di.appModule
import com.px4.hawkeye.core.navigation.di.navigationModule
import com.px4.hawkeye.feature.about.presentation.di.aboutPresentationModule
import com.px4.hawkeye.feature.home.presentation.di.homePresentationModule
import com.px4.hawkeye.feature.live.data.di.liveDataModule
import com.px4.hawkeye.feature.live.presentation.di.livePresentationModule
import com.px4.hawkeye.feature.replay.data.di.replayDataModule
import com.px4.hawkeye.feature.replay.presentation.di.replayPresentationModule
import com.px4.hawkeye.feature.settings.data.di.settingsDataModule
import com.px4.hawkeye.feature.settings.presentation.di.settingsPresentationModule
import org.koin.android.ext.koin.androidContext
import org.koin.core.context.startKoin

class HawkeyeApp : Application() {
    override fun onCreate() {
        super.onCreate()

        // Before the process guard below, because the renderer returns early and is exactly
        // where Crashlytics is most needed: Firebase's own ContentProvider does not reach a
        // secondary process, so without this the native signal handler never installs.
        initCrashReporting(this, Application.getProcessName())

        // The renderer runs in a separate ":renderer" process (see HawkeyeActivity) and is a
        // bare NativeActivity that needs no DI. Only wire Koin in the main process — this also
        // prevents two processes from opening the Room database concurrently.
        if (Application.getProcessName() != packageName) return
        val koin = startKoin {
            androidContext(this@HawkeyeApp)
            modules(
                appModule,
                navigationModule,
                homePresentationModule,
                settingsDataModule,
                settingsPresentationModule,
                aboutPresentationModule,
                replayDataModule,
                replayPresentationModule,
                liveDataModule,
                livePresentationModule,
            )
        }.koin

        koin.get<CrashReportingConsent>().start()
    }
}
