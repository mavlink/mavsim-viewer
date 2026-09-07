package com.px4.hawkeye.feature.replay.data.di

import androidx.room.Room
import com.px4.hawkeye.feature.replay.data.AndroidReplayFileManager
import com.px4.hawkeye.feature.replay.data.LibraryFileStore
import com.px4.hawkeye.feature.replay.data.ReplayFileManager
import com.px4.hawkeye.feature.replay.data.RoomReplayLibraryRepository
import com.px4.hawkeye.feature.replay.data.db.ReplayDatabase
import com.px4.hawkeye.core.domain.ReplayLibraryRepository
import org.koin.android.ext.koin.androidContext
import org.koin.dsl.module

val replayDataModule = module {
    single {
        Room.databaseBuilder(
            androidContext(),
            ReplayDatabase::class.java,
            "replay_library.db",
        ).build()
    }
    single { get<ReplayDatabase>().libraryDao() }
    single { LibraryFileStore(androidContext().filesDir, crashReporter = get()) }
    single<ReplayFileManager> { AndroidReplayFileManager(androidContext().contentResolver, get()) }
    single<ReplayLibraryRepository> {
        RoomReplayLibraryRepository(dao = get(), fileManager = get(), crashReporter = get())
    }
}
