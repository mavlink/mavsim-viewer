package com.px4.hawkeye.feature.replay.data

import android.database.sqlite.SQLiteFullException
import com.px4.hawkeye.core.domain.CrashReporter
import com.px4.hawkeye.core.domain.DataError
import com.px4.hawkeye.core.domain.EmptyResult
import com.px4.hawkeye.core.domain.Result
import com.px4.hawkeye.feature.replay.data.db.LibraryEntryEntity
import com.px4.hawkeye.feature.replay.data.db.ReplayLibraryDao
import com.px4.hawkeye.core.domain.LibraryEntry
import com.px4.hawkeye.core.domain.ReplayLibraryRepository
import kotlinx.coroutines.CoroutineDispatcher
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.withContext
import java.util.UUID

/**
 * Library repository backed by a Room DAO for metadata and [ReplayFileManager] for the
 * on-disk payloads. [clock] and [idGenerator] are injected for deterministic tests.
 */
class RoomReplayLibraryRepository(
    private val dao: ReplayLibraryDao,
    private val fileManager: ReplayFileManager,
    private val crashReporter: CrashReporter,
    private val ioDispatcher: CoroutineDispatcher = Dispatchers.IO,
    private val clock: () -> Long = System::currentTimeMillis,
    private val idGenerator: () -> String = { UUID.randomUUID().toString() },
) : ReplayLibraryRepository {

    override fun observeLibrary(): Flow<List<LibraryEntry>> =
        dao.observeAll().map { rows -> rows.map(LibraryEntryEntity::toLibraryEntry) }

    override suspend fun import(uri: String): Result<LibraryEntry, DataError.Local> =
        withContext(ioDispatcher) {
            val id = idGenerator()
            val fileName = "$id.ulg"
            val displayName = fileManager.resolveDisplayName(uri)
            when (val written = fileManager.import(uri, fileName)) {
                is Result.Error -> written
                is Result.Success -> {
                    val entry = LibraryEntry(
                        id = id,
                        displayName = displayName,
                        sizeBytes = written.data,
                        importedAtMillis = clock(),
                    )
                    runCatching { dao.insert(entry.toEntity(fileName)) }.fold(
                        onSuccess = { Result.Success(entry) },
                        onFailure = {
                            // Don't leave an orphaned payload if the metadata row fails.
                            fileManager.delete(fileName)
                            Result.Error(it.toLocalError())
                        },
                    )
                }
            }
        }

    override suspend fun delete(id: String): EmptyResult<DataError.Local> =
        withContext(ioDispatcher) {
            runCatching {
                val row = dao.getById(id)
                if (row == null) {
                    Result.Error(DataError.Local.NOT_FOUND)
                } else {
                    fileManager.delete(row.fileName)
                    dao.deleteById(id)
                    Result.Success(Unit)
                }
            }.getOrElse { Result.Error(it.toLocalError()) }
        }

    override suspend fun deleteAll(ids: List<String>): EmptyResult<DataError.Local> =
        withContext(ioDispatcher) {
            runCatching {
                // Resolve every id before touching the filesystem so an unknown id fails the
                // whole batch instead of leaving a selection half-deleted.
                val fileNames = ids.map { id -> dao.getById(id)?.fileName }
                if (fileNames.isEmpty() || fileNames.any { it == null }) {
                    Result.Error(DataError.Local.NOT_FOUND)
                } else {
                    fileNames.filterNotNull().forEach(fileManager::delete)
                    dao.deleteByIds(ids)
                    Result.Success(Unit)
                }
            }.getOrElse { Result.Error(it.toLocalError()) }
        }

    override suspend fun stageForPlayback(ids: List<String>): EmptyResult<DataError.Local> =
        withContext(ioDispatcher) {
            runCatching {
                // Resolve every id (preserving caller order — it becomes the drone order)
                // before staging anything, so a bad id never disturbs the current inbox.
                val fileNames = ids.map { id -> dao.getById(id)?.fileName }
                if (fileNames.isEmpty() || fileNames.any { it == null }) {
                    Result.Error(DataError.Local.NOT_FOUND)
                } else {
                    fileManager.stage(fileNames.filterNotNull())
                }
            }.getOrElse { Result.Error(it.toLocalError()) }
        }

    // Only the unclassified branch is reported. A full disk is an expected outcome the
    // user already sees; reporting it would bury the defects worth finding.
    private fun Throwable.toLocalError(): DataError.Local = when (this) {
        is SQLiteFullException -> DataError.Local.DISK_FULL
        else -> {
            crashReporter.recordException(this, ORIGIN)
            DataError.Local.UNKNOWN
        }
    }

    private companion object {
        const val ORIGIN = "replay-library-db"
    }
}
