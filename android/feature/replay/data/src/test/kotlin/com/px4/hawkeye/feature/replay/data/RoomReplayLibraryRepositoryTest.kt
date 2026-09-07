package com.px4.hawkeye.feature.replay.data

import assertk.assertThat
import assertk.assertions.containsExactly
import assertk.assertions.hasSize
import assertk.assertions.isEmpty
import assertk.assertions.isEqualTo
import assertk.assertions.isNotNull
import assertk.assertions.isNull
import com.px4.hawkeye.core.domain.DataError
import com.px4.hawkeye.core.domain.LibraryEntry
import com.px4.hawkeye.core.domain.Result
import com.px4.hawkeye.feature.replay.data.db.LibraryEntryEntity
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.test.UnconfinedTestDispatcher
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

@OptIn(ExperimentalCoroutinesApi::class)
class RoomReplayLibraryRepositoryTest {

    private val dispatcher = UnconfinedTestDispatcher()
    private lateinit var dao: FakeReplayLibraryDao
    private lateinit var files: FakeReplayFileManager
    private lateinit var crashReporter: RecordingCrashReporter

    @BeforeEach fun setUp() {
        dao = FakeReplayLibraryDao()
        files = FakeReplayFileManager()
        crashReporter = RecordingCrashReporter()
    }

    private fun repository() = RoomReplayLibraryRepository(
        dao = dao,
        fileManager = files,
        crashReporter = crashReporter,
        ioDispatcher = dispatcher,
        clock = { 1_000L },
        idGenerator = { "fixed-id" },
    )

    @Test
    fun `import writes the payload and inserts a metadata row`() = runTest {
        files.importResult = Result.Success(4096L)
        files.displayName = "log.ulg"

        val result = repository().import("content://doc")

        assertThat(result).isEqualTo(
            Result.Success(LibraryEntry("fixed-id", "log.ulg", 4096L, 1_000L)),
        )
        assertThat(files.importedFileNames).containsExactly("fixed-id.ulg")
        assertThat(dao.getById("fixed-id")).isNotNull()
    }

    @Test
    fun `import returns the write error and inserts nothing`() = runTest {
        files.importResult = Result.Error(DataError.Local.DISK_FULL)

        val result = repository().import("content://doc")

        assertThat(result).isEqualTo(Result.Error(DataError.Local.DISK_FULL))
        assertThat(dao.getById("fixed-id")).isNull()
        // A full disk is an expected outcome the user already sees. Reporting it would
        // bury the failures worth acting on.
        assertThat(crashReporter.recorded).isEmpty()
    }

    @Test
    fun `import maps a DAO failure to an error and removes the orphaned payload`() = runTest {
        dao.insertShouldThrow = RuntimeException("db locked")

        val result = repository().import("content://doc")

        assertThat(result).isEqualTo(Result.Error(DataError.Local.UNKNOWN))
        assertThat(files.deletedFileNames).containsExactly("fixed-id.ulg")
    }

    @Test
    fun `an unclassified DAO failure is reported with its origin`() = runTest {
        val boom = RuntimeException("db locked")
        dao.insertShouldThrow = boom

        repository().import("content://doc")

        assertThat(crashReporter.recorded).hasSize(1)
        assertThat(crashReporter.recorded.single().first).isEqualTo(boom)
        assertThat(crashReporter.recorded.single().second).isEqualTo("replay-library-db")
    }

    @Test
    fun `observeLibrary maps rows newest first`() = runTest {
        dao.seed(
            entity("a", importedAt = 10L),
            entity("b", importedAt = 30L),
            entity("c", importedAt = 20L),
        )

        val entries = repository().observeLibrary().first()

        assertThat(entries.map { it.id }).containsExactly("b", "c", "a")
    }

    @Test
    fun `delete removes the payload and the row`() = runTest {
        dao.seed(entity("a"))

        val result = repository().delete("a")

        assertThat(result).isEqualTo(Result.Success(Unit))
        assertThat(files.deletedFileNames).containsExactly("a.ulg")
        assertThat(dao.getById("a")).isNull()
    }

    @Test
    fun `delete returns NOT_FOUND for an unknown id`() = runTest {
        val result = repository().delete("missing")

        assertThat(result).isEqualTo(Result.Error(DataError.Local.NOT_FOUND))
        assertThat(files.deletedFileNames).isEqualTo(emptyList())
    }

    @Test
    fun `deleteAll removes every selected payload and row`() = runTest {
        dao.seed(entity("a"), entity("b"), entity("c"))

        val result = repository().deleteAll(listOf("a", "c"))

        assertThat(result).isEqualTo(Result.Success(Unit))
        assertThat(files.deletedFileNames).containsExactly("a.ulg", "c.ulg")
        assertThat(dao.getById("a")).isNull()
        assertThat(dao.getById("c")).isNull()
        assertThat(dao.getById("b")).isNotNull()
    }

    @Test
    fun `deleteAll fails with NOT_FOUND and deletes nothing when any id is unknown`() = runTest {
        dao.seed(entity("a"))

        val result = repository().deleteAll(listOf("a", "missing"))

        assertThat(result).isEqualTo(Result.Error(DataError.Local.NOT_FOUND))
        assertThat(files.deletedFileNames).isEqualTo(emptyList())
        assertThat(dao.getById("a")).isNotNull()
    }

    @Test
    fun `deleteAll with an empty list returns NOT_FOUND`() = runTest {
        val result = repository().deleteAll(emptyList())

        assertThat(result).isEqualTo(Result.Error(DataError.Local.NOT_FOUND))
        assertThat(files.deletedFileNames).isEqualTo(emptyList())
    }

    @Test
    fun `stageForPlayback stages the entry's payload`() = runTest {
        dao.seed(entity("a"))

        val result = repository().stageForPlayback(listOf("a"))

        assertThat(result).isEqualTo(Result.Success(Unit))
        assertThat(files.stagedBatches).containsExactly(listOf("a.ulg"))
    }

    @Test
    fun `stageForPlayback resolves payloads preserving the caller's id order`() = runTest {
        dao.seed(entity("a"), entity("b"), entity("c"))

        val result = repository().stageForPlayback(listOf("b", "a", "c"))

        assertThat(result).isEqualTo(Result.Success(Unit))
        assertThat(files.stagedBatches).containsExactly(listOf("b.ulg", "a.ulg", "c.ulg"))
    }

    @Test
    fun `stageForPlayback returns NOT_FOUND for an unknown id`() = runTest {
        val result = repository().stageForPlayback(listOf("missing"))

        assertThat(result).isEqualTo(Result.Error(DataError.Local.NOT_FOUND))
    }

    @Test
    fun `stageForPlayback with any unknown id fails before staging anything`() = runTest {
        dao.seed(entity("a"))

        val result = repository().stageForPlayback(listOf("a", "missing"))

        assertThat(result).isEqualTo(Result.Error(DataError.Local.NOT_FOUND))
        assertThat(files.stagedBatches).isEqualTo(emptyList<List<String>>())
    }

    private fun entity(id: String, importedAt: Long = 0L) = LibraryEntryEntity(
        id = id,
        displayName = "$id.ulg",
        sizeBytes = 1L,
        importedAtMillis = importedAt,
        fileName = "$id.ulg",
    )
}
