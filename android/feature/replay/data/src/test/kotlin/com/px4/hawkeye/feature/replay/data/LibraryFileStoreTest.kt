package com.px4.hawkeye.feature.replay.data

import assertk.assertThat
import assertk.assertions.isEqualTo
import assertk.assertions.isFalse
import assertk.assertions.isTrue
import assertk.assertions.hasSize
import assertk.assertions.isEmpty
import com.px4.hawkeye.core.domain.CrashReporter
import com.px4.hawkeye.core.domain.DataError
import com.px4.hawkeye.core.domain.Result
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.io.TempDir
import java.io.ByteArrayInputStream
import java.io.File

class LibraryFileStoreTest {

    private fun store(
        dir: File,
        now: Long = 1_000L,
        crashReporter: CrashReporter = CrashReporter.None,
    ) = LibraryFileStore(dir, crashReporter, clock = { now })

    @Test
    fun `write copies bytes into the library and returns the size`(@TempDir dir: File) {
        val bytes = "hello ulog".toByteArray()

        val result = store(dir).write(ByteArrayInputStream(bytes), "abc.ulg")

        assertThat(result).isEqualTo(Result.Success(bytes.size.toLong()))
        val written = File(File(dir, "library"), "abc.ulg")
        assertThat(written.exists()).isTrue()
        assertThat(written.readBytes().toList()).isEqualTo(bytes.toList())
    }

    @Test
    fun `stage copies the library file into the inbox and writes the millis token`(@TempDir dir: File) {
        val store = store(dir, now = 4_242L)
        val bytes = "payload".toByteArray()
        store.write(ByteArrayInputStream(bytes), "abc.ulg")

        val result = store.stage("abc.ulg")

        assertThat(result).isEqualTo(Result.Success(Unit))
        val current = File(File(dir, "inbox"), "current.ulg")
        val ready = File(File(dir, "inbox"), ".ready")
        assertThat(current.readBytes().toList()).isEqualTo(bytes.toList())
        assertThat(ready.readText()).isEqualTo("4242")
    }

    @Test
    fun `re-staging overwrites the inbox file`(@TempDir dir: File) {
        val store = store(dir)
        store.write(ByteArrayInputStream("first".toByteArray()), "a.ulg")
        store.write(ByteArrayInputStream("second".toByteArray()), "b.ulg")

        store.stage("a.ulg")
        store.stage("b.ulg")

        val current = File(File(dir, "inbox"), "current.ulg")
        assertThat(current.readText()).isEqualTo("second")
    }

    @Test
    fun `staging a missing file fails with NOT_FOUND`(@TempDir dir: File) {
        val result = store(dir).stage("ghost.ulg")

        assertThat(result).isEqualTo(Result.Error(DataError.Local.NOT_FOUND))
    }

    @Test
    fun `staging several files writes current plus swarm files in order with a count token`(@TempDir dir: File) {
        val store = store(dir, now = 4_242L)
        store.write(ByteArrayInputStream("alpha".toByteArray()), "a.ulg")
        store.write(ByteArrayInputStream("bravo".toByteArray()), "b.ulg")
        store.write(ByteArrayInputStream("charlie".toByteArray()), "c.ulg")

        val result = store.stage(listOf("a.ulg", "b.ulg", "c.ulg"))

        assertThat(result).isEqualTo(Result.Success(Unit))
        val inbox = File(dir, "inbox")
        assertThat(File(inbox, "current.ulg").readText()).isEqualTo("alpha")
        assertThat(File(inbox, "swarm_1.ulg").readText()).isEqualTo("bravo")
        assertThat(File(inbox, "swarm_2.ulg").readText()).isEqualTo("charlie")
        assertThat(File(inbox, ".ready").readText()).isEqualTo("4242 3")
    }

    @Test
    fun `staging a single-element list keeps the legacy token format`(@TempDir dir: File) {
        val store = store(dir, now = 4_242L)
        store.write(ByteArrayInputStream("alpha".toByteArray()), "a.ulg")

        val result = store.stage(listOf("a.ulg"))

        assertThat(result).isEqualTo(Result.Success(Unit))
        val inbox = File(dir, "inbox")
        assertThat(File(inbox, "current.ulg").readText()).isEqualTo("alpha")
        assertThat(File(inbox, ".ready").readText()).isEqualTo("4242")
    }

    @Test
    fun `staging fewer files removes stale swarm files from a previous session`(@TempDir dir: File) {
        val store = store(dir)
        store.write(ByteArrayInputStream("alpha".toByteArray()), "a.ulg")
        store.write(ByteArrayInputStream("bravo".toByteArray()), "b.ulg")
        store.write(ByteArrayInputStream("charlie".toByteArray()), "c.ulg")
        store.stage(listOf("a.ulg", "b.ulg", "c.ulg"))

        store.stage(listOf("c.ulg", "a.ulg"))

        val inbox = File(dir, "inbox")
        assertThat(File(inbox, "current.ulg").readText()).isEqualTo("charlie")
        assertThat(File(inbox, "swarm_1.ulg").readText()).isEqualTo("alpha")
        assertThat(File(inbox, "swarm_2.ulg").exists()).isFalse()
    }

    @Test
    fun `multi-staging with any missing file fails without touching the inbox`(@TempDir dir: File) {
        val store = store(dir, now = 4_242L)
        store.write(ByteArrayInputStream("alpha".toByteArray()), "a.ulg")
        store.stage("a.ulg")

        val later = store(dir, now = 9_999L)
        val result = later.stage(listOf("a.ulg", "ghost.ulg"))

        assertThat(result).isEqualTo(Result.Error(DataError.Local.NOT_FOUND))
        val inbox = File(dir, "inbox")
        assertThat(File(inbox, "current.ulg").readText()).isEqualTo("alpha")
        assertThat(File(inbox, ".ready").readText()).isEqualTo("4242")
        assertThat(File(inbox, "swarm_1.ulg").exists()).isFalse()
    }

    @Test
    fun `a copy failure mid-batch leaves the previous inbox fully intact`(@TempDir dir: File) {
        val store = store(dir, now = 4_242L)
        store.write(ByteArrayInputStream("old-current".toByteArray()), "old.ulg")
        store.write(ByteArrayInputStream("old-swarm".toByteArray()), "old2.ulg")
        store.stage(listOf("old.ulg", "old2.ulg"))
        // A directory passes the exists() pre-check but fails to open as a stream,
        // simulating an I/O failure after the first file of the batch copied.
        store.write(ByteArrayInputStream("good".toByteArray()), "good.ulg")
        File(File(dir, "library"), "broken.ulg").mkdirs()

        val later = store(dir, now = 9_999L)
        val result = later.stage(listOf("good.ulg", "broken.ulg"))

        assertThat(result is Result.Error).isTrue()
        val inbox = File(dir, "inbox")
        assertThat(File(inbox, "current.ulg").readText()).isEqualTo("old-current")
        assertThat(File(inbox, "swarm_1.ulg").readText()).isEqualTo("old-swarm")
        assertThat(File(inbox, ".ready").readText()).isEqualTo("4242 2")
        assertThat(inbox.listFiles()!!.none { it.name.endsWith(".tmp") }).isTrue()
    }

    @Test
    fun `staging an empty list fails with NOT_FOUND`(@TempDir dir: File) {
        val result = store(dir).stage(emptyList())

        assertThat(result).isEqualTo(Result.Error(DataError.Local.NOT_FOUND))
        assertThat(File(File(dir, "inbox"), ".ready").exists()).isFalse()
    }

    @Test
    fun `delete removes the library file`(@TempDir dir: File) {
        val store = store(dir)
        store.write(ByteArrayInputStream("x".toByteArray()), "a.ulg")
        val file = File(File(dir, "library"), "a.ulg")
        assertThat(file.exists()).isTrue()

        store.delete("a.ulg")

        assertThat(file.exists()).isFalse()
    }

    @Test
    fun `an unclassified failure is reported`(@TempDir dir: File) {
        val reporter = RecordingCrashReporter()
        // A directory where the payload belongs makes renameTo fail with a plain IOException,
        // which classify() cannot attribute — exactly the case worth reporting.
        File(File(dir, "library"), "abc.ulg").mkdirs()

        val result = store(dir, crashReporter = reporter)
            .write(ByteArrayInputStream("x".toByteArray()), "abc.ulg")

        assertThat(result).isEqualTo(Result.Error(DataError.Local.UNKNOWN))
        assertThat(reporter.recorded).hasSize(1)
        assertThat(reporter.recorded.single().second).isEqualTo("replay-library-files")
    }

    @Test
    fun `a missing file is an expected failure and is not reported`(@TempDir dir: File) {
        val reporter = RecordingCrashReporter()

        val result = store(dir, crashReporter = reporter).stage("nope.ulg")

        assertThat(result).isEqualTo(Result.Error(DataError.Local.NOT_FOUND))
        assertThat(reporter.recorded).isEmpty()
    }
}
