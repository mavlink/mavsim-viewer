package com.px4.hawkeye.feature.replay.data

import com.px4.hawkeye.core.domain.CrashReporter
import com.px4.hawkeye.core.domain.DataError
import com.px4.hawkeye.core.domain.EmptyResult
import com.px4.hawkeye.core.domain.Result
import java.io.File
import java.io.FileNotFoundException
import java.io.IOException
import java.io.InputStream

/**
 * The library's on-disk side: imported payloads live under `filesDir/library/`, and
 * staging copies them into `filesDir/inbox/` (`current.ulg`, plus `swarm_<i>.ulg` for a
 * multi-drone session) and bumps the `.ready` sentinel the native poll loop reads (a
 * millis token, so two stages in the same wall-clock second are still distinguishable).
 *
 * Pure file/JVM logic with no Android dependencies, so it is unit-testable against a
 * temp directory. [clock] is injected for deterministic sentinel tokens in tests.
 */
class LibraryFileStore(
    filesDir: File,
    private val crashReporter: CrashReporter,
    private val clock: () -> Long = System::currentTimeMillis,
) {
    private val libraryDir = File(filesDir, "library")
    private val inboxDir = File(filesDir, "inbox")

    /** Streams [source] into the library under [fileName] (atomic via .tmp + rename). */
    fun write(source: InputStream, fileName: String): Result<Long, DataError.Local> = runCatching {
        libraryDir.mkdirs()
        val target = File(libraryDir, fileName)
        val tmp = File(libraryDir, "$fileName.tmp")
        val bytes = source.use { input -> tmp.outputStream().use { output -> input.copyTo(output) } }
        if (!tmp.renameTo(target)) {
            tmp.delete()
            throw IOException("renameTo $target failed")
        }
        bytes
    }.fold(
        onSuccess = { Result.Success(it) },
        onFailure = { Result.Error(classify(it)) },
    )

    /** Copies the library payload [fileName] into the inbox and bumps the sentinel. */
    fun stage(fileName: String): EmptyResult<DataError.Local> = stage(listOf(fileName))

    /**
     * Stages [fileNames] (staged order = drone order) into the inbox and bumps the sentinel.
     * Index 0 keeps the legacy `current.ulg` name; indices 1..n-1 become `swarm_<i>.ulg`. A
     * single file writes the legacy bare-millis token; several write `"<millis> <count>"` for
     * the native swarm loader (an older binary's strtoll stops at the space and still reads
     * the millis).
     *
     * A failed batch never clobbers the previous session: every payload is copied to a
     * `.tmp` first and the live names are only renamed over (and stale extras deleted)
     * after the whole batch has copied, so any I/O failure leaves the inbox exactly as the
     * still-unchanged `.ready` token describes it.
     */
    fun stage(fileNames: List<String>): EmptyResult<DataError.Local> = runCatching {
        if (fileNames.isEmpty()) throw FileNotFoundException("empty stage batch")
        val sources = fileNames.map { name ->
            File(libraryDir, name).also {
                if (!it.exists()) throw FileNotFoundException("missing library file $name")
            }
        }
        inboxDir.mkdirs()
        val targets = List(fileNames.size) { index ->
            File(inboxDir, if (index == 0) "current.ulg" else "swarm_$index.ulg")
        }
        val tmps = targets.map { File(inboxDir, "${it.name}.tmp") }
        try {
            sources.forEachIndexed { index, source ->
                source.inputStream().use { input ->
                    tmps[index].outputStream().use { output -> input.copyTo(output) }
                }
            }
        } catch (e: Throwable) {
            tmps.forEach { it.delete() }
            throw e
        }
        // The whole batch is on disk; same-filesystem renames don't fail for space.
        tmps.forEachIndexed { index, tmp ->
            if (!tmp.renameTo(targets[index])) {
                tmps.forEach { it.delete() }
                throw IOException("renameTo ${targets[index]} failed")
            }
        }
        // Drop swarm payloads beyond the new batch only after it is fully in place.
        inboxDir.listFiles { file -> SWARM_FILE_PATTERN.matches(file.name) }
            ?.filterNot { it in targets }
            ?.forEach { it.delete() }
        val token = clock().toString()
        File(inboxDir, ".ready")
            .writeText(if (fileNames.size == 1) token else "$token ${fileNames.size}")
    }.fold(
        onSuccess = { Result.Success(Unit) },
        onFailure = { Result.Error(classify(it)) },
    )

    /** Removes the library payload [fileName]; no-op if it is already gone. */
    fun delete(fileName: String) {
        File(libraryDir, fileName).delete()
    }

    private fun classify(e: Throwable): DataError.Local = when {
        e is FileNotFoundException -> DataError.Local.NOT_FOUND
        e is IOException && e.message?.contains("ENOSPC", ignoreCase = true) == true ->
            DataError.Local.DISK_FULL
        e is IOException && e.message?.contains("space", ignoreCase = true) == true ->
            DataError.Local.DISK_FULL
        // Only the unclassified branch is reported: a missing file and a full disk are
        // expected outcomes the user already sees.
        else -> {
            crashReporter.recordException(e, ORIGIN)
            DataError.Local.UNKNOWN
        }
    }

    private companion object {
        const val ORIGIN = "replay-library-files"

        /** Extra swarm payloads (and their tmp files) from a previous multi-drone session. */
        val SWARM_FILE_PATTERN = Regex("""swarm_\d+\.ulg(\.tmp)?""")
    }
}
