package com.threedronemapping.fly

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.os.Environment
import java.io.File

/**
 * Pilot 2 storage probe — STRICTLY READ-ONLY.
 * Only calls exists()/isDirectory()/canRead()/canWrite()/list(); never creates, writes or deletes
 * anything outside our own app directories. File.canWrite() is only a permission check.
 */
object StorageProbe {
    private const val S = "PROBE"

    fun run(ctx: Context): String {
        val out = StringBuilder()
        fun line(s: String) { out.append(s).append('\n'); Phase0Log.i(S, s) }

        line("Android ${Build.VERSION.RELEASE} (API ${Build.VERSION.SDK_INT}), device=${Build.MANUFACTURER} ${Build.MODEL}")
        val readGranted = ctx.checkSelfPermission(Manifest.permission.READ_EXTERNAL_STORAGE) == PackageManager.PERMISSION_GRANTED
        line("READ_EXTERNAL_STORAGE granted=$readGranted")
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            line("isExternalStorageManager (All-files access)=${safe(S, "isExternalStorageManager") { Environment.isExternalStorageManager() }}")
        }
        val root = safe(S, "getExternalStorageDirectory") { Environment.getExternalStorageDirectory() } ?: File("/sdcard")
        line("external storage root=${root.absolutePath}")

        // 1. Is /sdcard/Android/data listable?
        val data = File(root, "Android/data")
        val listing = safe(S, "list Android/data") { data.list() }
        line("Android/data: exists=${data.exists()} canRead=${data.canRead()} list()=${listing?.size?.toString() ?: "null (not listable)"}")
        val visibleDji = listing?.filter { it.lowercase().contains("dji") }?.sorted().orEmpty()
        if (listing != null) line("Android/data entries containing 'dji': ${if (visibleDji.isEmpty()) "(none)" else visibleDji.joinToString()}")

        // 2. Installed DJI packages (needs QUERY_ALL_PACKAGES on API 30+, declared in the manifest).
        val pkgs = safe(S, "getInstalledPackages") {
            @Suppress("DEPRECATION")
            ctx.packageManager.getInstalledPackages(0).map { it.packageName }
        }.orEmpty().filter { it.lowercase().contains("dji") && it != ctx.packageName }.sorted()
        line("installed packages containing 'dji': ${if (pkgs.isEmpty()) "(none)" else pkgs.joinToString()}")

        // 3. Probe each candidate dir (visible listing + installed packages + known guesses).
        val candidates = linkedSetOf<String>()
        candidates.addAll(visibleDji)
        candidates.addAll(pkgs)
        candidates.addAll(listOf("dji.go.v5", "dji.pilot", "dji.pilot2", "dji.pilot.pad"))
        for (name in candidates) {
            val d = File(data, name)
            probeDir(d, ::line)
            for (sub in listOf("files", "files/waypoint", "files/wayline")) {
                val s = File(d, sub)
                if (safe(S, "exists $sub") { s.exists() } == true) probeDir(s, ::line)
            }
        }

        // 4. Shared-storage DJI folders (where Pilot 2 exports may land).
        for (p in listOf("DJI", "Android/media")) {
            val d = File(root, p)
            probeDir(d, ::line)
            val kids = safe(S, "list $p") { d.list() }
            if (kids != null && p != "Android/media") line("  $p entries: ${kids.sorted().take(40).joinToString()}")
            if (kids != null && p == "Android/media") line("  Android/media entries containing 'dji': ${kids.filter { it.lowercase().contains("dji") }.joinToString().ifEmpty { "(none)" }}")
        }

        // 5. Reference: our own dir (should be writable).
        ctx.getExternalFilesDir(null)?.let { probeDir(it, ::line) }
        line("probe done (read-only; nothing was written)")
        return out.toString()
    }

    private fun probeDir(d: File, line: (String) -> Unit) {
        val exists = safe(S, "exists") { d.exists() }
        val isDir = safe(S, "isDirectory") { d.isDirectory }
        val canRead = safe(S, "canRead") { d.canRead() }
        val canWrite = safe(S, "canWrite") { d.canWrite() }
        val n = safe(S, "list") { d.list() }?.size
        line("  ${d.absolutePath}: exists=$exists dir=$isDir canRead=$canRead canWrite=$canWrite list()=${n?.toString() ?: "null"}")
    }
}
