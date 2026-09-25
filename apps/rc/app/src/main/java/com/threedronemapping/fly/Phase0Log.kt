package com.threedronemapping.fly

import android.content.Context
import android.os.Handler
import android.os.Looper
import android.util.Log
import java.io.File
import java.io.FileWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.CopyOnWriteArrayList
import java.util.concurrent.Executors

/**
 * Timestamped log of every SDK callback / result / error.
 * Mirrored to logcat (tag "3DMFly"), to the on-screen panel, and appended to
 * getExternalFilesDir("logs")/phase0-<yyyyMMdd>.log so it can be pulled with adb.
 */
object Phase0Log {
    private const val TAG = "3DMFly"
    private const val MAX_LINES = 600

    private val ts = SimpleDateFormat("HH:mm:ss.SSS", Locale.US)
    private val day = SimpleDateFormat("yyyyMMdd", Locale.US)
    private val io = Executors.newSingleThreadExecutor()
    private val main = Handler(Looper.getMainLooper())
    private val lines = ArrayDeque<String>()
    private val listeners = CopyOnWriteArrayList<(String) -> Unit>()

    @Volatile
    var logFile: File? = null
        private set

    fun init(context: Context) {
        try {
            val dir = context.getExternalFilesDir("logs") ?: File(context.filesDir, "logs")
            dir.mkdirs()
            logFile = File(dir, "phase0-${day.format(Date())}.log")
        } catch (t: Throwable) {
            Log.e(TAG, "log file init failed", t)
        }
        i("LOG", "===== 3DM Fly ${BuildConfig.VERSION_NAME} started; log file: ${logFile?.absolutePath} =====")
    }

    fun i(section: String, msg: String) = write("I", section, msg, null)
    fun w(section: String, msg: String) = write("W", section, msg, null)
    fun e(section: String, msg: String, t: Throwable? = null) = write("E", section, msg, t)

    fun snapshot(): List<String> = synchronized(lines) { lines.toList() }

    fun addListener(l: (String) -> Unit) { listeners.add(l) }
    fun removeListener(l: (String) -> Unit) { listeners.remove(l) }

    private fun write(level: String, section: String, msg: String, t: Throwable?) {
        val line = buildString {
            append(ts.format(Date())).append(' ').append(level).append(" [").append(section).append("] ").append(msg)
            if (t != null) append(" | ").append(t.javaClass.simpleName).append(": ").append(t.message)
        }
        when (level) {
            "E" -> Log.e(TAG, line, t)
            "W" -> Log.w(TAG, line)
            else -> Log.i(TAG, line)
        }
        synchronized(lines) {
            lines.addLast(line)
            while (lines.size > MAX_LINES) lines.removeFirst()
        }
        val stack = t?.let { Log.getStackTraceString(it) }
        io.execute {
            try {
                logFile?.let { f ->
                    FileWriter(f, true).use { w ->
                        w.append(line).append('\n')
                        if (stack != null) w.append(stack).append('\n')
                    }
                }
            } catch (_: Throwable) {
                // never crash because of logging
            }
        }
        main.post { listeners.forEach { l -> try { l(line) } catch (_: Throwable) {} } }
    }
}
