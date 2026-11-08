package org.firstinspires.ftc.teamcode.Utility.Recording

import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.io.BufferedOutputStream
import java.io.File
import java.io.FileOutputStream
import java.io.OutputStream
import java.io.OutputStreamWriter
import java.io.Writer
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

/// everything an OpMode does, into one folder (/sdcard/FIRST/data/RobotLog/<time>-<OpMode>/), so a past match can be gone over
/// without the robot: frames.bin = one row of float64 (little-endian) per sample, the columns in channels.tsv (group, name, unit,
/// what it is); events.tsv = rare things with a time (swings, relocalizations, toggles, status changes); meta.tsv = context (every
/// constant, the hardware, the OpMode, the start). Channels are fixed at the first sample. The loop thread only copies numbers into
/// a reused buffer; one low-priority daemon thread writes. Read it with tools/robot_log.py
class Recorder(val folder: File) {

    private class Channel(val group: String, val name: String, val unit: String, val help: String, val source: DoubleSupplier) {
        var dead = false
    }

    private class Watch(val source: String, val name: String, val value: () -> String?) {
        var last: String? = null
    }

    private val channels = ArrayList<Channel>()
    private val prepares = ArrayList<Runnable>()
    private val watches = ArrayList<Watch>()
    private var values = DoubleArray(0)
    private val lock = Object()
    private val writeLock = Object()
    private var active: ByteBuffer? = null
    private var spare: ByteBuffer? = null
    private var events = StringBuilder()
    private var meta = StringBuilder()
    private var frames: OutputStream? = null
    private var eventsOut: Writer? = null
    private var metaOut: Writer? = null
    private var writer: Thread? = null
    private val startNs = System.nanoTime()

    @Volatile
    private var running = false

    var rows = 0L
        private set
    var dropped = 0L
        private set
    var started = false
        private set

    /// seconds since this recorder was made: the "t" every row and event carries
    fun time(): Double = (System.nanoTime() - startNs) * 1e-9

    /// one column. Read it from cached state only (no hardware calls): it runs every loop
    fun channel(group: String, name: String, unit: String, help: String, source: DoubleSupplier): Recorder {

        if (started) {
            RobotLog.ww(TAG, "channel $group.$name added after the first sample, ignored")
            return this
        }
        channels.add(Channel(group, name, unit, help, source))
        return this
    }

    fun flag(group: String, name: String, help: String, source: BooleanSupplier): Recorder = channel(group, name, "0/1", help, DoubleSupplier { if (source.asBoolean) 1.0 else 0.0 })

    /// runs once per sample before the channels (cache something several channels read, like the pose)
    fun prepare(step: Runnable): Recorder {

        if (!started) prepares.add(step)
        return this
    }

    /// a text state (a stage, a status): an event whenever it changes. The supplier should hand back the same String while
    /// nothing changes (a field, an enum name), or it allocates every loop
    fun watch(source: String, name: String, value: () -> String?): Recorder {

        if (!started) watches.add(Watch(source, name, value))
        return this
    }

    fun meta(key: String, value: Any?) {
        synchronized(lock) { meta.append(clean(key)).append('\t').append(clean(value?.toString())).append('\n') }
    }

    @JvmOverloads
    fun event(source: String, what: String, value: Double = Double.NaN) {

        val t = time()
        synchronized(lock) { events.append(String.format(Locale.US, "%.4f", t)).append('\t').append(clean(source)).append('\t').append(clean(what)).append('\t').append(value).append('\n') }
    }

    /// once per loop. The first call fixes the channels and starts writing
    fun sample() {

        if (!started) start()
        for (k in prepares.indices) {
            try {
                prepares[k].run()
            } catch (e: Exception) {
            }
        }
        val n = channels.size
        values[0] = time()
        for (k in 1 until n) {
            val c = channels[k]
            if (c.dead) {
                values[k] = Double.NaN
                continue
            }
            values[k] = try {
                c.source.asDouble
            } catch (e: Exception) {
                c.dead = true
                RobotLog.ww(TAG, "channel ${c.group}.${c.name} threw ${e.javaClass.simpleName}, NaN from now on")
                Double.NaN
            }
        }
        for (k in watches.indices) {
            val w = watches[k]
            val now = try {
                w.value()
            } catch (e: Exception) {
                null
            }
            if (now != w.last) {
                w.last = now
                event(w.source, w.name + "=" + now)
            }
        }
        synchronized(lock) {
            val b = active ?: return
            if (b.remaining() < 8 * n) {
                dropped++
                return
            }
            for (k in 0 until n) b.putDouble(values[k])
            rows++
        }
    }

    fun status(): String = if (!started) "waiting for the first loop" else folder.name + ", " + rows + " rows, " + channels.size + " channels" + if (dropped > 0) ", $dropped dropped" else ""

    fun close() {

        if (!started) return
        running = false
        writer?.interrupt()
        try {
            writer?.join(1500)
        } catch (e: InterruptedException) {
        }
        meta("rows", rows)
        meta("dropped_rows", dropped)
        meta("seconds", String.format(Locale.US, "%.3f", time()))
        flush()
        synchronized(writeLock) {
            try {
                frames?.close()
                eventsOut?.close()
                metaOut?.close()
            } catch (e: Exception) {
                RobotLog.ee(TAG, "closing $folder: $e")
            }
            frames = null
            eventsOut = null
            metaOut = null
        }
        if (current === this) current = null
    }

    private fun start() {

        started = true
        channels.add(0, Channel("time", "t", "s", "since the recorder started", DoubleSupplier { 0.0 }))
        values = DoubleArray(channels.size)
        if (!folder.exists() && !folder.mkdirs()) RobotLog.ee(TAG, "can't make $folder")
        try {
            val sb = StringBuilder("index\tgroup\tname\tunit\twhat\n")
            for (k in channels.indices) {
                val c = channels[k]
                sb.append(k).append('\t').append(clean(c.group)).append('\t').append(clean(c.name)).append('\t').append(clean(c.unit)).append('\t').append(clean(c.help)).append('\n')
            }
            OutputStreamWriter(FileOutputStream(File(folder, "channels.tsv")), Charsets.UTF_8).use { it.write(sb.toString()) }
            frames = BufferedOutputStream(FileOutputStream(File(folder, "frames.bin")), 1 shl 16)
            eventsOut = OutputStreamWriter(FileOutputStream(File(folder, "events.tsv")), Charsets.UTF_8)
            eventsOut?.write("t\tsource\twhat\tvalue\n")
            metaOut = OutputStreamWriter(FileOutputStream(File(folder, "meta.tsv")), Charsets.UTF_8)
            metaOut?.write("key\tvalue\n")
        } catch (e: Exception) {
            RobotLog.ee(TAG, "can't open $folder: $e")
        }
        //~1.5 s of rows per buffer at 40 Hz is plenty for a 250 ms writer; never less than 1 MB
        val bytes = Math.max(1 shl 20, 8 * channels.size * 64)
        active = ByteBuffer.allocate(bytes).order(ByteOrder.LITTLE_ENDIAN)
        spare = ByteBuffer.allocate(bytes).order(ByteOrder.LITTLE_ENDIAN)
        running = true
        current = this
        writer = Thread({
            while (running) {
                try {
                    Thread.sleep(250)
                } catch (e: InterruptedException) {
                    break
                }
                flush()
            }
        }, "Recorder").apply {
            priority = Thread.MIN_PRIORITY
            isDaemon = true
            start()
        }
    }

    private fun flush() {

        synchronized(writeLock) {
            val full: ByteBuffer
            val ev: String
            val me: String
            synchronized(lock) {
                full = active ?: return
                active = spare
                spare = full
                ev = events.toString()
                me = meta.toString()
                events.setLength(0)
                meta.setLength(0)
            }
            try {
                full.flip()
                frames?.write(full.array(), 0, full.limit())
                frames?.flush()
                if (ev.isNotEmpty()) eventsOut?.write(ev)
                eventsOut?.flush()
                if (me.isNotEmpty()) metaOut?.write(me)
                metaOut?.flush()
            } catch (e: Exception) {
                RobotLog.ee(TAG, "writing $folder: $e")
            } finally {
                full.clear()
            }
        }
    }

    companion object {

        const val TAG = "Recorder"

        /// the recorder of the OpMode running now (null between OpModes): code anywhere can drop an event in
        @JvmStatic
        @Volatile
        var current: Recorder? = null
            private set

        @JvmStatic
        fun note(source: String, what: String, value: Double) {
            current?.event(source, what, value)
        }

        @JvmStatic
        fun note(source: String, what: String) {
            current?.event(source, what)
        }

        @JvmStatic
        fun folderFor(opMode: String): File {

            val stamp = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date())
            return File(File(AppUtil.ROBOT_DATA_DIR, "RobotLog"), stamp + "-" + opMode.replace(Regex("[^A-Za-z0-9]"), ""))
        }

        private fun clean(s: String?): String = (s ?: "null").replace('\t', ' ').replace('\n', ' ').replace('\r', ' ')
    }
}
