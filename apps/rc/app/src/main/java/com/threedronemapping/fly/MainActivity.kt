package com.threedronemapping.fly

import android.Manifest
import android.app.Activity
import android.content.Intent
import android.content.pm.PackageManager
import android.content.res.Configuration
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.provider.Settings
import android.view.ViewGroup
import android.view.WindowManager
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.ProgressBar
import android.widget.ScrollView
import android.widget.TextView
import androidx.core.content.FileProvider
import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.key.DJIKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.LidarKey
import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.key.RemoteControllerKey
import dji.sdk.keyvalue.key.RtkMobileStationKey
import dji.sdk.keyvalue.value.camera.CameraType
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.common.LocationCoordinate2D
import dji.sdk.keyvalue.value.flightcontroller.FlightMode
import dji.sdk.keyvalue.value.lidar.PointCloudRecordCommand
import dji.sdk.keyvalue.value.product.ProductType
import dji.sdk.keyvalue.value.remotecontroller.RemoteControllerType
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.KeyManager
import dji.v5.manager.aircraft.simulator.InitializationSettings
import dji.v5.manager.aircraft.simulator.SimulatorManager
import dji.v5.manager.aircraft.simulator.SimulatorStatusListener
import dji.v5.manager.aircraft.waypoint3.WaylineExecutingInfoListener
import dji.v5.manager.aircraft.waypoint3.WaypointActionListener
import dji.v5.manager.aircraft.waypoint3.WaypointMissionExecuteStateListener
import dji.v5.manager.aircraft.waypoint3.WaypointMissionManager
import dji.v5.manager.aircraft.waypoint3.model.WaylineExecutingInfo
import java.io.File
import java.util.Locale

/**
 * Phase 0 go/no-go test console. One scrollable activity; log panel on the right.
 * Every SDK call is wrapped (safe{} / try-catch) so nothing crashes without an aircraft.
 */
class MainActivity : Activity() {

    private val main = Handler(Looper.getMainLooper())

    // Listener holders so we can cancel per group.
    private val telemetryHolder = Any()
    private val payloadHolder = Any()
    private val lidarHolder = Any()
    private var listenersAttached = false

    // ---- 1 SDK ----
    private lateinit var tvSdkVersion: TextView
    private lateinit var tvInit: TextView
    private lateinit var tvRegistered: TextView
    private lateinit var tvRegError: TextView
    private lateinit var tvNetwork: TextView
    private lateinit var tvProduct: TextView
    private lateinit var tvProductType: TextView
    private lateinit var tvRcType: TextView
    private lateinit var tvFcConn: TextView
    private lateinit var tvDb: TextView

    // ---- 2 Telemetry ----
    private lateinit var tvLatLon: TextView
    private lateinit var tvAlt: TextView
    private lateinit var tvGps: TextView
    private lateinit var tvRtk: TextView
    private lateinit var tvBatt: TextView
    private lateinit var tvMode: TextView
    private lateinit var tvMotors: TextView
    private var gpsSats: Int? = null
    private var gpsLevel: String? = null
    private var rtkConnected: Boolean? = null
    private var rtkEnabled: Boolean? = null
    private var rtkSolution: String? = null
    private var batt1: Int? = null
    private var batt2: Int? = null
    private var flightMode: FlightMode? = null
    private var flightModeString: String? = null
    private var motorsOn: Boolean? = null
    private var isFlying: Boolean? = null
    private var altFc: Double? = null
    private var alt3d: Double? = null

    // ---- 3 Payloads ----
    private val payloadPorts = listOf(
        ComponentIndexType.PORT_1, ComponentIndexType.PORT_2, ComponentIndexType.PORT_3,
        ComponentIndexType.PORT_4, ComponentIndexType.PORT_5, ComponentIndexType.PORT_6,
        ComponentIndexType.PORT_7,
        // legacy indices (M300/M350 style) for comparison only
        ComponentIndexType.LEFT_OR_MAIN, ComponentIndexType.RIGHT, ComponentIndexType.UP,
    )
    private val payloadRows = HashMap<ComponentIndexType, TextView>()
    private val camType = HashMap<ComponentIndexType, CameraType?>()
    private val camConn = HashMap<ComponentIndexType, Boolean?>()
    private val lidarConn = HashMap<ComponentIndexType, Boolean?>()

    // ---- 4 LiDAR ----
    private var lidarPort: ComponentIndexType = ComponentIndexType.PORT_1
    private var lidarPortManual = false
    private lateinit var tvLidarPort: TextView
    private lateinit var tvRecStatus: TextView
    private lateinit var tvWorkState: TextView
    private lateinit var tvScanMode: TextView
    private lateinit var tvEcho: TextView
    private lateinit var tvSampleRate: TextView
    private lateinit var tvExclusive: TextView
    private lateinit var tvLidarSupport: TextView
    private lateinit var tvLidarLast: TextView
    private val lidarKeysListened = ArrayList<DJIKey<*>>()

    // ---- 5 Mission ----
    private var missions: List<File> = emptyList()
    private var missionIdx = 0
    private lateinit var tvMissionDir: TextView
    private lateinit var tvMissionFile: TextView
    private lateinit var tvWaylineIds: TextView
    private lateinit var tvExecState: TextView
    private lateinit var tvWaylineInfo: TextView
    private lateinit var tvUpload: TextView
    private lateinit var tvAction: TextView
    private lateinit var pbUpload: ProgressBar

    private val missionStateListener = WaypointMissionExecuteStateListener { st ->
        Phase0Log.i("MISSION", "executeState=$st")
        main.post { tvExecState.text = "$st" }
    }
    private val waylineInfoListener = object : WaylineExecutingInfoListener {
        private var lastLogged = ""
        override fun onWaylineExecutingInfoUpdate(info: WaylineExecutingInfo) {
            val s = "wayline=${info.waylineID} wp=${info.currentWaypointIndex} file=${info.missionFileName}"
            if (s != lastLogged) { Phase0Log.i("MISSION", "executingInfo $s"); lastLogged = s }
            main.post { tvWaylineInfo.text = s }
        }

        override fun onWaylineExecutingInterruptReasonUpdate(error: IDJIError?) {
            if (error != null) {
                Phase0Log.w("MISSION", "interrupt reason ${error.fmt()}")
                main.post { tvWaylineInfo.text = "INTERRUPTED: ${error.errorCode()}" }
            }
        }
    }
    private val actionListener = object : WaypointActionListener {
        override fun onExecutionStart(actionId: Int) = act("actionStart id=$actionId")
        override fun onExecutionStart(actionGroup: Int, actionId: Int) = act("actionStart group=$actionGroup id=$actionId")
        override fun onExecutionFinish(actionId: Int, error: IDJIError?) =
            act("actionFinish id=$actionId ${if (error == null) "OK" else error.fmt()}")
        override fun onExecutionFinish(actionGroup: Int, actionId: Int, error: IDJIError?) =
            act("actionFinish group=$actionGroup id=$actionId ${if (error == null) "OK" else error.fmt()}")

        private fun act(s: String) {
            Phase0Log.i("MISSION", s)
            main.post { tvAction.text = s }
        }
    }

    // ---- 6 Simulator ----
    private lateinit var etSimLat: EditText
    private lateinit var etSimLon: EditText
    private lateinit var etSimSats: EditText
    private lateinit var tvSimEnabled: TextView
    private lateinit var tvSimState: TextView
    private var simEnabledLast: Boolean? = null
    private var simEnabledSince = 0L
    private var simLastUpdate = 0L
    private var simStreamLive = false
    private var simLastUi = 0L
    private var simMotors: Boolean? = null
    private var simFlying: Boolean? = null
    private val simListener = SimulatorStatusListener { st ->
        val now = SystemClock.elapsedRealtime()
        if (!simStreamLive) { simStreamLive = true; Phase0Log.i("SIM", "state updates started") }
        simLastUpdate = now
        try {
            val m = st.areMotorsOn(); val f = st.isFlying
            if (m != simMotors || f != simFlying) {
                Phase0Log.i("SIM", "state change motorsOn=$m flying=$f")
                simMotors = m; simFlying = f
            }
            if (now - simLastUi > 250) {
                simLastUi = now
                val loc = st.location
                val s = String.format(
                    Locale.US, "motors=%s flying=%s lat=%.7f lon=%.7f z=%.1f r/p/y=%.1f/%.1f/%.1f",
                    m, f, loc?.latitude ?: Double.NaN, loc?.longitude ?: Double.NaN,
                    st.positionZ, st.roll, st.pitch, st.yaw
                )
                main.post { tvSimState.text = s }
            }
        } catch (t: Throwable) {
            Phase0Log.e("SIM", "state listener", t)
        }
    }
    private val simPoll = object : Runnable {
        override fun run() {
            pollSimulator()
            main.postDelayed(this, 200)
        }
    }

    // ---- 7 Probe ----
    private lateinit var tvProbe: TextView

    // ---- 8 Log ----
    private lateinit var tvLog: TextView
    private lateinit var logScroll: ScrollView
    private val logListener: (String) -> Unit = { line -> appendLog(line) }

    private val sdkListener: () -> Unit = { refreshSdk() }

    // =====================================================================================

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        buildUi()
        Phase0Log.snapshot().forEach { appendLog(it) }
        Phase0Log.addListener(logListener)
        Sdk.addListener(sdkListener)
        refreshSdk()
        requestPermissionsIfNeeded()
        refreshMissionList()
        main.post(simPoll)
    }

    override fun onResume() {
        super.onResume()
        refreshSdk()
    }

    override fun onDestroy() {
        main.removeCallbacks(simPoll)
        Phase0Log.removeListener(logListener)
        Sdk.removeListener(sdkListener)
        detachListeners()
        super.onDestroy()
    }

    // =====================================================================================
    // UI

    private fun buildUi() {
        val landscape = resources.configuration.orientation == Configuration.ORIENTATION_LANDSCAPE
        val root = LinearLayout(this).apply {
            orientation = if (landscape) LinearLayout.HORIZONTAL else LinearLayout.VERTICAL
            setBackgroundColor(Ui.BG)
            val p = Ui.dp(this@MainActivity, 10)
            setPadding(p, p, p, p)
        }
        val leftScroll = ScrollView(this)
        val left = Ui.vertical(this)
        leftScroll.addView(left)
        val right = Ui.vertical(this)
        if (landscape) {
            root.addView(leftScroll, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.MATCH_PARENT, 0.58f))
            root.addView(right, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.MATCH_PARENT, 0.42f).apply {
                marginStart = Ui.dp(this@MainActivity, 10)
            })
        } else {
            root.addView(leftScroll, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, 0, 0.6f))
            root.addView(right, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, 0, 0.4f))
        }
        setContentView(root)

        Ui.text(left, "3DM Fly — Phase 0 go/no-go (M400 + L3, RC Plus 2). Props OFF. Simulator only.", 17f, Ui.ACCENT, mono = false)

        // 1 SDK
        val s1 = Ui.section(left, "1", "SDK / registration / connection",
            "First run needs internet for app-key registration (cached afterwards; again after reinstall).")
        tvSdkVersion = Ui.kv(s1, "MSDK version")
        tvInit = Ui.kv(s1, "Init")
        tvRegistered = Ui.kv(s1, "Registered")
        tvRegError = Ui.kv(s1, "Register error", "")
        tvNetwork = Ui.kv(s1, "Network")
        tvProduct = Ui.kv(s1, "Product connected")
        tvProductType = Ui.kv(s1, "Product type")
        tvRcType = Ui.kv(s1, "RC type")
        tvFcConn = Ui.kv(s1, "Flight controller")
        tvDb = Ui.kv(s1, "DB download")
        Ui.buttons(s1, "Register again" to { Sdk.register() }, "Re-attach listeners" to { detachListeners(); attachListeners() })

        // 2 Telemetry
        val s2 = Ui.section(left, "2", "Telemetry (KeyManager listeners)")
        tvLatLon = Ui.kv(s2, "Lat / Lon")
        tvAlt = Ui.kv(s2, "Altitude")
        tvGps = Ui.kv(s2, "GPS sats / signal")
        tvRtk = Ui.kv(s2, "RTK")
        tvBatt = Ui.kv(s2, "Battery %")
        tvMode = Ui.kv(s2, "Flight mode")
        tvMotors = Ui.kv(s2, "Motors / flying")

        // 3 Payloads
        val s3 = Ui.section(left, "3", "Payloads (CameraKey.KeyCameraType per port)",
            "M400 payloads are ComponentIndexType.PORT_1…PORT_7. LEFT_OR_MAIN/RIGHT/UP shown for comparison.")
        for (p in payloadPorts) payloadRows[p] = Ui.kv(s3, p.name)
        Ui.buttons(s3, "Rescan payloads" to { rescanPayloads() })

        // 4 LiDAR
        val s4 = Ui.section(left, "4", "LiDAR point-cloud recording (LidarKey)",
            "KeyPointCloudRecord action with PointCloudRecordCommand, addressed by component index (port).")
        tvLidarPort = Ui.kv(s4, "LiDAR port")
        Ui.buttons(s4, "◂ Port" to { cycleLidarPort(-1) }, "Port ▸" to { cycleLidarPort(+1) }, "Use detected L3" to { lidarPortManual = false; autoPickLidarPort(force = true) })
        tvRecStatus = Ui.kv(s4, "Record status")
        tvWorkState = Ui.kv(s4, "Work state")
        tvScanMode = Ui.kv(s4, "Scan mode")
        tvEcho = Ui.kv(s4, "Echo mode")
        tvSampleRate = Ui.kv(s4, "Sample rate")
        tvExclusive = Ui.kv(s4, "Exclusive status")
        tvLidarSupport = Ui.kv(s4, "Key supported")
        tvLidarLast = Ui.kv(s4, "Last result")
        Ui.buttons(s4,
            "Start rec" to { lidarCmd(PointCloudRecordCommand.START) },
            "Pause" to { lidarCmd(PointCloudRecordCommand.PAUSE) },
            "Resume" to { lidarCmd(PointCloudRecordCommand.RESUME) },
            "Stop rec" to { lidarCmd(PointCloudRecordCommand.STOP) },
        )

        // 5 Mission
        val s5 = Ui.section(left, "5", "Waypoint mission (WaypointMissionManager)",
            "Put the KMZ exported from the RC into files/missions/ with adb (see README). No sample KMZ is bundled.")
        tvMissionDir = Ui.kv(s5, "Missions dir")
        tvMissionFile = Ui.kv(s5, "Selected KMZ")
        tvWaylineIds = Ui.kv(s5, "Wayline IDs")
        Ui.buttons(s5, "Refresh list" to { refreshMissionList() }, "◂ Prev" to { selectMission(-1) }, "Next ▸" to { selectMission(+1) })
        tvUpload = Ui.kv(s5, "Upload")
        pbUpload = ProgressBar(this, null, android.R.attr.progressBarStyleHorizontal).apply { max = 100; progress = 0 }
        s5.addView(pbUpload, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, Ui.dp(this, 18)))
        Ui.buttons(s5, "Push to aircraft" to { pushMission() })
        tvExecState = Ui.kv(s5, "Execute state")
        tvWaylineInfo = Ui.kv(s5, "Executing info")
        tvAction = Ui.kv(s5, "Last action")
        Ui.buttons(s5,
            "Start" to { startMission() },
            "Pause" to { missionCall("pauseMission") { cb -> WaypointMissionManager.getInstance().pauseMission(cb) } },
            "Resume" to { missionCall("resumeMission") { cb -> WaypointMissionManager.getInstance().resumeMission(cb) } },
            "Stop" to { stopMission() },
        )

        // 6 Simulator
        val s6 = Ui.section(left, "6", "Simulator (SimulatorManager)",
            "Known issue (DJI GitHub #759): on M400 + RC Plus 2 the simulator may switch itself off after ~1 s " +
                "unless the aircraft is rebooted. Enabled state is polled every 200 ms and every change is logged with a timestamp.")
        etSimLat = Ui.input(s6, "Latitude", "-33.2270")
        etSimLon = Ui.input(s6, "Longitude", "22.0310")
        etSimSats = Ui.input(s6, "Satellites", "12")
        tvSimEnabled = Ui.kv(s6, "Simulator enabled")
        tvSimState = Ui.kv(s6, "State")
        Ui.buttons(s6, "Enable simulator" to { enableSimulator() }, "Disable simulator" to { disableSimulator() })

        // 7 Probe
        val s7 = Ui.section(left, "7", "Pilot 2 storage probe (read-only)",
            "Lists/permission-checks only. Never writes or deletes outside this app's own folders.")
        Ui.buttons(s7, "Run probe" to { runProbe() }, "All-files access…" to { openAllFilesSettings() })
        tvProbe = Ui.text(s7, "(not run)", 13f)

        // 8 Log (right pane)
        val header = Ui.row(this)
        header.addView(TextView(this).apply {
            text = "8  Log"
            setTextColor(Ui.ACCENT)
            textSize = 20f
        }, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        right.addView(header)
        Ui.buttons(right, "Mark" to { Phase0Log.i("MARK", "---------------- MARK ----------------") },
            "Share log" to { shareLog() }, "Clear view" to { tvLog.text = "" })
        Ui.text(right, "File: ${Phase0Log.logFile?.absolutePath ?: "—"}", 12f, Ui.DIM)
        logScroll = ScrollView(this)
        tvLog = TextView(this).apply {
            setTextColor(Ui.TEXT)
            textSize = 12f
            typeface = Ui.MONO
            setTextIsSelectable(true)
        }
        logScroll.addView(tvLog)
        logScroll.setBackgroundColor(Ui.CARD)
        right.addView(logScroll, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, 0, 1f).apply {
            topMargin = Ui.dp(this@MainActivity, 6)
        })
    }

    private fun appendLog(line: String) {
        if (!::tvLog.isInitialized) return
        tvLog.append(line + "\n")
        // keep the view bounded
        if (tvLog.lineCount > 800) {
            val t = tvLog.text.toString()
            tvLog.text = t.substring(t.length / 3)
        }
        logScroll.post { logScroll.fullScroll(ScrollView.FOCUS_DOWN) }
    }

    // =====================================================================================
    // 1 SDK

    private fun refreshSdk() {
        if (!::tvInit.isInitialized) return
        val st = Sdk.state
        tvSdkVersion.text = safe("SDK", "getSDKVersion") { dji.v5.manager.SDKManager.getInstance().sdkVersion } ?: "—"
        tvInit.text = st.initEvent
        val reg = if (st.registered == null) Sdk.isRegistered().takeIf { it } else st.registered
        tvRegistered.text = when (reg) { true -> "YES"; false -> "NO"; null -> "pending" }
        tvRegistered.setTextColor(Ui.colorFor(reg))
        tvRegError.text = st.registerError
        tvRegError.setTextColor(Ui.BAD)
        tvNetwork.text = st.network
        tvProduct.text = if (st.productConnected) "YES (productId=${st.productId})" else "no (productId=${st.productId})"
        tvProduct.setTextColor(Ui.colorFor(st.productConnected))
        tvDb.text = st.dbProgress.ifEmpty { "—" }
        if (reg == true && !listenersAttached) attachListeners()
        if (st.productConnected && !lastProductConnected && listenersAttached) {
            // Refresh one-shot values after (re)connection.
            rescanPayloads()
        }
        lastProductConnected = st.productConnected && listenersAttached
    }

    private var lastProductConnected = false

    // =====================================================================================
    // Listener wiring

    private fun attachListeners() {
        if (listenersAttached) return
        listenersAttached = true
        Phase0Log.i("SDK", "attaching key listeners")
        attachTelemetry()
        attachPayloads()
        attachLidar()
        safe("MISSION", "add mission listeners") {
            val m = WaypointMissionManager.getInstance()
            m.addWaypointMissionExecuteStateListener(missionStateListener)
            m.addWaylineExecutingInfoListener(waylineInfoListener)
            m.addWaypointActionListener(actionListener)
        }
        safe("SIM", "addSimulatorStateListener") { SimulatorManager.getInstance().addSimulatorStateListener(simListener) }
    }

    private fun detachListeners() {
        if (!listenersAttached) return
        listenersAttached = false
        Sdk.cancelListen(telemetryHolder)
        Sdk.cancelListen(payloadHolder)
        Sdk.cancelListen(lidarHolder)
        lidarKeysListened.clear()
        safe("MISSION", "remove mission listeners") {
            val m = WaypointMissionManager.getInstance()
            m.removeWaypointMissionExecuteStateListener(missionStateListener)
            m.removeWaylineExecutingInfoListener(waylineInfoListener)
            m.removeWaypointActionListener(actionListener)
        }
        safe("SIM", "removeSimulatorStateListener") { SimulatorManager.getInstance().removeSimulatorStateListener(simListener) }
    }

    // =====================================================================================
    // 2 Telemetry

    private fun attachTelemetry() {
        val h = telemetryHolder
        val T = "TELEM"
        Sdk.listen(T, "ProductKey.KeyConnection", safe(T, "key") { KeyTools.createKey(ProductKey.KeyConnection) }, h) { v ->
            Phase0Log.i(T, "product KeyConnection=$v")
        }
        Sdk.listen(T, "ProductKey.KeyProductType", safe(T, "key") { KeyTools.createKey(ProductKey.KeyProductType) }, h) { v ->
            if (v != null) Phase0Log.i(T, "productType=$v")
            tvProductType.text = v?.name ?: "—"
            tvProductType.setTextColor(if (v == ProductType.DJI_MATRICE_400) Ui.GOOD else Ui.TEXT)
        }
        Sdk.listen(T, "RemoteControllerKey.KeyRemoteControllerType", safe(T, "key") { KeyTools.createKey(RemoteControllerKey.KeyRemoteControllerType) }, h) { v ->
            if (v != null) Phase0Log.i(T, "rcType=$v")
            tvRcType.text = v?.name ?: "—"
            tvRcType.setTextColor(if (v == RemoteControllerType.DJI_RC_PLUS_2) Ui.GOOD else Ui.TEXT)
        }
        Sdk.listen(T, "FlightControllerKey.KeyConnection", safe(T, "key") { KeyTools.createKey(FlightControllerKey.KeyConnection) }, h) { v ->
            Phase0Log.i(T, "FC KeyConnection=$v")
            tvFcConn.text = when (v) { true -> "connected"; false -> "not connected"; null -> "—" }
            tvFcConn.setTextColor(Ui.colorFor(v))
        }
        Sdk.listen(T, "KeyAircraftLocation3D", safe(T, "key") { KeyTools.createKey(FlightControllerKey.KeyAircraftLocation3D) }, h) { v ->
            if (v == null || v.latitude == null) { tvLatLon.text = "—"; return@listen }
            tvLatLon.text = String.format(Locale.US, "%.7f, %.7f", v.latitude, v.longitude)
            alt3d = v.altitude
            renderAlt()
        }
        Sdk.listen(T, "KeyAltitude", safe(T, "key") { KeyTools.createKey(FlightControllerKey.KeyAltitude) }, h) { v ->
            altFc = v; renderAlt()
        }
        Sdk.listen(T, "KeyGPSSatelliteCount", safe(T, "key") { KeyTools.createKey(FlightControllerKey.KeyGPSSatelliteCount) }, h) { v ->
            gpsSats = v; renderGps()
        }
        Sdk.listen(T, "KeyGPSSignalLevel", safe(T, "key") { KeyTools.createKey(FlightControllerKey.KeyGPSSignalLevel) }, h) { v ->
            gpsLevel = v?.name; renderGps()
        }
        Sdk.listen(T, "RTK KeyIsRTKConnected", safe(T, "key") { KeyTools.createKey(RtkMobileStationKey.KeyIsRTKConnected) }, h) { v ->
            rtkConnected = v; renderRtk()
        }
        Sdk.listen(T, "RTK KeyRTKEnable", safe(T, "key") { KeyTools.createKey(RtkMobileStationKey.KeyRTKEnable) }, h) { v ->
            rtkEnabled = v; renderRtk()
        }
        Sdk.listen(T, "RTK KeyRTKLocation", safe(T, "key") { KeyTools.createKey(RtkMobileStationKey.KeyRTKLocation) }, h) { v ->
            val sol = v?.positioningSolution?.name
            if (sol != rtkSolution) Phase0Log.i(T, "RTK positioningSolution=$sol")
            rtkSolution = sol; renderRtk()
        }
        Sdk.listen(T, "Battery[0] %", safe(T, "key") { KeyTools.createKey(BatteryKey.KeyChargeRemainingInPercent, ComponentIndexType.LEFT_OR_MAIN) }, h) { v ->
            batt1 = v; renderBatt()
        }
        Sdk.listen(T, "Battery[1] %", safe(T, "key") { KeyTools.createKey(BatteryKey.KeyChargeRemainingInPercent, ComponentIndexType.RIGHT) }, h) { v ->
            batt2 = v; renderBatt()
        }
        Sdk.listen(T, "KeyFlightMode", safe(T, "key") { KeyTools.createKey(FlightControllerKey.KeyFlightMode) }, h) { v ->
            if (v != flightMode) Phase0Log.i(T, "flightMode=$v")
            flightMode = v; renderMode()
        }
        Sdk.listen(T, "KeyFlightModeString", safe(T, "key") { KeyTools.createKey(FlightControllerKey.KeyFlightModeString) }, h) { v ->
            flightModeString = v; renderMode()
        }
        Sdk.listen(T, "KeyAreMotorsOn", safe(T, "key") { KeyTools.createKey(FlightControllerKey.KeyAreMotorsOn) }, h) { v ->
            if (v != motorsOn) Phase0Log.i(T, "motorsOn=$v")
            motorsOn = v; renderMotors()
        }
        Sdk.listen(T, "KeyIsFlying", safe(T, "key") { KeyTools.createKey(FlightControllerKey.KeyIsFlying) }, h) { v ->
            if (v != isFlying) Phase0Log.i(T, "isFlying=$v")
            isFlying = v; renderMotors()
        }
    }

    private fun dash(v: Any?) = v?.toString() ?: "—"

    private fun renderAlt() {
        tvAlt.text = "FC ${altFc?.let { String.format(Locale.US, "%.1f m", it) } ?: "—"}  |  loc3D ${alt3d?.let { String.format(Locale.US, "%.1f m", it) } ?: "—"}"
    }
    private fun renderGps() { tvGps.text = "${dash(gpsSats)} sats  /  ${dash(gpsLevel)}" }
    private fun renderRtk() {
        tvRtk.text = "connected=${dash(rtkConnected)} enabled=${dash(rtkEnabled)} solution=${dash(rtkSolution)}"
        tvRtk.setTextColor(if (rtkSolution == "FIXED_POINT") Ui.GOOD else Ui.TEXT)
    }
    private fun renderBatt() { tvBatt.text = "[0] ${batt1?.let { "$it %" } ?: "—"}   [1] ${batt2?.let { "$it %" } ?: "—"}" }
    private fun renderMode() { tvMode.text = "${dash(flightMode)}  (${dash(flightModeString)})" }
    private fun renderMotors() { tvMotors.text = "motors=${dash(motorsOn)}  flying=${dash(isFlying)}" }

    // =====================================================================================
    // 3 Payloads

    private fun attachPayloads() {
        val P = "PAYLOAD"
        for (p in payloadPorts) {
            Sdk.listen(P, "CameraType@$p", safe(P, "key") { KeyTools.createKey(CameraKey.KeyCameraType, p) }, payloadHolder) { v ->
                if (v != camType[p]) Phase0Log.i(P, "$p cameraType=$v")
                camType[p] = v; renderPayload(p)
            }
            Sdk.listen(P, "CameraConn@$p", safe(P, "key") { KeyTools.createKey(CameraKey.KeyConnection, p) }, payloadHolder) { v ->
                if (v != camConn[p]) Phase0Log.i(P, "$p camera KeyConnection=$v")
                camConn[p] = v; renderPayload(p)
            }
            Sdk.listen(P, "LidarConn@$p", safe(P, "key") { KeyTools.createKey(LidarKey.KeyConnection, p) }, payloadHolder) { v ->
                if (v != lidarConn[p]) Phase0Log.i(P, "$p lidar KeyConnection=$v")
                lidarConn[p] = v; renderPayload(p)
            }
        }
    }

    private fun rescanPayloads() {
        val P = "PAYLOAD"
        for (p in payloadPorts) {
            Sdk.get(P, "CameraType@$p", safe(P, "key") { KeyTools.createKey(CameraKey.KeyCameraType, p) }) { v ->
                if (v != null && v != camType[p]) Phase0Log.i(P, "$p cameraType=$v (rescan)")
                if (v != null) camType[p] = v
                renderPayload(p)
            }
        }
    }

    private fun renderPayload(p: ComponentIndexType) {
        val row = payloadRows[p] ?: return
        val t = camType[p]
        row.text = "camera=${t?.name ?: "—"}  camConn=${dash(camConn[p])}  lidarConn=${dash(lidarConn[p])}"
        val isL3 = t == CameraType.ZENMUSE_L3
        row.setTextColor(if (isL3) Ui.GOOD else Ui.TEXT)
        if (isL3) row.text = "★ ZENMUSE_L3 ★  camConn=${dash(camConn[p])}  lidarConn=${dash(lidarConn[p])}"
        autoPickLidarPort(force = false)
    }

    // =====================================================================================
    // 4 LiDAR

    private fun detectedL3Port(): ComponentIndexType? =
        payloadPorts.firstOrNull { camType[it] == CameraType.ZENMUSE_L3 }
            ?: payloadPorts.firstOrNull { lidarConn[it] == true }

    private fun autoPickLidarPort(force: Boolean) {
        if (lidarPortManual && !force) return
        val d = detectedL3Port() ?: run {
            if (force) Phase0Log.w("LIDAR", "no L3 / LiDAR detected on any port; keeping $lidarPort")
            renderLidarPort(); return
        }
        if (d != lidarPort || force) {
            Phase0Log.i("LIDAR", "LiDAR port -> $d (auto${if (force) ", forced" else ""})")
            lidarPort = d
            if (listenersAttached) attachLidar()
        }
        renderLidarPort()
    }

    private fun cycleLidarPort(dir: Int) {
        val i = payloadPorts.indexOf(lidarPort)
        lidarPort = payloadPorts[(i + dir + payloadPorts.size) % payloadPorts.size]
        lidarPortManual = true
        Phase0Log.i("LIDAR", "LiDAR port -> $lidarPort (manual)")
        if (listenersAttached) attachLidar()
        renderLidarPort()
    }

    private fun renderLidarPort() {
        if (!::tvLidarPort.isInitialized) return
        val d = detectedL3Port()
        tvLidarPort.text = "$lidarPort (value=${safe("LIDAR", "value") { lidarPort.value() }})  ${if (lidarPortManual) "manual" else "auto"}  detected=${d ?: "none"}"
        tvLidarPort.setTextColor(if (d != null && d == lidarPort) Ui.GOOD else Ui.TEXT)
    }

    private fun attachLidar() {
        val L = "LIDAR"
        for (k in lidarKeysListened) Sdk.cancelListen(k, lidarHolder)
        lidarKeysListened.clear()
        val port = lidarPort

        val kStatus = safe(L, "key") { KeyTools.createKey(LidarKey.KeyPointCloudRecordStatus, port) }
        val kWork = safe(L, "key") { KeyTools.createKey(LidarKey.KeyLidarDataCurWorkState, port) }
        val kScan = safe(L, "key") { KeyTools.createKey(LidarKey.KeyScanMode, port) }
        val kEcho = safe(L, "key") { KeyTools.createKey(LidarKey.KeyEchoMode, port) }
        val kRate = safe(L, "key") { KeyTools.createKey(LidarKey.KeyLidarDataSampleRate, port) }
        val kExcl = safe(L, "key") { KeyTools.createKey(LidarKey.KeyPointCloudExclusiveStatus, port) }
        val kRec = safe(L, "key") { KeyTools.createKey(LidarKey.KeyPointCloudRecord, port) }
        listOfNotNull(kStatus, kWork, kScan, kEcho, kRate, kExcl).forEach { lidarKeysListened.add(it) }

        val support = listOf(
            "Record" to (kRec as DJIKey<*>?), "Status" to kStatus, "Work" to kWork, "Scan" to kScan,
        ).joinToString("  ") { (n, k) -> "$n=${dash(Sdk.isSupported(L, k))}" }
        tvLidarSupport.text = support
        Phase0Log.i(L, "isKeySupported @$port: $support")

        tvRecStatus.text = "—"; tvWorkState.text = "—"; tvScanMode.text = "—"
        tvEcho.text = "—"; tvSampleRate.text = "—"; tvExclusive.text = "—"
        Sdk.listen(L, "KeyPointCloudRecordStatus@$port", kStatus, lidarHolder) { v ->
            if (v != null) Phase0Log.i(L, "recordStatus=$v @$port")
            tvRecStatus.text = v?.name ?: "—"
            tvRecStatus.setTextColor(if (v?.name == "STARTED") Ui.GOOD else Ui.TEXT)
        }
        Sdk.listen(L, "KeyLidarDataCurWorkState@$port", kWork, lidarHolder) { v ->
            val s = safe(L, "curWorkState") { v?.curWorkState?.name }
            if (s != null) Phase0Log.i(L, "curWorkState=$s @$port")
            tvWorkState.text = s ?: "—"
        }
        Sdk.listen(L, "KeyScanMode@$port", kScan, lidarHolder) { v ->
            if (v != null) Phase0Log.i(L, "scanMode=$v @$port")
            tvScanMode.text = v?.name ?: "—"
        }
        Sdk.listen(L, "KeyEchoMode@$port", kEcho, lidarHolder) { v -> tvEcho.text = v?.name ?: "—" }
        Sdk.listen(L, "KeyLidarDataSampleRate@$port", kRate, lidarHolder) { v -> tvSampleRate.text = v?.name ?: "—" }
        Sdk.listen(L, "KeyPointCloudExclusiveStatus@$port", kExcl, lidarHolder) { v -> tvExclusive.text = v?.toString() ?: "—" }
    }

    private fun lidarCmd(cmd: PointCloudRecordCommand) {
        val L = "LIDAR"
        val port = lidarPort
        val key = safe(L, "createKey KeyPointCloudRecord") { KeyTools.createKey(LidarKey.KeyPointCloudRecord, port) } ?: return
        Phase0Log.i(L, "performAction KeyPointCloudRecord $cmd @$port …")
        tvLidarLast.text = "$cmd sent…"
        safe(L, "performAction $cmd") {
            KeyManager.getInstance().performAction(key, cmd, object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(t: EmptyMsg?) {
                    Phase0Log.i(L, "KeyPointCloudRecord $cmd @$port -> SUCCESS")
                    main.post { tvLidarLast.text = "$cmd OK"; tvLidarLast.setTextColor(Ui.GOOD) }
                }

                override fun onFailure(error: IDJIError) {
                    Phase0Log.e(L, "KeyPointCloudRecord $cmd @$port -> FAIL ${error.fmt()}")
                    main.post { tvLidarLast.text = "$cmd FAIL ${error.errorCode()}"; tvLidarLast.setTextColor(Ui.BAD) }
                }
            })
        }
    }

    // =====================================================================================
    // 5 Mission

    private fun missionsDir(): File? = safe("MISSION", "getExternalFilesDir(missions)") { getExternalFilesDir("missions")?.also { it.mkdirs() } }

    private fun refreshMissionList() {
        val dir = missionsDir()
        tvMissionDir.text = dir?.absolutePath ?: "—"
        missions = dir?.listFiles { f -> f.isFile && f.name.lowercase().endsWith(".kmz") }?.sortedBy { it.name }.orEmpty()
        Phase0Log.i("MISSION", "found ${missions.size} KMZ in ${dir?.absolutePath}: ${missions.joinToString { it.name }}")
        missionIdx = missionIdx.coerceIn(0, (missions.size - 1).coerceAtLeast(0))
        renderMission()
    }

    private fun selectMission(dir: Int) {
        if (missions.isEmpty()) { refreshMissionList(); return }
        missionIdx = (missionIdx + dir + missions.size) % missions.size
        renderMission()
    }

    private fun currentMission(): File? = missions.getOrNull(missionIdx)

    private fun renderMission() {
        val f = currentMission()
        if (f == null) {
            tvMissionFile.text = "(none — adb push a .kmz into the dir above)"
            tvWaylineIds.text = "—"
            return
        }
        tvMissionFile.text = "${missionIdx + 1}/${missions.size}  ${f.name}  (${f.length()} B)"
        val ids = safe("MISSION", "getAvailableWaylineIDs") { WaypointMissionManager.getInstance().getAvailableWaylineIDs(f.absolutePath) }
        tvWaylineIds.text = ids?.toString() ?: "—"
        Phase0Log.i("MISSION", "selected ${f.name}; getAvailableWaylineIDs=$ids")
    }

    private fun pushMission() {
        val f = currentMission() ?: run { Phase0Log.w("MISSION", "no KMZ selected"); return }
        Phase0Log.i("MISSION", "pushKMZFileToAircraft ${f.absolutePath} …")
        tvUpload.text = "pushing…"
        pbUpload.progress = 0
        safe("MISSION", "pushKMZFileToAircraft") {
            WaypointMissionManager.getInstance().pushKMZFileToAircraft(f.absolutePath,
                object : CommonCallbacks.CompletionCallbackWithProgress<Double> {
                    private var lastPct = -1
                    override fun onProgressUpdate(progress: Double?) {
                        val raw = progress ?: return
                        // Scale unknown (0..1 or 0..100) — show raw value and log it.
                        val pct = (if (raw <= 1.0) raw * 100 else raw).toInt().coerceIn(0, 100)
                        if (pct / 10 != lastPct / 10) Phase0Log.i("MISSION", "upload progress raw=$raw")
                        lastPct = pct
                        main.post { pbUpload.progress = pct; tvUpload.text = "raw progress=$raw" }
                    }

                    override fun onSuccess() {
                        Phase0Log.i("MISSION", "pushKMZFileToAircraft SUCCESS ${f.name}")
                        main.post { pbUpload.progress = 100; tvUpload.text = "uploaded OK"; tvUpload.setTextColor(Ui.GOOD) }
                    }

                    override fun onFailure(error: IDJIError) {
                        Phase0Log.e("MISSION", "pushKMZFileToAircraft FAIL ${error.fmt()}")
                        main.post { tvUpload.text = "FAIL ${error.errorCode()}"; tvUpload.setTextColor(Ui.BAD) }
                    }
                })
        }
    }

    private fun startMission() {
        val f = currentMission() ?: run { Phase0Log.w("MISSION", "no KMZ selected"); return }
        if (flightMode == FlightMode.GO_HOME || flightMode == FlightMode.AUTO_LANDING) {
            Phase0Log.w("MISSION", "refusing start: aircraft in $flightMode (same check as DJI sample)")
            return
        }
        // Mission ID = KMZ file name without extension (as in the DJI sample).
        val missionId = f.nameWithoutExtension
        val ids = safe("MISSION", "getAvailableWaylineIDs") { WaypointMissionManager.getInstance().getAvailableWaylineIDs(f.absolutePath) }
        Phase0Log.i("MISSION", "startMission id=$missionId waylineIDs=$ids")
        missionCall("startMission") { cb ->
            if (ids.isNullOrEmpty()) WaypointMissionManager.getInstance().startMission(missionId, cb)
            else WaypointMissionManager.getInstance().startMission(missionId, ids, cb)
        }
    }

    private fun stopMission() {
        val f = currentMission() ?: run { Phase0Log.w("MISSION", "no KMZ selected"); return }
        missionCall("stopMission(${f.nameWithoutExtension})") { cb -> WaypointMissionManager.getInstance().stopMission(f.nameWithoutExtension, cb) }
    }

    private fun missionCall(name: String, call: (CommonCallbacks.CompletionCallback) -> Unit) {
        Phase0Log.i("MISSION", "$name …")
        safe("MISSION", name) {
            call(object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() { Phase0Log.i("MISSION", "$name SUCCESS") }
                override fun onFailure(error: IDJIError) { Phase0Log.e("MISSION", "$name FAIL ${error.fmt()}") }
            })
        }
    }

    // =====================================================================================
    // 6 Simulator

    private fun enableSimulator() {
        val lat = etSimLat.text.toString().trim().toDoubleOrNull()
        val lon = etSimLon.text.toString().trim().toDoubleOrNull()
        val sats = etSimSats.text.toString().trim().toDoubleOrNull()?.toInt()
        if (lat == null || lon == null || sats == null || lat !in -90.0..90.0 || lon !in -180.0..180.0) {
            Phase0Log.w("SIM", "invalid lat/lon/sats input: '${etSimLat.text}', '${etSimLon.text}', '${etSimSats.text}'")
            return
        }
        Phase0Log.i("SIM", "enableSimulator lat=$lat lon=$lon sats=$sats …")
        safe("SIM", "enableSimulator") {
            val settings = InitializationSettings.createInstance(LocationCoordinate2D(lat, lon), sats)
            SimulatorManager.getInstance().enableSimulator(settings, object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() { Phase0Log.i("SIM", "enableSimulator SUCCESS") }
                override fun onFailure(error: IDJIError) { Phase0Log.e("SIM", "enableSimulator FAIL ${error.fmt()}") }
            })
        }
    }

    private fun disableSimulator() {
        Phase0Log.i("SIM", "disableSimulator …")
        safe("SIM", "disableSimulator") {
            SimulatorManager.getInstance().disableSimulator(object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() { Phase0Log.i("SIM", "disableSimulator SUCCESS") }
                override fun onFailure(error: IDJIError) { Phase0Log.e("SIM", "disableSimulator FAIL ${error.fmt()}") }
            })
        }
    }

    private var simPollErrorLogged = false
    private fun pollSimulator() {
        val now = SystemClock.elapsedRealtime()
        val enabled: Boolean? = try {
            SimulatorManager.getInstance().isSimulatorEnabled
        } catch (t: Throwable) {
            if (!simPollErrorLogged) { Phase0Log.e("SIM", "isSimulatorEnabled threw (logged once)", t); simPollErrorLogged = true }
            null
        }
        if (enabled != simEnabledLast) {
            if (enabled == true) {
                simEnabledSince = now
                Phase0Log.i("SIM", "isSimulatorEnabled -> true")
            } else if (simEnabledLast == true) {
                val secs = (now - simEnabledSince) / 1000.0
                Phase0Log.w("SIM", String.format(Locale.US, "isSimulatorEnabled -> %s after %.3f s enabled%s",
                    enabled, secs, if (secs < 3.0) "  <-- matches DJI issue #759 symptom?" else ""))
            } else {
                Phase0Log.i("SIM", "isSimulatorEnabled -> $enabled")
            }
            simEnabledLast = enabled
        }
        if (simStreamLive && now - simLastUpdate > 1500) {
            simStreamLive = false
            Phase0Log.w("SIM", "state updates stopped (no update for >1.5 s)")
        }
        if (::tvSimEnabled.isInitialized) {
            tvSimEnabled.text = when (enabled) {
                true -> String.format(Locale.US, "ON for %.1f s", (now - simEnabledSince) / 1000.0)
                false -> "off"
                null -> "—"
            }
            tvSimEnabled.setTextColor(Ui.colorFor(enabled))
        }
    }

    // =====================================================================================
    // 7 Probe

    private fun runProbe() {
        tvProbe.text = "running…"
        Thread {
            val r = try { StorageProbe.run(applicationContext) } catch (t: Throwable) {
                Phase0Log.e("PROBE", "probe failed", t); "probe failed: ${t.message}"
            }
            main.post { tvProbe.text = r }
        }.start()
    }

    private fun openAllFilesSettings() {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.R) {
            Phase0Log.i("PROBE", "All-files access is Android 11+ only"); return
        }
        safe("PROBE", "open all-files settings") {
            startActivity(Intent(Settings.ACTION_MANAGE_APP_ALL_FILES_ACCESS_PERMISSION, Uri.parse("package:$packageName")))
        }
    }

    // =====================================================================================
    // 8 Log share

    private fun shareLog() {
        val f = Phase0Log.logFile ?: return
        safe("LOG", "share") {
            val uri = FileProvider.getUriForFile(this, "$packageName.fileprovider", f)
            val i = Intent(Intent.ACTION_SEND).apply {
                type = "text/plain"
                putExtra(Intent.EXTRA_STREAM, uri)
                putExtra(Intent.EXTRA_SUBJECT, f.name)
                addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
            }
            startActivity(Intent.createChooser(i, "Share Phase 0 log"))
        }
    }

    // =====================================================================================
    // Permissions (same set the DJI sample requests at runtime)

    private fun requestPermissionsIfNeeded() {
        val wanted = mutableListOf(
            Manifest.permission.RECORD_AUDIO,
            Manifest.permission.KILL_BACKGROUND_PROCESSES,
            Manifest.permission.ACCESS_COARSE_LOCATION,
            Manifest.permission.ACCESS_FINE_LOCATION,
        )
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.TIRAMISU) {
            wanted += Manifest.permission.READ_EXTERNAL_STORAGE
            wanted += Manifest.permission.WRITE_EXTERNAL_STORAGE
        }
        val missing = wanted.filter { checkSelfPermission(it) != PackageManager.PERMISSION_GRANTED }
        if (missing.isNotEmpty()) {
            Phase0Log.i("PERM", "requesting ${missing.joinToString()}")
            requestPermissions(missing.toTypedArray(), 1)
        }
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        permissions.forEachIndexed { i, p ->
            Phase0Log.i("PERM", "$p -> ${if (grantResults.getOrNull(i) == PackageManager.PERMISSION_GRANTED) "granted" else "DENIED"}")
        }
    }
}
