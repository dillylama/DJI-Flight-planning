package com.threedronemapping.fly

import android.content.Context
import android.os.Handler
import android.os.Looper
import dji.sdk.keyvalue.key.DJIKey
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.common.register.DJISDKInitEvent
import dji.v5.manager.KeyManager
import dji.v5.manager.SDKManager
import dji.v5.manager.interfaces.SDKManagerCallback
import dji.v5.network.DJINetworkManager
import java.util.concurrent.CopyOnWriteArrayList

/** Formats an MSDK error with every field the interface exposes. */
fun IDJIError?.fmt(): String {
    if (this == null) return "error=null"
    return try {
        "type=${errorType()} code=${errorCode()} inner=${innerCode()} hint=${hint()} desc=${description()}"
    } catch (t: Throwable) {
        "error(unformattable: ${t.message})"
    }
}

/** Runs [block]; on any Throwable logs it and returns null. Every SDK call goes through this. */
inline fun <T> safe(section: String, what: String, block: () -> T): T? =
    try {
        block()
    } catch (t: Throwable) {
        Phase0Log.e(section, "$what threw", t)
        null
    }

/**
 * SDK init + registration, mirroring the official sample's MSDKManagerVM:
 * SDKManager.init(context, SDKManagerCallback) from Application.onCreate, registerApp() on
 * INITIALIZE_COMPLETE, and a DJINetworkManager listener that retries registration when the
 * network comes back (first run needs internet).
 */
object Sdk {
    private const val S = "SDK"
    private val main = Handler(Looper.getMainLooper())

    data class State(
        var initEvent: String = "not started",
        var initDone: Boolean = false,
        var registered: Boolean? = null,
        var registerError: String = "",
        var productConnected: Boolean = false,
        var productId: Int = -1,
        var dbProgress: String = "",
        var network: String = "unknown",
    )

    val state = State()
    private val listeners = CopyOnWriteArrayList<() -> Unit>()
    fun addListener(l: () -> Unit) { listeners.add(l) }
    fun removeListener(l: () -> Unit) { listeners.remove(l) }
    private fun changed() = main.post { listeners.forEach { try { it() } catch (_: Throwable) {} } }

    fun init(context: Context) {
        safe(S, "SDKManager.init") {
            Phase0Log.i(S, "SDKManager.init() … MSDK ${safe(S, "getSDKVersion") { SDKManager.getInstance().sdkVersion }}")
            SDKManager.getInstance().init(context, object : SDKManagerCallback {
                override fun onRegisterSuccess() {
                    Phase0Log.i(S, "onRegisterSuccess")
                    state.registered = true
                    state.registerError = ""
                    changed()
                }

                override fun onRegisterFailure(error: IDJIError) {
                    Phase0Log.e(S, "onRegisterFailure ${error.fmt()}")
                    state.registered = false
                    state.registerError = error.fmt()
                    changed()
                }

                override fun onProductDisconnect(productId: Int) {
                    Phase0Log.w(S, "onProductDisconnect productId=$productId")
                    state.productConnected = false
                    state.productId = productId
                    changed()
                }

                override fun onProductConnect(productId: Int) {
                    Phase0Log.i(S, "onProductConnect productId=$productId")
                    state.productConnected = true
                    state.productId = productId
                    changed()
                }

                override fun onProductChanged(productId: Int) {
                    Phase0Log.i(S, "onProductChanged productId=$productId")
                    state.productId = productId
                    changed()
                }

                override fun onInitProcess(event: DJISDKInitEvent, totalProcess: Int) {
                    Phase0Log.i(S, "onInitProcess event=$event totalProcess=$totalProcess")
                    state.initEvent = "$event ($totalProcess)"
                    if (event == DJISDKInitEvent.INITIALIZE_COMPLETE) {
                        state.initDone = true
                        register()
                    }
                    changed()
                }

                override fun onDatabaseDownloadProgress(current: Long, total: Long) {
                    state.dbProgress = "$current / $total"
                    if (current == total) Phase0Log.i(S, "onDatabaseDownloadProgress $current/$total")
                    changed()
                }
            })
        }

        safe(S, "DJINetworkManager.addNetworkStatusListener") {
            DJINetworkManager.getInstance().addNetworkStatusListener { isAvailable ->
                Phase0Log.i(S, "network available=$isAvailable")
                state.network = if (isAvailable) "online" else "offline"
                val registered = safe(S, "isRegistered") { SDKManager.getInstance().isRegistered } ?: false
                if (state.initDone && isAvailable && !registered) register()
                changed()
            }
        }
    }

    fun register() {
        safe(S, "registerApp") {
            Phase0Log.i(S, "registerApp() … (first run needs internet)")
            SDKManager.getInstance().registerApp()
        }
    }

    fun isRegistered(): Boolean = safe(S, "isRegistered") { SDKManager.getInstance().isRegistered } ?: false

    // ---------- KeyManager helpers (all guarded) ----------

    /** Listen to a key; the callback gets the new value (may be null) on the main thread. */
    fun <T> listen(section: String, name: String, key: DJIKey<T>?, holder: Any, onValue: (T?) -> Unit) {
        if (key == null) { Phase0Log.w(section, "$name: key is null"); return }
        safe(section, "listen $name") {
            KeyManager.getInstance().listen(key, holder, CommonCallbacks.KeyListener<T> { _, newValue ->
                main.post { try { onValue(newValue) } catch (t: Throwable) { Phase0Log.e(section, "$name UI update", t) } }
            })
        }
        // Prime with the cached value (listen() only fires on change).
        get(section, name, key) { v -> onValue(v) }
    }

    /** Async getValue; logs failures, callback on the main thread with the value or null. */
    fun <T> get(section: String, name: String, key: DJIKey<T>?, logFailure: Boolean = false, onValue: (T?) -> Unit) {
        if (key == null) return
        safe(section, "getValue $name") {
            KeyManager.getInstance().getValue(key, object : CommonCallbacks.CompletionCallbackWithParam<T> {
                override fun onSuccess(t: T) { main.post { onValue(t) } }
                override fun onFailure(error: IDJIError) {
                    if (logFailure) Phase0Log.w(section, "getValue $name failed: ${error.fmt()}")
                    main.post { onValue(null) }
                }
            })
        }
    }

    fun isSupported(section: String, key: DJIKey<*>?): Boolean? =
        if (key == null) null else safe(section, "isKeySupported") { KeyManager.getInstance().isKeySupported(key) }

    fun cancelListen(holder: Any) {
        safe(S, "cancelListen") { KeyManager.getInstance().cancelListen(holder) }
    }

    fun cancelListen(key: DJIKey<*>?, holder: Any) {
        if (key == null) return
        safe(S, "cancelListen(key)") { KeyManager.getInstance().cancelListen(key, holder) }
    }
}
