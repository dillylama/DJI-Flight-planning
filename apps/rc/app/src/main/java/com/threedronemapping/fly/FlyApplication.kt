package com.threedronemapping.fly

import android.app.Application
import android.content.Context

/**
 * Mirrors the DJI MSDK v5 sample (DJIAircraftApplication + DJIApplication):
 *  - attachBaseContext: super, then com.cySdkyc.clx.Helper.install(this)  (required by MSDK)
 *  - onCreate: SDKManager.init(...) first; registerApp() is called on INITIALIZE_COMPLETE.
 */
class FlyApplication : Application() {

    override fun attachBaseContext(base: Context?) {
        super.attachBaseContext(base)
        com.cySdkyc.clx.Helper.install(this)
    }

    override fun onCreate() {
        super.onCreate()
        Phase0Log.init(this)
        Sdk.init(this)
    }
}
