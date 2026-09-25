package com.threedronemapping.fly

import android.app.Activity
import android.content.Intent
import android.os.Bundle

/**
 * Same as the DJI sample's UsbAttachActivity: receives USB_ACCESSORY_ATTACHED (the RC's link to
 * the aircraft) and brings the main activity to the front.
 */
class UsbAttachActivity : Activity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        Phase0Log.i("USB", "USB accessory attached intent: ${intent?.action}")
        val i = Intent(this, MainActivity::class.java)
        i.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TASK)
        startActivity(i)
        finish()
    }
}
