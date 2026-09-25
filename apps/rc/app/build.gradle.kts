import java.util.Properties

plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

// The DJI app key lives ONLY in apps/rc/local.properties (gitignored): DJI_APP_KEY=...
// It is injected into the manifest via a placeholder and never written anywhere else.
val localProps = Properties().apply {
    val f = rootProject.file("local.properties")
    if (f.exists()) f.inputStream().use { load(it) }
}
val djiAppKey: String = localProps.getProperty("DJI_APP_KEY")?.trim().orEmpty()
if (djiAppKey.isEmpty()) {
    logger.warn("WARNING: DJI_APP_KEY missing from apps/rc/local.properties - SDK registration will fail.")
}

android {
    namespace = "com.threedronemapping.fly"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.threedronemapping.fly" // must match the DJI developer-portal app exactly
        minSdk = 24
        targetSdk = 35
        versionCode = 1
        versionName = "0.1.0-phase0"
        manifestPlaceholders["DJI_API_KEY"] = djiAppKey
        ndk {
            // MSDK v5 ships arm64-v8a native libs only (RC Plus 2 is arm64).
            abiFilters += "arm64-v8a"
        }
    }

    buildTypes {
        debug {
            isMinifyEnabled = false
        }
        release {
            // Phase 0 is debug-only. If minify is ever enabled, add the MSDK keep rules first.
            isMinifyEnabled = false
        }
    }

    compileOptions {
        // Same as the DJI sample.
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }
    kotlinOptions {
        jvmTarget = "1.8"
        freeCompilerArgs += listOf("-Xjvm-default=all")
    }

    buildFeatures {
        buildConfig = true
    }

    packaging {
        jniLibs {
            // From the DJI sample: libc++_shared.so is shipped by more than one MSDK artifact.
            pickFirsts += listOf("lib/arm64-v8a/libc++_shared.so", "lib/armeabi-v7a/libc++_shared.so")
            // From the DJI sample: these MSDK libs must not be stripped.
            keepDebugSymbols += listOf(
                "*/*/libconstants.so",
                "*/*/libdji_innertools.so",
                "*/*/libdjibase.so",
                "*/*/libDJICSDKCommon.so",
                "*/*/libDJIFlySafeCore-CSDK.so",
                "*/*/libdjifs_jni-CSDK.so",
                "*/*/libDJIRegister.so",
                "*/*/libdjisdk_jni.so",
                "*/*/libDJIUpgradeCore.so",
                "*/*/libDJIUpgradeJNI.so",
                "*/*/libDJIWaypointV2Core-CSDK.so",
                "*/*/libdjiwpv2-CSDK.so",
                "*/*/libFlightRecordEngine.so",
                "*/*/libvideo-framing.so",
                "*/*/libwaes.so",
                "*/*/libagora-rtsa-sdk.so",
                "*/*/libc++.so",
                "*/*/libc++_shared.so",
                "*/*/libmrtc_28181.so",
                "*/*/libmrtc_agora.so",
                "*/*/libmrtc_core.so",
                "*/*/libmrtc_core_jni.so",
                "*/*/libmrtc_data.so",
                "*/*/libmrtc_log.so",
                "*/*/libmrtc_onvif.so",
                "*/*/libmrtc_rtmp.so",
                "*/*/libmrtc_rtsp.so",
            )
            // Manifest also sets android:extractNativeLibs="true" (DJI requirement).
            useLegacyPackaging = true
        }
    }

    lint {
        checkReleaseBuilds = false
        abortOnError = false
    }
}

dependencies {
    // DJI Mobile SDK v5 (coordinates as in the official sample, dependencies.gradle)
    implementation("com.dji:dji-sdk-v5-aircraft:5.18.0")
    compileOnly("com.dji:dji-sdk-v5-aircraft-provided:5.18.0")
    runtimeOnly("com.dji:dji-sdk-v5-networkImp:5.18.0")

    implementation("androidx.core:core-ktx:1.13.1")
}
