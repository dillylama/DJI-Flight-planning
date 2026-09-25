# apps/rc: "3DM Fly" Phase 0 go/no-go app

Android, Kotlin, DJI Mobile SDK v5 **5.18.0**. Package `com.threedronemapping.fly`.
Target: **DJI Matrice 400 + Zenmuse L3 on the DJI RC Plus 2 Enterprise** (Android 11).

This build is a bench test console, not the flying app. It answers one question: can our own
MSDK app drive the M400 + L3 well enough to build Phase 1–3 on it? Later phases (sync, pre-flight
checklist, fly/monitor, auto-resume, sorties, as-flown logs) are described in `/CLAUDE.md`.

## What Phase 0 tests

| # | Section | What it proves |
|---|---|---|
| 1 | SDK | App key registers (`SDKManager.init` → `registerApp`), product connects, product type = `DJI_MATRICE_400`, RC type = `DJI_RC_PLUS_2` |
| 2 | Telemetry | `KeyManager` listeners deliver position, altitude, GPS sats and signal level, RTK (connected, enabled, positioning solution), battery %, flight mode, motors/flying |
| 3 | Payloads | `CameraKey.KeyCameraType` on `PORT_1…PORT_7` (plus legacy indices), and whether `ZENMUSE_L3` shows up and on which port |
| 4 | LiDAR | `LidarKey.KeyPointCloudRecord` START / PAUSE / RESUME / STOP on the L3 port; live `KeyPointCloudRecordStatus`, `KeyLidarDataCurWorkState`, `KeyScanMode` (plus echo mode, sample rate, exclusive status) |
| 5 | Waypoints | KMZ from `files/missions/`: `pushKMZFileToAircraft` (with progress), `startMission`, `pauseMission`, `resumeMission`, `stopMission`; live execute state, `WaylineExecutingInfo` (wayline id, waypoint index) and waypoint action events |
| 6 | Simulator | `SimulatorManager.enableSimulator` at a typed lat/lon/satellite count. The enabled state is polled every 200 ms and every change is logged with a timestamp, to catch DJI issue #759 (simulator turns itself off after about 1 s on M400 + RC Plus 2) |
| 7 | Pilot 2 probe | **Read-only.** Android version, whether `/sdcard/Android/data` can be listed, which DJI packages and folders are visible, and `canRead`/`canWrite` for each one. Nothing is written or deleted |
| 8 | Log | Every callback, result and error code, with timestamps. Shown on screen and appended to `files/logs/phase0-<yyyyMMdd>.log`. **Mark** adds a separator line. **Share log** sends the file |

Every SDK call is wrapped in try/catch, so the app must never crash when no aircraft is connected.
If it does crash, that counts as a finding: send the logcat.

## Build

Prerequisites: JDK 17 and an Android SDK with platform 35 and build-tools 35.0.0. Gradle comes
from the wrapper (8.12).

`apps/rc/local.properties` (gitignored, **never commit**):

```
sdk.dir=C:/Users/luke/AppData/Local/Android/Sdk
DJI_APP_KEY=<key from developer.dji.com for package com.threedronemapping.fly>
```

The key is read in `app/build.gradle.kts` and injected into the manifest
(`com.dji.sdk.API_KEY` ← `${DJI_API_KEY}` placeholder). It is not stored anywhere else.

```bash
cd apps/rc
export JAVA_HOME="/c/Program Files/Eclipse Adoptium/jdk-17.0.20.101-hotspot"   # Git Bash
./gradlew assembleDebug          # Windows cmd: gradlew.bat assembleDebug
# -> app/build/outputs/apk/debug/app-debug.apk
```

The first build downloads the MSDK (about 140 MB). The APK is arm64-v8a only, like the DJI sample.

## Install on the RC Plus 2

1. RC: **Settings → About → tap "Build number" 7×** to turn on Developer options. Then open
   **Settings → System → Developer options** and turn on **USB debugging**. Menu names can differ a
   little between RC firmware versions.
2. Connect the RC's USB-C port to the PC and accept the RSA prompt on the RC. Check with `adb devices`.
3. Install (or update) the app:
   ```bash
   adb install -r app/build/outputs/apk/debug/app-debug.apk
   ```
4. **First launch needs internet** (Wi-Fi on the RC) so the app key can register. Registration is
   cached after that, and needed again after a reinstall or clearing app data.
5. Push the test KMZ. This is the Waypoint Route exported from the RC (see `/CLAUDE.md` → "Blocked on").
   No sample KMZ is bundled, and we don't invent WPML.
   ```bash
   adb shell mkdir -p /sdcard/Android/data/com.threedronemapping.fly/files/missions
   adb push test.kmz /sdcard/Android/data/com.threedronemapping.fly/files/missions/
   ```
   In the app, tap **5 → Refresh list**.
6. Pull the logs afterwards:
   ```bash
   adb pull /sdcard/Android/data/com.threedronemapping.fly/files/logs ./phase0-logs
   adb logcat -d -s 3DMFly > phase0-logcat.txt     # same lines, plus any crash
   ```

**Close DJI Pilot 2 before starting 3DM Fly.** Running both at once can corrupt video and wayline
state (DJI release notes). Only one app can hold the aircraft link at a time.

## Safety

- **Props OFF for every test in this document.** The aircraft stays on the bench.
- **Simulator only** for anything that could spin motors or start a mission. Enable the simulator
  (section 6) and confirm "ON" before you press mission **Start**.
- If the simulator switches itself off (issue #759), **do not start or resume a mission**. Stop it,
  reboot the aircraft, and enable the simulator again.
- Keep the RC's physical **RTH/pause** button in reach. Pausing the mission from the RC overrides the app.
- The L3 can be powered and recording on the bench (no flight needed). Obey the L3's laser-safety
  guidance: don't look into the aperture, and point it away from people.

## Bench test script (tap **Mark** in the log before each step)

Record PASS/FAIL for each gate. Then pull `files/logs` and send it back along with these results.

**Setup:** RC Plus 2 on Wi-Fi, M400 on the bench with props off, L3 mounted, aircraft powered, Pilot 2 closed.

1. **Gate 1a: Registration.** Launch 3DM Fly.
   PASS: section 1 shows *Init* `INITIALIZE_COMPLETE`, *Registered* **YES**, and the *Register error* line is blank.
   FAIL: record the `type/code/inner/hint/desc` from the log.
2. **Gate 1b: Connect + identity.** Power the aircraft (if it isn't already on) and wait 30 s.
   PASS: *Product connected* YES, *Product type* `DJI_MATRICE_400`, *RC type* `DJI_RC_PLUS_2`,
   *Flight controller* connected.
3. **Gate 2: Telemetry.** Watch section 2 for 60 s (outdoors or near a window for GNSS).
   PASS: lat/lon are plausible and update, the satellite count is above 0, the GPS signal level is
   not `—`, battery % is shown for at least one index, and the flight mode is shown. The RTK line shows a
   value (`—` is acceptable when no RTK source is configured; note the solution if there is one).
4. **Gate 3: L3 detected.** In section 3, tap **Rescan payloads**.
   PASS: one port row shows **★ ZENMUSE_L3 ★**. Write down which port (for example `PORT_1`) and whether
   `lidarConn=true` on that same port. Section 4's *LiDAR port* switches to it automatically ("auto",
   shown in green).
5. **Gate 4: Point-cloud record start/stop.** In section 4, check *Key supported* (Record/Status =
   true is expected), then:
   1. **Start rec**. PASS: the log shows `KeyPointCloudRecord START … SUCCESS` and *Record status* moves to `STARTED` (it may pass through `STARTING`).
   2. After 20 s, **Pause**. The status should go to `PAUSING` or a paused state. Then **Resume**, and the status should go to `RESUME`/`STARTED`.
   3. **Stop rec**. PASS: SUCCESS, and the status goes to `STOPPED`.
   4. Check the L3's storage (or Pilot 2 → album) for a new point-cloud file.
   FAIL: record the error code. If nothing works on the auto port, use **◂ Port / Port ▸** to try
   the other ports, and also try `LEFT_OR_MAIN`, and repeat.
6. **Gate 5a: Simulator stays on.** In section 6 (defaults −33.2270, 22.0310, 12 sats), tap **Enable simulator**.
   PASS: `enableSimulator SUCCESS`, *Simulator enabled* stays **ON** for more than 60 s, and the telemetry lat/lon
   jump to the simulated position.
   FAIL (issue #759): the log shows `isSimulatorEnabled -> false after ~1 s`. Reboot the aircraft, enable
   again, and record whether the reboot fixes it.
7. **Gate 5b: KMZ push + start in the simulator.** With the simulator ON (Gate 5a passed):
   1. Section 5: **Refresh list**. The KMZ is listed and *Wayline IDs* is not empty (e.g. `[0]`).
   2. **Push to aircraft**. PASS: progress runs to `uploaded OK`, and the log shows `pushKMZFileToAircraft SUCCESS`.
   3. **Start**. PASS: `startMission SUCCESS`, and *Execute state* moves through `PREPARING`/`ENTER_WAYLINE` to
      `EXECUTING`. *Executing info* shows the wayline id and the waypoint index increasing.
      Waypoint action events (e.g. start/stop recording actions in the KMZ) appear under *Last action*.
      Also watch section 4: if the KMZ contains point-cloud actions, *Record status* should follow them.
   4. **Pause**, then **Resume**, then **Stop**. Each should log SUCCESS and change the execute state.
   5. **Disable simulator** at the end. PASS: `disableSimulator SUCCESS`.
8. **Gate 6: Pilot 2 probe.** Section 7, **Run probe**. Then tap **All-files access…**, grant it,
   come back, and **Run probe** again.
   No PASS/FAIL here. Record the Android version, whether `Android/data list()` is `null` (expected
   on Android 11), the installed DJI package names (this gives Pilot 2's real package id), and
   `canRead`/`canWrite` for each DJI folder, both without and with All-files access. This decides whether
   KMZ injection into Pilot 2's route library is possible (see `docs/msdk-research.md`).
9. **Crash check.** Power off the aircraft with the app open, then press every button once.
   PASS: no crash, and each failure is logged with an error code.

**Go:** gates 1–5b pass (5a may need an aircraft reboot).
**No-go / re-plan:** Gate 4 fails on every port (L3 recording then has to go through WPML actions or
Pilot 2), or Gate 5b fails (then we can't fly our own routes from our app).
