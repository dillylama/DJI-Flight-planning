# DJI SDK research: M400 + Zenmuse L3 (25 Sep 2026)

VERIFIED = found in an official DJI source. INFERRED = reasoned or from community sources.
NOT FOUND = no source found. Re-check against the RC sample export and on hardware.

## MSDK v5
| Item | Finding | Status |
|---|---|---|
| First MSDK with M400 | 5.15.0 (2025-06-10); payloads are addressed as `ComponentIndexType.PORT_1…PORT_7` | VERIFIED |
| First MSDK with L3 | 5.17.0 (2025-11-04) | VERIFIED |
| Latest | **5.18.0** (2026-05-22): `com.dji:dji-sdk-v5-aircraft:5.18.0`, `compileOnly dji-sdk-v5-aircraft-provided`, `runtimeOnly dji-sdk-v5-networkImp`, plus `com.dji:wpmzsdk:1.0.5.x` | VERIFIED |
| Toolchain | minSdk 24, target/compileSdk 35, Kotlin 2.1.0, Gradle 8.12, AGP 8.7.0, JDK 17; arm64-v8a only; `android:extractNativeLibs="true"` | VERIFIED |
| RC | "Matrice 400 + DJI RC Plus 2" supported (H30, P1, L2, L3); RC Plus 2 runs **Android 11** | VERIFIED |
| App key registration | Needs internet on first run, then cached; again after a reinstall. Local Data Mode is an offline option | VERIFIED |

## Zenmuse L3 / LiDAR keys
- `dji.sdk.keyvalue.key.LidarKey` (in the 5.18.0 jar; the online docs page 404s):
  - `KeyPointCloudRecord`: action taking `PointCloudRecordCommand` START / STOP / PAUSE / RESUME.
  - `KeyPointCloudRecordStatus`: STARTING / STARTED / PAUSING / RESUME / STOPPING / STOPPED.
  - Also `KeyLidarDataCurWorkState`, `KeyScanMode`, `KeyEchoMode`, `KeyLidarDataSampleRate`,
    `KeyPayloadPOSStatusInfo`, `KeyStartLidarSelfCalibration`.
- These keys exist (VERIFIED). That they **work on L3** is **NOT FOUND**, so Phase 0 must test it.
- Point-cloud live view (`POINT_CLOUD_CAMERA`) is documented for L2 only. For L3: NOT FOUND.

## Waypoint missions
- `WaypointMissionManager`: `pushKMZFileToAircraft`, `getAvailableWaylineIDs`, `startMission`,
  `pauseMission`, `resumeMission`, `stopMission`, `queryBreakPointInfoFromAircraft`; listeners
  for execute state (…EXECUTING / INTERRUPTED / RECOVERING / FINISHED), `WaylineExecutingInfo`
  (current waypoint index) and actions. VERIFIED.
- Used on M400 with MSDK 5.16 per a DJI GitHub issue (#778). INFERRED.
- WPML allows waypoint index 0–65535; the real M400 / Pilot 2 cap is NOT FOUND and must be tested.
- Simulator: `SimulatorManager` exists. Open issue #759: on M400 + RC Plus 2 the simulator turns
  itself off after about 1 s unless the aircraft is rebooted. **Risk for bench testing.**

## WPML
| Item | Finding | Status |
|---|---|---|
| M400 `droneEnumValue` | **103**, sub type 0 | VERIFIED (Cloud API product support) |
| L3 `payloadEnumValue` | Not published. WPMZ SDK has `EP820 (117)`; mapping it to L3 is a guess | NOT FOUND, take from RC sample |
| Point-cloud action | `recordPointCloud` with `wpml:recordPointCloudOperate` = startRecord / pauseRecord / resumeRecord / stopRecord (+ `payloadPositionIndex`). Documented for M300/M350 only | VERIFIED spec, M400 unconfirmed |
| Heights | `waylines.wpml` `executeHeightMode` = **WGS84 ellipsoidal** / relativeToStartPoint / realTimeFollowSurface. `template.kml` `heightMode` = **EGM96** / relativeToStartPoint / aboveGroundLevel / realTimeFollowSurface | VERIFIED |

**Height consequence:** GLO-30 heights are orthometric (EGM2008). `waylines.wpml` needs ellipsoidal
heights (h + N), and `template.kml` in EGM96 needs orthometric heights. EGM2008 and EGM96 differ by
up to a few metres, which is negligible at 500 m AGL but should be documented. The writer needs a
geoid model, not a single constant `geoidN`, for blocks with strong geoid slope.

## Pilot 2 injection
- The file path `/Android/data/dji.go.v5/files/waypoint/<uuid>/<uuid>.kmz` comes from community
  sources and is DJI **Fly**'s folder. For Pilot 2 it is NOT VERIFIED.
- Android 11 scoped storage: another app can't write into Pilot 2's `/Android/data` folder. INFERRED.
- Official options: Pilot 2 **Import Route** (file), or the **Cloud API waypoint library**
  (Pilot 2 logged into a third-party cloud; M400 + RC Plus 2 are listed). VERIFIED.
- Pilot 2 and an MSDK app can be switched between (since 5.6.0). Running both at once can corrupt
  video and wayline state, so close Pilot 2 before flying from our app. VERIFIED.

## Risks and what to do about them
1. L3 recording through LiDAR keys or `recordPointCloud` on M400 is unproven. **Phase 0** tests both,
   and the app can drive recording directly as a backup to the KMZ actions.
2. L3 payload enum unknown: copy it from the RC sample export.
3. File injection into Pilot 2 is fragile. Prefer flying from our app; for Pilot 2, use Import Route
   or a Cloud API waypoint library we host ourselves (an official route into Pilot 2).
4. Simulator instability on M400 (issue #759): allow for reboots in bench tests.
