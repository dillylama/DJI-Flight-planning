# apps/rc — "3DM Fly" (RC Plus 2 Enterprise)

Android, Kotlin, DJI Mobile SDK v5 + UX SDK. Package `com.threedronemapping.fly`.
The app key is read from `local.properties` (`DJI_APP_KEY=`), which is gitignored.

Planned:
- **Sync** mission packages from `apps/api`; everything cached for offline use.
- **Pre-flight checklist**: RTK fix, IMU/compass, L3 ready, SD space, battery vs sortie, RTH height vs
  max DEM terrain on the route.
- **Fly**: KMZ → `WaypointMissionManager` upload, start / pause / resume / stop.
- **Monitor**: current line/waypoint, AGL vs DEM, L3 recording state (alarm if off on a data line),
  time-to-go vs battery and RTH reserve.
- **Auto-resume**: pick up from the last line flown with recording on at a new speed, with a fresh figure-8.
- **Pilot 2 path**: write the KMZ into Pilot 2's route library, or save it for "Import route".
- **As-flown logs** back to the office.

Phase 0 (go/no-go test build) comes first: M400 connect + telemetry, L3 record start/stop,
tiny KMZ in the simulator, Pilot 2 injection.
