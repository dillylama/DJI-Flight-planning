# DJI-Flight-planning — M400 / L3 LiDAR mission system (UgCS-style)

## Architecture (decided 25 Sep 2026)
Plan in the office on the web → sync mission package to the RC → fly offline from our own Android app
(MSDK v5), or inject the KMZ into Pilot 2 and fly there. No internet needed in the field.

```
packages/core   TypeScript, shared by web + RC: route engine, WPML writer/reader, DEM sampling,
                validation, sortie split, resume, mission-package format. Tested in Node.
apps/web        Office planner (Vite + TS + MapLibre) → Cloudflare Pages.
apps/api        Cloudflare Worker + D1 (projects) + R2 (mission packages, as-flown logs), RC pairing.
apps/rc         Android "3DM Fly" (Kotlin, MSDK v5 + UX SDK, WebView running core).
index.html      LEGACY single-file KMZ patcher, still served by GitHub Pages. Don't break it.
route-engine.js LEGACY JS engine — being ported to packages/core; keep until the port passes its tests.
```

Mission package (per project): plan params, AOI, lines, computed waypoints, one KMZ per sortie,
DEM clip (AOI + buffer), offline basemap tiles.

## DJI app
- Mobile SDK app "3DM Fly", package `com.threedronemapping.fly` (must match applicationId exactly).
- App key lives ONLY in `apps/rc/local.properties` (`DJI_APP_KEY=`), gitignored. Never commit it.

## Route engine behaviour (carry over into core)
- Lines from block polygon + AGL, speed, sidelap, course; spacing from L3 swath. Serpentine.
- Outer lines (`edgeLines`, default 'inside', 15 Oct 2026, as FlightHub 2): the first and last line sit half a swath
  inside the block edges and the rest are spread evenly between them (never wider than the spacing); 'over' = the
  original centred layout with the outer lines up to half a strip outside. No run-out after the LAST line unless a
  figure-8 follows (`endRunOut` for the legacy layout): the route ends on the line where recording stops.
- Run-in / run-out 80 m by default (was 150 in the figure-8 era; must exceed the coordinated-turn radius, 81 m at
  17 m/s). Without a figure-8 the approach waypoint sits only max(20 m, 2 s) before the run-in (15 Oct 2026: it used
  to sit a radius plus an alignment straight before it, up to 400 m beyond the block).
- Waypoints every ~150 m, **absolute heights** from the DEM: each waypoint ≥ the highest DEM cell touching the
  corridor of both adjacent legs (±75 m, extended beyond the leg ends, sampled finer than the DEM with the
  highest of the 4 cells around each sample) + AGL. A DEM gap anywhere is an error (`TerrainGapError`).
- **Vertical profile** (docs/ugcs-notes.md): default `slow` = follow terrain and slow steep legs so the
  climb/descent rate stays within 4 / 3 m/s. A waypoint's speed is capped by its own leg, the leg before and the
  entry speed of the leg after (braking 2 m/s²); legs needing <1 m/s = TOO_STEEP. `raise` = gradient-limited
  waypoint raising. DJI `waypointSpeed` = speed from that waypoint to the NEXT → `legSpeed(wps, i) = wps[i].speed`.
- Fly-through: `toPointAndPassWithContinuityCurvature` with **`useStraightLine 1`**; damping < adjacent leg
  length, shortened so rounding stays ≤ 5 m vertically and inside the corridor sideways.
- **Safety (docs/safety.md — read it before touching heights, terrain, takeoff, wpml or the export path):**
  `safety.ts` = floors (AGL ≥ 50 m, corridor ≥ 30 m) + clearance budget (AGL − rounding − pull-up lag −
  uncertainty; export refused < 30 m). `verify.ts` = independent check of the exported XML against the DEM; it may
  import ONLY types from the rest of core (a test enforces this). The web app exports only the exact files that
  passed, per sortie, never on demo terrain, and `run()` leaves no result when compute throws.
  After changing planner maths or its tests run `npm run mutation` (breaks the planner 21 ways; every one
  must be caught by the suite).
- **Planner UI (14 Oct 2026)**: figure-8 controls removed (DJI IMU calibration does the job on the L3; core keeps the
  code and tests). Everyday fields in view, the rest under "Advanced". Base-map selector (Esri imagery, OSM,
  OpenTopoMap, Esri topo; remembered per browser) and a place search (Nominatim, or "lat, lon") in the map toolbar.
  Sync-to-RC section collapsed as "not in use". "Resume" renamed "Continue from a line" (new landing zone).
  Start corner (`startSide` left/right of the course, `startEnd` near/far; plan.ts orders the lines and sets the
  first direction; compass names in the UI), "Zoom to block", "Clear mission", "Follow contours (auto course)" =
  the course optimiser (lines along the slope). "Draw block" (click corners, Enter/double-click to finish, Esc,
  Backspace undoes a corner) and "Edit corners" (draggable markers; the block name gets "(edited)").
  Map: direction arrows along the legs and START / END labels (first/last waypoint) in the flat view; tilting the
  map by hand switches 3D on (terrain + route at real heights via deck.gl), flattening switches it off. The style
  needs the MapLibre demo glyphs URL for text labels.
- **Figure-8** (IMU excitation, core only, off in the planner) before the first line, before any resumed segment, AND after the last line
  (vendors + DJI L2/L3 manuals: align at both ends for forward/backward trajectory processing). Recording runs
  from the approach waypoint (START_RECORD) through the end figure-8 to the exit waypoint (STOP_RECORD).
  Straight level run ≥ alignStraightS (12 s) or 2.5 r before the first 8 and after the last.
- **Waypoint thinning** (`thinToleranceM`, default 10 m; `flightRoute()` in heights.ts = applyHeights + thinning):
  line waypoints are dropped wherever the straight leg between the survivors stays within the tolerance ABOVE the
  terrain-following height at every dropped waypoint. Legs are still held above the highest ground in their corridor
  (every leg is level with its own highest ground, so on a slope the excess grows with leg length and limits thinning
  by itself). Only ever flies higher; verified like any file. Luke (14 Oct 2026): be harsh, 10–20 m of height does
  not matter at the AGL he flies. The planner profile draws the dropped waypoints dashed.
- **Operating model (Luke, 14 Oct 2026)**: a job is ONE route per landing zone, flown to the end across however many
  batteries/days it takes, with Pilot 2's breakpoint resume for every interruption; the LZ move uses "from line N
  with a new home". Pre-planned sorties are only a time/battery estimate, not an operating unit. The file cap is the
  only reason a block becomes several files (chunked by lines in the office).
- **Transit in the route** (option, off by default): `buildRoute({ transit: { home, speedMs, clearanceM } })` adds
  role `transit` waypoints home → approach and exit → home; approach and exit become stop turns. `toUgcsCsv()`
  exports any route for a second look in UgCS. docs/ugcs-notes.md has the UgCS 6.1 comparison.
- **Resume (Luke's decision, 13 Oct 2026)**: after an interruption the job resumes from Pilot 2's breakpoint, the
  way Pilot 2 does it, not by re-flying a line. The planner's "re-fly from line N" route stays as the fallback.
  Confirmed in the simulator 13 Oct 2026 (3DM Fly, SDK breakpoint resume): the aircraft flies to the breakpoint,
  re-runs waypoint 1's actions there (L3 recording restarts within 2 s; calibration passes not flown), and carries
  on from the breakpoint. docs/safety.md "Resuming an interrupted job".
- **Sorties**: split into battery-sized routes along the same path as resume, one-line overlap.
- Regression test: synthetic Nimba block: 23 lines, 902 WP, 137 km. `nimba.test.ts` compares the port with the
  legacy engine on the legacy's spherical plane (`planLines(…, 'sphere')`); production uses WGS84 radii.
- Tools: `node tools/make-import-tests.ts` (RC import tests → samples/generated, each verified),
  `node tools/compare-kmz.ts ours.kmz pilot2.kmz --dem dem.tif` (what did Pilot 2 change on re-export?),
  `node tools/compare-ftr.ts route.kmz FTR_Record_….txt --mrk photos.MRK` (flown track from Pilot 2's replay record
  on the RC, `/sdcard/DJI/com.dji.industry.pilot/FlightRecord/FlightTaskReplay/`, and the L3's RTK photo positions,
  against the route; G3 result in docs/safety.md §5a).
- PPK (proven on G3, docs/ppk-notes.md has the recipe): L3 card `.RTK` = rover RTCM3 at 5 Hz, `.RTB` = base RTCM3
  (its 1006 position, without antenna height, is what the aircraft used), `.MRK` = per-photo RTK position + offsets,
  JPG XMP `UTCAtExposure` = GPST. Tools: `trim-rinex.ts`, `rtb-base.ts`, `ppk-drone.conf` (rnx2rtkp -k; L1+L2+L5 →
  100 % fixed), `ppk-cameras.ts` (PPK − MRK check, camera CSV, recovers photos missing from the MRK). RTKLIB demo5 in
  `C:\Claude\rtklib\`. G3: PPK − RTK 0.5 cm horizontal, 2.4 cm vertical. Ground points 15 Oct 2026
  (`tools/gcp-check.ts`): Terra RTK cloud +4 cm above the surveyed ground; Terra PPK clouds +0.9 m because Terra takes
  the base position from the RINEX header (single-point) → type the RTCM 1006 position into Terra's base dialog.
  G5 (15 Oct, flown with NO corrections): our PPK 100 % fixed; Terra's header base was 10 m off; corrected cloud +7 cm
  above the ground points → PPK without a correction link is production-ready. `tools/las-shift.ts` fixes a cloud
  processed with a wrong base (rigid translation, header offsets only; use the cloud's own units: metres for UTM,
  degrees when Terra wrote WGS 84).
- G4 flown 14 Oct 2026 (valley at home, lines 240–446 m above home): file flown within −0.6…+0.3 m; the aircraft's own
  waypoint-mission limits are ≈3.7 m/s up / 2.6 m/s down (planner assumes 4 / 3: safe, costs time). Point-cloud DEM
  check: `node tools/cloud-vs-dem.ts cloud.las dem.tif route.kmz --utm 34S --every 4 --agl N` (Terra LAS 1.4 in
  UTM, ellipsoidal heights). G3: ground within −5…+5 m of GLO-30, AGL never below 107 vs 100. G4 (45° walls): ground
  −23…+12 m from GLO-30, ≤ 11.6 m above it under the route, AGL never below 213 vs 200. **All four gates passed
  14 Oct 2026**; next is a production job with the rules in docs/safety.md.

## SDK facts (see docs/msdk-research.md for sources)
- MSDK **5.18.0**; M400 since 5.15, L3 since 5.17. Kotlin 2.1, AGP 8.7, Gradle 8.12, JDK 17,
  compile/target 35, minSdk 24, arm64-v8a. RC Plus 2 = Android 11.
- L3 recording: `LidarKey.KeyPointCloudRecord` / `KeyPointCloudRecordStatus` exist; untested on L3.
- WPML: M400 `droneEnumValue` = 103 (verified). waylines.wpml heights are WGS84 ellipsoidal;
  template.kml heightMode EGM96. `recordPointCloud` action documented for M300/M350 only.
- Pilot 2 file injection is unofficial; prefer flying from our app, or Import Route / Cloud API library.

## Equipment (docs/equipment.md, packages/core/src/equipment.ts)
- M400 official: 25 m/s, ascent 10 / descent 8 m/s, wind 12 m/s; flight time only published with H30T.
  Planner uses conservative limits (~40–60 % of max) and 30 min usable per battery set with L3.
- L3 pulse rate → max AGL: 100 kHz <500 m, 350 kHz <300 m, 1000 kHz <100 m, 2000 kHz <50 m (RTF).
- DJI documents NO figure-8 for L3 (it auto-calibrates inside Area Routes); ours is optional, default on.
- Planner supports LiDAR (L3) and photogrammetry (P1 24/35/50, L3 RGB) modes.

## WPML (from Luke's Pilot 2 samples, 25 Sep 2026; details in docs/wpml.md)
- Header VERIFIED: WPML 1.0.6, M400 droneEnumValue 103/0, L3 payloadEnumValue 117/0, payloadPositionIndex 0 (default port).
- Heights: template.kml heightMode EGM96 (height + ellipsoidHeight); waylines.wpml executeHeightMode WGS84 (ellipsoidal).
  egm96-universal reproduces Pilot 2's ellipsoidHeight to 0.1 mm.
- Recording: recordPointCloud start/pause/resume/stopRecord; DJI IMU calibration = aircraftCalibration action
  (3 passes, 30 m) before startRecord and after stopRecord; L3 RGB = startContinuousShooting on multipleDistance.
- Pilot 2 forces first/last waypoint to toPointAndStopWithDiscontinuityCurvature, damping 0.
- Only verified L3 values are written: samplingRate (Hz), returnMode sedecupleReturn, scanningMode repetitive.
  Other return/scan modes need another sample. payloadSubEnumValue was 0 (waypoint) vs 1 (area): we write 0.
- Samples + generated tests hold real site coordinates → gitignored (public repo) until Luke decides.
- G1 result (13 Oct 2026, docs/wpml.md): Pilot 2 keeps positions, heights, speeds, turn modes, straight legs and
  record start/stop; it shortens executed damping to ⅓ leg ≤ 10 m, drops aircraftCalibration and the start group,
  and turns continuous shooting into takePhoto on distance. The writer now writes exactly that.
- L3 RGB photos: with model colouring ON the L3 shoots every 2.0 s and ignores the distance trigger (G3/G4, 14 Oct
  2026); with colouring OFF the `takePhoto` distance trigger is honoured exactly (PT flight, 15 Oct 2026) and Terra
  colours the cloud from those photos anyway → the planner always writes `modelColoringEnable 0`.
- Breakpoint resume proven live 15 Oct 2026 (RTH mid-line, battery swap, aircraft + RC power cycle, continue from the
  breakpoint): rejoined 6 m before the stop, no calibration passes at the rejoin, photos continuous across the break.
- The aircraft flies ~13 % under the file's waypoint speeds on average (acceleration between waypoints).
- Vertical-rate defaults 3.5 / 2.5 m/s since 15 Oct 2026: the M400's own wayline limits (G4/G5/G6). Pilot 2's
  ascent/descent sliders do not apply to waylines (G6 flown at 8 / 6: no change). Faster on steep ground = geometry:
  lines along contours, or the "keep line speed" (raise) profile.
- **Speed rule changed 15 Oct 2026 (docs/safety.md §3)**: measured on G4/PT, the M400 brakes to a lower target in
  the last ~10 m BEFORE the waypoint (≥ 4 m/s²) and accelerates to a higher one AFTER it, so on a leg it is never
  faster than that leg's own target. A waypoint speed is now capped by its own leg's limit and by what can brake to
  the next leg's entry speed within the leg at 2 m/s² (no longer by the leg before or the next entry speed directly,
  which held level legs before steep ones to the steep speed). validate.ts, verify.ts (`V_BRAKE`, `brakeMs2`
  option) and the mutation list (M8/M9) follow the same model. G3 block: 13 → 5 slowed legs, lines 9.2 → 8.1 min.
- Waypoint cap: 1,040 opened in Pilot 2 ("ok-ish"), 1,270 and 1,524 import and open but the map can no longer be panned
  or zoomed (14 Oct 2026), 2,013 crashed it → 1,000 per file stays (`M400_LIMITS.maxWaypoints`); thinning is the lever.
- Corners between lines are DJI `coordinateTurn` (M400 simulator: 6–7 m/s through a 90° corner at 10 m/s, 4–6 m
  inside it; fly-through pivoted at ~3 m/s). Pilot 2 keeps the mode and its damping on import (test-01c round trip).
- G2 passed 13 Oct 2026 in the simulator from our app (apps/rc/README.md, docs/safety.md §5a): within 2 m of
  the file, limits held, L3 start/stop, RTH mid-route. Height reference settled on the first live flight (G3, 14 Oct 2026): 186 m at
  waypoint 1 vs 186 m planned, so the aircraft reads our heights as EGM96/above take-off as we assume. Open: upload
  of ~1,000 waypoints to the aircraft.
- The route file carries neither RTH height nor Max Altitude: the planner shows both per sortie, tags the file
  name (`_RTH330_ALT430`) and makes the pilot confirm them at export.

## Phase plan
0. MSDK spike (go/no-go): M400 connect + telemetry; L3 detect + record start/stop; tiny KMZ via
   WaypointMissionManager in simulator; KMZ injection into Pilot 2's route library.
1. core: TS port of engine + DEM + validation + package format; WPML writer scaffold.
2. web planner + api (projects, profiles, validation, publish, RC pairing via QR).
3. RC app: sync/offline, pre-flight checklist, fly/monitor, auto-resume, sorties, as-flown logs.
4. Pilot 2 injection path. 5. Field validation (WP cap, simulator, short live block).

## Sync backend (apps/api) + production
- Local dev on port 8788 (8787 is SurveyDeliver); local admin token in apps/api/.dev.vars.
- PRODUCTION (25 Sep 2026): Cloudflare Pages project m400-planner (classic Pages, created with --force) serves the
  planner + API (_worker.js) at https://m400-planner.pages.dev; D1 m400-api, R2 m400-packages; ADMIN_TOKEN secret
  (value in deploy/planner/admin-token.local). Deploy: node deploy/planner/deploy.mjs. See deploy/planner/README.md.
- 3dronemapping.com DNS is at Wix → custom domain planner.3dronemapping.com needs a Wix CNAME (Luke).
- Planner dev prefill: apps/web/.env.development.local (VITE_OPENTOPO_KEY, VITE_API_URL, VITE_API_ADMIN_TOKEN).

## Sorties (packages/core/src/sorties.ts)
- Split at whole lines by usable minutes/battery set; each sortie = transit + climb + fig-8 + lines + RTH + descent,
  timed with conservative vertical rates (4 up / 3 down m/s). Home placement dominates sortie count on high blocks.

## Conventions / constraints
- RC Plus 2 browser file picker is broken — anything on the RC loads via our app, not the browser.
- FlightHub 2 is not part of this workflow.
- DJI's default ASTER/SRTM DEM causes terrain-follow pitch spikes; use Copernicus GLO-30 (OpenTopography)
  or the client's own DTM.
- UI fonts: IBM Plex Mono + Barlow Semi Condensed.
- Operator is experienced (BVLOS-rated, LiDAR specialist): explain physics precisely, flag
  corrections clearly, don't oversimplify.
- Local tooling: Node 24, git, gh, Android Studio + JDK 17 (apps/rc).
- The project lives in `C:\Claude\M400`. Generated import tests: `samples\generated\` (gitignored).

## Progress tracker
- docs/progress.html is the project timeline/gates page, published as an artifact (URL in memory). Update it and
  republish at every milestone; commit docs-only changes straight to main.
- RC-lost behaviour is a take-off option (`rcLost`: goBack = DJI RC-lost action, continue = WPML exitOnRCLost goContinue),
  planner "If the RC link is lost" in Home & aircraft. Luke plans for link loss at some landing zones (15 Oct 2026).
