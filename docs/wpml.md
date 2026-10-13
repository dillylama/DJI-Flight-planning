# DJI WPML for the M400 + Zenmuse L3: what the Pilot 2 samples show

Source: two routes exported from DJI Pilot 2 on the RC Plus 2 (latest firmware, L3 on the default port), 25 Sep 2026:
a 5-waypoint **Waypoint Route** and a small L3 **Area Route** (LiDAR mapping). The files are kept locally in
`samples/` and gitignored, because they contain real site coordinates. The writer is
[`packages/core/src/wpml.ts`](../packages/core/src/wpml.ts).

## Header (both files)
| Element | Value |
|---|---|
| namespace | `http://www.dji.com/wpmz/1.0.6` |
| `droneEnumValue` / `droneSubEnumValue` | **103 / 0** (M400) |
| `payloadEnumValue` / `payloadSubEnumValue` | **117** (Zenmuse L3) / 0 in the waypoint route, 1 in the area route |
| `payloadPositionIndex` | 0 (default port) |
| `flyToWaylineMode` | `safely` |
| `finishAction` | `goHome` |
| `exitOnRCLost` / `executeRCLostAction` | `executeLostAction` / `goBack` |
| `takeOffSecurityHeight`, `globalTransitionalSpeed` | as set (60 m / 15 m/s in the waypoint sample) |

## Heights
- `template.kml`: `heightMode` **EGM96**. Each placemark carries `height` (EGM96) and `ellipsoidHeight`.
- `waylines.wpml`: `executeHeightMode` **WGS84**, `executeHeight` = ellipsoidal height.
- At the sample site, 150 m EGM96 = 182.16 m ellipsoidal (N = +32.16 m). The `egm96-universal` grid reproduces
  Pilot 2's `ellipsoidHeight` for all 5 waypoints to 0.1 mm, so we use it.
- Our DEM (GLO-30) is EGM2008. Treating it as EGM96 costs less than about 1–2 m, which is negligible against AGL and clearance margins.
- The area route used `relativeToStartPoint`. We never write relative heights.

## Turns
Pilot 2 writes the **first and last waypoint** as `toPointAndStopWithDiscontinuityCurvature` with damping 0. The
waypoints in between use the chosen mode (`toPointAndPassWithContinuityCurvature`, damping 10 m in the sample).

The waypoint sample was saved with `useStraightLine 0`: the whole path between waypoints is then a free curve
(DJI: "the whole trajectory of the segment is curved"), and nothing in the file bounds how far it strays from
the straight legs. Pilot 2's own L3 area route flies `useStraightLine 1` with stop turns at the line ends.

**Our routes write `useStraightLine 1`** on every waypoint and in `globalUseStraightLine`: straight legs,
rounded over the turn damping distance at each fly-through waypoint (DJI requires the damping for that
combination). Terrain clearance is computed for exactly that shape. Damping per waypoint is 40 % of the shorter
adjacent leg (max 60 m), then shortened wherever the rounding would pass more than 5 m below the legs or leave
the terrain corridor sideways; the two dampings on a leg always add up to less than the leg.
No Pilot 2 sample shows `useStraightLine 1` on fly-through waypoints yet: the import test covers it.

## Per-waypoint values and the global fallbacks
Every waypoint carries its own height, speed and turn settings (`useGlobalHeight 0`, `useGlobalSpeed 0`,
`useGlobalTurnParam 0` in template.kml). The globals are written so that a reader which ignored the
per-waypoint values would still be safe: `globalHeight` is the **highest** waypoint height and
`autoFlightSpeed` the **slowest** waypoint speed. Speeds and damping are rounded down to 0.01.
`waypointSpeed` is the speed from that waypoint to the next one.

## L3 actions (from the area route's `waylines.wpml`)
| Where | Trigger | Actions |
|---|---|---|
| `startActionGroup` | route start | `gimbalRotate` −90°, `hover` 1 s, `setFocusType` auto, `hover` 0.5 s, `focus` (calibration), `setFocusType` manual, `focus` ∞, `hover` 1 s |
| first line WP | `reachPoint` | `aircraftCalibration` (heading 0, 3 times, 30 m), then `recordPointCloud startRecord` |
| each line | `betweenAdjacentPoints` | `gimbalAngleLock` |
| each line | `multipleDistance` (30.83 m) | `gimbalRotate` −90°, `startContinuousShooting` |
| line end | `reachPoint` | `recordPointCloud pauseRecord`, `stopContinuousShooting`, `gimbalAngleUnlock` |
| next line start | `reachPoint` | `recordPointCloud resumeRecord` |
| last WP | `reachPoint` | `recordPointCloud stopRecord`, `stopContinuousShooting`, `aircraftCalibration` (heading 1), `gimbalAngleUnlock` |

`actionGroupId` values are unique and increase through the file. `actionId` restarts at 0 in each group.

**Our routes:**
- Recording **starts once on the approach** (before the figure-8) and **stops once at the end**. There is no
  pause/resume in turns, so the trajectory stays continuous.
- DJI's `aircraftCalibration` is added before start and after stop (option on by default).
- L3 RGB shooting runs from the first line waypoint to the **run-out** waypoint of each line and stops there, so
  the stop action (an arrival action) is never on a data-line waypoint.
- Arrival actions on one waypoint are merged into a single `reachPoint` group, in Pilot 2's order.

## Template payload parameters (L3)
`returnMode sedecupleReturn` (16 returns), `samplingRate 350000` (Hz), `scanningMode repetitive`, `modelColoringEnable 1`.
Only these spellings are verified. The writer refuses other return and scan modes until a sample shows them.

## Area-route extras (for reference)
`templateType mapping2d`, `caliFlightEnable 1`, `orthoLidarOverlapW 50 / H 75`, `orthoCameraOverlapW 68 / H 75`,
`direction 157`, `shootType distance`, `efficiencyFlightModeEnable 0`.

## Still to verify on the RC
1. Pilot 2 imports our KMZ (`samples/generated/test-01-small-l3-survey.kmz`) and shows the L3 actions.
2. Pilot 2 keeps the route as written: import `test-01`, save, export it again, then
   `node tools/compare-kmz.ts samples/generated/test-01-small-l3-survey.kmz <re-export>.kmz --dem samples/generated/dem.tif`.
3. Waypoint speeds above 15 m/s (`test-02` at 15, `test-03` at 17 m/s): accepted, or cut to 15?
4. The waypoint cap (`test-cap-0250` … `test-cap-5000`).
5. In the simulator: the L3 starts recording on the approach waypoint, the calibration passes fly as expected,
   and the aircraft's path at fly-through waypoints stays close to the straight legs.

Samples still wanted: a waypoint route with a per-waypoint speed and height change and an action on a waypoint
(the first sample had none), and an L3 area route with terrain follow on, to see how Pilot 2 itself writes
terrain-following lines.
