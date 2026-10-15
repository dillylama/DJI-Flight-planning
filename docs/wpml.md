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

**Corners between lines are `coordinateTurn`** (13 Oct 2026, M400 simulator): with 20 m damping on a 10 m/s route
the aircraft flew the 90° corners at 6–7 m/s, passing 4–6 m inside the corner. The fly-through mode slowed to about
3 m/s and pivoted at the corner, with 10 m or 20 m damping alike. Damping at a corner is the turn radius at
`turnBankDeg` (20°) for the speed flown in, held to a third of the shorter adjacent leg and 60 m. Pilot 2 keeps
`coordinateTurn` and its damping on import (`test-01c`, 13 Oct 2026: imported, saved, exported, no differences in
either file; the 23 m corner damping survived, where fly-through waypoints had been cut to 10 m).

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
- With terrain-following transit the route starts and ends above home, and the approach and exit waypoints
  (start and stop of recording) are written as stop turns, as the first and last waypoint are otherwise.

## Template payload parameters (L3)
`returnMode sedecupleReturn` (16 returns), `samplingRate 350000` (Hz), `scanningMode repetitive`, `modelColoringEnable 1`.

**RGB photos are on a 2 s timer, not our distance trigger (G3 and G4, 14 Oct 2026).** On both flights every photo
of both cameras came exactly 2.00 s after the previous one (G4: 257 of 259 gaps at 2.0 s), whatever the speed: 3–5 m
apart on the slowed legs, 20 m apart at 10 m/s. The `takePhoto` groups on `multipleDistance` (37.6 m on G4) left no
trace. That is the L3's model-colouring capture (`modelColoringEnable 1`), which runs on its own clock while the point
cloud records. Luke wants distance-based photos.

**Answered 15 Oct 2026: with `modelColoringEnable 0` the distance trigger is honoured.** Same block, `takePhoto` on
`multipleDistance` 18.8 m per line: 24 photos per full line per camera, exactly one every 18.8 m, none on run-ins,
turns or transit; 2.3 s apart at 8 m/s and 4.7 s on the slowed legs. So: colouring on = 2 s timer, colouring off =
our distance trigger. Whether Terra still colours the cloud from these photos is being checked in Terra.
The MRK still lacks the exposure time on about 60 % of its lines either way; the JPG XMP has every one.
Only these spellings are verified. The writer refuses other return and scan modes until a sample shows them.

## Area-route extras (for reference)
`templateType mapping2d`, `caliFlightEnable 1`, `orthoLidarOverlapW 50 / H 75`, `orthoCameraOverlapW 68 / H 75`,
`direction 157`, `shootType distance`, `efficiencyFlightModeEnable 0`.

## What Pilot 2 does to an imported route (gate G1, 13 Oct 2026)
`test-01` (132 waypoints, per-waypoint heights and speeds, straight legs, L3 actions) was imported into Pilot 2 on
the RC, saved unchanged and exported again; `tools/compare-kmz.ts` compared the two.

**Kept exactly:** every position, EGM96 height and ellipsoid height (0.000 m difference), every waypoint speed
(3.62 to 10 m/s), turn modes, `useStraightLine 1` on all fly-through waypoints, `executeHeightMode WGS84` /
`heightMode EGM96`, the L3 payload parameters, gimbal −90° + start recording on waypoint 1 and stop recording on
the last. Pilot 2 shows the heights as "ASL (EGM96)" with the ellipsoidal height beside it, the waypoint type as
"Turns before waypoint. Flies through", a speed range of 1–25 m/s, and the first and last as "Straight route.
Aircraft stops". Its terrain check result is the same as ours (102.2 m over the DEM).

**Changed by Pilot 2, and now written that way by us:**
| We wrote | Pilot 2 wrote back | Now |
|---|---|---|
| turn damping up to 60 m (40 % of the shorter leg) | executed damping = ⅓ of the shorter leg, at most 10 m (template keeps the imported value) | `dampingFrac 1/3`, `dampingMaxM 10` |
| `aircraftCalibration` before start and after stop, `caliFlightEnable 0` | removed, because the template's switch said off. With "IMU Calibration" switched on in Pilot 2 (second export): `caliFlightEnable 1` in template.kml, and in waylines.wpml `aircraftCalibration` (heading 0, 3 × 30 m) before `startRecord` on waypoint 1 and (heading 1) after `stopRecord` on the last; +84 s | flag in the template, actions in waylines only |
| `startActionGroup` (gimbal, focus) | removed | not written |
| per line: `gimbalAngleLock`, `gimbalRotate` + `startContinuousShooting` on distance, stop + unlock | one `takePhoto` group per line on `multipleDistance`, same waypoint range | `takePhoto` on `multipleDistance` |
| `useGlobalHeight/Speed/TurnParam 0` in template placemarks | flags omitted, values kept | flags omitted |
| group ids 0…n in file order | arrival groups first (0, 1), then the rest | ours stay in file order; the checker only needs them unique |

With those changes our `waylines.wpml` and the one Pilot 2 generates are the same in everything compared.
Confirmed on 13 Oct 2026 with a file from the new writer (`test-01b`, IMU calibration on): imported, saved, exported,
and `tools/compare-kmz.ts` reports no differences in either file.
Pilot 2 also shows a banner on routes that record point cloud: keep the gimbal at −90°, or ranging errors follow.

## Still to verify on the RC
1. ~~Pilot 2 imports our KMZ and shows the L3 actions.~~ Done 13 Oct 2026.
2. ~~Pilot 2 keeps the route as written.~~ Done 13 Oct 2026, see above.
3. ~~Waypoint speeds above 15 m/s.~~ Done 13 Oct 2026: `test-02` shows 15 m/s and `test-03` 17 m/s, no messages (range 1–25 m/s).
3b. ~~An export with Pilot 2's "IMU Calibration" switch on.~~ Done 13 Oct 2026, see the table above.
4. ~~The waypoint cap.~~ Done 13 Oct 2026: 268, 538 and 1,040 waypoints open in Pilot 2 (1,040 is slow to edit); 2,013 crashes it.
   14 Oct 2026: 1,270 and 1,524 import and open, but the map can no longer be panned or zoomed, so the route cannot be
   checked or edited on the RC. The usable ceiling is about 1,000; the planner's cap stays there.
   The planner cuts sorties at 1,000. Whether the aircraft accepts that many on upload is for the simulator.
5. In the simulator: the L3 starts recording on the approach waypoint, the calibration passes fly as expected,
   and the aircraft's path at fly-through waypoints stays close to the straight legs.

Samples still wanted: a waypoint route with a per-waypoint speed and height change and an action on a waypoint
(the first sample had none), and an L3 area route with terrain follow on, to see how Pilot 2 itself writes
terrain-following lines.
