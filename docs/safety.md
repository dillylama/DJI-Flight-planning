# Safety case: what the planner guarantees, what it assumes, what it cannot know

Routes are written with **absolute heights**. Nothing on the aircraft looks at the ground during a
waypoint route, so every error in the terrain model, the height datum, the file or the aircraft's
own height goes straight into terrain clearance. This page states what is checked, how, and where
the checks stop.

Code: [`packages/core/src`](../packages/core/src) (`terrain.ts`, `heights.ts`, `takeoff.ts`,
`safety.ts`, `verify.ts`). Tests: [`packages/core/test`](../packages/core/test).

## 1. What is guaranteed, by construction and by an independent check

**Planner (`terrain.ts`, `heights.ts`).** Every straight leg between two waypoints is at least the
planned AGL above the highest DEM cell that touches its corridor:

- the corridor is ±`corridorM` across-track (default 75 m, minimum 30 m) and is extended beyond
  both ends of the leg by the corridor width or the stopping distance at route speed
  (v² / 2·2 m/s²), whichever is longer, so the ground around a turn is covered;
- it is sampled at 0.7 × the DEM cell size or finer, each sample taking the **highest of the four
  cells around it**, and the search reaches half a cell diagonal + half a cell past the corridor
  edge, so every cell that touches the corridor has a sample;
- a waypoint takes the higher of its two legs, plus AGL;
- a cell with no data anywhere in a corridor, on the transit or on an RTH line **stops the plan**
  (`TerrainGapError`). A gap is unknown ground, never low ground;
- approach + figure-8 + first run-in, and last run-out + figure-8 + exit, are flown level.

**Independent verifier (`verify.ts`).** Before anything can be exported, the files that would be
flown are built, parsed back from XML by separate code, converted from ellipsoidal heights back to
EGM96, and checked against the DEM with a separate lattice and separate geodesy (WGS84 ellipsoid).
`verify.ts` imports only types from the rest of the core. It refuses a file when:

| Check | Catches |
|---|---|
| leg clearance < AGL anywhere within ±95 % of the corridor, incl. 95 % of it beyond each waypoint | wrong height, wrong datum, shifted or swapped coordinates, a planner or writer bug |
| `executeHeightMode` ≠ WGS84 or template `heightMode` ≠ EGM96 | heights that would be flown relative to take-off |
| template `height` + geoid ≠ `ellipsoidHeight`, or template ≠ waylines | the two files in the KMZ describing different routes |
| climb / descent rate over the limit at the fastest speed written on or next to a leg | a steep leg flown too fast |
| `useStraightLine` ≠ 1, damping ≥ leg, turn rounding outside the corridor | a path whose distance from the legs is not bounded |
| global height below the top waypoint, global speed above the slowest waypoint | an unsafe fallback if per-waypoint values were ignored |
| drone 103 / payload 117 / namespace / turn modes / action groups / record start and stop | a file Pilot 2 or the aircraft would read differently |
| DEM gap near a leg | terrain that is not known |

The web planner exports **the exact strings that passed**, zipped. The mission package carries each
KMZ with its SHA-256.

**Tests.** `verify.test.ts` injects 50+ faults (5 m low, no geoid, geoid with the wrong sign, route
shifted 60 m or 400 m, lat/lon swapped, wrong height mode, speed 0, curved legs, missing stop
record, DEM hole, …): each is refused. Random routes over a rough synthetic 1″ raster are checked
cell by cell against the raw raster. A single-cell spike is placed on every cell around a route in
turn. A mutation run breaks the planner in 18 ways (bilinear instead of highest cell, no padding,
coarse sampling, 3 m low, no geoid conversion, PixelIsPoint read as a corner, gaps skipped, speed
cap or braking envelope removed, …): all 18 are caught by the suite.

## 2. The clearance budget

`worst case = AGL − path error at waypoints − uncertainty allowance`

- **Path error** (computed per waypoint, worst one counts):
  - *rounding*: with turn damping `d` and a change Δγ in flight-path angle, the path stays within
    `d·tan(Δγ/2)` of the legs. Damping is shortened wherever that would exceed 5 m;
  - *pull-up lag*: where the path steepens by Δvz, an aircraft that only starts at the waypoint and
    holds 2 m/s² vertically drops Δvz² / 2a below the new leg. With the 4 / 3 m/s limits this is at
    most 12 m.
- **Uncertainty allowance** (default 20 m, minimum 5 m, set by the pilot): DEM height error,
  vegetation and buildings missing from the DEM, the aircraft's own height error.

Export is refused below 30 m and warned below 50 m. AGL below 50 m is refused outright: use DJI
Real-Time Follow for that.

## 3. Speed on steep ground

`waypointSpeed` is the speed from that waypoint to the next (DJI WPML). In the default "follow
terrain" profile each leg has a top speed from the climb / descent limits (4 / 3 m/s). A waypoint's
speed is capped by its own leg, the leg before it, and the entry speed of the leg after it (braking
at 2 m/s²), so the aircraft is never faster than a leg allows whether it changes speed after
passing a waypoint or ramps towards the next one. A leg that would need less than 1 m/s is an error
(`TOO_STEEP`).

## 4. Take-off, RTH and what the file cannot carry

- Transit home → waypoint 1 is modelled for both `flyToWaylineMode`s and must clear the corridor by
  the minimum clearance.
- The **RTH height** is the lowest value that clears terrain + minimum clearance on the straight
  line home from *any point of the route* (origins at every waypoint and at most a corridor width
  apart along every leg, neighbours paired).
- The route file carries **neither the RTH height nor Max Altitude**. The planner puts both in the
  file name (`…_RTH330_ALT430.kmz`), in the mission package, in a READ-ME inside multi-sortie ZIPs,
  and makes the pilot confirm them in a dialog before any export or publish. The file tells the
  aircraft to return home when the route ends and when the RC link is lost.
- Each sortie is its own route with its own values. All checks run per sortie.
- A home point is required for export, and a new block clears the old home.

## 5. Terrain data

- GeoTIFF registration: `RasterPixelIsPoint` (GLO-30 from OpenTopography) puts the tie point at the
  centre of pixel (0,0); `PixelIsArea` at its corner. Reading one as the other shifts terrain by half
  a cell (13 m E–W, 15 m N–S at 33° S). Both are handled and tested against a real tile.
- Vertical datum: published DEMs are heights above mean sea level; a self-made DTM is often
  ellipsoidal. The pilot states which; a file that declares the opposite, or a unit other than
  metres, is refused. Heights outside −500…9000 m are refused (wrong units).
- GLO-30 is EGM2008; the route files use EGM96. The difference is usually under 1–2 m and sits in
  the uncertainty allowance.
- The demo block uses made-up terrain: nothing can be exported from it, and it is dropped as soon as
  any other block is loaded.
- Projected (UTM) and rotated GeoTIFFs are refused.

## 6. What none of this covers

1. **Obstacles that are not in the DEM**: masts, power lines, cranes, turbines, new buildings, trees
   that have grown since 2011–2015.
2. **DEM error beyond the allowance**: GLO-30 is about 4 m (LE90) on open ground and can be 10–20 m
   on steep or forested slopes.
3. **The aircraft's height without RTK**: 5–15 m or more. Fly absolute-height routes with RTK fixed.
4. **What Pilot 2 and the aircraft do with the file.** Not yet confirmed on hardware:
   whether Pilot 2 keeps per-waypoint heights, speeds and turn settings when it imports and saves;
   whether it accepts waypoint speeds above 15 m/s; how the M400 changes speed between waypoints and
   how closely it follows steep legs; how it rounds fly-through waypoints with `useStraightLine 1`;
   the waypoint cap; L3 recording from a waypoint action.
5. **A take-off from somewhere other than the planned home**, or a mission started in the air.
6. **Wind**, battery condition, airspace, GEO zones.

Items 4 and 5 are what the gates are for.

## 7. Gates (never skip one)

| Gate | What | Pass |
|---|---|---|
| G1 | Import the test files into Pilot 2; save and re-export `test-01`; `tools/compare-kmz.ts` | Heights within 0.1 m, speeds, damping, `useStraightLine`, actions unchanged; cap known |
| G2 | DJI simulator, props off: full route, RTH mid-line, RC loss | Never more than a few metres below the straight-leg profile; rates within limits; RTH at the set height; L3 records |
| G3 | Live, flat open ground, ≥ 150 m AGL, RTK fixed | Height at waypoint 1 matches the planner within ~10 m; L3 range to ground within 5 m of plan |
| G4 | Live, real slopes, planned 100 m higher than the job needs | Measured AGL never below plan − 10 m; no leg at the speed floor |
| Job | Step down only if G4 held | Obstacle survey done; never below 100 m AGL on GLO-30 |

## 8. Pre-flight, every route

1. RTH height ≥ the value in the file name; preset-height RTH mode.
2. Max Altitude ≥ the value in the file name.
3. Take off from the planned home point, from the ground.
4. RTK fixed.
5. At waypoint 1 the RC shows the planned height above take-off (± ~10 m). If not, stop.
6. The area has been checked for obstacles that are not in the DEM.
