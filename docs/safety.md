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

### Terrain-following transit (option, off by default)

With "Fly to route: follow terrain" the way from home to the approach point and from the exit point back to
home become waypoints of the route (one every `wpSpacingM`). They get their heights from the same corridor
search as every other leg, at the higher of the AGL and the minimum clearance, and they are checked by the same
file verifier. The route then starts and ends directly above home. The approach and exit waypoints, where
recording starts and stops, become **stop turns**: that is where Pilot 2's own L3 routes put those actions.

Use it when ground between home and the block is higher than the block, where the aircraft's own level
"fly to route" leg would need a very high take-off security height.

Not yet tried on hardware: what the M400 does when a route ends above the home point and `finishAction` is
`goHome` (expected: it lands), and how it behaves at stop turns in the middle of a route. Simulator first.
RTH is unchanged: a straight line at the RTH height, checked from every point of the route including the transit.

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

## 5a. What the simulator showed (13 Oct 2026, gate G2)

The M400 flew `test-01b` (132 waypoints, lowered by a constant for the simulator) from our own app in DJI's
simulator, props off, with the flight data logged twice a second:

- every waypoint passed within 2.3 m; height within 2.1 m of the file at every waypoint and never more than 2.2 m
  below a straight leg (budgeted: 5 m);
- slowed legs flown at or just under the file's speeds; climb up to 4.4 m/s for a moment and 3.5 m/s sustained,
  descent up to 3.2 m/s, against the 4 / 3 m/s the plan assumes;
- line ends: with the fly-through mode the aircraft slowed to about 3 m/s and pivoted at the corner, whatever the
  damping; with DJI's `coordinateTurn` at the corners (now the default) it flew them at 6–7 m/s on a 10 m/s route,
  passing 4–6 m inside the corner with 20 m damping. The rounded turn stays inside the terrain corridor (checked);
- recording starts after the calibration passes on waypoint 1 and stops on the last waypoint; the aircraft
  refuses a route above its Max Altitude before take-off; it returns home when the route ends, and when RTH is
  pressed it ends the route, stops the recording itself and flies home at its current height (above the RTH setting).

**Height reference, settled on the first live flight (14 Oct 2026).** In the simulator the aircraft had held our
heights as ellipsoidal in two runs and as EGM96 in one (32 m apart, same file). On the live flight at the home
block (G3) the RC showed 186 m above take-off at waypoint 1 against the planner's 186 m, which assumes the EGM96
reading. So in flight the aircraft converts our ellipsoidal `executeHeight` to height above the take-off point the
same way we and Pilot 2 do; the two simulator runs that said otherwise were the simulator mixing a made-up position
with real receiver data. The waypoint-1 check stays in the pre-flight list; it costs nothing.

**G3 flight record against the route (14 Oct 2026, `node tools/compare-ftr.ts`).** Pilot 2 keeps a 5 Hz replay record
on the RC (`FlightRecord/FlightTaskReplay/FTR_Record_<id>/…txt`, gzip JSON with position, ellipsoidal altitude and
attitude). Route G3-flat-block: 48 waypoints, 100 m AGL, 8 m/s, 6 lines, mission 8.5 min from start to finish.
Flown route against the file, all 48 waypoints passed:

| Check | Result |
|---|---|
| Height at the waypoints (aircraft − file, ellipsoidal) | mean −0.2 m, worst −2.6 m (wp 33, the cut corner into the climb) … +0.3 m |
| Height between waypoints relative to the straight leg | never more than 2.6 m below |
| Pass distance at pass-through waypoints | 0.0–1.1 m |
| Pass distance at coordinateTurn corners | 3.3–9.6 m inside the corner, at 3.6–6.8 m/s (corridor ±40 m) |
| Ground speed on 8 m/s legs | 7.2–8.9 m/s; slowed legs at the file speed ±0.5 m/s |
| Vertical rate on the lines | up to 3.8 m/s climb, 3.2 m/s descent (planner assumes 4 / 3) |
| Calibration passes | wp 1: 25 s at 4.5 m/s level, height held ±0.2 m; last wp: three 30 m passes at 6–8 m/s |
| Route end | mission finished → RTH 1 s later → landed 84 s later (Pilot 2 triggers 43, 30, 31) |
| L3 RGB photo positions (MRK, RTK fixed 410/410) | antenna height − route leg: mean −0.1 m, spread −2.0 … +0.4 m |
| L3 mission report (`_D.RPT`) | 409 of 410 recording seconds RTK fixed, no base loss, no wind warning |

So a planner file is flown within about 1 m vertically and 1 m laterally on the lines, corners cut by ≤ 10 m, and
the two independent height sources (flight controller record, L3 RTK antenna) agree with the file. The planner's
AGL figure is therefore the height the aircraft flies; what remains for G4 is the DEM side (how far the real ground
is from GLO-30 on slopes), which the point cloud will show once processed in DJI Terra.

**G4 flight, aircraft side (14 Oct 2026, same afternoon).** Block 550 × 250 m straddling the valley at home, wall to
wall (ground −4 … +223 m against home), 3 east–west lines up one wall and down the other, planned 200 m AGL at 10 m/s,
80 m run-ins, 46 waypoints, L3 at 100 kHz; the planner held the lines 240–446 m above home. Max Altitude 480, RTH 150.
Flight record against the file: all 46 waypoints passed, heights −0.6 … +0.3 m from the file, never more than 1.6 m
below a leg, pass distance ≤ 0.7 m on the lines and 6 m inside the coordinated corners; waypoint 1 read 448 m against
446 planned. Mission 10.3 min. On the 33 slowed legs the aircraft flew **slower than the file**, not faster: climbs of
3.1–3.4 m/s at 4.0–4.4 m/s horizontal where the file said 5.1, descents of 2.0–2.4 m/s at 1.7–3.4 m/s where the file
said 2.6–4.0. Its own waypoint-mission limits are therefore about 3.7 m/s up and 2.6 m/s down, below the 4 / 3 the
planner assumes, so the planner's slowed legs are on the safe side of the aircraft's behaviour and the only effect is
time (the G4 lines took 10.3 min against 7.5 planned). Candidate change: planner defaults climb 3.5, descent 2.5 m/s
for honest timing. The DEM side of G4 (real ground vs GLO-30 on the walls) waits for the point cloud from DJI Terra.

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
| (any time) | Optional second opinion: export the UgCS CSV and look at the route over UgCS's own terrain |  Clearance looks the same in software that shares nothing with this planner |
| G1 | Import the test files into Pilot 2; save and re-export `test-01`; `tools/compare-kmz.ts` | Heights within 0.1 m, speeds, damping, `useStraightLine`, actions unchanged; cap known |
| G2 | DJI simulator, props off: full route, RTH mid-line | **Passed 13 Oct 2026** (section 5a). RC loss not simulated. |
| G3 | Live, open ground, RTK fixed | **Passed 14 Oct 2026** at the home block (100 m AGL, 6 lines): height at waypoint 1 186 m vs 186 m planned; flight record within −2.6…+0.3 m of the file at all 48 waypoints, L3 RTK photo heights within −2.0…+0.4 m of the route; L3 recorded, calibration passes flown, no messages. Point cloud still to be processed (DEM check, not an aircraft check). |
| G4 | Live, real slopes, planned 100 m higher than the job needs | **Flown 14 Oct 2026** across the home valley (ground −4…+223 m, lines 240–446 m above home): flight record within −0.6…+0.3 m of the file at all 46 waypoints, slowest leg flown 1.7 m/s (file 2.6), no leg at the floor. **Pending:** measured AGL from the point cloud never below plan − 10 m (DJI Terra). |
| Job | Step down only if G4 held | Obstacle survey done; never below 100 m AGL on GLO-30 |

## 8. Pre-flight, every route

1. RTH height ≥ the value in the file name; preset-height RTH mode.
2. Max Altitude ≥ the value in the file name.
3. Take off from the planned home point, from the ground.
4. RTK fixed.
5. At waypoint 1 the RC shows the planned height above take-off (± ~10 m). If not, stop.
6. The area has been checked for obstacles that are not in the DEM.
