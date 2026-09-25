# How UgCS plans LiDAR flights, and what we took from it (research 25 Sep 2026)

Sources: the official UgCS manual (manuals-ugcs.sphengineering.com), SPH Engineering's LiDAR guides, DJI's
WPML docs (dji-sdk/Cloud-API-Doc), the DJI L2/L3 user manuals, and LiDAR vendor field guides (Phoenix, NovAtel
Inertial Explorer, YellowScan, GeoCue). Everything below is from those documents unless marked *inferred*.

## Terrain following in UgCS
- Altitude modes: AMSL, AGL (height above the ground directly below), **Smart AGL** (constant distance to the
  nearest surface on the flight line: it raises waypoints where the ground ahead is higher), Rangefinder.
- **AGL tolerance**: extra waypoints are inserted wherever the DEM deviates from the target AGL by more than the
  tolerance. 0 = follow exactly with many waypoints. SPH recommends 3–5 m for LiDAR.
- **Max slope (%)** limits how steeply the drone may climb or descend between waypoints (a geometric cap).
- **UgCS does not reduce speed on steep segments.** The vehicle profile has max climb / descent rates, but the
  manual only documents them for take-off and landing, and the PX4 notes say the autopilot resolves vertical vs
  horizontal speed itself ("descend rapidly at 5 m/s vertical while holding ~1 m/s ground speed").
- Overshoot 20–50 m on each line end with its own **overshoot speed** (2–3 m/s suggested); corner radius 15–20 m;
  "Adaptive bank turn" cuts corners and skips waypoint actions; "Stop and turn" stops at each vertex.

## LiDAR and IMU calibration in UgCS
- LiDAR Area/Corridor tools take FOV, side overlap or side distance, AGL tolerance, turn type, overshoot, and an
  **IMU calibration** flag per segment.
- For DJI L1/L2/L3, UgCS adds DJI's own calibration segments **at the start and end of the mission** (30 m straight
  line, several times) and repeats them every 100 s (L1) or 200 s (L2/L3); any turn over 10° resets the timer.
- For other sensors it offers manual **U-figure / eight-figure** calibration patterns (width 30 m default, length,
  speed, cycles), placed near take-off before the first line and **after the last survey line**.
- "Forward passes" action mode: payload disabled in turns, i.e. recording can stop in the turns.

## Alignment at both ends (vendor guidance)
| Source | Guidance |
|---|---|
| Phoenix LiDAR | Figure-8 set before the AOI **and directly after**; then ≥ 10 s straight at ≥ 5 m/s before landing (kinematic alignment). Turn only at the perimeter, avoid extreme height changes. |
| NovAtel Inertial Explorer | "The same field procedures observed at the start of the survey should be mirrored at the end for the benefit of reverse processing." Avoid repeated tight turns at high angular rates. |
| YellowScan | U-pattern (120 m, 15 m/s) at the beginning **and the end** of the flight. |
| DJI L2 manual | "A calibration flight should be performed at both the start and end of a flight." |
| DJI L3 manual | "Perform IMU calibration before and after point cloud recording. Repeat during flight when prompted." |
| GeoCue | Heading drift grows without accelerations or heading changes; keep lines ≤ 500 m or add manoeuvres. |

## DJI WPML facts we rely on
- `waypointSpeed`: "Speed of drone flying **from current waypoint to the next waypoint**." Range (0, max speed].
- DJI (Fly article, *inferred* for Pilot 2): the aircraft accelerates/decelerates steadily between waypoints and
  reaches the preset speed **at** the waypoint; with close waypoints and high speed "the altitude of the actual flight
  route will be lower than that of the waypoints".
- `toPointAndPassWithContinuityCurvature` flies through the waypoint on a curve; damping distances must sum to
  less than the leg length.

## What the planner does with this
1. **Vertical profile = "Follow terrain, slow on climbs" (default).** Waypoints sit at AGL above the highest ground on
   their legs (corridor sampled every ≤ 30 m across-track). For every leg, vertical rate = |Δh| / d × v; if that exceeds
   the climb (4 m/s) or descent (3 m/s) limit the leg is flown at limit × d / |Δh|. Because DJI ramps speed to the next
   waypoint, both ends of a steep leg get the reduced speed, so the leg cannot exceed the limit under either reading.
   Legs that would need less than 1 m/s are flagged **too steep** (tighten waypoint spacing, raise AGL, or fly along the
   slope). The old behaviour is still available as "Raise waypoints (gradient limit)", which is what UgCS's Max slope does.
2. **Figure-8 at both ends**, recording on throughout, plus a straight level run of ≥ 12 s (≥ 2.5 radii) into the first
   figure-8 and out of the last one, then the exit point stops recording. DJI's `aircraftCalibration` is written before
   the start and after the stop as well (L3 manual).
3. **Run-in / run-out** 150 m (UgCS overshoot) at line speed; **recording continues through turns** so the
   trajectory stays continuous (unlike UgCS "forward passes"). Turns are outside the block.
4. **Overlap bands** between consecutive strips are drawn on the map with their width and percentage.

## Still to confirm on the aircraft
- Whether Pilot 2 / the M400 ramps speed as the DJI Fly article describes (watch the simulator telemetry on a slowed leg).
- How the M400 actually handles a leg whose demanded climb exceeds its capability (cuts below the waypoint height?).
