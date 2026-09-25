# samples/: RC exports that the KMZ writer is built from

Our route files have to match exactly what **DJI Pilot 2 on your RC Plus 2** writes for the **M400 + Zenmuse L3**.
DJI doesn't publish the L3 values for route files, and a wrong value means Pilot 2 rejects the route or the L3
doesn't record. So we copy them from real exports instead of guessing.

**These routes are only for export. You don't need to fly them.** Build them anywhere (the office is fine),
with the aircraft powered on and the L3 fitted so Pilot 2 offers the L3 actions.

Menu names below are from Pilot 2 in general and may differ slightly on your version. Use the closest match
and write down what you chose (see `notes.txt`).

---

## Sample 1: L3 waypoint reference (essential)

**File name:** `m400-l3-waypoint-reference.kmz`

1. Pilot 2 → **Flight Route** → **Create Route** → **Waypoint Route**.
2. Aircraft **M400**, payload **Zenmuse L3**. Note the gimbal port the L3 is on.
3. **Route (global) settings.** Change these on purpose so every field shows up in the file:

   | Setting | Set it to | What it tells us |
   |---|---|---|
   | Altitude mode | **ASL (EGM96)** / absolute. If it isn't offered, use "Relative to take-off" and note it | the height datum and whether we apply the geoid |
   | Route altitude | **150 m** | global height element |
   | Speed | **10 m/s** | global speed element |
   | Take-off security height | **60 m** | `takeOffSecurityHeight` |
   | Fly to first waypoint | whichever is offered, and note which (e.g. Safely) | `flyToWaylineMode` |
   | Aircraft heading | **Along route** | heading mode |
   | Waypoint type / turn | **Curved route, aircraft continues** (fly-through) | turn mode + damping distance |
   | Gimbal pitch | **−90°** if there is a global setting | gimbal element |
   | Upon completion | **Return to home** | finish action |
   | Signal lost | **Execute lost action → RTH** | lost-link element |

4. Add **4 waypoints** about 200 m apart (a straight line or an L shape):

   | WP | Change | What it tells us |
   |---|---|---|
   | **1** | Action **Start point cloud recording** (the L3 action). If it offers options (pulse rate, scan mode, returns), set **100 kHz, Linear, 3 returns** and note them | the exact L3 start-record action XML ← the key item |
   | **2** | Speed **12 m/s**, altitude **160 m** (different from global) | per-waypoint speed and height |
   | **3** | Action **Take photo** | a plain payload action, for comparison |
   | **4** | Action **Stop point cloud recording** | the stop-record action XML |

5. **Save**, then in the route library select the route → **⋯ / Share → Export** (KMZ).

**What it unlocks:** the M400/L3 file header (`droneEnumValue`, `payloadEnumValue`, payload position), the
start/stop recording actions, the height mode, and the turn/speed/finish/lost-link elements. With this, "Export KMZ"
in the planner goes live.

---

## Sample 2: L3 area route (very useful)

**File name:** `m400-l3-area-reference.kmz`

1. Pilot 2 → **Create Route** → **Area Route** (Mapping) with **Zenmuse L3**, LiDAR mapping.
2. Draw a small block, about **200 × 200 m**.
3. Set: altitude **150 m**, speed **10 m/s**, **LiDAR side overlap 50 %**, **Calibrate IMU ON**, elevation
   optimisation ON, and pulse rate / scan mode / returns as you normally fly them (note them).
4. Save → Export.

**What it unlocks:** DJI's own L3 values for pulse rate, scan mode and return mode, how Pilot 2 names the L3
payload, and how it encodes its automatic IMU-calibration segments. That lets us cross-check the header from Sample 1.

---

## Sample 3: P1 waypoint route (optional, only if you fly a P1)

**File name:** `m400-p1-waypoint-reference.kmz`. The same 4 waypoints as Sample 1 with **Zenmuse P1** fitted:
WP1 **Start interval shooting** (distance or time, and note which), WP4 **Stop interval shooting**.
This unlocks the photogrammetry export.

---

## `notes.txt` (please include)

A short text file with:

- **Versions:** Pilot 2 app version, RC Plus 2 firmware, M400 firmware, L3 firmware (Pilot 2 → About / HMS → Firmware).
- The exact wording of any option above that differed, or that you couldn't set.
- Which gimbal port the L3 is on, and whether RTK was connected.
- Optional: phone photos of the route-settings screens. They help match Pilot 2's labels to the XML.

---

## Getting the files onto this PC

1. Export from Pilot 2 as above. Pilot 2 saves to the RC's internal storage (often a `DJI/…/export` or `Download` folder;
   the export dialog shows the path).
2. Connect the RC to the PC by USB-C → on the RC choose **File transfer** → copy the `.kmz` files.
3. Put them in `C:\Claude\M400\samples\` with the names above, and tell me.
   Or email or Dropbox them and I'll pick them up.

## What happens next

1. I read the samples and build the KMZ writer from them: `template.kml` + `waylines.wpml`, written consistently.
2. I generate a **round-trip test**: the planner writes your Sample 1 route, and it must match Pilot 2's file field for field.
3. I generate a **large-route import test** (e.g. 300, 600, 1,000 waypoints) for you to import on the RC. That finds
   the real waypoint cap, which then goes into "Max WPs per route" and the sortie split.
