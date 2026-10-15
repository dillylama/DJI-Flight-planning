# PPK for the L3 (post-processed kinematic): proven on G3, 14 Oct 2026

Luke: "I think this means I can now PPK the flight? This was a huge problem before when using Metashape as I would
have to fly in RTK, which is not always possible. Post-processing is a much better option." Answer: yes. The L3 always
records what PPK needs, and a scripted RTKLIB pipeline reproduces the aircraft's RTK positions to the centimetre.

## What the L3 writes on the card (`DCIM/DJI_<date>_<n>_<route name>/`)

| File | What it is (verified by reading the bytes) |
|---|---|
| `…_L.RTK` | **Rover GNSS raw data, RTCM3**: MSM5 observations for GPS (1075), GLONASS (1085), Galileo (1095), BeiDou (1125) at 5 Hz, ephemerides (1019, 1020, 1046), a proprietary 4011 once per second. The PPK rover file. |
| `…_L.RTB` | **Base corrections the aircraft received, RTCM3**: 1006 station position and 1033 antenna every 10 s, MSM4 (1074, 1084, 1094, 1124) at 1 Hz. Carries the base position the aircraft used in flight. |
| `…_Timestamp.MRK` | One line per photo (`0003_LV`, `0003_RV`: left and right camera): GPS seconds of week + week, antenna→camera offset N/E/V in mm, RTK position and ellipsoidal height at exposure, standard deviations, fix quality (`50,Q` = fixed). |
| `…_D.RPT` | Mission report JSON (binary header + trailer): photo count and RTK state, recording seconds fixed, base loss, wind warnings. |
| `…_L.CLI`, `.CLC` | Factory calibration: LiDAR-to-IMU and LiDAR-to-camera (both RGB cameras). |
| `…_L.SIG` | Signature block `PPK_SERVICE_TYPE` listing the mission files (DJI signs the raw data for its own PPK service). |
| `…_L.RTL`, `.IMU`, `.RPOS`, `.LDR`, `.LDRT`, `.DBG` | Proprietary; DJI Terra only. |
| JPG / DNG | XMP `drone-dji:` block with `UTCAtExposure` to the microsecond (**GPST, despite the name**: it equals the MRK time exactly, verified against the flight record), RTK position, flight yaw/pitch/roll, RtkFlag, RtkStd. |

Findings on the G3 data:
- **Base position.** The base RINEX header carries the receiver's single-point position (about 1 m off). The aircraft
  uses the RTCM 1006 position from the `.RTB` **as broadcast, without adding the antenna height**: PPK − RTK heights
  were −1.7 cm that way and +7.8 cm with the 0.095 m height added. `node tools/rtb-base.ts file.RTB` prints the value
  for `rnx2rtkp -r`.
- **MRK offsets.** `Lat/Lon/Ellh` in the MRK are camera positions. PPK antenna position + N/E/V offsets (V positive =
  camera below the antenna) reproduces them to 0.5 cm horizontally; without the offsets the error is 50 cm.
- **Missing MRK times.** 143 of 410 MRK lines had no exposure time (`-259200`) and zero offsets, both cameras, no
  pattern. Every JPG carries the exposure time in its XMP, and the offsets follow from a body-frame lever arm (fitted on
  the lines that have them, rotated by the photo's attitude; LV 582/−413/238 mm, RV 583/−356/238 mm fwd/right/down,
  residuals ≤ 2 cm). `tools/ppk-cameras.ts --photos <dir>` does both, so every photo gets a PPK position.
- **Fix rate.** Default RTKLIB options: 8–11 % fixed. The demo5 configuration in `tools/ppk-drone.conf`: L1+L2 60 %,
  elevation mask 20° 93 %, **L1+L2+L5 100 %** of 2,817 epochs. In the combined solution even the float epochs agreed with
  the RTK to 1 cm horizontally and 4 cm vertically.

## Result (G3, 14 Oct 2026, RS4 base about 1.5 km from the block, 5 Hz, RINEX 3.03 from Emlid Flow)

PPK (L1+L2+L5, fixed throughout) minus the aircraft's RTK at the 267 photos with MRK offsets:

| | mean | rms | spread |
|---|---|---|---|
| north | −0.1 cm | 0.5 cm | −1.4 … +0.9 cm |
| east | +0.1 cm | 0.4 cm | −0.9 … +0.9 cm |
| up | −2.3 cm | 2.4 cm | −4.3 … −0.6 cm |

The 2 cm vertical offset is constant and most likely the base antenna model (ADVNULLANTENNA broadcast) versus RTKLIB's
handling; to be checked against a ground control point before PPK heights are used in anger.

**G4 (same day, 14:57–15:08, lines 240–446 m above the base):** 3,355 epochs, 100 % fixed with the same configuration;
518 photos, 180 recovered from JPG metadata (lever arms within 1 mm of the G3 fit). PPK − RTK at the 338 photos with MRK
offsets: north −0.4 cm (rms 0.5), east +0.2 cm (rms 0.5), **up −5.4 cm (rms 5.5, spread −7.8 … −2.7)**. The vertical
offset grew with the height above the base (G3 flew ~190 m above it, G4 up to 450 m), which points at tropospheric
modelling (RTKLIB Saastamoinen vs the aircraft's RTK engine) rather than an antenna constant. Either way it is a
few centimetres and systematic; the GCP check decides who is right. The RPT noted 34 s of lost base corrections
and one stage-1 wind warning; neither affected the fix.

## Recipe (what `tools/` does; run from the repo root)

1. Base: Emlid Flow RINEX export. Trim the 12-hour log to the flight window plus 5 min:
   `node tools/trim-rinex.ts base.26O base-trim.26O --from "2026-10-14 11:30:00" --to "2026-10-14 11:55:00"` (UTC).
2. Rover: `convbin.exe -r rtcm3 -tr 2026/10/14 11:30:00 -v 3.04 -od -os -oi -ot -ol -o rover.obs -n rover.nav rover.rtcm3`
   (copy the `.RTK` next to the output as `rover.rtcm3`; RTKLIB opens no `..\` paths). RTKLIB demo5 2.5.1 (rtklibexplorer)
   is in `C:\Claude\rtklib\RTKLIB_EX_2.5.1\`.
3. Base position: `node tools/rtb-base.ts file.RTB` → `x y z`.
4. Solution: `rnx2rtkp.exe -k tools/ppk-drone.conf -r x y z -o flight.pos rover.obs base-trim.26O rover.nav base.26P`
   (all files in one folder; the conf is combined forward+backward, kinematic, fix-and-hold, L1+L2+L5).
5. Cameras: `node tools/ppk-cameras.ts flight.pos photos.MRK --photos <JPG folder> --out cameras.csv`
   → CSV `photo, gpst_s, lat, lon, ellipsoidal_height, q, sd_n, sd_e, sd_u, offset_source, …` for Metashape
   (WGS84 ellipsoidal heights; convert to the project datum in Metashape).
6. Point cloud: DJI Terra with the base RINEX for a PPK trajectory (not yet tried; DJI's L1/L2 workflow puts the base
   `.obs` in the mission folder).

## Ground points, 15 Oct 2026: Terra's PPK takes the base position from the RINEX header, and that is 0.85 m off

Nine points surveyed with the RS4 rover (RTK fixed, 1–2 cm) on bare ground on the G3 block, checked against the
clouds with `node tools/gcp-check.ts` (cloud points within 1 m of each point):

| Cloud | Trajectory | Cloud lowest − point | Cloud median − point |
|---|---|---|---|
| G3 (14 Oct) | Terra, aircraft RTK | +4.0 cm (1.9…6.6) | +8.2 cm |
| PT cloud0 (15 Oct) | Terra PPK against the RS4 log | **+87.5 cm** (86.0…89.3) | +92.4 cm |
| G4 (14 Oct) | Terra PPK against the RS4 log | **+99.4 cm** (96.2…101.9) | +104.8 cm |

The RTK cloud sits 4 cm above the pole tips: as good as it gets. Both PPK clouds sit about 0.9 m too high, by a
constant. Cause, from Terra's own `processing/intermediate/ppk_list.json`: the base position it used for the PT run
is `4947334.3381 2008940.7522 -3477960.3590`, which is the **APPROX POSITION line of the RS4's RINEX header**, the
receiver's single-point fix. Against the true reference point (RTCM 1006, what the aircraft used in flight) that is
0.33 m west, 0.14 m north and **0.85 m up**. Every point of the cloud moves with it. GLO-30 sits 2.1 m above these
points (1.6…2.6), consistent with the earlier cloud-vs-DEM figures.

**Fix:** in Terra's base-station dialog type the 1006 position (`node tools/rtb-base.ts file.RTB --geodetic`):
for the home base that is −33.25422975, 22.10036736, 708.561 m ellipsoidal, antenna height 0 (the 1006 position
already is the phase-centre reference the aircraft used). Then reprocess. Our own RTKLIB chain uses that position
and agreed with the aircraft's RTK to 2–5 cm, so the same should follow in Terra. The same base coordinates must be
used for the RS4 rover survey (they were: the rover worked against the base's broadcast position).

No reprocessing is needed to repair a cloud that was processed with the wrong base: the error is a rigid translation
by the base error, so `node tools/las-shift.ts in.las out.las --east … --north … --up …` (header offsets only, exact,
seconds) puts it right. Done for both: PT cloud0 shifted by (+0.327, −0.143, −0.847) → **+2.7 cm** lowest / +7.4 cm
median above the points; G4 cloud0 (Terra's base there was 1.36 m south and 0.96 m up of the true point) shifted by
(+0.039, +1.360, −0.958) → **+5.6 cm** / +10.9 cm. Same as the RTK cloud. For a job, type the base position into Terra
once and nothing needs shifting.

## No-RTK flight (G5, 15 Oct 2026): PPK is production-ready

The valley block was flown with corrections switched off (the aircraft on single GPS, MRK fix quality 0/16). Our
chain: 3,028 epochs, 100 % fixed. Terra's PPK of the same data took the base position from the RINEX header, which on
a no-RTK day was **10.2 m high and 5 m off** (single-point fix); corrected by that vector with `las-shift`, the cloud
sits **+7.3 cm** (lowest) / +11.8 cm (median) above the nine surveyed points, the same as the RTK and corrected-PPK
clouds of the other flights. So a job can be flown without any correction link and post-processed, provided the base
position is typed into Terra (and the rover survey uses the same base position). Note Terra wrote this cloud in
geographic degrees (WGS 84), not UTM: `cloud-vs-dem` and `gcp-check` handle both, `las-shift` takes the shift in the
cloud's own units.

## To do

- [ ] Field rule: fly RTK when corrections exist, log the RS4 always; PPK the photos as standard, point cloud via Terra PPK.
- [ ] Check the 2 cm vertical offset against a surveyed point.
- [ ] Try the Terra PPK route for the point cloud with the same base file (G3 and G4 data).
- [ ] Run the recipe on a flight without RTK corrections (the real use case) and compare with a Terra PPK trajectory.
- [ ] RTKLIB and the recipe belong in a small script (`tools/ppk.ps1`) once the Terra step is settled.

## Sources (third-party)

- Emlid community: [PPK on DJI Matrice 350 with Zenmuse L2](https://community.emlid.com/t/ppk-on-dji-matrice-350-with-zenmuse-l2/41543),
  [DJI L2 PPK processing](https://community.emlid.com/t/dji-l2-ppk-processing/38965),
  [RTK and PPK using M300 & L1](https://community.emlid.com/t/rtk-and-ppk-using-m300-l1/36709)
- DJI Enterprise Insights: [PPK workflow for DJI Enterprise drones](https://enterprise-insights.dji.com/blog/ppk-post-processed-kinematics-workflow)
- LP360: [DJI PPK utility](https://support.lp360.com/hc/en-us/articles/38545494922515)
- RTKLIB demo5: [rtklibexplorer/RTKLIB releases](https://github.com/rtklibexplorer/RTKLIB/releases)
