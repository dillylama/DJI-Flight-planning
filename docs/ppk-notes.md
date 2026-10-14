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
