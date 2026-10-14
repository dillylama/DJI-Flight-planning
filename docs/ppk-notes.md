# PPK for the L3 (post-processed kinematic) — investigation notes

Luke, 14 Oct 2026: "I think this means I can now PPK the flight? This was a huge problem before when using Metashape
as I would have to fly in RTK, which is not always possible. Post-processing is a much better option." Status: open,
findings so far below. Nothing here has been run end to end yet.

## What the L3 wrote on the card (G3 flight, 14 Oct 2026, `DCIM/DJI_<date>_008_<route name>/`)

| File | Size | What it is (verified by reading the bytes) |
|---|---|---|
| `…_L.RTK` | 3.1 MB | **Rover GNSS raw data as an RTCM3 stream**: 16,177 frames, every CRC valid. MSM5 observations for GPS (1075), GLONASS (1085), Galileo (1095) and BeiDou (1125) at 5 Hz, ephemerides (1019, 1020, 1046) and a proprietary 4011 message once per second. This is the PPK rover file. |
| `…_L.RTB` | 0.3 MB | **Base corrections the aircraft received, RTCM3**: 2,442 frames, all valid. Station position 1006 and antenna 1033 every 10 s, MSM4 observations (1074, 1084, 1094, 1124) at 1 Hz. On the G3 flight the base was the network RTK service; with a D-RTK base this file would be its stream. |
| `…_L.RTL` | 0.5 MB | Not RTCM (no valid frames). Unknown; probably DJI's own log. |
| `…_Timestamp.MRK` | 56 KB | One line per photo (left and right camera, `0003_LV`, `0003_RV`): GPS time of exposure, antenna-to-camera offset (N, E, V in mm), the RTK position and ellipsoidal height at exposure, standard deviations, and fix quality (`50,Q` = RTK fixed). Same format DJI uses for P1/L1/L2 PPK. |
| `…_L.IMU`, `.RPOS`, `.LDR`, `.LDRT`, `.DBG` | 4 MB … 3.9 GB | IMU, real-time positions, raw and real-time LiDAR frames, debug. Proprietary; DJI Terra only. |
| `…_L.CLI`, `.CLC` | <1 KB | Factory calibration: LiDAR-to-IMU and LiDAR-to-camera (two RGB cameras, serials listed). |
| `…_D.RPT` | 140 KB | Mission report JSON (binary header + trailer): 410 photos all RTK fixed, 409 of 410 recording seconds RTK fixed, no RTB loss, no wind warnings. |
| `…_L.SIG` | 3 KB | A signature block titled `PPK_SERVICE_TYPE` listing the mission files. DJI signs the raw data for its own PPK service (DJI Terra). |

Conclusion so far: the aircraft **always** records what PPK needs, whether or not it had a fix in flight. The MRK on the
G3 flight is RTK-fixed because the network RTK worked; on a flight without corrections the MRK would carry single-point
positions and the `.RTK` file would still hold the raw observations.

## Two separate PPK problems

1. **The LiDAR trajectory** (point cloud). DJI Terra processes L3 data and accepts a base-station RINEX observation file
   for PPK (DJI's PPK workflow for L1/L2: put the base `.obs` or RINEX `.yyO` file in the mission folder, Terra then
   post-processes the trajectory instead of trusting the in-flight RTK). The L3 is only processed in Terra, so this is
   the only route for the point cloud. To test on the G3 data: a base RINEX covering 13:40–13:48 SAST on 14 Oct 2026
   is needed (own base, or a TrigNet CORS station near Prince Albert).
2. **The RGB photos** (Metashape). Metashape does not read the `.RTK` file. The route is: convert `.RTK` (RTCM3) to
   RINEX with RTKLIB `convbin` (`-r rtcm3`, and give it the date, since RTCM carries only time of week), post-process
   against the base RINEX with `rnx2rtkp` (or Emlid Studio) into a `.pos` trajectory, interpolate each MRK exposure
   time on that trajectory, apply the MRK antenna offsets, and write a camera-position CSV for Metashape. Forum users
   report exactly this for L1/L2 (rename `.RTK`/`PPKRAW.bin` to `.RTCM3` for Emlid Studio). Nothing DJI-specific in the
   way; the one check is the mapping of the MRK time stamps (GPS seconds + week) to the RINEX epochs.

## Base data for the G3 flight

Luke's Emlid Reach RS4 was logging at the time (it records continuously, 12-hour files). Flight window 13:39–13:50 SAST
= 11:39–11:50 UTC on 14 Oct 2026; trim the base log to 11:30–11:55 UTC with `node tools/trim-rinex.ts` (RINEX 2 or
3 observation files; navigation files stay whole). Files go in `samples/g3/base/` (local, gitignored) with a copy in
`SynologyDrive\3dronemapping\Drones\M400\Flights\2026-10-14 G3 home block\base\` (NAS + C2 backup).

Tools on this PC: Emlid Studio is installed (`C:\Program Files\Emlid Studio`) but its converter and post-processing
programs are GUI back-end services, not command-line tools. For a scripted pipeline RTKLIB's `convbin` and `rnx2rtkp`
(rtklibexplorer demo5 build) are needed; not installed yet.

## To do (in order)

- [x] Base data: RS4 log for 14 Oct 2026 (Luke, in progress).
- [ ] Trim the base log to the flight window; convert to RINEX if the RS4 wrote UBX/raw.
- [ ] `convbin` the `.RTK` to RINEX; check epoch count (expect ~5 Hz × 563 s) and the four constellations.
- [ ] `rnx2rtkp` kinematic solution; compare its positions at the MRK times with the MRK's own RTK positions (should
      agree to a few cm on a flight that was RTK-fixed throughout, which is the acceptance test for the pipeline).
- [ ] Small tool `tools/ppk-cameras.ts`: `.pos` + `.MRK` → camera CSV (lat, lon, ellipsoidal height, accuracy) for
      Metashape, with the antenna offsets applied.
- [ ] Try the Terra PPK route for the point cloud with the same base file.
- [ ] Then decide the field rule: fly RTK when available, but PPK is the fallback and the primary for photos.

## Sources (third-party, to be confirmed on our data)

- Emlid community: [PPK on DJI Matrice 350 with Zenmuse L2](https://community.emlid.com/t/ppk-on-dji-matrice-350-with-zenmuse-l2/41543),
  [DJI L2 PPK processing](https://community.emlid.com/t/dji-l2-ppk-processing/38965),
  [RTK and PPK using M300 & L1](https://community.emlid.com/t/rtk-and-ppk-using-m300-l1/36709),
  [What is a RINEX navigation file and an MRK file](https://community.emlid.com/t/what-is-a-rinex-navigation-file-and-an-mrk-file/31289)
- DJI Enterprise Insights: [PPK workflow for DJI Enterprise drones](https://enterprise-insights.dji.com/blog/ppk-post-processed-kinematics-workflow),
  [Geospatial solutions FAQ](https://enterprise-insights.dji.com/de/blog/geospatial-solutions-faq)
- LP360: [DJI PPK utility](https://support.lp360.com/hc/en-us/articles/38545494922515)
- DJI forum: [Terra file naming for L1 PPK](https://forum.dji.com/thread-272718-1-1.html)
