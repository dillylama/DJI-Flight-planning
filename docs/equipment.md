# Equipment specs and planning limits (checked 25 Sep 2026)

Sources: enterprise.dji.com spec/FAQ pages for M400, Zenmuse L3 and Zenmuse P1, the L3 User Manual v1.0,
and DJI's Cloud-API-Doc (WPML). Anything not published by DJI is marked **n/p** and never used as fact.
The values in code live in [`packages/core/src/equipment.ts`](../packages/core/src/equipment.ts).

## DJI Matrice 400
| Item | Official | Planner's conservative limit |
|---|---|---|
| Max horizontal speed | 25 m/s | line speed warn > 15, error > 20 m/s |
| Max ascent / descent | 10 / 8 m/s | climb warn > 4, error > 6; descent warn > 3, error > 5 m/s |
| Max pitch | 35° | figure-8 bank warn > 25°, error > 30° |
| Wind resistance | 12 m/s | shown as a note |
| Flight / hover time | 59 / 53 min, **H30T only** | 30 min usable per battery set with L3 (editable; no official L3 figure) |
| Max takeoff weight / payload | 15.8 kg / 6 kg | — |
| Ceiling / temperature / IP | 7000 m / −20 to 50 °C / IP55 | — |
| RTK hover / positioning | ±0.1 m / 1 cm + 1 ppm | — |
| takeOffSecurityHeight (WPML) | 1.2–1500 m (RC) | input range 2–1500 m |
| RTH altitude range in Pilot 2 | n/p (in the M400 User Manual) | warn above 1500 m |

## Zenmuse L3 LiDAR
| Pulse rate | DJI max AGL | Max distance to object | Returns | Range @10 % |
|---|---|---|---|---|
| 100 kHz | < 500 m | < 1500 m | 4/8/16 | 950 m |
| 350 kHz | < 300 m | < 430 m | 4/8/16 | 700 m |
| 1000 kHz | < 100 m (Real-Time Follow) | < 150 m | 4/8 | — |
| 2000 kHz | < 50 m (Real-Time Follow) | < 75 m | 4 | — |

- Scan modes: Linear 80°×3° (terrain mapping; used for DJI's accuracy spec), Star-shaped 80°×80° (forest,
  dense urban), Non-repetitive 80°×80° (power lines, forestry).
- Default firmware range cap 900 m; avoid targets within 10 m. Wavelength 1535 nm (Class 1).
- System accuracy: 120 m → V 3 cm / H 4 cm RMSE; 300 m → V 5 cm / H 7.5 cm (at 15 m/s, linear, Calibrate IMU on).
- Beam divergence 0.25 mrad (spot Φ41 mm @120 m, Φ86 mm @300 m). POS 200 Hz; yaw 0.02°, pitch/roll 0.01° post-processed.
- RGB: dual 4/3 cameras, 100 MP (12288×8192) or 25 MP, 62° × 41.2° each, 107° combined; min interval 1 s (100 MP) / 0.5 s (25 MP).
- Weight 1.60 kg + 145 g gimbal connector.
- **IMU calibration:** DJI documents automatic calibration segments inside Pilot 2 Area Routes ("Calibrate IMU"),
  and **no figure-8 requirement**. Our explicit waypoint routes don't get DJI's automatic segments, so the planner
  keeps a figure-8 (optional) before the first line and every resume.
- DJI's efficiency guidance uses 20 % LiDAR side overlap and 15–17 m/s; the planner defaults to 50 % and
  recommends 8–12 m/s (conservative).

## Zenmuse P1 (photogrammetry)
- 35.9 × 24 mm, 45 MP (8192×5460), 4.4 µm. Lenses 24/35/50 mm → GSD = H/55, H/80, H/114 cm/px (DJI).
- Min interval 0.7 s. Mechanical shutter 1/2000–1 s. Accuracy H 3 cm / V 5 cm at 3 cm GSD (75 % front / 55 % side).
- Supported on M400. No newer official photogrammetry payload ("P2") exists as of this date.

## Formulas used by the planner
- LiDAR swath W = 2·AGL·tan(FOV/2); spacing = W·(1 − sidelap).
- LiDAR density (uniform, one return per pulse): per strip PRR/(v·W); all strips PRR/(v·spacing).
- Slant range at the swath edge = AGL / cos(FOV/2), checked against min(max distance, 900 m cap).
- Photo GSD = 2·AGL·tan(HFOV/2) / image width px; photo spacing = footprint-along·(1 − frontlap);
  max speed = spacing / min interval; blur (px) = v·exposure / GSD.
