# DJI-Flight-planning — M400 / Zenmuse L3 LiDAR mission system

A UgCS-style mission system for **DJI Matrice 400 + Zenmuse L3** LiDAR survey, built by
[3DroneMapping](https://3dronemapping.com).

**Plan properly in the office → sync the mission to the RC → fly offline in the field**, either from
our own RC app (*3DM Fly*, DJI Mobile SDK v5) or by injecting the route into DJI Pilot 2.

> Status: **early development.** The route engine is done and tested. Web planner, sync and the
> RC app are being built. See [Roadmap](#roadmap).

---

## Why

DJI Pilot 2's Area Routes are hard to control precisely for LiDAR survey work. Common problems are
turn behaviour, terrain following that uses DJI's ASTER/SRTM DEM (which causes pitch spikes), no
IMU-calibration figure-8, and breakpoint resume that can't be trusted. This system generates
**explicit waypoint routes** where every height, speed, turn and recording action is set by us:

- **Parallel lines** from the block polygon, AGL, speed, sidelap and course. Line spacing comes from
  the L3 swath, and lines alternate direction.
- **~150 m run-in / run-out** on every line.
- **Absolute heights from a good DEM** (Copernicus GLO-30 or the client's DTM). Each waypoint is at
  least AGL above the highest terrain (with a 75 m corridor) along both adjacent legs, and
  climb/descent is gradient-limited by raising waypoints, never lowering them.
- **Fly-through turns** (`toPointAndPassWithContinuityCurvature`) with damping always shorter than
  the adjacent legs.
- **Figure-8 IMU excitation** before the first line and before every resumed segment, with
  point-cloud recording already running.
- **Resume** from any line at a new speed: the route restarts one line earlier with a fresh
  figure-8. It never depends on Pilot 2 breakpoints.
- **Sorties**: long jobs are split into battery-sized routes along the same path.

## Architecture

```
 OFFICE (web)                        CLOUD                          FIELD (RC Plus 2, offline)
┌───────────────────────┐   publish  ┌──────────────────┐  pull while  ┌──────────────────────────────┐
│ apps/web  planner     │──────────▶ │ apps/api         │  on wifi     │ apps/rc  "3DM Fly"           │
│  AOI, DEM, lines,     │            │  Cloudflare      │ ───────────▶ │  mission package cached      │
│  fig-8, profiles,     │ ◀───────── │  Worker, D1, R2  │              │  ├ fly it ourselves (MSDK)   │
│  validation, sorties  │  as-flown  │                  │ ◀─────────── │  └ inject KMZ into Pilot 2   │
└──────────┬────────────┘   logs     └──────────────────┘  logs back   └──────────────┬───────────────┘
           └────────────────────── packages/core (shared TypeScript) ─────────────────┘
```

| Path | What | Status |
|---|---|---|
| [`packages/core`](packages/core) | Route engine, WPML writer/reader, DEM sampling, validation, sorties, resume, mission package. Shared by web and RC. | Engine ported ✅, rest in progress |
| [`apps/web`](apps/web) | Office planner (Vite + TypeScript + MapLibre), deployed to Cloudflare Pages | Planned |
| [`apps/api`](apps/api) | Cloudflare Worker + D1 + R2: projects, mission packages, RC pairing, as-flown logs | Planned |
| [`apps/rc`](apps/rc) | Android app for the RC Plus 2 Enterprise (Kotlin, DJI MSDK v5 + UX SDK) | Planned |
| [`index.html`](index.html) | **Legacy** single-file KMZ patcher (still live on GitHub Pages) | Maintenance only |
| [`route-engine.js`](route-engine.js) | **Legacy** JS engine; the reference for the TypeScript parity test | Retire once core is complete |

More detail: [docs/architecture.md](docs/architecture.md) · Dev setup: [docs/dev-setup.md](docs/dev-setup.md)

## Roadmap

| Phase | Scope | Status |
|---|---|---|
| 0 | **MSDK test build (go/no-go)**: M400 connect + telemetry, L3 detect + record start/stop, KMZ via `WaypointMissionManager` in the simulator, KMZ injection into Pilot 2 | Next (needs RC + aircraft) |
| 1 | **core**: TypeScript engine ✅, DEM module, validation, sortie split, mission package, WPML writer | In progress |
| 2 | **web + api**: planner UI, profiles, validation panel, publish, RC pairing via QR | Planned |
| 3 | **RC app**: sync/offline, pre-flight checklist, fly/monitor, auto-resume, sorties, as-flown logs | Planned |
| 4 | **Pilot 2 injection** workflow | Planned |
| 5 | **Field validation**: waypoint cap, simulator run, short live block | Planned |

### Blocked on RC sample exports
Some values are never guessed, because a wrong one means Pilot 2 rejects the file or the L3 doesn't
record: the M400/L3 WPML enum values, the exact point-cloud record action XML, the waypoint cap and
the height mode. They will be copied from a **Waypoint Route** KMZ exported from the RC (L3 selected;
start recording on WP1, speed change at WP2, stop recording on WP4) placed in [`samples/`](samples).

## Quick start (core)

```bash
npm install
npm test          # parity + behaviour tests on a synthetic 3,900 ha Nimba block
npm run typecheck
```

Baseline (synthetic Nimba, 500 m AGL, 17 m/s, 50 % sidelap, 70° FOV): 23 lines, 902 WP, 137 km,
about 135 min. Resume from line 9 at 14 m/s: starts on line 8, 595 WP, about 106 min.

## Safety

This software generates flight routes for large UAS, often flown BVLOS. Every route must be reviewed
by the remote pilot before flight, including terrain clearance, RTH height, airspace and battery.
Validate new versions in the DJI simulator before flying them.
