# apps/api — sync backend

Cloudflare Worker (TypeScript, no framework) + **D1** (`DB`: projects, versions, devices, pairing codes, log index)
+ **R2** (`BUCKET`: mission packages, as-flown logs).

It links the office planner to the RC. The planner publishes **immutable mission package versions**.
The RC Plus 2 (*3DM Fly*) pairs once with a one-time code, pulls the latest version of every project
while it has internet, and uploads as-flown logs when it's back in coverage.

Status: **live** at https://m400-planner.pages.dev/api (Cloudflare Pages + D1 + R2); see
[Deploy](#deploy-live-since-25-sep-2026). Local dev runs in Miniflare (`wrangler dev --local`).

## Layout

| Path | |
|---|---|
| `src/index.ts` | Router, CORS, error handling |
| `src/office.ts` | Admin routes (projects, versions/upload, pairing codes, devices, logs) |
| `src/device.ts` | RC routes (pair, missions, downloads, log upload) |
| `src/auth.ts`, `src/crypto.ts` | Admin/device auth, hashing, token + pairing-code generation |
| `src/manifest.ts` | Summary extracted from `mission.json` (stored in D1 per version) |
| `migrations/0001_init.sql` | D1 schema (applied by `wrangler d1 migrations` and by the tests) |
| `test/api.test.mjs` | End-to-end tests: bundled worker in Miniflare, in-memory D1 + R2, no network |
| `scripts/build.mjs` | esbuild bundle → `dist/worker.js` (used by the tests) |

## Endpoints

All JSON unless noted. Errors are always `{ "error": "<code>", "message": "..." }` with status
400 / 401 / 403 / 404 / 405 / 409 / 413 / 500. Stack traces are never returned.
Office responses use D1 column names (`snake_case`); device responses use `camelCase`.

| Method | Path | Auth | Body → Response |
|---|---|---|---|
| GET | `/api/health` | none | → `{ok: true, version}` |
| POST | `/api/projects` | admin | `{name}` → project (201) |
| GET | `/api/projects` | admin | → `[{...project, latest: {id, n, created_at, note, manifest} \| null}]` |
| POST | `/api/projects/:id/versions` | admin | multipart: `mission` (required, ≤ 20 MB, JSON with `format: "3dm-mission"`), `dem` (optional GeoTIFF, ≤ 60 MB, `II*\0`/`MM\0*` header), `note` (optional) → version row (201) |
| GET | `/api/projects/:id/versions` | admin | → versions, newest first |
| POST | `/api/pairing` | admin | → `{code, expiresAt}` (8 chars, valid 10 min, single use) |
| GET | `/api/devices` | admin | → `[{id, name, created_at, last_seen, revoked}]` (never token hashes) |
| DELETE | `/api/devices/:id` | admin | → revoked device |
| GET | `/api/logs?device=&project=&limit=` | admin | → log rows with parsed `summary` (default 200, max 1000) |
| GET | `/api/logs/:id` | admin | → the full log JSON as uploaded |
| POST | `/api/device/pair` | none | `{code, name}` → `{deviceId, token}` (201). **Token is shown once.** |
| GET | `/api/device/missions` | device | → latest version per project: `[{projectId, projectName, versionId, n, createdAt, note, manifest, mission: {size, sha256, url}, dem: {size, sha256, url} \| null}]` |
| GET/HEAD | `/api/device/versions/:id/mission` | device | → `mission.json` stream, `ETag: "<sha256>"`, `If-None-Match` → 304 |
| GET/HEAD | `/api/device/versions/:id/dem` | device | → `dem.tif` stream (`image/tiff`), same ETag rules; 404 if the version has no DEM |
| POST | `/api/device/logs` | device | JSON object ≤ 5 MB `{projectId?, versionId?, summary?, ...}` → `{id, createdAt, size, projectId, versionId}` (201) |

**Versions.** `n` is 1, 2, 3 … per project, assigned inside the INSERT with a `UNIQUE(project_id, n)`
index. A concurrent publish gets a 409 and should retry. Versions are never updated or deleted.
R2 keys are `packages/{projectId}/{versionId}/mission.json` and `…/dem.tif`. The SHA-256 is computed
in the Worker and R2 verifies it on write.

**Manifest** (from `mission.json`): `{format, formatVersion, name (block.name), created, sensor
(sensor.kind), waypoints, routeKm, flightMin, sortieCount, sorties: [{index, fromLine, toLine,
waypoints, minutes}]}`. Fields the mission doesn't have are `null`, and `sorties` is `[]` until the planner writes sorties.

**Logs.** The whole body goes to `logs/{deviceId}/{timestamp}-{id}.json` in R2. D1 keeps the row with
`summary` (capped at 64 KB). If only `versionId` is sent, the Worker fills in `projectId`. Unknown ids are
stored as sent, so a field log is never refused.

## Auth model

- **Office → API:** `Authorization: Bearer <ADMIN_TOKEN>`. It's one shared secret, compared in constant
  time (SHA-256 both sides, XOR compare). If `ADMIN_TOKEN` is unset, the office routes return 500
  `server_misconfigured` (fails closed).
- **Pairing:** the planner calls `POST /api/pairing` and shows the code as a QR code or as text. The code is 8
  characters from `23456789ABCDEFGHJKMNPQRSTUVWXYZ` (no 0/O/1/I/L), valid for 10 minutes and single use.
  An atomic conditional `UPDATE` means two RCs can't redeem the same code. Input is case-insensitive
  and dashes/spaces are ignored. An unknown, expired or used code gets one generic 400 `invalid_code`.
- **RC → API:** `POST /api/device/pair` returns a 32-byte random token (base64url). Only its SHA-256 is
  stored. Device routes look the token up by hash. An unknown token gets 401 and a revoked device gets
  403 `device_revoked`. Each call updates `last_seen`.
- **CORS:** only origins in `ALLOWED_ORIGINS` (comma list, default `http://localhost:5178`) get
  `Access-Control-Allow-Origin`. The Worker handles OPTIONS preflight (`Authorization, Content-Type, If-None-Match`)
  and exposes `ETag, Content-Length, Content-Type`. The RC is a native app and doesn't use CORS.

## Local development

```sh
# once: create apps/api/.dev.vars (gitignored by the root .gitignore) with a random admin token
cp apps/api/.dev.vars.example apps/api/.dev.vars   # then replace the value, e.g. with:
node -e "console.log(require('crypto').randomBytes(32).toString('base64url'))"

# once (and after every new migration): create the local D1 schema in apps/api/.wrangler/state
npm run db:migrate:local -w @3dm/api        # = wrangler d1 migrations apply DB --local

npm run dev -w @3dm/api                     # wrangler dev --local → http://localhost:8788
curl localhost:8788/api/health
```

Everything runs in Miniflare/workerd on this machine, with no Cloudflare login. `send_metrics` is off in
`wrangler.jsonc`. For the planner (Vite on :5178), point it at `http://localhost:8788` and send the
`.dev.vars` token.

```sh
npm test -w @3dm/api        # esbuild bundle, then node --test against Miniflare (in-memory D1 + R2)
npm run typecheck -w @3dm/api
```

New schema changes go in `migrations/000N_*.sql`. Don't edit `0001_init.sql` after it has been applied remotely.

## Deploy (live since 25 Sep 2026)

Production runs as the Cloudflare Pages project **m400-planner** (planner + this API as `_worker.js`, same origin),
with D1 `m400-api` and R2 `m400-packages`. Build + deploy with `node deploy/planner/deploy.mjs`.
Custom domain, secrets and Cloudflare Access notes: [deploy/planner/README.md](../../deploy/planner/README.md).

## TODO

- Planner UI: publish and pairing code are done; still to do: pairing QR, device list/revoke, log viewer.
- RC side: sync client using `mission.sha256` + `If-None-Match` to skip unchanged downloads.
- Package extras from `docs/architecture.md` (`sorties/NN.kmz`, `tiles/`) aren't stored yet. Only
  `mission.json` + `dem.tif` are. Extend the multipart fields when the planner produces them.
- Optional idempotency key on `POST /api/device/logs` so an RC retry after a dropped response doesn't
  create a duplicate log.
