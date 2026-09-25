# apps/api — sync backend

Cloudflare Worker + D1 (projects, versions, devices) + R2 (mission packages, as-flown logs).

- `POST /projects/:id/publish`: store an immutable mission package version.
- RC pairing: the web planner shows a QR code carrying a one-time code; the RC exchanges it for a
  per-device token.
- `GET /device/missions`: the RC pulls published packages for offline use.
- `POST /device/logs`: the RC uploads as-flown logs (lines completed with recording on, resume points).
