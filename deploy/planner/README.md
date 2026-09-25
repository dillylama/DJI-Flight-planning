# Production: planner + sync API on Cloudflare Pages

| | |
|---|---|
| Project | Cloudflare Pages **`m400-planner`** (classic Pages, account luke@3dronemap.com) |
| URL | https://m400-planner.pages.dev → **https://planner.3dronemapping.com** once the CNAME is in place |
| Content | `apps/web` build (static) + `apps/api` bundled as `_worker.js`; `_routes.json` sends only `/api/*` to the worker |
| D1 | `m400-api` (`ee4a18fb-9535-491f-bb31-fa4b94398a7c`), migrations from `apps/api/migrations` |
| R2 | `m400-packages` |
| Secret | `ADMIN_TOKEN` (Pages secret). The value is in `deploy/planner/admin-token.local` (gitignored) |

Why Pages, not a plain Worker: 3dronemapping.com's DNS is hosted at **Wix**. Worker custom domains need the zone on
Cloudflare; Pages accepts a subdomain on outside DNS via a CNAME.

## Deploy
```bash
node deploy/planner/deploy.mjs            # builds apps/web + apps/api into deploy/planner/dist and deploys
```
Schema changes: `cd apps/api && npx wrangler d1 migrations apply DB --remote`.

## Custom domain (one-time, needs Luke)
1. Cloudflare dashboard → Workers & Pages → **m400-planner** → Custom domains → **Set up a domain** →
   `planner.3dronemapping.com`.
2. Wix → Domains → 3dronemapping.com → **Manage DNS records** → add **CNAME**: host `planner`, value
   `m400-planner.pages.dev`.
3. Wait for Cloudflare to show the domain as Active (SSL is issued automatically).

## Office access
The office routes require the admin token (paste it once into the planner's "Office token" field; it stays in that browser).
Recommended next step: put **Cloudflare Access** (email one-time PIN for @3dronemap.com) in front of the planner and
`/api/projects*`, `/api/pairing`, `/api/devices*`, `/api/logs*`, and bypass `/api/device/*` and `/api/health` for the RCs.
