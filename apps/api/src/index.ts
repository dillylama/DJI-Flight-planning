// M400/L3 sync backend: Cloudflare Worker + D1 (DB) + R2 (BUCKET). See ../README.md.

import { requireAdmin } from './auth.ts';
import { preflight, withCors } from './cors.ts';
import * as device from './device.ts';
import type { Ctx, Env } from './env.ts';
import { HttpError, errorResponse, json } from './http.ts';
import * as office from './office.ts';
import { VERSION } from './version.ts';

type Handler = (ctx: Ctx) => Promise<Response> | Response;
interface Route { method: string; re: RegExp; keys: string[]; auth: 'admin' | 'none'; handler: Handler }

const routes: Route[] = [];
/** `auth: 'none'` routes either need no auth or do device auth themselves. */
function route(method: string, path: string, auth: Route['auth'], handler: Handler) {
  const keys: string[] = [];
  const re = new RegExp('^' + path.replace(/:(\w+)/g, (_, k: string) => { keys.push(k); return '([^/]+)'; }) + '/?$');
  routes.push({ method, re, keys, auth, handler });
}

route('GET', '/api/health', 'none', () => json({ ok: true, version: VERSION }));

// Office
route('POST', '/api/projects', 'admin', office.createProject);
route('GET', '/api/projects', 'admin', office.listProjects);
route('POST', '/api/projects/:id/versions', 'admin', office.createVersion);
route('GET', '/api/projects/:id/versions', 'admin', office.listVersions);
route('POST', '/api/pairing', 'admin', office.createPairingCode);
route('GET', '/api/devices', 'admin', office.listDevices);
route('DELETE', '/api/devices/:id', 'admin', office.revokeDevice);
route('GET', '/api/logs', 'admin', office.listLogs);
route('GET', '/api/logs/:id', 'admin', office.getLog);

// Device
route('POST', '/api/device/pair', 'none', device.pair);
route('GET', '/api/device/missions', 'none', device.missions);
route('GET', '/api/device/versions/:id/mission', 'none', ctx => device.download(ctx, 'mission'));
route('GET', '/api/device/versions/:id/dem', 'none', ctx => device.download(ctx, 'dem'));
route('POST', '/api/device/logs', 'none', device.uploadLog);

async function dispatch(req: Request, env: Env): Promise<Response> {
  const url = new URL(req.url);
  const method = req.method === 'HEAD' ? 'GET' : req.method;
  let pathMatched = false;
  for (const r of routes) {
    const m = url.pathname.match(r.re);
    if (!m) continue;
    pathMatched = true;
    if (r.method !== method) continue;
    const params: Record<string, string> = {};
    r.keys.forEach((k, i) => { params[k] = decodeURIComponent(m[i + 1]); });
    if (r.auth === 'admin') await requireAdmin(req, env);
    return r.handler({ req, env, url, params });
  }
  if (pathMatched) throw new HttpError(405, 'method_not_allowed', `${req.method} is not allowed here.`);
  throw new HttpError(404, 'not_found', 'No such route.');
}

export default {
  async fetch(req: Request, env: Env): Promise<Response> {
    // Deployed on Pages: everything outside /api is the static planner.
    if (env.ASSETS && !new URL(req.url).pathname.startsWith('/api/')) return env.ASSETS.fetch(req);
    if (req.method === 'OPTIONS') return preflight(req, env);
    let res: Response;
    try {
      res = await dispatch(req, env);
    } catch (e) {
      if (e instanceof HttpError) {
        res = errorResponse(e);
      } else {
        console.error('unhandled', e instanceof Error ? e.stack ?? e.message : e);
        res = json({ error: 'internal', message: 'Internal server error.' }, 500);
      }
    }
    if (req.method === 'HEAD') res = new Response(null, res);
    return withCors(res, req, env);
  },
} satisfies ExportedHandler<Env>;
