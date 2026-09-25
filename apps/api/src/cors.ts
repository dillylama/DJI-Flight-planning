import type { Env } from './env.ts';

const DEFAULT_ORIGINS = 'http://localhost:5178';

function allowedOrigin(req: Request, env: Env): string | null {
  const origin = req.headers.get('Origin');
  if (!origin) return null;
  const list = (env.ALLOWED_ORIGINS ?? DEFAULT_ORIGINS).split(',').map(s => s.trim().replace(/\/+$/, '')).filter(Boolean);
  return list.includes(origin) ? origin : null;
}

export function preflight(req: Request, env: Env): Response {
  const h = new Headers({ Vary: 'Origin' });
  const origin = allowedOrigin(req, env);
  if (origin) {
    h.set('Access-Control-Allow-Origin', origin);
    h.set('Access-Control-Allow-Methods', 'GET, HEAD, POST, DELETE, OPTIONS');
    h.set('Access-Control-Allow-Headers', 'Authorization, Content-Type, If-None-Match');
    h.set('Access-Control-Max-Age', '86400');
  }
  return new Response(null, { status: 204, headers: h });
}

export function withCors(res: Response, req: Request, env: Env): Response {
  const origin = allowedOrigin(req, env);
  const out = new Response(res.body, res); // mutable copy of headers
  out.headers.append('Vary', 'Origin');
  if (origin) {
    out.headers.set('Access-Control-Allow-Origin', origin);
    out.headers.set('Access-Control-Expose-Headers', 'ETag, Content-Length, Content-Type');
  }
  return out;
}
