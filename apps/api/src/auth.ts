import { sha256Hex, timingSafeEqual } from './crypto.ts';
import type { DeviceRow, Env } from './env.ts';
import { HttpError } from './http.ts';

function bearer(req: Request): string | null {
  const h = req.headers.get('Authorization');
  const m = h?.match(/^Bearer\s+(\S+)\s*$/i);
  return m ? m[1] : null;
}

const unauthorized = (msg: string) => new HttpError(401, 'unauthorized', msg);

/** Office routes: Authorization: Bearer <ADMIN_TOKEN>, compared in constant time. Fails closed if unset. */
export async function requireAdmin(req: Request, env: Env): Promise<void> {
  if (!env.ADMIN_TOKEN) throw new HttpError(500, 'server_misconfigured', 'ADMIN_TOKEN is not configured.');
  const token = bearer(req);
  if (!token) throw unauthorized('Missing bearer token.');
  if (!(await timingSafeEqual(token, env.ADMIN_TOKEN))) throw unauthorized('Invalid token.');
}

/** Device routes: Authorization: Bearer <device token>, looked up by SHA-256; revoked → 403. Touches last_seen. */
export async function requireDevice(req: Request, env: Env): Promise<DeviceRow> {
  const token = bearer(req);
  if (!token) throw unauthorized('Missing bearer token.');
  const hash = await sha256Hex(token);
  const dev = await env.DB.prepare('SELECT id, name, created_at, last_seen, revoked FROM devices WHERE token_hash = ?')
    .bind(hash).first<DeviceRow>();
  if (!dev) throw unauthorized('Unknown device token.');
  if (dev.revoked) throw new HttpError(403, 'device_revoked', 'This device has been revoked. Pair it again.');
  const now = Date.now();
  await env.DB.prepare('UPDATE devices SET last_seen = ? WHERE id = ?').bind(now, dev.id).run();
  return { ...dev, last_seen: now };
}
