// Device (RC) routes. /api/device/pair is unauthenticated; the rest need the device bearer token.

import { requireDevice } from './auth.ts';
import { PAIRING_ALPHABET, newDeviceToken, sha256Hex } from './crypto.ts';
import type { Ctx } from './env.ts';
import { HttpError, bad, json, notFound, readBodyLimited, readJson } from './http.ts';
import { parseJson, type VersionRow } from './office.ts';

const MAX_LOG = 5 * 1024 * 1024;
const MAX_SUMMARY = 64 * 1024;
const CODE_RE = new RegExp(`^[${PAIRING_ALPHABET}]{8}$`);

/** Exchange a one-time pairing code for a device token. The token is returned once and stored only as a hash. */
export async function pair(ctx: Ctx): Promise<Response> {
  const { env } = ctx;
  const body = await readJson<{ code?: unknown; name?: unknown } | null>(ctx.req);
  const code = typeof body?.code === 'string' ? body.code.toUpperCase().replace(/[\s-]/g, '') : '';
  const rawName = typeof body?.name === 'string' ? body.name.trim() : '';
  if (rawName.length > 100) throw bad('`name` is longer than 100 characters.');
  const name = rawName || 'RC';
  const invalid = new HttpError(400, 'invalid_code', 'Pairing code is invalid, expired or already used.');
  if (!CODE_RE.test(code)) throw invalid;

  // Single use: the conditional UPDATE is atomic, so two RCs racing for one code cannot both win.
  const now = Date.now();
  const claimed = await env.DB.prepare('UPDATE pairing_codes SET used = 1 WHERE code = ? AND used = 0 AND expires_at > ?')
    .bind(code, now).run();
  if (claimed.meta.changes !== 1) throw invalid;

  const deviceId = crypto.randomUUID();
  const token = newDeviceToken();
  await env.DB.prepare('INSERT INTO devices (id, name, token_hash, created_at, last_seen, revoked) VALUES (?, ?, ?, ?, ?, 0)')
    .bind(deviceId, name, await sha256Hex(token), now, now).run();
  return json({ deviceId, token }, 201);
}

const fileUrl = (versionId: string, kind: 'mission' | 'dem') => `/api/device/versions/${versionId}/${kind}`;

/** Latest version of every project, for offline sync. */
export async function missions(ctx: Ctx): Promise<Response> {
  await requireDevice(ctx.req, ctx.env);
  const { results } = await ctx.env.DB.prepare(
    `SELECT v.*, p.name AS project_name
       FROM versions v JOIN projects p ON p.id = v.project_id
      WHERE v.n = (SELECT MAX(n) FROM versions WHERE project_id = v.project_id)
      ORDER BY v.created_at DESC`,
  ).all<VersionRow & { project_name: string }>();
  return json(results.map(v => ({
    projectId: v.project_id,
    projectName: v.project_name,
    versionId: v.id,
    n: v.n,
    createdAt: v.created_at,
    note: v.note,
    manifest: parseJson(v.manifest),
    mission: { size: v.mission_size, sha256: v.mission_sha256, url: fileUrl(v.id, 'mission') },
    dem: v.dem_key ? { size: v.dem_size, sha256: v.dem_sha256, url: fileUrl(v.id, 'dem') } : null,
  })));
}

/** True if an If-None-Match header matches the (strong) ETag. */
function etagMatches(header: string | null, etag: string): boolean {
  if (!header) return false;
  return header.split(',').map(s => s.trim().replace(/^W\//, '')).some(t => t === '*' || t === etag);
}

/** Stream mission.json or dem.tif of a version. ETag = "sha256"; If-None-Match → 304. Versions are immutable. */
export async function download(ctx: Ctx, kind: 'mission' | 'dem'): Promise<Response> {
  await requireDevice(ctx.req, ctx.env);
  const v = await ctx.env.DB.prepare('SELECT * FROM versions WHERE id = ?').bind(ctx.params.id).first<VersionRow>();
  if (!v) throw notFound('Version not found.');
  const key = kind === 'mission' ? v.mission_key : v.dem_key;
  const sha = kind === 'mission' ? v.mission_sha256 : v.dem_sha256;
  if (!key || !sha) throw notFound('This version has no DEM.');

  const etag = `"${sha}"`;
  const headers = new Headers({
    ETag: etag,
    'Cache-Control': 'private, max-age=31536000, immutable',
    'Content-Type': kind === 'mission' ? 'application/json' : 'image/tiff',
  });
  if (etagMatches(ctx.req.headers.get('If-None-Match'), etag)) return new Response(null, { status: 304, headers });

  if (ctx.req.method === 'HEAD') {
    const meta = await ctx.env.BUCKET.head(key);
    if (!meta) throw notFound('Package file is missing from storage.');
    headers.set('Content-Length', String(meta.size));
    return new Response(null, { status: 200, headers });
  }
  const obj = await ctx.env.BUCKET.get(key);
  if (!obj) throw notFound('Package file is missing from storage.');
  headers.set('Content-Length', String(obj.size));
  return new Response(obj.body, { status: 200, headers });
}

/** As-flown log upload: stored whole in R2, with a D1 row carrying the summary for listing. */
export async function uploadLog(ctx: Ctx): Promise<Response> {
  const dev = await requireDevice(ctx.req, ctx.env);
  const bytes = await readBodyLimited(ctx.req, MAX_LOG, 'Log');
  let log: unknown;
  try { log = JSON.parse(new TextDecoder().decode(bytes)); } catch { throw bad('Log must be valid JSON.', 'invalid_json'); }
  if (typeof log !== 'object' || log === null || Array.isArray(log)) throw bad('Log must be a JSON object.');
  const o = log as Record<string, unknown>;
  const optId = (k: string) => {
    const v = o[k];
    if (v === undefined || v === null) return null;
    if (typeof v !== 'string' || v.length > 100) throw bad(`\`${k}\` must be a string id.`);
    return v;
  };
  let projectId = optId('projectId');
  const versionId = optId('versionId');
  // Fill the project from the version when the RC only sent the version. Unknown ids are kept as sent:
  // a field log is never refused because the office side changed.
  if (versionId && !projectId) {
    const v = await ctx.env.DB.prepare('SELECT project_id FROM versions WHERE id = ?').bind(versionId).first<{ project_id: string }>();
    projectId = v?.project_id ?? null;
  }
  let summary = o.summary === undefined ? null : JSON.stringify(o.summary);
  if (summary && summary.length > MAX_SUMMARY) summary = JSON.stringify({ truncated: true, note: 'summary > 64 KB; see the full log' });

  const id = crypto.randomUUID();
  const now = Date.now();
  const key = `logs/${dev.id}/${now}-${id}.json`;
  await ctx.env.BUCKET.put(key, bytes, { httpMetadata: { contentType: 'application/json' } });
  await ctx.env.DB.prepare(
    'INSERT INTO logs (id, device_id, project_id, version_id, created_at, r2_key, size, summary) VALUES (?, ?, ?, ?, ?, ?, ?, ?)',
  ).bind(id, dev.id, projectId, versionId, now, key, bytes.byteLength, summary).run();
  return json({ id, createdAt: now, size: bytes.byteLength, projectId, versionId }, 201);
}
