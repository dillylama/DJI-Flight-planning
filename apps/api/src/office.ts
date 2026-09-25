// Office (admin) routes. All require Authorization: Bearer <ADMIN_TOKEN> (checked by the router).

import { newPairingCode, sha256Hex } from './crypto.ts';
import type { Ctx } from './env.ts';
import { HttpError, bad, checkContentLength, json, notFound, readJson, tooLarge } from './http.ts';
import { buildManifest, isMission } from './manifest.ts';

export const MAX_MISSION = 20 * 1024 * 1024;
export const MAX_DEM = 60 * 1024 * 1024;
const MAX_NOTE = 2000;
export const PAIRING_TTL_MS = 10 * 60 * 1000;

interface ProjectRow { id: string; name: string; created_at: number; updated_at: number }
export interface VersionRow {
  id: string; project_id: string; n: number; created_at: number; note: string | null; manifest: string;
  mission_key: string; mission_size: number; mission_sha256: string;
  dem_key: string | null; dem_size: number | null; dem_sha256: string | null;
}

export const parseJson = (s: string | null) => { try { return s === null ? null : JSON.parse(s); } catch { return null; } };
const versionOut = (v: VersionRow) => ({ ...v, manifest: parseJson(v.manifest) });

async function getProject(ctx: Ctx, id: string): Promise<ProjectRow> {
  const p = await ctx.env.DB.prepare('SELECT * FROM projects WHERE id = ?').bind(id).first<ProjectRow>();
  if (!p) throw notFound('Project not found.');
  return p;
}

export async function createProject(ctx: Ctx): Promise<Response> {
  const body = await readJson<{ name?: unknown } | null>(ctx.req);
  const name = typeof body?.name === 'string' ? body.name.trim() : '';
  if (!name) throw bad('`name` is required.');
  if (name.length > 200) throw bad('`name` is longer than 200 characters.');
  const now = Date.now();
  const p: ProjectRow = { id: crypto.randomUUID(), name, created_at: now, updated_at: now };
  await ctx.env.DB.prepare('INSERT INTO projects (id, name, created_at, updated_at) VALUES (?, ?, ?, ?)')
    .bind(p.id, p.name, p.created_at, p.updated_at).run();
  return json(p, 201);
}

export async function listProjects(ctx: Ctx): Promise<Response> {
  const { results } = await ctx.env.DB.prepare(
    `SELECT p.*, v.id AS v_id, v.n AS v_n, v.created_at AS v_created_at, v.note AS v_note, v.manifest AS v_manifest
       FROM projects p
       LEFT JOIN versions v ON v.project_id = p.id AND v.n = (SELECT MAX(n) FROM versions WHERE project_id = p.id)
      ORDER BY p.updated_at DESC`,
  ).all<ProjectRow & { v_id: string | null; v_n: number; v_created_at: number; v_note: string | null; v_manifest: string }>();
  return json(results.map(r => ({
    id: r.id, name: r.name, created_at: r.created_at, updated_at: r.updated_at,
    latest: r.v_id ? { id: r.v_id, n: r.v_n, created_at: r.v_created_at, note: r.v_note, manifest: parseJson(r.v_manifest) } : null,
  })));
}

export async function listVersions(ctx: Ctx): Promise<Response> {
  await getProject(ctx, ctx.params.id);
  const { results } = await ctx.env.DB.prepare('SELECT * FROM versions WHERE project_id = ? ORDER BY n DESC')
    .bind(ctx.params.id).all<VersionRow>();
  return json(results.map(versionOut));
}

const isTiff = (b: Uint8Array) =>
  b.length >= 4 && ((b[0] === 0x49 && b[1] === 0x49 && b[2] === 0x2a && b[3] === 0x00) // II*\0 little-endian
    || (b[0] === 0x4d && b[1] === 0x4d && b[2] === 0x00 && b[3] === 0x2a)); // MM\0* big-endian

/** Publish an immutable mission package version (multipart: mission, dem?, note?). */
export async function createVersion(ctx: Ctx): Promise<Response> {
  const { env, req } = ctx;
  const project = await getProject(ctx, ctx.params.id);
  if (!(req.headers.get('Content-Type') ?? '').toLowerCase().startsWith('multipart/form-data')) {
    throw bad('Expected multipart/form-data with a `mission` field.', 'expected_multipart');
  }
  checkContentLength(req, MAX_MISSION + MAX_DEM + 1024 * 1024, 'Upload');

  let form: FormData;
  try { form = await req.formData(); } catch { throw bad('Malformed multipart body.', 'invalid_multipart'); }

  // mission (required): a file/blob, or a plain text field
  const mField = form.get('mission');
  if (mField === null) throw bad('`mission` is required.', 'mission_required');
  const missionBytes = typeof mField === 'string' ? new TextEncoder().encode(mField) : new Uint8Array(await mField.arrayBuffer());
  if (missionBytes.byteLength > MAX_MISSION) throw tooLarge('`mission` exceeds 20 MB.');
  let mission: unknown;
  try { mission = JSON.parse(new TextDecoder().decode(missionBytes)); } catch { throw bad('`mission` is not valid JSON.', 'invalid_mission'); }
  if (!isMission(mission)) throw bad("`mission` must be a 3dm mission (format === '3dm-mission').", 'invalid_mission');

  // dem (optional): GeoTIFF
  const dField = form.get('dem');
  let demBytes: Uint8Array | null = null;
  if (dField !== null && dField !== '') {
    if (typeof dField === 'string') throw bad('`dem` must be a file.', 'invalid_dem');
    if (dField.size > MAX_DEM) throw tooLarge('`dem` exceeds 60 MB.');
    demBytes = new Uint8Array(await dField.arrayBuffer());
    if (!isTiff(demBytes)) throw bad('`dem` is not a TIFF (expected an II*\\0 or MM\\0* header).', 'invalid_dem');
  }

  const nField = form.get('note');
  const note = typeof nField === 'string' && nField.trim() ? nField.trim() : null;
  if (note && note.length > MAX_NOTE) throw bad(`\`note\` is longer than ${MAX_NOTE} characters.`);

  const id = crypto.randomUUID();
  const prefix = `packages/${project.id}/${id}`;
  const missionSha = await sha256Hex(missionBytes);
  const demSha = demBytes ? await sha256Hex(demBytes) : null;
  const missionKey = `${prefix}/mission.json`;
  const demKey = demBytes ? `${prefix}/dem.tif` : null;

  // R2 verifies the checksum on write.
  await env.BUCKET.put(missionKey, missionBytes, {
    httpMetadata: { contentType: 'application/json' }, sha256: missionSha, customMetadata: { sha256: missionSha },
  });
  if (demBytes && demKey && demSha) {
    await env.BUCKET.put(demKey, demBytes, {
      httpMetadata: { contentType: 'image/tiff' }, sha256: demSha, customMetadata: { sha256: demSha },
    });
  }

  const now = Date.now();
  const manifest = JSON.stringify(buildManifest(mission));
  let row: VersionRow | null;
  try {
    // n is assigned inside the INSERT; UNIQUE(project_id, n) turns a concurrent publish into a 409.
    row = await env.DB.prepare(
      `INSERT INTO versions (id, project_id, n, created_at, note, manifest, mission_key, mission_size, mission_sha256, dem_key, dem_size, dem_sha256)
       VALUES (?1, ?2, (SELECT COALESCE(MAX(n), 0) + 1 FROM versions WHERE project_id = ?2), ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11)
       RETURNING *`,
    ).bind(id, project.id, now, note, manifest, missionKey, missionBytes.byteLength, missionSha, demKey, demBytes?.byteLength ?? null, demSha)
      .first<VersionRow>();
  } catch (e) {
    await env.BUCKET.delete([missionKey, ...(demKey ? [demKey] : [])]).catch(() => {});
    if (String(e).includes('UNIQUE')) throw new HttpError(409, 'conflict', 'Another version was published at the same time. Retry.');
    throw e;
  }
  await env.DB.prepare('UPDATE projects SET updated_at = ? WHERE id = ?').bind(now, project.id).run();
  return json(versionOut(row!), 201);
}

export async function createPairingCode(ctx: Ctx): Promise<Response> {
  const now = Date.now();
  // Housekeeping: drop codes that expired more than a day ago.
  await ctx.env.DB.prepare('DELETE FROM pairing_codes WHERE expires_at < ?').bind(now - 86_400_000).run();
  for (let attempt = 0; attempt < 5; attempt++) {
    const code = newPairingCode();
    const expiresAt = now + PAIRING_TTL_MS;
    const r = await ctx.env.DB.prepare('INSERT OR IGNORE INTO pairing_codes (code, created_at, expires_at, used) VALUES (?, ?, ?, 0)')
      .bind(code, now, expiresAt).run();
    if (r.meta.changes === 1) return json({ code, expiresAt }, 201);
  }
  throw new Error('could not allocate a unique pairing code');
}

const DEVICE_COLS = 'id, name, created_at, last_seen, revoked';
const deviceOut = (d: Record<string, unknown>) => ({ ...d, revoked: !!d.revoked });

export async function listDevices(ctx: Ctx): Promise<Response> {
  const { results } = await ctx.env.DB.prepare(`SELECT ${DEVICE_COLS} FROM devices ORDER BY created_at DESC`).all();
  return json(results.map(deviceOut));
}

export async function revokeDevice(ctx: Ctx): Promise<Response> {
  const d = await ctx.env.DB.prepare(`UPDATE devices SET revoked = 1 WHERE id = ? RETURNING ${DEVICE_COLS}`)
    .bind(ctx.params.id).first<Record<string, unknown>>();
  if (!d) throw notFound('Device not found.');
  return json(deviceOut(d));
}

export async function listLogs(ctx: Ctx): Promise<Response> {
  const device = ctx.url.searchParams.get('device');
  const project = ctx.url.searchParams.get('project');
  const limit = Math.min(Math.max(Math.trunc(Number(ctx.url.searchParams.get('limit'))) || 200, 1), 1000);
  const where: string[] = [];
  const args: unknown[] = [];
  if (device) { where.push('device_id = ?'); args.push(device); }
  if (project) { where.push('project_id = ?'); args.push(project); }
  const sql = `SELECT * FROM logs ${where.length ? 'WHERE ' + where.join(' AND ') : ''} ORDER BY created_at DESC LIMIT ?`;
  const { results } = await ctx.env.DB.prepare(sql).bind(...args, limit).all<{ summary: string | null }>();
  return json(results.map(l => ({ ...l, summary: parseJson(l.summary) })));
}

/** The full as-flown log JSON, exactly as the RC uploaded it. */
export async function getLog(ctx: Ctx): Promise<Response> {
  const row = await ctx.env.DB.prepare('SELECT r2_key FROM logs WHERE id = ?').bind(ctx.params.id).first<{ r2_key: string }>();
  if (!row) throw notFound('Log not found.');
  const obj = await ctx.env.BUCKET.get(row.r2_key);
  if (!obj) throw notFound('Log object is missing from storage.');
  return new Response(obj.body, { headers: { 'Content-Type': 'application/json', 'Content-Length': String(obj.size) } });
}
