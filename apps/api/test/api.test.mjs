// End-to-end tests: the bundled worker (dist/worker.js) in Miniflare with in-memory D1 + R2. No network.
// Run with `npm test -w @3dm/api` (builds first).

import { after, before, describe, test } from 'node:test';
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFile } from 'node:fs/promises';
import { Miniflare } from 'miniflare';

const ADMIN_TOKEN = 'test-admin-token-0123456789abcdef';
const ORIGIN = 'http://localhost:5178';
const BASE = 'http://api.test';
const WORKER = new URL('../dist/worker.js', import.meta.url);
const sha256 = buf => createHash('sha256').update(buf).digest('hex');

let mf;
let db;

/** Miniflare 5 (the version wrangler bundles): one worker, in-memory D1 + R2, plain-text vars. */
async function startWorker(vars) {
  const env = { DB: { type: 'd1', id: 'test-db' }, BUCKET: { type: 'r2', name: 'test-bucket' } };
  for (const [k, value] of Object.entries(vars)) env[k] = { type: 'text', value };
  return new Miniflare({
    cf: false, // don't fetch request.cf data from Cloudflare: tests stay offline
    workers: [{
      config: {
        name: 'api',
        compatibilityDate: '2026-09-01',
        manifest: { mainModule: 'worker.js', modules: { 'worker.js': { type: 'esm', contents: await readFile(WORKER, 'utf8') } } },
        env,
      },
    }],
  });
}

async function applyMigrations() {
  const sql = await readFile(new URL('../migrations/0001_init.sql', import.meta.url), 'utf8');
  const stmts = sql.replace(/--[^\n]*/g, '').split(';').map(s => s.trim()).filter(Boolean);
  await db.batch(stmts.map(s => db.prepare(s)));
}

before(async () => {
  mf = await startWorker({ ADMIN_TOKEN, ALLOWED_ORIGINS: `${ORIGIN}, https://planner.example` });
  db = await mf.getD1Database('DB');
  await applyMigrations();
});

after(async () => { await mf?.dispose(); });

/** fetch against the worker. `auth`: 'admin' | device token string | undefined. */
async function call(method, path, { auth, json, body, headers = {} } = {}) {
  const h = { ...headers };
  if (auth) h.Authorization = `Bearer ${auth === 'admin' ? ADMIN_TOKEN : auth}`;
  if (json !== undefined) { h['Content-Type'] = 'application/json'; body = JSON.stringify(json); }
  const res = await mf.dispatchFetch(BASE + path, { method, headers: h, body });
  const ct = res.headers.get('content-type') ?? '';
  const data = ct.includes('application/json') && res.status !== 304 && method !== 'HEAD'
    ? await res.json() : Buffer.from(await res.arrayBuffer());
  return { status: res.status, headers: res.headers, data };
}

/** Serialise multipart/form-data with Node's own FormData (keeps Miniflare's undici out of it). */
async function multipart(fields) {
  const fd = new FormData();
  for (const [k, v] of Object.entries(fields)) {
    if (v && typeof v === 'object' && 'data' in v) fd.append(k, new Blob([v.data], { type: v.type }), v.name);
    else fd.append(k, v);
  }
  const r = new Response(fd);
  return { body: Buffer.from(await r.arrayBuffer()), headers: { 'Content-Type': r.headers.get('content-type') } };
}

function mission(name, extra = {}) {
  return {
    format: '3dm-mission', version: 1, created: '2026-09-25T08:00:00.000Z',
    block: { name, poly: [[8.5, 7.5], [8.6, 7.5], [8.6, 7.6]] },
    sensor: { kind: 'lidar', payload: 'Zenmuse L3' },
    stats: { routeKm: 137.21456, flightMin: 134.9, waypoints: 3, lines: 23 },
    sorties: [
      { index: 0, fromLine: 1, toLine: 12, waypoints: [{}, {}], minutes: 29.44 },
      { index: 1, fromLine: 12, toLine: 23, waypoints: 480, flightMin: 28.1 },
    ],
    waypoints: [{ i: 0 }, { i: 1 }, { i: 2 }],
    ...extra,
  };
}

// Minimal little-endian TIFF header + padding (content isn't parsed, only the magic is checked).
const TIFF_LE = Buffer.concat([Buffer.from([0x49, 0x49, 0x2a, 0x00, 8, 0, 0, 0]), Buffer.alloc(4096, 7)]);
const TIFF_BE = Buffer.concat([Buffer.from([0x4d, 0x4d, 0x00, 0x2a, 0, 0, 0, 8]), Buffer.alloc(100, 1)]);

async function upload(projectId, fields) {
  const mp = await multipart(fields);
  return call('POST', `/api/projects/${projectId}/versions`, { auth: 'admin', ...mp });
}

async function pairDevice(name = 'RC Plus 2 #1') {
  const { data: { code } } = await call('POST', '/api/pairing', { auth: 'admin' });
  const r = await call('POST', '/api/device/pair', { json: { code, name } });
  assert.equal(r.status, 201);
  return r.data;
}

describe('M400 sync API', () => {
  const s = {}; // shared state across the ordered tests

  test('health needs no auth', async () => {
    const r = await call('GET', '/api/health');
    assert.equal(r.status, 200);
    assert.equal(r.data.ok, true);
    assert.match(r.data.version, /^\d+\.\d+\.\d+$/);
  });

  test('office routes require the admin token (401)', async () => {
    for (const [m, p] of [['GET', '/api/projects'], ['POST', '/api/pairing'], ['GET', '/api/devices'], ['GET', '/api/logs']]) {
      const none = await call(m, p);
      assert.equal(none.status, 401, `${m} ${p}`);
      assert.equal(none.data.error, 'unauthorized');
      const wrong = await call(m, p, { auth: 'nope' });
      assert.equal(wrong.status, 401);
    }
    const devToken = await call('GET', '/api/projects', { auth: 'x'.repeat(43) });
    assert.equal(devToken.status, 401);
  });

  test('unknown route → 404 JSON error, wrong method → 405', async () => {
    const r = await call('GET', '/api/nope', { auth: 'admin' });
    assert.equal(r.status, 404);
    assert.deepEqual(Object.keys(r.data).sort(), ['error', 'message']);
    assert.equal((await call('PUT', '/api/projects', { auth: 'admin' })).status, 405);
  });

  test('create project', async () => {
    const r = await call('POST', '/api/projects', { auth: 'admin', json: { name: '  Nimba block A ' } });
    assert.equal(r.status, 201);
    assert.equal(r.data.name, 'Nimba block A');
    assert.match(r.data.id, /^[0-9a-f-]{36}$/);
    s.projectA = r.data;
    const b = await call('POST', '/api/projects', { auth: 'admin', json: { name: 'Block B' } });
    s.projectB = b.data;
    await call('POST', '/api/projects', { auth: 'admin', json: { name: 'Empty C' } });

    assert.equal((await call('POST', '/api/projects', { auth: 'admin', json: { name: '' } })).status, 400);
    assert.equal((await call('POST', '/api/projects', { auth: 'admin', json: {} })).status, 400);
    const badJson = await call('POST', '/api/projects', { auth: 'admin', body: '{nope', headers: { 'Content-Type': 'application/json' } });
    assert.equal(badJson.status, 400);
    assert.equal(badJson.data.error, 'invalid_json');
  });

  test('upload version with mission + dem: sha256, manifest, R2 keys', async () => {
    const mBuf = Buffer.from(JSON.stringify(mission('Nimba A')));
    s.mission1 = mBuf;
    const r = await upload(s.projectA.id, {
      mission: { data: mBuf, type: 'application/json', name: 'a.mission.json' },
      dem: { data: TIFF_LE, type: 'image/tiff', name: 'dem.tif' },
      note: 'first cut',
    });
    assert.equal(r.status, 201, JSON.stringify(r.data));
    const v = r.data;
    assert.equal(v.n, 1);
    assert.equal(v.project_id, s.projectA.id);
    assert.equal(v.note, 'first cut');
    assert.equal(v.mission_sha256, sha256(mBuf));
    assert.equal(v.mission_size, mBuf.length);
    assert.equal(v.dem_sha256, sha256(TIFF_LE));
    assert.equal(v.dem_size, TIFF_LE.length);
    assert.equal(v.mission_key, `packages/${s.projectA.id}/${v.id}/mission.json`);
    assert.equal(v.dem_key, `packages/${s.projectA.id}/${v.id}/dem.tif`);
    assert.deepEqual(v.manifest, {
      format: '3dm-mission', formatVersion: 1, name: 'Nimba A', created: '2026-09-25T08:00:00.000Z', sensor: 'lidar',
      waypoints: 3, routeKm: 137.215, flightMin: 134.9, sortieCount: 2,
      sorties: [
        { index: 0, fromLine: 1, toLine: 12, waypoints: 2, minutes: 29.4 },
        { index: 1, fromLine: 12, toLine: 23, waypoints: 480, minutes: 28.1 },
      ],
    });
    const bucket = await mf.getR2Bucket('BUCKET');
    const obj = await bucket.get(v.mission_key);
    assert.equal(sha256(Buffer.from(await obj.arrayBuffer())), v.mission_sha256);
    s.v1 = v;
  });

  test('second upload increments n to 2; big-endian TIFF and blob-less mission field accepted', async () => {
    const mBuf = Buffer.from(JSON.stringify(mission('Nimba A rev', { sorties: undefined })));
    s.mission2 = mBuf;
    const r = await upload(s.projectA.id, {
      mission: { data: mBuf, type: 'application/json', name: 'a2.json' },
      dem: { data: TIFF_BE, type: 'image/tiff', name: 'dem.tif' },
    });
    assert.equal(r.status, 201);
    assert.equal(r.data.n, 2);
    assert.equal(r.data.note, null);
    assert.equal(r.data.manifest.sortieCount, 0);
    s.v2 = r.data;

    // Block B: mission as a plain text field, no DEM
    const b = await upload(s.projectB.id, { mission: JSON.stringify(mission('Block B')) });
    assert.equal(b.status, 201);
    assert.equal(b.data.n, 1);
    assert.equal(b.data.dem_key, null);
    s.vB = b.data;
  });

  test('upload validation: non-3dm JSON, invalid JSON, non-TIFF dem, missing mission, unknown project', async () => {
    const notOurs = await upload(s.projectA.id, { mission: { data: Buffer.from('{"format":"geojson"}'), type: 'application/json', name: 'x.json' } });
    assert.equal(notOurs.status, 400);
    assert.equal(notOurs.data.error, 'invalid_mission');

    const garbage = await upload(s.projectA.id, { mission: { data: Buffer.from('not json'), type: 'application/json', name: 'x.json' } });
    assert.equal(garbage.status, 400);
    assert.equal(garbage.data.error, 'invalid_mission');

    const png = Buffer.from([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]);
    const badDem = await upload(s.projectA.id, {
      mission: { data: s.mission1, type: 'application/json', name: 'a.json' },
      dem: { data: png, type: 'image/tiff', name: 'dem.tif' },
    });
    assert.equal(badDem.status, 400);
    assert.equal(badDem.data.error, 'invalid_dem');

    const noMission = await upload(s.projectA.id, { note: 'x' });
    assert.equal(noMission.status, 400);

    const notMultipart = await call('POST', `/api/projects/${s.projectA.id}/versions`, { auth: 'admin', json: mission('x') });
    assert.equal(notMultipart.status, 400);

    const missing = await upload('00000000-0000-0000-0000-000000000000', { mission: JSON.stringify(mission('x')) });
    assert.equal(missing.status, 404);

    const huge = Buffer.concat([Buffer.from('{"format":"3dm-mission","pad":"'), Buffer.alloc(20 * 1024 * 1024, 0x61), Buffer.from('"}')]);
    const tooBig = await upload(s.projectA.id, { mission: { data: huge, type: 'application/json', name: 'huge.json' } });
    assert.equal(tooBig.status, 413);
    assert.equal(tooBig.data.error, 'payload_too_large');

    // Rejected uploads must not create versions
    const list = await call('GET', `/api/projects/${s.projectA.id}/versions`, { auth: 'admin' });
    assert.deepEqual(list.data.map(v => v.n), [2, 1]);
  });

  test('project list carries the latest version', async () => {
    const r = await call('GET', '/api/projects', { auth: 'admin' });
    assert.equal(r.status, 200);
    const a = r.data.find(p => p.id === s.projectA.id);
    assert.equal(a.latest.n, 2);
    assert.equal(a.latest.id, s.v2.id);
    assert.equal(a.latest.manifest.name, 'Nimba A rev');
    assert.equal(r.data.find(p => p.name === 'Empty C').latest, null);
  });

  test('pairing: code format, single use, expiry, token returned once', async () => {
    const p = await call('POST', '/api/pairing', { auth: 'admin' });
    assert.equal(p.status, 201);
    assert.match(p.data.code, /^[23456789ABCDEFGHJKMNPQRSTUVWXYZ]{8}$/);
    const stored = await db.prepare('SELECT created_at, expires_at, used FROM pairing_codes WHERE code = ?').bind(p.data.code).first();
    assert.equal(stored.expires_at, p.data.expiresAt);
    assert.equal(stored.expires_at - stored.created_at, 10 * 60_000);
    assert.ok(Math.abs(p.data.expiresAt - Date.now() - 10 * 60_000) < 30_000); // workerd clock vs Node clock

    // lower case with a dash is accepted (typed by hand on the RC)
    const typed = p.data.code.toLowerCase().replace(/^(.{4})/, '$1-');
    const ok = await call('POST', '/api/device/pair', { json: { code: typed, name: 'RC Plus 2 #1' } });
    assert.equal(ok.status, 201);
    assert.match(ok.data.deviceId, /^[0-9a-f-]{36}$/);
    assert.match(ok.data.token, /^[A-Za-z0-9_-]{43}$/); // 32 bytes base64url
    s.device = ok.data;

    const again = await call('POST', '/api/device/pair', { json: { code: p.data.code, name: 'thief' } });
    assert.equal(again.status, 400);
    assert.equal(again.data.error, 'invalid_code');

    const e = await call('POST', '/api/pairing', { auth: 'admin' });
    await db.prepare('UPDATE pairing_codes SET expires_at = ? WHERE code = ?').bind(Date.now() - 1, e.data.code).run();
    const expired = await call('POST', '/api/device/pair', { json: { code: e.data.code, name: 'late' } });
    assert.equal(expired.status, 400);

    assert.equal((await call('POST', '/api/device/pair', { json: { code: 'ZZZZZZZZ' } })).status, 400);
    assert.equal((await call('POST', '/api/device/pair', { json: { code: 'O0I1L' } })).status, 400);

    // only the hash is stored
    const row = await db.prepare('SELECT token_hash FROM devices WHERE id = ?').bind(s.device.deviceId).first();
    assert.equal(row.token_hash, sha256(s.device.token));
  });

  test('device missions: auth required, latest version per project only', async () => {
    assert.equal((await call('GET', '/api/device/missions')).status, 401);
    assert.equal((await call('GET', '/api/device/missions', { auth: 'admin' })).status, 401); // admin token is not a device token

    const r = await call('GET', '/api/device/missions', { auth: s.device.token });
    assert.equal(r.status, 200);
    assert.equal(r.data.length, 2); // A and B; C has no versions
    const a = r.data.find(m => m.projectId === s.projectA.id);
    assert.equal(a.versionId, s.v2.id);
    assert.equal(a.n, 2);
    assert.equal(a.projectName, 'Nimba block A');
    assert.equal(a.manifest.name, 'Nimba A rev');
    assert.deepEqual(a.mission, { size: s.mission2.length, sha256: sha256(s.mission2), url: `/api/device/versions/${s.v2.id}/mission` });
    assert.deepEqual(a.dem, { size: TIFF_BE.length, sha256: sha256(TIFF_BE), url: `/api/device/versions/${s.v2.id}/dem` });
    assert.equal(r.data.find(m => m.projectId === s.projectB.id).dem, null);
  });

  test('download mission / dem with ETag and 304', async () => {
    const url = `/api/device/versions/${s.v1.id}/mission`;
    assert.equal((await call('GET', url)).status, 401);
    const r = await call('GET', url, { auth: s.device.token });
    assert.equal(r.status, 200);
    const etag = `"${sha256(s.mission1)}"`;
    assert.equal(r.headers.get('etag'), etag);
    assert.match(r.headers.get('content-type'), /application\/json/);
    assert.equal(sha256(Buffer.from(JSON.stringify(r.data))), sha256(Buffer.from(JSON.stringify(JSON.parse(s.mission1)))));

    const nm = await call('GET', url, { auth: s.device.token, headers: { 'If-None-Match': etag } });
    assert.equal(nm.status, 304);
    assert.equal(nm.headers.get('etag'), etag);
    const stale = await call('GET', url, { auth: s.device.token, headers: { 'If-None-Match': '"deadbeef"' } });
    assert.equal(stale.status, 200);

    const dem = await call('GET', `/api/device/versions/${s.v1.id}/dem`, { auth: s.device.token });
    assert.equal(dem.status, 200);
    assert.equal(dem.headers.get('content-type'), 'image/tiff');
    assert.equal(sha256(dem.data), sha256(TIFF_LE));
    assert.equal(dem.headers.get('etag'), `"${sha256(TIFF_LE)}"`);

    assert.equal((await call('GET', `/api/device/versions/${s.vB.id}/dem`, { auth: s.device.token })).status, 404);
    assert.equal((await call('GET', '/api/device/versions/nope/mission', { auth: s.device.token })).status, 404);
  });

  test('device log upload, office log list + fetch', async () => {
    const log = { projectId: s.projectA.id, versionId: s.v2.id, summary: { linesDone: [1, 2, 3], resumeFromLine: 4 }, track: Array(50).fill([8.5, 7.5, 900]) };
    const r = await call('POST', '/api/device/logs', { auth: s.device.token, json: log });
    assert.equal(r.status, 201, JSON.stringify(r.data));
    s.logId = r.data.id;

    // version only → project filled in
    const r2 = await call('POST', '/api/device/logs', { auth: s.device.token, json: { versionId: s.vB.id, summary: 'short' } });
    assert.equal(r2.data.projectId, s.projectB.id);

    const list = await call('GET', `/api/logs?device=${s.device.deviceId}&project=${s.projectA.id}`, { auth: 'admin' });
    assert.equal(list.status, 200);
    assert.equal(list.data.length, 1);
    assert.deepEqual(list.data[0].summary, log.summary);
    assert.match(list.data[0].r2_key, new RegExp(`^logs/${s.device.deviceId}/\\d+-${s.logId}\\.json$`));
    assert.equal((await call('GET', `/api/logs?device=${s.device.deviceId}`, { auth: 'admin' })).data.length, 2);

    const full = await call('GET', `/api/logs/${s.logId}`, { auth: 'admin' });
    assert.deepEqual(full.data, log);

    assert.equal((await call('POST', '/api/device/logs', { auth: s.device.token, body: 'nope', headers: { 'Content-Type': 'application/json' } })).status, 400);
    assert.equal((await call('POST', '/api/device/logs', { auth: s.device.token, json: [1, 2] })).status, 400);
    const big = await call('POST', '/api/device/logs', { auth: s.device.token, json: { pad: 'x'.repeat(5 * 1024 * 1024) } });
    assert.equal(big.status, 413);
    assert.equal(big.data.error, 'payload_too_large');
    assert.equal((await call('POST', '/api/device/logs', { json: log })).status, 401);
  });

  test('device list hides token hashes; revoked device is refused (403)', async () => {
    const other = await pairDevice('RC Plus 2 #2');
    const list = await call('GET', '/api/devices', { auth: 'admin' });
    assert.equal(list.status, 200);
    assert.equal(list.data.length, 2);
    for (const d of list.data) {
      assert.equal(d.token_hash, undefined);
      assert.equal(d.revoked, false);
      assert.ok(d.last_seen > 0);
    }

    const rev = await call('DELETE', `/api/devices/${s.device.deviceId}`, { auth: 'admin' });
    assert.equal(rev.status, 200);
    assert.equal(rev.data.revoked, true);
    const refused = await call('GET', '/api/device/missions', { auth: s.device.token });
    assert.equal(refused.status, 403);
    assert.equal(refused.data.error, 'device_revoked');
    assert.equal((await call('POST', '/api/device/logs', { auth: s.device.token, json: {} })).status, 403);

    assert.equal((await call('GET', '/api/device/missions', { auth: other.token })).status, 200);
    assert.equal((await call('DELETE', '/api/devices/nope', { auth: 'admin' })).status, 404);
    assert.equal((await call('DELETE', `/api/devices/${other.deviceId}`)).status, 401);
  });

  test('CORS: preflight for allowed origins, nothing for others', async () => {
    const pf = await mf.dispatchFetch(`${BASE}/api/projects`, {
      method: 'OPTIONS',
      headers: { Origin: ORIGIN, 'Access-Control-Request-Method': 'POST', 'Access-Control-Request-Headers': 'authorization, content-type' },
    });
    assert.equal(pf.status, 204);
    assert.equal(pf.headers.get('access-control-allow-origin'), ORIGIN);
    assert.match(pf.headers.get('access-control-allow-methods'), /POST/);
    assert.match(pf.headers.get('access-control-allow-methods'), /DELETE/);
    assert.match(pf.headers.get('access-control-allow-headers'), /Authorization/);
    assert.match(pf.headers.get('access-control-allow-headers'), /Content-Type/);
    assert.match(pf.headers.get('vary'), /Origin/);

    const second = await mf.dispatchFetch(`${BASE}/api/projects`, { method: 'OPTIONS', headers: { Origin: 'https://planner.example' } });
    assert.equal(second.headers.get('access-control-allow-origin'), 'https://planner.example');

    const evil = await mf.dispatchFetch(`${BASE}/api/projects`, { method: 'OPTIONS', headers: { Origin: 'https://evil.example' } });
    assert.equal(evil.headers.get('access-control-allow-origin'), null);

    const get = await call('GET', '/api/projects', { auth: 'admin', headers: { Origin: ORIGIN } });
    assert.equal(get.headers.get('access-control-allow-origin'), ORIGIN);
    assert.match(get.headers.get('access-control-expose-headers'), /ETag/);
    const err = await call('GET', '/api/projects', { headers: { Origin: ORIGIN } });
    assert.equal(err.status, 401);
    assert.equal(err.headers.get('access-control-allow-origin'), ORIGIN); // planner can read the error
  });

  test('missing ADMIN_TOKEN fails closed', async () => {
    const mf2 = await startWorker({});
    try {
      const r = await mf2.dispatchFetch(`${BASE}/api/projects`, { headers: { Authorization: 'Bearer ' } });
      assert.equal(r.status, 500);
      const body = await r.json();
      assert.equal(body.error, 'server_misconfigured');
      assert.equal(body.stack, undefined);
    } finally {
      await mf2.dispose();
    }
  });
});
