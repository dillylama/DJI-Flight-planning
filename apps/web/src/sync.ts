// Client for apps/api (office routes). The admin token is kept in this browser only.
export interface ApiCfg { url: string; token: string }

async function call<T>(cfg: ApiCfg, path: string, init: RequestInit = {}): Promise<T> {
  const res = await fetch(cfg.url.replace(/\/+$/, '') + path, {
    ...init,
    headers: { Authorization: `Bearer ${cfg.token}`, ...(init.body && !(init.body instanceof FormData) ? { 'Content-Type': 'application/json' } : {}), ...init.headers },
  });
  const text = await res.text();
  const body = text ? JSON.parse(text) : null;
  if (!res.ok) throw new Error(body?.message ?? `HTTP ${res.status}`);
  return body as T;
}

export interface Project { id: string; name: string; latest: { id: string; n: number } | null }
export interface Version { id: string; n: number; created_at: number; mission_size: number; dem_size: number | null; manifest: { sortieCount?: number } | null }

export const health = (cfg: ApiCfg) => call<{ ok: boolean; version: string }>(cfg, '/api/health');

// Publish = find-or-create the project by name, then upload an immutable version.
export async function publish(cfg: ApiCfg, projectName: string, mission: object, dem: ArrayBuffer | null, note: string): Promise<{ project: Project; version: Version }> {
  const projects = await call<Project[]>(cfg, '/api/projects');
  const project = projects.find(p => p.name === projectName)
    ?? await call<Project>(cfg, '/api/projects', { method: 'POST', body: JSON.stringify({ name: projectName }) });
  const fd = new FormData();
  fd.append('mission', new Blob([JSON.stringify(mission)], { type: 'application/json' }), 'mission.json');
  if (dem) fd.append('dem', new Blob([dem], { type: 'image/tiff' }), 'dem.tif');
  if (note) fd.append('note', note);
  const version = await call<Version>(cfg, `/api/projects/${project.id}/versions`, { method: 'POST', body: fd });
  return { project, version };
}

export const newPairingCode = (cfg: ApiCfg) => call<{ code: string; expiresAt: number }>(cfg, '/api/pairing', { method: 'POST' });
export const listDevices = (cfg: ApiCfg) => call<{ id: string; name: string; last_seen: number | null; revoked: number }[]>(cfg, '/api/devices');
