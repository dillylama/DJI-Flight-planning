export interface Env {
  DB: D1Database;
  BUCKET: R2Bucket;
  /** Office bearer token. Secret: `.dev.vars` locally, `wrangler secret put ADMIN_TOKEN` in production. */
  ADMIN_TOKEN?: string;
  /** Comma-separated planner origins allowed by CORS. Default http://localhost:5178. */
  ALLOWED_ORIGINS?: string;
}

export interface Ctx {
  req: Request;
  env: Env;
  url: URL;
  params: Record<string, string>;
}

export interface DeviceRow {
  id: string;
  name: string;
  created_at: number;
  last_seen: number | null;
  revoked: number;
}
