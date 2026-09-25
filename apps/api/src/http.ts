// Small HTTP helpers: JSON responses, typed errors, bounded body reads.

export class HttpError extends Error {
  constructor(public status: number, public code: string, message: string) {
    super(message);
  }
}

export const bad = (message: string, code = 'bad_request') => new HttpError(400, code, message);
export const notFound = (message = 'Not found.') => new HttpError(404, 'not_found', message);
export const tooLarge = (message: string) => new HttpError(413, 'payload_too_large', message);

export function json(data: unknown, status = 200, headers?: HeadersInit): Response {
  const h = new Headers(headers);
  h.set('Content-Type', 'application/json; charset=utf-8');
  return new Response(JSON.stringify(data), { status, headers: h });
}

export function errorResponse(e: HttpError): Response {
  return json({ error: e.code, message: e.message }, e.status);
}

/** Reject early on a declared Content-Length above `max`. */
export function checkContentLength(req: Request, max: number, what: string): void {
  const cl = req.headers.get('Content-Length');
  if (cl !== null && Number(cl) > max) throw tooLarge(`${what} exceeds ${fmtBytes(max)}.`);
}

/** Read the whole body, aborting once more than `max` bytes arrive (covers chunked uploads). */
export async function readBodyLimited(req: Request, max: number, what: string): Promise<Uint8Array> {
  checkContentLength(req, max, what);
  if (!req.body) return new Uint8Array(0);
  const reader = req.body.getReader();
  const chunks: Uint8Array[] = [];
  let total = 0;
  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    total += value.byteLength;
    if (total > max) {
      await reader.cancel().catch(() => {});
      throw tooLarge(`${what} exceeds ${fmtBytes(max)}.`);
    }
    chunks.push(value);
  }
  const out = new Uint8Array(total);
  let off = 0;
  for (const c of chunks) { out.set(c, off); off += c.byteLength; }
  return out;
}

export async function readJson<T = unknown>(req: Request, max = 64 * 1024): Promise<T> {
  const bytes = await readBodyLimited(req, max, 'Request body');
  try {
    return JSON.parse(new TextDecoder().decode(bytes)) as T;
  } catch {
    throw bad('Body must be valid JSON.', 'invalid_json');
  }
}

export function fmtBytes(n: number): string {
  return n >= 1024 * 1024 ? `${(n / 1024 / 1024).toFixed(0)} MB` : `${(n / 1024).toFixed(0)} KB`;
}
