// Hashing, token generation and constant-time comparison (Web Crypto only).

const enc = new TextEncoder();

export function hex(buf: ArrayBuffer | Uint8Array): string {
  const b = buf instanceof Uint8Array ? buf : new Uint8Array(buf);
  let s = '';
  for (const x of b) s += x.toString(16).padStart(2, '0');
  return s;
}

export async function sha256(data: ArrayBuffer | Uint8Array | string): Promise<Uint8Array> {
  const bytes = typeof data === 'string' ? enc.encode(data) : data;
  return new Uint8Array(await crypto.subtle.digest('SHA-256', bytes));
}

export const sha256Hex = async (data: ArrayBuffer | Uint8Array | string) => hex(await sha256(data));

/** Constant-time string equality: compares SHA-256 digests so neither content nor length leaks via timing. */
export async function timingSafeEqual(a: string, b: string): Promise<boolean> {
  const [x, y] = await Promise.all([sha256(a), sha256(b)]);
  let diff = 0;
  for (let i = 0; i < x.length; i++) diff |= x[i] ^ y[i];
  return diff === 0;
}

export function base64url(bytes: Uint8Array): string {
  let s = '';
  for (const b of bytes) s += String.fromCharCode(b);
  return btoa(s).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');
}

/** 32 random bytes, base64url: the device bearer token (returned once, stored only as a hash). */
export function newDeviceToken(): string {
  return base64url(crypto.getRandomValues(new Uint8Array(32)));
}

/** No 0/O, 1/I/L: easy to read off a screen or type on the RC. */
export const PAIRING_ALPHABET = '23456789ABCDEFGHJKMNPQRSTUVWXYZ';

/** Uniform random code via rejection sampling (no modulo bias). */
export function newPairingCode(len = 8): string {
  const n = PAIRING_ALPHABET.length;
  const limit = 256 - (256 % n);
  let out = '';
  while (out.length < len) {
    for (const b of crypto.getRandomValues(new Uint8Array(len * 2))) {
      if (b < limit && out.length < len) out += PAIRING_ALPHABET[b % n];
    }
  }
  return out;
}
