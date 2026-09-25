// IndexedDB cache for the loaded DEM (GeoTIFF bytes), so terrain survives a page reload.
// Browser storage can be unavailable (private mode, blocked site data): every call fails soft.
const DB = '3dm-planner', STORE = 'blobs';

function open(): Promise<IDBDatabase> {
  return new Promise((res, rej) => {
    const r = indexedDB.open(DB, 1);
    r.onupgradeneeded = () => r.result.createObjectStore(STORE);
    r.onsuccess = () => res(r.result);
    r.onerror = () => rej(r.error);
  });
}

export async function putBlob(key: string, value: unknown): Promise<void> {
  try {
    const db = await open();
    await new Promise<void>((res, rej) => {
      const tx = db.transaction(STORE, 'readwrite');
      tx.objectStore(STORE).put(value, key);
      tx.oncomplete = () => res(); tx.onerror = () => rej(tx.error);
    });
  } catch { /* storage unavailable */ }
}

export async function getBlob<T>(key: string): Promise<T | null> {
  try {
    const db = await open();
    return await new Promise<T | null>((res, rej) => {
      const r = db.transaction(STORE).objectStore(STORE).get(key);
      r.onsuccess = () => res((r.result as T) ?? null); r.onerror = () => rej(r.error);
    });
  } catch { return null; }
}

export async function delBlob(key: string): Promise<void> {
  try {
    const db = await open();
    db.transaction(STORE, 'readwrite').objectStore(STORE).delete(key);
  } catch { /* ignore */ }
}
