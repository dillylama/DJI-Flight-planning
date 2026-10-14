// Trims a RINEX observation file (version 2 or 3) to a time window, keeping the header intact.
//   node tools/trim-rinex.ts <in.obs|.yyO|.rnx> <out> --from "2026-10-14 11:30:00" --to "2026-10-14 11:55:00" [--utc]
// Times are read as UTC (GPS time without the leap-second offset, which is what RINEX epochs carry; the 18 s do not
// matter for a window). Give the window with a margin of a few minutes either side of the flight. Navigation files
// are small and need no trimming. Streams the file, so a 12-hour log is fine.
import { createReadStream, createWriteStream } from 'node:fs';
import { createInterface } from 'node:readline';

const args = process.argv.slice(2);
const flag = (name: string) => { const i = args.indexOf('--' + name); return i >= 0 ? args.splice(i, 2)[1] : undefined; };
const fromS = flag('from'), toS = flag('to');
const [inPath, outPath] = args;
if (!inPath || !outPath || !fromS || !toS) { console.error('usage: node tools/trim-rinex.ts <in> <out> --from "YYYY-MM-DD HH:MM:SS" --to "YYYY-MM-DD HH:MM:SS"'); process.exit(2); }
const parse = (s: string) => { const m = s.match(/(\d{4})-(\d\d)-(\d\d)[ T](\d\d):(\d\d):(\d\d)/); if (!m) throw new Error('bad time ' + s); return Date.UTC(+m[1], +m[2] - 1, +m[3], +m[4], +m[5], +m[6]); };
const tFrom = parse(fromS), tTo = parse(toS);

const out = createWriteStream(outPath);
let inHeader = true, version = 3, kept = 0, dropped = 0, keep = false, first: string | null = null, last: string | null = null;
const rl = createInterface({ input: createReadStream(inPath), crlfDelay: Infinity });
for await (const line of rl) {
  if (inHeader) {
    out.write(line + '\n');
    if (/RINEX VERSION/.test(line)) version = Math.floor(parseFloat(line.slice(0, 9)));
    if (/END OF HEADER/.test(line)) inHeader = false;
    continue;
  }
  // epoch line: v3 starts with '>' ; v2 is " yy mm dd hh mm ss.sssssss  flag nsat"
  let t: number | null = null;
  if (version >= 3) {
    if (line.startsWith('>')) { const f = line.slice(2, 29).trim().split(/\s+/).map(Number); t = Date.UTC(f[0], f[1] - 1, f[2], f[3], f[4], Math.floor(f[5])); }
  } else if (/^ \d\d [ \d]\d [ \d]\d [ \d]\d [ \d]\d [ \d]\d\.\d/.test(line)) {
    const f = line.slice(1, 26).trim().split(/\s+/).map(Number); const yy = f[0] < 80 ? 2000 + f[0] : 1900 + f[0];
    t = Date.UTC(yy, f[1] - 1, f[2], f[3], f[4], Math.floor(f[5]));
  }
  if (t !== null) { keep = t >= tFrom && t <= tTo; if (keep) { kept++; last = line; if (!first) first = line; } else dropped++; }
  if (keep) out.write(line + '\n');
}
out.end();
console.log(`RINEX ${version}: kept ${kept} epochs, dropped ${dropped}`);
if (first) console.log(`first kept epoch: ${first.slice(0, 30).trim()}\nlast kept epoch:  ${last!.slice(0, 30).trim()}`);
else console.log('nothing in that window — check the dates and that times are UTC');
