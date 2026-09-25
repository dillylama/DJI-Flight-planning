// Build the planner + API into deploy/planner/dist and deploy it to Cloudflare Pages (m400-planner).
//   node deploy/planner/deploy.mjs            build + deploy
//   node deploy/planner/deploy.mjs --build    build only
import { execSync } from 'node:child_process';
import { cpSync, mkdirSync, rmSync, writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { build } from 'esbuild';

const here = fileURLToPath(new URL('.', import.meta.url));
const repo = fileURLToPath(new URL('../../', import.meta.url));
const dist = here + 'dist';
const run = (cmd, cwd = repo) => execSync(cmd, { cwd, stdio: 'inherit' });

rmSync(dist, { recursive: true, force: true });
run('npm run build -w @3dm/web');                                  // production build: no dev keys
cpSync(repo + 'apps/web/dist', dist, { recursive: true });
await build({
  absWorkingDir: repo + 'apps/api', entryPoints: ['src/index.ts'], outfile: dist + '/_worker.js',
  bundle: true, format: 'esm', platform: 'neutral', target: 'es2022', conditions: ['workerd', 'worker', 'browser'], logLevel: 'warning',
});
// Only /api/* wakes the worker; everything else is served straight from static assets.
writeFileSync(dist + '/_routes.json', JSON.stringify({ version: 1, include: ['/api/*'], exclude: [] }));
mkdirSync(dist, { recursive: true });
console.log('built', dist);

if (!process.argv.includes('--build')) run('npx wrangler pages deploy --project-name m400-planner --branch main --commit-dirty=true', here);
