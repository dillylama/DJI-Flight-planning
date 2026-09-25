# Development setup (Windows)

## Toolchain

| Tool | Version | Used by |
|---|---|---|
| Node.js | 24.x | `packages/core`, `apps/web`, `apps/api` (Node runs `.ts` directly via type stripping) |
| Git + GitHub CLI | current | repo, CI |
| Eclipse Temurin JDK | 17 | `apps/rc` Gradle build |
| Android Studio + Android SDK | current stable | `apps/rc` |

Install on Windows:

```powershell
winget install --id EclipseAdoptium.Temurin.17.JDK --exact
winget install --id Google.AndroidStudio --exact
```

## Core

```bash
npm install
npm test
npm run typecheck
```

`packages/core` is plain TypeScript with no build step for tests: imports use `.ts` extensions and
only erasable syntax is allowed (no `enum`, no parameter properties), so Node 24 can run it
directly. The web app bundles it with Vite; the RC app loads a bundled copy in a WebView.

## RC app (`apps/rc`)

1. DJI developer account → Mobile SDK app **3DM Fly**, package `com.threedronemapping.fly`.
2. Put the app key in `apps/rc/local.properties` (gitignored, never commit):
   ```properties
   DJI_APP_KEY=<your key>
   ```
3. The RC Plus 2 Enterprise needs developer options + USB debugging on for `adb install`.
4. The first run of an MSDK app needs internet to register the app key.
5. Test missions in the DJI simulator with props **off** before any live flight.

## CI

GitHub Actions ([.github/workflows/ci.yml](../.github/workflows/ci.yml)) runs the core tests and
typecheck on every push and PR.
