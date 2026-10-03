import { test } from 'node:test';
import assert from 'node:assert/strict';
import { planLines, buildRoute, applyHeights, stats, validate, coverage, legSpeed, dist, clearanceBudget, safetyChecks, type LonLat } from '../src/index.ts';

const lat0 = -33.2, lon0 = 22.0, m = 1 / 111320, k = Math.cos(lat0 * Math.PI / 180);
const P = (x: number, y: number): LonLat => [lon0 + x * m / k, lat0 + y * m];
const poly = [P(-1500, -1500), P(1500, -1500), P(1500, 1500), P(-1500, 1500)];
// Lines run N–S (course 0). A 300 m high ridge across the middle (x = −200…200) that every line crosses.
const ridge = (lon: number) => { const x = (lon - lon0) / m * k; return Math.abs(x) < 200 ? 250 : 100; };   // 150 m step: needs 2.7 m/s at the 4 m/s climb limit, above the 1 m/s floor

test('slow mode: AGL stays nominal on the flat and legs onto the ridge are slowed to the climb limit', () => {
  const plan = planLines(poly, { aglM: 120, speedMs: 12, courseDeg: 90, wpSpacingM: 100, climbMs: 4, descentMs: 3 });   // course 90: lines E–W cross the ridge
  const f = applyHeights(plan, buildRoute(plan, { fig8: false }), ridge);
  const st = stats(plan, f);
  assert.ok(st.aglOnLineMin >= 120 - 1e-6);
  assert.ok(st.slowedLegs > 0, 'some legs slowed');
  assert.ok(st.lineSpeedMin < 12 && st.lineSpeedMax === 12, `${st.lineSpeedMin}–${st.lineSpeedMax}`);
  // every slowed leg sits exactly at the limit; no unslowed leg exceeds it
  for (let i = 0; i < f.wps.length - 1; i++) {
    const d = dist(f.wps[i].xy, f.wps[i + 1].xy), dh = f.wps[i + 1].h - f.wps[i].h;
    if (d < 1e-6) continue;
    const vz = Math.abs(dh) / d * legSpeed(f.wps, i);
    const lim = dh > 0 ? 4 : 3;
    assert.ok(vz <= lim + 1e-6, `leg ${i}: ${vz} > ${lim}`);
  }
  // at least one slowed leg sits exactly on the limit (others may be held lower by the ramp cap of a neighbour)
  assert.ok(f.wps.some((w, i) => w.slowed && i < f.wps.length - 1 && Math.abs(Math.abs(f.wps[i + 1].h - w.h) / dist(w.xy, f.wps[i + 1].xy) * legSpeed(f.wps, i) - (f.wps[i + 1].h > w.h ? 4 : 3)) < 1e-6));
  assert.ok(!validate(plan, f).some(i => i.code === 'GRADIENT' || i.code === 'TOO_STEEP'));
  assert.ok(validate(plan, f).some(i => i.code === 'SLOWED'));
  // flight time counts the slow legs
  const uniform = f.wps.slice(1).reduce((t, w, i) => t + dist(w.xy, f.wps[i].xy), 0) / 12 / 60;
  assert.ok(st.flightMin > uniform);
});

// Fly the route the way the aircraft could: on each leg it moves from the speed it arrives with towards the
// leg's target, braking no harder than `a`. Returns the worst vertical rate over its limit on any leg.
function simulate(wps: { xy: [number, number]; h: number; speed: number }[], a: number, climb: number, descent: number, instant = false): number {
  let u = 0, worst = -Infinity;
  for (let i = 0; i < wps.length - 1; i++) {
    const d = dist(wps[i].xy, wps[i + 1].xy), dh = wps[i + 1].h - wps[i].h, v = wps[i].speed;
    if (d < 1e-6) continue;
    const vEnd = instant ? v : u > v ? Math.max(v, Math.sqrt(Math.max(0, u * u - 2 * a * d))) : v;
    const vMax = instant ? v : Math.max(u, v);
    if (dh !== 0) worst = Math.max(worst, (Math.abs(dh) / d) * vMax - (dh > 0 ? climb : descent));
    u = vEnd;
  }
  return worst;
}

test('slow mode: never faster than a leg allows, however the aircraft changes speed between waypoints', () => {
  const steps = (lon: number, lat: number) => {
    const x = (lon - lon0) / m * k, y = (lat - lat0) / m;
    return 300 + 180 * Math.sin(x / 140) + 120 * Math.cos(y / 90) + (Math.abs(x - 400) < 60 ? 250 : 0);
  };
  for (const wpSpacingM of [150, 60, 25]) {
    const plan = planLines(poly, { aglM: 150, speedMs: 17, courseDeg: 90, wpSpacingM, climbMs: 4, descentMs: 3 });
    const f = applyHeights(plan, buildRoute(plan), steps);
    const tooSteep = validate(plan, f).some(i => i.code === 'TOO_STEEP');
    const w = f.wps;
    // reading 1: the speed changes after passing the waypoint, braking at 2 m/s² (the assumed minimum)
    const afterWp = simulate(w, 2, 4, 3);
    // reading 2: the aircraft ramps between the two waypoint speeds along the leg
    let ramp = -Infinity;
    for (let i = 0; i < w.length - 1; i++) {
      const d = dist(w[i].xy, w[i + 1].xy), dh = w[i + 1].h - w[i].h;
      if (d > 1e-6 && dh !== 0) ramp = Math.max(ramp, (Math.abs(dh) / d) * Math.max(w[i].speed, w[i + 1].speed) - (dh > 0 ? 4 : 3));
    }
    assert.ok(w.some(p => p.slowed), `spacing ${wpSpacingM}: some legs slowed`);
    if (!tooSteep) {
      assert.ok(afterWp <= 1e-6, `spacing ${wpSpacingM}: ${afterWp.toFixed(3)} m/s over the limit when braking after the waypoint`);
      assert.ok(ramp <= 1e-6, `spacing ${wpSpacingM}: ${ramp.toFixed(3)} m/s over the limit when ramping`);
    }
    // and the route is flagged whenever either reading breaks a limit
    assert.equal(tooSteep, afterWp > 1e-6 || ramp > 1e-6 || simulate(w, 2, 4, 3, true) > 1e-6, `spacing ${wpSpacingM}: TOO_STEEP must match the limits`);
  }
});

test('turn rounding stays inside the terrain corridor; rounding and pull-up lag are bounded and recorded', () => {
  const hills = (lon: number, lat: number) => { const x = (lon - lon0) / m * k, y = (lat - lat0) / m; return 300 + 150 * Math.sin(x / 200) * Math.cos(y / 260); };
  for (const corridorM of [30, 75]) {
    const plan = planLines(poly, { aglM: 150, speedMs: 17, corridorM });
    const f = applyHeights(plan, buildRoute(plan), hills);
    const n = f.wps.length;
    assert.ok(f.wps.every(w => w.turnM <= 0.9 * corridorM + 1e-9), `corridor ${corridorM}: turn rounding within 90 % of the corridor`);
    assert.ok(f.wps.every(w => w.roundingM <= plan.o.roundingMaxM + 1e-9 || w.dampingM === 1), 'vertical rounding capped by shortening the damping');
    assert.ok(f.wps.every(w => w.lagM >= 0 && w.lagM <= (4 + 3) ** 2 / (2 * 2) + 1e-9), 'lag never above the two rate limits stacked');
    assert.ok(f.wps.some(w => w.lagM > 0) && f.wps.some(w => w.roundingM > 0));
    assert.equal(f.wps[0].lagM + f.wps[0].roundingM + f.wps[n - 1].lagM + f.wps[n - 1].roundingM, 0, 'the first and last waypoint are stops');
    assert.ok(!validate(plan, f).some(i => i.code === 'TURN_CORRIDOR'));
    // a 90° turn between lines on level ground: 40 % of the cross leg, cut to 90 % of a narrow corridor
    const level = applyHeights(plan, buildRoute(plan), () => 100).wps;
    const turnWp = level.findIndex((w, i) => w.role === 'runout' && i + 1 < level.length && level[i + 1].role === 'runin');
    assert.ok(Math.abs(level[turnWp].dampingM - Math.min(0.4 * plan.spacing, 0.9 * corridorM)) < 1e-6, `damping at a line turn: ${level[turnWp].dampingM}`);
    assert.ok(Math.abs(level[turnWp].turnM - level[turnWp].dampingM) < 1e-6, 'a 90° turn can swing its whole damping distance');
  }
  // level ground: nothing to round vertically and nothing to lag behind
  const plan = planLines(poly, { aglM: 150 });
  const flat = applyHeights(plan, buildRoute(plan), () => 100);
  assert.ok(flat.wps.every(w => w.roundingM === 0 && w.lagM === 0));
  const b = clearanceBudget(plan, flat, 20);
  assert.deepEqual([b.aglM, b.pathM, b.uncertaintyM, b.worstCaseM], [150, 0, 20, 130]);
});

test('safety floors: low AGL, narrow corridor, thin budget and small uncertainty are refused', () => {
  const codes = (o: Record<string, number>, unc = 20, minClr = 60) => {
    const plan = planLines(poly, { speedMs: 10, ...o });
    const f = applyHeights(plan, buildRoute(plan), () => 100);
    return safetyChecks(plan, f, { uncertaintyM: unc, minClearanceM: minClr }).map(i => i.severity + ':' + i.code);
  };
  assert.deepEqual(codes({ aglM: 120 }), []);
  assert.ok(codes({ aglM: 40 }).includes('error:AGL_MIN') && codes({ aglM: 40 }).includes('error:CLEARANCE_BUDGET'));
  assert.ok(codes({ aglM: 120, corridorM: 10 }).includes('error:CORRIDOR_MIN'));
  assert.deepEqual(codes({ aglM: 60 }), ['warn:CLEARANCE_BUDGET']);                 // 60 − 0 − 20 = 40 m: allowed with a warning
  assert.ok(codes({ aglM: 120 }, 2).includes('error:UNCERTAINTY'));
  assert.ok(codes({ aglM: 120 }, NaN).includes('error:UNCERTAINTY'));
  assert.ok(codes({ aglM: 120 }, 20, 30).includes('error:MIN_CLEARANCE'));          // transit/RTH clearance must be ≥ 20 + 20
  assert.ok(codes({ aglM: 120 }, 100).includes('error:CLEARANCE_BUDGET'));
});

test('inputs the geometry cannot work with are refused, not planned', () => {
  for (const bad of [{ aglM: 0 }, { aglM: NaN }, { speedMs: 0 }, { sidelapPct: 100 }, { fovDeg: 0 }, { wpSpacingM: 0 }, { dampingFrac: 0.5 }, { corridorM: -1 }, { fig8BankDeg: 0 }]) {
    assert.throws(() => planLines(poly, bad), /must be/, JSON.stringify(bad));
  }
  assert.throws(() => planLines(poly, { aglM: 1, fovDeg: 1, sidelapPct: 90 }), /lines for this block/);
  assert.throws(() => planLines([P(0, 0), P(100, 0)]), /at least 3/);
  assert.throws(() => planLines([[179.9, 10], [-179.9, 10], [179.9, 10.1]]), /spans more than/);
  assert.throws(() => planLines([[10, 85], [10.01, 85], [10, 85.01]]), /80° latitude/);
  assert.throws(() => planLines([[10, NaN], [10.01, 5], [10, 5.01]]), /not a valid/);
});

test('raise mode keeps line speed and lifts waypoints instead', () => {
  const plan = planLines(poly, { aglM: 120, speedMs: 12, courseDeg: 90, wpSpacingM: 100, verticalMode: 'raise', maxGradient: 0.15 });
  const f = applyHeights(plan, buildRoute(plan, { fig8: false }), ridge);
  const st = stats(plan, f);
  assert.equal(st.slowedLegs, 0);
  assert.equal(st.lineSpeedMin, 12);
  assert.ok(st.aglOnLineMax > 120 + 50, 'waypoints raised well above nominal');
});

test('a vertical wall inside one leg is flagged as too steep', () => {
  const wall = (lon: number) => { const x = (lon - lon0) / m * k; return x > 0 && x < 40 ? 2000 : 100; };
  const plan = planLines(poly, { aglM: 100, speedMs: 12, courseDeg: 90, wpSpacingM: 50, climbMs: 4, descentMs: 3 });
  const f = applyHeights(plan, buildRoute(plan, { fig8: false }), wall);
  assert.ok(validate(plan, f).some(i => i.code === 'TOO_STEEP'));
});

test('figure-8 at the end: after the last run-out, then an exit point that stops recording', () => {
  const plan = planLines(poly, { aglM: 120 });
  const r = buildRoute(plan);                                   // fig8 + fig8End default on
  const roles = r.wps.map(w => w.role);
  const lastRunout = roles.lastIndexOf('runout');
  assert.ok(roles.slice(lastRunout + 1, -1).every(x => x === 'fig8'), 'figure-8 after the last run-out');
  assert.equal(roles.at(-1), 'exit');
  assert.deepEqual(r.wps.at(-1)!.actions, ['STOP_RECORD']);
  assert.equal(r.wps.filter(w => w.actions.includes('STOP_RECORD')).length, 1);
  const noEnd = buildRoute(plan, { fig8End: false });
  assert.equal(noEnd.wps.at(-1)!.role, 'runout');
  assert.ok(r.wps.length > noEnd.wps.length);
});

test('overlap bands between consecutive lines: flat ground gives the planned sidelap', () => {
  const plan = planLines(poly, { aglM: 300 });
  const f = applyHeights(plan, buildRoute(plan), () => 100);
  const c = coverage(plan, f);
  assert.equal(c.overlaps.length, plan.lines.length - 1);
  for (const o of c.overlaps) {
    assert.ok(Math.abs(o.pctMin - 50) < 1e-6 && Math.abs(o.pctMax - 50) < 1e-6, `${o.pctMin}–${o.pctMax}`);
    assert.ok(Math.abs(o.widthMinM - plan.swath / 2) < 1e-6);
    assert.ok(o.poly.length >= 4);
  }
});
