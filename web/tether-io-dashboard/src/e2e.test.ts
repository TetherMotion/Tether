// @vitest-environment node
/**
 * Real end-to-end smoke test (plan item 96): connect the actual client to a
 * running `web_dashboard_example` over a real WebSocket, negotiate the V6
 * schema, and exercise the read-only surface. Skipped unless TETHER_E2E_URL
 * is set, e.g.
 *
 *   TETHER_E2E_URL=ws://127.0.0.1:8080/tether-io npm test
 */
import { describe, expect, it } from 'vitest';
import { TetherIOClient } from './client';
import { MachineControlClient } from './domain/machine-control';

const url = process.env.TETHER_E2E_URL;
const e2e = url ? describe : describe.skip;

e2e('live server', () => {
  it('negotiates V6 schema and reads machine data', async () => {
    const client = new TetherIOClient();
    await client.connect(`${url}?role=technician`, { enabled: true });
    try {
      expect(client.schemaCatalog).toBeTruthy();
      const signals = await client.list('signals');
      const functions = await client.listFunctions();
      expect(signals.length).toBeGreaterThan(0);
      const names = new Set(signals.map((s) => s.name));
      expect(names.has('machine.snapshot')).toBe(true);
      // Optional surfaces wired by the example: profile + recipes + metrics.
      expect(names.has('machine.app.profile')).toBe(true);
      const fnNames = new Set(functions.map((f) => f.name));
      expect(fnNames.has('machine.recipe.list')).toBe(true);
      expect(fnNames.has('machine.metrics')).toBe(true);

      const control = new MachineControlClient(client, functions);
      expect(control.recipesAvailable).toBe(true);
      const recipes = await control.recipeList();
      expect(recipes.length).toBeGreaterThan(0);
      expect(recipes.some((r) => r.name === 'default-wave')).toBe(true);
      const metricsFn = functions.find((f) => f.name === 'machine.metrics')!;
      const metrics = await client.callFunctionOrThrow(metricsFn.id, []);
      expect(metrics.length).toBeGreaterThan(0);
    } finally {
      client.disconnect();
    }
  }, 20000);

  // Security gate (plan item 99): an unauthenticated session is a
  // read-only observer — technician functions must fail over the wire.
  it('rejects technician functions for unauthenticated clients', async () => {
    const client = new TetherIOClient();
    await client.connect(url!, { enabled: true });
    try {
      const functions = await client.listFunctions();
      const control = new MachineControlClient(client, functions);
      await expect(control.recipeApply('default-wave')).rejects.toThrow();
    } finally {
      client.disconnect();
    }
  }, 20000);

  // Load smoke (plan item 98): TETHER_E2E_LOAD=N issues N sequential
  // metrics invokes and reports throughput. Bound is generous (50ms/call)
  // — this is a regression tripwire, not a benchmark.
  const loadCount = Number(process.env.TETHER_E2E_LOAD ?? 0);
  it.runIf(loadCount > 0)('sustains sequential function invokes', async () => {
    const client = new TetherIOClient();
    await client.connect(`${url}?role=observer`, { enabled: true });
    try {
      const functions = await client.listFunctions();
      const metricsFn = functions.find((f) => f.name === 'machine.metrics')!;
      const start = performance.now();
      for (let i = 0; i < loadCount; i += 1) {
        expect((await client.callFunctionOrThrow(metricsFn.id, [])).length).toBeGreaterThan(0);
      }
      const elapsed = performance.now() - start;
      console.log(`[e2e-load] ${loadCount} invokes in ${elapsed.toFixed(0)}ms (${((loadCount / elapsed) * 1000).toFixed(0)}/s)`);
      expect(elapsed / loadCount).toBeLessThan(50);
    } finally {
      client.disconnect();
    }
  }, 120000);
});
