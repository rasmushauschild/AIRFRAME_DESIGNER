// The 3D view and the physics must place the same wing in the same place: this runs the browser's
// wingOutline() and compares it with the corner points geometry/wings.py produced for the same wing.
import assert from 'node:assert/strict';
import test from 'node:test';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const { wingOutlinePoints } = await import(join(here, '..', 'ui', 'scene_math.js'));
const cases = JSON.parse(readFileSync(join(here, 'data', 'wing_outlines.json'), 'utf8'));

test('scene.js places a wing where geometry/wings.py does', () => {
  for (const { wing, outline } of cases) {
    const got = wingOutlinePoints(wing);
    assert.equal(got.length, outline.length, wing.name);
    got.forEach((half, h) => half.forEach((p, k) => p.forEach((v, c) => {
      assert.ok(Math.abs(v - outline[h][k][c]) < 1e-9,
        `${wing.name} half ${h} corner ${k} axis ${c}: js ${v} vs py ${outline[h][k][c]}`);
    })));
  }
});
