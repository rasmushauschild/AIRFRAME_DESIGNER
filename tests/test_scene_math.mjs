import assert from 'node:assert/strict';
import test from 'node:test';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

import { frameOffsetForCg, builtAxis, axisToTiltCant, tiltCantToAxis } from '../ui/scene_math.js';

test('structural model origin is offset from the simulated CG pose', () => {
  assert.deepEqual(frameOffsetForCg([-1, 0, 0]), [1, 0, 0]);
  assert.deepEqual(frameOffsetForCg([0.4, -0.2, 0.3]), [-0.4, 0.3, 0.2]);
});

test('missing or zero CG keeps the legacy origin', () => {
  assert.deepEqual(frameOffsetForCg(), [0, 0, 0]);
  assert.deepEqual(frameOffsetForCg([0, 0, 0]), [0, 0, 0]);
});

// A rotor bolted on crooked is drawn crooked, and the view may not disagree with the physics about by how
// much. The cases come from geometry/propulsion.py (tests/test_mounting_error.py reads the same file).
test('the view turns a mis-mounted rotor exactly as the physics does', () => {
  const here = dirname(fileURLToPath(import.meta.url));
  const cases = JSON.parse(readFileSync(join(here, 'data', 'rotor_axes.json'), 'utf8'));
  for (const { rotor, built_axis, built_tilt_cant } of cases) {
    const got = builtAxis(rotor);
    got.forEach((v, i) => assert.ok(Math.abs(v - built_axis[i]) < 1e-9,
      `${JSON.stringify(rotor)} axis ${i}: js ${v} vs py ${built_axis[i]}`));
    const tc = axisToTiltCant(got, rotor.pos[1]);
    tc.forEach((v, i) => assert.ok(Math.abs(v - built_tilt_cant[i]) < 1e-6,
      `${JSON.stringify(rotor)} tilt/cant ${i}: js ${v} vs py ${built_tilt_cant[i]}`));
  }
});

test('taking the mounting error back off a built axis gives the design again', () => {
  // what the 3D gizmo does when you drag a crooked rotor: it must not quietly absorb the error
  const r = { pos: [0.3, -0.25, 0], axis: [0, 0, -1], tilt_error_deg: 8, cant_error_deg: -4 };
  const [t, c] = axisToTiltCant(builtAxis(r), r.pos[1]);
  const back = tiltCantToAxis(t - r.tilt_error_deg, c - r.cant_error_deg, r.pos[1]);
  back.forEach((v, i) => assert.ok(Math.abs(v - r.axis[i]) < 1e-6, `axis ${i}: ${v} vs ${r.axis[i]}`));
});
