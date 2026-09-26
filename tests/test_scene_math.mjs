import assert from 'node:assert/strict';
import test from 'node:test';

import { frameOffsetForCg } from '../ui/scene_math.js';

test('structural model origin is offset from the simulated CG pose', () => {
  assert.deepEqual(frameOffsetForCg([-1, 0, 0]), [1, 0, 0]);
  assert.deepEqual(frameOffsetForCg([0.4, -0.2, 0.3]), [-0.4, 0.3, 0.2]);
});

test('missing or zero CG keeps the legacy origin', () => {
  assert.deepEqual(frameOffsetForCg(), [0, 0, 0]);
  assert.deepEqual(frameOffsetForCg([0, 0, 0]), [0, 0, 0]);
});
