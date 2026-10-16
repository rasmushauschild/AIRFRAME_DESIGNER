// Tying a rotor or a leg to its opposite number: the part that has to be right is *which* one it picks.
// Pick wrong and the mirror quietly drags a leg or a motor across the aircraft, which is what used to
// happen to a pair of legs bolted either side of the spine.
import test from 'node:test';
import assert from 'node:assert/strict';
import { makePair, mirrorLegOnto, mirrorRotorOnto, legFoot, twinOf, unpair, pairedOrder } from '../ui/pairing.js';

const leg = (name, y, extra = {}) => ({
  name, attach: [0.086, y, -0.124], length: 0.97, tilt_deg: -34.4, cant_deg: 52.2,
  foot_radius: 0.012, stiffness: 1500, damping: 80, friction: 0.8, enabled: true, pair: '', ...extra,
});
const rotor = (name, x, y, extra = {}) => ({
  name, pos: [x, y, 0.3], axis: [0, 0, -1], km: 0.002, max_thrust: 30, max_power_w: 0,
  tau: 0.12, diameter: 0.1, thrust_exponent: 2, enabled: true, pair: '', ...extra,
});
const tieLegs = (legs, i) => makePair(legs, i, mirrorLegOnto, legFoot, (src) => ({ ...src, name: src.name + ' R', pair: '' }));
const tieRotors = (rs, i) => makePair(rs, i, mirrorRotorOnto, (x) => x.pos, (src) => ({ ...src, name: src.name + ' R', pair: '' }));

test('legs either side of the spine tie to each other and do not move', () => {
  // the attachments are 60 mm apart, the feet 1.3 m apart: the pair is obvious at the foot and ambiguous
  // at the mount, which is why the mount is not what decides it
  const legs = [leg('L', -0.03), leg('R', 0.03), leg('N (front foot)', 0, { length: 0.198, tilt_deg: 0, cant_deg: 0, attach: [0.5, 0, 0] })];
  const before = JSON.parse(JSON.stringify(legs));
  assert.equal(tieLegs(legs, 0), '');
  assert.equal(legs.length, 3, 'no leg invented');
  assert.equal(twinOf(legs, 0), 1, 'L is tied to R');
  for (const k of ['attach', 'length', 'tilt_deg', 'cant_deg']) {
    assert.deepEqual(legs[0][k], before[0][k], `L kept its ${k}`);
    assert.deepEqual(legs[1][k], before[1][k], `R kept its ${k}`);
  }
  assert.ok(Math.abs(legFoot(legs[1])[1] - 0.662) < 0.01, 'the right foot is still out at 0.66 m');
});

test('the nose foot on the centreline is never mistaken for the twin', () => {
  const legs = [leg('L', -0.03), leg('N', 0, { length: 0.198, tilt_deg: 0, cant_deg: 0, attach: [0.5, 0, 0] })];
  assert.equal(tieLegs(legs, 0), '');
  assert.equal(legs.length, 3, 'the other side was empty, so one was made');
  assert.equal(legs[2].name, 'L R');
  assert.deepEqual(legs[1].attach, [0.5, 0, 0], 'the nose foot was left alone');
  assert.ok(legFoot(legs[2])[1] > 0.5, 'the new leg stands on the other side');
});

test('a leg on the centreline cannot be tied', () => {
  const legs = [leg('N', 0, { cant_deg: 0 })];
  assert.match(tieLegs(legs, 0), /centreline/);
  assert.equal(legs.length, 1);
});

test('a rotor ties to its mirror image, not to the neighbour one bay forward', () => {
  // four in a row down each side: by y alone the next one along is as good a match as the real twin
  const rs = [rotor('M1', -0.50, -0.21), rotor('M2', -0.50, 0.21),
              rotor('M3', -0.30, -0.19), rotor('M4', -0.30, 0.19)];
  assert.equal(tieRotors(rs, 0), '');
  assert.equal(twinOf(rs, 0), 1, 'M1 is tied to M2, not to M4');
  assert.equal(rs.length, 4);
  assert.deepEqual(rs[3].pos, [-0.30, 0.19, 0.3], 'M4 was not dragged anywhere');
});

test('mirroring keeps each motor its own spin direction', () => {
  const rs = [rotor('M1', -0.5, -0.21, { km: 0.002 }), rotor('M2', -0.5, 0.21, { km: -0.002 })];
  tieRotors(rs, 0);
  rs[0].pos = [-0.55, -0.25, 0.32];
  mirrorRotorOnto(rs[0], rs[1]);
  assert.deepEqual(rs[1].pos, [-0.55, 0.25, 0.32], 'the twin followed');
  assert.equal(rs[1].km, -0.002, 'and kept turning the way it turns');
});

test('an item whose other side is already taken says so instead of stealing it', () => {
  const rs = [rotor('M1', -0.5, -0.21), rotor('M2', -0.5, 0.21, { pair: 'x' }), rotor('M3', -0.5, 0.21, { pair: 'x' })];
  assert.match(tieRotors(rs, 0), /already tied/);
  assert.equal(rs.length, 3, 'and nothing was created');
});

test('untying leaves both sides free and the order is by couple', () => {
  const rs = [rotor('M1', -0.5, -0.21), rotor('M9', 0.4, 0), rotor('M2', -0.5, 0.21)];
  tieRotors(rs, 0);
  assert.deepEqual(pairedOrder(rs), [0, 2, 1], 'a couple is listed together');
  unpair(rs, 0);
  assert.equal(rs[0].pair, '');
  assert.equal(rs[2].pair, '');
});
