/* Mirrored couples: a rotor or a leg can be tied to its opposite number, so moving one moves the other.
   They stay two separate things - two outputs, two names, two spin directions, two contact points -
   because that is what they are on the aircraft; what the tie keeps the same is the geometry.

   Kept apart from the app so the one hard part, finding which item *is* the opposite number, can be
   tested: picking the wrong one silently drags a leg or a rotor across the aircraft. */

// --- mirrored couples: a rotor or a leg can be tied to its opposite number, so moving one moves the
// other. They stay two separate things — two outputs, two names, two spin directions, two contact points —
// because that is what they are on the aircraft; what the tie keeps the same is the geometry.
export const pairToken = () => 'p' + Math.random().toString(36).slice(2, 9);
export const twinOf = (list, i) => {
  const tok = list[i] && list[i].pair;
  if (!tok) return -1;
  const others = list.map((x, k) => (k !== i && x.pair === tok ? k : -1)).filter(k => k >= 0);
  return others.length === 1 ? others[0] : -1;    // a couple is two; anything else is a mistake to leave alone
};
export function unpair(list, i) {
  const j = twinOf(list, i);
  if (j >= 0) list[j].pair = '';
  if (list[i]) list[i].pair = '';
}
// Copy the geometry of `src` onto its twin, mirrored about the centreline.
export function mirrorRotorOnto(src, dst) {
  dst.pos = [src.pos[0], -src.pos[1], src.pos[2]];
  dst.axis = [src.axis[0], -src.axis[1], src.axis[2]];
  dst.max_thrust = src.max_thrust; dst.max_power_w = src.max_power_w;
  dst.tau = src.tau; dst.diameter = src.diameter; dst.thrust_exponent = src.thrust_exponent;
  dst.km = (dst.km >= 0 ? 1 : -1) * Math.abs(src.km);     // same motor, its own direction of rotation
}
export function mirrorLegOnto(src, dst) {
  dst.attach = [src.attach[0], -src.attach[1], src.attach[2]];
  dst.length = src.length; dst.tilt_deg = src.tilt_deg; dst.cant_deg = src.cant_deg;   // cant is outward, so it mirrors as it is
  dst.foot_radius = src.foot_radius; dst.stiffness = src.stiffness; dst.damping = src.damping; dst.friction = src.friction;
}
/** Tie item `i` to its opposite number, making one only if the other side really is empty. Returns a reason
 *  when it cannot, or '' when it is done. An existing item on the other side is always preferred to a new
 *  one, even if it is carrying a tie of its own: duplicating a motor that is already there is worse than
 *  re-tying it, and a list that quietly grows an extra rotor is a list you cannot trust. */
export function makePair(list, i, mirrorOnto, keyPos, mk) {
  const src = list[i];
  const p = keyPos(src), y = p[1];
  if (Math.abs(y) < 1e-6) return 'it sits on the centreline, so it has no opposite side';
  // The twin is the one standing where this one's mirror image would stand - so look for it there, in all
  // three axes, and take the nearest. A window on y alone picks the wrong partner twice over: too wide for
  // legs bolted either side of the spine (anything near the middle looks like a twin and gets dragged on
  // top of the source) and blind to x for rotors in a row, where the neighbour one bay forward matches
  // just as well as the real opposite number.
  const want = [p[0], -y, p[2]];
  const near = Math.max(0.02, 0.25 * Math.abs(y));
  const away = (x) => { const q = keyPos(x); return Math.hypot(q[0] - want[0], q[1] - want[1], q[2] - want[2]); };
  const mirrored = (x, k) => {
    const o = keyPos(x)[1];
    return k !== i && Math.abs(o) > 1e-6 && (o < 0) !== (y < 0) && away(x) <= near;
  };
  const nearest = (extra) => list.reduce((best, x, k) =>
    (mirrored(x, k) && extra(x, k) && (best < 0 || away(x) < away(list[best]))) ? k : best, -1);
  let j = nearest((x) => !x.pair);                       // free, on the other side
  if (j < 0) j = nearest((x, k) => twinOf(list, k) < 0);  // there, but half tied
  if (j < 0 && list.some((x, k) => mirrored(x, k))) return 'the item on the other side is already tied to another';
  if (j < 0) { list.push(mk(src)); j = list.length - 1; }
  const tok = pairToken();
  src.pair = tok; list[j].pair = tok;
  mirrorOnto(src, list[j]);
  return '';
}


/** The foot of a leg in the structural frame: where it actually stands, which is what makes it a left or
 *  a right leg - the attachment point can be anywhere, including hard against the centreline. */
export function legFoot(l) {
  const side = l.attach[1] < 0 ? -1 : 1;
  const t = (l.tilt_deg || 0) * Math.PI / 180, c = (l.cant_deg || 0) * Math.PI / 180 * side;
  const d = [Math.sin(t), Math.sin(c) * Math.cos(t), Math.cos(c) * Math.cos(t)];
  return d.map((v, k) => +l.attach[k] + (+l.length || 0) * v);
}

export function pairedOrder(list) {
  const out = [], done = new Set();
  list.forEach((_, i) => {
    if (done.has(i)) return;
    done.add(i); out.push(i);
    const j = twinOf(list, i);
    if (j >= 0 && !done.has(j)) { done.add(j); out.push(j); }
  });
  return out;
}
