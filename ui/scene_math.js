// Pure coordinate helpers shared by the browser scene and its lightweight tests.
// The simulator pose is the CG in NED, while model geometry is stored relative
// to the structural origin in FRD. The structural origin must therefore be
// drawn at -CG inside the vehicle group.
export function frameOffsetForCg(cg = [0, 0, 0]) {
  const x = Number(cg[0] || 0), y = Number(cg[1] || 0), z = Number(cg[2] || 0);
  return [x ? -x : 0, z, y ? -y : 0];
}

// ---------------------------------------------------------------- wing geometry
// The port of geometry/wings.py span_curve() and wing_outline(): a flat panel, one sweep and one dihedral,
// the chord running linearly from root to tip. It lives here, in plain arrays and with no three.js,
// because the 3D view and the physics have to place the same wing in exactly the same place, and that is
// only worth claiming if a test can run both and compare them.
const RAD = Math.PI / 180;

/** Where the span line has got to at span fraction f, and which way it points there, for one half. */
export function spanCurve(w, f, sd) {
  const sym = w.symmetric !== false;
  const hs = sym ? (+w.span || 0) / 2 : (+w.span || 0);
  const a = (+w.dihedral_deg || 0) * RAD, p = (+w.sweep_deg || 0) * RAD, s = hs * f;
  return { off: [-s * Math.tan(p), sd * s * Math.cos(a), -s * Math.sin(a)],
           es: [0, sd * Math.cos(a), -Math.sin(a)] };
}

const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

/** Rotate v about a unit axis by `ang` radians (Rodrigues). */
export function rotateAbout(v, axis, ang) {
  const n = Math.hypot(axis[0], axis[1], axis[2]) || 1;
  const k = [axis[0] / n, axis[1] / n, axis[2] / n];
  const c = Math.cos(ang), s = Math.sin(ang);
  const kv = cross(k, v), kd = k[0] * v[0] + k[1] * v[1] + k[2] * v[2];
  return v.map((x, i) => x * c + kv[i] * s + k[i] * kd * (1 - c));
}

/** Corner points of each wing half: root LE, tip LE, tip TE, root TE, structural frame. */
export function wingOutlinePoints(w) {
  const root = [+w.pos[0] || 0, +w.pos[1] || 0, +w.pos[2] || 0];
  const pr = (+w.pitch_deg || 0) * RAD;
  const pitchRoot = (p) => {
    const d = [p[0] - root[0], p[1] - root[1], p[2] - root[2]];
    const r = rotateAbout(d, [0, 1, 0], pr);
    return [root[0] + r[0], root[1] + r[1], root[2] + r[2]];
  };
  const out = [];
  for (const sd of (w.symmetric !== false ? [1, -1] : [1])) {
    const pts = [];
    for (const [f, c, inc] of [[0, +w.root_chord || 0, +w.incidence_deg || 0],
                               [1, +w.tip_chord || 0, (+w.incidence_deg || 0) + (+w.twist_deg || 0)]]) {
      const { off, es } = spanCurve(w, f, sd);
      const ec = rotateAbout([1, 0, 0], es.map(v => v * sd), inc * RAD);
      const le = [root[0] + off[0], root[1] + sd * (+w.root_y || 0) + off[1], root[2] + off[2]];
      pts.push([le, [le[0] - ec[0] * c, le[1] - ec[1] * c, le[2] - ec[2] * c]]);
    }
    const [[rle, rte], [tle, tte]] = pts;
    out.push([rle, tle, tte, rte].map(pitchRoot));
  }
  return out;
}

// ---------------------------------------------------------------- rotor mounting
// The port of geometry/frames.py tilt_cant_to_axis / axis_to_tilt_cant and of Rotor.built_axis. Tilt is the
// forward lean of the thrust axis from vertical (negative = backward); cant is the sideways lean, measured
// outward from the centreline on whichever side the rotor sits.
//
// `axis` is the design: what the table edits, what the 3D gizmo moves, and what PX4 is told. The mounting
// error is how far the rotor really sits from that — a motor bolted on ten degrees out is not a different
// design, it is the same design built wrong — so everything that pushes the aircraft, here and in the
// physics, uses the built axis instead.
const OUTWARD = (y) => (y < 0 ? -1 : 1);

export function axisToTiltCant(a, y = 1) {
  const n = Math.hypot(a[0], a[1], a[2]) || 1;
  const tilt = Math.asin(Math.max(-1, Math.min(1, a[0] / n))) / RAD;
  const cant = Math.hypot(a[1], a[2]) < 1e-9 ? 0 : Math.atan2(a[1], -a[2]) / RAD;
  return [tilt, cant * OUTWARD(y)];
}

export function tiltCantToAxis(tilt, cant, y = 1) {
  const t = tilt * RAD, c = cant * OUTWARD(y) * RAD;
  return [+Math.sin(t).toFixed(6), +(Math.sin(c) * Math.cos(t)).toFixed(6), +(-Math.cos(c) * Math.cos(t)).toFixed(6)];
}

/** [tilt error, cant error] of a rotor, degrees; [0, 0] for one mounted as designed. */
export function mountError(r) {
  return [+((r || {}).tilt_error_deg) || 0, +((r || {}).cant_error_deg) || 0];
}

/** The thrust axis this rotor really has: its design axis turned by its mounting error. */
export function builtAxis(r) {
  const a = (r && r.axis) || [0, 0, -1];
  const [dt, dc] = mountError(r);
  if (!dt && !dc) return [...a];
  const y = ((r && r.pos) || [0, 0, 0])[1];
  const [tilt, cant] = axisToTiltCant(a, y);
  return tiltCantToAxis(tilt + dt, cant + dc, y);
}
