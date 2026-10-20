let lastPrearmReportRequest = 0;
let prearmReportRequestPending = false;
const V = new URL(import.meta.url).search;
const { createScene } = await import('/static/scene.js' + V);
const { pairToken, twinOf, unpair, mirrorRotorOnto, mirrorLegOnto, makePair, legFoot, pairedOrder }
  = await import('/static/pairing.js' + V);
const { builtAxis, mountError, axisToTiltCant } = await import('/static/scene_math.js' + V);

const $ = (s) => document.querySelector(s);
const $$ = (s) => Array.from(document.querySelectorAll(s));
const api = async (path, body, method) => {
  const r = await fetch(path, { method: method || (body === undefined ? 'GET' : 'POST'),
    headers: { 'Content-Type': 'application/json' }, body: body === undefined ? undefined : JSON.stringify(body) });
  let j = null; try { j = await r.json(); } catch { }
  if (!r.ok) throw new Error((j && j.error) || r.statusText);
  return j;
};
// Right-align a number into a fixed character width with a fixed number of decimals, so a readout never
// changes width and nothing beside it shifts when a digit appears or disappears.
const pad = (v, d, w) => (v == null || !isFinite(v) ? '—' : (+v).toFixed(d)).padStart(w, ' ');
const fmt = (v, d = 3) => (typeof v === 'number' ? v.toFixed(d) : String(v));
const fmtDate = (iso) => { try { return new Date(iso).toLocaleString(undefined, { dateStyle: 'medium', timeStyle: 'short' }); } catch { return String(iso); } };
const api_get = (path) => api(path);
const deg = (r) => r * 180 / Math.PI;

let airframe = null;
let selected = -1;
let status = {};
let params = {};
let meta = {};
let pushTimer = null;
let joyWs = null;          // websocket handle used by the USB remote (declared early: connectWs() runs before the joystick code)

// ============================================================ scene
let scene;
try {
  scene = createScene($('#c'), {
    onSelect: (i) => { selected = i; if (i >= 0) cadSelected = null; renderRotorTable(); renderReadout(); if (i >= 0) renderCadTable(); },
    onRotorChanged: (i, r, commit) => {
      const twin = twinOf(airframe.rotors, i);
      if (twin >= 0) { mirrorRotorOnto(r, airframe.rotors[twin]); scene.updateRotorNode(twin, airframe.rotors[twin]); renderRotorRow(twin); }
      renderRotorRow(i); renderReadout(); if (commit) pushAirframe();
    },
    onCadSelect: (id) => { cadSelected = id; if (id !== null && selected >= 0) { selected = -1; renderRotorTable(); renderReadout(); } renderCadTable(); },
    onCadGroupChanged: (changes, commit) => {
      for (const { id, offset } of changes) { const b = cadBody(id); if (b) { b.offset = offset; renderCadRow(id); } }
      renderCadTotals(); if (commit) pushAirframe(true);
    },
    onCadChanged: (id, offset, commit) => { const b = cadBody(id); if (!b) return; b.offset = offset; renderCadRow(id); renderCadTotals(); if (commit) pushAirframe(true); },
  });
} catch (e) {
  // no WebGL (hidden window, remote desktop, old GPU): keep the rest of the app working without the 3D view
  console.warn('3D view unavailable:', e.message);
  // Anything the app asks of the scene answers and does nothing. A hand-written list of stubs went stale
  // every time the 3D view grew a method, and the missing one threw halfway through loading this file -
  // which silently left every button defined below it unbound. A missing 3D view must cost the 3D view
  // and nothing else.
  scene = new Proxy({ selected: -1, selectedCad: null, selectedCadIds: [] },
                    { get: (t, k) => (k in t ? t[k] : () => { }) });
  $('#viewport').insertAdjacentHTML('afterbegin', '<div class="hint" style="padding:18px">3D view unavailable in this window (no WebGL). Everything else works.</div>');
}

// ============================================================ tabs
$$('.tabs button').forEach(b => b.addEventListener('click', () => {
  $$('.tabs button').forEach(x => x.classList.toggle('active', x === b));
  $$('.tab').forEach(t => t.classList.toggle('active', t.id === 'tab-' + b.dataset.tab));
  if (b.dataset.tab === 'airframe') loadExport();
  if (b.dataset.tab === 'px4') { refreshConnection(); ensureParams(); loadExport(); refreshFirmware(); }
  if (b.dataset.tab === 'live') { ensureParams(); renderRcLive(); renderModeButtons(); loadWindModel(); }
  if (b.dataset.tab === 'sequences') { refreshBatch(); ensureParams().then(fillTakeoffLanding).catch(() => { }); fillTakeoffLanding(); renderMissions(); pollMission(); }
}));
function openTab(name) { $$('.tabs button').find(b => b.dataset.tab === name)?.click(); }


// ============================================================ confirmation without dialogs
// window.confirm() returns false at once inside the Claude desktop app's web view (and other embedded browsers),
// so destructive buttons ask on the button itself: the first click turns it into "Really …?" for 8 s,
// and the second does the thing. `label` is the short verb phrase that question is built from.
const confirmArm = new Map();   // key -> expiry (ms)
let armedBtn = null;            // the one button currently asking "Really?"
function confirmArmed(key) { return (confirmArm.get(key) || 0) > Date.now(); }
/** Put the armed button back to what it said. */
function disarmConfirm() {
  const b = armedBtn;
  armedBtn = null;
  confirmArm.clear();
  if (b && b.tagName === 'BUTTON' && document.body.contains(b) && b.dataset.label !== undefined) {
    b.innerHTML = b.dataset.label; b.classList.remove('confirm');   // innerHTML: some of them are an icon
  }
}
// A question you did not answer is a no: clicking anywhere else, or Escape, takes it back.
document.addEventListener('click', (e) => { if (armedBtn && !e.composedPath().includes(armedBtn)) disarmConfirm(); }, true);
document.addEventListener('keydown', (e) => { if (e.key === 'Escape') disarmConfirm(); });
function confirmTwice(key, btn, label) {
  if (confirmArmed(key)) {
    // answered: put the button back to what it said, for the ones that are not rebuilt after the deed
    confirmArm.delete(key); btn.classList.remove('confirm'); armedBtn = null;
    if (btn.tagName === 'BUTTON' && btn.dataset.label !== undefined) btn.innerHTML = btn.dataset.label;
    return true;
  }
  confirmArm.set(key, Date.now() + 8000);
  if (btn.tagName === 'BUTTON') {
    if (btn.dataset.label === undefined) btn.dataset.label = btn.innerHTML;
    btn.title = btn.title || label;
    btn.textContent = 'Really?'; btn.classList.add('confirm');
    armedBtn = btn;
  } else markSaved('Really ' + label + '? Choose it again', 8000);
  setTimeout(() => { if (!confirmArmed(key) && armedBtn === btn) disarmConfirm(); }, 8100);
  return false;
}

// ============================================================ airframe editing
// An empty or unparsable number box yields NaN, which JSON turns into null and the server rejects. Replace every
// non-finite number with 0 before sending and say so, so a stray blank field never blocks a push or a save.
function sanitizeNumbers(obj, path = '', bad = []) {
  if (Array.isArray(obj)) obj.forEach((v, i) => { if (typeof v === 'number' && !Number.isFinite(v)) { obj[i] = 0; bad.push(`${path}[${i}]`); } else if (v && typeof v === 'object') sanitizeNumbers(v, `${path}[${i}]`, bad); });
  else if (obj && typeof obj === 'object') Object.keys(obj).forEach(k => { const v = obj[k]; if (typeof v === 'number' && !Number.isFinite(v)) { obj[k] = 0; bad.push(path ? `${path}.${k}` : k); } else if (v && typeof v === 'object') sanitizeNumbers(v, path ? `${path}.${k}` : k, bad); });
  return bad;
}
function pushAirframe(immediate = false) {
  clearTimeout(pushTimer);
  const doPush = async () => {
    try {
      const bad = sanitizeNumbers(airframe);
      if (bad.length) logLine('[ui] empty or invalid number fields set to 0: ' + bad.join(', '));
      const res = await api('/api/airframe', { airframe, keep_state: true });
      if (res.airframe) {
        airframe.landed_pitch_deg = res.airframe.landed_pitch_deg;
        airframe.px4_overrides = { ...(airframe.px4_overrides || {}), NLF_LAND_ANG: res.airframe.landed_pitch_deg };
        $('#af-landed').value = res.airframe.landed_pitch_deg;
      }
      // the server resolves mass.from_items and normalises rotor axes; take its mass block back so the card is right
      if (res.airframe && res.airframe.mass && airframe.mass && airframe.mass.from_items) { airframe.mass = res.airframe.mass; fillMassCard(); scene.setAirframe(airframe); }
      showProblems(res.problems);
      showHover(res.hover);
      markDirty();
      loadExport();
    } catch (e) { logLine('[ui] airframe rejected: ' + e.message); }
  };
  if (immediate) doPush(); else pushTimer = setTimeout(doPush, 120);
}
function showProblems(p) { $('#af-problems').textContent = (p && p.length) ? '⚠ ' + p.join('\n⚠ ') : ''; }
function showHover(h) {
  if (!h || !h.shares) return;
  // hover_check reports enabled rotors only, in order
  const enabledRows = $$('#rotor-table tr[data-i]').filter(tr => airframe.rotors[+tr.dataset.i] && airframe.rotors[+tr.dataset.i].enabled !== false);
  $$('#rotor-table tr[data-i]').forEach(tr => { const c = tr.querySelector('td.hover'); if (c) c.innerHTML = ''; });
  enabledRows.forEach((tr, i) => {
    const cell = tr.querySelector('td.hover'); if (!cell) return;
    const u = h.hover_utilisation ? h.hover_utilisation[i] : null;
    const bad = h.negative && h.negative.includes(i + 1);
    cell.innerHTML = bad ? '<span class="err" title="allocator needs negative thrust here">neg</span>'
      : (u == null || !isFinite(u) ? '' : `<span class="bar ${u > 0.85 ? 'warn-bar' : ''}" style="width:${Math.round(Math.min(u, 1) * 40)}px" title="hover: ${(u * 100).toFixed(0)}% of max thrust"></span>`);
  });
}

// defaults for schema-2 sub-objects so a partial airframe never throws
function normaliseAirframe(af) {
  af.mass = af.mass && typeof af.mass === 'object' ? af.mass : { mass: +af.mass || 1.5 };
  af.mass.cg = af.mass.cg || [0, 0, 0];
  af.mass.inertia = af.mass.inertia || [0.02, 0.02, 0.035];
  af.mass.inertia_products = af.mass.inertia_products || [0, 0, 0];
  af.mass.items = af.mass.items || [];
  af.body = af.body || {};
  af.body.size = af.body.size || [0.16, 0.16, 0.06];
  af.body.drag_quadratic = af.body.drag_quadratic || [0.1, 0.1, 0.2];
  af.body.drag_angular = af.body.drag_angular || [0.005, 0.005, 0.005];
  af.body.drag_center = af.body.drag_center || [0, 0, 0];
  af.rotors = af.rotors || [];
  af.rotors.forEach((r, i) => { if (!r.name) r.name = 'M' + (i + 1); if (r.enabled === undefined) r.enabled = true; if (r.diameter == null) r.diameter = r.prop_diameter ?? 0.25; delete r.prop_diameter; });
  af.wings = af.wings || [];
  af.wings.forEach(w => { w.aero = w.aero || { ...WING_AERO_DEFAULT }; if (w.enabled === undefined) w.enabled = true; if (w.symmetric === undefined) w.symmetric = true; w.pos = w.pos || [0, 0, 0]; });
  af.legs = af.legs || [];
  af.legs.forEach(l => { if (l.enabled === undefined) l.enabled = true; l.attach = l.attach || [0, 0, 0]; });
  af.design = af.design || {};
  af.px4_overrides = af.px4_overrides || {};
  return af;
}

function setAirframe(af) {
  airframe = normaliseAirframe(af);
  scene.setAirframe(airframe);
  $('#af-name').value = af.name;
  if (document.activeElement !== $('#af-name')) $('#af-name').value = af.name || '';
  $('#view-title').textContent = af.name || 'Airframe';
  if (graphOn) loadRunList();          // the runs listed are this design's, so a new design means a new list
  fillMassCard();
  ['bx', 'by', 'bz'].forEach((k, i) => $('#af-' + k).value = af.body.size[i]);
  ['dragx', 'dragy', 'dragz'].forEach((k, i) => $('#af-' + k).value = af.body.drag_quadratic[i]);
  ['adragx', 'adragy', 'adragz'].forEach((k, i) => $('#af-' + k).value = af.body.drag_angular[i]);
  ['dcx', 'dcy', 'dcz'].forEach((k, i) => $('#af-' + k).value = af.body.drag_center[i]);
  renderRotorTable();
  renderMountTable();
  renderWingTable();
  renderLegTable();
  renderMotorSliders();
  fillMotorCard();
  fillCadCard();
  fillSensorCard();
  fillTakeoffLanding();
  loadExport();          // fills the edited/affected-parameters section (needs the server's export view)
}
function fillMassCard() {
  const m = airframe.mass;
  const derived = !!m.from_items;
  ['mass', 'cgx', 'cgy', 'cgz', 'ixx', 'iyy', 'izz'].forEach(k => { const el = $('#af-' + k); el.disabled = derived; el.title = derived ? 'derived from the CAD bodies / mass items (untick "Mass, CG & inertia from bodies" to type it)' : 'Manually entered value'; });
  $('#af-estimate').disabled = derived;
  $('#af-mass').value = m.mass;
  ['cgx', 'cgy', 'cgz'].forEach((k, i) => $('#af-' + k).value = +(+m.cg[i]).toFixed(4));
  ['ixx', 'iyy', 'izz'].forEach((k, i) => $('#af-' + k).value = +(+m.inertia[i]).toFixed(5));
  $('#af-battery').value = (airframe.design && airframe.design.battery_wh) || 0;
  $('#af-hover').value = airframe.hover_pitch_deg || 0;
  $('#af-landed').value = airframe.landed_pitch_deg || 0;
}
// Where the flight controller and the H-flow sensor sit, and which way they face. Both are given in the
// structural frame (0° = square with the airframe); the export folds the hover pitch in on top.
const flowSensor = () => ((airframe.design ||= {}).flow_sensor ||= { enabled: false, position: [...airframe.mass.cg], rotation: [0, 0, 0], range_m: 4 });
function fillSensorCard() {
  if (!airframe || !$('#imu-x')) return;
  const imu = airframe.design?.pixhawk_position || airframe.mass.cg;
  const rot = airframe.design?.pixhawk_rotation || [0, 0, 0];
  ['x', 'y', 'z'].forEach((axis, i) => { $('#imu-' + axis).value = +(+imu[i]).toFixed(4); });
  ['rr', 'rp', 'ry'].forEach((k, i) => { $('#imu-' + k).value = +(+rot[i]).toFixed(1); });
  const f = airframe.design?.flow_sensor;
  $('#flow-on').checked = !!f?.enabled;
  const pos = f?.position || imu, frot = f?.rotation || [0, 0, 0];
  ['x', 'y', 'z'].forEach((axis, i) => { $('#flow-' + axis).value = +(+pos[i]).toFixed(4); });
  ['rr', 'rp', 'ry'].forEach((k, i) => { $('#flow-' + k).value = +(+frot[i]).toFixed(1); });
  $('#flow-range').value = f?.range_m ?? 4;
  $('#flow-indoor').checked = !!airframe.design?.indoor;
  $$('.flow-fields input').forEach(el => el.disabled = !f?.enabled);
  // What the beam does in hover is the thing that decides whether the sensor is usable, so it is the note.
  let note = '';
  if (f?.enabled) {
    const [r, pch, y] = (frot || [0, 0, 0]).map(v => (+v || 0) * Math.PI / 180);
    const hover = (+airframe.hover_pitch_deg || 0) * Math.PI / 180;
    // the sensor's own +Z, turned into the hover frame: the beam
    const Rz = [[Math.cos(y), -Math.sin(y), 0], [Math.sin(y), Math.cos(y), 0], [0, 0, 1]];
    const Ry = [[Math.cos(pch), 0, Math.sin(pch)], [0, 1, 0], [-Math.sin(pch), 0, Math.cos(pch)]];
    const Rx = [[1, 0, 0], [0, Math.cos(r), -Math.sin(r)], [0, Math.sin(r), Math.cos(r)]];
    const Rh = [[Math.cos(hover), 0, Math.sin(hover)], [0, 1, 0], [-Math.sin(hover), 0, Math.cos(hover)]];
    const mul = (A, B) => A.map(row => B[0].map((_, j) => row.reduce((t, v, k) => t + v * B[k][j], 0)));
    const beam = mul(mul(Rh, mul(Rz, mul(Ry, Rx))), [[0], [0], [1]]).map(v => v[0]);
    const off = Math.acos(Math.max(-1, Math.min(1, beam[2]))) * 180 / Math.PI;
    note = off < 1 ? 'flow looks straight down in hover'
         : `flow beam ${off.toFixed(0)}° off vertical in hover` + (off > 30 ? ' — PX4 cannot correct that' : '');
  }
  $('#sensor-note').textContent = note;
}
function fillMotorCard() {
  const r = airframe.rotors[selected >= 0 ? selected : 0]; if (!r) return;
  $('#m-power').value = r.max_power_w || 0;
  $('#m-tmax').value = r.max_thrust; $('#m-tau').value = r.tau;
  $('#m-km').value = Math.abs(r.km); $('#m-dia').value = r.diameter; $('#m-exp').value = r.thrust_exponent;
}
function applyMotorCard() {
  const tmax = +$('#m-tmax').value, tau = +$('#m-tau').value, km = Math.abs(+$('#m-km').value);
  const dia = +$('#m-dia').value, ex = +$('#m-exp').value;
  // one card for every rotor: they are all the same kind of thing now, and km keeps its sign per rotor
  airframe.rotors.forEach(r => { r.max_thrust = tmax; r.tau = tau; r.km = (r.km >= 0 ? 1 : -1) * km; r.diameter = dia; r.thrust_exponent = ex; });
  setAirframe(airframe); pushAirframe(true);
}
$('#m-power').addEventListener('change', () => {
  const watts = Number($('#m-power').value);
  if (!Number.isFinite(watts) || watts < 0) { fillMotorCard(); return; }
  airframe.rotors.forEach(r => { r.max_power_w = watts; });
  pushAirframe(true);
});
['m-tmax', 'm-tau', 'm-km', 'm-dia', 'm-exp'].forEach(id => $('#' + id).addEventListener('change', applyMotorCard));

// ============================================================ CAD bodies (STEP)
let cadSelected = null;              // body id highlighted in the table and the 3D view
const cadCentroids = {};             // id -> centroid in the structural frame without the drag offset
let cadMeshKey = null;               // file|rotation|origin|scale of the meshes currently in the scene
const cadKey = (cad) => [cad.file, (cad.rotation_deg || []).join(','), (cad.origin || []).join(','), cad.scale].join('|');
const cadBody = (id) => (airframe && airframe.cad && airframe.cad.bodies || []).find(b => b.id === id);
const cadPos = (b) => { const c = cadCentroids[b.id]; return c ? c.map((v, i) => v + (b.offset ? b.offset[i] : 0)) : (b.pos || [0, 0, 0]); };
function fillCadCard() {
  const cad = airframe.cad;
  $('#cad-use').checked = !!airframe.mass.from_items;
  // An OBJ or STL is triangles: there are no solids to weigh, so the switch that would weigh them is off
  // and out of reach, unless the airframe carries mass items of its own that it could still add up.
  const visual = !!(cad && cad.visual_only) && !(airframe.mass.items || []).length;
  $('#cad-use').disabled = visual;
  $('#cad-use').closest('label').classList.toggle('disabled', visual);
  $('#cad-use').closest('label').title = visual
    ? 'this model is a mesh (OBJ/STL): it has no solids to take a mass from'
    : 'compute the aircraft mass, CG and inertia from the CAD bodies (plus any mass items) instead of the numbers typed above';
  if (cad && cad.file) {
    for (const b of cad.bodies || []) if (b.pos) cadCentroids[b.id] = b.pos.map((v, i) => v - (b.offset ? b.offset[i] : 0));
    $('#cad-show').checked = cad.visible !== false;
    ['rx', 'ry', 'rz'].forEach((k, i) => $('#cad-' + k).value = +(+(cad.rotation_deg || [0, 0, 0])[i]).toFixed(2));
    ['ox', 'oy', 'oz'].forEach((k, i) => $('#cad-' + k).value = +(+(cad.origin || [0, 0, 0])[i]).toFixed(4));
    $('#cad-scale').value = cad.scale ?? 1;
    $('#cad-frame-row').style.display = '';
    $('#cad-remove-all').hidden = false;
    const key = cadKey(cad);
    if (key !== cadMeshKey) loadCadMesh(key);
  } else {
    $('#cad-frame-row').style.display = 'none';
    $('#cad-remove-all').hidden = true;
    if (cadMeshKey) { cadMeshKey = null; scene.setCad(null); }
    cadSelected = null;
  }
  renderCadTable();
}
async function loadCadMesh(key) {
  cadMeshKey = key;
  try {
    const r = await api('/api/cad/mesh');
    if (cadMeshKey !== key) return;      // superseded
    for (const b of r.bodies || []) cadCentroids[b.id] = b.centroid;
    scene.setCad(r);
    if (cadSelected !== null) scene.selectCad(cadSelected);
    renderCadTable();
  } catch (e) { logLine('[cad] mesh: ' + e.message); }
}
function cadRowHtml(b, k) {
  const p = cadPos(b);
  const moved = (b.offset || [0, 0, 0]).some(v => Math.abs(v) > 1e-6);
  return `<tr data-cad="${esc(b.id)}" class="${(scene.selectedCadIds || []).includes(b.id) ? 'selected' : ''}"><td class="idx">${k + 1}</td><td class="mono">${esc(b.name)}</td>
  <td class="num" title="volume from the CAD solid">${(b.volume * 1e3).toFixed(3)}</td>
  <td>${(airframe.cad || {}).visual_only
    ? '<span class="hint">—</span>'
    : `<input type="number" step="0.01" min="0" data-k="mass" value="${+(+b.mass || 0).toFixed(4)}" title="mass of this body, kg (0 = ignored)">`}</td>
  <td class="num pos" title="centroid, structural frame, m${moved ? ' (dragged by ' + b.offset.map(v => v.toFixed(3)).join(', ') + ')' : ''}">${p.map(v => v.toFixed(3)).join('  ')}${moved ? ' <span class="warn" title="moved from the CAD position">•</span>' : ''}</td>
  <td><button class="del" title="remove this body from the list">✕</button></td></tr>`;
}
function renderCadTable() {
  const el = $('#cad-table'); const cad = airframe && airframe.cad;
  if (!cad || !cad.file) { el.innerHTML = ''; $('#cad-totals').innerHTML = ''; $('#cad-summary').textContent = ''; return; }
  const live = (cad.bodies || []).filter(b => !b.removed);
  el.innerHTML = `<table class="grid cad"><thead><tr><th>#</th><th>Body</th><th class="num" title="litres">Vol L</th><th>Mass kg</th><th title="centroid in the structural frame (x fwd, y right, z down), m: click a row to highlight the body, drag it in the 3D view along its axes">X Y Z</th><th></th></tr></thead><tbody>${live.map(cadRowHtml).join('') || '<tr><td colspan="6" class="hint">All bodies removed.</td></tr>'}</tbody></table>`;
  el.querySelectorAll('tr[data-cad]').forEach(tr => {
    const id = tr.dataset.cad;
    tr.addEventListener('click', (e) => { if (['INPUT', 'BUTTON'].includes(e.target.tagName)) return; scene.selectCad(id, e.shiftKey); cadSelected = scene.selectedCad; if (cadSelected !== null && selected >= 0) { selected = -1; renderRotorTable(); renderReadout(); } renderCadTable(); });
    tr.querySelector('input[data-k="mass"]')?.addEventListener('change', (e) => { const b = cadBody(id); b.mass = Math.max(0, parseFloat(e.target.value) || 0); scene.syncCad(); renderCadTotals(); pushAirframe(true); });
    tr.querySelector('.del').addEventListener('click', () => { const b = cadBody(id); b.removed = true; if (cadSelected === id) { cadSelected = null; scene.selectCad(null); } scene.syncCad(); renderCadTable(); pushAirframe(true); });
  });
  renderCadTotals();
}
function renderCadRow(id) {
  const tr = $(`#cad-table tr[data-cad="${CSS.escape(id)}"]`); const b = cadBody(id); if (!tr || !b) return;
  const p = cadPos(b); const cell = tr.querySelector('td.pos'); if (cell) cell.innerHTML = p.map(v => v.toFixed(3)).join('  ') + ' <span class="warn" title="moved from the CAD position">•</span>';
}
function renderCadTotals() {
  const cad = airframe && airframe.cad; const el = $('#cad-totals'); if (!cad || !cad.file) return;
  const live = (cad.bodies || []).filter(b => !b.removed && (+b.mass || 0) > 0);
  const removed = (cad.bodies || []).filter(b => b.removed).length;
  const m = live.reduce((a, b) => a + (+b.mass), 0);
  let cgTxt = '';
  if (m > 0) { const cg = [0, 0, 0]; for (const b of live) { const p = cadPos(b); for (let i = 0; i < 3; i++) cg[i] += p[i] * b.mass / m; } cgTxt = `CG ${cg[0].toFixed(3)}  ${cg[1].toFixed(3)}  ${cg[2].toFixed(3)} m`; }
  const shownBodies = (cad.bodies || []).length - removed;
  el.innerHTML = (cad.visual_only
      ? `<b>${shownBodies} ${shownBodies === 1 ? 'object' : 'objects'}</b> · visual only, no mass`
      : `<b>${m.toFixed(2)} kg</b> · ${live.length} of ${shownBodies} bodies${m > 0 ? ' · ' + cgTxt : ''}`
        + `${airframe.mass.from_items ? ' · <span class="ok">the aircraft follows the bodies</span>' : ''}`)
    + `${removed ? ` · ${removed} removed <a href="#" id="cad-restore">restore</a>` : ''}`;
  const shown = shownBodies;
  $('#cad-summary').textContent = cad.visual_only
    ? `${cad.file.replace(/^airframes\/cad\//, '')} · ${shown} ${shown === 1 ? 'object' : 'objects'} · visual only`
    : `${cad.file.replace(/^airframes\/cad\//, '')} · ${shown} bodies${live.length ? ` (${live.length} with mass)` : ''} · ${m.toFixed(1)} kg`;
  const rs = $('#cad-restore'); if (rs) rs.addEventListener('click', (e) => { e.preventDefault(); cad.bodies.forEach(b => b.removed = false); scene.syncCad(); renderCadTable(); pushAirframe(true); });
}
$('#cad-import').addEventListener('click', () => $('#cad-file').click());
$('#cad-file').addEventListener('change', async (e) => {
  const f = e.target.files && e.target.files[0]; if (!f) return;
  const btn = $('#cad-import'); btn.disabled = true; btn.textContent = 'Importing…';
  try {
    await api('/api/airframe', { airframe, keep_state: true });    // the import attaches to the server's copy
    const r = await fetch('/api/cad/import?filename=' + encodeURIComponent(f.name), { method: 'POST', headers: { 'Content-Type': 'application/octet-stream' }, body: await f.arrayBuffer() });
    const j = await r.json().catch(() => null);
    if (!r.ok || !j || !j.ok) throw new Error((j && j.error) || r.statusText);
    cadMeshKey = cadKey(j.airframe.cad);
    for (const b of j.mesh.bodies || []) cadCentroids[b.id] = b.centroid;
    scene.setCad(j.mesh);
    selected = -1; cadSelected = null;
    setAirframe(j.airframe); markDirty();
    $('#cad-details').open = true;
    logLine(`[cad] imported ${f.name}: ${j.bodies} bodies`);
  } catch (err) { logLine('[cad] import failed: ' + err.message); alert('STEP import failed: ' + err.message); }
  btn.disabled = false; btn.textContent = 'Import STEP…'; e.target.value = '';
});
$('#cad-show').addEventListener('change', (e) => { if (!airframe.cad) return; airframe.cad.visible = e.target.checked; scene.syncCad(); pushAirframe(true); });
$('#cad-use').addEventListener('change', (e) => { const m = airframe.mass;
  if (e.target.checked && !m.from_items) {
    m.manual = structuredClone({ mass: m.mass, cg: m.cg, inertia: m.inertia, inertia_products: m.inertia_products || [0, 0, 0] });
  } else if (!e.target.checked && m.manual) {
    Object.assign(m, structuredClone(m.manual));
  }
  m.from_items = e.target.checked; fillMassCard(); renderCadTotals(); pushAirframe(true); });
async function cadFrameChanged() {
  const cad = airframe.cad; if (!cad) return;
  cad.rotation_deg = ['rx', 'ry', 'rz'].map(k => parseFloat($('#cad-' + k).value) || 0);
  cad.origin = ['ox', 'oy', 'oz'].map(k => parseFloat($('#cad-' + k).value) || 0); cad.scale = Math.max(1e-4, parseFloat($('#cad-scale').value) || 1);
  try {
    const res = await api('/api/airframe', { airframe, keep_state: true });
    if (res.airframe) { airframe.mass = res.airframe.mass; airframe.cad = res.airframe.cad; }
    fillMassCard(); markDirty();
    for (const b of airframe.cad.bodies || []) if (b.pos) cadCentroids[b.id] = b.pos.map((v, i) => v - (b.offset ? b.offset[i] : 0));
    scene.setAirframe(airframe);
    loadCadMesh(cadKey(airframe.cad));
  } catch (err) { logLine('[cad] ' + err.message); }
}
['cad-rx', 'cad-ry', 'cad-rz', 'cad-ox', 'cad-oy', 'cad-oz', 'cad-scale'].forEach(id => $('#' + id).addEventListener('change', cadFrameChanged));
$('#cad-reset-offsets').addEventListener('click', () => { if (!airframe.cad) return; airframe.cad.bodies.forEach(b => b.offset = [0, 0, 0]); scene.syncCad(); renderCadTable(); pushAirframe(true); });
$('#cad-remove-all').addEventListener('click', (e) => {
  const cad = airframe.cad;
  if (!cad) return;
  const n = (cad.bodies || []).length;
  if (!confirmTwice('cad-remove', e.currentTarget, `remove the model and its ${n} ${n === 1 ? 'body' : 'bodies'}`)) return;
  airframe.cad = null; cadSelected = null; scene.selectCad(null);
  // With the model gone, mass from bodies has nothing to add up unless there are mass items of their own.
  if (airframe.mass.from_items && !(airframe.mass.items || []).length) airframe.mass.from_items = false;
  setAirframe(airframe); pushAirframe(true);
  logLine('[cad] removed ' + (cad.file || 'the model'));
});

function bindNumber(id, fn) {
  $('#' + id).addEventListener('change', (e) => { fn(parseFloat(e.target.value) || 0); scene.setAirframe(airframe); pushAirframe(true); });
}
// Editing the name renames this design, file and all: it does not leave the old one behind and start a new one.
$('#af-name').addEventListener('change', async e => {
  const name = e.target.value.trim();
  if (!name || name === airframe.name) { e.target.value = airframe.name; return; }
  try {
    const r = await api('/api/airframe/rename', { name });
    airframe.name = r.name;
    logLine('[preset] renamed to ' + r.file);
  } catch (err) { e.target.value = airframe.name; logLine('[preset] ' + err.message); }
  await loadPresetList(); await loadVersionList();
});
$('#af-duplicate').addEventListener('click', async (e) => {
  e.currentTarget.disabled = true;
  try {
    const r = await api('/api/airframe/duplicate', {});
    selected = -1; setAirframe(r.airframe); showProblems(r.problems); showHover(r.hover);
    currentVersion = '';
    await loadPresetList(); await loadVersionList();
    logLine('[preset] duplicated as ' + r.airframe.name);
    $('#af-name').focus(); $('#af-name').select();
  } catch (err) { logLine('[preset] ' + err.message); }
  e.currentTarget.disabled = false;
});
bindNumber('af-mass', v => airframe.mass.mass = v);
bindNumber('af-cgx', v => airframe.mass.cg[0] = v);
bindNumber('af-cgy', v => airframe.mass.cg[1] = v);
bindNumber('af-cgz', v => airframe.mass.cg[2] = v);
bindNumber('af-battery', v => { airframe.design = airframe.design || {}; airframe.design.battery_wh = Math.max(0, v); });
bindNumber('af-ixx', v => airframe.mass.inertia[0] = v);
bindNumber('af-iyy', v => airframe.mass.inertia[1] = v);
bindNumber('af-izz', v => airframe.mass.inertia[2] = v);
bindNumber('af-hover', v => airframe.hover_pitch_deg = v);

bindNumber('af-bx', v => airframe.body.size[0] = v);
bindNumber('af-by', v => airframe.body.size[1] = v);
bindNumber('af-bz', v => airframe.body.size[2] = v);
bindNumber('af-dragx', v => airframe.body.drag_quadratic[0] = v);
bindNumber('af-dragy', v => airframe.body.drag_quadratic[1] = v);
bindNumber('af-dragz', v => airframe.body.drag_quadratic[2] = v);
bindNumber('af-adragx', v => airframe.body.drag_angular[0] = v);
bindNumber('af-adragy', v => airframe.body.drag_angular[1] = v);
bindNumber('af-adragz', v => airframe.body.drag_angular[2] = v);
bindNumber('af-dcx', v => airframe.body.drag_center[0] = v);
bindNumber('af-dcy', v => airframe.body.drag_center[1] = v);
bindNumber('af-dcz', v => airframe.body.drag_center[2] = v);
$('#af-estimate').addEventListener('click', async () => {
  try {
    await api('/api/airframe', { airframe, keep_state: true });
    const r = await api('/api/airframe/estimate_inertia', {});
    airframe.mass.inertia = r.inertia; setAirframe(airframe);
  } catch (e) { logLine('[ui] estimate failed: ' + e.message); }
});
// There is no Save button. Every accepted edit, on any tab, is written straight back to the preset file by
// the server; this just shows that it happened, so "saved" is a fact on screen rather than a thing to remember.
let savedMarkTimer = null;
function markSaved(text = 'saved', ms = 1200) {
  const el = $('#toast');
  if (!el) return;
  el.textContent = text;
  el.classList.add('on');
  clearTimeout(savedMarkTimer);
  savedMarkTimer = setTimeout(() => el.classList.remove('on'), ms);
}

// Which file a design is kept in: the same slug the server uses for autosave (server/app.py preset_path).
const presetFile = (name) => (String(name || '').trim().toLowerCase().replace(/[^a-z0-9]+/g, '_').replace(/^_+|_+$/g, '')
  || 'untitled') + '.json';
let presetFiles = [], currentVersion = '';
async function loadPresetList() {
  const r = await api('/api/airframes');
  presetFiles = r.files || [];
  const sel = $('#af-preset');
  const cur = airframe ? presetFile(airframe.name) : '';
  // The closed control is a bare chevron: the name beside it already says which design is open.
  sel.innerHTML = '<option value="" hidden selected></option>'
    + '<option value="__new__">New design (empty)</option>'
    + presetFiles.map(f => `<option value="${esc(f)}">${esc(f.replace(/\.json$/, ''))}${f === cur ? ' ✓' : ''}</option>`).join('');
}
async function openDesign(name) {
  const r = await api('/api/airframe/load', { name });
  selected = -1; setAirframe(r.airframe); showProblems(r.problems); showHover(r.hover);
  currentVersion = '';
  await loadPresetList(); await loadVersionList();
  return r;
}
$('#af-preset').addEventListener('change', async (e) => {
  const name = e.target.value;
  e.target.value = '';
  if (!name) return;
  try {
    if (name === '__new__') {
      const r = await api('/api/airframe/new', {});
      selected = -1; setAirframe(r.airframe); showProblems(r.problems); showHover(r.hover);
      currentVersion = '';
      await loadPresetList(); await loadVersionList();
      logLine('[preset] new design ' + r.airframe.name + ': no rotors yet, four legs to stand on');
      $('#af-name').focus(); $('#af-name').select();
    } else await openDesign(name);
  } catch (err) { logLine('[ui] load failed: ' + err.message); }
});
$('#af-delete').addEventListener('click', async (e) => {
  const b = e.currentTarget, cur = presetFile(airframe && airframe.name);
  const next = presetFiles.find(f => f !== cur);
  if (!next) { logLine('[preset] this is the only design there is; make another one before deleting it.'); return; }
  if (!presetFiles.includes(cur)) { logLine('[preset] this design has no file of its own yet.'); return; }
  if (!confirmTwice('af-delete', b, 'delete ' + (airframe.name || cur))) return;
  try {
    await openDesign(next);                      // step off it first: autosave would write a deleted file back
    await api('/api/airframe/delete', { name: cur });
    logLine('[preset] deleted ' + cur);
    await loadPresetList();
  } catch (err) { logLine('[preset] ' + err.message); }
});

// --- rotor helpers: axis <-> tilt/direction
// Tilt: forward lean of the thrust axis from vertical (negative = backward). Cant: sideways lean, positive = outward
// (away from the centreline; for a rotor on the centreline positive = right).
const outward = (y) => (y < 0 ? -1 : 1);
const axisToTilt = (a, y = 1) => {
  const n = Math.hypot(a[0], a[1], a[2]) || 1;
  const tilt = deg(Math.asin(Math.max(-1, Math.min(1, a[0] / n))));
  const cantRight = (Math.hypot(a[1], a[2]) < 1e-6) ? 0 : deg(Math.atan2(a[1], -a[2]));
  return [tilt, cantRight * outward(y)];
};
const tiltToAxis = (tilt, cant, y = 1) => {
  const t = tilt * Math.PI / 180, c = cant * outward(y) * Math.PI / 180;
  return [+Math.sin(t).toFixed(4), +(Math.sin(c) * Math.cos(t)).toFixed(4), +(-Math.cos(c) * Math.cos(t)).toFixed(4)];
};
const esc = (s) => String(s ?? '').replace(/&/g, '&amp;').replace(/"/g, '&quot;').replace(/</g, '&lt;');
const nf = (v, d = 3) => (typeof v === 'number' && isFinite(v)) ? +v.toFixed(d) : (v ?? '');

// The order the tables are drawn in: a tied couple is kept together, whatever order the two happen to sit
// in the data. Only the drawing is reordered — every row keeps its real index, so the output numbering,
// the PX4 export and everything else are untouched by how the list looks.

function renderRotorTable() {
  const el = $('#rotor-table');
  const rows = pairedOrder(airframe.rotors).map(i => rotorRowHtml(i, airframe.rotors[i])).join('');
  el.innerHTML = `<table class="grid"><thead><tr><th title="Physical PWM port: 1–8 = MAIN 1–8; 9–10 = AUX 1–2">#</th><th title="enabled: takes part in the simulation and the PX4 export">On</th><th title="tied to the rotor on the other side: moving one moves the other. They keep their own name, output and spin">Sym</th><th>Name</th><th title="position, m, structural frame (FRD like PX4): X forward">X</th><th title="Y right (+)">Y</th><th title="Z DOWN (+): a rotor 10 cm above the reference point has Z = -0.10">Z</th><th title="forward lean of the thrust axis from vertical, degrees; negative = backward">Tilt°</th><th title="sideways lean, degrees; positive = outward from the centreline (right for a rotor on the centreline)">Cant°</th><th title="resulting unit thrust vector = CA_ROTORn_AX / AY / AZ">Axis AX AY AZ</th><th title="direction of rotation; click a cell to flip it (the sign of KM)">Spin</th><th title="max thrust N">Tmax</th><th title="share of max thrust this rotor needs to hover, as PX4's allocator would solve it">Hover</th><th></th></tr></thead><tbody>${rows || '<tr><td colspan="14" class="hint">No rotors. Add one below.</td></tr>'}</tbody></table>`;
  api('/api/airframe/hover_check').then(showHover).catch(() => { });
  el.querySelectorAll('tr[data-i]').forEach(tr => {
    const i = +tr.dataset.i;
    tr.addEventListener('click', (e) => { if (!['INPUT', 'SELECT', 'OPTION', 'BUTTON'].includes(e.target.tagName) && !e.target.classList.contains('spin') && !e.target.classList.contains('del')) { selected = i; scene.select(i); renderRotorTable(); renderReadout(); } });
    tr.querySelectorAll('input:not(.sym), select:not(.rotor-port)').forEach(inp => inp.addEventListener('change', () => applyRow(i, tr)));
    tr.querySelector('input.sym').addEventListener('change', (e) => toggleRotorPair(i, e.target.checked));
    tr.querySelector('.rotor-port').addEventListener('change', e => changeMotorPort(e.target));
    tr.querySelector('.spin').addEventListener('click', () => { airframe.rotors[i].km = -airframe.rotors[i].km; scene.updateRotorNode(i, airframe.rotors[i]); renderRotorTable(); pushAirframe(true); });
    tr.querySelector('.del').addEventListener('click', () => { unpair(airframe.rotors, i);
      airframe.rotors.splice(i, 1); selected = -1; setAirframe(airframe); pushAirframe(true); });
  });
}
const axisText = (a) => a.map(v => (v >= 0 ? ' ' : '') + v.toFixed(2)).join(' ');
function rotorRowHtml(i, r) {
  const ccw = r.km >= 0;
  const [tilt, cant] = axisToTilt(r.axis, r.pos[1]);
  const pcls = r.pair ? (twinOf(airframe.rotors, i) > i ? ' pair-top' : ' pair-bottom') : '';
  return `<tr data-i="${i}" class="${i === selected ? 'selected' : ''} ${r.enabled === false ? 'off' : ''}${pcls}"><td class="idx">${rotorPortHtml(i, r)}</td>
  <td><input type="checkbox" data-k="on" ${r.enabled === false ? '' : 'checked'}></td>
  <td><input type="checkbox" class="sym" data-k="sym" ${r.pair ? 'checked' : ''} title="${r.pair ? 'tied to its opposite number' : 'tie this rotor to the one on the other side'}"></td>
  <td><input type="text" class="name" data-k="name" value="${esc(r.name)}"></td>
  <td><input type="number" step="0.005" data-k="x" value="${r.pos[0]}"></td>
  <td><input type="number" step="0.005" data-k="y" value="${r.pos[1]}"></td>
  <td><input type="number" step="0.005" data-k="z" value="${r.pos[2]}"></td>
  <td><input type="number" step="1" data-k="tilt" value="${+tilt.toFixed(1)}"></td>
  <td><input type="number" step="1" data-k="cant" value="${+cant.toFixed(1)}"></td>
  <td class="axis" title="CA_ROTOR${i}_AX / AY / AZ">${axisText(r.axis)}</td>
  <td><span class="spin ${ccw ? 'ccw' : 'cw'}" title="click to flip (KM=${r.km})">${ccw ? 'CCW' : 'CW'}</span></td>
  <td><input type="number" step="0.5" data-k="tmax" value="${r.max_thrust}"></td>
  <td class="hover"></td>
  <td><button class="del" title="remove rotor">✕</button></td></tr>`;
}
function renderRotorRow(i) {
  const tr = document.querySelector(`#rotor-table tr[data-i="${i}"]`);
  if (!tr) return;
  const r = airframe.rotors[i];
  const set = (k, v) => { const inp = tr.querySelector(`input[data-k="${k}"]`); if (inp && document.activeElement !== inp) inp.value = v; };
  const [tilt, cant] = axisToTilt(r.axis, r.pos[1]);
  set('x', r.pos[0]); set('y', r.pos[1]); set('z', r.pos[2]); set('tilt', +tilt.toFixed(1)); set('cant', +cant.toFixed(1));
  const ax = tr.querySelector('td.axis'); if (ax) ax.textContent = axisText(r.axis);
}
function applyRow(i, tr) {
  const r = airframe.rotors[i];
  const g = (k) => parseFloat(tr.querySelector(`input[data-k="${k}"]`).value) || 0;
  r.enabled = tr.querySelector('input[data-k="on"]').checked;
  r.name = tr.querySelector('input[data-k="name"]').value.trim() || ('M' + (i + 1));
  r.pos = [g('x'), g('y'), g('z')];
  r.axis = tiltToAxis(g('tilt'), g('cant'), r.pos[1]);
  r.max_thrust = g('tmax');
  const twin = twinOf(airframe.rotors, i);
  if (twin >= 0) { mirrorRotorOnto(r, airframe.rotors[twin]); scene.updateRotorNode(twin, airframe.rotors[twin]); }
  tr.classList.toggle('off', !r.enabled);
  scene.updateRotorNode(i, r);
  renderRotorRow(i);
  renderRotorTable();
  renderReadout();
  pushAirframe(true);
}
function toggleRotorPair(i, on) {
  const r = airframe.rotors[i];
  if (!r) return;
  if (!on) { unpair(airframe.rotors, i); setAirframe(airframe); pushAirframe(true); return; }
  const why = makePair(airframe.rotors, i, mirrorRotorOnto, (x) => x.pos, (src) => {
    const n = JSON.parse(JSON.stringify(src));
    n.name = (src.name || 'M') + ' R';
    n.km = -src.km;                       // a new opposite number spins the other way; change it if you want
    n.pair = '';
    return n;
  });
  if (why) { logLine('[rotors] ' + (r.name || 'this rotor') + ' cannot be mirrored: ' + why); renderRotorTable(); return; }
  selected = i;
  setAirframe(airframe); scene.select(i); pushAirframe(true);
}
function renderReadout() {
  const el = $('#rotor-readout');
  if (selected < 0 || !airframe.rotors[selected]) { el.classList.remove('show'); return; }
  const r = airframe.rotors[selected];
  const [tilt, cant] = axisToTilt(r.axis, r.pos[1]);
  const [dt, dc] = mountError(r);
  const built = (dt || dc) ? (([bt, bc]) => ` · <span class="warn">built ${bt.toFixed(1)}° / ${bc.toFixed(1)}°</span>`)(axisToTiltCant(builtAxis(r), r.pos[1])) : '';
  el.classList.add('show');
  el.innerHTML = `<b>${esc(r.name || 'Motor ' + (selected + 1))}</b> pos [${r.pos.map(v => fmt(v)).join(', ')}] · axis [${r.axis.map(v => fmt(v, 3)).join(', ')}] (tilt ${tilt.toFixed(1)}°, cant ${cant.toFixed(1)}°)${built} · ${r.km >= 0 ? 'CCW' : 'CW'}${r.enabled === false ? ' · disabled' : ''}`;
}
$('#rotor-add').addEventListener('click', () => {
  const base = airframe.rotors[selected] || airframe.rotors[airframe.rotors.length - 1] || { pos: [0.2, 0, 0], axis: [0, 0, -1], km: 0.05, max_thrust: 8, max_power_w: 0, tau: 0.04, diameter: 0.25, thrust_exponent: 2, enabled: true };
  const n = JSON.parse(JSON.stringify(base));
  n.pos = [n.pos[0] + 0.05, n.pos[1] + 0.05, n.pos[2]];
  n.name = 'M' + (airframe.rotors.length + 1); n.enabled = true; n.pair = '';
  airframe.rotors.push(n); selected = airframe.rotors.length - 1;
  setAirframe(airframe); scene.select(selected); pushAirframe(true);
});

// ============================================================ mounting errors
// The one place where the aircraft and the model of it are allowed to disagree. `axis` stays the design -
// the Rotors table, the export and PX4 all keep it - and the error here turns the axis the simulation
// actually pushes along. The limits beside each box come from the server (`/api/airframe/mount_margin`):
// how far that rotor could be out before the hover has nothing left to spare, and how far before there is
// no mix of thrusts that holds the aircraft at all.
let mountMargin = null, mountTimer = 0;
// One number, and it is the honest one: how far this rotor can be out *either* way before the hover has
// nothing to spare. The two sides are rarely the same and the whole story is in the tooltip.
const DEG = (v) => (v < 0 ? '−' : '+') + Math.abs(v).toFixed(0) + '°';
const mountRange = (pair, cap) => !pair ? '—'
  : (Math.abs(pair[0]) >= cap && pair[1] >= cap) ? `beyond ±${cap}°`
  : `±${Math.min(Math.abs(pair[0]), Math.abs(pair[1])).toFixed(0)}°`;
function mountTitle(row, axis, cap) {
  if (!row || !row[axis]) return row && row.why ? row.why : '';
  const side = (p) => `${Math.abs(p[0]) >= cap ? 'beyond ' + DEG(-cap) : DEG(p[0])} to ${p[1] >= cap ? 'beyond ' + DEG(cap) : DEG(p[1])}`;
  return `room to spare: ${side(row[axis])}\nstill hovers at all: ${side(row[axis + '_wall'] || row[axis])}`
    + (row.why ? `\npast that, ${row.why}` : '');
}

function mountZone(v, row, axis) {          // fine / tight / past it, for one typed angle
  if (!v || !row || !row[axis]) return '';
  const soft = row[axis], hard = row[axis + '_wall'] || soft;
  if (v < hard[0] || v > hard[1]) return 'bad';
  if (v < soft[0] || v > soft[1]) return 'warn';
  return '';
}
function renderMountTable() {
  const el = $('#mount-table');
  if (!el || !airframe) return;
  const cap = (mountMargin && mountMargin.limit_deg) || 45;
  const rowFor = (i) => (mountMargin && mountMargin.rotors || []).find(x => x.rotor === i);
  const body = pairedOrder(airframe.rotors).map(i => {
    const r = airframe.rotors[i], m = rowFor(i);
    const [dt, dc] = mountError(r);
    const [bt, bc] = axisToTiltCant(builtAxis(r), r.pos[1]);
    return `<tr data-i="${i}" class="${r.enabled === false ? 'off' : ''}">
      <td>${esc(r.name || 'M' + (i + 1))}</td>
      <td><input type="number" step="1" data-k="tilt" class="${mountZone(dt, m, 'tilt')}" value="${+dt}"></td>
      <td class="hint num" title="${esc(mountTitle(m, 'tilt', cap))}">${esc(mountRange(m && m.tilt, cap))}</td>
      <td><input type="number" step="1" data-k="cant" class="${mountZone(dc, m, 'cant')}" value="${+dc}"></td>
      <td class="hint num" title="${esc(mountTitle(m, 'cant', cap))}">${esc(mountRange(m && m.cant, cap))}</td>
      <td class="num">${bt.toFixed(1)}° / ${bc.toFixed(1)}°</td></tr>`;
  }).join('');
  el.innerHTML = `<table class="grid"><thead><tr><th>Rotor</th>
    <th title="degrees added to this rotor's designed tilt by the way it is actually mounted">Δ Tilt°</th>
    <th title="how far out this rotor could be before the hover has nothing left to spare, and past the second figure nothing holds the aircraft at all">Allowed</th>
    <th title="degrees added to this rotor's designed cant">Δ Cant°</th>
    <th title="how far out this rotor could be before the hover has nothing left to spare">Allowed</th>
    <th title="the tilt and cant the rotor really has: the design plus the error">As built</th></tr></thead>
    <tbody>${body || '<tr><td colspan="6" class="hint">No rotors.</td></tr>'}</tbody></table>`;
  el.querySelectorAll('tr[data-i] input').forEach(inp => inp.addEventListener('change', (e) => {
    const i = +e.target.closest('tr').dataset.i, r = airframe.rotors[i];
    const v = parseFloat(e.target.value) || 0;
    // the error is this rotor's own: a mirrored couple shares its geometry, not its bad luck
    if (e.target.dataset.k === 'tilt') r.tilt_error_deg = v; else r.cant_error_deg = v;
    scene.updateRotorNode(i, r);
    renderMountTable(); renderReadout(); pushAirframe(true); loadMountMargin(true);
  }));
  showMountNote();
}
function showMountNote() {
  const off = (airframe.rotors || []).filter(r => mountError(r).some(v => v));
  const note = $('#mount-note'), why = $('#mount-why');
  if (note) {
    note.textContent = !off.length ? 'all mounted true'
      : off.map(r => `${r.name || 'rotor'} ${[mountError(r)[0] ? mountError(r)[0] + '° tilt' : '', mountError(r)[1] ? mountError(r)[1] + '° cant' : ''].filter(Boolean).join(', ')}`).join(' · ');
    note.classList.toggle('bad', !!off.length);
  }
  if (why) {
    const b = mountMargin && mountMargin.built;
    why.textContent = !b ? '' : b.ok
      ? `As built it still hovers: the busiest motor is at ${(b.worst * 100).toFixed(0)}%${b.lean_deg > 0.5 ? `, leaning ${b.lean_deg.toFixed(0)}°` : ''}.`
      : `As built it cannot hover: ${b.why}.`;
    why.classList.toggle('bad', !!(b && !b.ok));
  }
}
function loadMountMargin(soon = false) {
  if (!$('#sec-mount') || !$('#sec-mount').open) return;
  clearTimeout(mountTimer);
  mountTimer = setTimeout(async () => {
    try { mountMargin = await api_get('/api/airframe/mount_margin'); } catch { mountMargin = null; }
    renderMountTable();
  }, soon ? 250 : 0);
}
$('#sec-mount') && $('#sec-mount').addEventListener('toggle', () => loadMountMargin());
$('#mount-clear') && $('#mount-clear').addEventListener('click', () => {
  airframe.rotors.forEach((r, i) => {
    if (!mountError(r).some(v => v)) return;
    r.tilt_error_deg = 0; r.cant_error_deg = 0;
    scene.updateRotorNode(i, r);
  });
  renderMountTable(); renderReadout(); pushAirframe(true); loadMountMargin(true);
});

// ============================================================ wings
const WING_AERO_DEFAULT = { model: 'linear', cl_alpha: 6.2832, cl0: 0, cd0: 0.02, oswald: 0.85, stall_deg: 15, stall_blend_deg: 15, cd_flat: 1.2, cm0: 0, vortex_lift: true };
const WING_DEFAULT = { name: 'wing', enabled: true, pos: [0.1, 0, 0], span: 1.0, root_chord: 0.3, tip_chord: 0.2, sweep_deg: 5, dihedral_deg: 2, incidence_deg: 3, twist_deg: 0, symmetric: true, panels: 6, aspect_ratio: null };
const FIN_DEFAULT = { name: 'fin', enabled: true, pos: [-0.5, 0, -0.02], span: 0.2, root_chord: 0.16, tip_chord: 0.08, sweep_deg: 30, dihedral_deg: 90, incidence_deg: 0, twist_deg: 0, symmetric: false, panels: 3, aspect_ratio: null };
const wingHalfSpan = (w) => (w.symmetric === false ? +w.span : +w.span / 2) || 0;
const wingArea = (w) => (+w.root_chord + +w.tip_chord) / 2 * wingHalfSpan(w) * (w.symmetric === false ? 1 : 2);
const wingAR = (w) => { const a = wingArea(w); if (!(a > 1e-9)) return 0; if (w.aspect_ratio) return +w.aspect_ratio; const b = w.symmetric === false ? 2 * +w.span : +w.span; return b * b / a; };
const wingsOpen = new Set();   // indices whose coefficient row is expanded
function renderWingTable() {
  const el = $('#wing-table');
  const rows = airframe.wings.map((w, i) => wingRowHtml(i, w)).join('');
  el.innerHTML = `<table class="grid wings"><thead><tr><th>On</th><th>Name</th><th title="root leading edge, structural frame, m">X</th><th title="symmetric: how far the root sits out from the centreline, the same on both sides. Single panel: the panel's own y">Y</th><th title="Z DOWN (+), like PX4: a wing above the reference point has negative Z">Z</th><th title="tip to tip (symmetric) or panel length (single panel), m">Span</th><th title="root chord, m">Root</th><th title="tip chord, m">Tip</th><th title="leading-edge sweep, back positive">Swp°</th><th title="whole-wing pitch: the entire wing rotated rigidly about the aircraft pitch axis through its root point, nose-up positive (with sweep the tips move up/down with it)">Pitch°</th><th title="dihedral, tips up positive; 90 + single panel = vertical fin">Dih°</th><th title="section incidence at the root: each section turned about the span line, leading edge up positive">Inc°</th><th title="extra incidence at the tip (washout negative)">Tw°</th><th title="symmetric: left + right halves">Sym</th><th>Model</th><th title="stall angle of attack">Stall°</th><th title="planform area m² · aspect ratio">Area · AR</th><th></th></tr></thead>
    <tbody>${rows || '<tr><td colspan="17" class="hint">No wings. A pure multirotor needs none.</td></tr>'}</tbody></table>`;
  el.querySelectorAll('tr[data-w]').forEach(tr => {
    const i = +tr.dataset.w;
    tr.querySelectorAll('input, select').forEach(inp => inp.addEventListener('change', () => applyWingRow(i)));
    tr.querySelector('.del').addEventListener('click', () => { airframe.wings.splice(i, 1); wingsOpen.clear(); renderWingTable(); scene.setAirframe(airframe); pushAirframe(true); });
    tr.querySelector('.expand').addEventListener('click', () => { if (wingsOpen.has(i)) wingsOpen.delete(i); else wingsOpen.add(i); renderWingTable(); });
  });
  el.querySelectorAll('.polar-build').forEach(b => b.addEventListener('click', async () => {
    const i = +b.dataset.w, a = airframe.wings[i].aero, info = el.querySelector(`.polar-info[data-w="${i}"]`);
    applyWingCoeffs(i);
    const names = [...new Set([a.airfoil_root, a.airfoil_tip].filter(Boolean))];
    if (!names.length) { info.textContent = 'enter a root airfoil first'; return; }
    b.disabled = true; info.textContent = 'building polar' + (names.length > 1 ? 's' : '') + ' (XFOIL / NeuralFoil)…';
    const out = [];
    for (const n of names) {
      try {
        const r = await api('/api/airfoil/polar', { name: n, source: a.polar_source || 'auto', ncrit: a.ncrit ?? 9, force: false });
        const sm = r.summary;
        out.push(`${r.name} (${r.source}${r.note ? ', ' + r.note : ''}): CLmax ${sm.cl_max} @ ${sm.alpha_cl_max_deg}°, zero lift ${sm.alpha_zero_lift_deg}°, CDmin ${sm.cd_min}, L/D max ${sm.ld_max} @ ${sm.alpha_ld_max_deg}°, Cm0 ${sm.cm0} (Re ${(sm.re / 1e6).toFixed(1)}M)`);
      } catch (e) { out.push(`${n}: ${e.message}`); }
    }
    info.textContent = out.join(' · ');
    b.disabled = false;
    pushAirframe(true);
  }));
  el.querySelectorAll('tr[data-wc]').forEach(tr => {
    const i = +tr.dataset.wc;
    tr.querySelectorAll('input, select').forEach(inp => inp.addEventListener('change', () => applyWingCoeffs(i)));
  });
}
function wingRowHtml(i, w) {
  const a = w.aero || WING_AERO_DEFAULT;
  const n = (k, step, v) => `<input type="number" step="${step}" data-k="${k}" value="${nf(v, 4)}">`;
  const main = `<tr data-w="${i}" class="${w.enabled === false ? 'off' : ''}">
    <td><input type="checkbox" data-k="on" ${w.enabled === false ? '' : 'checked'}></td>
    <td><input type="text" class="name" data-k="name" value="${esc(w.name)}"></td>
    <td>${n('x', 0.01, w.pos[0])}</td><td>${n('y', 0.01, w.symmetric === false ? w.pos[1] : (w.root_y || 0))}</td><td>${n('z', 0.01, w.pos[2])}</td>
    <td>${n('span', 0.01, w.span)}</td><td>${n('root', 0.01, w.root_chord)}</td><td>${n('tip', 0.01, w.tip_chord)}</td>
    <td>${n('sweep', 1, w.sweep_deg)}</td><td>${n('pitch', 0.5, w.pitch_deg || 0)}</td><td>${n('dih', 1, w.dihedral_deg)}</td><td>${n('inc', 0.5, w.incidence_deg)}</td><td>${n('twist', 0.5, w.twist_deg)}</td>
    <td><input type="checkbox" data-k="sym" ${w.symmetric === false ? '' : 'checked'}></td>
    <td><select data-k="model" title="linear: lift slope + induced drag · polhamus: sharp-edged delta with vortex lift · polar: airfoil section data (XFOIL / NeuralFoil / UIUC) looked up at the live angle of attack and Reynolds number"><option value="linear" ${['polhamus', 'polar'].includes(a.model) ? '' : 'selected'}>linear</option><option value="polhamus" ${a.model === 'polhamus' ? 'selected' : ''}>polhamus</option><option value="polar" ${a.model === 'polar' ? 'selected' : ''}>polar (airfoil)</option></select></td>
    <td>${n('stall', 1, a.stall_deg)}</td>
    <td class="calc num" title="area m² · aspect ratio">${wingArea(w).toFixed(3)} · ${wingAR(w).toFixed(2)}</td>
    <td class="nowrap"><button class="expand" title="coefficients">${wingsOpen.has(i) ? '▾' : '▸'}</button><button class="del" title="remove wing">✕</button></td></tr>`;
  if (!wingsOpen.has(i)) return main;
  const c = (k, step, v, title) => `<label title="${title || ''}">${k.replace(/_/g, ' ')} <input type="number" step="${step}" data-k="${k}" value="${nf(v, 5)}"></label>`;
  const polarRow = `<div class="row tight airfoil-row">
    <label title="section at the root: NACA 4/5-digit (naca2412, naca23006), a file in airfoils/&lt;name&gt;.dat, or a UIUC database name (e423, sd7037, clarky …)">root airfoil <input type="text" data-k="airfoil_root" value="${esc(a.airfoil_root || '')}" placeholder="naca23006" style="width:110px"></label>
    <label title="section at the tip (blank = same as root)">tip airfoil <input type="text" data-k="airfoil_tip" value="${esc(a.airfoil_tip || '')}" placeholder="same" style="width:110px"></label>
    <label title="where the polar comes from: auto = XFOIL when installed, else NeuralFoil (a neural surrogate of XFOIL)">source <select data-k="polar_source"><option value="auto" ${(a.polar_source || 'auto') === 'auto' ? 'selected' : ''}>auto</option><option value="neuralfoil" ${a.polar_source === 'neuralfoil' ? 'selected' : ''}>NeuralFoil</option><option value="xfoil" ${a.polar_source === 'xfoil' ? 'selected' : ''}>XFOIL</option></select></label>
    <label title="transition criterion N_crit (9 = clean flow, 5 = turbulent/rough)">n crit <input type="number" step="1" min="1" max="14" data-k="ncrit" value="${a.ncrit ?? 9}" style="width:52px"></label>
    <button class="pill small polar-build" data-w="${i}" title="build (or rebuild) the polar tables and show CL max, stall angle, CD min, L/D max">Build polar</button>
    <span class="hint polar-info" data-w="${i}">${a.model === 'polar' ? 'oswald e below is used for the induced drag; stall comes from the polar itself' : ''}</span>
  </div>`;
  return main + `<tr data-wc="${i}" class="coeffs"><td colspan="18">${a.model === 'polar' ? polarRow : ''}<div class="row tight">
    ${c('cl_alpha', 0.1, a.cl_alpha, '2-D lift slope, 1/rad (2π thin airfoil)')}
    ${c('cl0', 0.05, a.cl0, 'lift at zero geometric angle of attack')}
    ${c('cd0', 0.005, a.cd0, 'parasite drag')}
    ${c('oswald', 0.05, a.oswald, 'span efficiency for induced drag (linear model)')}
    ${c('cm0', 0.01, a.cm0, 'pitching moment about the quarter chord, nose-up positive')}
    ${c('stall_blend_deg', 1, a.stall_blend_deg, 'width of the blend into flat-plate behaviour')}
    ${c('cd_flat', 0.1, a.cd_flat, 'flat-plate drag past the stall')}
    <label title="Mirrored distance from the centerline to each segment root, m">root offset <input type="number" step="0.01" min="0" data-k="root_y" value="${w.root_y ?? 0}"></label>
    <label title="spanwise strips per half">panels <input type="number" step="1" min="1" max="40" data-k="panels" value="${w.panels ?? 6}"></label>
    <label title="override the geometric aspect ratio used for lift slope / induced drag (blank = geometric)">AR override <input type="number" step="0.1" min="0" data-k="ar" value="${w.aspect_ratio ?? ''}"></label>
    <label class="row" style="align-self:end;padding-bottom:6px" title="polhamus: add the vortex-lift term"><input type="checkbox" data-k="vortex" ${a.vortex_lift === false ? '' : 'checked'}> vortex lift</label>
  </div></td></tr>`;
}
function applyWingRow(i) {
  const tr = $(`#wing-table tr[data-w="${i}"]`), w = airframe.wings[i]; if (!tr || !w) return;
  const g = (k) => parseFloat(tr.querySelector(`[data-k="${k}"]`).value) || 0;
  w.enabled = tr.querySelector('[data-k="on"]').checked;
  w.name = tr.querySelector('[data-k="name"]').value.trim() || ('wing' + (i + 1));
  w.symmetric = tr.querySelector('[data-k="sym"]').checked;
  // A symmetric wing is two halves about the centreline. Y is how far its root sits out from that line,
  // the same on both sides; writing it into pos would slide the pair sideways and leave it off centre.
  if (w.symmetric) { w.root_y = g('y'); w.pos = [g('x'), 0, g('z')]; }
  else { w.root_y = 0; w.pos = [g('x'), g('y'), g('z')]; }
  w.span = Math.max(0, g('span')); w.root_chord = Math.max(0, g('root')); w.tip_chord = Math.max(0, g('tip'));
  w.sweep_deg = g('sweep'); w.pitch_deg = g('pitch'); w.dihedral_deg = g('dih'); w.incidence_deg = g('inc'); w.twist_deg = g('twist');
  w.aero = w.aero || { ...WING_AERO_DEFAULT };
  const newModel = tr.querySelector('[data-k="model"]').value;
  const modelChanged = newModel !== w.aero.model;
  w.aero.model = newModel;
  w.aero.stall_deg = g('stall');
  if (modelChanged) { if (newModel === 'polar') wingsOpen.add(i); setTimeout(renderWingTable, 0); }
  tr.classList.toggle('off', !w.enabled);
  const calc = tr.querySelector('td.calc'); if (calc) calc.textContent = `${wingArea(w).toFixed(3)} · ${wingAR(w).toFixed(2)}`;
  scene.setAirframe(airframe); pushAirframe(true);
}
function applyWingCoeffs(i) {
  const tr = $(`#wing-table tr[data-wc="${i}"]`), w = airframe.wings[i]; if (!tr || !w) return;
  const a = w.aero = w.aero || { ...WING_AERO_DEFAULT };
  const g = (k) => parseFloat(tr.querySelector(`[data-k="${k}"]`).value);
  for (const k of ['cl_alpha', 'cl0', 'cd0', 'oswald', 'cm0', 'stall_blend_deg', 'cd_flat']) { const v = g(k); if (isFinite(v)) a[k] = v; }
  a.vortex_lift = tr.querySelector('[data-k="vortex"]').checked;
  const ar_ = tr.querySelector('[data-k="airfoil_root"]'), at_ = tr.querySelector('[data-k="airfoil_tip"]'), src = tr.querySelector('[data-k="polar_source"]'), nc = tr.querySelector('[data-k="ncrit"]');
  if (ar_) a.airfoil_root = ar_.value.trim().toLowerCase().replace(/\s+/g, '');
  if (at_) a.airfoil_tip = at_.value.trim().toLowerCase().replace(/\s+/g, '');
  if (src) a.polar_source = src.value;
  if (nc && isFinite(parseFloat(nc.value))) a.ncrit = parseFloat(nc.value);
  w.root_y = Math.max(0, g('root_y') || 0);
  w.panels = Math.max(1, Math.round(g('panels') || 6));
  const ar = g('ar'); w.aspect_ratio = isFinite(ar) && ar > 0 ? ar : null;
  const main = $(`#wing-table tr[data-w="${i}"] td.calc`); if (main) main.textContent = `${wingArea(w).toFixed(3)} · ${wingAR(w).toFixed(2)}`;
  pushAirframe(true);
}
function addWing(tpl) {
  const w = JSON.parse(JSON.stringify(tpl));
  w.aero = { ...WING_AERO_DEFAULT };
  const base = tpl.name; let k = 1; while (airframe.wings.some(x => x.name === w.name)) w.name = base + (++k);
  airframe.wings.push(w);
  renderWingTable(); scene.setAirframe(airframe); pushAirframe(true);
}
$('#wing-add').addEventListener('click', () => addWing(WING_DEFAULT));
$('#fin-add').addEventListener('click', () => addWing(FIN_DEFAULT));

// ============================================================ legs
const LEG_DEFAULT = { name: 'leg', attach: [0.15, 0.15, 0], length: 0.2, tilt_deg: 0, cant_deg: 0, foot_radius: 0, stiffness: 3000, damping: 150, friction: 0.8, enabled: true };
function renderLegTable() {
  const el = $('#leg-table');
  const rows = pairedOrder(airframe.legs).map(i => legRowHtml(i, airframe.legs[i])).join('');
  el.innerHTML = `<table class="grid legs"><thead><tr><th>On</th><th title="tied to the leg on the other side: moving one moves the other">Sym</th><th>Name</th><th title="attachment point on the structure, structural frame, m: X forward">X</th><th title="Y right (+)">Y</th><th title="Z DOWN (+), like PX4: legs usually have positive Z">Z</th><th title="attachment to foot, m">Len</th><th title="lean forward (+) / backward (-) from straight down">Tilt°</th><th title="lean outward (+) / inward (-) from the centreline">Cant°</th><th title="contact starts this far above the foot point (ball or skid tube), m">Foot r</th><th title="N/m">Stiff</th><th title="N/(m/s)">Damp</th><th title="foot point, structural frame">Foot</th><th></th></tr></thead>
    <tbody>${rows || '<tr><td colspan="14" class="hint">No legs. The vehicle needs at least 3 to stand.</td></tr>'}</tbody></table>`;
  el.querySelectorAll('tr[data-l]').forEach(tr => {
    const i = +tr.dataset.l;
    tr.querySelectorAll('input:not(.sym)').forEach(inp => inp.addEventListener('change', () => applyLegRow(i)));
    tr.querySelector('input.sym').addEventListener('change', (e) => toggleLegPair(i, e.target.checked));
    tr.querySelector('.del').addEventListener('click', () => { unpair(airframe.legs, i); airframe.legs.splice(i, 1); renderLegTable(); scene.setAirframe(airframe); pushAirframe(true); });
  });
}
function legRowHtml(i, l) {
  const n = (k, step, v, extra = '') => `<input type="number" step="${step}" data-k="${k}" value="${nf(v, 4)}" ${extra}>`;
  const f = legFoot(l);
  const pcls = l.pair ? (twinOf(airframe.legs, i) > i ? ' pair-top' : ' pair-bottom') : '';
  return `<tr data-l="${i}" class="${l.enabled === false ? 'off' : ''}${pcls}">
    <td><input type="checkbox" data-k="on" ${l.enabled === false ? '' : 'checked'}></td>
    <td><input type="checkbox" class="sym" data-k="sym" ${l.pair ? 'checked' : ''} title="${l.pair ? 'tied to its opposite number' : 'tie this leg to the one on the other side'}"></td>
    <td><input type="text" class="name" data-k="name" value="${esc(l.name)}"></td>
    <td>${n('x', 0.01, l.attach[0])}</td><td>${n('y', 0.01, l.attach[1])}</td><td>${n('z', 0.01, l.attach[2])}</td>
    <td>${n('len', 0.01, l.length, 'min="0"')}</td><td>${n('tilt', 1, l.tilt_deg)}</td><td>${n('cant', 1, l.cant_deg)}</td>
    <td>${n('footr', 0.005, l.foot_radius, 'min="0"')}</td><td>${n('stiff', 100, l.stiffness, 'min="0"')}</td><td>${n('damp', 10, l.damping, 'min="0"')}</td>
    <td class="calc num" title="foot point x y z">${f.map(v => v.toFixed(2)).join(' ')}</td>
    <td><button class="del" title="remove leg">✕</button></td></tr>`;
}
function applyLegRow(i) {
  const tr = $(`#leg-table tr[data-l="${i}"]`), l = airframe.legs[i]; if (!tr || !l) return;
  const g = (k) => parseFloat(tr.querySelector(`[data-k="${k}"]`).value) || 0;
  l.enabled = tr.querySelector('[data-k="on"]').checked;
  l.name = tr.querySelector('[data-k="name"]').value.trim() || ('leg' + (i + 1));
  l.attach = [g('x'), g('y'), g('z')];
  l.length = Math.max(0, g('len')); l.tilt_deg = g('tilt'); l.cant_deg = g('cant');
  l.foot_radius = Math.max(0, g('footr')); l.stiffness = Math.max(0, g('stiff')); l.damping = Math.max(0, g('damp'));
  const twin = twinOf(airframe.legs, i);
  if (twin >= 0) { mirrorLegOnto(l, airframe.legs[twin]); renderLegTable(); }
  tr.classList.toggle('off', !l.enabled);
  const calc = tr.querySelector('td.calc'); if (calc) calc.textContent = legFoot(l).map(v => v.toFixed(2)).join(' ');
  scene.setAirframe(airframe); pushAirframe(true);
}
function toggleLegPair(i, on) {
  const l = airframe.legs[i];
  if (!l) return;
  if (!on) { unpair(airframe.legs, i); renderLegTable(); scene.setAirframe(airframe); pushAirframe(true); return; }
  const why = makePair(airframe.legs, i, mirrorLegOnto, (x) => legFoot(x), (src) => {
    const n = JSON.parse(JSON.stringify(src));
    n.name = (src.name || 'leg') + ' R';
    n.pair = '';
    return n;
  });
  if (why) { logLine('[legs] ' + (l.name || 'this leg') + ' cannot be mirrored: ' + why); renderLegTable(); return; }
  renderLegTable(); scene.setAirframe(airframe); pushAirframe(true);
}
$('#leg-add').addEventListener('click', () => {
  const base = airframe.legs[airframe.legs.length - 1] || LEG_DEFAULT;
  const l = JSON.parse(JSON.stringify(base));
  l.name = 'leg' + (airframe.legs.length + 1); l.enabled = true; l.pair = '';
  if (airframe.legs.length) l.attach = [l.attach[0] - 0.05, l.attach[1], l.attach[2]];
  airframe.legs.push(l);
  renderLegTable(); scene.setAirframe(airframe); pushAirframe(true);
});
$('#leg-auto-btn').addEventListener('click', async () => {
  try {
    await api('/api/airframe', { airframe, keep_state: true });
    const r = await api('/api/airframe/legs/auto', {});
    airframe.legs = r.legs; renderLegTable(); scene.setAirframe(airframe);
    $('#leg-static').textContent = `static sink ${(r.static.compression_m * 100).toFixed(1)} cm · damping ratio ${r.static.zeta.toFixed(2)}`;
    logLine(`[ui] legs sized for ${airframe.mass.mass} kg: k ${r.legs[0].stiffness} N/m, c ${r.legs[0].damping} N/(m/s)`);
    markDirty();
  } catch (e) { logLine('[ui] auto legs: ' + e.message); }
});

// ============================================================ PX4 export
async function loadExport() {
  let r;
  try { r = await api('/api/px4/export'); } catch (e) { return; }
  lastExport = r;
  // the server is the source of truth for edited parameters (they can also be set through the API or by
  // another browser tab); keep this tab's copy in sync so a geometry edit never pushes a stale set back
  if (airframe && r.overrides) airframe.px4_overrides = r.overrides;
  renderOverrides(r);
  $('#px4-export-status').innerHTML = r.problems.length ? `<div class="problems">⚠ ${r.problems.join('<br>⚠ ')}</div>` : '';
}
let lastExport = null;
$('#px4-refresh').addEventListener('click', loadExport);
async function pushToPX4(statusEl, short = false) {
  const fw0 = status.firmware || {};
  const fwStale = !!(fw0.needs_rebuild || fw0.needs_relaunch || fw0.needs_flash);
  statusEl.innerHTML = fwStale ? (fw0.board ? '<span class="muted">Building if needed, flashing the board, then pushing… (a few minutes; watch the log)</span>' : '<span class="muted">Rebuilding firmware, relaunching PX4, then pushing… (watch the log)</span>') : '<span class="muted">Pushing…</span>';
  // The push can take a minute over a radio. Count it out loud rather than leaving a button that looks
  // stuck: the write happens on the server, and this asks it how far it has got while it works.
  let ticking = true;
  const tick = async () => {
    if (!ticking) return;
    try {
      const p = await api_get('/api/px4/push/progress');
      if (ticking && p.running && p.total) {
        statusEl.innerHTML = `<span class="muted">Pushing… ${p.done} of ${p.total}`
          + `${p.failed ? ` · ${p.failed} refused` : ''}</span>`;
      } else if (ticking && p.running && p.step) {
        statusEl.innerHTML = `<span class="muted">${esc(p.step)}…</span>`;
      }
    } catch { }
    if (ticking) setTimeout(tick, 500);
  };
  tick();
  try {
    await api('/api/airframe', { airframe, keep_state: true });
    const r = await api('/api/px4/push', { save: true, firmware: true });
    const failed = r.results.filter(x => !x.ok);
    const fw = r.firmware || {};
    const fwNote = fw.flashed ? `${fw.rebuilt ? 'firmware built and ' : ''}board flashed · ` : (fw.rebuilt ? `firmware rebuilt${fw.build_seconds ? ' in ' + Math.round(fw.build_seconds) + ' s' : ''}, PX4 relaunched · ` : (fw.relaunched ? 'PX4 relaunched on the new firmware · ' : ''));
    // what the design set, and then everything else that was not the simulator's
    const rest = r.rest || {};
    const also = rest.written ? ` + ${rest.written} to match SITL` : '';
    const left = rest.skipped ? ` (${rest.skipped} calibration/RC/safety left alone)` : '';
    statusEl.innerHTML = r.ok
      ? `<span class="ok">✓ ${fwNote}PX4 updated (${r.results.length} parameters${also})${left}</span>`
      : `<span class="err">${failed.length} failed: ${failed.map(f => f.name + ' (' + f.error + ')').join(', ')}</span>`;
    if (rest.why) statusEl.innerHTML += `<div class="muted">the rest were left as they are: ${esc(rest.why)}</div>`;
    if (!short && r.missing && r.missing.length) statusEl.innerHTML += `<div class="muted">not present in this firmware: ${r.missing.join(', ')}</div>`;
    if (failed.length) {
      // the whole list, wrapped and readable, instead of one truncated name
      statusEl.innerHTML = `<span class="err">${failed.length} of ${r.results.length} refused</span>`
        + `<div class="muted" style="white-space:normal">${failed.map(f => esc(f.name)).join(', ')}`
        + ` — ${esc(failed[0].error || 'no answer')}</div>`;
    }
    geometryDirty = false;
    await loadExport();
  } catch (e) { statusEl.innerHTML = `<span class="err">${e.message}</span>`; }
  finally { ticking = false; }
}
$('#btn-update').addEventListener('click', () => {
  if (status.armed) { $('#update-status').innerHTML = '<span class="err">Disarm before updating PX4</span>'; return; }
  pushToPX4($('#update-status'), true);
});
let geometryDirty = false;
function markDirty() { geometryDirty = true; markSaved(); $('#update-status').innerHTML = '<span class="warn">Changed since last update</span>'; }
// The flight controller is running the ATLAS ground-sequence module, so Takeoff and Land are one MAVLink
// command rather than anything this app does. Kept when the simulator-side nose-lift card was removed.
function nativeModuleActive() {
  const has = status.mode === 'sitl' || !!status.native_module;
  const enabled = status.nlf_enable ?? (params.NLF_ENABLE || {}).value ?? ((airframe && airframe.px4_overrides) || {}).NLF_ENABLE;
  return has && +enabled === 1;
}

function updateFooter() {
  const armBtn = $('#btn-arm');
  armBtn.textContent = status.armed ? 'Kill' : 'Arm';
  armBtn.title = status.armed ? 'Force disarm immediately (motors stop, even in the air)' : 'Arm the vehicle';
  armBtn.classList.toggle('armed', !!status.armed);
  armBtn.disabled = (!status.armed && !!status.resetting) || !status.ctl_connected || (!status.armed && !status.arm_ready);
  if (!status.armed && status.ctl_connected && !status.arm_ready) armBtn.title = 'Not armable yet: ' + (status.arm_block_reason || 'estimator not ready');
  const tko = $('#btn-takeoff');
  if (status.mode === 'sitl' && status.ctl_connected && !status.armed &&
      !status.arm_ready && !status.resetting &&
      !prearmReportRequestPending && Date.now() - lastPrearmReportRequest > 10000) {
    lastPrearmReportRequest = Date.now();
    prearmReportRequestPending = true;
    api('/api/px4/shell', { command: 'commander check', timeout: 0.5 })
      .catch(e => logLine('[ui] Could not refresh arming checks: ' + e.message))
      .finally(() => { prearmReportRequestPending = false; });
  }
  const nativeNoseLift = nativeModuleActive();
  const manualPrototype = nativeNoseLift && Number(params.NLF_MANUAL?.value ?? 0) === 1;
  const pilotHandover = nativeNoseLift ? Number(params.NLF_PILOT?.value ?? airframe?.px4_overrides?.NLF_PILOT ?? 0) : 0;   // what the flight controller runs now
  const stagedRc = nativeNoseLift && (manualPrototype || (Number(params.NLF_RC_THR?.value ?? 0) === 1 && !pilotHandover));
  tko.title = nativeNoseLift ? 'PX4 arms, raises the nose to the configured target, then climbs and holds heading' : 'Automatic takeoff';
  tko.textContent = 'Takeoff';
  if (stagedRc) tko.title = 'First press: RC-controlled nose lift. Once nose ready, press again to climb to 1 m. Lower throttle to request abort/landing.';
  if (manualPrototype) tko.title = 'Simulation only: RC nose lift, then second press for 2 m altitude hold with RC power ceiling and attitude control.';
  if (pilotHandover) tko.title = status.armed ? 'Second press (throttle at minimum): you get the throttle; raise it slowly to lift off' : `Arms, raises the nose to level and waits for "nose ready"; a second press gives you the throttle, PX4 (${pilotHandover === 2 ? 'Altitude' : 'Stabilized'}) takes over once airborne`;
  tko.disabled = !!status.resetting || !status.ctl_connected || (nativeNoseLift && status.armed && !stagedRc && !pilotHandover) || (!status.armed && !status.arm_ready);
  { const me = $('#motor-enable'); if (me) { const hitl = status.conn_mode === 'hitl'; me.disabled = hitl && (status.armed || !status.ctl_connected); if (me.disabled && me.checked) { stopMotorTest(); me.checked = false; } me.closest('label') && (me.closest('label').title = hitl ? 'Real motor tests require a connected, disarmed flight controller' : ''); } }
  const landButton = $('#btn-land');
  if (landButton) {
    landButton.textContent = 'Land';
    landButton.disabled = manualPrototype || !status.ctl_connected || !status.armed;
    landButton.title = manualPrototype ? 'Manual prototype has no automatic landing; use RC throttle and attitude controls.' : (nativeNoseLift && pilotHandover) ? 'Before liftoff: lower the nose again. In the air: the module takes the motors back; lower the throttle to descend, the nose is lowered on rear-leg contact' : nativeNoseLift ? 'From native hover: descend onto rear legs, lower the nose at 4 degrees/s, then disarm' : 'Land';
  }
  $('#af-hover').disabled = !!status.armed;
  $('#af-landed').readOnly = true;
  const targetInput = $('#af-takeoff');
  // One nose-lift target, and it is the firmware's: the module that raises the nose in SITL is the same
  // module that raises it on the aircraft, so there is no simulator-side figure to keep beside it.
  targetInput.disabled = !!status.armed || !status.ctl_connected || !status.native_module;
  targetInput.step = 'any';
  targetInput.title = 'PX4 NLF_TARGET: firmware nose-lift target. Editable while connected and disarmed.';
  if (document.activeElement !== targetInput)
    targetInput.value = (params.NLF_TARGET || {}).value ?? airframe.px4_overrides?.NLF_TARGET ?? ((airframe.hover_pitch_deg ?? 25) - 10);

  tko.classList.toggle('hidden', !!status.armed && !status.on_ground_hint && false);
  const upd = $('#btn-update');
  upd.disabled = !status.ctl_connected || !!status.armed;
  const fw = status.firmware || {};
  const fwStale = !!(fw.needs_rebuild || fw.needs_relaunch || fw.needs_flash);
  const onBoard = !!fw.board;
  upd.textContent = fwStale && onBoard ? 'Flash firmware' : 'Update PX4';
  upd.title = status.armed ? 'Disarm first: PX4 rebuilds its allocation when these parameters change' :
    (fwStale ? (onBoard ? `Module sources changed (${(fw.stale || []).slice(0, 3).join(', ') || 'new image'}): builds the ATLAS firmware if needed, flashes the board (archived), then pushes the parameters`
                        : `Firmware sources changed (${(fw.stale || []).slice(0, 3).join(', ') || 'new binary'}): rebuilds PX4 SITL, relaunches it, then pushes the parameters`)
      : (status.ctl_connected ? 'Write the rotor geometry and output mapping to the flight controller and save it' : 'PX4 not connected'));
  upd.classList.toggle('fw-stale', fwStale);
  if (fwStale && !geometryDirty && !$('#update-status').textContent) $('#update-status').innerHTML = onBoard ? '<span class="warn">The board firmware is out of date</span>' : '<span class="warn">Firmware sources changed: Update PX4 rebuilds it</span>';
}
// ============================================================ the parameter table
// Two kinds of row, and the difference is not a matter of taste: a *geometry dependent* parameter is one
// the design works out and keeps working out - move a rotor and it moves - while an *added* one exists
// only because you added it. Editing a geometry row does not promote it out of its section; it stays
// where it is, with the value you typed, what the geometry would say, and one click to go back.
function renderOverrides(r) {
  r = r || lastExport;
  const ov = (r && r.overrides) || (airframe && airframe.px4_overrides) || {};
  if (airframe) airframe.px4_overrides = ov;
  const el = $('#overrides');
  const f = (x) => (typeof x === 'number' && !Number.isInteger(x)) ? +x.toFixed(4) : x;
  const cur = (r && r.current) || {};
  const geo = (r && r.geometry) || {};
  const flight = (r && r.flight) || {};
  const notes = (r && r.flight_notes) || {};
  const P = (r && r.params) || {};
  const same = (a, b) => a != null && b != null && Math.abs(+a - +b) < 1e-4;
  const vehicleCell = (k, v) => `<span class="ov-cur ${cur[k] == null ? 'muted' : same(cur[k], v) ? 'ok' : 'warn'}" title="the value the vehicle has right now">${cur[k] == null ? '—' : f(cur[k])}</span>`;
  // PX4's own default, so every row says what it was moved away from
  const defCell = (k, v) => {
    const d = (meta[k] || {}).default;
    if (d == null) return '<span class="ov-def muted" title="this firmware does not describe a default">—</span>';
    const at = Math.abs(+d - +v) <= Math.max(1e-9, Math.abs(+d) * 1e-6);
    return `<span class="ov-def ${at ? 'muted' : 'off'}" title="PX4's default${at ? ' (this is it)' : ''}">${f(+d)}</span>`;
  };
  // What a real aircraft gets instead. Same value, a different one, or nothing at all - and when it is
  // nothing, why. This is the column that answers "what will actually be different when I fly it".
  const flightCell = (k, v) => {
    if (r && r.flight_error && !Object.keys(flight).length) return '<span class="ov-fly muted" title="the real-flight set cannot be built for this design yet">—</span>';
    if (!(k in flight)) return `<span class="ov-fly muted" title="${esc(notes[k] || 'not written to a real aircraft')}">not sent</span>`;
    return same(flight[k], v)
      ? '<span class="ov-fly ok" title="the same on a real aircraft">same</span>'
      : `<span class="ov-fly warn" title="a real aircraft gets this instead">${f(flight[k])}</span>`;
  };
  const head = `<div class="ov-row head"><span>Parameter</span><span>Set value</span><span>Default</span>`
    + `<span>On vehicle</span><span>In flight</span><span>What it is</span><span></span></div>`;
  const row = (k, value, kind) => {
    const m = meta[k] || {};
    const g = geo[k];
    const edited = k in ov;
    const drift = kind === 'geo' && edited && g != null && !same(g, ov[k]);
    const note = drift ? `<span class="warn" title="the design works this out as ${f(g)}; your value is used instead"> · geometry says ${f(g)}</span>` : '';
    const back = kind === 'geo'
      ? (edited ? `<button class="del" data-reset="${k}" title="back to the value the geometry works out (${f(g)})">↺</button>` : '<span></span>')
      : `<button class="del" data-n="${k}" title="forget this parameter">✕</button>`;
    return `<div class="ov-row${drift ? ' drift' : ''}"><span class="pname">${k}</span>`
      + `<input type="number" step="any" value="${value}" data-n="${k}">`
      + `${defCell(k, value)}${vehicleCell(k, value)}${flightCell(k, value)}`
      + `<span class="pdesc" title="${(m.short || '').replace(/"/g, '&quot;')}">${m.short || ''}${note}</span>${back}</div>`;
  };

  let html = '';
  // A module switched off takes its parameters out of the export, so they come out of the table too -
  // including one you had set by hand. The value stays in the design and comes back with the module.
  const gone = new Set((r && r.modules_off) || []);
  const geoNames = Object.keys(geo).filter(k => !gone.has(k)).sort();
  const added = Object.keys(ov).filter(k => !(k in geo) && !gone.has(k)).sort();
  if (added.length) {
    html += '<div class="hint" style="margin:2px 0 4px">Added parameters</div>' + head;
    added.forEach(k => { html += row(k, ov[k], 'added'); });
  }
  if (geoNames.length) {
    if (html) html += '<div class="ov-sep"></div>';
    html += '<div class="hint" style="margin:2px 0 4px">Geometry dependent parameters</div>' + head;
    geoNames.forEach(k => { html += row(k, k in ov ? ov[k] : (P[k] ?? geo[k]), 'geo'); });
  }
  el.innerHTML = html || '<div class="hint">Nothing here yet. Add a parameter above, or change the design on the Geometry tab.</div>';
  $$('#overrides input').forEach(inp => inp.addEventListener('change', () => setParamValue(inp.dataset.n, parseFloat(inp.value))));
  $$('#overrides .del[data-n]').forEach(b => b.addEventListener('click', async () => { await api('/api/airframe/override_remove', { name: b.dataset.n }); delete airframe.px4_overrides[b.dataset.n]; loadExport(); markDirty(); }));
  $$('#overrides .del[data-reset]').forEach(b => b.addEventListener('click', async () => {
    const n = b.dataset.reset;
    await api('/api/airframe/override_remove', { name: n });
    delete airframe.px4_overrides[n];
    logLine(`[export] ${n} is back to what the geometry works out`);
    loadExport(); markDirty();
  }));
}
async function setParamValue(n, value) {
  if (status.ctl_connected) {
    const r = await api('/api/params/set', { name: n, value });
    if (!r.ok) { logLine('[ui] ' + n + ': ' + r.error); return r; }
    params[n] = params[n] || { type: 9 }; params[n].value = r.value;
    airframe.px4_overrides[n] = r.value;
  } else {
    await api('/api/airframe/override', { name: n, value });
    airframe.px4_overrides[n] = value;
  }
  loadExport(); renderFind(); markDirty();
  return { ok: true, value };
}

// ============================================================ parameters
let paramsLoaded = false;
let paramsSession = null;
async function ensureParams() {
  if (paramsLoaded) return;
  await loadMeta();
  await loadParams();
}
async function loadMeta() {
  const r = await api('/api/params/meta');
  meta = r.meta || {};
}
async function loadParams() {
  const r = await api('/api/params');
  params = r.params || {};
  paramsLoaded = Object.keys(params).length > 0;
  loadExport();          // vehicle values are now known: refresh the ✓/≠ marks
  const described = Object.keys(meta).length;
  $('#param-count').textContent = `${Object.keys(params).length}/${r.count} on the vehicle · ${described} described`;
  // Update PX4 writes the parameters and saves them to flash itself, so there is no button for that; this
  // one only matters when the descriptions are missing, which happens with firmware this computer did not build.
  $('#param-meta-fetch').hidden = described > 50;
  renderFind();
}
const num6 = (v) => typeof v === 'number' && !Number.isInteger(v) ? +v.toPrecision(6) : v;
// What PX4 ships with, beside what this vehicle has. A parameter sitting at its default is the uninteresting
// case, so it is the one that is greyed; one that has been moved says what it was moved from.
function defaultCell(name, value) {
  const d = (meta[name] || {}).default;
  if (d == null) return '<span class="pdef muted">—</span>';
  const same = Math.abs(+d - +value) <= Math.max(1e-9, Math.abs(+d) * 1e-6);
  return same ? '<span class="pdef muted">default</span>'
              : `<span class="pdef off" title="PX4's default">≠ ${num6(+d)}</span>`;
}
// Add parameter: the same search as before, but it ends in the table above instead of a second editor.
// Picking one writes it as an override at the value it has now, which is the thing you then edit.
let findOpen = false;
function renderFind() {
  if (!findOpen) return;
  const q = $('#param-search').value.trim().toLowerCase();
  const names = Object.keys(params).sort();
  const ov = (airframe && airframe.px4_overrides) || {};
  if (!q) { $('#param-list').innerHTML = `<div class="hint">${names.length} parameters · type a name or a word from its description.</div>`; return; }
  const html = [];
  let shown = 0;
  for (const n of names) {
    const m = meta[n] || {};
    if (!(n.toLowerCase().includes(q) || (m.short || '').toLowerCase().includes(q))) continue;
    if (++shown > 30) break;
    const v = params[n].value;
    html.push(`<div class="prow${n in ov ? ' edited' : ''}" data-n="${n}"><span class="pname">${n in ov ? '● ' : ''}${n}</span>`
      + `<span class="pval">${num6(v)}${m.unit ? ' <span class="muted">' + m.unit + '</span>' : ''}</span>`
      + `${defaultCell(n, v)}<span class="pdesc" title="${(m.short || '').replace(/"/g, '&quot;')}">${m.short || ''}</span></div>`);
  }
  $('#param-list').innerHTML = html.join('') || '<div class="hint">nothing matches that.</div>';
  $$('#param-list .prow').forEach(el => el.addEventListener('click', () => addOverride(el.dataset.n)));
}
async function addOverride(n) {
  const current = params[n] ? params[n].value : ((meta[n] || {}).default ?? 0);
  try {
    await api('/api/airframe/override', { name: n, value: current });
    airframe.px4_overrides = { ...(airframe.px4_overrides || {}), [n]: current };
    markDirty();
    await loadExport();
    openFind(false);
    const row = $(`#overrides input[data-n="${n}"]`);
    if (row) { row.focus(); row.select(); }
  } catch (e) { logLine('[params] ' + e.message); }
}
function openFind(on) {
  findOpen = on;
  $('#param-find').hidden = !on;
  $('#param-add').classList.toggle('active', on);
  if (on) { ensureParams().then(renderFind).catch(() => { }); $('#param-search').value = ''; $('#param-search').focus(); renderFind(); }
}
$('#param-add').addEventListener('click', () => openFind(!findOpen));
$('#param-search').addEventListener('input', renderFind);
$('#param-search').addEventListener('keydown', (e) => { if (e.key === 'Escape') openFind(false); });
$('#param-meta-fetch').addEventListener('click', async () => {
  $('#param-count').textContent = 'downloading descriptions via MAVLink FTP…';
  try { const r = await api('/api/params/meta/fetch', {}); await loadMeta(); logLine(`[ui] ${r.count} parameter descriptions from ${r.source}`); }
  catch (e) { logLine('[ui] ' + e.message); }
  await loadParams();
});

// ============================================================ flight / sim
// ============================================================ takeoff and landing
// What the two buttons do is a choice, kept with the design: the nose-lift ground sequence in the
// firmware, or PX4's own auto takeoff and land. The numbers beside it are the PX4 parameters those use,
// edited here as overrides so they travel with the airframe and go out with the next Update PX4.
const TL_PARAMS = { 'tl-alt': 'MIS_TAKEOFF_ALT', 'tl-climb': 'MPC_TKO_SPEED', 'tl-nose': 'NLF_TARGET',
                    'tl-descend': 'MPC_LAND_SPEED', 'tl-disarm': 'COM_DISARM_LAND' };
const flightPlan = () => ((airframe && (airframe.design ||= {})).flight ||= {});
// 'px4' is PX4's own takeoff and land; anything else is the directory of a module in the Firmware tab,
// which is asked to do it over MAV_CMD_USER_1.
const takeoffMode = () => (airframe?.design?.flight?.takeoff) || 'px4';
const landMode = () => (airframe?.design?.flight?.land) || 'px4';
const moduleFor = (dir) => fwGroups.find(g => g.dir === dir && g.kind === 'module');
function paramNow(name, fallback) {
  const v = airframe?.px4_overrides?.[name] ?? params[name]?.value;
  return v === undefined || v === null ? fallback : v;
}
function tlOptions(sel, chosen) {
  const mods = fwGroups.filter(g => g.kind === 'module' && g.user_command);
  sel.innerHTML = `<option value="px4">PX4's own</option>`
    + mods.map(g => `<option value="${esc(g.dir)}">${esc(g.name)}</option>`).join('');
  sel.value = [...sel.options].some(o => o.value === chosen) ? chosen : 'px4';
}
function fillTakeoffLanding() {
  if (!airframe || !$('#tl-takeoff')) return;
  loadFirmwareModules().then(() => {
    tlOptions($('#tl-takeoff'), takeoffMode());
    tlOptions($('#tl-land'), landMode());
  });
  for (const [id, name] of Object.entries(TL_PARAMS)) {
    const el = $('#' + id);
    if (document.activeElement !== el) el.value = nf(paramNow(name, ''), 3);
    el.disabled = !!status.armed;
  }
  const noseTakeoff = takeoffMode() !== 'px4', noseLand = landMode() !== 'px4';
  $('#tl-nose').disabled = !!status.armed || !noseTakeoff;
  $('#tl-alt').disabled = $('#tl-climb').disabled = !!status.armed || noseTakeoff;
  $('#tl-descend').disabled = !!status.armed || noseLand;
  const nameOf = (m) => m === 'px4' ? 'PX4' : (moduleFor(m)?.name || m);
  $('#tl-now').textContent = `${nameOf(takeoffMode())} up, ${nameOf(landMode())} down`;
  const usesModule = noseTakeoff || noseLand;
  const nlf = (noseTakeoff && moduleFor(takeoffMode())?.params?.includes('NLF_ENABLE'))
           || (noseLand && moduleFor(landMode())?.params?.includes('NLF_ENABLE'));
  $('#tl-note').textContent = !usesModule
    ? "PX4's own takeoff and landing. The numbers above are its parameters."
    : nlf && +paramNow('NLF_ENABLE', 0) !== 1
      ? 'The module is switched off on the vehicle (NLF_ENABLE = 0). Update PX4 to turn it on.'
      : 'The module is asked over MAV_CMD_USER_1. Saved with the design; Update PX4 writes the parameters.';
}
$$('#tl-takeoff, #tl-land').forEach(sel => sel.addEventListener('change', async (e) => {
  const plan = flightPlan();
  plan.takeoff = takeoffMode(); plan.land = landMode();
  plan[e.target.id === 'tl-takeoff' ? 'takeoff' : 'land'] = e.target.value;
  // Choosing a module is choosing to run it: its own enable parameter follows the choice.
  const mods = [plan.takeoff, plan.land].map(moduleFor).filter(Boolean);
  if (mods.some(m => m.params.includes('NLF_ENABLE')) || 'NLF_ENABLE' in params) {
    const want = mods.some(m => m.params.includes('NLF_ENABLE')) ? 1 : 0;
    if (+paramNow('NLF_ENABLE', 0) !== want) {
      try {
        await api('/api/airframe/override', { name: 'NLF_ENABLE', value: want });
        airframe.px4_overrides.NLF_ENABLE = want;
        if (status.ctl_connected && !status.armed) await api('/api/params/set', { name: 'NLF_ENABLE', value: want }).catch(() => { });
        logLine('[flight] NLF_ENABLE = ' + want);
      } catch (err) { logLine('[flight] ' + err.message); }
    }
  }
  fillTakeoffLanding(); pushAirframe(true); renderOverrides();
}));
$$('#tl-alt, #tl-climb, #tl-nose, #tl-descend, #tl-disarm').forEach(inp => inp.addEventListener('change', async (e) => {
  const name = TL_PARAMS[e.target.id];
  const value = parseFloat(e.target.value);
  if (!isFinite(value)) return;
  try {
    await api('/api/airframe/override', { name, value });
    airframe.px4_overrides[name] = value;
    if (status.ctl_connected && !status.armed) await api('/api/params/set', { name, value }).catch(() => { });
    markDirty();
    logLine(`[flight] ${name} = ${value}`);
  } catch (err) { logLine('[flight] ' + err.message); }
  renderOverrides();
}));

// Arm and Takeoff lost their handlers when the Flight tab went: the footer kept labelling and enabling two
// buttons that did nothing at all when pressed.
$('#btn-arm').addEventListener('click', async () => {
  try { await api('/api/px4/command', status.armed ? { command: 'kill' } : { command: 'arm' }); }
  catch (e) { logLine('[ui] ' + e.message); }
});
$('#btn-takeoff').addEventListener('click', async () => {
  const b = $('#btn-takeoff');
  b.disabled = true;
  try {
    if (takeoffMode() !== 'px4') {
      await api('/api/px4/command', { command: 'nose_lift_takeoff' });
      logLine('[ui] takeoff: handed to ' + (moduleFor(takeoffMode())?.name || takeoffMode()));
    } else {
      if (!status.armed) await api('/api/px4/command', { command: 'arm' });
      await api('/api/px4/command', { command: 'takeoff' });
      logLine('[ui] takeoff: PX4 auto takeoff to MIS_TAKEOFF_ALT');
    }
  } catch (e) { logLine('[ui] takeoff: ' + e.message); }
  b.disabled = false;
});

$$('#btn-land').forEach(b => b.addEventListener('click', async () => {
  try {
    if (b.dataset.mode === 'land' && landMode() !== 'px4') {
      await api('/api/px4/command', { command: 'nose_lift_land' });
      return;
    }
    await api('/api/px4/command', { command: b.dataset.cmd, mode: b.dataset.mode, force: b.dataset.cmd === 'kill' });
  } catch(e) { logLine('[ui] ' + e.message); }
}));
async function recover(btn, full) {
  const label = btn.textContent; btn.textContent = 'Resetting…'; btn.disabled = true;
  try { const r = await api('/api/px4/reset_all', {}); logLine('[ui] reset: ' + (r.steps || []).join(', ')); }
  catch (e) { logLine('[ui] reset failed: ' + e.message); }
  btn.textContent = label; btn.disabled = false;
}
$('#btn-reset').addEventListener('click', (e) => recover(e.target, true));     // everything: vehicle + PX4 reboot
$('#btn-pause').addEventListener('click', async () => {
  const r = await api('/api/sim/pause', {});
  if (!r.paused) { stopPlaying(); replayT = null; pausedByScrub = false; }   // running again: back to the live view
  $('#btn-pause').textContent = r.paused ? 'Resume' : 'Pause';
});
// The camera follows the aircraft, always: there is no mode to choose. Move it yourself and the only
// button is the one that brings it back.
let camMode = 'follow';
scene.setCameraMode('follow');
scene.onCameraMoved(() => { $('#recenter').hidden = false; });
$('#recenter').addEventListener('click', (e) => { scene.recenterView(); e.currentTarget.hidden = true; });

// Hardware sliders send short PX4 actuator-test pulses only while held.
let motorTestTimer = null, motorTestBusy = false, heldMotor = null, motorTestStarted = 0;
const hardwareMotors = () => (status.conn_mode || status.mode) === 'hitl';
// Physical connector numbering is independent of allocator motor identity.
function rotorPortKey(port) {
  return port <= 8 ? `PWM_MAIN_FUNC${port}` : `PWM_AUX_FUNC${port - 8}`;
}
function motorPortValues() {
  const values = {}, overrides = airframe.px4_overrides || {};
  // Keep a complete saved wiring map once edited. Older models default to 1..N.
  const saved = Array.from({length:10}, (_,i) => rotorPortKey(i+1)).some(k => k in overrides);
  const live = Array.from({length:10}, (_,i) => Number(params[rotorPortKey(i+1)]?.value || 0));
  const liveMapped = live.some(v => v >= 101 && v <= 116);
  const count = airframe.rotors.filter(r => r.enabled !== false).length;
  for (let port=1; port<=10; port++) {
    const key = rotorPortKey(port);
    values[key] = saved ? Number(overrides[key] || 0) : liveMapped ? live[port-1] : port <= count ? 100+port : 0;
  }
  return values;
}
function rotorPortHtml(i, r) {
  const motor = r.enabled === false ? 0 : airframe.rotors.slice(0,i+1).filter(x => x.enabled !== false).length;
  const values = motorPortValues();
  const port = Array.from({length:10},(_,i)=>i+1).find(p=>values[rotorPortKey(p)] === 100+motor);
  return `<select class="rotor-port" data-motor="${motor}" aria-label="PWM port for ${esc(r.name)}" ${!motor || status.armed || $('#motor-enable').checked ? 'disabled' : ''}><option value="">—</option>${Array.from({length:10},(_,i)=>i+1).map(p=>`<option value="${rotorPortKey(p)}" ${p===port?'selected':''}>${p}</option>`).join('')}</select>`;
}
function updateMotorPorts() {
  const values = motorPortValues();
  $$('#rotor-table select.rotor-port').forEach(sel => {
    sel.disabled = !!status.armed || $('#motor-enable').checked || !Number(sel.dataset.motor);
    if (document.activeElement !== sel) sel.value = Object.keys(values).find(k=>values[k]===100+Number(sel.dataset.motor)) || '';
  });
}
function changeMotorPort(sel) {
  if (status.armed || $('#motor-enable').checked) { updateMotorPorts(); return; }
  const values = motorPortValues(), fn = 100 + Number(sel.dataset.motor), target = sel.value;
  const old = Object.keys(values).filter(k => values[k] === fn);
  if (target && !(target in values)) return;
  const occupant = target ? values[target] : 0;
  if (occupant && (occupant < 101 || occupant > 116)) { logLine('[motor ports] This port is assigned to a non-motor output.'); sel.blur(); updateMotorPorts(); return; }
  if (occupant && occupant !== fn && old.length !== 1) { logLine('[motor ports] Assign this motor to a free port before swapping.'); sel.blur(); updateMotorPorts(); return; }
  for (const k of old) values[k] = 0;
  if (target) values[target] = fn;
  if (occupant && occupant !== fn) values[old[0]] = occupant;
  airframe.px4_overrides = {...airframe.px4_overrides, ...values};
  sel.blur(); updateMotorPorts(); pushAirframe(true);
}
function renderMotorSliders() {
  const el = $('#motor-sliders');
  const hardware = hardwareMotors();
  let output = 0;
  el.innerHTML = airframe.rotors.map((r, i) => {
    const motor = r.enabled === false ? 0 : ++output;
    const label = r.name === `M${motor}` ? r.name : `M${motor || '–'} ${r.name}`;   // not "M1 M1"
    return `<label><b title="${esc(r.name)}">${esc(label)}</b> <input type="range" min="0" max="${hardware ? .15 : 1}" step="0.005" value="0" data-i="${i}" data-motor="${motor}" disabled><span class="mv num">0%</span></label>`;
  }).join('');
  updateMotorPorts();
  el.querySelectorAll('input').forEach(inp => {
    inp.addEventListener('pointerdown', () => {
      if (!hardwareMotors() || !$('#motor-enable').checked || status.armed) return;
      stopMotorTest();
      scene.highlightMotorTest?.(Number(inp.dataset.i));
      heldMotor = inp; motorTestStarted = Date.now()/1000;
      motorTestTimer = setInterval(sendHardwareMotor, 150);
    });
    inp.addEventListener('input', () => {
      if ($('#motor-enable').checked) scene.highlightMotorTest?.(Number(inp.dataset.i));
      hardwareMotors() ? sendHardwareMotor() : sendOverride();
    });
  });
}
async function sendHardwareMotor() {
  if (!heldMotor || motorTestBusy) return;
  if (!hardwareMotors() || status.armed || !status.ctl_connected || !$('#motor-enable').checked) { stopMotorTest(); return; }
  motorTestBusy = true;
  const inp = heldMotor;
  try {
    const value = Number(inp.value);
    const result = await api('/api/px4/motor_test', {motor: Number(inp.dataset.motor), value, confirmed:true});
    if (!result.ok) throw new Error(result.error || 'Motor test rejected');
    inp.parentElement.querySelector('.mv').textContent = (value*100).toFixed(1)+'% command';
  } catch(e) { logLine('[motor test] '+e.message); stopMotorTest(); }
  finally { motorTestBusy = false; }
}
function stopMotorTest() {
  scene.highlightMotorTest?.(-1);
  clearInterval(motorTestTimer); motorTestTimer = null;
  const inp = heldMotor; heldMotor = null;
  if (inp) {
    inp.value = 0;
    inp.parentElement.querySelector('.mv').textContent = '0% command';
    api('/api/px4/motor_test', {motor:Number(inp.dataset.motor), value:0, confirmed:true}).catch(()=>{});
  }
}
window.addEventListener('pointerup', stopMotorTest);
window.addEventListener('pointercancel', stopMotorTest);
window.addEventListener('blur', stopMotorTest);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopMotorTest(); });
function sendOverride() {
  const vals = $$('#motor-sliders input').map(i => parseFloat(i.value));
  if ($('#motor-enable').checked && !hardwareMotors()) api('/api/sim/motor_override', {values:vals});
}
function updateMotorSliders(st) {
  updateMotorPorts();
  const hardware = hardwareMotors();
  $('#motor-test-help').textContent = hardware ? 'REAL MOTORS · Disarmed only. Hold one slider to run; release to stop. Maximum 15% command. Yellow vector = requested fan, not measured rotation. Wait one second before another motor.' : 'Simulator motor override';
  if (hardware && (status.armed || !status.ctl_connected)) { stopMotorTest(); $('#motor-enable').checked = false; }
  const manual = $('#motor-enable').checked;
  $$('#motor-sliders input').forEach(inp => {
    inp.max = hardware ? .15 : 1;
    inp.disabled = !manual || (hardware && (!Number(inp.dataset.motor) || status.armed || !status.ctl_connected));
    if (hardware && inp !== heldMotor) inp.value = 0;
  });
  if (!hardware) st.rotors.forEach((x, i) => {
    const inp = document.querySelector(`#motor-sliders input[data-i="${i}"]`);
    if (inp && !manual) inp.value = x.cmd.toFixed(2);
    if (inp) inp.parentElement.querySelector('.mv').textContent = (x.cmd*100).toFixed(0)+'%';
  });
  if (hardware && status.last_ack?.command === 310 && status.last_ack.result !== 0 && status.last_ack.t >= motorTestStarted && heldMotor) {
    logLine('[motor test] PX4 rejected the test. Check its safety button and actuator-test permissions.'); stopMotorTest();
  }
}
$('#motor-enable').addEventListener('change', (e) => {
  stopMotorTest();
  if (e.target.checked && hardwareMotors()) {
    if (status.armed || !status.ctl_connected || !confirm('REAL MOTOR TEST: remove propellers/impellers where the manufacturer permits, secure the aircraft, and keep clear of all fans. Hold a slider to run one motor; release to stop. Ready?')) e.target.checked = false;
  }
  $$('#motor-sliders input').forEach(inp => {inp.value=0; inp.disabled=!e.target.checked;});
  if (!hardwareMotors()) { if (e.target.checked) sendOverride(); else api('/api/sim/motor_override', {values:null}); }
});

// ============================================================ websocket + status
function applyStatus(s) {
  status = s;
  if (s.params_session !== undefined && s.params_session !== paramsSession) {   // board (re)connected or reflashed: forget the old list
    paramsSession = s.params_session; paramsLoaded = false; params = {};
  }
  if (s.params_loaded && s.param_count && s.params_loaded >= s.param_count && !paramsLoaded
      && (hardwareMotors() || rcVisible() || $('#tab-px4').classList.contains('active') || $('#tab-sequences').classList.contains('active')))
    ensureParams().then(fillTakeoffLanding).catch(() => { });
  if (rcVisible()) renderModeButtons();
  if ($('#tab-sequences').classList.contains('active')) fillTakeoffLanding();
  // the switch over the 3D view says which way the whole thing is running, on every tab
  paintModeSwitch(s);
  updateFooter();
}
// Green means this mode is doing what it says: SITL flying the simulation, the board flying it in HITL
// (HIL actually enabled), or the aircraft's telemetry arriving in FLIGHT. Anything else is red, and the
// tooltip says what is missing.
let lastStatusAt = 0;
function paintModeSwitch(s) {
  if (s) lastStatusAt = Date.now();
  s = s || {};
  if (modeWanted && modeWanted === linkReal(s)) modeWanted = null;   // the wish came true: follow the link again
  const shown = modeShown(s);
  $$('#mode-switch button').forEach(b => b.classList.toggle('active', b.dataset.link === shown));
  let ok = false, why = '';
  const onBoard = s.conn_mode === 'hitl';
  // While the switch is doing its work, what it is doing is the most useful thing the light can say
  if (modeJob && modeJob.state === 'running' && (modeJob.mode === shown || !shown)) {
    $('#link-light i').classList.remove('on');
    $('#link-light').classList.add('crit');
    const label = { sitl: 'SITL', hitl: 'HITL', real: 'FLIGHT' }[shown || modeJob.mode] || '';
    $('#link-light').title = `${label} · ${modeJob.step || 'working…'}`;
    return;
  }
  const flying = (s.mode_name && !/^mode\d+$/.test(s.mode_name) ? s.mode_name : '—') + (s.armed ? ' · ARMED' : '');
  if (s.resetting > 0) why = `resetting, ${Math.ceil(s.resetting)} s`;
  else if (s.flashing) why = 'flashing the board';
  else if (shown === 'sitl') {
    ok = !!(s.conn_mode === 'sitl' && s.connected && s.ctl_connected);
    why = ok ? flying
             : s.conn_mode !== 'sitl' ? 'not started: press SITL'
             : s.px4_running ? 'PX4 starting…' : 'no PX4';
  } else if (shown === 'hitl') {
    ok = !!(onBoard && s.ctl_connected && s.hil_enabled && !s.listening);
    why = !onBoard ? `no flight controller on USB (still ${s.conn_mode || 'off'})`
        : !s.ctl_connected ? 'no answer from the board'
        : s.listening ? 'the board is on its own sensors (real flight)'
        : !s.hil_enabled ? 'the board has HIL off (SYS_HITL = 0)'
        : flying;
  } else if (shown === 'real') {
    const fresh = s.telemetry_age != null && s.telemetry_age < 3;
    ok = !!(onBoard && s.listening && s.ctl_connected && fresh && !s.hil_enabled);
    const radio = s.radio && s.radio.age != null && s.radio.age < 5;
    // A board still set to SYS_HITL = 1 has its own IMU switched off and is waiting for a simulator to
    // send it one. Nothing is wrong with the link, the aircraft simply has no attitude and no position to
    // report - which otherwise reads as "not saying where it is" and sends you hunting the wrong thing.
    why = !onBoard ? `no radio or board on USB (still ${s.conn_mode || 'off'})`
        : !s.ctl_connected ? (radio ? 'the radio is up, the aircraft is not answering it (powered? paired? same NETID?)'
                                    : 'no link to the aircraft')
        : s.hil_enabled ? 'the board is still in HITL (SYS_HITL = 1): its own sensors are off and it is waiting '
                        + 'for simulated ones, so it has no attitude or position to send. Set up for real flight '
                        + 'switches it over.'
        : !s.listening ? 'connected, but the simulator is still driving it'
        : !fresh ? 'link up, but the aircraft is not saying where it is'
        : flying;
  } else why = 'nothing connected';
  if (s.conn_error) { ok = false; why = s.conn_error; }
  $('#link-light i').classList.toggle('on', ok);
  $('#link-light').classList.toggle('crit', !ok);
  const MODE_LABEL = { sitl: 'SITL', hitl: 'HITL', real: 'FLIGHT' };
  $('#link-light').title = `${MODE_LABEL[shown] || 'NO MODE'} · ${why}`;
  $('#link-light').classList.toggle('unknown', !lastStatusAt);
}
const effectiveMax = (r) => (r.enabled === false ? 0 : r.max_thrust);
function applyState(st) {
  if (replayT == null) scene.updateState(st);   // while scrubbing, the view belongs to the recording
  // The recording keeps the wind's strength but not its direction, so the arrow belongs to the live
  // simulation only; while you are inside a recording there is nothing honest to point.
  scene.setWind(replayT == null ? st.wind : null);
  $('#btn-pause').textContent = st.paused ? 'Resume' : 'Pause';
  $('#btn-pause').classList.toggle('armed', !!st.paused);

  const f = st.forces || {};

  // how long the charge lasts at this draw: the battery chip says it, if it is on the overlay
  const cap = +((airframe && airframe.design && airframe.design.battery_wh) || 0), pw = +st.power_avg_w || 0;
  batteryMinutes = cap > 0 && pw > 5 ? (Math.max(0, cap - (+st.energy_wh || 0)) / pw) * 60 : null;
  const battChip = $('#overlay-chips [data-v="battery"]');
  if (battChip) {
    const pct = cap > 0 ? 100 * Math.max(0, cap - (+st.energy_wh || 0)) / cap : 100;
    battChip.parentElement.classList.toggle('warn', cap > 0 && pct <= 25 && pct > 10);
    battChip.parentElement.classList.toggle('crit', cap > 0 && pct <= 10);
  }
  // a stalled wing is worth saying out loud wherever the lift chip happens to be
  const liftChip = $('#overlay-chips [data-v="lift"]');
  if (liftChip) liftChip.parentElement.classList.toggle('warn', !!f.stalled);
  showWindNow(st);
  // while you are inside the recording the sliders belong to it, not to the aircraft flying now
  if (airframe) updateMotorSliders(replayT == null ? st : { rotors: replayRotors || [] });
}
const logPre = $('#log-pre');
// This window is what the flight controller said, and nothing else: PX4's own console and the events it
// sends ([px4] ...). The app's own chatter goes to the browser console, where it belongs - the places that
// need to tell you something (the sections, the status lines) say it themselves.
let flightLog = '';
function logLine(s) {
  if (!String(s).startsWith('[px4]')) { console.log(s); return; }
  flightLog += String(s).slice(6) + '\n';
  if (flightLog.length > 60000) flightLog = flightLog.slice(-45000);
  if (shownRunLog === null) showLogText(flightLog);
}
let shownRunLog = null;                    // the log of a saved run, while one is selected
function showLogText(text) {
  logPre.textContent = text;
  const b = $('#log-body');
  b.scrollTop = b.scrollHeight;
}

// ============================================================ the log window: terminal, graphs, replay
// Two views of one flight. The terminal is what the app and PX4 said; the graph is what the aircraft did.
// The server records every run at 20 Hz with the airframe and the parameters that produced it, so a flight
// can be scrubbed afterwards — the 3D view follows the slider — kept, named and loaded back. The graph
// itself is drawn from a buffer this page fills from the live state, so it is there while the sim runs.
// Every column the recorder writes can be plotted and can be put on the 3D view. There are too many to
// list flat - ten motors are thirty of them - so each one says which drawer it belongs in, and the pickers
// are built from that. The catalogue follows the recording: a design with more rotors simply has more
// entries, and a run saved earlier brings its own columns with it.
const SERIES_BASE = {
  power:     { label: 'power', unit: 'W', c: '#f59e0b', cat: 'Power' },
  battery:   { label: 'battery', unit: '%', c: '#22c55e', cat: 'Power' },
  thrust:    { label: 'thrust', unit: 'N', c: '#a78bfa', cat: 'Flight' },
  lift:      { label: 'wing lift', unit: 'N', c: '#38bdf8', cat: 'Flight' },
  airspeed:  { label: 'airspeed', unit: 'm/s', c: '#06b6d4', cat: 'Flight' },
  speed:     { label: 'ground speed', unit: 'm/s', c: '#0ea5e9', cat: 'Flight' },
  height:    { label: 'height', unit: 'm', c: '#ef4444', cat: 'Flight' },
  wind:      { label: 'wind', unit: 'm/s', c: '#94a3b8', cat: 'Flight' },
  n:         { label: 'north', unit: 'm', c: '#4ade80', cat: 'Flight' },
  e:         { label: 'east', unit: 'm', c: '#2dd4bf', cat: 'Flight' },
  d:         { label: 'down', unit: 'm', c: '#14b8a6', cat: 'Flight' },
  vn:        { label: 'speed north', unit: 'm/s', c: '#34d399', cat: 'Flight' },
  ve:        { label: 'speed east', unit: 'm/s', c: '#5eead4', cat: 'Flight' },
  vd:        { label: 'climb rate', unit: 'm/s', c: '#60a5fa', cat: 'Flight' },
  roll:      { label: 'roll', unit: '°', c: '#f472b6', cat: 'Attitude' },
  pitch:     { label: 'pitch', unit: '°', c: '#fb923c', cat: 'Attitude' },
  yaw:       { label: 'yaw', unit: '°', c: '#c084fc', cat: 'Attitude' },
  p:         { label: 'roll rate', unit: '°/s', c: '#fda4af', cat: 'Attitude' },
  q:         { label: 'pitch rate', unit: '°/s', c: '#fcd34d', cat: 'Attitude' },
  r:         { label: 'yaw rate', unit: '°/s', c: '#d8b4fe', cat: 'Attitude' },
  ax:        { label: 'accel X', unit: 'm/s²', c: '#f87171', cat: 'Sensors' },
  ay:        { label: 'accel Y', unit: 'm/s²', c: '#fb7185', cat: 'Sensors' },
  az:        { label: 'accel Z', unit: 'm/s²', c: '#f43f5e', cat: 'Sensors' },
  gx:        { label: 'gyro X', unit: '°/s', c: '#a3e635', cat: 'Sensors' },
  gy:        { label: 'gyro Y', unit: '°/s', c: '#84cc16', cat: 'Sensors' },
  gz:        { label: 'gyro Z', unit: '°/s', c: '#65a30d', cat: 'Sensors' },
  alt_baro:  { label: 'baro height', unit: 'm', c: '#fca5a5', cat: 'Sensors' },
  flow_x:    { label: 'flow X', unit: 'rad/s', c: '#38bdf8', cat: 'H-flow' },
  flow_y:    { label: 'flow Y', unit: 'rad/s', c: '#0284c7', cat: 'H-flow' },
  flow_range:{ label: 'flow range', unit: 'm', c: '#7dd3fc', cat: 'H-flow' },
  flow_q:    { label: 'flow quality', unit: '', c: '#bae6fd', cat: 'H-flow' },
  armed:     { label: 'armed', unit: '', c: '#64748b', cat: 'State' },
  on_ground: { label: 'on ground', unit: '', c: '#475569', cat: 'State' },
  mode:      { label: 'flight mode', unit: '', c: '#818cf8', cat: 'State',
               desc: "the mode PX4 is in, as it reports it in its heartbeat" },
  qw: null, qx: null, qy: null, qz: null,            // the quaternion is for replay, not for reading
};
const MOTOR_KIND = {
  cmd:    { label: 'throttle', unit: '', hue: 24, desc: 'what PX4 asks this motor for, 0 to 1' },
  spool:  { label: 'spooled', unit: '', hue: 150, desc: 'what the motor has actually reached: the throttle after the spool-up lag' },
  thrust: { label: 'thrust', unit: 'N', hue: 272, desc: 'the force this rotor is making at that spooled throttle, newtons' },
};
const CAT_ORDER = ['Flight', 'Attitude', 'Motors', 'Sensors', 'H-flow', 'Power', 'State'];
// Inside a drawer the order is the one you would read them in, not the order the recorder happens to
// write them: the three positions together, then the three speeds, and so on.
const SERIES_ORDER = ['height', 'speed', 'airspeed', 'vd', 'n', 'e', 'd', 'vn', 've', 'thrust', 'lift', 'wind',
  'mode',
  'roll', 'pitch', 'yaw', 'p', 'q', 'r',
  'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'alt_baro',
  'flow_x', 'flow_y', 'flow_range', 'flow_q', 'power', 'battery', 'armed', 'on_ground'];
const seriesRank = (key) => {
  const i = SERIES_ORDER.indexOf(key);
  if (i >= 0) return i;
  const m = key.match(/^m(\d+)_(cmd|spool|thrust)$/);   // motor by motor, in the order a motor spins up
  return m ? 1000 + (+m[1]) * 10 + ['cmd', 'spool', 'thrust'].indexOf(m[2]) : 9000;
};
function describeSeries(key) {
  if (key in SERIES_BASE) return SERIES_BASE[key];
  const m = key.match(/^m(\d+)_(cmd|spool|thrust)$/);
  if (m) {
    const k = MOTOR_KIND[m[2]], i = +m[1];
    return { label: `motor ${i} ${k.label}`, unit: k.unit, cat: 'Motors', desc: k.desc,
             c: `hsl(${(k.hue + i * 37) % 360} 70% 52%)` };
  }
  return { label: key, unit: '', c: '#94a3b8', cat: 'State' };
}
let SERIES = [];
let seriesSignature = null;
function rebuildSeries(cols) {
  // Compare against the columns, not against the catalogue: some columns (the quaternion) have no entry,
  // so the two lengths never match and this used to rebuild on every poll - four times a second, which
  // replaced the chips under the pointer and made them impossible to drag.
  const keys = (cols || LIVE_COLS).filter(k => k !== 't');
  const sig = keys.join(',');
  if (sig === seriesSignature) return;
  seriesSignature = sig;
  SERIES = keys.map(k => ({ key: k, ...describeSeries(k) })).filter(x => x.label);
  buildPicker();
  renderOverlayChips();
}
// The columns the recorder writes on the server (sim/recorder.py), before the per-rotor ones it appends for
// the design being flown. What is plotted is exactly what gets saved; never reorder these.
const LIVE_COLS = ['t', 'n', 'e', 'd', 'vn', 've', 'vd', 'qw', 'qx', 'qy', 'qz', 'roll', 'pitch', 'yaw', 'p', 'q',
  'r', 'height', 'speed', 'airspeed', 'thrust', 'power', 'lift', 'battery', 'wind', 'armed', 'on_ground', 'ax', 'ay',
  'az', 'gx', 'gy', 'gz', 'alt_baro', 'flow_x', 'flow_y', 'flow_range', 'flow_q'];
let liveRun = { columns: LIVE_COLS, rows: [] };
let shownRun = null;                    // a loaded saved run, or null while the live recording is shown
let graphOn = false;                    // the graph view is the one on screen
let replayT = null;                     // scrubbed time; null means "follow the live state"
let graphTimer = null;
const picked = new Set(['power', 'battery', 'height', 'pitch']);
try { const p = JSON.parse(localStorage.getItem('airframe-graph') || 'null'); if (Array.isArray(p) && p.length) { picked.clear(); p.forEach(k => picked.add(k)); } } catch { }
const saveGraphPrefs = () => { try { localStorage.setItem('airframe-graph', JSON.stringify([...picked])); } catch { } };

// One poll at a time. Two in flight ask for the same rows, both append them, and the buffer then holds
// more samples than the recorder does — which looks exactly like a reset, so the graph would empty itself
// and refill every few seconds.
let polling = false;
async function pollLive(again = true) {
  if (polling) return;
  polling = true;
  let j;
  try { j = await api('/api/runs/current?since=' + liveRun.rows.length); }
  catch { return; }
  finally { polling = false; }
  if (j.total < liveRun.rows.length || j.since > liveRun.rows.length) {   // the sim was reset: a new run
    liveRun = { columns: j.columns || LIVE_COLS, rows: [] };
    rebuildSeries(liveRun.columns);
    replayT = null;
    if (again) await pollLive(false);                 // refill now, so the plot never blinks empty
    return;
  }
  liveRun.columns = j.columns || LIVE_COLS;
  if (j.rows && j.rows.length) liveRun.rows = liveRun.rows.concat(j.rows);
  if (!shownRun) rebuildSeries(liveRun.columns);
  updateReadouts();
}

const runData = () => shownRun || liveRun;
// The plot shows a slice of the recording: all of it until you zoom, and then whatever you zoomed to.
// Zooming is on the time axis only - the traces keep their own scales, now taken over what is on screen,
// so zooming into a quiet stretch shows what happened in it instead of a flat line.
let viewT0 = null, viewT1 = null;
const zoomed = () => viewT0 != null;
// Zoomed in with the right edge against the newest sample means "watch what is happening now": the window
// then travels with the recording, keeping its span, instead of the live end running off to the right.
// Scrolling back off the end stops that - you are looking at something, so nothing moves under you.
let followTail = true;
function viewWindow(rows) {
  const t0 = rows[0][0], t1 = rows[rows.length - 1][0];
  if (!zoomed()) return [t0, t1];
  if (followTail && replayT == null) {
    const span = Math.max(0.2, viewT1 - viewT0);
    viewT1 = t1; viewT0 = Math.max(t0, t1 - span);
  }
  const lo = Math.max(t0, Math.min(viewT0, t1 - 0.2)), hi = Math.min(t1, Math.max(lo + 0.2, viewT1));
  return [lo, hi];
}
function setWindow(lo, hi, rows) {
  const t0 = rows[0][0], t1 = rows[rows.length - 1][0];
  const span = Math.max(0.2, Math.min(hi - lo, t1 - t0));
  if (span >= (t1 - t0) - 1e-6) { viewT0 = viewT1 = null; followTail = true; }   // all of it: follows anyway
  else {
    // A pan keeps its width and stops against the ends; clamping the edges separately would squash the
    // window instead, so scrolling towards the live end would never actually arrive there.
    let a = lo, b = lo + span;
    if (b > t1) { b = t1; a = t1 - span; }
    if (a < t0) { a = t0; b = t0 + span; }
    viewT0 = a; viewT1 = b;
    // "at the end" with a little slack, so a pan that lands a pixel short still keeps up
    followTail = (t1 - viewT1) <= Math.max(0.25, 0.02 * span);
  }
  $('#scrub-fit').hidden = !zoomed();
}
function rowAt(data, t) {
  const rows = data.rows;
  if (!rows.length) return -1;
  if (t == null) return rows.length - 1;
  let lo = 0, hi = rows.length - 1;
  while (lo < hi) { const mid = (lo + hi) >> 1; if (rows[mid][0] < t) lo = mid + 1; else hi = mid; }
  return lo;
}

// Every number on screen that belongs to a column - the chips in the picker, the legend, and the ones
// chosen onto the 3D view - is written here, from the row under the cursor (or the last one).
function updateReadouts() {
  const data = runData(), rows = data.rows || [], cols = data.columns || LIVE_COLS;
  if (!rows.length) return;
  const row = rows[rowAt(data, replayT)];
  const chips = {};
  $$('[data-v]').forEach(el => (chips[el.dataset.v] = chips[el.dataset.v] || []).push(el));
  for (const key of Object.keys(chips)) {
    const ci = cols.indexOf(key);
    chips[key].forEach(el => el.textContent = ci < 0 ? '—' : formatSeries(key, row[ci]));
  }
}
// A few of them read better with what they are a share of, which is the airframe's business, not the
// recording's: power against what the motors can draw, lift against the weight.
// PX4's mode as the recorder keeps it: main mode, Auto's sub-mode behind the point (see Simulator._link_status)
const PX4_MAIN_MODES = { 1: 'Manual', 2: 'Altitude', 3: 'Position', 5: 'Acro', 6: 'Offboard', 7: 'Stabilized',
                         8: 'Rattitude', 9: 'Simple', 10: 'Termination' };
const PX4_AUTO_MODES = { 1: 'Ready', 2: 'Takeoff', 3: 'Hold', 4: 'Mission', 5: 'RTL', 6: 'Land', 8: 'Follow', 9: 'Precland' };
function modeLabel(v) {
  const main = Math.floor(v + 1e-6), sub = Math.round((v - main) * 100);
  if (!main) return '—';
  return main === 4 ? (PX4_AUTO_MODES[sub] || `Auto ${sub}`) : (PX4_MAIN_MODES[main] || `mode ${main}`);
}
function formatSeries(key, v) {
  if (!isFinite(v)) return '—';
  const u = (describeSeries(key) || {}).unit || '';
  const num = Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 10 ? v.toFixed(1) : v.toFixed(2);
  if (key === 'power' && airframe) {
    const cap = airframe.rotors.reduce((a, r) => a + (r.enabled === false ? 0 : (+r.max_power_w || 0)), 0);
    if (cap > 0) return `${v.toFixed(0)} W  ${(100 * v / cap).toFixed(0)}%`;
  }
  if (key === 'lift' && airframe && airframe.mass) {
    const w = (+airframe.mass.mass || 0) * 9.80665;
    if (w > 0) return `${v.toFixed(0)} N  ${(100 * v / w).toFixed(0)}%`;
  }
  if (key === 'armed' || key === 'on_ground') return v > 0.5 ? 'yes' : 'no';
  if (key === 'mode') return modeLabel(v);
  if (key === 'battery') return `${v.toFixed(0)} %${batteryMinutes == null ? '' : '  ' + batteryMinutes.toFixed(0) + ' min'}`;
  return num + (u ? ' ' + u : '');
}
function drawGraph() {
  if (!graphOn) return;
  const host = $('#graph-plot'), data = runData(), rows = data.rows || [];
  const cols = data.columns || LIVE_COLS;
  const W = Math.max(120, host.clientWidth), H = Math.max(60, host.clientHeight);
  const T = 6, B = 14, L = 2, R = 2, ph = H - T - B, pw = W - L - R;
  const cursor = rowAt(data, replayT);
  updateReadouts();
  if (rows.length < 2) {
    host.innerHTML = `<svg viewBox="0 0 ${W} ${H}"><text x="${W / 2}" y="${H / 2}" text-anchor="middle">`
      + `${shownRun ? 'this run has no samples' : 'nothing recorded yet — start the simulation'}</text></svg>`;
    $('#scrub-label').textContent = '—';               // never a time that belongs to a plot that is not there
    return;
  }
  const t1 = rows[rows.length - 1][0];
  const [vt0, vt1] = viewWindow(rows), span = Math.max(1e-6, vt1 - vt0);
  const x = (t) => L + pw * (t - vt0) / span;
  const i0 = Math.max(0, rowAt(data, vt0) - 1), i1 = Math.min(rows.length - 1, rowAt(data, vt1) + 1);
  const stride = Math.max(1, Math.ceil((i1 - i0 + 1) / Math.max(200, pw)));

  let g = '';
  for (let i = 0; i <= 4; i++) { const y = T + ph * i / 4; g += `<line class="grid" x1="${L}" y1="${y.toFixed(1)}" x2="${W - R}" y2="${y.toFixed(1)}"/>`; }
  const step = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 30, 60, 120, 300, 600].find(s => span / s <= 8) || Math.ceil(span / 8);
  for (let t = Math.ceil(vt0 / step) * step; t <= vt1; t += step) {
    g += `<line class="grid" x1="${x(t).toFixed(1)}" y1="${T}" x2="${x(t).toFixed(1)}" y2="${T + ph}"/>`
      + `<text x="${x(t).toFixed(1)}" y="${H - 3}" text-anchor="middle">${t.toFixed(step < 1 ? 1 : 0)}s</text>`;
  }

  let lines = '';
  for (const s of SERIES) {
    const ci = cols.indexOf(s.key);
    if (ci < 0 || !picked.has(s.key)) continue;
    // Each trace gets its own scale, since they share no unit. The scale includes zero unless the trace
    // actually moves through a good part of its own range: without that, a quantity sitting still at 0.35 m
    // with a millimetre of noise would be drawn as a full-height sawtooth and read as violent motion.
    let lo = Infinity, hi = -Infinity;
    // the same stride the line is drawn with: at twenty frames a second, scanning every row of a long
    // recording for its extremes costs more than drawing it, and the pixels are identical
    for (let i = i0; i <= i1; i += stride) { const v = rows[i][ci]; if (v < lo) lo = v; if (v > hi) hi = v; }
    const z0 = Math.min(lo, 0), z1 = Math.max(hi, 0);
    if ((hi - lo) <= 0.25 * (z1 - z0)) { lo = z0; hi = z1; }
    const pad2 = (hi - lo) < 1e-9 ? 1 : (hi - lo) * 0.08;
    lo -= pad2; hi += pad2;
    const y = (v) => T + ph * (1 - (v - lo) / (hi - lo));
    let d = '';
    for (let i = i0; i <= i1; i += stride) d += (d ? 'L' : 'M') + x(rows[i][0]).toFixed(1) + ' ' + y(rows[i][ci]).toFixed(1);
    d += 'L' + x(rows[i1][0]).toFixed(1) + ' ' + y(rows[i1][ci]).toFixed(1);
    lines += `<path class="ln" d="${d}" stroke="${s.c}"/>`;
  }
  const ct = rows[cursor][0], cx = x(ct).toFixed(1);
  const cur = replayT == null || ct < vt0 || ct > vt1 ? ''
    : `<line class="cursor" x1="${cx}" y1="${T}" x2="${cx}" y2="${T + ph}"/>`;
  host.innerHTML = `<svg viewBox="0 0 ${W} ${H}">${g}${lines}${cur}</svg>`;
  $('#scrub-label').textContent = `${pad(ct, 1, 6)} / ${t1.toFixed(1)} s`
    + (zoomed() ? ` · ${span.toFixed(span < 10 ? 1 : 0)} s shown` : '')
    + (replayT == null ? ' · live' : playing ? ' · playing the recording' : ' · scrubbed, the 3D view is at this instant');
  $('#scrub-resume').hidden = replayT == null;
  $('#scrub-play').textContent = playing ? 'Pause' : 'Play';
  $('#scrub-play').classList.toggle('primary', playing);
}

// The traces are chosen from one dropdown of checkboxes; the ones that are on are listed beside it with
// their value at the cursor, so the row stays a single line however many are plotted.
function pickerHtml(set, withValue) {
  const has = (k) => set.has ? set.has(k) : set.includes(k);
  const cats = [...new Set(SERIES.map(s => s.cat))].sort((a, b) => CAT_ORDER.indexOf(a) - CAT_ORDER.indexOf(b));
  return cats.map(cat => `<div class="cat">${esc(cat)}</div>`
    + SERIES.filter(s => s.cat === cat).sort((a, b) => seriesRank(a.key) - seriesRank(b.key)).map(s =>
      `<label data-key="${s.key}" style="--c:${s.c}" title="${esc(s.desc || s.label)}">`
      + `<input type="checkbox" ${has(s.key) ? 'checked' : ''}>`
      + `<i class="sw"></i><span class="nm">${esc(s.label)}</span>`
      + (withValue ? `<b class="num" data-v="${esc(s.key)}"></b>` : '') + `</label>`).join('')).join('');
}
function buildPicker() {
  const pop = $('#series-pop');
  pop.innerHTML = pickerHtml(picked, true);
  pop.querySelectorAll('label').forEach(lab => {
    const key = lab.dataset.key;
    lab.querySelector('input').addEventListener('change', (e) => {
      if (e.target.checked) picked.add(key); else picked.delete(key);
      saveGraphPrefs(); buildLegend(); drawGraph();
    });
  });
  buildLegend();
  updateReadouts();
}
// The same catalogue, chosen onto the 3D view instead of into the plot.
let batteryMinutes = null;             // what the gauge knows and the recording does not
// A list, not a set: the order is the order they sit in on the view, and dragging one changes it.
let overlayPicked = ['battery', 'power', 'lift', 'speed', 'height', 'wind'];
try { const p = JSON.parse(localStorage.getItem('airframe-overlay') || 'null'); if (Array.isArray(p)) overlayPicked = [...new Set(p)]; } catch { }
const saveOverlay = () => {
  overlayPicked = [...new Set(overlayPicked)];       // one chip per quantity, however it was dragged
  try { localStorage.setItem('airframe-overlay', JSON.stringify(overlayPicked)); } catch { }
};
// Dragging a chip never touches the DOM while the pointer is down: the old version moved nodes about as
// you went, and a redraw landing mid-drag left it holding a detached node, which it then put back - that
// is where the duplicates came from. Now the move is worked out from the pointer and applied once, on
// release, from the list rather than from the elements.
let chipDrag = null;
function renderOverlayChips() {
  const host = $('#overlay-chips');
  if (!host) return;
  overlayPicked = [...new Set(overlayPicked)];
  const known = new Map(SERIES.map(s => [s.key, s]));
  host.innerHTML = overlayPicked.filter(k => known.has(k)).map(k => {
    const s = known.get(k);
    return `<span class="pill st ro chip" data-k="${esc(k)}" title="${esc(s.desc || s.label)} · drag to move it">`
      + `<span class="k">${esc(s.label)}</span><b data-v="${esc(k)}">—</b></span>`;
  }).join('');
  host.querySelectorAll('.chip').forEach(el => {
    el.addEventListener('pointerdown', (e) => {
      if (e.button !== 0) return;
      chipDrag = { key: el.dataset.k, el, x0: e.clientX, moved: false, target: null, before: false };
      el.setPointerCapture(e.pointerId);
    });
    el.addEventListener('pointermove', (e) => {
      const d = chipDrag;
      if (!d || d.el !== el) return;
      if (!d.moved && Math.abs(e.clientX - d.x0) < 4) return;
      d.moved = true;
      el.classList.add('dragging');
      const chips = [...host.querySelectorAll('.chip')];
      chips.forEach(c => c.classList.remove('drop-before', 'drop-after'));
      const over = chips.find(c => {
        const r = c.getBoundingClientRect();
        return e.clientX >= r.left && e.clientX <= r.right && e.clientY >= r.top - 8 && e.clientY <= r.bottom + 8;
      });
      if (over && over !== el) {
        const r = over.getBoundingClientRect();
        d.before = e.clientX < r.left + r.width / 2;
        d.target = over.dataset.k;
        over.classList.add(d.before ? 'drop-before' : 'drop-after');
      } else d.target = null;
    });
    const finish = () => {
      const d = chipDrag;
      chipDrag = null;
      el.classList.remove('dragging');
      host.querySelectorAll('.chip').forEach(c => c.classList.remove('drop-before', 'drop-after'));
      if (!d || !d.moved || !d.target) return;
      const list = overlayPicked.filter(k => k !== d.key);
      const at = list.indexOf(d.target) + (d.before ? 0 : 1);
      list.splice(Math.max(0, at), 0, d.key);
      overlayPicked = list;
      saveOverlay();
      renderOverlayChips();
    };
    el.addEventListener('pointerup', finish);
    el.addEventListener('pointercancel', finish);
  });
  const pop = $('#overlay-pop');
  pop.innerHTML = pickerHtml(overlayPicked, false);
  pop.querySelectorAll('label').forEach(lab => {
    const key = lab.dataset.key;
    lab.querySelector('input').addEventListener('change', (e) => {
      overlayPicked = e.target.checked ? [...overlayPicked.filter(k => k !== key), key]
                                       : overlayPicked.filter(k => k !== key);
      saveOverlay();
      renderOverlayChips();
    });
  });
  updateReadouts();
}
$('#overlay-btn').addEventListener('click', (e) => {
  e.stopPropagation();
  const pop = $('#overlay-pop');
  pop.hidden = !pop.hidden;
  if (!pop.hidden) {                       // fixed, so it is not cut off by the panel it sits in
    const r = e.currentTarget.getBoundingClientRect();
    pop.style.left = Math.max(8, Math.min(r.left, innerWidth - 340)) + 'px';
    pop.style.top = (r.bottom + 6) + 'px';
  }
});
document.addEventListener('click', (e) => {
  if ($('#overlay-pop').hidden || e.composedPath().includes($('#overlay-menu'))) return;
  $('#overlay-pop').hidden = true;
});
function buildLegend() {
  $('#series-btn').textContent = `Data (${picked.size})`;
  $('#series-legend').innerHTML = SERIES.filter(s => picked.has(s.key)).map(s =>
    `<span class="lg" style="--c:${s.c}"><i class="sw"></i>${esc(s.label)} <b class="num" data-v="${esc(s.key)}"></b></span>`).join('')
    || '<span class="hint">no traces chosen</span>';
}
$('#series-btn').addEventListener('click', (e) => {
  e.stopPropagation();
  $('#series-pop').hidden = !$('#series-pop').hidden;
  $('#series-btn').classList.toggle('active', !$('#series-pop').hidden);
});
document.addEventListener('click', (e) => {
  if ($('#series-pop').hidden || e.target.closest('.series-menu')) return;
  $('#series-pop').hidden = true; $('#series-btn').classList.remove('active');
});
document.addEventListener('keydown', (e) => {
  if (e.key === 'Escape' && !$('#series-pop').hidden) { $('#series-pop').hidden = true; $('#series-btn').classList.remove('active'); }
});
buildPicker();

// Scrubbing: the slider is a clock. Everything the 3D view shows is the recorded pose at that instant.
let replayRotors = null;        // the motors at the scrubbed instant, so the live state cannot overwrite them
function applyReplay() {
  const data = runData(), rows = data.rows || [], cols = data.columns || LIVE_COLS;
  if (!rows.length) return;
  const r = rows[rowAt(data, replayT)];
  const i = (k) => cols.indexOf(k);
  if (i('n') < 0 || i('qw') < 0) return;
  // The motors are recorded too, so the discs, the thrust arrows and the sliders show what each one was
  // doing at this instant instead of going blank while you scrub. A run saved before those columns
  // existed has none, and then they stay empty rather than claim everything was at zero.
  const rotors = [];
  for (let k = 1; i(`m${k}_cmd`) >= 0; k++) {
    rotors.push({ cmd: r[i(`m${k}_cmd`)], omega: r[i(`m${k}_spool`)] ?? r[i(`m${k}_cmd`)], thrust: r[i(`m${k}_thrust`)] ?? 0 });
  }
  scene.updateState({ pos: [r[i('n')], r[i('e')], r[i('d')]], q: [r[i('qw')], r[i('qx')], r[i('qy')], r[i('qz')]],
                      forces: {}, rotors });
  replayRotors = rotors.length ? rotors : null;
  if (rotors.length) updateMotorSliders({ rotors });
}
// Dragging anywhere on the plot moves the aircraft to that instant. The simulation stops while you are in
// the recording — a 3D view that is partly a replay and partly live would be a lie — and Resume starts it again.
let pausedByScrub = false;
function scrubTo(clientX) {
  const data = runData(), rows = data.rows || [];
  if (rows.length < 2) return;
  replayT = timeAt(clientX, rows);
  applyReplay(); drawGraph();
}
function timeAt(clientX, rows) {
  const box = $('#graph-plot').getBoundingClientRect();
  const f = Math.max(0, Math.min(1, (clientX - box.left - 2) / Math.max(1, box.width - 4)));
  const [vt0, vt1] = viewWindow(rows);
  return vt0 + f * (vt1 - vt0);
}
$('#graph-plot').addEventListener('pointerdown', async (e) => {
  const rows = (runData().rows || []);
  if (rows.length < 2) return;
  e.preventDefault();
  stopPlaying();
  // capture keeps the drag alive when the pointer leaves the plot; a browser that refuses it must not
  // take the scrub down with it
  try { $('#graph-plot').setPointerCapture(e.pointerId); } catch { }
  pauseForReplay();
  scrubTo(e.clientX);
});
$('#graph-plot').addEventListener('pointermove', (e) => { if (e.buttons & 1) scrubTo(e.clientX); });
// Playing the recording back: the cursor is a clock that runs on its own, and the 3D view follows it. The
// simulation stays paused throughout - the view cannot be half replay and half live.
let playing = false, playFrom = 0, playWall = 0, playFrame = 0;
const playSpeed = () => +$('#scrub-speed').value || 1;
// changing the speed mid-play keeps the cursor where it is and runs on from there
$('#scrub-speed').addEventListener('change', () => { if (playing) { playFrom = replayT; playWall = performance.now(); } });
let pauseAskedAt = 0;        // the status takes a moment to agree that we paused; until then, do not read it
async function pauseForReplay() {
  if (!shownRun && !status.paused && !pausedByScrub) {
    pausedByScrub = true;
    pauseAskedAt = Date.now();
    try { await api('/api/sim/pause', { paused: true }); status.paused = true; }
    catch (err) { logLine('[ui] ' + err.message); }
  }
}
function stopPlaying() {
  playing = false;
  cancelAnimationFrame(playFrame);
  drawGraph();
}
function playTick() {
  if (!playing) return;
  const data = runData(), rows = data.rows || [];
  if (rows.length < 2) return stopPlaying();
  const t1 = rows[rows.length - 1][0];
  replayT = playFrom + (performance.now() - playWall) / 1000 * playSpeed();
  if (replayT >= t1) { replayT = t1; applyReplay(); return stopPlaying(); }
  // keep the cursor on screen when the view is zoomed in
  if (zoomed()) {
    const span = viewT1 - viewT0;
    if (replayT > viewT1 - span * 0.1) setWindow(replayT - span * 0.9, replayT + span * 0.1, rows);
    else if (replayT < viewT0) setWindow(replayT, replayT + span, rows);
  }
  applyReplay(); drawGraph();
  playFrame = requestAnimationFrame(playTick);
}
$('#scrub-play').addEventListener('click', async () => {
  if (playing) return stopPlaying();
  const data = runData(), rows = data.rows || [];
  if (rows.length < 2) return;
  await pauseForReplay();
  const t0 = viewWindow(rows)[0], t1 = rows[rows.length - 1][0];
  if (replayT == null || replayT >= t1 - 1e-3) replayT = t0;      // at the end, or live: start from the left
  playFrom = replayT; playWall = performance.now();
  playing = true;
  playFrame = requestAnimationFrame(playTick);
  drawGraph();
});
$('#scrub-fit').addEventListener('click', () => {
  viewT0 = viewT1 = null; $('#scrub-fit').hidden = true; drawGraph();
});
// Pinch on a trackpad arrives as a wheel event with ctrlKey: zoom the time axis about the pointer. A plain
// two-finger scroll pans, so the two gestures do what they do everywhere else.
$('#graph-plot').addEventListener('wheel', (e) => {
  const rows = (runData().rows || []);
  if (rows.length < 2) return;
  e.preventDefault();
  const [vt0, vt1] = viewWindow(rows), span = vt1 - vt0;
  if (e.ctrlKey || e.metaKey || Math.abs(e.deltaY) > Math.abs(e.deltaX)) {
    const at = timeAt(e.clientX, rows);
    const k = Math.exp((e.ctrlKey || e.metaKey ? e.deltaY * 0.01 : e.deltaY * 0.0015));
    const next = Math.max(0.2, Math.min(rows[rows.length - 1][0] - rows[0][0], span * k));
    const f = span > 0 ? (at - vt0) / span : 0.5;
    setWindow(at - next * f, at + next * (1 - f), rows);
  } else {
    const dt = span * e.deltaX * 0.002;
    setWindow(vt0 + dt, vt1 + dt, rows);
  }
  drawGraph();
}, { passive: false });
$('#graph-plot').addEventListener('dblclick', () => { viewT0 = viewT1 = null; $('#scrub-fit').hidden = true; drawGraph(); });
$('#scrub-resume').addEventListener('click', async () => {
  stopPlaying();
  replayT = null;
  if (shownRun) await selectRun('');            // a kept run has no "now" to go back to: return to the live one
  if (pausedByScrub) {
    pausedByScrub = false;
    try { await api('/api/sim/pause', { paused: false }); } catch (err) { logLine('[ui] ' + err.message); }
  }
  drawGraph();
});

function showGraph(on) {
  graphOn = on;
  $('#log-body').hidden = on;
  $('#graph-body').hidden = !on;
  $('#view-log').classList.toggle('active', !on);
  $('#view-graph').classList.toggle('active', on);
  const log = $('#log');
  if (on) {
    log.classList.remove('collapsed');
    if (log.getBoundingClientRect().height < 260) log.style.height = '300px';
    loadRunList();
    drawGraph();
    clearInterval(graphTimer);
    const tick = async () => {
      if (!graphOn) return;
      // Pause was released somewhere else (the button over the 3D view): the recording is no longer where
      // the aircraft is, so go back to following it.
      if (replayT != null && pausedByScrub && !status.paused && Date.now() - pauseAskedAt > 1500) {
        replayT = null; pausedByScrub = false; stopPlaying();
      }
      if (!shownRun) await pollLive();
      drawGraph();
    };
    tick();
    // The recorder keeps twenty samples a second (fifty down a cable). Drawing four times a second threw
    // four fifths of that away and made every trace a staircase; this draws every sample it has.
    graphTimer = setInterval(tick, 50);
  } else {
    clearInterval(graphTimer); graphTimer = null;
  }
  liveTick();
  try { localStorage.setItem('airframe-logview', on ? 'graph' : 'log'); } catch { }
}
// The overlay reads the same recording as the graph, so it is polled whether or not the graph is on screen.
let liveTimer = null;
async function liveTick() {
  clearTimeout(liveTimer);
  if (!graphOn && !shownRun) { await pollLive(); updateReadouts(); }
  liveTimer = setTimeout(liveTick, 200);
}
liveTick();
$('#view-log').addEventListener('click', () => showGraph(false));
$('#view-graph').addEventListener('click', () => showGraph(true));
new ResizeObserver(() => drawGraph()).observe($('#graph-plot'));

// ------------------------------------------------------------ the list of kept runs
function runLabel(r) {
  const when = new Date((r.started || 0) * 1000).toLocaleString(undefined, { month: 'short', day: 'numeric', hour: '2-digit', minute: '2-digit' });
  return `${r.name || 'flight'} · ${when} · ${(+r.duration || 0).toFixed(0)} s`;
}
async function loadRunList(select) {
  let j;
  try { j = await api('/api/runs'); } catch { return; }
  const pick = $('#run-pick'), want = select !== undefined ? select : pick.value;
  // only this design's flights: a run from another airframe is not a comparison, it is a different aircraft
  const mine = (j.runs || []).filter(r => !airframe || !r.airframe_name || r.airframe_name === airframe.name);
  pick.innerHTML = '<option value="">Live</option>'
    + mine.map(r => `<option value="${esc(r.id)}">${esc(runLabel(r))}</option>`).join('');
  pick.value = mine.some(r => r.id === want) ? want : '';
  if (pick.value !== want) selectRun('');
  runButtons();
}
function runButtons() {
  const has = !!$('#run-pick').value;
  ['#run-rename', '#run-delete', '#run-load'].forEach(s => $(s).hidden = !has);
  $('#run-save').hidden = has;         // a saved run is already saved; this button is for the live one
}
async function selectRun(id) {
  $('#run-pick').value = id || '';        // the picker says what is on screen, however we got here
  if (!id) {
    shownRun = null; replayT = null;
    shownRunLog = null; showLogText(flightLog);
  } else {
    try { shownRun = await api('/api/runs/' + encodeURIComponent(id)); }
    catch (err) { logLine('[runs] ' + err.message); shownRun = null; }
    replayT = shownRun && shownRun.rows.length ? shownRun.rows[0][0] : null;
    shownRunLog = (shownRun && shownRun.log)
      ? shownRun.log.map(l => String(l[1]).replace(/^\[px4\] /, '')).join('\n') : '';
    showLogText(shownRunLog || 'this run was saved without the flight computer log');
    if (shownRun) rebuildSeries(shownRun.columns);
    applyReplay();
  }
  runButtons();
  drawGraph();
}
$('#run-pick').addEventListener('change', (e) => selectRun(e.target.value));
$('#run-reset').addEventListener('click', async (e) => {
  if (!confirmTwice('run-reset', e.currentTarget, 'throw this recording away')) return;
  if (shownRun) await selectRun('');     // whatever you were looking at, this ends on the live recording
  try {
    await api('/api/runs/clear', {});
    liveRun = { columns: LIVE_COLS, rows: [] };
    flightLog = ''; shownRunLog = null; showLogText('');   // the terminal belongs to the same run
    replayT = null; viewT0 = viewT1 = null; $('#scrub-fit').hidden = true;
    stopPlaying();
    drawGraph();
    console.log('[runs] graph and log reset; the simulation kept running');
  } catch (err) { logLine('[runs] ' + err.message); }
});
$('#run-save').addEventListener('click', async (e) => {
  e.target.disabled = true;
  try {
    const r = await api('/api/runs/save', { name: '', clear: true });
    if (r.ok) {
      liveRun = { columns: LIVE_COLS, rows: [] };     // the graph starts again, the simulation never stopped
      flightLog = ''; shownRunLog = null; showLogText('');   // and so does the log: that run has both halves
      replayT = null; viewT0 = viewT1 = null; $('#scrub-fit').hidden = true;
      stopPlaying();
      await loadRunList('');
      drawGraph();
      logLine(`[runs] saved ${r.id} (${(+r.duration).toFixed(1)} s) · the graph starts again`);
    } else logLine('[runs] nothing to save: ' + (r.error || ''));
  } catch (err) { logLine('[runs] ' + err.message); }
  e.target.disabled = false;
});
$('#run-rename').addEventListener('click', () => {
  const id = $('#run-pick').value;
  if (!id) return;
  const box = $('#run-name');
  box.hidden = false; $('#run-pick').hidden = true;
  box.value = (shownRun && shownRun.name) || '';
  box.focus(); box.select();
  const done = async (commit) => {
    box.hidden = true; $('#run-pick').hidden = false;
    box.onblur = box.onkeydown = null;
    if (!commit) return;
    try { await api(`/api/runs/${encodeURIComponent(id)}/rename`, { name: box.value }); if (shownRun) shownRun.name = box.value; await loadRunList(id); }
    catch (err) { logLine('[runs] ' + err.message); }
  };
  box.onblur = () => done(true);
  box.onkeydown = (ev) => { if (ev.key === 'Enter') done(true); if (ev.key === 'Escape') done(false); };
});
$('#run-delete').addEventListener('click', async (e) => {
  const id = $('#run-pick').value;
  if (!id || !confirmTwice('run-delete', e.target, 'delete')) return;
  try { await api(`/api/runs/${encodeURIComponent(id)}/delete`, {}); await selectRun(''); await loadRunList(''); }
  catch (err) { logLine('[runs] ' + err.message); }
});
$('#run-load').addEventListener('click', async (e) => {
  const id = $('#run-pick').value;
  if (!id || !confirmTwice('run-load', e.target, 'load this design')) return;
  try {
    const r = await api(`/api/runs/${encodeURIComponent(id)}/load`, {});
    if (r.airframe) { setAirframe(r.airframe); logLine('[runs] loaded the design from ' + id); }
  } catch (err) { logLine('[runs] ' + err.message); }
});
try { if (localStorage.getItem('airframe-logview') === 'graph') showGraph(true); } catch { }
// Light or dark is the computer's business, not a button here: follow it, and keep following when it changes.
const systemDark = window.matchMedia('(prefers-color-scheme: dark)');
function applySystemTheme() {
  const name = systemDark.matches ? 'dark' : 'light';
  document.documentElement.dataset.theme = name;
  scene.setTheme(name);
}
systemDark.addEventListener('change', applySystemTheme);
applySystemTheme();

function connectWs() {
  const ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws');
  joyWs = ws;
  ws.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.type === 'airframe') { setAirframe(m.airframe); }
    if (m.type === 'state') { lastRc = m.rc || {}; lastManual = m.manual || {}; applyState(m.state); if (m.status) applyStatus(m.status); if (m.log) m.log.forEach(l => logLine(l[1])); if (rcVisible()) renderRcLive(); }
  };
  ws.onclose = () => { $('#link-light i').classList.remove('on'); console.log('[ui] websocket closed; reconnecting'); setTimeout(connectWs, 800); };
}
connectWs();
loadPresetList();
loadVersionList();
api('/api/log?since=0').then(lines => lines.forEach(l => logLine(l[1]))).catch(() => { });


// ============================================================ sections
// Every section on every tab is the same thing: a white box with its title as the handle. Which ones are
// open is the user's, so it is remembered; a section with nothing in it yet still opens, because an empty
// box that says what belongs in it is clearer than no box.
(function () {
  let open = {};
  try { open = JSON.parse(localStorage.getItem('airframe-sections') || '{}'); } catch { }
  $$('details.sec').forEach(d => {
    if (d.id && open[d.id] !== undefined) d.open = !!open[d.id];
    d.addEventListener('toggle', () => {
      if (!d.id) return;
      open[d.id] = d.open;
      try { localStorage.setItem('airframe-sections', JSON.stringify(open)); } catch { }
    });
  });
})();

// ============================================================ resizable panels
(function () {
  const side = $('#side'), log = $('#log');
  let layout = {};
  try { layout = JSON.parse(localStorage.getItem('airframe-layout') || '{}'); } catch { }
  if (layout.sideW) side.style.width = layout.sideW + 'px';
  if (layout.logH) log.style.height = layout.logH + 'px';
  const save = () => { try { localStorage.setItem('airframe-layout', JSON.stringify(layout)); } catch { } };

  function drag(handle, axis, onMove) {
    handle.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      handle.setPointerCapture(e.pointerId);
      handle.classList.add('dragging');
      document.body.classList.add('resizing', axis === 'x' ? 'resizing-x' : 'resizing-y');
      const start = axis === 'x' ? e.clientX : e.clientY;
      const move = (ev) => onMove((axis === 'x' ? ev.clientX : ev.clientY) - start);
      const up = () => {
        handle.classList.remove('dragging');
        document.body.classList.remove('resizing', 'resizing-x', 'resizing-y');
        handle.removeEventListener('pointermove', move); handle.removeEventListener('pointerup', up);
        save();
      };
      handle.addEventListener('pointermove', move); handle.addEventListener('pointerup', up);
      onMove.begin && onMove.begin();
    });
  }

  const sideMove = (dx) => {
    const w = Math.min(window.innerWidth * 0.7, Math.max(380, sideMove.startW - dx));
    side.style.width = w + 'px'; layout.sideW = Math.round(w);
  };
  sideMove.begin = () => { sideMove.startW = side.getBoundingClientRect().width; };
  drag($('#resize-side'), 'x', sideMove);

  const logMove = (dy) => {
    log.classList.remove('collapsed');
    const h = Math.min(window.innerHeight * 0.7, Math.max(48, logMove.startH - dy));
    log.style.height = h + 'px'; layout.logH = Math.round(h);
  };
  logMove.begin = () => { logMove.startH = log.getBoundingClientRect().height; };
  drag($('#resize-log'), 'y', logMove);
})();


// ============================================================ the flight controller, and where it runs
// One choice, not two: SITL (PX4 on this computer), HITL (the board over USB, flying the simulation) or
// Real flight (the board on its own sensors and outputs). The old page asked what to connect to and then,
// separately, what state to be in, and kept a checklist of everything that might still be wrong.
let connTimer = null;
const LINK_LABEL = { sitl: 'SITL', hitl: 'Pixhawk (HITL)', real: 'Pixhawk (real flight)' };
let lastPorts = [];

// Which of the three the app is in. One function, because two of them disagreeing is what made the pill
// flicker between HITL and FLIGHT: the status poll knew the link was only listening and the connection
// poll did not, and each repainted the switch twice a second with its own answer.
const linkReal = (s) => !s.conn_mode ? '' : s.conn_mode === 'sitl' ? 'sitl'
  : (s.hil_enabled === false || s.listening ? 'real' : 'hitl');
function modeShown(s) { return modeWanted || linkReal(s || status || {}); }
// Pressing one of the three is a decision, not a request that can be refused: the switch moves there and
// stays there. Whether it is actually working is the light's job - green when the link is doing what the
// mode says it should, red when it is not, with the reason on hover.
let modeWanted = null;
function wantMode(k) { modeWanted = k; paintModeSwitch(status); }
// One press, whatever it takes. SITL, HITL and FLIGHT are statements about what the aircraft *is*, so
// the app makes them true: a board left in HITL has its own sensors switched off and cannot fly for real,
// a board on its own sensors cannot be driven by the simulator, and both take a parameter, a save and a
// reboot that nobody should have to remember. The work happens on the server, bounded and in the
// background; this only says where it has got to.
let modeJob = null, modeTimer = null;
async function switchMode(mode, btn) {
  if (mode === 'real' && !confirmTwice('listen-flight', btn, 'stop simulating and listen to the aircraft')) return;
  wantMode(mode);
  modeJob = { mode, state: 'running', step: 'starting' };
  paintModeSwitch(status);
  try {
    // The port is only a hint: with nothing connected the server picks one, and with a link already up it
    // keeps it. Asking the ports list here is what used to make this fail from a tab that never polls it.
    const dev = (status.conn_mode === 'hitl' && status.ctl_connected) ? null : ((await boardPort(true)) || {}).device;
    await api('/api/connection/mode', { mode, serial: dev, baud: chosenBaud() });
  } catch (e) {
    modeJob = { mode, state: 'failed', error: e.message };
    markSaved(e.message, 5000);
  }
  pollMode();
}
async function pollMode() {
  clearTimeout(modeTimer);
  try { modeJob = (await api_get('/api/connection/mode')).job; } catch { }
  paintModeSwitch(status);
  if (modeJob && modeJob.state === 'running') { modeTimer = setTimeout(pollMode, 600); return; }
  if (modeJob && modeJob.state === 'failed') markSaved(modeJob.error, 6000);
  refreshConnection(true);
  paramsLoaded = false;
}
const goSitl = () => switchMode('sitl');
// The switch works from any tab, so it cannot rely on a port list that only the PX4 tab keeps up to date,
// and it cannot answer in a box on a tab you are not looking at: it says what happened in the bar.
async function boardPort(fresh = false) {
  if (fresh || !lastPorts.length) {
    try { lastPorts = (await api('/api/connection')).ports || []; } catch { if (!lastPorts.length) return null; }
  }
  return lastPorts.find(p => p.likely_px4) || lastPorts[0] || null;
}
const goHitl = (btn) => switchMode('hitl', btn);
const goFlight = (btn) => switchMode('real', btn);
$('#conn-baud').addEventListener('change', (e) => { e.target.dataset.touched = '1'; });
$('#conn-sitl').addEventListener('click', goSitl);
$('#conn-hitl').addEventListener('click', (e) => goHitl(e.currentTarget));
$$('#mode-switch button').forEach(b => b.addEventListener('click', (e) => {
  const k = b.dataset.link;
  if (k === 'sitl') goSitl(); else if (k === 'hitl') goHitl(e.currentTarget); else goFlight(e.currentTarget);
}));
$('#conn-real').addEventListener('click', async (e) => {
  const b = e.currentTarget;
  if (status.armed || !status.ctl_connected || !hardwareMotors()) { $('#real-flight-result').textContent = 'Connect a disarmed Pixhawk over USB first.'; return; }
  if (!confirmTwice('real-flight-setup', b, 'prepare the board for real flight')) return;
  stopMotorTest(); $('#motor-enable').checked = false; b.disabled = true;
  $('#real-flight-result').textContent = 'Preparing real flight… watch the log. Keep the board connected.';
  try {
    const r = await api('/api/firmware/real_flight', { confirmed: true });
    $('#real-flight-result').textContent = r.ok ? r.message : 'Setup incomplete: ' + r.error;
    paramsLoaded = false;
  } catch (err) { $('#real-flight-result').textContent = 'Setup incomplete: ' + err.message; }
  finally { b.disabled = false; b.textContent = 'Set up for real flight'; refreshConnection(); }
});
$('#conn-rescan').addEventListener('click', refreshConnection);
$('#conn-disconnect').addEventListener('click', async () => { await connCall('/api/connection/disconnect', {}); });
$('#conn-reboot').addEventListener('click', async (e) => { if (confirmTwice('conn-reboot', e.currentTarget, 'reboot the board')) { await api('/api/px4/command', { command: 'reboot' }); setTimeout(refreshConnection, 3000); } });
$('#conn-hw-ok').addEventListener('click', async (e) => {
  if (!confirmTwice('enable_hw', e.currentTarget, 'allow motors on the real aircraft')) return;
  await api('/api/params/set', { name: 'NLF_HW_OK', value: 1 });
  paramsLoaded = false; setTimeout(refreshConnection, 1500);
});
// Over a radio the streams are a budget, not a wish: this says what was asked for, what is arriving, and
// how the radio itself is coping. A gap between the first two means the link is full, and the dial beside
// it is how you trade resolution for keeping up.
function showTelemetry(L) {
  const row = $('#tele-row'), t = (L || {}).telemetry;
  if (!row) return;
  row.hidden = !(t && t.budget_bps);                 // a cable has no budget worth showing
  if (row.hidden) return;
  if (document.activeElement !== $('#tele-share')) $('#tele-share').value = t.share || 'balanced';
  const kb = (b) => (b >= 1000 ? (b / 1000).toFixed(1) + ' kB/s' : Math.round(b) + ' B/s');
  const radio = (L || {}).radio;
  const buf = radio && radio.age < 5 && radio.txbuf != null ? ` · radio buffer ${radio.txbuf}%` : '';
  const sig = radio && radio.age < 5 ? ` · rssi ${radio.rssi}/${radio.remrssi}` : '';
  const short = t.asked_bps && t.bytes_s < 0.6 * t.asked_bps;
  // When the aircraft's own MAVLink cap is what is holding this back, say so and name the parameter:
  // no amount of turning this dial up will get past it.
  const cap = t.board_cap_bps && t.budget_bps && t.board_cap_bps <= t.budget_bps + 1
    ? ` · the aircraft caps this port at ${kb(t.board_cap_bps)} (MAV_0_RATE)` : '';
  $('#tele-now').innerHTML = `<span class="${short ? 'warn' : ''}">${t.msg_s} msg/s · ${kb(t.bytes_s)}`
    + ` of ${kb(t.asked_bps)} asked</span><span class="hint">${buf}${sig}${cap}</span>`;
}
$('#tele-share').addEventListener('change', async (e) => {
  try { await api('/api/connection/telemetry', { share: e.target.value }); markSaved('Telemetry: ' + e.target.value, 3000); }
  catch (err) { $('#conn-error').textContent = err.message; }
  refreshConnection(true);
});
async function connectHitl(serial) {
  // Picking a port in the list means "use this one" - which mode it ends up in is the switch's business,
  // and HITL is what this list has always meant.
  return switchModeOn(serial);
}
async function switchModeOn(serial) {
  wantMode('hitl');
  modeJob = { mode: 'hitl', state: 'running', step: 'opening the link' };
  paintModeSwitch(status);
  try { await api('/api/connection/mode', { mode: 'hitl', serial, baud: chosenBaud() }); }
  catch (e) { $('#conn-error').textContent = e.message; }
  pollMode();
}
// An untouched Baud box is not an instruction: the server opens a Pixhawk fast and a radio at 57600, and
// gets it right more often than a box remembering the last board you plugged in.
const chosenBaud = () => ($('#conn-baud').dataset.touched ? +$('#conn-baud').value || undefined : undefined);
async function connCall(path, body) {
  $('#conn-error').textContent = '';
  $('#conn-current').innerHTML = '<span class="muted">Working…</span>';
  try { await api(path, body); paramsLoaded = false; } catch (e) { $('#conn-error').textContent = e.message; }
  await refreshConnection(true);
}
// The flight controller section lives on the PX4 tab. Anywhere else there is nothing on screen to keep
// up to date - except right after pressing one of its buttons, which refreshes whatever tab you are on.
async function refreshConnection(force = false) {
  clearTimeout(connTimer);
  if (!force && !$('#tab-px4').classList.contains('active')) return;
  let c;
  try { c = await api('/api/connection'); } catch (e) { $('#conn-error').textContent = e.message; return; }
  const L = c.link, where = modeShown(status);
  $$('#conn-modes button').forEach(b => b.classList.toggle('active', b.dataset.link === where));
  paintModeSwitch(status);        // the switch has one painter; this is it
  if (!c.mode) $('#conn-current').innerHTML = '';
  else if (c.mode === 'sitl') $('#conn-current').innerHTML =
    `<span class="hint">instance ${c.px4_instance ?? 0} · tcp ${esc(L.address)} · ${esc(L.ctl_address)}</span>`;
  else $('#conn-current').innerHTML =
    `<span class="hint"><span class="dev">${esc(c.serial)}</span> @ ${c.baud} · QGroundControl over UDP ${esc(c.qgc)}</span>`;
  $('#conn-head').innerHTML = c.mode
    ? `${esc(LINK_LABEL[where] || '')} <span class="${L.connected ? 'ok' : 'muted'}">${L.connected ? '● connected'
        : (c.mode === 'sitl' && c.px4_running ? '○ starting…' : '○ waiting')}</span>`
    : '<span class="muted">not connected</span>';
  $('#conn-error').textContent = c.error || '';
  showTelemetry(L);
  // the box shows the speed the link is really running at, so it cannot quietly say 921600 at a radio
  if (c.mode === 'hitl' && c.baud && !$('#conn-baud').dataset.touched && document.activeElement !== $('#conn-baud')) {
    $('#conn-baud').value = c.baud;
  }
  lastPorts = c.ports || [];
  $('#conn-ports').innerHTML = lastPorts.map(p =>
    `<div class="conn-row"><div><b>${p.likely_px4 ? 'Pixhawk' : 'Telemetry radio'}</b> <span class="hint">${esc(p.description || '')}</span> <span class="dev">${esc(p.device)}</span>${p.likely_px4 ? '' : ' <span class="hint">· opens at 57600; FLIGHT finds this by itself</span>'}</div>
      <button class="pill small ${p.likely_px4 ? 'primary' : ''}" data-dev="${esc(p.device)}">${c.mode === 'hitl' && c.serial === p.device ? 'Reconnect' : 'Connect'}</button></div>`).join('');
  $$('#conn-ports button[data-dev]').forEach(b => b.addEventListener('click', () => connectHitl(b.dataset.dev)));
  // The one interlock worth keeping from the old checklist: the firmware will not run the motors on the
  // real aircraft until this is set, and nothing else on this page can set it.
  $('#conn-hw-ok').hidden = !(where === 'real' && nativeModuleActive() && pv('NLF_HW_OK', 1) === 0);
  connTimer = setTimeout(refreshConnection, 2000);
}

// The light is only as fresh as the status behind it: if the app stops answering, that is what it says.
setInterval(() => {
  if (!lastStatusAt || Date.now() - lastStatusAt < 5000) return;
  const el = $('#link-light');
  el.querySelector('i').classList.remove('on');
  el.classList.add('crit');
  el.title = 'no answer from the app (the page lost its connection)';
}, 2000);

// ------------------------------------------------------------ flight mode
// The modes a pilot picks by hand. PX4 reports the one it is in; the rest are one click away.
const FLIGHT_MODES = [['manual', 'Manual'], ['stabilized', 'Stabilized'], ['altitude', 'Altitude'],
  ['position', 'Position'], ['hold', 'Hold'], ['land', 'Land'], ['rtl', 'Return']];
function renderModeButtons() {
  const host = $('#mode-buttons');
  if (!host) return;
  if (!host.children.length) {
    host.innerHTML = FLIGHT_MODES.map(([k, label]) =>
      `<button class="pill small" data-mode="${k}" title="ask PX4 for ${label} mode">${label}</button>`).join('');
    $$('#mode-buttons button').forEach(b => b.addEventListener('click', async () => {
      try { await api('/api/px4/command', { command: 'mode', mode: b.dataset.mode }); }
      catch (e) { logLine('[mode] ' + e.message); }
    }));
  }
  const now = (status.mode_name || '').toLowerCase();
  $$('#mode-buttons button').forEach(b => b.classList.toggle('active', now.startsWith(b.dataset.mode.slice(0, 5))));
  $('#mode-now').textContent = status.connected ? (status.mode_name || '—') + (status.armed ? ' · armed' : '') : 'no link';
  $$('#mode-buttons button').forEach(b => b.disabled = !status.connected);
}

// ============================================================ preset versions (GitHub)
// The version control shows the version this design is at — the newest pushed one, until an older one is
// opened — rather than a prompt. Picking a different one replaces the design, so it is asked for twice.
async function loadVersionList() {
  const sel = $('#af-version');
  if (!sel) return;
  try {
    const r = await api_get('/api/preset/versions');
    const rows = r.versions || [];
    if (!rows.length) { sel.innerHTML = '<option value="">no versions</option>'; currentVersion = ''; return; }
    if (!rows.some(v => v.id === currentVersion)) currentVersion = rows[0].id;
    const when = (t) => { try { return new Date(t).toLocaleString(undefined, { month: 'short', day: 'numeric', hour: '2-digit', minute: '2-digit' }); } catch { return String(t); } };
    sel.innerHTML = rows.map((v, i) =>
      `<option value="${esc(v.id)}" title="${esc(fmtDate(v.time))}${v.note ? ' · ' + esc(v.note) : ''}">v${rows.length - i} · ${esc(when(v.time))}${v.note ? ' · ' + esc(v.note) : ''}</option>`).join('');
    sel.value = currentVersion;
    sel.title = r.archive && r.archive.web ? 'versions of this design on ' + r.archive.web : 'versions of this design';
  } catch (e) { sel.innerHTML = '<option value="">versions unavailable</option>'; }
}
$('#af-version').addEventListener('change', async e => {
  const id = e.target.value;
  if (!id || id === currentVersion) return;
  if (!confirmTwice('load-version', e.target, 'open version ' + (e.target.selectedOptions[0] || {}).text)) {
    e.target.value = currentVersion; return;
  }
  try {
    const r = await api('/api/archive/load_airframe', { id });
    if (r.airframe) { currentVersion = id; setAirframe(r.airframe); logLine('[preset] opened version ' + id); await loadVersionList(); }
  } catch (err) { logLine('[preset] ' + err.message); e.target.value = currentVersion; }
});
// Going back: the version in the box is the one you get, with everything in it - the geometry, the design
// and every parameter the design sets. What you changed since is gone, so it is asked for twice.
$('#af-revert').addEventListener('click', async e => {
  const b = e.currentTarget, sel = $('#af-version'), id = sel.value;
  if (!id) { logLine('[preset] this design has no pushed version to go back to.'); return; }
  const what = (sel.selectedOptions[0] || {}).text || id;
  if (!confirmTwice('revert-version', b, 'go back to ' + what + ', losing everything changed since')) return;
  try {
    const r = await api('/api/archive/load_airframe', { id });
    if (!r.airframe) throw new Error(r.error || 'nothing in that version');
    currentVersion = id;
    setAirframe(r.airframe);
    await loadVersionList();
    logLine('[preset] back to ' + what);
    markSaved('Back to ' + what, 4000);
  } catch (err) { logLine('[preset] revert failed: ' + err.message); markSaved('Revert failed: ' + err.message, 5000); }
});
$('#af-push').addEventListener('click', async e => {
  const b = e.currentTarget, label = b.textContent;
  b.disabled = true; b.textContent = 'Pushing…';
  try {
    const r = await api('/api/preset/push', {});
    if (r.ok) { currentVersion = r.id; logLine('[preset] pushed version ' + r.id); loadVersionList(); }
    else logLine('[preset] push failed: ' + (r.error || 'unknown'));
  } catch (err) { logLine('[preset] push failed: ' + err.message); }
  b.disabled = false; b.textContent = label;
});

// ============================================================ the aircraft's own flight logs
// The board logs everything at two hundred samples a second; a radio carries ten or twenty. So the live
// view is for flying and this is for looking afterwards - and once a log is here it is a run like any
// other, in the same list, on the same graph, scrubbing the same 3D view.
let logsTimer = 0, logsAll = false, logsBusy = null;   // a board keeps hundreds; the newest are the ones you want
const MB = (b) => (b / 1e6).toFixed(1) + ' MB';
const mmss = (s) => s == null ? '—' : s >= 90 ? `${Math.round(s / 60)} min` : `${Math.round(s)} s`;

async function listLogs(refresh = false) {
  const note = $('#logs-note'), table = $('#logs-table');
  note.textContent = 'asking the aircraft…';
  try {
    const r = await api_get('/api/logs' + (refresh ? '?refresh=1' : ''));
    const all = (r.logs || []).slice().reverse();                    // newest first: the flight you just flew
    const logs = logsAll ? all : all.slice(0, 25);
    note.textContent = all.length ? `${all.length} on the card` : 'none on the card';
    $('#logs-rate').textContent = r.measured
      ? `the last one came down at ${(r.assumed_bps / 1000).toFixed(1)} kB/s`
      : `estimated at ${(r.assumed_bps / 1000).toFixed(1)} kB/s — the first download measures it`;
    table.innerHTML = !logs.length ? '<p class="hint">The card has no logs on it.</p>'
      : `<table class="grid"><thead><tr><th>When</th><th>Size</th><th>Over this link</th><th></th></tr></thead><tbody>`
      + logs.map(l => `<tr data-row="${l.id}"><td>${esc((l.label || '').split(' · ')[0])}</td>
          <td class="num">${MB(l.size)}</td><td class="num">${mmss(l.eta_s)}</td>
          <td class="dl"><button class="pill small" data-log="${l.id}">Download</button></td></tr>`).join('')
      + '</tbody></table>'
      + (all.length > logs.length ? `<p class="hint">showing the newest ${logs.length} of ${all.length} ·
           <a href="#" id="logs-all">show them all</a></p>` : '');
    table.querySelectorAll('button[data-log]').forEach(b => b.addEventListener('click', () => downloadLog(+b.dataset.log)));
    const more = $('#logs-all');
    if (more) more.addEventListener('click', (ev) => { ev.preventDefault(); logsAll = true; listLogs(); });
    if (r.job && r.job.state === 'running') { logsBusy = r.job.log_id; pollLogJob(); }
  } catch (e) {
    note.textContent = e.message;
    table.innerHTML = '';
  }
}
// The progress belongs on the line of the log it is about: with four hundred of them, a bar somewhere
// underneath says nothing about which one is coming down.
function showLogProgress(j) {
  const row = j && $(`#logs-table tr[data-row="${j.log_id}"]`);
  $$('#logs-table tr[data-row] td.dl').forEach(td => {
    if (row && td.parentElement === row) return;
    if (td.dataset.was) { td.innerHTML = td.dataset.was; delete td.dataset.was; }
    td.querySelectorAll('button[data-log]').forEach(b => b.disabled = !!(j && j.state === 'running'));
  });
  if (!row) return;
  const td = row.querySelector('td.dl');
  if (td.dataset.was === undefined) td.dataset.was = td.innerHTML;
  const rate = j.bps ? ` · ${(j.bps / 1000).toFixed(1)} kB/s` : '';
  td.innerHTML = j.state === 'running'
    ? `<div class="logs-bar"><i style="width:${j.percent}%"></i></div>
       <span class="hint">${j.percent}%${rate} · ${mmss(j.eta_s)} left</span>
       <button class="pill small danger" id="logs-stop">Stop</button>`
    : j.state === 'done' ? `<span class="hint ok">in the graph · ${mmss(j.elapsed)}${rate}</span>`
    : `<span class="hint bad">${esc(j.state)}${j.error ? ': ' + esc(j.error) : ''}</span>`;
  const stop = $('#logs-stop');
  if (stop) stop.addEventListener('click', () => api('/api/logs/cancel', {}));
}
async function downloadLog(id) {
  try {
    await api('/api/logs/download', { id });
    logsBusy = id;
    pollLogJob();
  } catch (e) { $('#logs-note').textContent = e.message; }
}
async function pollLogJob() {
  clearTimeout(logsTimer);
  let j = null;
  try { j = (await api_get('/api/logs/job')).job; } catch { }
  showLogProgress(j);
  if (!j || j.state === 'running') { logsTimer = setTimeout(pollLogJob, 700); return; }
  logsBusy = null;
  if (j.state === 'done') {
    listLogs();                           // the rate is measured now: every estimate below re-bases on it
    showGraph(true);
    await loadRunList();
    if (j.run_id) selectRun(j.run_id);
  }
}
// Bench test: the land detector is what tells the estimator to hold the aircraft still, so a hand-carried
// aircraft cannot show what its flow sensor sees until the detector is out of the way.
async function benchSet(on) {
  $('#bench-note').textContent = on ? 'stopping the land detector…' : 'starting the land detector…';
  try {
    const r = await fetch('/api/px4/bench', { method: 'POST', headers: { 'content-type': 'application/json' },
                                              body: JSON.stringify({ on }) }).then(x => x.json());
    if (!r.ok) throw new Error(r.error || 'the aircraft did not answer');
    const off = r.land_detector === 'stopped';
    $('#bench-state').textContent = off ? 'land detector stopped — put it back before flying' : '';
    $('#bench-state').classList.toggle('warn', off);
    $('#bench-note').textContent = off ? 'move it by hand now: the position follows the H-flow'
                                       : 'normal: the estimator holds it still while it sits on the ground';
  } catch (e) {
    $('#bench-note').textContent = e.message;
  }
}
$('#bench-on').addEventListener('click', () => benchSet(true));
$('#bench-off').addEventListener('click', () => benchSet(false));
$('#logs-refresh').addEventListener('click', () => listLogs(true));
$('#logs-import').addEventListener('click', () => $('#logs-file').click());
$('#logs-file').addEventListener('change', async (e) => {
  const f = e.target.files[0];
  if (!f) return;
  $('#logs-note').textContent = `reading ${f.name}…`;
  const body = new FormData();
  body.append('file', f);
  try {
    const r = await fetch('/api/logs/import', { method: 'POST', body }).then(x => x.json());
    if (!r.ok) throw new Error(r.error || 'could not read it');
    $('#logs-note').textContent = `${f.name}: ${r.run.duration.toFixed(0)} s, ${r.run.samples} samples`;
    showGraph(true);                      // the point of opening a log is to look at it
    await loadRunList();
    selectRun(r.run.id);
  } catch (err) { $('#logs-note').textContent = err.message; }
  e.target.value = '';
});
$('#sec-logs').addEventListener('toggle', (e) => { if (e.target.open && !$('#logs-table').innerHTML) listLogs(); });

// ============================================================ firmware: the custom PX4 modules
let fwLoaded = null, fwDirty = false;
// Your modules, as the build sees them: a directory whose CMakeLists.txt adds a PX4 module. Pick one and
// its files are in the editor below. The groups that are not modules — the allocator overlay, the uORB
// messages — are in the same list, because they are yours to edit too.
let fwGroups = [], fwGroup = '';
async function loadFirmwareModules(force = false) {
  if (fwGroups.length && !force) return;
  try { fwGroups = (await api_get('/api/firmware/modules')).groups || []; }
  catch (e) { $('#fw-note').textContent = e.message; return; }
  const sel = $('#fw-module');
  const mods = fwGroups.filter(g => g.kind === 'module');
  const rest = fwGroups.filter(g => g.kind !== 'module');
  const opt = (g) => `<option value="${esc(g.dir)}">${esc(g.name)}${g.kind === 'module' && g.enabled === false ? ' (off)' : ''} — ${g.files.length} file${g.files.length === 1 ? '' : 's'}${g.kind === 'module' && g.user_command ? ', answers MAV_CMD_USER_1' : ''}</option>`;
  sel.innerHTML = (mods.length ? `<optgroup label="Modules">${mods.map(opt).join('')}</optgroup>` : '')
    + (rest.length ? `<optgroup label="Other firmware sources">${rest.map(opt).join('')}</optgroup>` : '');
  if (!fwGroups.some(g => g.dir === fwGroup)) fwGroup = (mods[0] || fwGroups[0] || {}).dir || '';
  sel.value = fwGroup;
  await selectFirmwareGroup(fwGroup, true);
}
async function selectFirmwareGroup(dir, keepFile = false) {
  const g = fwGroups.find(x => x.dir === dir);
  if (!g) return;
  const was = $('#fw-file').value;
  fwGroup = dir;
  $('#fw-module').value = dir;
  // A module is either part of this aircraft or it is not, and the Parameters list follows that.
  $('#fw-use-wrap').hidden = g.kind !== 'module';
  $('#fw-use').checked = g.enabled !== false;
  $('#fw-where').textContent = g.kind === 'module'
    ? `${g.module}${g.params.length ? ` · ${g.params.length} parameters` : ''}${g.enabled === false ? ' · not in this design' : ''}`
    : 'sources';
  const sel = $('#fw-file');
  sel.innerHTML = g.files.map(f => `<option value="${esc(f)}">${esc(f.startsWith(g.dir + '/') ? f.slice(g.dir.length + 1) : f)}</option>`).join('')
    || '<option value="">no files</option>';
  const want = keepFile && g.files.includes(was) ? was : (g.files[0] || '');
  if (want) { sel.value = want; await openFirmwareFile(want); }
  else { $('#fw-text').value = ''; }
}
$('#fw-module').addEventListener('change', (e) => selectFirmwareGroup(e.target.value));
$('#fw-use').addEventListener('change', async (e) => {
  const g = fwGroups.find(x => x.dir === fwGroup);
  if (!g) return;
  try {
    await api('/api/firmware/modules/enable', { dir: fwGroup, on: e.target.checked });
    logLine(`[firmware] ${g.name} is ${e.target.checked ? 'in' : 'not in'} this design`);
    await loadFirmwareModules(true);          // the list says (off) and the editor keeps its file
    await selectFirmwareGroup(fwGroup, true);
    loadExport();                             // its parameters appear or disappear with it
    markDirty();
  } catch (err) { $('#fw-note').textContent = err.message; e.target.checked = !e.target.checked; }
});
$('#fw-new').addEventListener('click', () => {
  const box = $('#fw-module'), btn = $('#fw-new');
  const input = document.createElement('input');
  input.type = 'text'; input.placeholder = 'module name, e.g. atlas_hover_guard'; input.className = 'name'; input.style.flex = '1';
  box.replaceWith(input); input.focus();
  const done = async (commit) => {
    input.replaceWith(box);
    if (!commit || !input.value.trim()) return;
    btn.disabled = true;
    try {
      const r = await api('/api/firmware/module/new', { name: input.value.trim() });
      logLine(`[firmware] new module ${r.name} in ${r.dir}`);
      fwGroup = r.dir;
      await loadFirmwareModules(true);
    } catch (err) { logLine('[firmware] ' + err.message); }
    btn.disabled = false;
  };
  input.onblur = () => done(true);
  input.onkeydown = (ev) => { if (ev.key === 'Enter') done(true); if (ev.key === 'Escape') done(false); };
});
async function refreshFirmware() {
  await loadFirmwareModules();
  const fw = status.firmware || {};
  const stale = !!(fw.needs_rebuild || fw.needs_relaunch || fw.needs_flash);
  $('#fw-state').classList.toggle('on', !stale);
  $('#fw-state-text').textContent = !fw.custom ? 'stock PX4'
    : stale ? 'sources changed since the last build' : (fw.board ? 'board up to date' : 'SITL up to date');
}
async function openFirmwareFile(path) {
  try {
    const r = await api_get('/api/firmware/file?path=' + encodeURIComponent(path));
    $('#fw-text').value = r.text || '';
    fwLoaded = r.text || ''; fwDirty = false;
    $('#fw-save').disabled = true;
  } catch (e) { $('#fw-text').value = ''; $('#fw-note').textContent = e.message; }
}
$('#fw-file').addEventListener('change', e => openFirmwareFile(e.target.value));
$('#fw-text').addEventListener('input', () => {
  fwDirty = $('#fw-text').value !== fwLoaded;
  $('#fw-save').disabled = !fwDirty;
});
$('#fw-revert').addEventListener('click', () => openFirmwareFile($('#fw-file').value));
$('#fw-code').addEventListener('click', async (e) => {
  const path = $('#fw-file').value;
  if (!path) return;
  e.currentTarget.disabled = true;
  try {
    await api('/api/firmware/open', { path });
    logLine('[firmware] opened ' + path + ' in VS Code');
  } catch (err) { logLine('[firmware] ' + err.message); }
  e.currentTarget.disabled = false;
});
$('#fw-save').addEventListener('click', async e => {
  const b = e.currentTarget;
  b.disabled = true;
  try {
    await api('/api/firmware/file', { path: $('#fw-file').value, text: $('#fw-text').value });
    fwLoaded = $('#fw-text').value; fwDirty = false;
    logLine('[firmware] saved ' + $('#fw-file').value + '. Build to compile it.');
    refreshFirmware();
  } catch (err) { logLine('[firmware] ' + err.message); b.disabled = false; }
});
$('#fw-build').addEventListener('click', async e => {
  const b = e.currentTarget, label = b.textContent;
  b.disabled = true; b.textContent = 'Building…';
  try { const r = await api('/api/firmware/build', { atlas: true }); logLine('[firmware] ' + (r.ok ? 'built' : 'build failed: ' + (r.error || ''))); }
  catch (err) { logLine('[firmware] ' + err.message); }
  b.disabled = false; b.textContent = label; refreshFirmware();
});
$('#fw-flash').addEventListener('click', async e => {
  const b = e.currentTarget, label = b.textContent;
  if (!confirmTwice('fw-flash', b, 'flash the board')) return;
  b.disabled = true; b.textContent = 'Flashing…';
  try { const r = await api('/api/px4/push', { save: true, firmware: true }); logLine('[firmware] ' + JSON.stringify(r.firmware || r)); }
  catch (err) { logLine('[firmware] ' + err.message); }
  b.disabled = false; b.textContent = label; refreshFirmware();
});

// ============================================================ Remote control (PX4 tab): indicator + one row per channel
let lastRc = {}, lastManual = {}, rcLastRender = 0, rcIndKey = '', rcBuiltFor = '';
let rcCal = null;   // stick calibration in progress: { step, min: [], max: [] }
// what a channel can control (PX4's RC_MAP_* functions); 'switch' ones have a threshold, 'mode' has six slots
const RC_FUNCS = [
  { p: 'RC_MAP_ROLL', label: 'Roll', kind: 'stick' }, { p: 'RC_MAP_PITCH', label: 'Pitch', kind: 'stick' }, { p: 'RC_MAP_THROTTLE', label: 'Throttle', kind: 'stick' }, { p: 'RC_MAP_YAW', label: 'Yaw', kind: 'stick' },
  { p: 'RC_MAP_FLTMODE', label: 'Flight mode', kind: 'mode' },
  { p: 'NLF_RC_CH', label: 'Nose-lift takeoff (button)', kind: 'button' }, { p: 'NLF_RC_LAND', label: 'Nose-lift land (button)', kind: 'button' },
  { p: 'RC_MAP_ARM_SW', label: 'Arm', kind: 'switch', th: 'RC_ARMSWITCH_TH' }, { p: 'RC_MAP_KILL_SW', label: 'Kill', kind: 'switch', th: 'RC_KILLSWITCH_TH' },
  { p: 'RC_MAP_RETURN_SW', label: 'Return home', kind: 'switch', th: 'RC_RETURN_TH' }, { p: 'RC_MAP_LOITER_SW', label: 'Hold', kind: 'switch', th: 'RC_LOITER_TH' },
  { p: 'RC_MAP_OFFB_SW', label: 'Offboard', kind: 'switch', th: 'RC_OFFB_TH' }, { p: 'RC_MAP_TRANS_SW', label: 'VTOL transition', kind: 'switch', th: 'RC_TRANS_TH' },
  { p: 'RC_MAP_GEAR_SW', label: 'Landing gear', kind: 'switch', th: 'RC_GEAR_TH' }, { p: 'RC_MAP_ENG_MOT', label: 'Engine', kind: 'switch', th: 'RC_ENG_MOT_TH' },
  { p: 'RC_MAP_PAY_SW', label: 'Payload', kind: 'switch', th: 'RC_PAYLOAD_TH' }, { p: 'RC_MAP_TERM_SW', label: 'Flight termination', kind: 'switch' },
  { p: 'RC_MAP_FAILSAFE', label: 'Failsafe', kind: 'aux' }, { p: 'RC_MAP_FLAPS', label: 'Flaps', kind: 'aux' },
  { p: 'RC_MAP_AUX1', label: 'Aux 1', kind: 'aux' }, { p: 'RC_MAP_AUX2', label: 'Aux 2', kind: 'aux' }, { p: 'RC_MAP_AUX3', label: 'Aux 3', kind: 'aux' },
  { p: 'RC_MAP_AUX4', label: 'Aux 4', kind: 'aux' }, { p: 'RC_MAP_AUX5', label: 'Aux 5', kind: 'aux' }, { p: 'RC_MAP_AUX6', label: 'Aux 6', kind: 'aux' },
  { p: 'RC_MAP_PARAM1', label: 'Tuning 1', kind: 'aux' }, { p: 'RC_MAP_PARAM2', label: 'Tuning 2', kind: 'aux' }, { p: 'RC_MAP_PARAM3', label: 'Tuning 3', kind: 'aux' },
];
const MODE_SLOTS = { 0: [1, 2], 50: [3, 4], 100: [5, 6] };   // a 3-position switch lands in slots 1, 4 and 6; keep each third consistent
const rcFresh = () => lastRc && lastRc.count > 0 && lastRc.t && (Date.now() / 1000 - lastRc.t) < 3;
const rcAlive = () => rcFresh() && (lastRc.channels || []).some(v => v > 0);      // a bound receiver with the transmitter on
// The remote's channels, from whichever path is live: the receiver on the flight controller (RC_CHANNELS), or a
// remote on this computer (USB/serial/gamepad). Both are presented as PWM, so one mapping table serves both and the
// same transmitter keeps its channel assignments whether it talks to the board or to the app.
function rcSource() {
  if (rcAlive()) return { channels: lastRc.channels || [], count: lastRc.count, via: 'board' };
  const src = joyCurrent();
  if (src) return { channels: src.axes.map(v => Math.round(1500 + Math.max(-1, Math.min(1, v)) * 500)), count: src.axes.length, via: 'app', name: src.name };
  return null;
}
const pv = (name, d = null) => (params[name] && params[name].value != null) ? +params[name].value : d;
function rcNorm(ch, v) {   // PWM -> -1..1 with the board's calibration (RCn_MIN/TRIM/MAX), like PX4's rc_update
  const mn = pv(`RC${ch}_MIN`, 1000), mx = pv(`RC${ch}_MAX`, 2000), tr = pv(`RC${ch}_TRIM`, 1500), rev = pv(`RC${ch}_REV`, 1);
  const x = v > tr ? (v - tr) / Math.max(1, mx - tr) : (v - tr) / Math.max(1, tr - mn);
  return Math.max(-1, Math.min(1, x * (rev < 0 ? -1 : 1)));
}
const rcThird = (x) => x < -0.33 ? 0 : x > 0.33 ? 100 : 50;
function renderRcIndicator() {
  const src = joyCurrent();
  const viaBoard = rcAlive(), viaApp = !!src;
  let text = 'not detected', on = false;
  if (viaApp && viaBoard) { text = `${src.kind === 'serial' ? 'USB serial' : src.kind === 'hid' ? 'USB' : 'gamepad'} + Pixhawk`; on = true; }
  else if (viaApp) { text = src.kind === 'serial' ? 'USB serial' : src.kind === 'hid' ? 'USB' : 'gamepad'; on = true; }
  else if (viaBoard) { text = 'via Pixhawk'; on = true; }
  else if (rcFresh()) { text = 'receiver bound · transmitter off'; }
  const key = text + on;
  if (key !== rcIndKey) { rcIndKey = key; const el = $('#rc-ind'); el.classList.toggle('on', on); $('#rc-ind-text').textContent = text; }

}
// functions this firmware has; with no (or a partial) parameter list, offer them all rather than nothing
const rcKnownFuncs = () => params.RC_MAP_ROLL ? RC_FUNCS.filter(f => params[f.p]) : RC_FUNCS;
const rcFuncOf = (ch) => rcKnownFuncs().find(f => pv(f.p, 0) === ch) || null;
async function rcSet(name, value) {
  try { await api('/api/params/set', { name, value }); params[name] = { ...(params[name] || {}), value }; } catch (e) { logLine('[rc] ' + e.message); }
}
async function rcAssign(ch, p) {        // give channel ch the function p (moving it away from wherever it was), or none
  const had = rcFuncOf(ch);
  if (had && had.p !== p) await rcSet(had.p, 0);
  if (p) await rcSet(p, ch);
  rcBuiltFor = '';
}
function switchOnAt(f, pos) {          // does this switch function count as ON at the 0 / 50 / 100 % stick position?
  const th = pv(f.th, 0.5), x = pos === 0 ? -1 : pos === 100 ? 1 : 0;
  return th >= 0 ? x > th : x < th;
}
function renderRcBoard() {
  const el = $('#rc-board');
  const src = rcSource();
  el.hidden = false;                 // mapping is done here, with or without a transmitter switched on
  const n = 18;   // PX4's RC input carries 18 channels at most; rows beyond what the receiver sends stay empty
  const funcs = rcKnownFuncs();
  const names = (airframe && airframe.design && airframe.design.rc_names) || {};
  const key = `${n}|${funcs.map(f => f.p + ':' + pv(f.p, 0)).join(',')}|${[1, 2, 3, 4, 5, 6].map(i => pv('COM_FLTMODE' + i, -1)).join(',')}|${JSON.stringify(names)}`;
  if (el.dataset.mode !== 'rc' || rcBuiltFor !== key) {
    el.dataset.mode = 'rc'; rcBuiltFor = key;
    const modeVals = ((meta.COM_FLTMODE1 || {}).values || [{ value: -1, description: 'Unassigned' }, { value: 0, description: 'Manual' }, { value: 1, description: 'Altitude' }, { value: 2, description: 'Position' }, { value: 3, description: 'Mission' }, { value: 4, description: 'Hold' }, { value: 5, description: 'Return' }, { value: 6, description: 'Acro' }, { value: 7, description: 'Offboard' }, { value: 8, description: 'Stabilized' }, { value: 10, description: 'Takeoff' }, { value: 11, description: 'Land' }]);
    const modeSel = (pos) => `<select class="rc-mode" data-pos="${pos}">${modeVals.map(v => `<option value="${v.value}" ${pv('COM_FLTMODE' + MODE_SLOTS[pos][1], -1) === v.value ? 'selected' : ''}>${esc(v.description.replace(' / Cruise', ''))}</option>`).join('')}</select>`;
    const rows = Array.from({ length: n }, (_, i) => {
      const ch = i + 1, f = rcFuncOf(ch);
      const fnSel = `<select class="rc-fn-sel" data-ch="${ch}"><option value="">–</option>${funcs.map(x => `<option value="${x.p}" ${f && f.p === x.p ? 'selected' : ''}>${x.label}</option>`).join('')}</select>`;
      let cells = '<span class="rc-c"></span><span class="rc-c"></span><span class="rc-c"></span>';
      if (f && f.kind === 'mode') cells = [0, 50, 100].map(pos => `<span class="rc-c" data-pos="${pos}">${modeSel(pos)}</span>`).join('');
      else if (f && f.kind === 'switch') cells = [0, 50, 100].map(pos => `<span class="rc-c" data-pos="${pos}">${f.th ? `<button class="rc-sw ${switchOnAt(f, pos) ? 'on' : ''}" data-ch="${ch}" data-pos="${pos}" title="click: ${f.label} is ON at this position">${switchOnAt(f, pos) ? 'ON' : 'off'}</button>` : `<span class="rc-dim">${pos === 100 ? 'ON' : 'off'}</span>`}</span>`).join('');
      else if (f && f.kind === 'stick') cells = `<span class="rc-c rc-dim">${f.p === 'RC_MAP_THROTTLE' ? 'idle' : 'full −'}</span><span class="rc-c rc-dim">centre</span><span class="rc-c rc-dim">${f.p === 'RC_MAP_THROTTLE' ? 'full' : 'full +'}</span>`;
      else if (f && f.kind === 'button') cells = `<span class="rc-c rc-dim" data-pos="0">released</span><span class="rc-c rc-dim" data-pos="50">–</span><span class="rc-c" data-pos="100"><span class="rc-tag on">${f.p === 'NLF_RC_CH' ? 'take off' : 'land'}</span></span>`;
      return `<div class="rc-line" data-ch="${ch}"><span class="rc-n">${ch}</span><input class="rc-name" data-ch="${ch}" value="${esc(names[ch] || '')}" placeholder="name" maxlength="18" title="your own name for this channel (saved with the airframe)"><span class="rc-bar"><i data-ch="${ch}" style="width:50%"></i></span><span class="rc-val" data-ch="${ch}">–</span><span class="rc-fsel">${fnSel}</span>${cells}</div>`;
    }).join('');
    el.innerHTML = `<div class="rc-table"><div class="rc-line rc-head"><span></span><span>Name</span><span></span><span></span><span>Controls</span><span>0 %</span><span>50 %</span><span>100 %</span></div>${rows}</div>
      <div id="rc-cal" class="rc-cal"></div><div class="rc-foot hint" id="rc-foot"></div>`;
    $$('#rc-board .rc-fn-sel').forEach(sel => sel.addEventListener('change', () => rcAssign(+sel.dataset.ch, sel.value || null)));
    $$('#rc-board .rc-name').forEach(inp => inp.addEventListener('change', () => { airframe.design = airframe.design || {}; const nm = airframe.design.rc_names = airframe.design.rc_names || {}; const v = inp.value.trim(); if (v) nm[inp.dataset.ch] = v; else delete nm[inp.dataset.ch]; rcBuiltFor = ''; pushAirframe(true); }));
    $$('#rc-board .rc-mode').forEach(sel => sel.addEventListener('change', async () => { const v = +sel.value; for (const slot of MODE_SLOTS[+sel.dataset.pos]) await rcSet('COM_FLTMODE' + slot, v); rcBuiltFor = ''; }));
    $$('#rc-board .rc-sw').forEach(b => b.addEventListener('click', async () => {
      const f = rcFuncOf(+b.dataset.ch); if (!f || !f.th) return;
      const pos = +b.dataset.pos;                                     // ON at 100 % -> threshold +0.5, ON at 0 % -> -0.5; 50 % cannot be isolated on a two-state switch
      if (pos === 50) return;
      await rcSet(f.th, pos === 100 ? 0.5 : -0.5); rcBuiltFor = '';
    }));
  }
  const ch = (src && src.channels) || [];
  ch.forEach((v, i) => {
    const bar = el.querySelector(`.rc-bar i[data-ch="${i + 1}"]`), val = el.querySelector(`.rc-val[data-ch="${i + 1}"]`), line = el.querySelector(`.rc-line[data-ch="${i + 1}"]`);
    if (bar) bar.style.width = (Math.max(0, Math.min(1, (v - 1000) / 1000)) * 100).toFixed(0) + '%';
    if (val) val.textContent = v ? v : '–';
    if (line) { const third = v ? rcThird(rcNorm(i + 1, v)) : null; line.querySelectorAll('.rc-c').forEach(c => c.classList.toggle('active', third != null && +c.dataset.pos === third)); }
  });
  // stick calibration flow
  const cal = $('#rc-cal');
  if (cal) {
    if (rcCal) {
      ch.forEach((v, i) => { if (v) { rcCal.min[i] = Math.min(rcCal.min[i] ?? v, v); rcCal.max[i] = Math.max(rcCal.max[i] ?? v, v); } });
      const seen = rcCal.min.filter((m, i) => m != null && rcCal.max[i] - m > 400).length;
      if (rcCal.step === 1 && cal.dataset.step !== '1') { cal.dataset.step = '1'; cal.innerHTML = `<b>Calibrating · 1/2</b> Move every stick and switch to both ends. <span id="rc-cal-n"></span> <button class="pill small primary" id="rc-cal-next">Next</button> <button class="pill small" id="rc-cal-stop">Cancel</button>`; $('#rc-cal-next').onclick = () => { rcCal.step = 2; }; $('#rc-cal-stop').onclick = () => { rcCal = null; cal.dataset.step = ''; cal.innerHTML = ''; }; }
      if (rcCal.step === 1) { const nEl = $('#rc-cal-n'); if (nEl) nEl.textContent = `${seen} channels done.`; }
      if (rcCal.step === 2 && cal.dataset.step !== '2') { cal.dataset.step = '2'; cal.innerHTML = `<b>Calibrating · 2/2</b> Centre roll, pitch and yaw, throttle down, switches off. <button class="pill small primary" id="rc-cal-done">Finish</button> <button class="pill small" id="rc-cal-stop">Cancel</button>`;
        $('#rc-cal-stop').onclick = () => { rcCal = null; cal.dataset.step = ''; cal.innerHTML = ''; };
        $('#rc-cal-done').onclick = async () => {
          const out = {}; let cnt = 0;
          const thr = pv('RC_MAP_THROTTLE', 0);
          ch.forEach((v, i) => { const mn = rcCal.min[i], mx = rcCal.max[i]; if (mn == null || mx == null || mx - mn < 400) return; cnt = i + 1; out[`RC${i + 1}_MIN`] = mn; out[`RC${i + 1}_MAX`] = mx; out[`RC${i + 1}_TRIM`] = (i + 1 === thr) ? mn : Math.max(mn, Math.min(mx, v)); });
          out.RC_CHAN_CNT = cnt;
          cal.innerHTML = '<span class="muted">Writing…</span>';
          try { const r = await api('/api/params/set_many', { params: out }); Object.entries(out).forEach(([k, v]) => params[k] = { ...(params[k] || {}), value: v }); cal.innerHTML = r.ok ? `<span class="ok">Calibrated ${cnt} channels.</span>` : `<span class="err">${(r.results || []).filter(x => !x.ok).map(x => x.name).join(', ')} failed</span>`; }
          catch (e) { cal.innerHTML = `<span class="err">${esc(e.message)}</span>`; }
          rcCal = null; cal.dataset.step = 'done'; setTimeout(() => { if (cal.dataset.step === 'done') { cal.dataset.step = ''; cal.innerHTML = ''; } }, 6000);
        }; }
    } else if (cal.dataset.step !== '' && cal.dataset.step !== 'done') { cal.dataset.step = ''; cal.innerHTML = ''; }
  }
  const foot = $('#rc-foot'); if (foot) foot.innerHTML = [
    params.RC_MAP_ROLL ? '' : '<span class="warn">Reading the board\'s parameters…</span>',
    src && src.via === 'app' ? `${esc(src.name || 'remote')}: ${src.count} channels through this computer (the app relays the sticks to PX4).`
      : `${(src ? src.count : 0) <= 18 ? (src ? src.count : 0) : '?'} channels from the receiver (PX4 carries at most 18).`,
    pv('RC_CHAN_CNT', 0) > 0 ? `Sticks calibrated.` : '<span class="warn">Sticks not calibrated.</span>',
    `<a href="#" id="rc-cal-start">${pv('RC_CHAN_CNT', 0) > 0 ? 'Recalibrate' : 'Calibrate'}</a>`].join(' ');
  const cs = $('#rc-cal-start'); if (cs) cs.onclick = (e) => { e.preventDefault(); rcCal = { step: 1, min: [], max: [] }; };
}
// The remote (indicator, channel table, mapping) is on the Controller tab, beside the link it comes in on.
const rcVisible = () => $('#tab-live').classList.contains('active');
function renderRcLive() {
  const now = performance.now();
  if (now - rcLastRender < 80) return;
  rcLastRender = now;
  renderRcIndicator(); renderRcBoard();
}
setInterval(() => { if (rcVisible()) renderRcLive(); }, 500);

// ============================================================ USB remote (WebHID picker, Gamepad API fallback) -> MANUAL_CONTROL
// current input source: { name, axes: [-1..1], buttons: [bool], kind: 'hid' | 'gamepad' }
let joySrc = null;
let hidDevice = null;

// --- WebHID: parse the device's own report descriptor so any joystick layout works (8-bit, 11-bit, 16-bit axes…)
const HID_AXIS_USAGES = { 0x30: 'X', 0x31: 'Y', 0x32: 'Z', 0x33: 'Rx', 0x34: 'Ry', 0x35: 'Rz', 0x36: 'Slider', 0x37: 'Dial', 0x38: 'Wheel' };
function hidBuildLayout(device) {
  const reports = {};   // reportId -> { axes: [{bit,size,min,max,name}], buttons: [{bit}] }
  for (const col of device.collections) {
    for (const rep of col.inputReports) {
      const lay = reports[rep.reportId] || (reports[rep.reportId] = { axes: [], buttons: [] });
      let bit = 0;
      for (const it of rep.items) {
        for (let k = 0; k < it.reportCount; k++) {
          const usage = it.isRange ? (it.usageMinimum + k) : (it.usages[k] ?? it.usages[0] ?? 0);
          const page = usage >>> 16, id = usage & 0xffff;
          if (page === 0x01 && HID_AXIS_USAGES[id] && it.reportSize > 1) {
            lay.axes.push({ bit, size: it.reportSize, min: it.logicalMinimum, max: it.logicalMaximum, name: HID_AXIS_USAGES[id] });
          } else if (page === 0x09 && it.reportSize === 1) {
            lay.buttons.push({ bit });
          }
          bit += it.reportSize;
        }
      }
    }
  }
  return reports;
}
function hidBits(dv, bit, size, signed) {
  let v = 0;
  for (let i = 0; i < size; i++) { const b = bit + i; if ((dv.getUint8(b >> 3) >> (b & 7)) & 1) v |= (1 << i); }
  if (signed && (v & (1 << (size - 1)))) v -= (1 << size);
  return v;
}
async function hidUse(device) {
  try {
    if (!device.opened) await device.open();
  } catch (e) { logLine('[ui] could not open the radio: ' + e.message); return; }
  const layout = hidBuildLayout(device);
  hidDevice = device;
  joySrc = { name: device.productName || 'HID joystick', axes: [], buttons: [], kind: 'hid' };
  device.addEventListener('inputreport', (e) => {
    const lay = layout[e.reportId] || layout[0]; if (!lay) return;
    const dv = e.data;
    try {
      joySrc.axes = lay.axes.map(a => { const signed = a.min < 0; const v = hidBits(dv, a.bit, a.size, signed); return Math.max(-1, Math.min(1, ((v - a.min) / (a.max - a.min)) * 2 - 1)); });
      joySrc.buttons = lay.buttons.map(b => !!hidBits(dv, b.bit, 1, false));
    } catch { }
  });
  const n = Object.values(layout).reduce((a, l) => a + l.axes.length, 0);
  logLine(`[ui] radio connected: ${joySrc.name} (${n} axes)`);
  joyRenderMap();
}
async function joyConnect() {
  if (!navigator.hid) {
    logLine('[ui] this browser has no device picker (WebHID). Open the app in Chrome or Edge, or move a stick so the gamepad fallback finds the radio.');
    return;
  }
  try {
    const devices = await navigator.hid.requestDevice({ filters: [] });
    if (devices.length) await hidUse(devices[0]);
  } catch (e) { logLine('[ui] radio picker: ' + e.message); }
}
async function hidReconnect() {   // devices the user already granted come back without the picker
  if (!navigator.hid) return;
  try {
    const devs = await navigator.hid.getDevices();
    const joy = devs.find(d => d.collections.some(c => c.usagePage === 1 && (c.usage === 4 || c.usage === 5))) || devs[0];
    if (joy) await hidUse(joy);
  } catch { }
}
function joyGamepad() {
  const pads = navigator.getGamepads ? Array.from(navigator.getGamepads()).filter(Boolean) : [];
  const p = pads[0];
  if (!p) return null;
  return { name: p.id.replace(/\s*\(.*$/, '').slice(0, 48), axes: Array.from(p.axes), buttons: p.buttons.map(b => b.pressed), kind: 'gamepad' };
}
// --- Web Serial: RadioMaster T8L in config (VCP) mode. Protocol taken from RadioMaster's own web configurator:
// 460800 baud, poll with A5 55 1B 0D 0A every 20 ms, the radio answers frames  EE len <6 bytes header> ch1..ch10 (u16 LE) crc8
// where crc8 (CRSF polynomial 0xD5) covers bytes [2 .. len] and channels are 988..2012 with 1500 centred.
let serialPort = null, serialReader = null, serialWriter = null, serialTimer = null;
const T8L_VID = 0x19F5;
function crc8(bytes) { let c = 0; for (const b of bytes) { c ^= b; for (let i = 0; i < 8; i++) c = (c & 0x80) ? ((c << 1) ^ 0xD5) & 0xFF : (c << 1) & 0xFF; } return c; }
async function serialConnect() {
  if (!navigator.serial) { logLine('[ui] this browser has no Web Serial (Safari). Open the page in Chrome or Edge.'); return; }
  let port;
  try {
    port = await navigator.serial.requestPort({ filters: [{ usbVendorId: T8L_VID }] });
  } catch (e) {
    if (e.name === 'NotFoundError') { try { port = await navigator.serial.requestPort(); } catch (e2) { return; } } else return;
  }
  await serialUse(port);
}
async function serialUse(port) {
  try { await port.open({ baudRate: 460800 }); } catch (e) { logLine('[ui] could not open the radio port: ' + e.message + ' (is the RadioMaster config page still connected to it?)'); return; }
  await serialDisconnect(false);
  serialPort = port; serialWriter = port.writable.getWriter(); serialReader = port.readable.getReader();
  joySrc = { name: 'RadioMaster T8L (serial)', axes: new Array(10).fill(0), buttons: [], kind: 'serial', frames: 0, t: 0 };
  hidDevice = null;
  const poll = new Uint8Array([0xA5, 0x55, 0x1B, 0x0D, 0x0A]);
  serialTimer = setInterval(() => { if (serialWriter) serialWriter.write(poll).catch(() => { }); }, 20);
  logLine('[ui] radio serial link open at 460800, polling channels');
  joyRenderMap();
  (async () => {
    let buf = [];
    try {
      while (serialReader) {
        const { value, done } = await serialReader.read();
        if (done) break;
        for (const b of value) buf.push(b);
        while (buf.length >= 3) {
          if (buf[0] !== 0xEE) { buf.shift(); continue; }
          const len = buf[1];
          if (len < 3 || len > 80) { buf.shift(); continue; }
          if (buf.length < len + 2) break;
          const frame = buf.slice(0, len + 2);
          if (crc8(frame.slice(2, 2 + len - 1)) === frame[len + 1] && len >= 27) {
            const ch = []; for (let i = 0; i < 10; i++) ch.push(frame[8 + 2 * i] | (frame[9 + 2 * i] << 8));
            joySrc.axes = ch.map(v => Math.max(-1, Math.min(1, (v - 1500) / 512)));
            joySrc.frames++; joySrc.t = performance.now();
          }
          buf.splice(0, len + 2);
        }
        if (buf.length > 4096) buf = [];
      }
    } catch (e) { logLine('[ui] radio serial read ended: ' + e.message); }
    await serialDisconnect(true);
  })();
}
async function serialDisconnect(announce) {
  if (serialTimer) { clearInterval(serialTimer); serialTimer = null; }
  const r = serialReader, w = serialWriter, p = serialPort;
  serialReader = null; serialWriter = null; serialPort = null;
  try { if (r) { await r.cancel(); r.releaseLock(); } } catch { }
  try { if (w) { w.releaseLock(); } } catch { }
  try { if (p) await p.close(); } catch { }
  if (joySrc && joySrc.kind === 'serial') joySrc = null;
  if (announce && p) { logLine('[ui] radio serial link closed'); joyRenderMap(); }
}
async function serialReconnect() {   // a port granted earlier comes back without the picker
  if (!navigator.serial) return;
  try {
    const ports = await navigator.serial.getPorts();
    const p = ports.find(x => (x.getInfo().usbVendorId === T8L_VID));
    if (p) await serialUse(p);
  } catch { }
}
if (navigator.serial) navigator.serial.addEventListener('disconnect', (e) => { if (e.target === serialPort) serialDisconnect(true); });

function joyCurrent() {
  if (joySrc && joySrc.kind === 'serial' && serialPort) return (performance.now() - joySrc.t < 1000 || joySrc.frames === 0) ? joySrc : { ...joySrc, name: joySrc.name + ' · no data' };
  return null;
}
function joyRenderMap() { rcBuiltFor = ''; }   // the USB remote shares the channel table below; nothing separate to draw
// The value of whatever is mapped to this function, from the app-side remote, using the board's own channel
// assignment and calibration: the transmitter behaves the same here as it does through the receiver.
function rcFuncValue(pwm, p) {
  const c = pv(p, 0);
  if (!c || c < 1 || c > pwm.length || !pwm[c - 1]) return 0;
  const v = rcNorm(c, pwm[c - 1]);
  return Math.abs(v) < 0.02 ? 0 : v;              // deadband
}
// Throttle is not a centred stick: PX4 scales it from the channel's minimum to its maximum, and calibration puts
// RCn_TRIM at the minimum, so the centred normalisation above would never let it below half.
function rcThrottleValue(pwm) {
  const c = pv('RC_MAP_THROTTLE', 0);
  if (!c || c < 1 || c > pwm.length || !pwm[c - 1]) return 0;
  const mn = pv(`RC${c}_MIN`, 1000), mx = pv(`RC${c}_MAX`, 2000);
  let x = (pwm[c - 1] - mn) / Math.max(1, mx - mn);
  if (pv(`RC${c}_REV`, 1) < 0) x = 1 - x;
  return Math.max(0, Math.min(1, x));
}
// COM_FLTMODEn -> the mode name the server understands
const RC_MODE_NAMES = { 0: 'manual', 1: 'altitude', 2: 'position', 3: 'mission', 4: 'hold', 5: 'rtl', 6: 'acro', 7: 'offboard', 8: 'stabilized' };
const rcEdge = {};        // function parameter -> last state, for rising edges and mode changes
// A remote on this computer arrives with nothing assigned (PX4's RC_MAP_* are 0 until a receiver is set up). Seed the
// usual transmitter order once, so the sticks work immediately; the table then edits the same parameters a receiver
// on the flight controller would use, and the assignment survives the switch between SITL and the board.
let rcSeeded = false;
async function rcSeedDefaults() {
  if (rcSeeded || !params.RC_MAP_ROLL) return;
  rcSeeded = true;
  if (RC_FUNCS.some(f => pv(f.p, 0) > 0)) return;      // the user (or the board) already assigned something
  logLine('[ui] remote: assigning the usual channel order (1 roll, 2 pitch, 3 throttle, 4 yaw). Change it in the table.');
  for (const [p, ch] of [['RC_MAP_ROLL', 1], ['RC_MAP_PITCH', 2], ['RC_MAP_THROTTLE', 3], ['RC_MAP_YAW', 4]]) await rcSet(p, ch);
  rcBuiltFor = '';
}
let joyLastSend = 0;
function joyTick() {
  const src = joyCurrent();
  if (!src) { rcIndKey = ''; }
  else {
    if (!rcSeeded && !rcAlive()) rcSeedDefaults();
    const pwm = src.axes.map(v => Math.round(1500 + Math.max(-1, Math.min(1, v)) * 500));
    const now = performance.now();
    if ((status.conn_mode === 'sitl' || status.hil_enabled) && joyWs && joyWs.readyState === 1 && now - joyLastSend > 20) {   // 50 Hz: SITL, or a board in HIL
      joyLastSend = now;
      const aux = src.axes.slice(4, 10).map(v => +v.toFixed(3));
      joyWs.send(JSON.stringify({
        type: 'manual', roll: rcFuncValue(pwm, 'RC_MAP_ROLL'), pitch: rcFuncValue(pwm, 'RC_MAP_PITCH'),
        throttle: rcThrottleValue(pwm), yaw: rcFuncValue(pwm, 'RC_MAP_YAW'), buttons: 0, aux,
      }));
      // Switches: a receiver on the board drives these itself. Through the app the sticks arrive as MANUAL_CONTROL
      // only, so the app sends the matching command on the switch's edge, once, and never while calibrating.
      if (!rcAlive() && !rcCal) {
        const fire = (p, go) => {
          const on = rcFuncValue(pwm, p) > 0.5, was = rcEdge[p];
          rcEdge[p] = on;
          if (on && was === false) go();
        };
        fire('NLF_RC_CH', () => { logLine('[ui] remote: nose-lift takeoff'); api('/api/px4/command', { command: 'nose_lift_takeoff' }).catch(e => logLine('[ui] ' + e.message)); });
        fire('NLF_RC_LAND', () => { logLine('[ui] remote: nose-lift land'); api('/api/px4/command', { command: 'nose_lift_land' }).catch(e => logLine('[ui] ' + e.message)); });
        fire('RC_MAP_KILL_SW', () => { logLine('[ui] remote: kill'); api('/api/px4/command', { command: 'kill' }).catch(e => logLine('[ui] ' + e.message)); });
        const fm = pv('RC_MAP_FLTMODE', 0);
        if (fm) {
          const third = pwm[fm - 1] ? rcThird(rcNorm(fm, pwm[fm - 1])) : null;
          if (third != null && third !== rcEdge.fltmode) {
            const first = rcEdge.fltmode === undefined;
            rcEdge.fltmode = third;
            const mode = RC_MODE_NAMES[pv('COM_FLTMODE' + MODE_SLOTS[third][1], -1)];
            if (mode && !first) { logLine('[ui] remote: ' + mode); api('/api/px4/command', { command: 'mode', mode }).catch(e => logLine('[ui] ' + e.message)); }
          }
        }
      }
    }
  }
  if (joyCurrent() || rcVisible()) requestAnimationFrame(joyTick); else setTimeout(joyTick, 500);
}
if (navigator.hid) navigator.hid.addEventListener('disconnect', (e) => { if (e.device === hidDevice) { hidDevice = null; joySrc = null; logLine('[ui] radio disconnected'); joyRenderMap(); } });
$('#joy-connect').addEventListener('click', serialConnect);
joyRenderMap(); joyTick(); serialReconnect();


// ============================================================ batch: a list of actions, run in order
// A batch is whatever you put in the list. The actions run top to bottom, and `Repeat` runs the whole list
// again. Nothing is implicit: a scenario job carries the airframe exactly as the actions before it left it.
// ============================================================ wind, and missions
// A mission is a list of plain actions — take off, hover, move, cruise, land — kept with the design and
// flown by the scenario runner, which is the same machinery the scripted tests use. Nothing here is
// specific to one aircraft: the actions say what to do, not which motor does it.
const windFromVec = (n, e) => (Math.hypot(n, e) < 1e-6 ? 0 : (Math.atan2(-e, -n) * 180 / Math.PI + 360) % 360);
const windVec = (speed, fromDeg) => {
  const a = fromDeg * Math.PI / 180;
  return [-speed * Math.cos(a), -speed * Math.sin(a)];   // blowing towards, in NED
};
function showWindNow(st) {
  const w = (st && st.wind) || [0, 0, 0];
  const sp = Math.hypot(w[0], w[1]);
  $('#wind-now').textContent = sp < 0.05 && Math.abs(w[2]) < 0.05 ? 'calm'
    : `${sp.toFixed(1)} m/s from ${windFromVec(w[0], w[1]).toFixed(0)}°${Math.abs(w[2]) > 0.05 ? `, ${w[2].toFixed(1)} down` : ''}`;
}
// The wind is a model on the server (sim/wind.py): a mean, and optionally gusts that wander around it.
// The preview is drawn from what that model says it will blow, asked for over the wire, so the picture and
// the air the aircraft flies in come from one piece of arithmetic.
let windGusty = false;
const windModel = () => ({
  speed: +$('#wind-speed').value || 0, direction_deg: +$('#wind-dir').value || 0, down: +$('#wind-down').value || 0,
  gusty: windGusty, gust_speed: +$('#wind-gust').value || 0,
  gust_period_s: +$('#wind-period').value || 8, gust_dir_deg: +$('#wind-swing').value || 0,
});
function fillWindModel(w) {
  if (!w) return;
  windGusty = !!w.gusty;
  $('#wind-speed').value = +(+w.speed).toFixed(2);
  $('#wind-dir').value = +(+w.direction_deg).toFixed(0);
  $('#wind-down').value = +(+w.down).toFixed(2);
  $('#wind-gust').value = +(+w.gust_speed).toFixed(1);
  $('#wind-period').value = +(+w.gust_period_s).toFixed(1);
  $('#wind-swing').value = +(+w.gust_dir_deg).toFixed(0);
  windKindUi();
}
function windKindUi() {
  $$('#wind-kind button').forEach(b => b.classList.toggle('active', (b.dataset.kind === 'gusty') === windGusty));
  $('#wind-gust-fields').hidden = !windGusty;
  drawWindPreview();
}
let windPreviewTimer = null, windPreview = null;
function drawWindPreview() {
  clearTimeout(windPreviewTimer);
  windPreviewTimer = setTimeout(async () => {
    try { windPreview = await api('/api/sim/wind/preview', { ...windModel(), seconds: 60 }); }
    catch { return; }
    paintWindPreview();
  }, 120);
}
function paintWindPreview() {
  const c = $('#wind-preview'), p = windPreview;
  if (!c || !p) return;
  const w = c.clientWidth || 300, h = 64;
  const dpr = devicePixelRatio || 1;
  c.width = w * dpr; c.height = h * dpr;
  const g = c.getContext('2d'); g.scale(dpr, dpr); g.clearRect(0, 0, w, h);
  const css = getComputedStyle(document.documentElement);
  const hi = Math.max(1, ...p.speed) * 1.15;
  const x = (i) => w * i / (p.t.length - 1), y = (v) => h - 4 - (h - 12) * v / hi;
  // the mean, so a gust is visibly a swing about something
  const mean = +$('#wind-speed').value || 0;
  g.strokeStyle = (css.getPropertyValue('--text-2') || '#999').trim(); g.setLineDash([3, 3]); g.lineWidth = 1;
  g.beginPath(); g.moveTo(0, y(mean)); g.lineTo(w, y(mean)); g.stroke(); g.setLineDash([]);
  g.strokeStyle = '#0ea5e9'; g.lineWidth = 1.5; g.beginPath();
  p.speed.forEach((v, i) => (i ? g.lineTo(x(i), y(v)) : g.moveTo(x(i), y(v))));
  g.stroke();
  // the direction as a band around the mean heading, drawn against the same box
  if (windGusty && (+$('#wind-swing').value || 0) > 0) {
    const d0 = +$('#wind-dir').value || 0, sw = Math.max(1, +$('#wind-swing').value || 1);
    g.strokeStyle = 'rgba(148,163,184,0.85)'; g.lineWidth = 1; g.beginPath();
    p.dir.forEach((d, i) => { const yy = h / 2 - (h / 2 - 6) * (d - d0) / (sw * 1.3); i ? g.lineTo(x(i), yy) : g.moveTo(x(i), yy); });
    g.stroke();
  }
  g.fillStyle = (css.getPropertyValue('--text-2') || '#999').trim();
  g.font = '10px -apple-system, system-ui, sans-serif';
  g.fillText(`${Math.min(...p.speed).toFixed(1)}–${Math.max(...p.speed).toFixed(1)} m/s over 60 s`, 4, 11);
}
async function applyWind() {
  try {
    const r = await api('/api/sim/wind', windModel());
    fillWindModel(r.wind);
    const m = r.wind;
    logLine(`[wind] ${m.gusty ? `${m.speed.toFixed(1)} ±${m.gust_speed.toFixed(1)} m/s gusting every ${m.gust_period_s}s` : `${m.speed.toFixed(1)} m/s`} from ${m.direction_deg.toFixed(0)}°`);
  } catch (e) { logLine('[wind] ' + e.message); }
}
$$('#wind-kind button').forEach(b => b.addEventListener('click', () => { windGusty = b.dataset.kind === 'gusty'; windKindUi(); }));
$$('#wind-speed, #wind-dir, #wind-down, #wind-gust, #wind-period, #wind-swing').forEach(el => el.addEventListener('input', drawWindPreview));
$('#wind-apply').addEventListener('click', applyWind);
$('#wind-calm').addEventListener('click', () => {
  windGusty = false;
  $('#wind-speed').value = 0; $('#wind-down').value = 0; $('#wind-gust').value = 0;
  windKindUi(); applyWind();
});
// the fields are the wind as it is set, so they come back from the server: a Set wind step in a sequence
// or another window changed it, and the tab should say so rather than show what was typed here last
const loadWindModel = () => api_get('/api/sim/wind').then(r => fillWindModel(r.wind)).catch(() => { });
loadWindModel();
new ResizeObserver(() => paintWindPreview()).observe($('#wind-preview'));

// ---- the actions a mission is built from, and the scenario phase each one becomes
const MISSION_ACTIONS = {
  takeoff: { label: 'Take off', fields: [['alt', 'height m', 2]],
             phase: (a) => takeoffMode() === 'px4' ? { type: 'takeoff', alt: a.alt, settle: 2, timeout: 60 }
                                                   : { type: 'module', action: 'takeoff', alt: a.alt, timeout: 90 } },
  hover:   { label: 'Hover', fields: [['s', 'seconds', 10]], phase: (a) => ({ type: 'hold', duration: a.s }) },
  move:    { label: 'Move to', fields: [['n', 'north m', 0], ['e', 'east m', 0], ['alt', 'height m', 2]],
             phase: (a) => ({ type: 'offboard_position', pos: [a.n, a.e, -a.alt], tolerance: 1.0, timeout: 60 }) },
  cruise:  { label: 'Cruise at', fields: [['speed', 'm/s', 10], ['s', 'seconds', 15]],
             phase: (a) => ({ type: 'cruise_velocity', speed: a.speed, duration: a.s }) },
  mode:    { label: 'Set mode', fields: [['mode', 'mode', 'hold', FLIGHT_MODES.map(m => m[0])]],
             phase: (a) => ({ type: 'mode', mode: a.mode }) },
  wind:    { label: 'Set wind', fields: [['speed', 'm/s', 0], ['dir', 'from°', 0]],
             phase: (a) => ({ type: 'wind', ned: [...windVec(a.speed, a.dir), 0] }) },
  param:   { label: 'Set a parameter', fields: [['name', 'name', 'MPC_XY_VEL_MAX'], ['value', 'value', 0]],
             phase: (a) => ({ type: 'param', name: a.name, value: a.value }) },
  motor:   { label: 'Fail a motor', fields: [['rotor', 'rotor #', 1], ['scale', 'thrust left', 0]],
             phase: (a) => ({ type: 'motor_failure', rotor: Math.max(0, a.rotor - 1), scale: a.scale }) },
  wait:    { label: 'Wait', fields: [['s', 'seconds', 5]], phase: (a) => ({ type: 'wait', duration: a.s }) },
  land:    { label: 'Land', fields: [],
             phase: () => landMode() === 'px4' ? { type: 'land', timeout: 90 } : { type: 'module', action: 'land', timeout: 120 } },
};
let editingStep = -1;              // the step whose fields are open, or -1
const missions = () => ((airframe && (airframe.design ||= {})).missions ||= []);
const missionIndex = () => Math.max(0, Math.min(missions().length - 1, +(airframe?.design?.mission_index ?? 0)));
const currentMission = () => missions()[missionIndex()];

function newMission(name) {
  return { name: name || `Mission ${missions().length + 1}`, max_time: 180,
           steps: [{ kind: 'takeoff', alt: 2 }, { kind: 'hover', s: 15 }, { kind: 'land' }] };
}
function describeStep(a) {
  const spec = MISSION_ACTIONS[a.kind];
  if (!spec) return a.kind;
  const bits = spec.fields.map(([k, label]) => `${a[k]} ${label.replace(/°$/, '°')}`).join(', ');
  return spec.label + (bits ? ' · ' + bits : '');
}
/** The fields of one action, as inputs. `values` pre-fills them when an existing step is being edited. */
function actionFieldsHtml(kind, values) {
  const spec = MISSION_ACTIONS[kind];
  return (spec.fields || []).map(([k, label, dflt, options]) => {
    const v = values && values[k] !== undefined ? values[k] : dflt;
    return options
      ? `<label>${esc(label)}<select data-f="${k}">${options.map(o => `<option${o === v ? ' selected' : ''}>${esc(o)}</option>`).join('')}</select></label>`
      : `<label>${esc(label)}<input data-f="${k}" type="${typeof dflt === 'number' ? 'number' : 'text'}" step="any" value="${esc(v)}" style="width:${typeof dflt === 'number' ? 70 : 130}px"></label>`;
  }).join('') || '<span class="hint">nothing to set</span>';
}
const readFields = (root, kind) => {
  const spec = MISSION_ACTIONS[kind], out = { kind };
  (spec.fields || []).forEach(([k, , dflt]) => {
    const el = root.querySelector(`[data-f="${k}"]`);
    if (!el) return;
    out[k] = typeof dflt === 'number' ? (parseFloat(el.value) || 0) : el.value.trim();
  });
  return out;
};
function renderMissionFields() {
  $('#mi-fields').innerHTML = actionFieldsHtml($('#mi-kind').value);
}
function renderMissions() {
  if (!airframe || !$('#mi-pick')) return;
  if (!missions().length) missions().push(newMission('First flight'));
  const idx = missionIndex();
  const m = currentMission();
  $('#mi-pick').innerHTML = missions().map((x, i) => `<option value="${i}"${i === idx ? ' selected' : ''}>${esc(x.name)} · ${x.steps.length} step${x.steps.length === 1 ? '' : 's'}</option>`).join('');
  $('#mi-max').value = m.max_time ?? 180;
  $('#mi-steps').innerHTML = m.steps.map((a, i) => {
    const open = i === editingStep;
    const body = open
      ? `<div class="row tight" data-edit="${i}"><b>${i + 1}.</b>
           <select data-f="kind" style="width:130px">${Object.entries(MISSION_ACTIONS).map(([k, v]) =>
             `<option value="${k}"${k === a.kind ? ' selected' : ''}>${esc(v.label)}</option>`).join('')}</select>
           ${actionFieldsHtml(a.kind, a)}</div>`
      : `<div class="step-text"><b>${i + 1}.</b> ${esc(describeStep(a))}</div>`;
    return `<div class="conn-row step ${open ? 'editing' : ''}" data-step="${i}">${body}
      <div class="row tight"><button class="pill small" data-up="${i}" ${i ? '' : 'disabled'}>↑</button>
      <button class="pill small" data-del="${i}">✕</button></div></div>`;
  }).join('') || '<div class="hint">No actions yet. Add one below; they run in order. Click one to change it.</div>';
  $$('#mi-steps [data-del]').forEach(b => b.addEventListener('click', (e) => {
    e.stopPropagation(); m.steps.splice(+b.dataset.del, 1); editingStep = -1; renderMissions(); pushAirframe(true);
  }));
  $$('#mi-steps [data-up]').forEach(b => b.addEventListener('click', (e) => {
    e.stopPropagation();
    const i = +b.dataset.up; [m.steps[i - 1], m.steps[i]] = [m.steps[i], m.steps[i - 1]];
    editingStep = editingStep === i ? i - 1 : editingStep === i - 1 ? i : editingStep;
    renderMissions(); pushAirframe(true);
  }));
  // Click a step to open it; its fields are the same ones it was added with.
  $$('#mi-steps .step-text').forEach(el => el.addEventListener('click', () => {
    editingStep = +el.closest('.step').dataset.step; renderMissions();
  }));
  $$('#mi-steps [data-edit]').forEach(box => {
    const i = +box.dataset.edit;
    const kindSel = box.querySelector('[data-f="kind"]');
    kindSel.addEventListener('change', () => { m.steps[i] = { kind: kindSel.value }; renderMissions(); pushAirframe(true); });
    box.querySelectorAll('[data-f]:not([data-f="kind"])').forEach(inp => inp.addEventListener('change', () => {
      m.steps[i] = readFields(box, kindSel.value); renderMissions(); pushAirframe(true);
    }));
    const first = box.querySelector('[data-f]:not([data-f="kind"])');
    if (first && document.activeElement === document.body) first.focus();
  });
  $('#mi-state').textContent = `${missions().length} sequence${missions().length === 1 ? '' : 's'}`;
  if (!$('#mi-kind').options.length) {
    $('#mi-kind').innerHTML = Object.entries(MISSION_ACTIONS).map(([k, v]) => `<option value="${k}">${esc(v.label)}</option>`).join('');
    renderMissionFields();
  }
}
$('#mi-kind').addEventListener('change', renderMissionFields);
$('#mi-pick').addEventListener('change', (e) => { airframe.design.mission_index = +e.target.value; renderMissions(); pushAirframe(true); });
$('#mi-new').addEventListener('click', () => { missions().push(newMission()); airframe.design.mission_index = missions().length - 1; renderMissions(); pushAirframe(true); });
$('#mi-rename').addEventListener('click', () => {
  const m = currentMission(); if (!m) return;
  const box = $('#mi-pick');
  const input = document.createElement('input');
  input.type = 'text'; input.value = m.name; input.className = 'name'; input.style.flex = '1';
  box.replaceWith(input); input.focus(); input.select();
  const done = (commit) => { if (commit) { m.name = input.value.trim() || m.name; pushAirframe(true); } input.replaceWith(box); renderMissions(); };
  input.onblur = () => done(true);
  input.onkeydown = (ev) => { if (ev.key === 'Enter') done(true); if (ev.key === 'Escape') done(false); };
});
$('#mi-delete').addEventListener('click', (e) => {
  if (missions().length <= 1) { logLine('[mission] this is the only mission'); return; }
  if (!confirmTwice('mi-delete', e.currentTarget, 'delete this mission')) return;
  missions().splice(missionIndex(), 1);
  airframe.design.mission_index = 0;
  renderMissions(); pushAirframe(true);
});
$('#mi-max').addEventListener('change', () => {
  const m = currentMission(); if (!m) return;
  m.max_time = Math.max(10, +$('#mi-max').value || 180);
  pushAirframe(true);
});
$('#mi-add').addEventListener('click', () => {
  const m = currentMission(); if (!m) return;
  m.steps.push(readFields($('#mi-fields'), $('#mi-kind').value));
  editingStep = -1;
  renderMissions(); pushAirframe(true);
});
// Clicking anywhere else closes the open step, so the list goes back to reading as a list. The path is
// read from the event, not from the target: opening a step re-renders the row, and by the time this runs
// the element that was clicked is no longer in the document.
document.addEventListener('click', (e) => {
  if (editingStep < 0) return;
  if (e.composedPath().some(n => n && n.id === 'mi-steps')) return;
  editingStep = -1; renderMissions();
});

// ---- flying it: the mission becomes a scenario and the runner takes it from there
function missionScenario(m) {
  return {
    name: m.name, max_time: m.max_time || 180,
    phases: [{ type: 'wait_ready', timeout: 60 },
             ...m.steps.map(a => ({ name: describeStep(a), ...MISSION_ACTIONS[a.kind].phase(a) }))],
  };
}
$('#mi-run').addEventListener('click', async () => {
  const m = currentMission(); if (!m || !m.steps.length) { logLine('[mission] nothing to fly'); return; }
  try {
    const r = await api('/api/sim/scenario/start', missionScenario(m));
    logLine(`[mission] flying ${r.name}`);
  } catch (err) { logLine('[mission] ' + err.message); }
  pollMission();
});
$('#mi-stop').addEventListener('click', async () => {
  try { await api('/api/sim/scenario/stop', {}); logLine('[sequence] stopped; the aircraft is still flying'); } catch (err) { logLine('[sequence] ' + err.message); }
});
let missionTimer = null;
// Which step is being flown, marked on the step itself. The scenario's phase 0 is the wait for PX4 to be
// ready, so step i is phase i + 1.
function markRunningStep(step, state) {
  $$('#mi-steps .step').forEach(el => {
    const i = +el.dataset.step;
    el.classList.toggle('now', i === step);
    el.classList.toggle('done', step >= 0 && i < step);
  });
  $('#mi-state').textContent = state || `${missions().length} sequence${missions().length === 1 ? '' : 's'}`;
}
async function pollMission() {
  clearTimeout(missionTimer);
  if (!$('#tab-sequences').classList.contains('active')) return;
  try {
    const r = await api_get('/api/sim/scenario');
    const p = r && r.phase;
    const steps = (currentMission() || {}).steps || [];
    const step = r && r.running && r.index > 0 && r.index - 1 < steps.length ? r.index - 1 : -1;
    const what = !r || (!r.running && !p) ? ''
      : r.running ? `step ${Math.max(1, r.index)} of ${steps.length}${p ? ' · ' + p.name : ''}`
      : `${r.ok === false ? 'failed' : 'finished'}${(r.failures || []).length ? ' · ' + r.failures[0] : ''}`;
    $('#mi-progress').textContent = what;
    markRunningStep(step, r && (r.running || p) ? what : '');
  } catch { }
  missionTimer = setTimeout(pollMission, 1000);
}

let batchTimer = null, lastJobs = null;
const jobsOpen = new Set();
const mnum = (v, d = 2) => (v == null || !isFinite(v)) ? '—' : (+v).toFixed(d);
async function refreshBatch() {
  clearTimeout(batchTimer);
  if (!$('#tab-sequences').classList.contains('active')) return;
  pollJobs();
}
async function pollJobs() {
  try { const r = await api('/api/batch/jobs'); lastJobs = r; renderJobs(r); } catch (e) { }
  clearTimeout(batchTimer);
  if ($('#tab-sequences').classList.contains('active')) batchTimer = setTimeout(pollJobs, 1500);
}
function renderJobs(r) {
  const jobs = r.jobs || [];
  const el = $('#batch-jobs');
  if (!jobs.length) { el.innerHTML = `<div class="hint">No jobs yet.${r.free_instances ? ' Free PX4 instances: ' + r.free_instances.join(', ') : ''}</div>`; return; }
  const now = Date.now() / 1000;
  const rows = jobs.map(j => {
    const res = j.result || {};
    const t = res.timing || {};
    const st = j.running ? '<span class="warn">running</span>' : res.ok ? '<span class="ok">ok</span>' : '<span class="err">FAILED</span>';
    const wall = j.running ? (now - (j.t0 || now)) : (t.wall_s ?? ((j.t1 || now) - (j.t0 || now)));
    const simT = t.sim_s ?? res.sim_time;
    const open = jobsOpen.has(j.id);
    const fails = (res.failures || []).length ? `<div class="err small">${(res.failures || []).map(esc).join('<br>')}</div>` : '';
    const detail = open ? `<tr class="detail"><td colspan="7">${j.running ? `<pre class="joblog">${(j.log || []).map(esc).join('\n')}</pre>` : `<pre class="joblog">${esc(JSON.stringify({ metrics: res.metrics, timing: res.timing, failures: res.failures, px4_params_verified: res.px4_params_verified, log: j.log }, null, 1))}</pre>`}</td></tr>` : '';
    return `<tr data-job="${esc(j.id)}" class="${open ? 'open' : ''}"><td class="idx mono">${esc(j.id)}</td><td>${esc(j.scenario)}</td><td>${st}</td><td class="num">${mnum(simT, 1)} s</td><td class="num">${mnum(wall, 0)} s</td><td class="num">${t.rtf != null ? mnum(t.rtf, 1) : '—'}</td>
      <td class="metrics">${j.running ? `<span class="hint">${esc((j.log || []).slice(-1)[0] || 'starting…')}</span>` : phaseSummary(res) + fails}</td></tr>` + detail;
  }).join('');
  el.innerHTML = `<table class="grid jobs"><thead><tr><th>Job</th><th>Scenario</th><th>Status</th><th title="simulated time">Sim</th><th title="wall-clock time">Wall</th><th title="real-time factor">RTF</th><th>Metrics</th></tr></thead><tbody>${rows}</tbody></table>`;
  $$('#batch-jobs tr[data-job]').forEach(tr => tr.addEventListener('click', () => { const id = tr.dataset.job; if (jobsOpen.has(id)) jobsOpen.delete(id); else jobsOpen.add(id); if (lastJobs) renderJobs(lastJobs); }));
}


$('#af-takeoff').addEventListener('change', async () => {
  const inp = $('#af-takeoff');
  const value = Number(inp.value);
  try {
    if (status.armed || !status.ctl_connected || !Number.isFinite(value)) throw new Error('Enter a finite takeoff pitch while connected and disarmed');
    const saved = await api('/api/params/set', { name: 'NLF_TARGET', value });
    if (!saved.ok || Math.abs(Number(saved.value) - value) > 0.01) throw new Error(saved.error || 'PX4 did not confirm the takeoff pitch');
    params.NLF_TARGET = { ...(params.NLF_TARGET || {}), value };
    airframe.px4_overrides = { ...(airframe.px4_overrides || {}), NLF_TARGET: value };
    logLine('[ui] Takeoff pitch saved: ' + value + '°');
  } catch (e) {
    logLine('[ui] ' + e.message);
    inp.value = (params.NLF_TARGET || {}).value ?? airframe.px4_overrides?.NLF_TARGET ?? 25;
  }
});

['x', 'y', 'z'].forEach((axis, i) => bindNumber('imu-' + axis, v => {
  airframe.design ||= {};
  airframe.design.pixhawk_position ||= [...airframe.mass.cg];
  airframe.design.pixhawk_position[i] = v;
}));
['rr', 'rp', 'ry'].forEach((k, i) => bindNumber('imu-' + k, v => {
  airframe.design ||= {};
  airframe.design.pixhawk_rotation ||= [0, 0, 0];
  airframe.design.pixhawk_rotation[i] = v;
  fillSensorCard();
}));
['x', 'y', 'z'].forEach((axis, i) => bindNumber('flow-' + axis, v => { flowSensor().position[i] = v; }));
['rr', 'rp', 'ry'].forEach((k, i) => bindNumber('flow-' + k, v => { flowSensor().rotation[i] = v; fillSensorCard(); }));
bindNumber('flow-range', v => { flowSensor().range_m = Math.max(0.5, v); });
// Indoors is a fact about where this aircraft flies, not a parameter: it decides which sensors the
// estimator is allowed to wait for, and the export writes that out.
$('#flow-indoor').addEventListener('change', e => {
  (airframe.design ||= {}).indoor = e.target.checked;
  fillSensorCard(); pushAirframe(true); markDirty();
});
$('#flow-on').addEventListener('change', e => {
  const f = flowSensor();
  f.enabled = e.target.checked;
  if (f.enabled && !f.position.some(Number.isFinite)) f.position = [...airframe.mass.cg];
  fillSensorCard(); scene.setAirframe(airframe); pushAirframe(); markDirty();
});

$('#motor-test-stop').addEventListener('click', () => { stopMotorTest(); $('#motor-enable').checked=false; });


