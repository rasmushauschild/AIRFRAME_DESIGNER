let lastPrearmReportRequest = 0;
let prearmReportRequestPending = false;
import { noseSelection, removeNoseMotor, simulatorNoseSelected } from '/static/nose_selection.js';
const V = new URL(import.meta.url).search;
const { createScene } = await import('/static/scene.js' + V);

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
    onRotorChanged: (i, r, commit) => { renderRotorRow(i); renderReadout(); if (commit) pushAirframe(); },
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
  const noop = () => { };
  scene = { setAirframe: noop, updateState: noop, select: noop, updateRotorNode: noop, setTheme: noop, setFollow: noop, setCameraMode: noop, setMode: noop, selected: -1, focusOrigin: noop, setCad: noop, selectCad: noop, syncCad: noop, selectedCad: null };
  $('#viewport').insertAdjacentHTML('afterbegin', '<div class="hint" style="padding:18px">3D view unavailable in this window (no WebGL). Everything else works.</div>');
}

// ============================================================ tabs
$$('.tabs button').forEach(b => b.addEventListener('click', () => {
  $$('.tabs button').forEach(x => x.classList.toggle('active', x === b));
  $$('.tab').forEach(t => t.classList.toggle('active', t.id === 'tab-' + b.dataset.tab));
  if (b.dataset.tab === 'airframe') loadExport();
  if (b.dataset.tab === 'px4') { ensureParams(); loadExport(); }
  if (b.dataset.tab === 'controller') { refreshConnection(); ensureParams(); renderRcLive(); renderModeButtons(); }
  if (b.dataset.tab === 'batch') refreshBatch();
  if (b.dataset.tab === 'firmware') refreshFirmware();
}));
function openTab(name) { $$('.tabs button').find(b => b.dataset.tab === name)?.click(); }


// ============================================================ confirmation without dialogs
// window.confirm() returns false at once inside the Claude desktop app's web view (and other embedded browsers),
// so destructive buttons ask on the button itself: the first click turns it into "Really …?" for 8 s,
// and the second does the thing. `label` is the short verb phrase that question is built from.
const confirmArm = new Map();   // key -> expiry (ms)
function confirmArmed(key) { return (confirmArm.get(key) || 0) > Date.now(); }
function confirmTwice(key, btn, label) {
  if (confirmArmed(key)) { confirmArm.delete(key); btn.classList.remove('confirm'); return true; }
  confirmArm.set(key, Date.now() + 8000);
  if (btn.tagName === 'BUTTON') {
    btn.dataset.label = btn.dataset.label || btn.textContent;
    btn.textContent = 'Really ' + label + '?'; btn.classList.add('confirm');
  } else markSaved('Really ' + label + '? Choose it again', 8000);
  setTimeout(() => { if (!confirmArmed(key)) { confirmArm.delete(key); if (btn.tagName === 'BUTTON' && document.body.contains(btn)) { btn.textContent = btn.dataset.label; btn.classList.remove('confirm'); } } }, 8100);
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
  fillMassCard();
  ['bx', 'by', 'bz'].forEach((k, i) => $('#af-' + k).value = af.body.size[i]);
  ['dragx', 'dragy', 'dragz'].forEach((k, i) => $('#af-' + k).value = af.body.drag_quadratic[i]);
  ['adragx', 'adragy', 'adragz'].forEach((k, i) => $('#af-' + k).value = af.body.drag_angular[i]);
  ['dcx', 'dcy', 'dcz'].forEach((k, i) => $('#af-' + k).value = af.body.drag_center[i]);
  renderRotorTable();
  renderWingTable();
  renderLegTable();
  renderMotorSliders();
  fillMotorCard();
  fillCadCard();
  loadExport();          // fills the edited/affected-parameters section (needs the server's export view)
}
function fillMassCard() {
  const m = airframe.mass;
  const derived = !!m.from_items;
  ['mass', 'cgx', 'cgy', 'cgz', 'ixx', 'iyy', 'izz'].forEach(k => { const el = $('#af-' + k); el.disabled = derived; el.title = derived ? 'derived from the CAD bodies / mass items (untick "Mass, CG & inertia from bodies" to type it)' : 'Manually entered value'; });
  $('#af-estimate').disabled = derived;
  $('#af-mass').value = m.mass;
  const imu = airframe.design?.pixhawk_position || m.cg;
  ['x', 'y', 'z'].forEach((axis, i) => { $('#imu-' + axis).value = imu[i]; });
  ['cgx', 'cgy', 'cgz'].forEach((k, i) => $('#af-' + k).value = +(+m.cg[i]).toFixed(4));
  ['ixx', 'iyy', 'izz'].forEach((k, i) => $('#af-' + k).value = +(+m.inertia[i]).toFixed(5));
  $('#af-battery').value = (airframe.design && airframe.design.battery_wh) || 0;
  $('#af-hover').value = airframe.hover_pitch_deg || 0;
  $('#af-landed').value = airframe.landed_pitch_deg || 0;
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
  if (cad && cad.file) {
    for (const b of cad.bodies || []) if (b.pos) cadCentroids[b.id] = b.pos.map((v, i) => v - (b.offset ? b.offset[i] : 0));
    $('#cad-filename').textContent = cad.file.replace(/^airframes\/cad\//, '');
    $('#cad-show').checked = cad.visible !== false;
    ['rx', 'ry', 'rz'].forEach((k, i) => $('#cad-' + k).value = +(+(cad.rotation_deg || [0, 0, 0])[i]).toFixed(2));
    ['ox', 'oy', 'oz'].forEach((k, i) => $('#cad-' + k).value = +(+(cad.origin || [0, 0, 0])[i]).toFixed(4));
    $('#cad-scale').value = cad.scale ?? 1;
    $('#cad-frame-row').style.display = '';
    $('#cad-remove-all').hidden = false;
    const key = cadKey(cad);
    if (key !== cadMeshKey) loadCadMesh(key);
  } else {
    $('#cad-filename').textContent = '';
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
  <td><input type="number" step="0.01" min="0" data-k="mass" value="${+(+b.mass || 0).toFixed(4)}" title="mass of this body, kg (0 = ignored)"></td>
  <td class="num pos" title="centroid, structural frame, m${moved ? ' (dragged by ' + b.offset.map(v => v.toFixed(3)).join(', ') + ')' : ''}">${p.map(v => v.toFixed(3)).join('  ')}${moved ? ' <span class="warn" title="moved from the CAD position">•</span>' : ''}</td>
  <td><button class="del" title="remove this body from the list">✕</button></td></tr>`;
}
function renderCadTable() {
  const el = $('#cad-table'); const cad = airframe && airframe.cad;
  if (!cad || !cad.file) { el.innerHTML = '<div class="hint">No CAD file</div>'; $('#cad-totals').innerHTML = ''; $('#cad-summary').textContent = ''; return; }
  const live = (cad.bodies || []).filter(b => !b.removed);
  el.innerHTML = `<table class="grid cad"><thead><tr><th>#</th><th>Body</th><th class="num" title="litres">Vol L</th><th>Mass kg</th><th title="centroid in the structural frame (x fwd, y right, z down), m: click a row to highlight the body, drag it in the 3D view along its axes">X Y Z</th><th></th></tr></thead><tbody>${live.map(cadRowHtml).join('') || '<tr><td colspan="6" class="hint">All bodies removed.</td></tr>'}</tbody></table>`;
  el.querySelectorAll('tr[data-cad]').forEach(tr => {
    const id = tr.dataset.cad;
    tr.addEventListener('click', (e) => { if (['INPUT', 'BUTTON'].includes(e.target.tagName)) return; scene.selectCad(id, e.shiftKey); cadSelected = scene.selectedCad; if (cadSelected !== null && selected >= 0) { selected = -1; renderRotorTable(); renderReadout(); } renderCadTable(); });
    tr.querySelector('input[data-k="mass"]').addEventListener('change', (e) => { const b = cadBody(id); b.mass = Math.max(0, parseFloat(e.target.value) || 0); scene.syncCad(); renderCadTotals(); pushAirframe(true); });
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
  let cgTxt = 'no masses yet';
  if (m > 0) { const cg = [0, 0, 0]; for (const b of live) { const p = cadPos(b); for (let i = 0; i < 3; i++) cg[i] += p[i] * b.mass / m; } cgTxt = `CG of bodies x ${cg[0].toFixed(3)}  y ${cg[1].toFixed(3)}  z ${cg[2].toFixed(3)} m`; }
  el.innerHTML = `<b>${m.toFixed(2)} kg</b> in ${live.length} of ${(cad.bodies || []).length - removed} bodies · ${cgTxt}${airframe.mass.from_items ? ` · <span class="ok">aircraft CG follows the bodies</span>` : ' · tick "Mass, CG & inertia from bodies" to use it'}${removed ? ` · ${removed} removed <a href="#" id="cad-restore">restore</a>` : ''}`;
  const shown = (cad.bodies || []).length - removed;
  $('#cad-summary').textContent = `${cad.file.replace(/^airframes\/cad\//, '')} · ${shown} bodies${live.length ? ` (${live.length} with mass)` : ''} · ${m.toFixed(1)} kg`;
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
$('#af-name').addEventListener('change', e => { airframe.name = e.target.value.trim(); pushAirframe(true); loadPresetList(); loadVersionList(); });
bindNumber('af-mass', v => airframe.mass.mass = v);
bindNumber('af-cgx', v => airframe.mass.cg[0] = v);
bindNumber('af-cgy', v => airframe.mass.cg[1] = v);
bindNumber('af-cgz', v => airframe.mass.cg[2] = v);
bindNumber('af-battery', v => { airframe.design = airframe.design || {}; airframe.design.battery_wh = Math.max(0, v); });
$('#ro-batt').addEventListener('click', () => { openTab('airframe'); const el = $('#af-battery'); el.focus(); el.select(); });
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
  const el = $('#af-saved');
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
    + (r.presets || []).map(p => `<option value="${esc(p)}">New ${esc(p)}</option>`).join('')
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

function renderRotorTable() {
  const el = $('#rotor-table');
  const rows = airframe.rotors.map((r, i) => rotorRowHtml(i, r)).join('');
  el.innerHTML = `<table class="grid"><thead><tr><th title="Physical PWM port: 1–8 = MAIN 1–8; 9–10 = AUX 1–2">#</th><th title="enabled: takes part in the simulation and the PX4 export">On</th><th>Name</th><th title="position, m, structural frame (FRD like PX4): X forward">X</th><th title="Y right (+)">Y</th><th title="Z DOWN (+): a rotor 10 cm above the reference point has Z = -0.10">Z</th><th title="forward lean of the thrust axis from vertical, degrees; negative = backward">Tilt°</th><th title="sideways lean, degrees; positive = outward from the centreline (right for a rotor on the centreline)">Cant°</th><th title="resulting unit thrust vector = CA_ROTORn_AX / AY / AZ">Axis AX AY AZ</th><th title="direction of rotation; click a cell to flip it (the sign of KM)">Spin</th><th title="max thrust N">Tmax</th><th title="share of max thrust this rotor needs to hover, as PX4's allocator would solve it">Hover</th><th></th></tr></thead><tbody>${rows || '<tr><td colspan="13" class="hint">No rotors. Add one below.</td></tr>'}</tbody></table>`;
  api('/api/airframe/hover_check').then(showHover).catch(() => { });
  el.querySelectorAll('tr[data-i]').forEach(tr => {
    const i = +tr.dataset.i;
    tr.addEventListener('click', (e) => { if (!['INPUT', 'SELECT', 'OPTION', 'BUTTON'].includes(e.target.tagName) && !e.target.classList.contains('spin') && !e.target.classList.contains('del')) { selected = i; scene.select(i); renderRotorTable(); renderReadout(); } });
    tr.querySelectorAll('input, select:not(.rotor-port)').forEach(inp => inp.addEventListener('change', () => applyRow(i, tr)));
    tr.querySelector('.rotor-port').addEventListener('change', e => changeMotorPort(e.target));
    tr.querySelector('.spin').addEventListener('click', () => { airframe.rotors[i].km = -airframe.rotors[i].km; scene.updateRotorNode(i, airframe.rotors[i]); renderRotorTable(); pushAirframe(true); });
    tr.querySelector('.del').addEventListener('click', () => { airframe.design ||= {}; airframe.design.nose_lift ||= {};
      airframe.design.nose_lift.motors = removeNoseMotor(noseSelection(airframe.rotors, airframe.design.nose_lift.motors), i);
      airframe.rotors.splice(i, 1); selected = -1; setAirframe(airframe); pushAirframe(true); });
  });
}
const axisText = (a) => a.map(v => (v >= 0 ? ' ' : '') + v.toFixed(2)).join(' ');
function rotorRowHtml(i, r) {
  const ccw = r.km >= 0;
  const [tilt, cant] = axisToTilt(r.axis, r.pos[1]);
  return `<tr data-i="${i}" class="${i === selected ? 'selected' : ''} ${r.enabled === false ? 'off' : ''}"><td class="idx">${rotorPortHtml(i, r)}</td>
  <td><input type="checkbox" data-k="on" ${r.enabled === false ? '' : 'checked'}></td>
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
  tr.classList.toggle('off', !r.enabled);
  scene.updateRotorNode(i, r);
  renderRotorRow(i);
  renderRotorTable();
  renderReadout();
  pushAirframe(true);
}
function renderReadout() {
  const el = $('#rotor-readout');
  if (selected < 0 || !airframe.rotors[selected]) { el.classList.remove('show'); return; }
  const r = airframe.rotors[selected];
  const [tilt, cant] = axisToTilt(r.axis, r.pos[1]);
  el.classList.add('show');
  el.innerHTML = `<b>${esc(r.name || 'Motor ' + (selected + 1))}</b> pos [${r.pos.map(v => fmt(v)).join(', ')}] · axis [${r.axis.map(v => fmt(v, 3)).join(', ')}] (tilt ${tilt.toFixed(1)}°, cant ${cant.toFixed(1)}°) · ${r.km >= 0 ? 'CCW' : 'CW'}${r.enabled === false ? ' · disabled' : ''}`;
}
$('#rotor-add').addEventListener('click', () => {
  const base = airframe.rotors[selected] || airframe.rotors[airframe.rotors.length - 1] || { pos: [0.2, 0, 0], axis: [0, 0, -1], km: 0.05, max_thrust: 8, tau: 0.04, diameter: 0.25, thrust_exponent: 2, kind: 'prop', ram_drag: false, duct_axis: null, turn_loss: 0.1, enabled: true };
  const n = JSON.parse(JSON.stringify(base));
  n.pos = [n.pos[0] + 0.05, n.pos[1] + 0.05, n.pos[2]];
  n.name = 'M' + (airframe.rotors.length + 1); n.enabled = true;
  airframe.rotors.push(n); selected = airframe.rotors.length - 1;
  setAirframe(airframe); scene.select(selected); pushAirframe(true);
});
function mirror(axisIdx) {
  if (selected < 0 || !airframe.rotors[selected]) return;
  const n = JSON.parse(JSON.stringify(airframe.rotors[selected]));
  n.pos[axisIdx] = -n.pos[axisIdx]; n.axis[axisIdx] = -n.axis[axisIdx]; n.km = -n.km;
  n.name = 'M' + (airframe.rotors.length + 1);
  airframe.rotors.push(n); selected = airframe.rotors.length - 1;
  setAirframe(airframe); scene.select(selected); pushAirframe(true);
}
$('#rotor-mirror-y').addEventListener('click', () => mirror(1));
$('#rotor-mirror-x').addEventListener('click', () => mirror(0));
$('#rotor-apply-all').addEventListener('click', () => {
  if (selected < 0 || !airframe.rotors[selected]) return;
  const s = airframe.rotors[selected];
  airframe.rotors.forEach(r => { r.max_power_w = s.max_power_w || 0; r.max_thrust = s.max_thrust; r.tau = s.tau; r.diameter = s.diameter; r.thrust_exponent = s.thrust_exponent; r.kind = s.kind; r.ram_drag = s.ram_drag; r.turn_loss = s.turn_loss; r.km = (r.km >= 0 ? 1 : -1) * Math.abs(s.km); });
  setAirframe(airframe); pushAirframe(true);
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
  el.innerHTML = `<table class="grid wings"><thead><tr><th>On</th><th>Name</th><th title="root leading edge on the centreline, structural frame, m">X</th><th>Y</th><th title="Z DOWN (+), like PX4: a wing above the reference point has negative Z">Z</th><th title="tip to tip (symmetric) or panel length (single panel), m">Span</th><th title="root chord, m">Root</th><th title="tip chord, m">Tip</th><th title="leading-edge sweep, back positive">Swp°</th><th title="whole-wing pitch: the entire wing rotated rigidly about the aircraft pitch axis through its root point, nose-up positive (with sweep the tips move up/down with it)">Pitch°</th><th title="dihedral, tips up positive; 90 + single panel = vertical fin">Dih°</th><th title="section incidence at the root: each section turned about the span line, leading edge up positive">Inc°</th><th title="extra incidence at the tip (washout negative)">Tw°</th><th title="symmetric: left + right halves">Sym</th><th>Model</th><th title="stall angle of attack">Stall°</th><th title="planform area m² · aspect ratio">Area · AR</th><th></th></tr></thead>
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
    <td>${n('x', 0.01, w.pos[0])}</td><td>${n('y', 0.01, w.pos[1])}</td><td>${n('z', 0.01, w.pos[2])}</td>
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
  w.pos = [g('x'), g('y'), g('z')];
  w.span = Math.max(0, g('span')); w.root_chord = Math.max(0, g('root')); w.tip_chord = Math.max(0, g('tip'));
  w.sweep_deg = g('sweep'); w.pitch_deg = g('pitch'); w.dihedral_deg = g('dih'); w.incidence_deg = g('inc'); w.twist_deg = g('twist');
  w.symmetric = tr.querySelector('[data-k="sym"]').checked;
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
  const rows = airframe.legs.map((l, i) => legRowHtml(i, l)).join('');
  el.innerHTML = `<table class="grid legs"><thead><tr><th>On</th><th>Name</th><th title="attachment point on the structure, structural frame, m: X forward">X</th><th title="Y right (+)">Y</th><th title="Z DOWN (+), like PX4: legs usually have positive Z">Z</th><th title="attachment to foot, m">Len</th><th title="lean forward (+) / backward (-) from straight down">Tilt°</th><th title="lean outward (+) / inward (-) from the centreline">Cant°</th><th title="contact starts this far above the foot point (ball or skid tube), m">Foot r</th><th title="N/m">Stiff</th><th title="N/(m/s)">Damp</th><th title="foot point, structural frame">Foot</th><th></th></tr></thead>
    <tbody>${rows || '<tr><td colspan="13" class="hint">No legs. The vehicle needs at least 3 to stand; use Generate below.</td></tr>'}</tbody></table>`;
  el.querySelectorAll('tr[data-l]').forEach(tr => {
    const i = +tr.dataset.l;
    tr.querySelectorAll('input').forEach(inp => inp.addEventListener('change', () => applyLegRow(i)));
    tr.querySelector('.del').addEventListener('click', () => { airframe.legs.splice(i, 1); renderLegTable(); scene.setAirframe(airframe); pushAirframe(true); });
  });
}
const legFoot = (l) => {
  const side = l.attach[1] < 0 ? -1 : 1;
  const t = (l.tilt_deg || 0) * Math.PI / 180, c = (l.cant_deg || 0) * Math.PI / 180 * side;
  const d = [Math.sin(t), Math.sin(c) * Math.cos(t), Math.cos(c) * Math.cos(t)];
  return d.map((v, k) => +l.attach[k] + (+l.length || 0) * v);
};
function legRowHtml(i, l) {
  const n = (k, step, v, extra = '') => `<input type="number" step="${step}" data-k="${k}" value="${nf(v, 4)}" ${extra}>`;
  const f = legFoot(l);
  return `<tr data-l="${i}" class="${l.enabled === false ? 'off' : ''}">
    <td><input type="checkbox" data-k="on" ${l.enabled === false ? '' : 'checked'}></td>
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
  tr.classList.toggle('off', !l.enabled);
  const calc = tr.querySelector('td.calc'); if (calc) calc.textContent = legFoot(l).map(v => v.toFixed(2)).join(' ');
  scene.setAirframe(airframe); pushAirframe(true);
}
$('#leg-add').addEventListener('click', () => {
  const base = airframe.legs[airframe.legs.length - 1] || LEG_DEFAULT;
  const l = JSON.parse(JSON.stringify(base));
  l.name = 'leg' + (airframe.legs.length + 1); l.enabled = true;
  if (airframe.legs.length) l.attach = [l.attach[0] - 0.05, l.attach[1], l.attach[2]];
  airframe.legs.push(l);
  renderLegTable(); scene.setAirframe(airframe); pushAirframe(true);
});
$('#leg-mirror').addEventListener('click', () => {
  const src = airframe.legs[airframe.legs.length - 1]; if (!src) return;
  const l = JSON.parse(JSON.stringify(src));
  l.attach[1] = -l.attach[1];
  l.name = src.name.endsWith('L') ? src.name.slice(0, -1) + 'R' : src.name.endsWith('R') ? src.name.slice(0, -1) + 'L' : src.name + '_m';
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
  try {
    await api('/api/airframe', { airframe, keep_state: true });
    const r = await api('/api/px4/push', { save: true, firmware: true });
    const failed = r.results.filter(x => !x.ok);
    const fw = r.firmware || {};
    const fwNote = fw.flashed ? `${fw.rebuilt ? 'firmware built and ' : ''}board flashed · ` : (fw.rebuilt ? `firmware rebuilt${fw.build_seconds ? ' in ' + Math.round(fw.build_seconds) + ' s' : ''}, PX4 relaunched · ` : (fw.relaunched ? 'PX4 relaunched on the new firmware · ' : ''));
    statusEl.innerHTML = r.ok
      ? `<span class="ok">✓ ${fwNote}PX4 updated (${r.results.length} parameters)</span>`
      : `<span class="err">${failed.length} failed: ${failed.map(f => f.name + ' (' + f.error + ')').join(', ')}</span>`;
    if (!short && r.missing && r.missing.length) statusEl.innerHTML += `<div class="muted">not present in this firmware: ${r.missing.join(', ')}</div>`;
    geometryDirty = false;
    await loadExport();
  } catch (e) { statusEl.innerHTML = `<span class="err">${e.message}</span>`; }
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
  const stagedCheckbox = $('#staged-rc-takeoff');
  stagedCheckbox.disabled = !!status.armed || manualPrototype || !!pilotHandover;
  { const sel = $('#handover-mode');
    sel.disabled = !!status.armed || !nativeNoseLift;
    if (document.activeElement !== sel) sel.value = String(airframe?.px4_overrides?.NLF_PILOT ?? params.NLF_PILOT?.value ?? 0);
    const chosen = Number(sel.value);
    sel.title = nativeNoseLift ? (chosen && chosen !== pilotHandover ? 'Flash the firmware to apply' : '') : 'Needs the native nose-lift module'; }
  if (document.activeElement !== stagedCheckbox) stagedCheckbox.checked = Number(airframe.px4_overrides?.NLF_RC_THR ?? params.NLF_RC_THR?.value ?? 0) === 1;
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
  const simPitch = status.mode === 'sitl';
  targetInput.disabled = !!status.armed || !status.ctl_connected || (!simPitch && !status.native_module);
  targetInput.step = 'any';
  targetInput.title = simPitch
    ? 'SITL nose-lift target, shared with Flight and scenarios. Does not change hover pitch.'
    : 'PX4 NLF_TARGET: firmware nose-lift target. Editable while connected and disarmed.';
  if (document.activeElement !== targetInput) targetInput.value = simPitch
    ? (airframe.design?.nose_lift?.target_pitch_deg ?? airframe.hover_pitch_deg ?? 0)
    : ((params.NLF_TARGET || {}).value ?? airframe.px4_overrides?.NLF_TARGET ?? ((airframe.hover_pitch_deg ?? 25) - 10));

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
// ============================================================ edited parameters (overrides saved with the airframe)
function renderOverrides(r) {
  r = r || lastExport;
  const ov = (r && r.overrides) || (airframe && airframe.px4_overrides) || {};
  if (airframe) airframe.px4_overrides = ov;
  const el = $('#overrides');
  const f = (x) => (typeof x === 'number' && !Number.isInteger(x)) ? +x.toFixed(4) : x;
  const cur = (r && r.current) || {};
  const same = (k, v) => cur[k] != null && Math.abs(+cur[k] - +v) < 1e-4;
  const vehicleCell = (k, v) => `<span class="ov-cur ${cur[k] == null ? 'muted' : same(k, v) ? 'ok' : 'warn'}" title="value on the vehicle">${cur[k] == null ? '—' : f(cur[k])}</span>`;
  let html = '';
  // hand-edited, editable
  Object.keys(ov).sort().forEach(n => {
    const m = meta[n] || {};
    html += `<div class="ov-row"><span class="pname">${n}</span><input type="number" step="any" value="${ov[n]}" data-n="${n}">${vehicleCell(n, ov[n])}<span class="pdesc" title="${(m.short || '').replace(/"/g, '&quot;')}">${m.short || ''}</span><button class="del" data-n="${n}" title="forget this edit">✕</button></div>`;
  });
  // derived from the geometry, read-only, grouped per rotor
  if (r && r.params) {
    const P = r.params, keys = r.geometry_keys || [];
    const rotors = {}; const rest = [];
    keys.forEach(k => { const m = k.match(/^CA_ROTOR(\d+)_(PX|PY|PZ|AX|AY|AZ|KM)$/); if (m) (rotors[m[1]] = rotors[m[1]] || {})[m[2]] = P[k]; else if (!/^(HIL_ACT_FUNC|PWM_MAIN_FUNC)\d+$/.test(k)) rest.push(k); });
    const diff = (prefix) => keys.filter(k => k.startsWith(prefix)).some(k => !same(k, P[k]));
    if (html) html += '<div class="ov-sep"></div>';
    html += '<div class="hint" style="margin:2px 0 4px">From the geometry above · read-only</div>';
    rest.forEach(k => { html += `<div class="ov-row ro"><span class="pname">${k}</span><span class="ov-val">${f(P[k])}</span>${vehicleCell(k, P[k])}<span class="pdesc">${(meta[k] || {}).short || ''}</span><span></span></div>`; });
    Object.keys(rotors).sort((a, b) => a - b).forEach(i => {
      const R = rotors[i]; const d = diff(`CA_ROTOR${i}_`);
      html += `<div class="ov-row ro rotor"><span class="pname">CA_ROTOR${i}_*</span><span class="ov-val rot">P ${f(R.PX)} ${f(R.PY)} ${f(R.PZ)} · A ${f(R.AX)} ${f(R.AY)} ${f(R.AZ)} · KM ${f(R.KM)}</span><span class="ov-cur ${d ? 'warn' : 'ok'}">${d ? '≠ vehicle' : '✓'}</span><span class="pdesc">motor ${+i + 1}</span><span></span></div>`;
    });
    const funcs = keys.filter(k => /^(HIL_ACT_FUNC|PWM_MAIN_FUNC)\d+$/.test(k)).sort((a, b) => parseInt(a.match(/\d+$/)[0]) - parseInt(b.match(/\d+$/)[0]));
    if (funcs.length) {
      const on = funcs.filter(k => P[k] > 0), d = funcs.some(k => !same(k, P[k]));
      html += `<div class="ov-row ro"><span class="pname">${funcs[0].replace(/\d+$/, '')}1‑${funcs.length}</span><span class="ov-val rot">${on.map(k => P[k]).join(', ')}${on.length < funcs.length ? ', rest 0' : ''}</span><span class="ov-cur ${d ? 'warn' : 'ok'}">${d ? '≠ vehicle' : '✓'}</span><span class="pdesc">output → motor mapping</span><span></span></div>`;
    }
  }
  el.innerHTML = html || '<div class="hint">Nothing yet. Edit any parameter below and it appears here.</div>';
  $$('#overrides input').forEach(inp => inp.addEventListener('change', () => setParamValue(inp.dataset.n, parseFloat(inp.value))));
  $$('#overrides .del').forEach(b => b.addEventListener('click', async () => { await api('/api/airframe/override_remove', { name: b.dataset.n }); delete airframe.px4_overrides[b.dataset.n]; loadExport(); markDirty(); }));
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
  loadExport(); renderParams(); markDirty();
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
  const groups = [...new Set(Object.values(meta).map(m => m.group))].sort();
  $('#param-group').innerHTML = '<option value="">all groups</option>' + groups.map(g => `<option>${g}</option>`).join('');
}
async function loadParams() {
  const r = await api('/api/params');
  params = r.params || {};
  paramsLoaded = Object.keys(params).length > 0;
  loadExport();          // vehicle values are now known: refresh the ✓/≠ marks
  $('#param-count').textContent = `${Object.keys(params).length}/${r.count} loaded · ${Object.keys(meta).length} described`;
  renderParams();
}
function renderParams() {
  const q = $('#param-search').value.trim().toLowerCase();
  const grp = $('#param-group').value;
  const names = Object.keys(params).sort();
  let shown = 0;
  const html = [];
  if (!q && !grp) { $('#param-list').innerHTML = `<div class="hint">${names.length} parameters loaded · type to search, or pick a group.</div>`; return; }
  for (const n of names) {
    const m = meta[n] || {};
    if (grp && m.group !== grp) continue;
    if (q && !(n.toLowerCase().includes(q) || (m.short || '').toLowerCase().includes(q))) continue;
    if (++shown > 40) break;
    const v = params[n].value;
    const edited = airframe && airframe.px4_overrides && (n in airframe.px4_overrides);
    html.push(`<div class="prow ${n === selectedParam ? 'selected' : ''} ${edited ? 'edited' : ''}" data-n="${n}"><span class="pname">${edited ? '● ' : ''}${n}</span><span class="pval">${typeof v === 'number' && !Number.isInteger(v) ? +v.toPrecision(6) : v}${m.unit ? ' <span class="muted">' + m.unit + '</span>' : ''}</span><span class="pdesc" title="${(m.short || '').replace(/"/g, '&quot;')}">${m.short || ''}</span></div>`);
  }
  $('#param-list').innerHTML = html.join('') + (shown > 40 ? '<div class="hint">… more matches, refine the search</div>' : '');
  $$('#param-list .prow').forEach(el => el.addEventListener('click', () => openParam(el.dataset.n)));
}
let selectedParam = null;
function openParam(n) {
  selectedParam = n;
  const p = params[n], m = meta[n] || {};
  const ed = $('#param-editor');
  ed.classList.remove('hidden');
  let input;
  if (m.values && m.values.length) {
    input = `<select id="pe-value">${m.values.map(o => `<option value="${o.value}" ${+o.value === +p.value ? 'selected' : ''}>${o.value}: ${o.description}</option>`).join('')}</select>`;
  } else if (m.bitmask && m.bitmask.length) {
    input = `<div id="pe-bits">${m.bitmask.map(b => `<label class="row"><input type="checkbox" data-bit="${b.index}" ${(p.value >> b.index) & 1 ? 'checked' : ''}> ${b.index}: ${b.description}</label>`).join('')}</div>`;
  } else {
    input = `<input id="pe-value" type="number" step="${m.increment || (p.type === 6 ? 1 : 'any')}" value="${p.value}" ${m.min != null ? 'min="' + m.min + '"' : ''} ${m.max != null ? 'max="' + m.max + '"' : ''}>`;
  }
  ed.innerHTML = `<div><span class="pn">${n}</span> <span class="muted">${m.group || ''}</span></div>
    <div>${m.short || ''}</div>
    <div class="long">${m.long || ''}</div>
    <div class="meta">${p.type === 6 ? 'INT32' : 'FLOAT'}${m.unit ? ' · ' + m.unit : ''}${m.min != null ? ' · min ' + m.min : ''}${m.max != null ? ' · max ' + m.max : ''}${m.default != null ? ' · default ' + m.default : ''}${m.reboot ? ' · <span class="warn">reboot required</span>' : ''}</div>
    <div class="row">${input}<button id="pe-set" class="primary small">set</button><button id="pe-close" class="small">close</button><span id="pe-result"></span></div>`;
  $('#pe-close').addEventListener('click', () => { ed.classList.add('hidden'); selectedParam = null; renderParams(); });
  $('#pe-set').addEventListener('click', async () => {
    let value;
    if ($('#pe-bits')) value = $$('#pe-bits input').reduce((acc, cb) => acc | (cb.checked ? (1 << +cb.dataset.bit) : 0), 0);
    else value = parseFloat($('#pe-value').value);
    $('#pe-result').textContent = '…';
    try {
      const r = await setParamValue(n, value);
      $('#pe-result').innerHTML = r.ok ? `<span class="ok">✓ ${r.value} · saved with the airframe</span>` : `<span class="err">${r.error}</span>`;
    } catch (e) { $('#pe-result').innerHTML = `<span class="err">${e.message}</span>`; }
  });
  renderParams();
}
$('#param-search').addEventListener('input', renderParams);
$('#param-group').addEventListener('change', renderParams);
$('#param-refresh').addEventListener('click', async () => { $('#param-count').textContent = 'loading…'; await api('/api/params/refresh', {}); await loadParams(); });
$('#param-save').addEventListener('click', () => api('/api/params/save', {}));
$('#param-reboot').addEventListener('click', () => { if (confirmTwice('param-reboot', $('#param-reboot'), 'reboot')) api('/api/px4/command', { command: 'reboot' }); });
$('#param-meta-fetch').addEventListener('click', async () => {
  $('#param-count').textContent = 'downloading descriptions via MAVLink FTP…';
  try { const r = await api('/api/params/meta/fetch', {}); await loadMeta(); logLine(`[ui] ${r.count} parameter descriptions from ${r.source}`); }
  catch (e) { logLine('[ui] ' + e.message); }
  await loadParams();
});

// ============================================================ flight / sim
$$('#btn-land').forEach(b => b.addEventListener('click', async () => {
  try {
    if (b.dataset.mode === 'land' && nativeModuleActive()) {
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
$('#btn-pause').addEventListener('click', async () => { const r = await api('/api/sim/pause', {}); $('#btn-pause').textContent = r.paused ? 'Resume' : 'Pause'; });
const CAM_MODES = ['static', 'track', 'follow'], CAM_LABELS = { static: 'Static', track: 'Track', follow: 'Follow' };
let camMode = 'static';
$('#btn-follow').addEventListener('click', (e) => {
  camMode = CAM_MODES[(CAM_MODES.indexOf(camMode) + 1) % CAM_MODES.length];
  e.target.textContent = CAM_LABELS[camMode];
  e.target.classList.toggle('on', camMode !== 'static');
  scene.setCameraMode(camMode);
});

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
  const none = !s.conn_mode;
  // One readout for "who is flying": the link, the mode and whether it is armed.
  const where = none ? 'no link' : s.flashing ? 'flashing' : (s.mode === 'hitl' ? 'Pixhawk' : 'SITL');
  let state;
  if (s.resetting > 0) state = `resetting ${Math.ceil(s.resetting)} s`;
  else if (s.paused) state = 'paused';
  else if (!s.connected) state = none ? 'open Connect' : (s.px4_running ? 'starting' : 'waiting');
  else state = (s.mode_name || '—') + (s.armed ? ' · armed' : '');
  $('#ro-fc-v').textContent = `${where} · ${state}`;
  $('#ro-fc').classList.toggle('warn', !s.connected && !none);
  $('#ro-fc').classList.toggle('crit', !!s.conn_error);
  $('#ro-fc .dot').classList.toggle('on', !!s.connected);
  $('#ro-fc').title = none ? 'Not connected. Click to open Connect.'
    : `${s.address || 'SITL'}${s.mode === 'hitl' && !s.hil_enabled ? ' · HIL OFF (set SYS_HITL=1)' : ''}`;
  if (!s.flashing && s.mode !== 'hitl' && s.px4_ports && s.px4_ports.length) {
    $('#ro-fc-v').textContent = `Pixhawk on ${s.px4_ports[0].replace('/dev/', '')} · connect`;
    $('#ro-fc').classList.add('warn');
  }
  if (s.params_session !== undefined && s.params_session !== paramsSession) {   // board (re)connected or reflashed: forget the old list
    paramsSession = s.params_session; paramsLoaded = false; params = {};
  }
  if (s.params_loaded && s.param_count && s.params_loaded >= s.param_count && !paramsLoaded && (hardwareMotors() || rcVisible() || $('#tab-px4').classList.contains('active'))) ensureParams();
  if (rcVisible()) renderModeButtons();
  updateFooter();
}
const effectiveMax = (r) => (r.enabled === false ? 0 : r.max_thrust);
function applyState(st) {
  if (replayT == null) scene.updateState(st);   // while scrubbing, the view belongs to the recording
  $('#btn-pause').textContent = st.paused ? 'Resume' : 'Pause';
  $('#btn-pause').classList.toggle('armed', !!st.paused);

  const f = st.forces || {};
  const weight = airframe && airframe.mass ? (+airframe.mass.mass || 0) * 9.80665 : 0;

  // Power: what the motors are drawing, and how much of what they can draw. Every field is written with a
  // fixed number of digits into a fixed-width box, so nothing on this bar moves as the numbers change.
  {
    const w = +f.power || 0;
    const cap = airframe ? airframe.rotors.reduce((a, r) => a + (r.enabled === false ? 0 : (+r.max_power_w || 0)), 0) : 0;
    $('#ro-power').textContent = cap > 0 ? `${pad(w, 0, 5)} W  ${pad(100 * w / cap, 0, 3)}%` : `${pad(w, 0, 5)} W`;
  }
  {
    const cap = +((airframe && airframe.design && airframe.design.battery_wh) || 0);
    const el = $('#ro-batt'), v = $('#ro-batt-v');
    if (cap > 0) {
      const used = +st.energy_wh || 0, left = Math.max(0, cap - used), pct = 100 * left / cap, pw = +st.power_avg_w || 0;
      const mins = pw > 5 ? (left / pw) * 60 : null;
      v.textContent = `${pad(pct, 0, 3)}%  ${mins == null ? ' — ' : pad(mins, 0, 3) + ' min'}`;
      el.title = `${left.toFixed(1)} of ${cap.toFixed(0)} Wh left · ${pw.toFixed(0)} W average. Click to set the capacity`;
      el.classList.toggle('warn', pct <= 25 && pct > 10);
      el.classList.toggle('crit', pct <= 10);
    } else { v.textContent = '  —'; el.title = 'Set the battery capacity (Wh) on the Geometry tab'; el.classList.remove('warn', 'crit'); }
  }
  {
    const hasWing = airframe && (airframe.wings || []).some(w => w.enabled !== false);
    const el = $('#ro-lift');
    el.textContent = hasWing && f.lift != null && weight > 0
      ? `${pad(+f.lift, 0, 4)} N  ${pad(100 * f.lift / weight, 0, 3)}%` : '   —';
    el.parentElement.classList.toggle('warn', !!f.stalled);
    el.parentElement.title = 'lift from the wings, and as a share of the weight' + (f.stalled ? ' · a wing is STALLED' : '');
  }
  $('#ro-speed').textContent = `${pad(+f.airspeed || 0, 1, 4)} m/s`;
  $('#ro-alt').textContent = `${pad(-st.pos[2], 2, 5)} m`;
  {
    const w = st.wind || [0, 0, 0];
    const sp = Math.hypot(w[0], w[1], w[2]);
    const from = sp > 0.05 ? ((Math.atan2(-w[1], -w[0]) * 180 / Math.PI) + 360) % 360 : null;
    $('#ro-wind').textContent = sp < 0.05 ? ` 0.0 m/s` : `${pad(sp, 1, 4)} m/s ${pad(from, 0, 3)}°`;
  }
  if (airframe) updateMotorSliders(st);
}
const logPre = $('#log-pre');
function logLine(s) { logPre.textContent += s + '\n'; if (logPre.textContent.length > 40000) logPre.textContent = logPre.textContent.slice(-30000); const b = $('#log-body'); b.scrollTop = b.scrollHeight; }
$('#log-toggle').addEventListener('click', (e) => { const c = $('#log').classList.toggle('collapsed'); e.target.textContent = c ? 'Show' : 'Hide'; });

// ============================================================ the log window: terminal, graphs, replay
// Two views of one flight. The terminal is what the app and PX4 said; the graph is what the aircraft did.
// The server records every run at 20 Hz with the airframe and the parameters that produced it, so a flight
// can be scrubbed afterwards — the 3D view follows the slider — kept, named and loaded back. The graph
// itself is drawn from a buffer this page fills from the live state, so it is there while the sim runs.
const SERIES = [
  { key: 'power', label: 'power', unit: 'W', c: '#f59e0b' },
  { key: 'battery', label: 'battery', unit: '%', c: '#22c55e' },
  { key: 'lift', label: 'wing lift', unit: 'N', c: '#38bdf8' },
  { key: 'thrust', label: 'thrust', unit: 'N', c: '#a78bfa' },
  { key: 'airspeed', label: 'airspeed', unit: 'm/s', c: '#06b6d4' },
  { key: 'speed', label: 'ground speed', unit: 'm/s', c: '#0ea5e9' },
  { key: 'height', label: 'height', unit: 'm', c: '#ef4444' },
  { key: 'wind', label: 'wind', unit: 'm/s', c: '#94a3b8' },
  { key: 'roll', label: 'roll', unit: '°', c: '#f472b6' },
  { key: 'pitch', label: 'pitch', unit: '°', c: '#fb923c' },
  { key: 'yaw', label: 'yaw', unit: '°', c: '#c084fc' },
  { key: 'p', label: 'roll rate', unit: '°/s', c: '#fda4af' },
  { key: 'q', label: 'pitch rate', unit: '°/s', c: '#fcd34d' },
  { key: 'r', label: 'yaw rate', unit: '°/s', c: '#d8b4fe' },
  { key: 'n', label: 'north', unit: 'm', c: '#4ade80' },
  { key: 'e', label: 'east', unit: 'm', c: '#2dd4bf' },
  { key: 'vd', label: 'climb rate', unit: 'm/s', c: '#60a5fa' },
  { key: 'armed', label: 'armed', unit: '', c: '#64748b' },
];
// The columns the recorder writes on the server (sim/recorder.py). The live graph is that recording, polled
// while it is on screen, not a second copy kept here: what is plotted is exactly what gets saved.
const LIVE_COLS = ['t', 'n', 'e', 'd', 'vn', 've', 'vd', 'qw', 'qx', 'qy', 'qz', 'roll', 'pitch', 'yaw',
  'p', 'q', 'r', 'height', 'speed', 'airspeed', 'thrust', 'power', 'lift', 'battery', 'wind', 'armed', 'on_ground'];
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
    replayT = null;
    if (again) await pollLive(false);                 // refill now, so the plot never blinks empty
    return;
  }
  liveRun.columns = j.columns || LIVE_COLS;
  if (j.rows && j.rows.length) liveRun.rows = liveRun.rows.concat(j.rows);
}

const runData = () => shownRun || liveRun;
function rowAt(data, t) {
  const rows = data.rows;
  if (!rows.length) return -1;
  if (t == null) return rows.length - 1;
  let lo = 0, hi = rows.length - 1;
  while (lo < hi) { const mid = (lo + hi) >> 1; if (rows[mid][0] < t) lo = mid + 1; else hi = mid; }
  return lo;
}

function drawGraph() {
  if (!graphOn) return;
  const host = $('#graph-plot'), data = runData(), rows = data.rows || [];
  const cols = data.columns || LIVE_COLS;
  const W = Math.max(120, host.clientWidth), H = Math.max(60, host.clientHeight);
  const T = 6, B = 14, L = 2, R = 2, ph = H - T - B, pw = W - L - R;
  const cursor = rowAt(data, replayT);
  const chips = {};
  $$('#graph-body [data-v]').forEach(el => (chips[el.dataset.v] = chips[el.dataset.v] || []).push(el));
  const show = (key, text) => (chips[key] || []).forEach(el => el.textContent = text);
  if (rows.length < 2) {
    host.innerHTML = `<svg viewBox="0 0 ${W} ${H}"><text x="${W / 2}" y="${H / 2}" text-anchor="middle">`
      + `${shownRun ? 'this run has no samples' : 'nothing recorded yet — start the simulation'}</text></svg>`;
    Object.keys(chips).forEach(k => show(k, ''));
    $('#scrub-label').textContent = '—';               // never a time that belongs to a plot that is not there
    return;
  }
  const t0 = rows[0][0], t1 = rows[rows.length - 1][0], span = Math.max(1e-6, t1 - t0);
  const x = (t) => L + pw * (t - t0) / span;
  const stride = Math.max(1, Math.ceil(rows.length / Math.max(200, pw)));

  let g = '';
  for (let i = 0; i <= 4; i++) { const y = T + ph * i / 4; g += `<line class="grid" x1="${L}" y1="${y.toFixed(1)}" x2="${W - R}" y2="${y.toFixed(1)}"/>`; }
  const step = [0.5, 1, 2, 5, 10, 20, 30, 60, 120, 300, 600].find(s => span / s <= 8) || Math.ceil(span / 8);
  for (let t = Math.ceil(t0 / step) * step; t <= t1; t += step) {
    g += `<line class="grid" x1="${x(t).toFixed(1)}" y1="${T}" x2="${x(t).toFixed(1)}" y2="${T + ph}"/>`
      + `<text x="${x(t).toFixed(1)}" y="${H - 3}" text-anchor="middle">${t.toFixed(step < 1 ? 1 : 0)}s</text>`;
  }

  let lines = '';
  for (const s of SERIES) {
    const ci = cols.indexOf(s.key);
    if (ci < 0) { show(s.key, ''); continue; }
    const v = rows[cursor][ci];
    show(s.key, (Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 10 ? v.toFixed(1) : v.toFixed(2)) + (s.unit ? ' ' + s.unit : ''));
    if (!picked.has(s.key)) continue;
    // Each trace gets its own scale, since they share no unit. The scale includes zero unless the trace
    // actually moves through a good part of its own range: without that, a quantity sitting still at 0.35 m
    // with a millimetre of noise would be drawn as a full-height sawtooth and read as violent motion.
    let lo = Infinity, hi = -Infinity;
    for (const r of rows) { const v = r[ci]; if (v < lo) lo = v; if (v > hi) hi = v; }
    const z0 = Math.min(lo, 0), z1 = Math.max(hi, 0);
    if ((hi - lo) <= 0.25 * (z1 - z0)) { lo = z0; hi = z1; }
    const pad2 = (hi - lo) < 1e-9 ? 1 : (hi - lo) * 0.08;
    lo -= pad2; hi += pad2;
    const y = (v) => T + ph * (1 - (v - lo) / (hi - lo));
    let d = '';
    for (let i = 0; i < rows.length; i += stride) d += (d ? 'L' : 'M') + x(rows[i][0]).toFixed(1) + ' ' + y(rows[i][ci]).toFixed(1);
    const last = rows[rows.length - 1];
    d += 'L' + x(last[0]).toFixed(1) + ' ' + y(last[ci]).toFixed(1);
    lines += `<path class="ln" d="${d}" stroke="${s.c}"/>`;
  }
  const cx = x(rows[cursor][0]).toFixed(1);
  const cur = replayT == null ? '' : `<line class="cursor" x1="${cx}" y1="${T}" x2="${cx}" y2="${T + ph}"/>`;
  host.innerHTML = `<svg viewBox="0 0 ${W} ${H}">${g}${lines}${cur}</svg>`;
  $('#scrub-label').textContent = `${pad(rows[cursor][0], 1, 6)} / ${t1.toFixed(1)} s`
    + (replayT == null ? ' · live' : ' · scrubbed, the 3D view is at this instant');
  $('#scrub-resume').hidden = replayT == null;
}

// The traces are chosen from one dropdown of checkboxes; the ones that are on are listed beside it with
// their value at the cursor, so the row stays a single line however many are plotted.
function buildPicker() {
  const pop = $('#series-pop');
  pop.innerHTML = SERIES.map(s => `<label data-key="${s.key}" style="--c:${s.c}">`
    + `<input type="checkbox" ${picked.has(s.key) ? 'checked' : ''}><i class="sw"></i>`
    + `<span class="nm">${esc(s.label)}</span><b class="num" data-v="${esc(s.key)}"></b></label>`).join('');
  pop.querySelectorAll('label').forEach(lab => {
    const key = lab.dataset.key;
    lab.querySelector('input').addEventListener('change', (e) => {
      if (e.target.checked) picked.add(key); else picked.delete(key);
      saveGraphPrefs(); buildLegend(); drawGraph();
    });
  });
  buildLegend();
}
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
function applyReplay() {
  const data = runData(), rows = data.rows || [], cols = data.columns || LIVE_COLS;
  if (!rows.length) return;
  const r = rows[rowAt(data, replayT)];
  const i = (k) => cols.indexOf(k);
  if (i('n') < 0 || i('qw') < 0) return;
  scene.updateState({ pos: [r[i('n')], r[i('e')], r[i('d')]], q: [r[i('qw')], r[i('qx')], r[i('qy')], r[i('qz')]],
                      forces: {}, rotors: [] });
}
// Dragging anywhere on the plot moves the aircraft to that instant. The simulation stops while you are in
// the recording — a 3D view that is partly a replay and partly live would be a lie — and Resume starts it again.
let pausedByScrub = false;
function scrubTo(clientX) {
  const data = runData(), rows = data.rows || [];
  if (rows.length < 2) return;
  const box = $('#graph-plot').getBoundingClientRect();
  const f = Math.max(0, Math.min(1, (clientX - box.left - 2) / Math.max(1, box.width - 4)));
  const t0 = rows[0][0], t1 = rows[rows.length - 1][0];
  replayT = t0 + f * (t1 - t0);
  applyReplay(); drawGraph();
}
$('#graph-plot').addEventListener('pointerdown', async (e) => {
  const rows = (runData().rows || []);
  if (rows.length < 2) return;
  e.preventDefault();
  $('#graph-plot').setPointerCapture(e.pointerId);
  if (!shownRun && !status.paused && !pausedByScrub) {
    pausedByScrub = true;
    api('/api/sim/pause', { paused: true }).catch(err => logLine('[ui] ' + err.message));
  }
  scrubTo(e.clientX);
});
$('#graph-plot').addEventListener('pointermove', (e) => { if (e.buttons & 1) scrubTo(e.clientX); });
$('#scrub-resume').addEventListener('click', async () => {
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
    if (log.classList.contains('collapsed')) { log.classList.remove('collapsed'); $('#log-toggle').textContent = 'Hide'; }
    if (log.getBoundingClientRect().height < 260) log.style.height = '300px';
    loadRunList();
    drawGraph();
    clearInterval(graphTimer);
    const tick = async () => {
      if (!graphOn) return;
      // Pause was released somewhere else (the button over the 3D view): the recording is no longer where
      // the aircraft is, so go back to following it.
      if (replayT != null && pausedByScrub && !status.paused) { replayT = null; pausedByScrub = false; }
      if (!shownRun) await pollLive();
      drawGraph();
    };
    tick();
    graphTimer = setInterval(tick, 250);
  } else {
    clearInterval(graphTimer); graphTimer = null;
  }
  try { localStorage.setItem('airframe-logview', on ? 'graph' : 'log'); } catch { }
}
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
  pick.innerHTML = '<option value="">Live</option>'
    + (j.runs || []).map(r => `<option value="${esc(r.id)}">${esc(runLabel(r))}</option>`).join('');
  pick.value = (j.runs || []).some(r => r.id === want) ? want : '';
  if (pick.value !== want) selectRun('');
  runButtons();
}
function runButtons() {
  const has = !!$('#run-pick').value;
  ['#run-rename', '#run-delete', '#run-load'].forEach(s => $(s).hidden = !has);
  $('#run-save').hidden = has;
}
async function selectRun(id) {
  if (!id) {
    shownRun = null; replayT = null;
  } else {
    try { shownRun = await api('/api/runs/' + encodeURIComponent(id)); }
    catch (err) { logLine('[runs] ' + err.message); shownRun = null; }
    replayT = shownRun && shownRun.rows.length ? shownRun.rows[0][0] : null;
    applyReplay();
  }
  runButtons();
  drawGraph();
}
$('#run-pick').addEventListener('change', (e) => selectRun(e.target.value));
$('#run-save').addEventListener('click', async (e) => {
  e.target.disabled = true;
  try {
    const r = await api('/api/runs/save', { name: '' });
    if (r.ok) { await loadRunList(r.id); await selectRun(r.id); logLine(`[runs] kept ${r.id} (${(+r.duration).toFixed(1)} s)`); }
    else logLine('[runs] nothing to keep: ' + (r.error || ''));
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
$('#btn-theme').addEventListener('click', () => {
  const dark = document.documentElement.dataset.theme !== 'dark';
  document.documentElement.dataset.theme = dark ? 'dark' : 'light';
  try { localStorage.setItem('airframe-theme', dark ? 'dark' : 'light'); } catch { }
  scene.setTheme(dark ? 'dark' : 'light');
});

function connectWs() {
  const ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws');
  joyWs = ws;
  ws.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.type === 'airframe') { setAirframe(m.airframe); }
    if (m.type === 'state') { lastRc = m.rc || {}; lastManual = m.manual || {}; applyState(m.state); if (m.status) applyStatus(m.status); if (m.log) m.log.forEach(l => logLine(l[1])); if (rcVisible()) renderRcLive(); }
  };
  ws.onclose = () => { $('#ro-fc-v').textContent = 'disconnected'; logLine('[ui] websocket closed; reconnecting'); setTimeout(connectWs, 800); };
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
    if (log.classList.contains('collapsed')) { log.classList.remove('collapsed'); $('#log-toggle').textContent = 'Hide'; }
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

function linkNow(c) {
  if (!c || !c.mode) return '';
  if (c.mode === 'sitl') return 'sitl';
  return status.hil_enabled === false ? 'real' : 'hitl';
}
$('#ro-fc').addEventListener('click', async () => { openTab('controller'); });
$('#conn-sitl').addEventListener('click', async () => { await connCall('/api/connection/connect', { mode: 'sitl' }); });
$('#conn-hitl').addEventListener('click', async (e) => {
  if (!lastPorts.length) { $('#conn-error').textContent = 'No flight controller on USB. Plug it in and click Rescan.'; return; }
  if (status.hil_enabled === false && !confirmTwice('return-hitl', e.currentTarget, 'put the board back into HITL')) return;
  if (status.hil_enabled === false) { const r = await api('/api/connection/enable_hitl', {}); $('#real-flight-result').textContent = r.ok ? 'Returning to HITL…' : r.error; return; }
  await connectHitl((lastPorts.find(p => p.likely_px4) || lastPorts[0]).device);
});
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
  finally { b.disabled = false; b.textContent = 'Real flight'; refreshConnection(); }
});
$('#conn-rescan').addEventListener('click', refreshConnection);
$('#conn-disconnect').addEventListener('click', async () => { await connCall('/api/connection/disconnect', {}); });
$('#conn-reboot').addEventListener('click', async (e) => { if (confirmTwice('conn-reboot', e.currentTarget, 'reboot the board')) { await api('/api/px4/command', { command: 'reboot' }); setTimeout(refreshConnection, 3000); } });
$('#conn-hw-ok').addEventListener('click', async (e) => {
  if (!confirmTwice('enable_hw', e.currentTarget, 'allow motors on the real aircraft')) return;
  await api('/api/params/set', { name: 'NLF_HW_OK', value: 1 });
  paramsLoaded = false; setTimeout(refreshConnection, 1500);
});
async function connectHitl(serial) {
  await connCall('/api/connection/connect', { mode: 'hitl', serial, baud: +$('#conn-baud').value || 921600 });
}
async function connCall(path, body) {
  $('#conn-error').textContent = '';
  $('#conn-current').innerHTML = '<span class="muted">Working…</span>';
  try { await api(path, body); paramsLoaded = false; } catch (e) { $('#conn-error').textContent = e.message; }
  await refreshConnection();
}
async function refreshConnection() {
  clearTimeout(connTimer);
  if (!$('#tab-controller').classList.contains('active')) return;
  let c;
  try { c = await api('/api/connection'); } catch (e) { $('#conn-error').textContent = e.message; return; }
  const L = c.link, where = linkNow(c);
  $$('#conn-modes button').forEach(b => b.classList.toggle('active', b.dataset.link === where));
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
  lastPorts = c.ports || [];
  $('#conn-ports').innerHTML = lastPorts.map(p =>
    `<div class="conn-row"><div><b>${p.likely_px4 ? 'Pixhawk' : 'Serial device'}</b> <span class="hint">${esc(p.description || '')}</span> <span class="dev">${esc(p.device)}</span></div>
      <button class="pill small ${p.likely_px4 ? 'primary' : ''}" data-dev="${esc(p.device)}">${c.mode === 'hitl' && c.serial === p.device ? 'Reconnect' : 'Connect'}</button></div>`).join('');
  $$('#conn-ports button[data-dev]').forEach(b => b.addEventListener('click', () => connectHitl(b.dataset.dev)));
  // The one interlock worth keeping from the old checklist: the firmware will not run the motors on the
  // real aircraft until this is set, and nothing else on this page can set it.
  $('#conn-hw-ok').hidden = !(where === 'real' && nativeModuleActive() && pv('NLF_HW_OK', 1) === 0);
  connTimer = setTimeout(refreshConnection, 2000);
}

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

// ============================================================ firmware: the custom PX4 modules
let fwLoaded = null, fwDirty = false;
async function refreshFirmware() {
  const sel = $('#fw-file');
  if (sel.options.length <= 1 || sel.value === '') {
    try {
      const r = await api_get('/api/firmware/files');
      sel.innerHTML = (r.files || []).map(f => `<option value="${esc(f)}">${esc(f)}</option>`).join('')
        || '<option value="">no module sources found</option>';
      $('#fw-note').textContent = r.root || '';
      if (sel.options.length) { sel.value = sel.options[0].value; await openFirmwareFile(sel.value); }
    } catch (e) { $('#fw-note').textContent = e.message; }
  }
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
const rcVisible = () => $('#tab-controller').classList.contains('active');
function renderRcLive() {
  const now = performance.now();
  if (now - rcLastRender < 80) return;
  rcLastRender = now;
  renderRcIndicator(); renderRcBoard();
}
setInterval(() => { if (rcVisible()) renderRcLive(); }, 500);

// ============================================================ board vs SITL parameter sync
let syncTimer = null, syncLast = null;
async function refreshSync() {
  clearTimeout(syncTimer);
  const card = $('#sync-card');
  if (!$('#tab-px4').classList.contains('active')) return;
  if (status.conn_mode !== 'hitl') { card.hidden = true; syncTimer = setTimeout(refreshSync, 3000); return; }
  card.hidden = false;
  try {
    const d = await api('/api/px4/sitl_diff');
    if (!d.ok) { $('#sync-text').textContent = d.error; $('#sync-btn').disabled = true; $('#sync-list').textContent = ''; }
    else {
      syncLast = d;
      const when = new Date(d.reference_time * 1000).toLocaleString(undefined, { dateStyle: 'medium', timeStyle: 'short' });
      $('#sync-text').textContent = d.differ.length ? `${d.differ.length} of ${d.compared} parameters differ from the SITL run of ${when}${d.skipped ? ` (${d.skipped} hardware/safety ones left alone)` : ''}` : `identical to the SITL run of ${when}${d.skipped ? ` (${d.skipped} hardware/safety ones left alone)` : ''}`;
      $('#sync-btn').disabled = !d.differ.length || !!status.armed;
      $('#sync-list').textContent = d.differ.slice(0, 40).map(x => `${x.name}: ${x.board} → ${x.sitl}`).join('   ') + (d.differ.length > 40 ? `   … +${d.differ.length - 40}` : '');
    }
  } catch (e) { $('#sync-text').textContent = e.message; }
  syncTimer = setTimeout(refreshSync, 5000);
}
$('#sync-btn').addEventListener('click', async () => {
  const b = $('#sync-btn');
  if (!confirmTwice('sync-sitl', b, `write ${syncLast ? syncLast.differ.length : ''} parameters`)) return;
  b.disabled = true; b.textContent = 'Writing…';
  try { const r = await api('/api/px4/sync_sitl', {}); $('#sync-text').textContent = r.ok ? `${r.written} parameters written in ${r.seconds} s` : `failed: ${(r.failed || []).join(', ') || r.error}`; paramsLoaded = false; ensureParams(); }
  catch (e) { $('#sync-text').textContent = e.message; }
  b.textContent = 'Sync board to SITL'; b.dataset.label = b.textContent; refreshSync();
});
$$('.tabs button').forEach(b => b.addEventListener('click', () => { if (b.dataset.tab === 'px4') refreshSync(); }));

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
const BATCH_ACTIONS = {
  scenario: { label: 'Run a scenario', fields: [{ k: 'scenario', type: 'select', options: () => (scenariosCache || []).map(s => s.file.replace(/\.json$/, '')) }] },
  param:    { label: 'Set a PX4 parameter', fields: [{ k: 'name', type: 'text', placeholder: 'MC_PITCHRATE_P' }, { k: 'value', type: 'number' }] },
  geometry: { label: 'Set a geometry value', fields: [{ k: 'path', type: 'text', placeholder: 'rotors[*].tilt_deg' }, { k: 'value', type: 'number' }] },
  wind:     { label: 'Set the wind', fields: [{ k: 'north', type: 'number' }, { k: 'east', type: 'number' }, { k: 'down', type: 'number' }] },
  reset:    { label: 'Reset the aircraft', fields: [] },
};
let batchList = [];
try { batchList = JSON.parse(localStorage.getItem('airframe-batch') || '[]'); } catch { }
const saveBatch = () => { try { localStorage.setItem('airframe-batch', JSON.stringify(batchList)); } catch { } };

function renderBatchFields() {
  const kind = $('#ba-kind').value;
  const spec = BATCH_ACTIONS[kind];
  $('#ba-fields').innerHTML = (spec.fields || []).map(f => {
    if (f.type === 'select') return `<label style="flex:1">${f.k}<select data-f="${f.k}">${(f.options() || []).map(o => `<option>${esc(o)}</option>`).join('')}</select></label>`;
    return `<label style="flex:1">${f.k}<input data-f="${f.k}" type="${f.type}" ${f.type === 'number' ? 'step="any" value="0"' : ''} placeholder="${esc(f.placeholder || '')}"></label>`;
  }).join('') || '<span class="hint">no settings</span>';
}
function renderBatchList() {
  const el = $('#ba-list');
  if (!batchList.length) { el.innerHTML = '<div class="hint">No actions yet. Add one above; they run in order.</div>'; return; }
  el.innerHTML = batchList.map((a, i) => {
    const bits = Object.entries(a).filter(([k]) => k !== 'kind').map(([k, v]) => `${k} ${v}`).join(', ');
    return `<div class="conn-row"><div><b>${i + 1}.</b> ${esc(BATCH_ACTIONS[a.kind].label)}${bits ? ' <span class="hint">' + esc(bits) + '</span>' : ''}</div>
      <div class="row tight"><button class="pill small" data-up="${i}" ${i ? '' : 'disabled'}>↑</button>
      <button class="pill small" data-del="${i}">✕</button></div></div>`;
  }).join('');
  $$('#ba-list [data-del]').forEach(b => b.addEventListener('click', () => { batchList.splice(+b.dataset.del, 1); saveBatch(); renderBatchList(); }));
  $$('#ba-list [data-up]').forEach(b => b.addEventListener('click', () => {
    const i = +b.dataset.up; [batchList[i - 1], batchList[i]] = [batchList[i], batchList[i - 1]]; saveBatch(); renderBatchList();
  }));
}
$('#ba-kind').addEventListener('change', renderBatchFields);
$('#ba-add').addEventListener('click', () => {
  const kind = $('#ba-kind').value;
  const action = { kind };
  $$('#ba-fields [data-f]').forEach(el => { action[el.dataset.f] = el.type === 'number' ? (parseFloat(el.value) || 0) : el.value.trim(); });
  batchList.push(action); saveBatch(); renderBatchList();
});
$('#ba-clear').addEventListener('click', () => { batchList = []; saveBatch(); renderBatchList(); });

async function runBatchAction(a) {
  if (a.kind === 'param') return api('/api/params/set', { name: a.name, value: a.value });
  if (a.kind === 'geometry') return api('/api/airframe/path', { path: a.path, value: a.value });
  if (a.kind === 'wind') return api('/api/sim/wind', { north: a.north, east: a.east, down: a.down });
  if (a.kind === 'reset') return api('/api/sim/reset', {});
  if (a.kind === 'scenario') {
    await api('/api/airframe', { airframe });                      // the job copies the live airframe
    const r = await api('/api/batch/run', { scenario: a.scenario, options: {} });
    logLine(`[batch] started ${r.id} (${a.scenario})`);
    return r;
  }
}
$('#ba-run').addEventListener('click', async e => {
  const b = e.currentTarget, label = b.textContent;
  if (!batchList.length) { logLine('[batch] the list is empty'); return; }
  b.disabled = true;
  const repeats = Math.max(1, +$('#ba-repeat').value || 1);
  try {
    for (let pass = 1; pass <= repeats; pass++) {
      for (let i = 0; i < batchList.length; i++) {
        b.textContent = `${pass}/${repeats} · step ${i + 1}/${batchList.length}`;
        try { await runBatchAction(batchList[i]); }
        catch (err) { logLine(`[batch] step ${i + 1} failed: ${err.message}`); }
      }
    }
    logLine(`[batch] finished ${repeats} pass${repeats === 1 ? '' : 'es'} of ${batchList.length} action(s)`);
  } finally { b.disabled = false; b.textContent = label; pollJobs(); }
});

let batchTimer = null, scenariosCache = null, lastJobs = null;
const jobsOpen = new Set();
const mnum = (v, d = 2) => (v == null || !isFinite(v)) ? '—' : (+v).toFixed(d);
async function refreshBatch() {
  clearTimeout(batchTimer);
  if (!$('#tab-batch').classList.contains('active')) return;
  if (!scenariosCache) {
    try { scenariosCache = (await api('/api/scenarios')).scenarios || []; } catch { scenariosCache = []; }
  }
  renderBatchFields(); renderBatchList(); pollJobs();
}
async function pollJobs() {
  try { const r = await api('/api/batch/jobs'); lastJobs = r; renderJobs(r); } catch (e) { }
  clearTimeout(batchTimer);
  if ($('#tab-batch').classList.contains('active')) batchTimer = setTimeout(pollJobs, 1500);
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
      <td class="metrics">${j.running ? `<span class="hint">${esc((j.log || []).slice(-1)[0] || 'starting…')}</span>` : phaseSummary(res) + fails + (res.animation_path ? `<a class="pill small" href="/api/batch/jobs/${encodeURIComponent(j.id)}/animation" data-animation-download>Download Blender animation</a>` : '')}</td></tr>` + detail;
  }).join('');
  el.innerHTML = `<table class="grid jobs"><thead><tr><th>Job</th><th>Scenario</th><th>Status</th><th title="simulated time">Sim</th><th title="wall-clock time">Wall</th><th title="real-time factor">RTF</th><th>Metrics</th></tr></thead><tbody>${rows}</tbody></table>`;
  $$('#batch-jobs [data-animation-download]').forEach(a => a.addEventListener('click', e => { e.preventDefault(); e.stopPropagation(); downloadAnimation(a.href, a); }));
  $$('#batch-jobs tr[data-job]').forEach(tr => tr.addEventListener('click', () => { const id = tr.dataset.job; if (jobsOpen.has(id)) jobsOpen.delete(id); else jobsOpen.add(id); if (lastJobs) renderJobs(lastJobs); }));
}

const liveBadge = document.createElement('div');
liveBadge.style.cssText = 'position:absolute;top:12px;left:12px;background:#fff;color:#222;padding:10px 14px;border-radius:12px;z-index:4;display:none;box-shadow:0 2px 12px #0002';
const liveLabel = document.createElement('span');
const liveStop = document.createElement('button');
liveStop.textContent = 'Stop test'; liveStop.style.marginLeft = '12px';
liveStop.onclick = async () => { try { await api('/api/sim/scenario/stop', {}); } catch(e) { liveLabel.textContent = e.message; } };
const liveAnimation = document.createElement('a');
liveAnimation.textContent = 'Download Blender animation';
liveAnimation.className = 'pill small';
liveAnimation.href = '/api/sim/scenario/animation';
liveAnimation.onclick = e => { e.preventDefault(); downloadAnimation(liveAnimation.href, liveAnimation); };
liveAnimation.style.marginLeft = '12px';
liveBadge.append(liveLabel, liveStop, liveAnimation); $('#viewport').append(liveBadge);
let livePolling = false;
setInterval(async () => {
  if (livePolling) return; livePolling = true;
  try {
    const r = await api('/api/sim/scenario');
    liveBadge.style.display = r.phase ? 'block' : 'none';
    liveLabel.textContent = r.phase ? `${r.running ? 'LIVE TEST' : r.status.toUpperCase()} · ${r.phase.name || r.phase.type}` : '';
    liveStop.hidden = !r.running;
  } catch {} finally { livePolling = false; }
}, 1000);

$('#af-takeoff').addEventListener('change', async () => {
  const inp = $('#af-takeoff');
  const value = Number(inp.value);
  try {
    if (status.armed || !status.ctl_connected || !Number.isFinite(value)) throw new Error('Enter a finite takeoff pitch while connected and disarmed');
    if (status.mode === 'sitl') {
      airframe.design ||= {};
      airframe.design.nose_lift = { ...(airframe.design.nose_lift || {}), target_pitch_deg: value };
      $('#nl-target').value = value;
      pushAirframe(true);
      updateFooter();
      return;
    }
    const saved = await api('/api/params/set', { name: 'NLF_TARGET', value });
    if (!saved.ok || Math.abs(Number(saved.value) - value) > 0.01) throw new Error(saved.error || 'PX4 did not confirm the takeoff pitch');
    params.NLF_TARGET = { ...(params.NLF_TARGET || {}), value };
    airframe.px4_overrides = { ...(airframe.px4_overrides || {}), NLF_TARGET: value };
    logLine('[ui] Takeoff pitch saved: ' + value + '°');
  } catch (e) {
    logLine('[ui] ' + e.message);
    inp.value = status.mode === 'sitl'
      ? (airframe.design?.nose_lift?.target_pitch_deg ?? airframe.hover_pitch_deg ?? 0)
      : ((params.NLF_TARGET || {}).value ?? airframe.px4_overrides?.NLF_TARGET ?? 25);
  }
});

['x', 'y', 'z'].forEach((axis, i) => bindNumber('imu-' + axis, v => {
  airframe.design ||= {};
  airframe.design.pixhawk_position ||= [...airframe.mass.cg];
  airframe.design.pixhawk_position[i] = v;
}));

$('#motor-test-stop').addEventListener('click', () => { stopMotorTest(); $('#motor-enable').checked=false; });


$('#handover-mode').addEventListener('change', async e => {
  if (status.armed) return;
  const value = Number(e.target.value);
  try {
    await api('/api/airframe/override', { name: 'NLF_PILOT', value });
    airframe.px4_overrides.NLF_PILOT = value;
    markDirty();
    logLine('[ui] Handover after nose lift: ' + e.target.selectedOptions[0].textContent + '. Update PX4 while disarmed to apply.');
  } catch (err) { logLine('[ui] ' + err.message); }
});
// Indoor sensor set: no GPS fusion, barometer height. Position modes vanish, exactly as on the real aircraft indoors.
$('#staged-rc-takeoff').addEventListener('change', async e => {
  if (status.armed) return;
  const value=e.target.checked?1:0;
  try {
    await api('/api/airframe/override', {name:'NLF_RC_THR',value});
    airframe.px4_overrides.NLF_RC_THR=value;
    markDirty();
    logLine('[ui] Staged RC takeoff '+(value?'selected':'disabled')+'. Update PX4 while disarmed to apply. No board change made yet.');
  } catch(err) { e.target.checked=!value; logLine('[ui] '+err.message); }
});
