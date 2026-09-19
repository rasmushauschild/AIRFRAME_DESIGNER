let lastPrearmReportRequest = 0;
let prearmReportRequestPending = false;
import { createScene } from '/static/scene.js';

const $ = (s) => document.querySelector(s);
const $$ = (s) => Array.from(document.querySelectorAll(s));
const api = async (path, body, method) => {
  const r = await fetch(path, { method: method || (body === undefined ? 'GET' : 'POST'),
    headers: { 'Content-Type': 'application/json' }, body: body === undefined ? undefined : JSON.stringify(body) });
  let j = null; try { j = await r.json(); } catch { }
  if (!r.ok) throw new Error((j && j.error) || r.statusText);
  return j;
};
const fmt = (v, d = 3) => (typeof v === 'number' ? v.toFixed(d) : String(v));
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
  if (b.dataset.tab === 'connect') refreshConnection();
  if (b.dataset.tab === 'design') refreshDesign();
  if (b.dataset.tab === 'batch') refreshBatch();
}));
function openTab(name) { $$('.tabs button').find(b => b.dataset.tab === name)?.click(); }

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
      if ($('#tab-design').classList.contains('active')) refreshDesign();
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
  $('#title-name').textContent = af.name;
  fillMassCard();
  ['bx', 'by', 'bz'].forEach((k, i) => $('#af-' + k).value = af.body.size[i]);
  ['dragx', 'dragy', 'dragz'].forEach((k, i) => $('#af-' + k).value = af.body.drag_quadratic[i]);
  ['adragx', 'adragy', 'adragz'].forEach((k, i) => $('#af-' + k).value = af.body.drag_angular[i]);
  ['dcx', 'dcy', 'dcz'].forEach((k, i) => $('#af-' + k).value = af.body.drag_center[i]);
  designGroups = null;
  $('#d-speed').value = airframe.design.cruise_speed_kmh ?? 50;
  renderRotorTable();
  renderWingTable();
  renderLegTable();
  renderMotorSliders();
  fillNoseLiftCard();
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
  $('#af-hover').value = airframe.hover_pitch_deg || 0;
  $('#af-landed').value = airframe.landed_pitch_deg || 0;
}
const KIND_DEFAULTS = { prop: { km: 0.05, tau: 0.04, diameter: 0.25, thrust_exponent: 2, ram_drag: false },
                        ducted: { km: 0.01, tau: 0.12, diameter: 0.12, thrust_exponent: 2, ram_drag: true } };
function fillMotorCard() {
  const r = airframe.rotors[selected >= 0 ? selected : 0]; if (!r) return;
  $('#m-kind').value = r.kind || 'prop'; $('#m-tmax').value = r.max_thrust; $('#m-tau').value = r.tau;
  $('#m-km').value = Math.abs(r.km); $('#m-dia').value = r.diameter; $('#m-exp').value = r.thrust_exponent; $('#m-ram').checked = !!r.ram_drag;
  $('#m-turnloss').value = Math.round((r.turn_loss ?? 0.1) * 100);
}
function applyMotorCard(kindChanged) {
  const kind = $('#m-kind').value;
  if (kindChanged) { const d = KIND_DEFAULTS[kind]; $('#m-tau').value = d.tau; $('#m-km').value = d.km; $('#m-dia').value = d.diameter; $('#m-exp').value = d.thrust_exponent; $('#m-ram').checked = d.ram_drag; }
  const tmax = +$('#m-tmax').value, tau = +$('#m-tau').value, km = Math.abs(+$('#m-km').value), dia = +$('#m-dia').value, ex = +$('#m-exp').value, ram = $('#m-ram').checked;
  const turnLoss = Math.max(0, +$('#m-turnloss').value) / 100;
  airframe.rotors.forEach(r => { r.kind = kind; r.max_thrust = tmax; r.tau = tau; r.km = (r.km >= 0 ? 1 : -1) * km; r.diameter = dia; r.thrust_exponent = ex; r.ram_drag = ram; r.turn_loss = turnLoss; if (kind !== 'ducted') r.duct_axis = null; });
  setAirframe(airframe); pushAirframe(true);
}
$('#m-kind').addEventListener('change', () => applyMotorCard(true));
['m-tmax', 'm-tau', 'm-km', 'm-dia', 'm-exp', 'm-ram', 'm-turnloss'].forEach(id => $('#' + id).addEventListener('change', () => applyMotorCard(false)));

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
    const key = cadKey(cad);
    if (key !== cadMeshKey) loadCadMesh(key);
  } else {
    $('#cad-filename').textContent = '';
    $('#cad-frame-row').style.display = 'none';
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
  if (!cad || !cad.file) { el.innerHTML = '<div class="hint">No CAD file. Import a STEP file to place its bodies and give them masses.</div>'; $('#cad-totals').innerHTML = ''; $('#cad-summary').textContent = ''; return; }
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
$('#cad-remove-all').addEventListener('click', () => { if (!airframe.cad || !confirm('Remove the CAD file and all its bodies from this airframe?')) return; airframe.cad = null; cadSelected = null; scene.selectCad(null); setAirframe(airframe); pushAirframe(true); });

function bindNumber(id, fn) {
  $('#' + id).addEventListener('change', (e) => { fn(parseFloat(e.target.value) || 0); scene.setAirframe(airframe); pushAirframe(true); });
}
$('#af-name').addEventListener('change', e => { airframe.name = e.target.value; $('#title-name').textContent = airframe.name; pushAirframe(true); });
bindNumber('af-mass', v => airframe.mass.mass = v);
bindNumber('af-cgx', v => airframe.mass.cg[0] = v);
bindNumber('af-cgy', v => airframe.mass.cg[1] = v);
bindNumber('af-cgz', v => airframe.mass.cg[2] = v);
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
$('#af-save').addEventListener('click', async () => {
  const b = $('#af-save');
  try {
    const typed = $('#af-name').value.trim();
    if (typed) { airframe.name = typed; $('#title-name').textContent = typed; }
    const name = airframe.name.trim().toLowerCase().replace(/[^a-z0-9]+/g, '_').replace(/^_+|_+$/g, '') || 'airframe';
    const bad = sanitizeNumbers(airframe);
    if (bad.length) logLine('[ui] empty or invalid number fields set to 0: ' + bad.join(', '));
    await api('/api/airframe', { airframe, keep_state: true });
    const r = await api('/api/airframe/save', { name });
    logLine('[ui] saved ' + r.path);
    b.textContent = 'Saved'; setTimeout(() => b.textContent = 'Save', 1500);
    loadPresetList();
  } catch (e) {
    logLine('[ui] SAVE FAILED: ' + e.message);
    b.textContent = 'Save failed'; b.classList.add('danger'); setTimeout(() => { b.textContent = 'Save'; b.classList.remove('danger'); }, 4000);
    alert('Save failed: ' + e.message + '\n\nThe design is still in the running app; fix the problem and save again.');
  }
});
async function loadPresetList() {
  const r = await api('/api/airframes');
  const sel = $('#af-preset');
  sel.innerHTML = '<option value="">Choose…</option>' +
    r.files.map(f => `<option value="${f}">${f.replace(/\.json$/, '')}</option>`).join('') +
    r.presets.map(p => `<option value="${p}">built-in ${p}</option>`).join('');
}
$('#af-preset').addEventListener('change', async (e) => {
  if (!e.target.value) return;
  try {
    const r = await api('/api/airframe/load', { name: e.target.value });
    selected = -1; setAirframe(r.airframe); showProblems(r.problems); showHover(r.hover);
  } catch (err) { logLine('[ui] load failed: ' + err.message); }
  e.target.value = '';
});

// --- rotor helpers: axis <-> tilt/direction
// Tilt: forward lean of the thrust axis from vertical (negative = backward). Cant: sideways lean, positive = outward
// (away from the centreline; for a rotor on the centreline positive = right). Same convention as the Optimize tab.
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
  el.innerHTML = `<table class="grid"><thead><tr><th>#</th><th title="enabled: takes part in the simulation and the PX4 export">On</th><th>Name</th><th title="position, m, structural frame (FRD like PX4): X forward">X</th><th title="Y right (+)">Y</th><th title="Z DOWN (+): a rotor 10 cm above the reference point has Z = -0.10">Z</th><th title="forward lean of the thrust axis from vertical, degrees; negative = backward">Tilt°</th><th title="sideways lean, degrees; positive = outward from the centreline (right for a rotor on the centreline)">Cant°</th><th title="resulting unit thrust vector = CA_ROTORn_AX / AY / AZ">Axis AX AY AZ</th><th title="ducted fans: the fan along the thrust (jet), or a horizontal fan whose jetfoil bends the jet to the thrust axis (foil)">Duct</th><th>Spin</th><th title="max thrust N">Tmax</th><th title="share of max thrust this rotor needs to hover, as PX4's allocator would solve it">Hover</th><th></th></tr></thead><tbody>${rows || '<tr><td colspan="14" class="hint">No rotors. Add one below.</td></tr>'}</tbody></table>`;
  api('/api/airframe/hover_check').then(showHover).catch(() => { });
  el.querySelectorAll('tr[data-i]').forEach(tr => {
    const i = +tr.dataset.i;
    tr.addEventListener('click', (e) => { if (!['INPUT', 'SELECT', 'OPTION', 'BUTTON'].includes(e.target.tagName) && !e.target.classList.contains('spin') && !e.target.classList.contains('del')) { selected = i; scene.select(i); renderRotorTable(); renderReadout(); } });
    tr.querySelectorAll('input, select').forEach(inp => inp.addEventListener('change', () => applyRow(i, tr)));
    tr.querySelector('.spin').addEventListener('click', () => { airframe.rotors[i].km = -airframe.rotors[i].km; scene.updateRotorNode(i, airframe.rotors[i]); renderRotorTable(); pushAirframe(true); });
    tr.querySelector('.del').addEventListener('click', () => { airframe.rotors.splice(i, 1); selected = -1; setAirframe(airframe); pushAirframe(true); });
  });
}
const axisText = (a) => a.map(v => (v >= 0 ? ' ' : '') + v.toFixed(2)).join(' ');
function rotorRowHtml(i, r) {
  const ccw = r.km >= 0;
  const [tilt, cant] = axisToTilt(r.axis, r.pos[1]);
  return `<tr data-i="${i}" class="${i === selected ? 'selected' : ''} ${r.enabled === false ? 'off' : ''}"><td class="idx">${i + 1}</td>
  <td><input type="checkbox" data-k="on" ${r.enabled === false ? '' : 'checked'}></td>
  <td><input type="text" class="name" data-k="name" value="${esc(r.name)}"></td>
  <td><input type="number" step="0.005" data-k="x" value="${r.pos[0]}"></td>
  <td><input type="number" step="0.005" data-k="y" value="${r.pos[1]}"></td>
  <td><input type="number" step="0.005" data-k="z" value="${r.pos[2]}"></td>
  <td><input type="number" step="1" data-k="tilt" value="${+tilt.toFixed(1)}"></td>
  <td><input type="number" step="1" data-k="cant" value="${+cant.toFixed(1)}"></td>
  <td class="axis" title="CA_ROTOR${i}_AX / AY / AZ">${axisText(r.axis)}</td>
  <td>${r.kind === 'ducted' ? `<select data-k="duct"><option value="jet" ${r.duct_axis ? '' : 'selected'}>jet</option><option value="foil" ${r.duct_axis ? 'selected' : ''}>foil</option></select>` : ''}</td>
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
  const duct = tr.querySelector('select[data-k="duct"]');
  if (duct) r.duct_axis = duct.value === 'foil' ? [1, 0, 0] : null;
  tr.classList.toggle('off', !r.enabled);
  scene.updateRotorNode(i, r);
  renderRotorRow(i);
  renderReadout();
  pushAirframe(true);
}
function renderReadout() {
  const el = $('#rotor-readout');
  if (selected < 0 || !airframe.rotors[selected]) { el.classList.remove('show'); return; }
  const r = airframe.rotors[selected];
  const [tilt, cant] = axisToTilt(r.axis, r.pos[1]);
  el.classList.add('show');
  let foil = '';
  if (r.duct_axis) {
    const a = r.axis, d = r.duct_axis, na = Math.hypot(...a) || 1, nd = Math.hypot(...d) || 1;
    const bend = deg(Math.acos(Math.max(-1, Math.min(1, (a[0] * d[0] + a[1] * d[1] + a[2] * d[2]) / (na * nd)))));
    foil = ` · jetfoil bends ${bend.toFixed(0)}° (${((1 - (r.turn_loss ?? 0.1) * bend / 90) * 100).toFixed(0)}% thrust)`;
  }
  el.innerHTML = `<b>${esc(r.name || 'Motor ' + (selected + 1))}</b> pos [${r.pos.map(v => fmt(v)).join(', ')}] · axis [${r.axis.map(v => fmt(v, 3)).join(', ')}] (tilt ${tilt.toFixed(1)}°, cant ${cant.toFixed(1)}°)${foil} · ${r.km >= 0 ? 'CCW' : 'CW'}${r.enabled === false ? ' · disabled' : ''}`;
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
  airframe.rotors.forEach(r => { r.max_thrust = s.max_thrust; r.tau = s.tau; r.diameter = s.diameter; r.thrust_exponent = s.thrust_exponent; r.kind = s.kind; r.ram_drag = s.ram_drag; r.turn_loss = s.turn_loss; r.km = (r.km >= 0 ? 1 : -1) * Math.abs(s.km); });
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
$('#leg-generate-btn').addEventListener('click', async () => {
  try {
    await api('/api/airframe', { airframe, keep_state: true });   // the generator uses the live airframe's CG
    const r = await api('/api/airframe/legs/generate', { height: +$('#lg-height').value, spread_x: +$('#lg-sx').value, spread_y: +$('#lg-sy').value,
      landed_pitch_deg: airframe.landed_pitch_deg || 0, attach_z: +$('#lg-az').value });
    airframe.legs = r.legs || [];
    renderLegTable(); scene.setAirframe(airframe); pushAirframe(true);
  } catch (e) { logLine('[ui] leg generation failed: ' + e.message); }
});

// ============================================================ design / optimize
let designTimer = null, designGroups = null, optPoll = null, lastAnalysis = null;
const pct = (v) => (v == null || !isFinite(v)) ? '—' : (v * 100).toFixed(0) + '%';
const num = (v, d = 1, unit = '') => (v == null || !isFinite(v)) ? '—' : v.toFixed(d) + unit;
function designSpec() {
  const groups = {};
  for (const [name, g] of Object.entries(designGroups || {})) {
    const row = $(`#d-groups tr[data-g="${name}"]`);
    const on = (k) => row && row.querySelector(`input[data-k="${k}-on"]`).checked;
    const rng = (k) => [+row.querySelector(`input[data-k="${k}-lo"]`).value, +row.querySelector(`input[data-k="${k}-hi"]`).value];
    groups[name] = { rotors: g.rotors, tilt: on('tilt') ? rng('tilt') : null, cant: on('cant') ? rng('cant') : null };
  }
  return { groups, hover_pitch: $('#d-hp-on').checked ? [+$('#d-hp-lo').value, +$('#d-hp-hi').value] : null,
           weight: 1 - (+$('#d-weight').value) / 100, speed_kmh: +$('#d-speed').value, samples: +$('#d-samples').value, refine: 6 };
}
function groupsFromRotors() {   // saved assignment (airframe.design.groups) if it still fits the rotor count, else the server's default
  const saved = airframe.design && airframe.design.groups;
  if (saved && Object.values(saved).flatMap(g => g.rotors).length === airframe.rotors.length) return JSON.parse(JSON.stringify(saved));
  return lastAnalysis ? JSON.parse(JSON.stringify(lastAnalysis.groups)) : null;
}
function renderGroups() {
  if (!designGroups) return;
  const rows = Object.entries(designGroups).map(([name, g]) => {
    const r0 = airframe.rotors[g.rotors[0]];
    const [tilt, cant] = r0 ? axisToTilt(r0.axis, r0.pos[1]) : [0, 0];
    const prev = g.ui || {};
    return `<tr data-g="${name}"><td class="idx">${name}</td><td class="motors">${g.rotors.map(i => 'M' + (i + 1)).join(' ')}</td>
      <td><input type="checkbox" data-k="tilt-on" ${prev.tilt === false ? '' : 'checked'}> <span class="num">${tilt.toFixed(0)}°</span></td>
      <td><input type="number" data-k="tilt-lo" value="${prev.tiltLo ?? 0}"> – <input type="number" data-k="tilt-hi" value="${prev.tiltHi ?? 90}"></td>
      <td><input type="checkbox" data-k="cant-on" ${prev.cant ? 'checked' : ''}> <span class="num">${cant.toFixed(0)}°</span></td>
      <td><input type="number" data-k="cant-lo" value="${prev.cantLo ?? 0}"> – <input type="number" data-k="cant-hi" value="${prev.cantHi ?? 30}"></td></tr>`;
  }).join('');
  const motorRow = airframe.rotors.map((r, i) => {
    const g = Object.entries(designGroups).find(([, g]) => g.rotors.includes(i));
    return `<label class="row tight" style="gap:2px"><span class="hint">M${i + 1}</span><input type="text" data-m="${i}" value="${g ? g[0] : ''}" maxlength="1"></label>`;
  }).join('');
  $('#d-groups').innerHTML = `<table class="grid"><thead><tr><th>Group</th><th>Motors</th><th title="tilt from vertical, forward/back">Tilt</th><th>Range°</th><th title="lateral cant, symmetric left/right">Cant</th><th>Range°</th></tr></thead><tbody>${rows}</tbody></table>
    <div class="row tight" style="margin-top:6px">${motorRow}</div>`;
  $$('#d-groups input[data-m]').forEach(inp => inp.addEventListener('change', () => {
    const letter = inp.value.trim().toUpperCase() || 'A'; const i = +inp.dataset.m;
    for (const g of Object.values(designGroups)) g.rotors = g.rotors.filter(k => k !== i);
    (designGroups[letter] = designGroups[letter] || { rotors: [] }).rotors.push(i);
    for (const k of Object.keys(designGroups)) if (!designGroups[k].rotors.length) delete designGroups[k];
    designGroups = Object.fromEntries(Object.keys(designGroups).sort().map(k => [k, designGroups[k]]));
    saveGroups(); renderGroups();
  }));
  $$('#d-groups tr[data-g] input').forEach(inp => inp.addEventListener('change', saveGroups));
}
function saveGroups() {
  for (const [name, g] of Object.entries(designGroups)) {
    const row = $(`#d-groups tr[data-g="${name}"]`); if (!row) continue;
    const v = (k) => row.querySelector(`input[data-k="${k}"]`);
    g.ui = { tilt: v('tilt-on').checked, tiltLo: +v('tilt-lo').value, tiltHi: +v('tilt-hi').value, cant: v('cant-on').checked, cantLo: +v('cant-lo').value, cantHi: +v('cant-hi').value };
  }
  airframe.design.groups = designGroups;
}
async function refreshDesign() {
  clearTimeout(designTimer);
  designTimer = setTimeout(async () => {
    if (!airframe) return;
    let r;
    try { r = await api('/api/design/analysis', { airframe, speed_kmh: +$('#d-speed').value }); } catch (e) { $('#d-now').textContent = e.message; return; }
    lastAnalysis = r;
    if (!designGroups) designGroups = groupsFromRotors(); else saveGroups();
    if (!document.activeElement || !document.activeElement.closest('#d-groups')) renderGroups();   // refresh current angles
    renderNow(r);
  }, 80);
}
function renderNow(r) {
  const h = r.hover, c = r.cruise;
  const auth = h.authority || {};
  const probs = [...(h.problems || []), ...(c.problems || []), ...(r.notes || [])];
  $('#d-notes').textContent = '';
  $('#d-now').innerHTML = `<h4>Hover · ${airframe.hover_pitch_deg || 0}° nose-up</h4><div class="kv">
      <div><span>Busiest motor</span><span class="${h.max_util > 0.85 ? 'err' : ''}">${pct(h.max_util)}</span></div>
      <div><span>Wasted thrust</span><span>${pct(h.waste)}</span></div>
      <div><span>Power</span><span>${num(h.power, 0, ' W')}</span></div>
      <div><span>Roll / pitch / yaw</span><span>${num(auth.roll, 1)} / ${num(auth.pitch, 1)} / ${num(auth.yaw, 1)} Nm</span></div></div>
    <h4>Cruise · ${(r.airspeed * 3.6).toFixed(0)} km/h</h4><div class="kv">
      <div><span>Body pitch</span><span>${num(c.pitch_deg, 1, '°')}</span></div>
      <div><span>PX4 pitch</span><span class="${Math.abs(c.px4_pitch_deg) > (r.tilt_limit_deg || 45) ? 'err' : ''}">${num(c.px4_pitch_deg, 1, '°')}</span></div>
      <div><span>Busiest motor</span><span class="${c.max_util > 0.85 ? 'err' : ''}">${pct(c.max_util)}</span></div>
      <div><span>Power vs hover</span><span>${pct(c.power_ratio)}</span></div>
      <div><span>Total thrust</span><span>${num(c.total_thrust, 0, ' N')}</span></div>
      <div><span>Wing lift</span><span>${pct(c.lift_share)} of weight</span></div>
      <div><span>Wing AoA</span><span>${num(c.alpha_deg, 1, '°')}</span></div>
      <div><span>Ram / wing / body drag</span><span>${num(c.ram_drag, 0)} / ${num(c.wing_drag, 0)} / ${num(c.body_drag, 0)} N</span></div></div>
    ${probs.length ? `<div class="problems">⚠ ${probs.join('<br>⚠ ')}</div>` : ''}`;
}
$('#d-speed').addEventListener('change', () => { airframe.design.cruise_speed_kmh = +$('#d-speed').value; pushAirframe(true); });
$('#d-run').addEventListener('click', async () => {
  saveGroups();
  const spec = designSpec();
  $('#d-run').disabled = true; $('#d-progress').textContent = 'starting…'; $('#d-results').innerHTML = '';
  try { await api('/api/design/optimize', { airframe, spec }); } catch (e) { $('#d-progress').textContent = e.message; $('#d-run').disabled = false; return; }
  clearInterval(optPoll);
  optPoll = setInterval(async () => {
    let j; try { j = await api('/api/design/optimize'); } catch { return; }
    if (j.running) { $('#d-progress').textContent = `${j.message} · ${Math.round(j.progress * 100)}%`; return; }
    clearInterval(optPoll); $('#d-run').disabled = false;
    if (j.error) { $('#d-progress').textContent = j.error; return; }
    if (j.result && j.result.ok === false) { $('#d-progress').textContent = j.result.error; return; }
    if (j.result) renderResults(j.result);
  }, 400);
});
function renderResults(res) {
  $('#d-progress').textContent = `${res.evaluated} designs evaluated, ${res.feasible} feasible`;
  const vars = res.variables;
  const head = vars.map(v => `<th title="${esc(v.path || '')}">${v.kind === 'hover_pitch' ? 'Hover°' : (v.group ? v.group + ' ' + v.kind + '°' : esc(v.path || v.kind))}</th>`).join('');
  const row = (m, i, cls, label) => `<tr class="${cls}"><td class="idx">${label}</td>${m.x.map(x => `<td class="num">${x.toFixed(1)}</td>`).join('')}
    <td class="num ${m.hover.ok ? '' : 'err'}">${pct(m.hover.max_util)}</td><td class="num">${num(m.hover.authority.yaw, 1)}</td>
    <td class="num ${m.cruise.converged ? '' : 'err'}">${pct(m.cruise.power_ratio)}</td><td class="num">${num(m.cruise.px4_pitch_deg, 0, '°')}</td><td class="num">${pct(m.cruise.lift_share)}</td>
    <td class="num">${pct(m.cruise.max_util)}</td><td>${i >= 0 ? `<button class="pill small" data-apply="${i}">Apply</button>` : ''}</td></tr>`;
  $('#d-results').innerHTML = `<table class="grid"><thead><tr><th></th>${head}<th title="busiest motor in hover">Hover</th><th title="yaw torque available in hover, Nm">Yaw</th><th title="cruise power relative to hover power">Cruise</th><th>PX4 pitch</th><th>Wing</th><th title="busiest motor in cruise">Motor</th><th></th></tr></thead>
    <tbody>${row(res.current, -1, 'current', 'now')}${res.results.map((m, i) => row(m, i, m.pareto ? 'pareto' : '', String(i + 1))).join('')}</tbody></table>`;
  $$('#d-results button[data-apply]').forEach(b => b.addEventListener('click', () => {
    const m = res.results[+b.dataset.apply];
    (m.axes || []).forEach((a, i) => { if (airframe.rotors[i]) airframe.rotors[i].axis = a; });
    if (m.hover_pitch_deg != null) airframe.hover_pitch_deg = m.hover_pitch_deg;
    setAirframe(airframe); pushAirframe(true);
    b.textContent = 'Applied'; setTimeout(() => b.textContent = 'Apply', 1500);
  }));
}

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
  statusEl.innerHTML = '<span class="muted">Pushing…</span>';
  try {
    await api('/api/airframe', { airframe, keep_state: true });
    const r = await api('/api/px4/push', { save: true });
    const failed = r.results.filter(x => !x.ok);
    statusEl.innerHTML = r.ok
      ? `<span class="ok">✓ PX4 updated (${r.results.length} parameters)</span>`
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
function markDirty() { geometryDirty = true; $('#update-status').innerHTML = '<span class="warn">Changed since last update</span>'; }
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
  const nativeNoseLift = status.mode === 'sitl' && +((params.NLF_ENABLE || {}).value ?? (airframe.px4_overrides || {}).NLF_ENABLE) === 1;
  tko.title = nativeNoseLift ? 'PX4 arms, raises the nose to the configured target, then climbs and holds heading' : 'Automatic takeoff';
  tko.textContent = 'Takeoff';
  tko.disabled = !!status.resetting || !status.ctl_connected || (nativeNoseLift && status.armed) || (!status.armed && !status.arm_ready);
  $$('#nl-card input, #nl-card button').forEach(el => { el.disabled = !!nativeNoseLift; });
  const landButton = $('#btn-land');
  if (landButton) {
    landButton.textContent = 'Land';
    landButton.disabled = !status.ctl_connected || !status.armed;
    landButton.title = nativeNoseLift ? 'From native hover: descend onto rear legs, lower the nose at 4 degrees/s, then disarm' : 'Land';
  }
  $('#af-hover').disabled = !!status.armed;
  $('#af-landed').readOnly = true;
  const targetInput = $('#af-takeoff');
  targetInput.disabled = !nativeNoseLift || !!status.armed || !status.ctl_connected;
  $('#nl-target').closest('label').hidden = !!nativeNoseLift;
  if (nativeNoseLift) {
    targetInput.disabled = !!status.armed || !status.ctl_connected;
    targetInput.step = 'any';
    targetInput.title = 'PX4 NLF_TARGET: ground nose lift angle. Hover pitch is configured separately.';
    if (document.activeElement !== targetInput) targetInput.value = (params.NLF_TARGET || {}).value ?? (airframe.px4_overrides || {}).NLF_TARGET ?? airframe.hover_pitch_deg ?? 25;
  }
  const nlHint = $('#nl-card .hint');
  if (nlHint && nativeNoseLift) nlHint.textContent = 'Native PX4 module ready. Use Takeoff below. Set Takeoff pitch° in the Geometry tab before takeoff. Hover pitch and Landed pitch are configured in Geometry. Then use Land below. Other controls in this card are simulator-only and inactive.';
  $$('#mode-pills .pill').forEach(el => { if (el.textContent.toLowerCase() === 'takeoff') { el.disabled = !!nativeNoseLift; el.title = nativeNoseLift ? 'Use Takeoff below' : ''; } });

  tko.classList.toggle('hidden', !!status.armed && !status.on_ground_hint && false);
  const upd = $('#btn-update');
  upd.disabled = !status.ctl_connected || !!status.armed;
  upd.title = status.armed ? 'Disarm first: PX4 rebuilds its allocation when these parameters change' :
    (status.ctl_connected ? 'Write the rotor geometry and output mapping to the flight controller and save it' : 'PX4 not connected');
  $$('#mode-pills .pill').forEach(b => b.classList.toggle('active', status.connected && (status.mode_name || '').toLowerCase() === b.textContent.toLowerCase()));
}
// ---- nose lift (ground sequence): settings live in airframe.design.nose_lift
let lastNoseLift = null;
function noseLiftCfg() {
  airframe.design = airframe.design || {};
  const d = airframe.design.nose_lift || {};
  const motors = $$('#nl-motors input[data-m]').filter(c => c.checked).map(c => +c.dataset.m);
  return { enabled: $('#nl-use').checked, motors: motors.length ? motors : (d.motors || []), target_pitch_deg: +$('#nl-target').value,
           rate_deg_s: +$('#nl-rate').value || 8, assist_cmd: (+$('#nl-assist').value || 0) / 100 };
}
function fillNoseLiftCard() {
  if (!airframe) return;
  const d = (airframe.design && airframe.design.nose_lift) || {};
  const def = d.motors || airframe.rotors.map((r, i) => (r.pos[0] > 0.2 ? i : -1)).filter(i => i >= 0);   // front motors by default
  $('#nl-motors').innerHTML = airframe.rotors.map((r, i) => `<label class="row tight" style="gap:3px"><input type="checkbox" data-m="${i}" ${def.includes(i) ? 'checked' : ''}> <span class="hint">M${i + 1}</span></label>`).join('');
  $('#nl-target').value = d.target_pitch_deg ?? airframe.hover_pitch_deg ?? 0;
  $('#nl-rate').value = d.rate_deg_s ?? 8;
  $('#nl-assist').value = Math.round((d.assist_cmd ?? 0) * 100);
  $('#nl-use').checked = !!d.enabled;
  $$('#nl-card input').forEach(inp => { inp.onchange = () => {
    if (status.mode === 'sitl' && +((params.NLF_ENABLE || {}).value ?? (airframe.px4_overrides || {}).NLF_ENABLE) === 1) return;
    airframe.design.nose_lift = noseLiftCfg(); pushAirframe(true);
  }; });
}
async function startNoseLift() {
  const c = noseLiftCfg();
  if (!c.motors.length) { logLine('[ui] nose lift: choose the motors first'); return false; }
  const assist = airframe.rotors.map((_, i) => i).filter(i => !c.motors.includes(i));
  try {
    await api('/api/sim/nose_lift', { motors: c.motors, target_pitch_deg: c.target_pitch_deg, rate_deg_s: c.rate_deg_s,
                                       assist_motors: c.assist_cmd > 0 ? assist : [], assist_cmd: c.assist_cmd });
    return true;
  } catch (e) { logLine('[ui] nose lift: ' + e.message); $('#nl-status').textContent = e.message; return false; }
}
$('#nl-start').addEventListener('click', startNoseLift);
$('#nl-stop').addEventListener('click', () => api('/api/sim/nose_lift', { stop: true }));
function updateNoseLift(st) {
  lastNoseLift = st.nose_lift || null;
  const el = $('#nl-status'); if (!el) return;
  const n = lastNoseLift;
  el.textContent = n ? `${n.state}: nose ${n.pitch_deg.toFixed(1)}° → ${n.target_deg}°, cmd ${(n.cmd * 100).toFixed(0)}%${n.reason ? ' · ' + n.reason : ''}` : '';
  el.classList.toggle('err', !!(n && n.state === 'failed'));
}
async function waitNoseLift(timeoutMs = 40000) {
  const t0 = Date.now();
  while (Date.now() - t0 < timeoutMs) {
    await new Promise(r => setTimeout(r, 100));
    if (!lastNoseLift) continue;
    if (lastNoseLift.state === 'holding' || lastNoseLift.state === 'handover') return true;
    if (lastNoseLift.state === 'failed') return false;
  }
  return false;
}
$('#btn-takeoff').addEventListener('click', async () => {
  try {
    if (status.mode === 'sitl' && +((params.NLF_ENABLE || {}).value ?? (airframe.px4_overrides || {}).NLF_ENABLE) === 1) {
      if (status.armed) return;
      const landing = await api('/api/params/set', { name: 'NLF_LAND_ANG', value: Number(airframe.landed_pitch_deg) });
      if (!landing.ok) throw new Error(landing.error || 'Could not apply landed pitch');
      logLine('[ui] native PX4 nose lift: normal arming, front-fan lift, then heading-held hover');
      await api('/api/sim/nose_lift', { stop: true });
      const moduleStatus = await api('/api/px4/shell', { command: 'atlas_nose_lift status', timeout: 0.5 });
      if (/not running/i.test(moduleStatus.output || '')) await api('/api/px4/shell', { command: 'atlas_nose_lift start', timeout: 0.5 });
      const result = await api('/api/px4/shell', { command: 'atlas_nose_lift takeoff', timeout: 0.5 });
      if (result.output) logLine(result.output);
      return;
    }

    const useNl = airframe && airframe.design && airframe.design.nose_lift && airframe.design.nose_lift.enabled;
    if (useNl && !status.armed) {
      logLine('[ui] takeoff: lifting the nose first');
      if (!(await startNoseLift())) return;
      if (!(await waitNoseLift())) { logLine('[ui] takeoff cancelled: nose lift ' + (lastNoseLift ? lastNoseLift.state + ' ' + (lastNoseLift.reason || '') : 'timed out')); return; }
    }
    if (!status.armed) { await api('/api/px4/command', { command: 'arm' }); await new Promise(r => setTimeout(r, 1500)); }
    await api('/api/px4/command', { command: 'takeoff' });
  } catch (e) { logLine('[ui] ' + e.message); }
});
$('#btn-arm').addEventListener('click', async () => {
  try { await api('/api/px4/command', status.armed ? { command: 'kill', force: true } : { command: 'arm' }); } catch (e) { logLine('[ui] ' + e.message); }
});

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
$('#param-reboot').addEventListener('click', () => { if (confirm('Reboot the flight controller?')) api('/api/px4/command', { command: 'reboot' }); });
$('#param-meta-fetch').addEventListener('click', async () => {
  $('#param-count').textContent = 'downloading descriptions via MAVLink FTP…';
  try { const r = await api('/api/params/meta/fetch', {}); await loadMeta(); logLine(`[ui] ${r.count} parameter descriptions from ${r.source}`); }
  catch (e) { logLine('[ui] ' + e.message); }
  await loadParams();
});

// ============================================================ flight / sim
$$('#tab-sim button[data-cmd], #btn-land').forEach(b => b.addEventListener('click', async () => {
  try {
    if (b.dataset.mode === 'land' && status.mode === 'sitl' && +((params.NLF_ENABLE || {}).value ?? (airframe.px4_overrides || {}).NLF_ENABLE) === 1) {
      logLine('[ui] native landing: rear legs first, slow nose lowering, then motors off');
      const result = await api('/api/px4/shell', { command: 'atlas_nose_lift land', timeout: 0.5 });
      if (result.output) logLine(result.output);
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
$('#sim-speed').addEventListener('change', e => api('/api/sim/speed', { speed: parseFloat(e.target.value) }));
$('#sim-physics').addEventListener('change', async e => {
  try { const r = await api('/api/sim/physics', { physics: e.target.value }); logLine('[ui] physics engine: ' + r.physics); }
  catch (err) { logLine('[ui] physics switch failed: ' + err.message); }
});
$('#sim-noise').addEventListener('change', e => api('/api/sim/noise', { enabled: e.target.checked }));
$('#wind-apply').addEventListener('click', () => api('/api/sim/wind', { north: +$('#wind-n').value, east: +$('#wind-e').value, down: +$('#wind-d').value }));
$('#home-apply').addEventListener('click', () => api('/api/sim/home', { lat: +$('#home-lat').value, lon: +$('#home-lon').value, alt: +$('#home-alt').value }));

// one set of sliders: shows PX4's live motor commands, or drives the physics directly with Manual override on
function renderMotorSliders() {
  const el = $('#motor-sliders');
  el.innerHTML = airframe.rotors.map((r, i) => `<label><b title="${esc(r.name)}">M${i + 1}</b> <input type="range" min="0" max="1" step="0.01" value="0" data-i="${i}" ${$('#motor-enable').checked ? '' : 'disabled'}><span class="mv num">0%</span></label>`).join('');
  el.querySelectorAll('input').forEach(inp => inp.addEventListener('input', sendOverride));
}
function sendOverride() {
  const vals = $$('#motor-sliders input').map(i => parseFloat(i.value));
  if ($('#motor-enable').checked) api('/api/sim/motor_override', { values: vals });
}
function updateMotorSliders(st) {
  const manual = $('#motor-enable').checked;
  st.rotors.forEach((x, i) => {
    const inp = document.querySelector(`#motor-sliders input[data-i="${i}"]`), lab = document.querySelector(`#motor-sliders label:nth-child(${i + 1}) .mv`);
    if (inp && !manual) inp.value = x.cmd.toFixed(2);
    const pct = airframe.rotors[i] && airframe.rotors[i].max_thrust > 0 ? x.thrust / airframe.rotors[i].max_thrust * 100 : 0;
    if (lab) lab.textContent = pct.toFixed(0) + '%';
  });
}
$('#motor-enable').addEventListener('change', (e) => {
  $$('#motor-sliders input').forEach(inp => inp.disabled = !e.target.checked);
  if (e.target.checked) sendOverride(); else api('/api/sim/motor_override', { values: null });
});

// ============================================================ websocket + status
let homeFilled = false;
function applyStatus(s) {
  status = s;
  const none = !s.conn_mode;
  $('#st-mode').textContent = (none ? 'No link' : (s.mode === 'hitl' ? 'Pixhawk' : 'SITL')) + (s.physics === 'jsbsim' ? ' · JSBSim' : '');
  if (s.physics && document.activeElement !== $('#sim-physics')) $('#sim-physics').value = s.physics;
  $('#st-conn').textContent = s.paused ? 'PAUSED — PX4 gets no data, press Resume' : none ? (s.conn_error ? 'failed — open Connect' : 'open Connect')
    : s.connected ? (s.mode === 'hitl' ? s.address.replace('/dev/', '') + (s.hil_enabled ? '' : ' · HITL off') : 'PX4 connected')
    : (s.mode === 'sitl' ? (s.px4_running ? 'PX4 starting…' : 'waiting for PX4') : 'no data from ' + s.address.replace('/dev/', ''));
  $('#st-link .dot').classList.toggle('on', s.connected);
  const joyCard = $('#joy-card');
  if (joyCard) joyCard.classList.toggle('hidden', s.conn_mode !== 'sitl');
  const det = $('#st-detected');
  const showDet = !s.flashing && s.mode !== 'hitl' && s.px4_ports && s.px4_ports.length > 0;
  if (s.flashing) { $('#st-mode').textContent = 'Pixhawk'; $('#st-conn').textContent = 'flashing firmware…'; }
  det.classList.toggle('hidden', !showDet); det.classList.toggle('blink', showDet);
  if (showDet) det.textContent = `Pixhawk on ${s.px4_ports[0].replace('/dev/', '')} · connect`;
  $('#st-armed').textContent = s.resetting > 0 ? `Resetting… ${Math.ceil(s.resetting)} s` : (s.armed ? 'Armed' : 'Disarmed');
  $('#st-armed').classList.toggle('armed', s.armed);
  $('#st-flightmode').textContent = s.connected ? s.mode_name + (s.mode === 'hitl' && !s.hil_enabled ? ' · HIL OFF (set SYS_HITL=1)' : '') : '—';
  if (!homeFilled) { $('#home-lat').value = s.home.lat; $('#home-lon').value = s.home.lon; $('#home-alt').value = s.home.alt; homeFilled = true; }
  if (s.params_loaded && !paramsLoaded && $('#tab-px4').classList.contains('active')) ensureParams();
  updateFooter();
}
const effectiveMax = (r) => {
  if (r.enabled === false) return 0;
  if (!r.duct_axis) return r.max_thrust;
  const a = r.axis, d = r.duct_axis, na = Math.hypot(...a) || 1, nd = Math.hypot(...d) || 1;
  const bend = deg(Math.acos(Math.max(-1, Math.min(1, (a[0] * d[0] + a[1] * d[1] + a[2] * d[2]) / (na * nd)))));
  return r.max_thrust * Math.max(0, 1 - (r.turn_loss ?? 0.1) * bend / 90);
};
function applyState(st) {
  scene.updateState(st);
  updateNoseLift(st);
  $('#btn-pause').textContent = st.paused ? 'Resume' : 'Pause';
  $('#btn-pause').classList.toggle('armed', !!st.paused);
  $('#st-time').textContent = `${st.t.toFixed(1)} s`;
  if (airframe && st.rotors && st.rotors.length) {
    const total = st.rotors.reduce((a, r) => a + (r.thrust || 0), 0);
    const max = airframe.rotors.reduce((a, r) => a + effectiveMax(r), 0);
    $('#st-thrust').textContent = max > 0 ? `Thrust ${(100 * total / max).toFixed(0)}% · ${total.toFixed(0)} N` : 'Thrust —';
  }
  {
    const f0 = st.forces || {}, weight = airframe && airframe.mass ? (+airframe.mass.mass || 0) * 9.80665 : 0;
    const hasWing = airframe && (airframe.wings || []).some(w => w.enabled !== false);
    const el = $('#st-lift');
    el.textContent = hasWing && f0.lift != null && weight > 0 ? `Lift ${(100 * f0.lift / weight).toFixed(0)}% · ${(+f0.lift).toFixed(0)} N` : 'Lift —';
    el.classList.toggle('armed', !!f0.stalled);
    el.title = 'wing lift as a share of the weight, and in newtons' + (f0.stalled ? ' · a wing is STALLED' : '');
  }
  $('#st-rtf').textContent = `RTF ${st.rtf ? st.rtf.toFixed(2) : '—'}${st.lockstep_timeouts ? ' · ' + st.lockstep_timeouts + ' waits' : ''}`;
  const [r, p, y] = st.euler.map(deg);
  $('#st-pose').textContent = `N ${st.pos[0].toFixed(1)} E ${st.pos[1].toFixed(1)} alt ${(-st.pos[2]).toFixed(2)} m · R ${r.toFixed(0)}° P ${p.toFixed(0)}° Y ${y.toFixed(0)}°${st.on_ground ? ' · on ground' + (st.feet_down != null ? ` (${st.feet_down} feet)` : '') : ''}`;
  const f = st.forces || {};
  const parts = [];
  if (f.airspeed != null) parts.push(`airspeed ${(+f.airspeed).toFixed(1)} m/s`);
  if (f.lift != null) parts.push(`lift ${(+f.lift).toFixed(0)} N` + (f.stalled ? ' (stalled)' : ''));
  if (f.alpha && f.alpha.length) parts.push(`α ${f.alpha.map(a => deg(+a).toFixed(1)).join('/')}°`);
  if (f.wing_drag != null || f.ram_drag != null || f.body_drag != null) parts.push(`drag wing ${(+f.wing_drag || 0).toFixed(0)} · ram ${(+f.ram_drag || 0).toFixed(0)} · body ${(+f.body_drag || 0).toFixed(0)} N`);
  if (f.power != null) parts.push(`power ${(+f.power).toFixed(0)} W`);
  if (st.rotor_health && st.rotor_health.some(h => h < 0.999)) parts.push('rotor health ' + st.rotor_health.map(h => (h * 100).toFixed(0) + '%').join(' '));
  $('#st-aero').textContent = parts.join(' · ');
  if ($('#tab-sim').classList.contains('active') && airframe) updateMotorSliders(st);
}
const logPre = $('#log-pre');
function logLine(s) { logPre.textContent += s + '\n'; if (logPre.textContent.length > 40000) logPre.textContent = logPre.textContent.slice(-30000); const b = $('#log-body'); b.scrollTop = b.scrollHeight; }
$('#log-toggle').addEventListener('click', (e) => { const c = $('#log').classList.toggle('collapsed'); e.target.textContent = c ? 'Show' : 'Hide'; });
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
    if (m.type === 'state') { applyState(m.state); if (m.status) applyStatus(m.status); if (m.log) m.log.forEach(l => logLine(l[1])); }
  };
  ws.onclose = () => { logLine('[ui] connection to simulator lost, retrying…'); setTimeout(connectWs, 1500); };
}
connectWs();
loadPresetList();
api('/api/log?since=0').then(lines => lines.forEach(l => logLine(l[1]))).catch(() => { });


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


// ============================================================ connection / HITL
let connTimer = null;
$('#st-link').addEventListener('click', () => openTab('connect'));
$('#st-detected').addEventListener('click', async () => { openTab('connect'); await connectHitl(); });
$('#conn-sitl').addEventListener('click', async () => { await connCall('/api/connection/connect', { mode: 'sitl' }); });
$('#conn-rescan').addEventListener('click', refreshConnection);
$('#conn-disconnect').addEventListener('click', async () => { await connCall('/api/connection/disconnect', {}); });
$('#conn-reboot').addEventListener('click', async () => { if (confirm('Reboot the flight controller? The link reconnects by itself.')) { await api('/api/px4/command', { command: 'reboot' }); setTimeout(refreshConnection, 3000); } });
// ---- PX4 messages (decoded events) in the Flight tab
let eventsTimer = null;
async function refreshEvents() {
  clearTimeout(eventsTimer);
  if (!$('#tab-sim').classList.contains('active')) return;
  try {
    const r = await api('/api/events');
    const ev = r.events || [];
    let summary = null;
    for (let i = ev.length - 1; i >= 0; i--) if (ev[i].name === 'commander_arming_check_summary') { summary = ev[i]; break; }
    const el = $('#px4-messages');
    let html = '';
    if (summary) {
      const d = {}; (summary.arg_names || []).forEach((k, i) => d[k] = summary.args[i]);
      const canArm = String(d.can_arm || '').split('|').filter(x => x && !/^\d+$/.test(x));
      const errs = String(d.error || '').split('|').filter(x => x && x !== '0' && x.toLowerCase() !== 'system');   // "system" = offboard/mission checks
      html += `<div class="conn-row"><div><b>${canArm.length ? 'Can arm in' : 'Cannot arm'}</b> <span class="hint">${canArm.length ? canArm.join(', ') : 'see messages below'}</span>` +
        `${errs.length ? `<div class="err">blocking: ${errs.join(', ').replace(/_/g, ' ')}</div>` : ''}</div></div>`;
    }
    // PX4 sends its arming/health checks as a batch right before each summary. Show only the batch that belongs
    // to the latest summary (everything older is history, not the current state), plus anything newer than it.
    const modeNow = (status.mode_name || '').toLowerCase();
    const lastSummary = [...ev].reverse().find(x => x.name === 'commander_arming_check_summary');
    const tCut = lastSummary ? lastSummary.t - 0.6 : (Date.now() / 1000 - 10);
    const seen = new Set();
    const shown = ev.filter(x => x.t >= tCut && x.level <= 6 && x.group !== 'protocol' && !x.name.includes('summary'))
      .filter(x => !(/offboard/i.test(x.text) && modeNow !== 'offboard') && !(/mission/i.test(x.text) && modeNow !== 'mission'))
      .reverse().filter(x => { if (seen.has(x.text)) return false; seen.add(x.text); return true; }).slice(0, 8);
    html += shown.map(x => `<div class="msg ${x.level <= 3 ? 'err' : x.level === 4 ? 'warn' : ''}"><span class="lvl">${x.level_name}</span> ${x.text}</div>`).join('') || '<div class="hint">no messages from PX4 yet</div>';
    el.innerHTML = html;
  } catch (e) { }
  eventsTimer = setTimeout(refreshEvents, 2000);
}
$$('.tabs button').forEach(b => b.addEventListener('click', () => { if (b.dataset.tab === 'sim') refreshEvents(); }));
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
  if (!$('#tab-connect').classList.contains('active')) return;
  let c;
  try { c = await api('/api/connection'); } catch (e) { $('#conn-error').textContent = e.message; return; }
  const L = c.link;
  const cur = $('#conn-current');
  if (!c.mode) cur.innerHTML = '<div class="conn-row"><div><b>Not connected</b><div class="hint">Pick PX4 SITL or a Pixhawk below.</div></div></div>';
  else if (c.mode === 'sitl') cur.innerHTML = `<div class="conn-row"><div><b>PX4 SITL</b> <span class="${L.connected ? 'ok' : 'muted'}">${L.connected ? '● connected' : (c.px4_running ? '○ starting…' : '○ waiting for PX4')}</span><div class="hint">instance ${c.px4_instance ?? 0} · simulator tcp ${L.address} · control ${L.ctl_address}</div></div></div>`;
  else cur.innerHTML = `<div class="conn-row"><div><b>Pixhawk</b> <span class="${L.connected ? 'ok' : 'muted'}">${L.connected ? '● link up' : '○ no data yet'}</span><div class="dev">${c.serial} @ ${c.baud}</div><div class="hint">QGroundControl: connect over UDP ${c.qgc}</div></div></div>`;
  $('#conn-error').textContent = c.error || '';
  $('#conn-sitl-card').classList.toggle('current', c.mode === 'sitl');
  const ports = c.ports || [];
  $('#conn-ports').innerHTML = ports.length ? ports.map(p => `
    <div class="card ${c.mode === 'hitl' && c.serial === p.device ? 'current' : ''}"><div class="conn-row">
      <div><b>${p.likely_px4 ? 'Pixhawk' : 'Serial device'}</b> <span class="hint">${p.description || ''}</span><div class="dev">${p.device}</div></div>
      <button class="pill small ${p.likely_px4 ? 'primary' : ''}" data-dev="${p.device}">${c.mode === 'hitl' && c.serial === p.device ? 'Reconnect' : 'Connect'}</button>
    </div></div>`).join('')
    : '<div class="card"><div class="conn-row"><div><b>No USB flight controller found</b><div class="hint">Plug the Pixhawk in over USB and click Rescan. If QGroundControl is open, close it or disable its serial auto-connect.</div></div></div></div>';
  $$('#conn-ports button[data-dev]').forEach(b => b.addEventListener('click', () => connectHitl(b.dataset.dev)));
  const steps = c.checklist || [];
  $('#conn-checklist').innerHTML = steps.map(s => `<div class="check ${s.ok ? 'ok' : ''}"><div class="mark">${s.ok ? '✓' : ''}</div>
    <div class="body"><div class="label">${s.label}</div>${s.detail ? `<div class="detail">${s.detail}</div>` : ''}</div>
    ${s.busy ? '<span class="pill small">Working…</span>' : ''}
    ${s.action === 'enable_hitl' ? '<button class="pill small primary" data-act="enable_hitl">Enable HITL</button>' : ''}
    ${s.action === 'build_firmware' ? '<button class="pill small primary" data-act="build_firmware">Build firmware</button>' : ''}
    ${s.action === 'upload_firmware' ? '<button class="pill small primary" data-act="upload_firmware">Flash firmware</button>' : ''}
    ${s.action === 'push' ? '<button class="pill small primary" data-act="push">Push geometry</button>' : ''}
    ${s.action === 'reboot' ? '<button class="pill small" data-act="reboot">Reboot board</button>' : ''}
    ${s.action === 'ekf' ? '<button class="pill small" data-act="ekf">Restart estimator</button>' : ''}</div>`).join('');
  $$('#conn-checklist button[data-act]').forEach(b => b.addEventListener('click', async () => {
    if (b.dataset.act === 'enable_hitl') { b.textContent = 'Rebooting…'; await connCall('/api/connection/enable_hitl', {}); }
    if (b.dataset.act === 'push') { if (status.armed) { alert('Disarm before updating PX4'); return; } await pushToPX4($('#update-status'), true); await refreshConnection(); }
    if (b.dataset.act === 'build_firmware') { b.textContent = 'Building…'; await connCall('/api/firmware/build', {}); }
    if (b.dataset.act === 'ekf') { b.textContent = 'Restarting…'; await api('/api/connection/restart_estimator', {}); setTimeout(refreshConnection, 3000); }
    if (b.dataset.act === 'reboot') { b.textContent = 'Rebooting…'; await api('/api/px4/command', { command: 'reboot' }); setTimeout(refreshConnection, 3000); }
    if (b.dataset.act === 'upload_firmware') {
      if (!confirm('Flash the HITL-capable firmware to the board now? It reboots and reconnects when done. Parameters are kept.')) return;
      b.textContent = 'Flashing…'; await connCall('/api/firmware/upload', {});
    }
  }));
  connTimer = setTimeout(refreshConnection, 2000);
}


// ============================================================ USB remote (WebHID picker, Gamepad API fallback) -> MANUAL_CONTROL
const JOY_FUNCS = [
  { key: 'roll', label: 'Roll', axis: 0, invert: false },
  { key: 'pitch', label: 'Pitch', axis: 1, invert: true },      // stick forward is negative on most devices
  { key: 'throttle', label: 'Throttle', axis: 2, invert: false },
  { key: 'yaw', label: 'Yaw', axis: 3, invert: false },
];
let joyMap = JOY_FUNCS.map(f => ({ ...f }));
try { const saved = JSON.parse(localStorage.getItem('airframe-joystick') || 'null'); if (saved && saved.length === 4) joyMap = saved; } catch { }
let joyLearn = null, joyLearnBase = null;
// current input source: { name, axes: [-1..1], buttons: [bool], kind: 'hid' | 'gamepad' }
let joySrc = null;
let hidDevice = null;
function joySave() { try { localStorage.setItem('airframe-joystick', JSON.stringify(joyMap)); } catch { } }

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
    $('#joy-hint').textContent = 'This browser cannot show a device picker (Safari has no WebHID). Open the page in Chrome/Edge, or move a stick: the gamepad fallback may still find it.';
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
  if (!navigator.serial) { $('#joy-hint').textContent = 'This browser has no Web Serial (Safari). Open the page in Chrome or Edge.'; return; }
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
  // channel values grow when the stick goes forward/right/up, unlike HID gamepads (forward = negative): unless the
  // user has customised the mapping, do not invert pitch for a serial radio. Learn fixes any remaining sign.
  let customised = false; try { customised = !!localStorage.getItem('airframe-joystick'); } catch { }
  if (!customised) { const pf = joyMap.find(f => f.key === 'pitch'); if (pf) pf.invert = false; }
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
function joyRenderMap() {
  const src = joyCurrent();
  const n = src && src.axes.length ? src.axes.length : 8;
  $('#joy-map').innerHTML = joyMap.map((f, i) => `<div class="joy-row"><span class="joy-label">${f.label}</span>
    <select data-i="${i}" class="joy-axis">${Array.from({ length: n }, (_, a) => `<option value="${a}" ${a === f.axis ? 'selected' : ''}>axis ${a + 1}</option>`).join('')}</select>
    <label class="row" style="margin:0"><input type="checkbox" data-i="${i}" class="joy-inv" ${f.invert ? 'checked' : ''}> invert</label>
    <button class="pill small joy-learn" data-i="${i}" title="click, then push this stick fully in its positive direction: forward for pitch, right for roll and yaw, up for throttle; the axis and its sign are learned">${joyLearn === i ? ({ pitch: 'push forward…', roll: 'push right…', throttle: 'push up…', yaw: 'push right…' }[f.key] || 'move it…') : 'Learn'}</button>
    <span class="rc-bar joy-bar"><i data-i="${i}" style="width:50%"></i></span><span class="rc-val num joy-val" data-i="${i}">—</span></div>`).join('');
  $$('.joy-axis').forEach(sel => sel.addEventListener('change', () => { joyMap[+sel.dataset.i].axis = +sel.value; joySave(); }));
  $$('.joy-inv').forEach(cb => cb.addEventListener('change', () => { joyMap[+cb.dataset.i].invert = cb.checked; joySave(); }));
  $$('.joy-learn').forEach(b => b.addEventListener('click', () => { joyLearn = +b.dataset.i; const s = joyCurrent(); joyLearnBase = s ? s.axes.slice() : null; joyRenderMap(); }));
}
function joyValue(src, f) {
  if (!src) return 0;
  let v = src.axes[f.axis] ?? 0;
  if (Math.abs(v) < 0.02) v = 0;                  // deadband
  return f.invert ? -v : v;
}
let joyLastSend = 0;
function joyTick() {
  const src = joyCurrent();
  const nameEl = $('#joy-name');
  if (!src) {
    nameEl.textContent = 'No radio connected';
    $('#joy-hint').textContent = (status.radio_vcp_ports && status.radio_vcp_ports.length) ? status.radio_vcp_ports[0].replace('/dev/', '') : '';
  }
  else {
    nameEl.textContent = src.name + (src.kind === 'gamepad' ? ' (gamepad)' : '');
    $('#joy-hint').textContent = src.kind === 'serial' ? `${src.axes.length} channels` : `${src.axes.length} axes`;
    if (joyLearn != null && joyLearnBase) {
      let best = -1, bestD = 0.3;
      src.axes.forEach((v, a) => { const d = Math.abs(v - (joyLearnBase[a] ?? 0)); if (d > bestD) { bestD = d; best = a; } });
      if (best >= 0) {
        // the user pushed the stick in the positive direction (forward, right, up, right): the sign of the move fixes the inversion
        joyMap[joyLearn].axis = best;
        joyMap[joyLearn].invert = (src.axes[best] - (joyLearnBase[best] ?? 0)) < 0;
        logLine(`[ui] ${joyMap[joyLearn].label}: axis ${best + 1}${joyMap[joyLearn].invert ? ' (inverted)' : ''}`);
        joyLearn = null; joyLearnBase = null; joySave(); joyRenderMap();
      }
    }
    joyMap.forEach((f, i) => {
      const v = joyValue(src, f); const bar = document.querySelector(`.joy-bar i[data-i="${i}"]`), val = document.querySelector(`.joy-val[data-i="${i}"]`);
      if (bar) bar.style.width = ((v + 1) / 2 * 100).toFixed(0) + '%';
      if (val) val.textContent = v.toFixed(2);
    });
    const now = performance.now();
    if (status.conn_mode === 'sitl' && joyWs && joyWs.readyState === 1 && now - joyLastSend > 20) {   // 50 Hz, SITL only
      joyLastSend = now;
      const g = (k) => joyValue(src, joyMap.find(f => f.key === k));
      const aux = src.axes.slice(4, 10).map(v => +v.toFixed(3));
      let buttons = 0; src.buttons.forEach((b, i) => { if (b && i < 16) buttons |= (1 << i); });
      joyWs.send(JSON.stringify({ type: 'manual', roll: g('roll'), pitch: g('pitch'), throttle: (g('throttle') + 1) / 2, yaw: g('yaw'), buttons, aux }));
    }
  }
  if (src || $('#tab-connect').classList.contains('active')) requestAnimationFrame(joyTick); else setTimeout(joyTick, 500);
}
if (navigator.hid) navigator.hid.addEventListener('disconnect', (e) => { if (e.device === hidDevice) { hidDevice = null; joySrc = null; logLine('[ui] radio disconnected'); joyRenderMap(); } });
$('#joy-connect').addEventListener('click', serialConnect);
joyRenderMap(); joyTick(); serialReconnect();


// ============================================================ batch: scenarios, headless jobs, studies
let batchTimer = null, studyTimer = null, scenariosCache = null, studiesCache = null, lastJobs = null;
const jobsOpen = new Set();
const KEY_METRICS = ['alt_mean', 'pos_std_xy', 'roll_rms_deg', 'pitch_rms_deg', 'util_max', 'power_mean', 'time_to_alt', 'speed_mean', 'lift_share_mean'];
const mnum = (v, d = 2) => (v == null || !isFinite(v)) ? '—' : (+v).toFixed(d);
async function refreshBatch() {
  clearTimeout(batchTimer); clearTimeout(studyTimer);
  if (!$('#tab-batch').classList.contains('active')) return;
  if (!scenariosCache) await loadScenarios();
  if (!studiesCache) await loadStudies();
  pollJobs(); pollStudy();
}
async function loadScenarios() {
  const el = $('#scenario-list');
  try {
    const r = await api('/api/scenarios');
    scenariosCache = r.scenarios || [];
  } catch (e) { el.innerHTML = `<div class="hint">${esc(e.message)}</div>`; return; }
  el.innerHTML = scenariosCache.length ? scenariosCache.map(s => `<div class="card sc"><div class="conn-row"><div><b>${esc(s.name)}</b> <span class="hint mono">${esc(s.file)}</span>
      <div class="hint">${esc(s.description || '')}</div><div class="phases">${(s.phases || []).map(p => `<span class="phase">${esc(p)}</span>`).join('')}</div></div>
      <div class="row tight"><select class="sc-physics" data-sc="${esc(s.file)}" title="physics engine for this headless run"><option value="python">Python</option><option value="jsbsim">JSBSim</option></select>
      <button class="pill small primary" data-run="${esc(s.file)}">Run headless</button>
      <button class="pill small" data-visible="${esc(s.file)}" title="Run at normal speed in the live viewport with the currently loaded aircraft and physics engine">Run visibly</button>
      <button class="pill small" data-compare="${esc(s.file)}" title="run this scenario on BOTH engines and show the differences">Compare physics</button></div></div></div>`).join('')
    : '<div class="hint">No scenarios in scenarios/.</div>';
  const start = async (file, physics) => {
    await api('/api/airframe', { airframe, keep_state: true });   // the job copies the live airframe
    const r = await api('/api/batch/run', { scenario: file.replace(/\.json$/, ''), options: { physics } });
    logLine(`[batch] started ${r.id} (${file}, ${physics})`);
    return r.id;
  };
  $$('#scenario-list button[data-run]').forEach(b => b.addEventListener('click', async () => {
    b.disabled = true; b.textContent = 'Starting…';
    const physics = ($(`.sc-physics[data-sc="${b.dataset.run}"]`) || {}).value || 'python';
    try { await start(b.dataset.run, physics); pollJobs(); } catch (e) { logLine('[batch] ' + e.message); }
    b.disabled = false; b.textContent = 'Run headless';
  }));
  $$('#scenario-list button[data-visible]').forEach(b => b.addEventListener('click', async () => {
    b.disabled = true; b.textContent = 'Starting…';
    try {
      const r = await api('/api/sim/scenario/start', { scenario: b.dataset.visible });
      logLine(`[live test] started ${r.name}`);
    } catch (e) { logLine('[live test] ' + e.message); }
    b.disabled = false; b.textContent = 'Run visibly';
  }));
  $$('#scenario-list button[data-compare]').forEach(b => b.addEventListener('click', async () => {
    b.disabled = true; b.textContent = 'Starting…';
    try { const a = await start(b.dataset.compare, 'python'); const c = await start(b.dataset.compare, 'jsbsim'); comparePairs.push([a, c]); pollJobs(); }
    catch (e) { logLine('[batch] ' + e.message); }
    b.disabled = false; b.textContent = 'Compare physics';
  }));
}
const comparePairs = [];   // [pythonJobId, jsbsimJobId] pairs started by "Compare physics"
const CMP_KEYS = ['alt_mean', 'alt_std', 'pos_std_xy', 'pos_drift', 'speed_mean', 'roll_rms_deg', 'pitch_rms_deg', 'pitch_mean_deg', 'yaw_drift_deg', 'rates_rms_deg_s', 'util_max', 'power_mean', 'lift_share_mean', 'time_to_alt', 'time_to_pitch'];
function renderCompare(jobs) {
  const el = $('#batch-compare'); if (!el) return;
  const byId = Object.fromEntries(jobs.map(j => [j.id, j]));
  const done = comparePairs.filter(([a, c]) => byId[a] && byId[c] && !byId[a].running && !byId[c].running);
  if (!done.length) { el.innerHTML = comparePairs.length ? '<div class="hint">comparison running…</div>' : ''; return; }
  const [a, c] = done[done.length - 1]; const ra = byId[a].result || {}, rc = byId[c].result || {};
  const pa = (ra.metrics || {}).phases || {}, pc = (rc.metrics || {}).phases || {};
  const f = (v) => (typeof v === 'number' ? v.toFixed(3) : (v == null ? '—' : String(v)));
  let html = `<h4>Python vs JSBSim · ${esc(ra.scenario || '')} <span class="hint">python ${ra.ok ? 'ok' : 'FAILED'} · jsbsim ${rc.ok ? 'ok' : 'FAILED'}</span></h4><table class="grid"><thead><tr><th>phase</th><th>metric</th><th class="num">python</th><th class="num">jsbsim</th><th class="num">delta</th></tr></thead><tbody>`;
  for (const ph of new Set([...Object.keys(pa), ...Object.keys(pc)])) {
    if (ph === 'wait_ready') continue;
    for (const k of CMP_KEYS) {
      const x = (pa[ph] || {})[k], y = (pc[ph] || {})[k]; if (x == null && y == null) continue;
      const d = (typeof x === 'number' && typeof y === 'number') ? y - x : null;
      html += `<tr><td>${esc(ph)}</td><td>${k.replace(/_/g, ' ')}</td><td class="num">${f(x)}</td><td class="num">${f(y)}</td><td class="num ${d != null && Math.abs(d) > 0.2 * Math.max(Math.abs(x), 1e-9) && Math.abs(d) > 0.02 ? 'warn' : ''}">${d == null ? '—' : (d >= 0 ? '+' : '') + d.toFixed(3)}</td></tr>`;
    }
  }
  el.innerHTML = html + '</tbody></table>';
}
async function pollJobs() {
  clearTimeout(batchTimer);
  if (!$('#tab-batch').classList.contains('active')) return;
  try { const r = await api('/api/batch/jobs'); lastJobs = r; renderJobs(r); renderCompare(r.jobs || []); } catch (e) { $('#batch-jobs').innerHTML = `<div class="hint">${esc(e.message)}</div>`; }
  batchTimer = setTimeout(pollJobs, 1500);
}
function phaseSummary(res) {
  const m = (res && res.metrics) || {};
  const phases = m.phases || {};
  const rows = Object.entries(phases).map(([name, d]) => {
    const kv = KEY_METRICS.filter(k => d[k] != null).map(k => `<span title="${k}"><i>${k.replace(/_/g, ' ')}</i> ${mnum(d[k], k.includes('deg') || k === 'power_mean' ? 1 : 3)}</span>`).join('');
    return `<div class="phase-row"><b>${esc(name)}</b>${kv}</div>`;
  });
  const overall = [];
  if (m.crashed != null) overall.push(`<span class="${m.crashed ? 'err' : 'ok'}"><i>crashed</i> ${m.crashed ? 'yes' + (m.crash_reason ? ' (' + esc(m.crash_reason) + ')' : '') : 'no'}</span>`);
  if (m.energy_wh != null) overall.push(`<span><i>energy</i> ${mnum(m.energy_wh, 3)} Wh</span>`);
  if (m.max_tilt_deg != null) overall.push(`<span><i>max tilt</i> ${mnum(m.max_tilt_deg, 1)}°</span>`);
  if (m.saturation_fraction != null) overall.push(`<span><i>saturation</i> ${(m.saturation_fraction * 100).toFixed(0)}%</span>`);
  return (overall.length ? `<div class="phase-row">${overall.join('')}</div>` : '') + rows.join('');
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
    const detail = open ? `<tr class="detail"><td colspan="8">${j.running ? `<pre class="joblog">${(j.log || []).map(esc).join('\n')}</pre>` : `<pre class="joblog">${esc(JSON.stringify({ metrics: res.metrics, timing: res.timing, failures: res.failures, px4_params_verified: res.px4_params_verified, log: j.log }, null, 1))}</pre>`}</td></tr>` : '';
    return `<tr data-job="${esc(j.id)}" class="${open ? 'open' : ''}"><td class="idx mono">${esc(j.id)}</td><td>${esc(j.scenario)}</td><td>${esc(j.physics || 'python')}</td><td>${st}</td><td class="num">${mnum(simT, 1)} s</td><td class="num">${mnum(wall, 0)} s</td><td class="num">${t.rtf != null ? mnum(t.rtf, 1) : '—'}</td>
      <td class="metrics">${j.running ? `<span class="hint">${esc((j.log || []).slice(-1)[0] || 'starting…')}</span>` : phaseSummary(res) + fails}</td></tr>` + detail;
  }).join('');
  el.innerHTML = `<table class="grid jobs"><thead><tr><th>Job</th><th>Scenario</th><th title="physics engine">Physics</th><th>Status</th><th title="simulated time">Sim</th><th title="wall-clock time">Wall</th><th title="real-time factor">RTF</th><th>Metrics</th></tr></thead><tbody>${rows}</tbody></table>`;
  $$('#batch-jobs tr[data-job]').forEach(tr => tr.addEventListener('click', () => { const id = tr.dataset.job; if (jobsOpen.has(id)) jobsOpen.delete(id); else jobsOpen.add(id); if (lastJobs) renderJobs(lastJobs); }));
}

async function loadStudies() {
  const el = $('#study-list');
  try {
    const r = await api('/api/studies');
    studiesCache = r.studies || [];
  } catch (e) { el.innerHTML = `<div class="hint">${esc(e.message)}</div>`; studiesCache = []; return; }
  el.innerHTML = studiesCache.length ? studiesCache.map(s => `<div class="card sc"><div class="conn-row"><div><b>${esc(s.name)}</b> <span class="hint mono">${esc(s.file)}</span>
      <div class="hint">${esc(s.description || '')}</div>
      ${s.objective ? `<div class="hint"><i>objective</i> <code>${esc(s.objective)}</code></div>` : ''}
      <div class="phases">${(s.variables || []).map(v => `<span class="phase" title="${esc(JSON.stringify(v))}">${esc(v.path || v.name || JSON.stringify(v))}${v.range ? ' ' + esc(JSON.stringify(v.range)) : ''}</span>`).join('')}</div></div>
      <button class="pill small primary" data-study="${esc(s.file)}">Run study</button></div></div>`).join('')
    : '<div class="hint">No studies in studies/. A study is a JSON file with variables (parameter paths + ranges), scenarios and an objective.</div>';
  $$('#study-list button[data-study]').forEach(b => b.addEventListener('click', async () => {
    b.disabled = true;
    try {
      await api('/api/airframe', { airframe, keep_state: true });
      await api('/api/study/run', { spec: b.dataset.study, use_current_airframe: true });
      logLine('[study] started ' + b.dataset.study);
      pollStudy();
    } catch (e) { logLine('[study] ' + e.message); }
    b.disabled = false;
  }));
}
async function pollStudy() {
  clearTimeout(studyTimer);
  if (!$('#tab-batch').classList.contains('active')) return;
  let j;
  try { j = await api('/api/study/status'); } catch (e) { $('#study-status').innerHTML = `<div class="hint">${esc(e.message)}</div>`; return; }
  renderStudy(j);
  if (j.running) studyTimer = setTimeout(pollStudy, 1500);
}
function renderStudy(j) {
  const el = $('#study-status');
  if (!j.name && !j.running && !j.summary && !j.error) { el.innerHTML = ''; return; }
  const trials = j.trials || [];
  const best = (j.summary && j.summary.best) || (trials.length ? trials.reduce((a, t) => (t.score < a.score ? t : a), trials[0]) : null);
  const paths = best ? Object.keys(best.values || {}) : (trials[0] ? Object.keys(trials[0].values || {}) : []);
  const head = `<tr><th>#</th><th>Gen</th>${paths.map(p => `<th title="${esc(p)}">${esc(p.split('.').slice(-2).join('.'))}</th>`).join('')}<th>Objective</th><th>Score</th><th>Feasible</th></tr>`;
  const rows = trials.slice(-60).reverse().map((t, k) => `<tr class="${t === best ? 'pareto' : ''} ${t.feasible ? '' : 'infeasible'}"><td class="idx">${trials.length - k}</td><td class="num">${t.generation ?? ''}</td>${paths.map(p => `<td class="num">${mnum((t.values || {})[p], 3)}</td>`).join('')}
    <td class="num">${mnum(t.objective, 4)}</td><td class="num">${mnum(t.score, 4)}</td><td>${t.feasible ? '<span class="ok">yes</span>' : `<span class="err" title="${esc((t.violations || []).join(', '))}">no</span>`}</td></tr>`).join('');
  el.innerHTML = `<div class="card"><div class="conn-row"><div><b>${esc(j.name || 'study')}</b> ${j.running ? '<span class="warn">running</span>' : j.error ? `<span class="err">${esc(j.error)}</span>` : '<span class="ok">finished</span>'}
      ${j.summary ? `<div class="hint">${j.summary.evaluations} evaluations · ${j.summary.feasible} feasible${j.summary.best ? ` · best score ${mnum(j.summary.best.score, 4)}` : ''}</div>` : ''}</div>
      ${best ? `<button class="pill small primary" id="study-apply" title="apply the best trial's variables to the live airframe">Apply best</button>` : ''}</div>
    <pre class="joblog">${(j.log || []).map(esc).join('\n')}</pre>
    ${trials.length ? `<div class="tbl"><table class="grid trials"><thead>${head}</thead><tbody>${rows}</tbody></table></div>` : ''}</div>`;
  const ab = $('#study-apply');
  if (ab && best) ab.addEventListener('click', async () => {
    ab.disabled = true;
    try {
      const r = await api('/api/airframe/apply_variables', { variables: best.values });
      selected = -1; setAirframe(r.airframe); showProblems([]); markDirty();
      logLine('[study] applied best: ' + JSON.stringify(best.values));
      ab.textContent = 'Applied'; setTimeout(() => { ab.textContent = 'Apply best'; ab.disabled = false; }, 1500);
    } catch (e) { logLine('[study] apply failed: ' + e.message); ab.disabled = false; }
  });
}

// Keep scripted flights visible in the same scene as interactive flying.
const liveBadge = document.createElement('div');
liveBadge.style.cssText = 'position:absolute;top:12px;left:12px;background:#fff;color:#222;padding:10px 14px;border-radius:12px;z-index:4;display:none;box-shadow:0 2px 12px #0002';
const liveLabel = document.createElement('span');
const liveStop = document.createElement('button');
liveStop.textContent = 'Stop test'; liveStop.style.marginLeft = '12px';
liveStop.onclick = async () => { try { await api('/api/sim/scenario/stop', {}); } catch(e) { liveLabel.textContent = e.message; } };
liveBadge.append(liveLabel, liveStop); $('#viewport').append(liveBadge);
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
    const saved = await api('/api/params/set', { name: 'NLF_TARGET', value });
    if (!saved.ok || Math.abs(Number(saved.value) - value) > 0.01) throw new Error(saved.error || 'PX4 did not confirm the takeoff pitch');
    params.NLF_TARGET = { ...(params.NLF_TARGET || {}), value };
    airframe.px4_overrides = { ...(airframe.px4_overrides || {}), NLF_TARGET: value };
    logLine('[ui] Takeoff pitch saved: ' + value + '°');
  } catch (e) {
    logLine('[ui] ' + e.message);
    inp.value = (params.NLF_TARGET || {}).value ?? (airframe.px4_overrides || {}).NLF_TARGET ?? 25;
  }
});

['x', 'y', 'z'].forEach((axis, i) => bindNumber('imu-' + axis, v => {
  airframe.design ||= {};
  airframe.design.pixhawk_position ||= [...airframe.mass.cg];
  airframe.design.pixhawk_position[i] = v;
}));
