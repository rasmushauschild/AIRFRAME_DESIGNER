// 3D scene: vehicle body, rotors with drag gizmos, wing planforms, landing legs, CG marker, live thrust vectors.
// Frames: structural FRD (x fwd, y right, z down; origin = reference point)  ->  three.js (x, y=up, z)  via  (x, -z, y).
// The world frame is NED with the same mapping.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { frameOffsetForCg, spanCurve, wingOutlinePoints, builtAxis, mountError, axisToTiltCant, tiltCantToAxis } from './scene_math.js';

export const frdToThree = (v) => new THREE.Vector3(v[0], -v[2], v[1]);
export const threeToFrd = (v) => [v.x, v.z, -v.y];
const UP = new THREE.Vector3(0, 1, 0);

export function createScene(canvas, handlers) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  const scene = new THREE.Scene();
  const THEMES = {
    light: { bg: 0xf6f6f8, grid1: 0xb9bac4, grid2: 0xd3d4dc, ground: 0xf3f3f6, body: 0x3a3a3f, arm: 0x9a9aa3, motor: 0x2a2a2f },
    dark: { bg: 0x131419, grid1: 0x3a3d48, grid2: 0x262931, ground: 0x15161b, body: 0x4a4d57, arm: 0x8a8f9a, motor: 0x22242b },
  };
  let theme = THEMES.light;

  const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 500);
  camera.position.set(1.4, 1.0, 1.6);

  const orbit = new OrbitControls(camera, canvas);
  orbit.enableDamping = true;
  orbit.dampingFactor = 0.12;
  orbit.target.set(0, 0.1, 0);

  scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x1a1e26, 1.0));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6);
  sun.position.set(5, 10, 4);
  scene.add(sun);

  // ground
  // a modest grid that fades into the background with distance (fog on the line material), so far-away
  // lines never merge into a solid "ground" tint
  // "infinite" ground: a grid that follows the camera target, snapped to its cell size so the lines never appear
  // to slide, plus a coarse 10 m grid; both fade into the fog so the horizon stays clean wherever you fly
  const GRID_SIZE = 120, GRID_DIV = 120, GRID_CELL = GRID_SIZE / GRID_DIV, FOG_NEAR = 12, FOG_FAR = 60;
  let grid = new THREE.GridHelper(GRID_SIZE, GRID_DIV, theme.grid1, theme.grid2);
  grid.material.transparent = true; grid.material.opacity = 0.9;
  scene.add(grid);
  let gridCoarse = new THREE.GridHelper(GRID_SIZE, GRID_DIV / 10, theme.grid1, theme.grid1);
  gridCoarse.material.transparent = true; gridCoarse.material.opacity = 0.5; gridCoarse.position.y = 0.002;
  scene.add(gridCoarse);
  function followGrid() {
    const t = orbit.target;
    const gx = Math.round(t.x / GRID_CELL) * GRID_CELL, gz = Math.round(t.z / GRID_CELL) * GRID_CELL;
    grid.position.set(gx, 0, gz);
    gridCoarse.position.set(Math.round(t.x / (GRID_CELL * 10)) * GRID_CELL * 10, 0.002, Math.round(t.z / (GRID_CELL * 10)) * GRID_CELL * 10);
  }
  scene.background = new THREE.Color(theme.bg);
  scene.fog = new THREE.Fog(theme.bg, FOG_NEAR, FOG_FAR);
  // no ground plane: just the grid on the background colour
  // world axes: N (red x), E (blue z), up (green)
  scene.add(new THREE.AxesHelper(0.5));
  addLabel(scene, 'N', [0.55, 0.02, 0], 0xff3b30);
  addLabel(scene, 'E', [0, 0.02, 0.55], 0x0a84ff);

  // vehicle
  const vehicle = new THREE.Group();
  scene.add(vehicle);
  const frame = new THREE.Group();     // geometry only (edited)
  vehicle.add(frame);
  const armMat = new THREE.MeshStandardMaterial({ color: theme.arm, roughness: 0.7 });
  const matCCW = new THREE.MeshStandardMaterial({ color: 0x34c759, transparent: true, opacity: 0.35, side: THREE.DoubleSide, depthWrite: false });
  const matCW = new THREE.MeshStandardMaterial({ color: 0xff9500, transparent: true, opacity: 0.35, side: THREE.DoubleSide, depthWrite: false });
  const matSel = new THREE.MeshStandardMaterial({ color: 0x0a84ff, roughness: 0.4 });
  const motorMat = new THREE.MeshStandardMaterial({ color: theme.motor, roughness: 0.5, metalness: 0.5 });
  const ductMat = new THREE.MeshStandardMaterial({ color: 0x8a8f9a, roughness: 0.5, metalness: 0.3, transparent: true, opacity: 0.55, side: THREE.DoubleSide, depthWrite: false });
  const bodyAxes = new THREE.AxesHelper(0.25);
  frame.add(bodyAxes);

  let rotorNodes = [];   // { group, disc, motor, arrow, label, ring, rotorIndex }
  let legNodes = [];
  let wingNodes = [];
  let wingArrows = [];   // one force arrow per wing (index = wing index in airframe.wings)
  let cgNodes = [];
  // see-through surfaces must not write depth: a depth-writing transparent quad hides whatever is drawn after it, and
  // three.js re-sorts transparent objects every frame, so things behind the wing would pop in and out of view
  const wingMat = new THREE.MeshStandardMaterial({ color: 0x8a8f9a, roughness: 0.6, metalness: 0.2, transparent: true, opacity: 0.45, side: THREE.DoubleSide, depthWrite: false });
  const wingMatOff = new THREE.MeshStandardMaterial({ color: 0x8a8f9a, roughness: 0.6, metalness: 0.2, transparent: true, opacity: 0.12, side: THREE.DoubleSide, depthWrite: false });
  const wingSolidMat = new THREE.MeshStandardMaterial({ color: 0x9a9fab, roughness: 0.55, metalness: 0.25, side: THREE.DoubleSide });
  const airfoilCache = new Map();      // name -> [[x, y], ...] chord-normalised section coordinates
  const airfoilPending = new Set();
  function loadAirfoil(name, af) {
    if (airfoilCache.has(name) || airfoilPending.has(name)) return;
    airfoilPending.add(name);
    fetch('/api/airfoil/coords?name=' + encodeURIComponent(name)).then(r => r.json()).then(j => {
      airfoilPending.delete(name);
      if (j && j.ok) { airfoilCache.set(name, j.coords); if (airframe === af) setAirframe(af); }
    }).catch(() => airfoilPending.delete(name));
  }
  const wingEdgeMat = new THREE.LineBasicMaterial({ color: 0x6b6f7a });
  const wingEdgeMatOff = new THREE.LineBasicMaterial({ color: 0x6b6f7a, transparent: true, opacity: 0.3 });
  const legMat = new THREE.MeshStandardMaterial({ color: theme.arm, roughness: 0.7 });
  const legMatOff = new THREE.MeshStandardMaterial({ color: theme.arm, roughness: 0.7, transparent: true, opacity: 0.25, depthWrite: false });
  const footMat = new THREE.MeshStandardMaterial({ color: 0x5a5e69, roughness: 0.8, transparent: true, opacity: 0.6, depthWrite: false });
  const cgMat = new THREE.MeshStandardMaterial({ color: 0xff2d92, emissive: 0xff2d92, emissiveIntensity: 0.5, roughness: 0.4 });
  const pxMat = new THREE.MeshStandardMaterial({ color: 0xff9500, emissive: 0xff9500, emissiveIntensity: 0.25, roughness: 0.5 });
  const pxNoseMat = new THREE.MeshStandardMaterial({ color: 0xfff1d6, emissive: 0xffd28a, emissiveIntensity: 0.4, roughness: 0.5 });
  const flowMat = new THREE.MeshStandardMaterial({ color: 0x00c7be, emissive: 0x00c7be, emissiveIntensity: 0.35, roughness: 0.45 });
  const flowBeamMat = new THREE.MeshBasicMaterial({ color: 0x00c7be, transparent: true, opacity: 0.13, side: THREE.DoubleSide, depthWrite: false });
  const OFF_OPACITY = 0.22;   // disabled rotors
  let airframe = null;
  let selected = -1;
  // CAD bodies (STEP solids): kept across setAirframe (the meshes are heavy); positions/flags synced from af.cad
  const cadGroup = new THREE.Group();
  frame.add(cadGroup);
  const cadNodes = new Map();      // id -> { mesh, marker, centroid (FRD, offset-free) }
  let selectedCad = null;          // body id
  const cadMat = new THREE.MeshStandardMaterial({ color: 0xaab0bb, roughness: 0.55, metalness: 0.15, transparent: true, opacity: 0.85, depthWrite: true });
  const cadHeavyColor = new THREE.Color(0xff2020);
  const cadMarkerMat = new THREE.MeshStandardMaterial({ color: 0xff2d92, emissive: 0xff2d92, emissiveIntensity: 0.4, roughness: 0.4 });
  let camMode = 'follow';   // 'static' | 'track' (look at the drone) | 'follow' (move with it)
  // Turning around the aircraft and zooming in on it are still following it; dragging the view away from
  // it is not. There is no public way to ask OrbitControls which gesture is happening, but there is no
  // need: a pan moves its target and a rotation does not, so a target that has drifted from where we put
  // it last frame *is* the user taking the camera. That is when following stops and the button appears.
  let viewMoved = false, onViewMoved = null, settleUntil = 0;
  const lastVehiclePos = new THREE.Vector3();
  const lastTarget = new THREE.Vector3(NaN, NaN, NaN);

  // gizmo
  const gizmo = new TransformControls(camera, canvas);
  gizmo.setSize(0.55);
  gizmo.addEventListener('dragging-changed', (e) => { orbit.enabled = !e.value; if (!e.value) commitGizmo(); });
  gizmo.addEventListener('objectChange', () => onGizmoChange());
  const gizmoHelper = gizmo.getHelper ? gizmo.getHelper() : gizmo;
  scene.add(gizmoHelper);

  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  let downPos = null;
  canvas.addEventListener('pointerdown', (e) => { downPos = [e.clientX, e.clientY]; });
  canvas.addEventListener('pointerup', (e) => {
    if (!downPos) return;
    const moved = Math.hypot(e.clientX - downPos[0], e.clientY - downPos[1]);
    downPos = null;
    if (moved > 4 || gizmo.dragging) return;
    const r = canvas.getBoundingClientRect();
    pointer.x = ((e.clientX - r.left) / r.width) * 2 - 1;
    pointer.y = -((e.clientY - r.top) / r.height) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);
    const cadMeshes = cadGroup.visible ? [...cadNodes.values()].filter(n => n.mesh.visible).map(n => n.mesh) : [];
    const hits = raycaster.intersectObjects([...rotorNodes.flatMap(n => [n.disc, n.motor]), ...cadMeshes], false);
    if (hits.length && hits[0].object.userData.cadId !== undefined) {
      const id = hits[0].object.userData.cadId;
      selectCad(id, e.shiftKey);
      handlers.onCadSelect && handlers.onCadSelect(selectedCad);
    } else if (hits.length) {
      const idx = hits[0].object.userData.rotorIndex;
      select(idx);
      handlers.onSelect && handlers.onSelect(idx);
    } else if (!gizmo.axis) {
      select(-1);
      handlers.onSelect && handlers.onSelect(-1);
    }
  });
  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT' || e.target.tagName === 'TEXTAREA') return;
    if (e.key === 'w' || e.key === 'W') gizmo.setMode('translate');
    if ((e.key === 'e' || e.key === 'E') && selectedCad === null) gizmo.setMode('rotate');
    if (e.key === 'Escape') { select(-1); handlers.onSelect && handlers.onSelect(-1); if (selectedCad !== null) { selectCad(null); handlers.onCadSelect && handlers.onCadSelect(null); } }
  });

  function onGizmoChange() {
    if (selectedCad !== null) {
      const n = cadNodes.get(selectedCad); if (!n) return;
      const delta = n.mesh.position.clone().sub(cadDragPosition);
      cadDragPosition.copy(n.mesh.position);
      for (const id of selectedCadIds) {
        const body = cadNodes.get(id); if (!body) continue;
        if (id !== selectedCad) body.mesh.position.add(delta);
      }
      handlers.onCadGroupChanged && handlers.onCadGroupChanged([...selectedCadIds].map(id => ({ id, offset: cadOffset(cadNodes.get(id)) })), false);
      return;
    }
    if (selected < 0 || !airframe) return;
    const n = rotorNodes[selected];
    const r = airframe.rotors[selected];
    r.pos = threeToFrd(n.group.position).map(v => +v.toFixed(4));
    // The gizmo turns the rotor as it is built; what it writes is the design, with the mounting error
    // taken back off, so dragging a crooked rotor moves the drawing and leaves the crookedness alone.
    const ax = threeToFrd(UP.clone().applyQuaternion(n.group.quaternion)).map(v => +v.toFixed(4));
    const [dt, dc] = mountError(r);
    if (!dt && !dc) r.axis = ax;
    else {
      const [t, c] = axisToTiltCant(ax, r.pos[1]);
      r.axis = tiltCantToAxis(t - dt, c - dc, r.pos[1]);
    }
    showDesignGhost(n, r);
    updateArm(n, r);
    handlers.onRotorChanged && handlers.onRotorChanged(selected, r, false);
  }
  function commitGizmo() {
    if (selectedCad !== null) {
      const n = cadNodes.get(selectedCad); if (!n) return;
      handlers.onCadGroupChanged && handlers.onCadGroupChanged([...selectedCadIds].map(id => ({ id, offset: cadOffset(cadNodes.get(id)) })), true);
      return;
    }
    if (selected < 0 || !airframe) return;
    handlers.onRotorChanged && handlers.onRotorChanged(selected, airframe.rotors[selected], true);
  }

  // ------------------------------------------------------------ build
  function setAirframe(af) {
    airframe = af;
    const cg = (af.mass && af.mass.cg) || [0, 0, 0];
    frame.position.fromArray(frameOffsetForCg(cg));
    const keepSel = selected;
    gizmo.detach();
    for (const n of rotorNodes) frame.remove(n.group), frame.remove(n.arm);
    for (const l of legNodes) frame.remove(l);
    for (const w of wingNodes) frame.remove(w);
    for (const a of wingArrows) if (a) frame.remove(a);
    wingArrows = [];
    for (const c of cgNodes) frame.remove(c);
    rotorNodes = []; legNodes = []; wingNodes = []; cgNodes = [];

    // body.size is not drawn: it is a block the inertia estimate uses, not the shape of the aircraft, and
    // a black box over the CAD only ever hid the real thing.

    (af.rotors || []).forEach((r, i) => {
      const group = new THREE.Group();
      group.position.copy(frdToThree(r.pos));
      // where this rotor really pushes, not where it was drawn: a mounting error is a fact about the
      // aircraft, and the view is of the aircraft
      group.quaternion.setFromUnitVectors(UP, frdToThree(builtAxis(r)).normalize());
      const rad = Math.max(0.03, (r.diameter ?? r.prop_diameter ?? 0.25) / 2);
      const disc = new THREE.Mesh(new THREE.CylinderGeometry(rad, rad, 0.004, 40), (r.km >= 0 ? matCCW : matCW).clone());
      disc.position.y = 0.02;
      disc.userData.rotorIndex = i;
      const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.016, 0.03, 18), motorMat);
      motor.userData.rotorIndex = i;
      // every rotor is a ducted jet: a duct ring around a disc, both along the thrust axis
      const ring = new THREE.Mesh(new THREE.CylinderGeometry(rad * 1.08, rad * 1.08, rad * 1.1, 40, 1, true), ductMat);
      ring.position.y = 0.02;
      const ductHolder = new THREE.Group();
      ductHolder.add(ring);
      const arrow = makeThrustArrow();
      const spinArrow = makeSpinArrow(rad * 0.75, r.km >= 0);
      spinArrow.position.y = 0.024;
      ductHolder.add(disc, motor, spinArrow);
      const label = makeSprite(String(i + 1), document.documentElement.dataset.theme === 'dark' ? '#fafafa' : '#171717');
      label.position.set(0, 0.09, 0);
      const ghost = makeDesignGhost();                 // where PX4 thinks it points, when that is elsewhere
      group.add(ductHolder, arrow, label, ghost);
      const arm = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 1, 8), armMat);
      const node = { group, disc, motor, arrow, ring, label, arm, spinArrow, ductHolder, ghost, rad, rotorIndex: i, enabled: r.enabled !== false };
      showDesignGhost(node, r);
      updateArm(node, r);
      setRotorEnabled(node, r.enabled !== false);
      frame.add(group, arm);
      rotorNodes.push(node);
    });

    // landing legs: strut from the attachment to the foot, a sphere at the foot
    for (const l of af.legs || []) {
      if (!l || !l.attach) continue;
      const on = l.enabled !== false;
      const a = frdToThree(l.attach), f = frdToThree(legFoot(l));
      const d = f.clone().sub(a), len = d.length();
      if (len > 1e-4) {
        const strut = new THREE.Mesh(new THREE.CylinderGeometry(0.004, 0.004, len, 6), on ? legMat : legMatOff);
        strut.position.copy(a.clone().add(d.clone().multiplyScalar(0.5)));
        strut.quaternion.setFromUnitVectors(UP, d.clone().normalize());
        frame.add(strut); legNodes.push(strut);
      }
      const foot = new THREE.Mesh(new THREE.SphereGeometry(Math.max(0.01, +l.foot_radius || 0), 12, 12), on ? footMat : legMatOff);
      foot.position.copy(f);
      frame.add(foot); legNodes.push(foot);
    }

    // wings: real planform from the outline corners (root LE, tip LE, tip TE, root TE) of each half; wings with
    // airfoil polars are additionally lofted as a solid surface from their root and tip sections
    for (const w of af.wings || []) {
      if (!w || !w.pos) { wingArrows.push(null); continue; }
      const on = w.enabled !== false;
      let lofted = false;
      if (w.aero && w.aero.model === 'polar' && w.aero.airfoil_root) {
        const rootName = w.aero.airfoil_root, tipName = w.aero.airfoil_tip || rootName;
        const rc = airfoilCache.get(rootName), tc = airfoilCache.get(tipName);
        if (rc && tc) {
          for (const geo of wingLoft(w, rc, tc)) {
            const color = af.design?.visual?.wing_colors?.[(af.wings || []).indexOf(w)];
            const material = on && color ? wingSolidMat.clone() : (on ? wingSolidMat : wingMatOff);
            if (on && color) material.color.set(color);
            const mesh = new THREE.Mesh(geo, material);
            frame.add(mesh); wingNodes.push(mesh);
            lofted = true;
          }
        } else {
          for (const n of new Set([rootName, tipName])) loadAirfoil(n, af);
        }
      }
      // aerodynamic force vector of this wing (green), placed and scaled live from the simulation
      const fa = makeForceArrow(liftArrowMat);
      fa.visible = false;
      frame.add(fa); wingArrows.push(fa);
      for (const corners of wingOutline(w)) {
        const verts = corners.map(c => frdToThree(c));
        // The planform is filled in only when there is no lofted surface to look at. Drawing the
        // half-transparent quad over a solid wing is what made the wing look hazy and let the background
        // through it; with a loft, the planform is just its outline.
        if (!lofted) {
          const geo = new THREE.BufferGeometry().setFromPoints(verts);
          geo.setIndex([0, 1, 2, 0, 2, 3]); geo.computeVertexNormals();
          const mesh = new THREE.Mesh(geo, on ? wingMat : wingMatOff);
          mesh.renderOrder = 1;
          frame.add(mesh); wingNodes.push(mesh);
        }
        const edge = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(verts), on ? wingEdgeMat : wingEdgeMatOff);
        frame.add(edge);
        wingNodes.push(edge);
      }
    }

    // centre of gravity (structural frame) and the reference point (bodyAxes at the frame origin)
    const cgMark = new THREE.Mesh(new THREE.SphereGeometry(0.014, 16, 16), cgMat);
    cgMark.position.copy(frdToThree(cg));
    cgMark.renderOrder = 3;
    const cgLabel = makeSprite('CG', '#ff2d92');
    cgLabel.scale.set(0.05, 0.05, 1);
    cgLabel.position.copy(frdToThree(cg)).add(new THREE.Vector3(0, 0.05, 0));
    frame.add(cgMark, cgLabel);
    cgNodes.push(cgMark, cgLabel);
    // flight controller: a small orange board with its forward edge marked, at design.pixhawk_position (structural frame)
    const pxPos = af.design && af.design.pixhawk_position;
    if (pxPos && pxPos.length === 3) {
      const fc = new THREE.Group();
      const board = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.014, 0.036), pxMat);      // 50 x 36 mm, 14 mm tall (x = forward, z = right)
      const nose = new THREE.Mesh(new THREE.BoxGeometry(0.006, 0.016, 0.036), pxNoseMat);  // forward edge
      nose.position.x = 0.025;
      const label = makeSprite('PX4', '#ff9500');
      label.scale.set(0.06, 0.06, 1); label.position.set(0, 0.045, 0);
      fc.add(board, nose, label);
      fc.position.copy(frdToThree(pxPos));
      fc.renderOrder = 3;
      frame.add(fc);
      cgNodes.push(fc);
    }

    // H-flow sensor: the lens where it sits, and the cone it looks down. Its own axes are turned into
    // structural axes by roll-pitch-yaw exactly as geometry/airframe.py mount_rotation() does it, so what
    // is drawn here is the sensor the simulation reads and the one PX4 is told about.
    const fl = af.design && af.design.flow_sensor;
    if (fl && fl.enabled && (fl.position || []).length === 3) {
      const [rr, rp, ry] = [0, 1, 2].map(i => (((fl.rotation || [])[i] || 0) * Math.PI) / 180);
      const [cr, sr, cp, sp, cy, sy] = [Math.cos(rr), Math.sin(rr), Math.cos(rp), Math.sin(rp), Math.cos(ry), Math.sin(ry)];
      const ex = [cy * cp, sy * cp, -sp];                                        // the sensor's own x, in structural axes
      const ez = [cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr];      // and the way it looks
      const g = new THREE.Group();
      const look = frdToThree(ez).normalize(), fwd = frdToThree(ex).normalize();
      g.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(fwd, look, fwd.clone().cross(look)));
      const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.013, 0.013, 0.006, 20), flowMat);   // the lens faces the way it looks
      const tick = new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.005, 0.004), flowMat);            // and this is its own forward
      tick.position.x = 0.017;
      const L = 0.3;                        // the cone shows which way it looks, not how far it reads
      const beam = new THREE.Mesh(new THREE.ConeGeometry(L * Math.tan(0.37), L, 24, 1, true), flowBeamMat);
      beam.position.y = L / 2; beam.rotation.x = Math.PI;        // apex at the lens, opening out along the look
      const label = makeSprite('FLOW', '#00a79e');
      label.scale.set(0.06, 0.06, 1);
      g.add(lens, tick, beam);
      g.position.copy(frdToThree(fl.position));
      g.renderOrder = 3;
      label.position.copy(g.position).add(new THREE.Vector3(0, 0.05, 0));
      frame.add(g, label);
      cgNodes.push(g, label);
    }

    syncCad(af);
    if (selectedCad !== null && cadNodes.has(selectedCad)) selectCad(selectedCad, false, true);
    else if (keepSel >= 0 && keepSel < rotorNodes.length) select(keepSel);
  }

  const selectedCadIds = new Set();
  const cadDragPosition = new THREE.Vector3();

  // ------------------------------------------------------------ CAD bodies
  const cadOffset = (n) => threeToFrd(n.mesh.position).map((v, i) => +(v - n.centroid[i]).toFixed(4));
  function clearCad() {
    for (const n of cadNodes.values()) { cadGroup.remove(n.mesh); n.mesh.geometry.dispose(); n.mesh.material.dispose(); n.marker.geometry.dispose(); }
    cadNodes.clear();
    selectedCadIds.clear();
    if (selectedCad !== null) { selectedCad = null; gizmo.detach(); }
  }
  // payload = /api/cad/mesh: bodies with vertices/indices in the structural frame (offsets not applied)
  function setCad(payload) {
    clearCad();
    for (const b of (payload && payload.bodies) || []) {
      const g = new THREE.BufferGeometry();
      const v = b.vertices, pos = new Float32Array(v.length);
      for (let i = 0; i < v.length; i += 3) { pos[i] = v[i]; pos[i + 1] = -v[i + 2]; pos[i + 2] = v[i + 1]; }   // FRD -> three
      g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
      g.setIndex(v.length / 3 > 65535 ? new THREE.Uint32BufferAttribute(b.indices, 1) : new THREE.Uint16BufferAttribute(b.indices, 1));
      g.computeVertexNormals();
      // the mesh origin is the body's centroid, so the drag gizmo sits on the body and moving it moves the mass point
      const c = frdToThree(b.centroid);
      g.translate(-c.x, -c.y, -c.z);
      const mesh = new THREE.Mesh(g, cadMat.clone());
      mesh.position.copy(c);
      mesh.userData.cadId = b.id;
      const marker = new THREE.Mesh(new THREE.SphereGeometry(0.008, 12, 12), cadMarkerMat);
      marker.visible = false;
      mesh.add(marker);
      cadGroup.add(mesh);
      cadNodes.set(b.id, { mesh, marker, centroid: b.centroid.slice() });
    }
    if (airframe) syncCad(airframe);
  }
  function syncCad(af) {
    const cad = af && af.cad;
    const showCad = !!(cad && cad.file && cad.visible !== false);
    if (!cad || !cad.file) { if (cadNodes.size) clearCad(); return; }
    cadGroup.visible = showCad;
    const maxMass = Math.max(0, ...(cad.bodies || []).filter(b => !b.removed).map(b => Math.max(0, +b.mass || 0)));
    for (const b of cad.bodies || []) {
      const n = cadNodes.get(b.id); if (!n) continue;
      if (!gizmo.dragging || !selectedCadIds.has(b.id)) n.mesh.position.copy(frdToThree(n.centroid.map((v, i) => v + ((b.offset || [0, 0, 0])[i] || 0))));
      const massFraction = maxMass > 0 ? Math.max(0, +b.mass || 0) / maxMass : 0;
      n.mesh.material.color.copy(cadMat.color).lerp(cadHeavyColor, massFraction);
      n.mesh.visible = !b.removed;
      n.marker.visible = !b.removed && (+b.mass || 0) > 0;
    }
  }
  function selectCad(id, additive = false, preserve = false) {
    if (id !== null && !cadNodes.has(id)) id = null;
    if (id !== null && selected >= 0) select(-1);
    if (!preserve) {
      if (!additive) selectedCadIds.clear();
      if (id !== null) {
        if (additive && selectedCadIds.has(id)) selectedCadIds.delete(id);
        else selectedCadIds.add(id);
      }
    }
    selectedCad = selectedCadIds.has(id) ? id : ([...selectedCadIds].pop() ?? null);
    id = selectedCad;
    for (const [k, n] of cadNodes) {
      n.mesh.material.emissive.setHex(selectedCadIds.has(k) ? 0x0a84ff : 0x000000);
      n.mesh.material.emissiveIntensity = selectedCadIds.has(k) ? 0.25 : 0;
    }
    if (id !== null) { gizmo.setMode('translate'); gizmo.setSpace('local'); gizmo.attach(cadNodes.get(id).mesh); cadDragPosition.copy(cadNodes.get(id).mesh.position); }
    else if (selected < 0) gizmo.detach();
  }

  // A requested motor test, not measured rotation feedback from the aircraft.
  function highlightMotorTest(index = -1) {
    rotorNodes.forEach((node, i) => {
      const color = i === index && node.enabled ? 0xffd60a : 0x0a84ff;
      node.arrow.traverse(mesh => {
        if (!mesh.material) return;
        if (!mesh.userData.ownMat) {
          mesh.material = mesh.material.clone();
          mesh.userData.ownMat = true;
        }
        mesh.material.color.setHex(color);
        mesh.material.emissive.setHex(color);
      });
    });
  }

  function setRotorEnabled(node, on) {
    node.enabled = on;
    const alpha = on ? 1 : OFF_OPACITY;
    node.disc.material.opacity = on ? 0.35 : 0.12;
    [node.arrow, node.spinArrow, node.ring, node.motor].forEach(o => o.traverse(m => {
      if (!m.material) return;
      if (!m.userData.ownMat) { m.userData.baseOpacity = m.material.opacity; m.material = m.material.clone(); m.userData.ownMat = true; }
      m.material.transparent = true; m.material.opacity = (m.userData.baseOpacity ?? 1) * alpha;
    }));
    node.label.material.opacity = on ? 1 : 0.4;
  }

  function updateArm(node, r) {
    node.arm.visible = false;          // a duct sits on its thrust origin; there is nothing to connect it to
  }

  function updateRotorNode(i, r) {   // called when the table edits a rotor
    const n = rotorNodes[i];
    if (!n) return;
    n.group.position.copy(frdToThree(r.pos));
    n.group.quaternion.setFromUnitVectors(UP, frdToThree(builtAxis(r)).normalize());
    showDesignGhost(n, r);
    const m = r.km >= 0 ? matCCW : matCW;
    n.disc.material = m.clone();
    updateArm(n, r);
    setRotorEnabled(n, r.enabled !== false);
  }

  function select(i) {
    selected = i;
    if (i >= 0 && selectedCad !== null) { selectedCad = null; selectedCadIds.clear(); for (const n of cadNodes.values()) n.mesh.material.emissive.setHex(0x000000); gizmo.setSpace('world'); }
    rotorNodes.forEach((n, k) => { n.motor.material = (k === i ? matSel : motorMat).clone(); n.motor.userData.ownMat = true; n.motor.material.transparent = true; n.motor.material.opacity = n.enabled ? 1 : OFF_OPACITY; });
    if (i >= 0 && rotorNodes[i]) gizmo.attach(rotorNodes[i].group);
    else if (selectedCad === null) gizmo.detach();
  }

  // ------------------------------------------------------------ live
  const tmpQ = new THREE.Quaternion();
  function updateState(st) {
    if (!st) return;
    vehicle.position.set(st.pos[0], -st.pos[2], st.pos[1]);
    const [w, x, y, z] = st.q;
    tmpQ.set(x, -z, y, w);
    vehicle.quaternion.copy(tmpQ);
    const weight = airframe && airframe.mass ? (+airframe.mass.mass || 1) * 9.80665 : 1;
    const wf = (st.forces && st.forces.wing_forces) || [];
    wingArrows.forEach((arrow, i) => {
      if (!arrow) return;
      const f = wf[i];
      const mag = f ? Math.hypot(f.F[0], f.F[1], f.F[2]) : 0;
      if (!f || mag < 0.02 * weight) { arrow.visible = false; return; }
      arrow.visible = true;
      arrow.position.copy(frdToThree(f.pos));
      arrow.quaternion.setFromUnitVectors(UP, frdToThree(f.F).normalize());
      const frac = Math.min(1.5, mag / weight);
      setThrustArrow(arrow, 0.06 + 0.5 * frac, Math.min(1, frac));
    });
    // The arrows are how hard each motor is working. In a simulated flight that is the thrust it makes;
    // a real aircraft reports its *output* and never the thrust, so a flight watched over a radio and a
    // log read off the card carry a thrust of zero all the way down. When no rotor has any thrust the
    // arrows follow the outputs instead - still a number out of the flight, never one worked out from
    // the other - and in a simulated run that is a moment with every motor off, where they sit at rest
    // whichever way it is read.
    const rotorStates = st.rotors || [];
    const byOutput = rotorStates.length > 0 && rotorStates.every((rs) => !rs.thrust);
    rotorStates.forEach((rs, i) => {
      const n = rotorNodes[i];
      if (!n || !airframe || !airframe.rotors[i]) return;
      const r = airframe.rotors[i];
      const frac = byOutput ? Math.max(0, Math.min(1, +rs.cmd || 0))
                 : (r.max_thrust > 0 ? rs.thrust / r.max_thrust : 0);
      setThrustArrow(n.arrow, ARROW_BASE + frac * ARROW_GROW, frac);
      n.disc.rotation.y += (r.km >= 0 ? 1 : -1) * rs.omega * 0.6;
      n.disc.material.opacity = n.enabled ? 0.25 + 0.5 * rs.omega : 0.12;
    });
    if (camMode === 'track') {
      orbit.target.lerp(vehicle.position, 0.15);
    } else if (camMode === 'follow') {
      // after being handed back, the controls are still easing out of the last gesture: that tail is not
      // the user panning again, so the check waits for it to die down
      if (isFinite(lastTarget.x) && performance.now() > settleUntil && orbit.target.distanceTo(lastTarget) > 1e-4) {
        camMode = 'static';                       // panned away: the view is the user's now
        if (!viewMoved) { viewMoved = true; if (onViewMoved) onViewMoved(); }
      } else {
        camera.position.add(vehicle.position.clone().sub(lastVehiclePos));
        orbit.target.copy(vehicle.position);
        lastTarget.copy(orbit.target);
      }
    }
    lastVehiclePos.copy(vehicle.position);
  }

  // ------------------------------------------------------------ loop
  function resize() {
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (canvas.width !== w || canvas.height !== h) {
      renderer.setSize(w, h, false);
      camera.aspect = w / h; camera.updateProjectionMatrix();
    }
  }
  const axesInset = makeAxesInset();
  const windInset = makeWindInset();
  function animate() {
    resize();
    orbit.update();
    followGrid();
    renderer.setScissorTest(false);
    renderer.render(scene, camera);
    axesInset.render(renderer, camera, canvas);
    windInset.render(renderer, camera, canvas);
    requestAnimationFrame(animate);
  }
  animate();

  function setTheme(name) {
    theme = THEMES[name] || THEMES.light;
    scene.background = new THREE.Color(theme.bg);
    scene.fog = new THREE.Fog(theme.bg, FOG_NEAR, FOG_FAR);
    scene.remove(grid); scene.remove(gridCoarse);
    grid = new THREE.GridHelper(GRID_SIZE, GRID_DIV, theme.grid1, theme.grid2);
    grid.material.transparent = true; grid.material.opacity = 0.9;
    gridCoarse = new THREE.GridHelper(GRID_SIZE, GRID_DIV / 10, theme.grid1, theme.grid1);
    gridCoarse.material.transparent = true; gridCoarse.material.opacity = 0.5;
    scene.add(grid, gridCoarse);
    axesInset.setTheme(name);
    windInset.setTheme(name);
    armMat.color.set(theme.arm); motorMat.color.set(theme.motor); legMat.color.set(theme.arm); legMatOff.color.set(theme.arm);
    rotorNodes.forEach(n => { n.label.material.map = makeSprite(String(n.rotorIndex + 1), name === 'dark' ? '#fafafa' : '#171717', name === 'dark').material.map; });
  }
  setTheme(document.documentElement.dataset.theme === 'dark' ? 'dark' : 'light');

  return {
    setAirframe, updateState, select, updateRotorNode, setTheme, highlightMotorTest,
    setWind: (ned) => windInset.setWind(ned),
    get selectedCadIds() { return [...selectedCadIds]; },
    setCad, selectCad, syncCad: () => airframe && syncCad(airframe),
    get selectedCad() { return selectedCad; },
    setCameraMode: (m) => {
      camMode = m;
      if (m === 'static') orbit.target.set(0, 0.1, 0);
      if (m === 'follow') {
        // Acquire the vehicle even when follow is enabled after it has flown far from the origin.
        const size = new THREE.Box3().setFromObject(frame).getSize(new THREE.Vector3());
        const distance = Math.max(2, size.length() * 1.6);
        const offset = new THREE.Vector3(1.4, 1.0, 1.6).normalize().multiplyScalar(distance);
        camera.position.copy(vehicle.position).add(offset);
        orbit.target.copy(vehicle.position);
        lastVehiclePos.copy(vehicle.position);
        orbit.update();
        lastTarget.copy(orbit.target);
      }
    },
    setFollow: (b) => { camMode = b ? 'track' : 'static'; if (!b) orbit.target.set(0, 0.1, 0); },
    onCameraMoved: (cb) => { onViewMoved = cb; },
    recenterView() { viewMoved = false; settleUntil = performance.now() + 600; this.setCameraMode('follow'); },
    setMode: (m) => gizmo.setMode(m),
    get selected() { return selected; },
    focusOrigin: () => { orbit.target.set(0, 0.1, 0); camera.position.set(1.4, 1.0, 1.6); },
  };
}

// Thrust vector: a solid arrow along the rotor axis (local +Y). Always visible at ARROW_BASE length so the
// direction reads at a glance; grows with live thrust and brightens as the motor spins up.
const ARROW_BASE = 0.16, ARROW_GROW = 0.22, ARROW_R = 0.007;
const arrowMat = new THREE.MeshStandardMaterial({ color: 0x0a84ff, emissive: 0x0a84ff, emissiveIntensity: 0.35, roughness: 0.4 });
const liftArrowMat = new THREE.MeshStandardMaterial({ color: 0x34c759, emissive: 0x34c759, emissiveIntensity: 0.35, roughness: 0.4 });
function makeForceArrow(mat) {
  const g = new THREE.Group();
  const shaft = new THREE.Mesh(new THREE.CylinderGeometry(ARROW_R * 1.3, ARROW_R * 1.3, 1, 12), mat);
  const head = new THREE.Mesh(new THREE.ConeGeometry(ARROW_R * 4, ARROW_R * 8, 16), mat);
  const base = new THREE.Mesh(new THREE.SphereGeometry(ARROW_R * 2, 12, 12), mat);
  g.add(shaft, head, base);
  g.userData = { shaft, head };
  g.renderOrder = 2;
  setThrustArrow(g, ARROW_BASE, 0);
  return g;
}
// The axis PX4 was given, drawn only when the rotor is not mounted on it: a thin dashed stick out of the
// hub, so the gap between what the aircraft does and what its controller believes is a visible angle.
const ghostMat = new THREE.LineDashedMaterial({ color: 0xff9500, dashSize: 0.012, gapSize: 0.008, transparent: true, opacity: 0.9 });
function makeDesignGhost() {
  const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0.16, 0)]);
  const line = new THREE.Line(geo, ghostMat);
  line.computeLineDistances();
  line.visible = false;
  line.renderOrder = 4;
  return line;
}
function showDesignGhost(node, r) {
  const g = node.ghost;
  if (!g) return;
  const [dt, dc] = mountError(r);
  g.visible = !!(dt || dc) && node.enabled !== false;
  if (!g.visible) return;
  // the group is already turned to the built axis, so the design is a direction inside it
  const design = frdToThree(r.axis).normalize().applyQuaternion(node.group.quaternion.clone().invert());
  g.quaternion.setFromUnitVectors(UP, design);
}
function makeThrustArrow() {
  const g = makeForceArrow(arrowMat);
  g.position.y = 0.02;
  return g;
}
function setThrustArrow(g, len, frac) {
  const { shaft, head } = g.userData;
  const headLen = ARROW_R * 7;
  const shaftLen = Math.max(0.001, len - headLen);
  shaft.scale.y = shaftLen; shaft.position.y = shaftLen / 2;
  head.position.y = shaftLen + headLen / 2;
  const s = 1 + frac * 0.6;
  head.scale.set(s, 1, s);
}

// Coordinate indicator: a small scene with the world axes (N red, E blue, Up green) rendered in the
// bottom-right corner with a camera that copies the main camera's orientation, so it always shows where north and up are.
function makeAxesInset() {
  const scene = new THREE.Scene();
  const cam = new THREE.PerspectiveCamera(44, 1, 0.1, 20);
  const group = new THREE.Group();
  const mk = (dir, color) => { const a = new THREE.ArrowHelper(dir, new THREE.Vector3(), 1.2, color, 0.36, 0.18); group.add(a); };
  mk(new THREE.Vector3(1, 0, 0), 0xff3b30);   // N  (FRD x)
  mk(new THREE.Vector3(0, 0, 1), 0x0a84ff);   // E  (FRD y)
  mk(new THREE.Vector3(0, 1, 0), 0x34c759);   // Up (FRD -z)
  // arrows only (red = north / FRD x, blue = east / FRD y, green = up), no text
  scene.add(group);
  scene.add(new THREE.AmbientLight(0xffffff, 1));
  const dir = new THREE.Vector3();
  return {
    render(renderer, mainCam, canvas) {
      // hard into the bottom-right corner, drawn over the main scene without clearing the colour (so no
      // dark square), with the camera just far enough back that the arrows never leave the little frustum
      // whatever the view direction. three.js viewport/scissor coordinates are CSS pixels.
      // A small box in the bottom-right corner, far enough in that the rounded corner of the panel does not
      // cut the arrows off, and with the camera back far enough that the triad stays inside the box.
      const sz = renderer.getSize(new THREE.Vector2());
      const size = 100, off = 1.32;          // the triad sits at 0.66 of the box: 34 px from the corner
      const x = sz.x - size, y = 0;
      mainCam.getWorldDirection(dir);
      cam.position.copy(dir).multiplyScalar(-6.6 * off); cam.up.copy(mainCam.up); cam.lookAt(0, 0, 0);
      // Render the top-left part of a wider frustum to move the triad towards the corner, with the camera
      // pulled back by the same factor so that crop does not magnify it into the rounded corner.
      cam.setViewOffset(size * off, size * off, 0, 0, size, size);
      cam.updateProjectionMatrix();
      const autoClear = renderer.autoClear;
      renderer.autoClear = false;
      renderer.clearDepth();
      renderer.setScissorTest(true);
      renderer.setScissor(x, y, size, size);
      renderer.setViewport(x, y, size, size);
      renderer.render(scene, cam);
      renderer.setScissorTest(false);
      renderer.setViewport(0, 0, sz.x, sz.y);
      renderer.autoClear = autoClear;
    },
    setTheme() { },
  };
}

// The wind, beside the coordinate indicator: one arrow pointing where the air is going. How hard it blows
// is the length and the colour - pale grey in a breath of air, deep blue at 12 m/s and above - so there is
// no text to read in a 100-pixel box. The number is on the overlay above the view.
function makeWindInset() {
  const scene = new THREE.Scene();
  const cam = new THREE.PerspectiveCamera(44, 1, 0.1, 20);
  const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1.2, 0x94a3b8, 0.38, 0.22);
  // Still air is a fact about the flight, not an absence of one: the box says so instead of going blank.
  const calmLabel = makeWindLabel('no wind');
  scene.add(arrow, calmLabel, new THREE.AmbientLight(0xffffff, 1));
  const dir = new THREE.Vector3();
  const calm = new THREE.Color(0x94a3b8), blown = new THREE.Color(0x0369a1), shade = new THREE.Color();
  const FULL = 12;                                  // the speed at which the arrow is as long and as deep as it gets
  return {
    setWind(ned) {
      const speed = ned && ned.length >= 2 ? Math.hypot(ned[0], ned[1], ned[2] || 0) : 0;
      calmLabel.visible = speed < 0.05;
      arrow.visible = !calmLabel.visible;
      if (calmLabel.visible) return;
      // NED -> three (x north, y up, z east); the arrow points where the air is going
      arrow.setDirection(new THREE.Vector3(ned[0], -(ned[2] || 0), ned[1]).normalize());
      const f = Math.min(1, speed / FULL);
      arrow.setLength(0.45 + 0.85 * f, 0.3 + 0.1 * f, 0.18 + 0.06 * f);
      arrow.setColor(shade.copy(calm).lerp(blown, f));
    },
    render(renderer, mainCam) {
      if (!arrow.visible && !calmLabel.visible) return;
      // The same camera and the same crop as the axes, in a box 58 px to their left: the two arrows are then
      // drawn at the same scale and sit on the same line, which is what makes them read as one indicator.
      const sz = renderer.getSize(new THREE.Vector2());
      const size = 100, off = 1.32, gap = 58;
      const x = Math.max(0, sz.x - size - gap), y = 0;
      mainCam.getWorldDirection(dir);
      cam.position.copy(dir).multiplyScalar(-6.6 * off); cam.up.copy(mainCam.up); cam.lookAt(0, 0, 0);
      cam.setViewOffset(size * off, size * off, 0, 0, size, size);
      cam.updateProjectionMatrix();
      const autoClear = renderer.autoClear;
      renderer.autoClear = false;
      renderer.clearDepth();
      renderer.setScissorTest(true);
      renderer.setScissor(x, y, size, size);
      renderer.setViewport(x, y, size, size);
      renderer.render(scene, cam);
      renderer.setScissorTest(false);
      renderer.setViewport(0, 0, sz.x, sz.y);
      renderer.autoClear = autoClear;
    },
    setTheme(name) { calmLabel.material.map = makeWindLabel('no wind', name === 'dark').material.map; },
  };
}

// Plain words for the wind box, at the size the rest of the app writes in. No box around them: there is
// nothing to press, it is just what the air is doing.
function makeWindLabel(text, dark = document.documentElement.dataset.theme === 'dark') {
  const FONT = '600 40px -apple-system, BlinkMacSystemFont, "SF Pro Text", sans-serif';
  const c = document.createElement('canvas');
  const probe = c.getContext('2d');
  probe.font = FONT;
  const w = Math.ceil(probe.measureText(text).width) + 8, h = 56;
  c.width = w; c.height = h;
  const g = c.getContext('2d');
  g.font = FONT;
  g.fillStyle = dark ? 'rgba(235,235,240,0.80)' : 'rgba(90,95,105,0.80)';
  g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillText(text, w / 2, h / 2);
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthTest: false }));
  // the inset box is about 19 screen pixels per world unit: 0.62 units is a 12 pixel line of text
  sprite.scale.set(0.62 * (w / h), 0.62, 1);
  return sprite;
}

function makeSprite(text, color = '#171717', dark = document.documentElement.dataset.theme === 'dark') {
  const c = document.createElement('canvas'); c.width = 64; c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = dark ? 'rgba(18,19,23,0.92)' : 'rgba(255,255,255,0.92)'; g.beginPath(); g.arc(32, 32, 28, 0, Math.PI * 2); g.fill();
  g.strokeStyle = dark ? 'rgba(255,255,255,0.15)' : 'rgba(0,0,0,0.12)'; g.lineWidth = 2; g.stroke();
  const font = (px) => `700 ${px}px -apple-system, BlinkMacSystemFont, "SF Pro Text", sans-serif`;
  g.fillStyle = color; g.font = font(32); g.textAlign = 'center'; g.textBaseline = 'middle';
  const wide = g.measureText(text).width;                     // a longer word shrinks to fit the badge
  if (wide > 48) g.font = font(Math.floor(32 * 48 / wide));
  g.fillText(text, 32, 34);
  const tex = new THREE.CanvasTexture(c);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false }));
  s.scale.set(0.06, 0.06, 1);
  return s;
}
function addLabel(scene, text, pos, color) {
  const s = makeSprite(text, '#' + color.toString(16).padStart(6, '0'));
  s.position.set(...pos); s.scale.set(0.12, 0.12, 1);
  scene.add(s);
}
function makeSpinArrow(radius, ccw) {
  // arc with an arrowhead showing spin direction when viewed from above (three: looking down -y)
  const pts = [];
  const a0 = 0, a1 = Math.PI * 1.4;
  for (let k = 0; k <= 24; k++) {
    const a = a0 + (a1 - a0) * k / 24;
    // CCW from above: in three coords (x right? no) -> viewed from +y looking down, x to the right and z toward viewer.
    // A rotation that is CCW as seen from above corresponds to positive rotation about +y (right hand rule): x -> -z.
    const s = ccw ? 1 : -1;
    pts.push(new THREE.Vector3(radius * Math.cos(a), 0, -s * radius * Math.sin(a)));
  }
  const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),
    new THREE.LineBasicMaterial({ color: ccw ? 0x34c759 : 0xff9500 }));
  const tip = pts[pts.length - 1], prev = pts[pts.length - 2];
  const dir = tip.clone().sub(prev).normalize();
  const head = new THREE.ArrowHelper(dir, prev, 0.02, ccw ? 0x34c759 : 0xff9500, 0.02, 0.012);
  const g = new THREE.Group(); g.add(line, head);
  return g;
}

// ------------------------------------------------------------ geometry helpers (structural frame, plain arrays)
const rad = (d) => (+d || 0) * Math.PI / 180;
// foot point of a leg: attach + length * dir, dir = [sin(tilt), side*sin(cant)*cos(tilt), cos(cant)*cos(tilt)] (see geometry/gear.py)
export function legFoot(l) {
  const side = l.attach[1] < 0 ? -1 : 1;
  const t = rad(l.tilt_deg), c = rad(l.cant_deg) * side;
  const d = [Math.sin(t), Math.sin(c) * Math.cos(t), Math.cos(c) * Math.cos(t)];
  return d.map((v, k) => (+l.attach[k] || 0) + (+l.length || 0) * v);
}
// corner points of each wing half (root LE, tip LE, tip TE, root TE), port of geometry/wings.py wing_outline()
// Resample a section polyline to n points by normalised index (root and tip must match point for point).
function resampleSection(pts, n) {
  const out = [];
  for (let k = 0; k < n; k++) {
    const f = k * (pts.length - 1) / (n - 1), i = Math.min(pts.length - 2, Math.floor(f)), t = f - i;
    out.push([pts[i][0] * (1 - t) + pts[i + 1][0] * t, pts[i][1] * (1 - t) + pts[i + 1][1] * t]);
  }
  return out;
}
// Lofted airfoil surface of a wing: sections placed along the span with chord, incidence + twist, sweep and
// dihedral applied, root and tip section shapes blended linearly. Returns one BufferGeometry per half (FRD -> three).
export function wingLoft(w, rootPts, tipPts, nSpan = 10, nPts = 64) {
  const geos = [];
  const sym = w.symmetric !== false;
  const root = new THREE.Vector3(+w.pos[0] || 0, +w.pos[1] || 0, +w.pos[2] || 0);
  const rc = +w.root_chord || 0, tc = +w.tip_chord || 0, inc0 = +w.incidence_deg || 0, tw = +w.twist_deg || 0;
  const R = resampleSection(rootPts, nPts), T = resampleSection(tipPts, nPts);
  for (const sd of (sym ? [1, -1] : [1])) {
    const verts = [];
    for (let j = 0; j <= nSpan; j++) {
      const f = j / nSpan, c = rc + (tc - rc) * f, inc = inc0 + tw * f;
      const sc = spanCurve(w, f, sd);
      const off = new THREE.Vector3(...sc.off), es = new THREE.Vector3(...sc.es);
      const pitchAxis = es.clone().multiplyScalar(sd);
      const en0 = new THREE.Vector3().crossVectors(pitchAxis, new THREE.Vector3(1, 0, 0));   // "up" of the panel
      const ec = new THREE.Vector3(1, 0, 0).applyAxisAngle(pitchAxis, rad(inc));
      const en = en0.clone().applyAxisAngle(pitchAxis, rad(inc));
      const le = root.clone().add(new THREE.Vector3(0, sd * (+w.root_y || 0), 0)).add(off);
      const pr = rad(w.pitch_deg || 0), yAxis = new THREE.Vector3(0, 1, 0);
      for (let k = 0; k < nPts; k++) {
        const x = R[k][0] * (1 - f) + T[k][0] * f, y = R[k][1] * (1 - f) + T[k][1] * f;
        const p = le.clone().sub(ec.clone().multiplyScalar(x * c)).add(en.clone().multiplyScalar(y * c));
        p.sub(root).applyAxisAngle(yAxis, pr).add(root);                       // whole-wing pitch about the root point
        verts.push(p.x, -p.z, p.y);                                            // FRD -> three
      }
    }
    const idx = [];
    for (let j = 0; j < nSpan; j++) for (let k = 0; k < nPts - 1; k++) {
      const a = j * nPts + k, b = a + 1, c2 = a + nPts, d = c2 + 1;
      idx.push(a, c2, b, b, c2, d);
    }
    // close the tip with a fan (and the root for a single panel) so the section shape reads clearly
    const capAt = (j0) => { const base = j0 * nPts; for (let k = 1; k < nPts - 1; k++) idx.push(base, base + k, base + k + 1); };
    capAt(nSpan); if (!sym) capAt(0);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
    geo.setIndex(idx); geo.computeVertexNormals();
    geos.push(geo);
  }
  return geos;
}

export function wingOutline(w) {
  return wingOutlinePoints(w);   // shared with geometry/wings.py through ui/scene_math.js
}

