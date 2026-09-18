"""Create a session-specific UI copy; do not edit the user's shared app."""
from pathlib import Path
import shutil
base=Path(__file__).resolve().parent
original=base.parents[1]/'ui'
shutil.copytree(original,base/'ui',dirs_exist_ok=True)
p=base/'ui/app.js';s=p.read_text()
needle="  tko.disabled = !status.ctl_connected || (!status.armed && !status.arm_ready);"
assert needle in s
s=s.replace(needle,"""  const nativeNoseLift = status.mode === 'sitl' && +((params.NLF_ENABLE || {}).value ?? (airframe.px4_overrides || {}).NLF_ENABLE) === 1;
  tko.title = nativeNoseLift ? 'PX4 arms, raises the nose to 25 degrees, then climbs 1 metre and holds heading' : 'Automatic takeoff';
  tko.textContent = nativeNoseLift ? 'Nose lift + Takeoff (PX4)' : 'Takeoff';
  tko.disabled = !status.ctl_connected || (nativeNoseLift && status.armed) || (!status.armed && !status.arm_ready);
  $$('#nl-card input, #nl-card button').forEach(el => { el.disabled = !!nativeNoseLift; });
  const nlHint = $('#nl-card .hint');
  if (nlHint && nativeNoseLift) nlHint.textContent = 'Native PX4 module ready. Use Nose lift + Takeoff (PX4) below. It arms, lifts the nose to 25°, then takes off and holds heading. These simulator-only controls are inactive.';
  $$('#mode-pills .pill').forEach(el => { if (el.textContent.toLowerCase() === 'takeoff') { el.disabled = !!nativeNoseLift; el.title = nativeNoseLift ? 'Use Nose lift + Takeoff (PX4) below' : ''; } });
""")
needle="$('#btn-takeoff').addEventListener('click', async () => {\n  try {"
assert needle in s
s=s.replace(needle,needle+"""
    if (status.mode === 'sitl' && +((params.NLF_ENABLE || {}).value ?? (airframe.px4_overrides || {}).NLF_ENABLE) === 1) {
      if (status.armed) return;
      logLine('[ui] native PX4 nose lift: normal arming, front-fan lift, then heading-held hover');
      await api('/api/sim/nose_lift', { stop: true });
      await api('/api/px4/shell', { command: 'atlas_nose_lift start', timeout: 0.5 });
      const result = await api('/api/px4/shell', { command: 'atlas_nose_lift takeoff', timeout: 0.5 });
      if (result.output) logLine(result.output);
      return;
    }
""")
needle = "$$('#tab-sim button[data-cmd]').forEach(b => b.addEventListener('click', () =>\n  api('/api/px4/command', { command: b.dataset.cmd, mode: b.dataset.mode, force: b.dataset.cmd === 'kill' }).catch(e => logLine('[ui] ' + e.message))));"
assert needle in s
s=s.replace(needle,"""$$('#tab-sim button[data-cmd]').forEach(b => b.addEventListener('click', async () => {
  try {
    if (b.dataset.mode === 'land' && status.mode === 'sitl' && +((params.NLF_ENABLE || {}).value ?? (airframe.px4_overrides || {}).NLF_ENABLE) === 1) {
      logLine('[ui] native landing: rear legs first, slow nose lowering, then motors off');
      const result = await api('/api/px4/shell', { command: 'atlas_nose_lift land', timeout: 0.5 });
      if (result.output) logLine(result.output);
      return;
    }
    await api('/api/px4/command', { command: b.dataset.cmd, mode: b.dataset.mode, force: b.dataset.cmd === 'kill' });
  } catch(e) { logLine('[ui] ' + e.message); }
}));""")
s=s.replace("  const nlHint = $('#nl-card .hint');", """  const landButton = $('#mode-pills button[data-mode="land"]');
  if (landButton) {
    landButton.textContent = nativeNoseLift ? 'Land + lower nose' : 'Land';
    landButton.disabled = nativeNoseLift && !status.armed;
    landButton.title = nativeNoseLift ? 'From native hover: descend onto rear legs, lower the nose at 4 degrees/s, then disarm' : 'Land';
  }
  const nlHint = $('#nl-card .hint');""")
s=s.replace('These simulator-only controls are inactive.', 'Then use Land + lower nose above. These simulator-only controls are inactive.')
# The native target writes a real PX4 parameter; legacy settings retain their own handler.
s=s.replace("  const nlHint = $('#nl-card .hint');", """  const targetInput = $('#af-takeoff');
  targetInput.disabled = !nativeNoseLift || !!status.armed || !status.ctl_connected;
  $('#nl-target').closest('label').hidden = !!nativeNoseLift;
  if (nativeNoseLift) {
    targetInput.disabled = !!status.armed || !status.ctl_connected;
    targetInput.step = 'any';
    targetInput.title = 'PX4 NLF_TARGET: ground nose lift angle. Hover pitch is configured separately.';
    if (document.activeElement !== targetInput) targetInput.value = (params.NLF_TARGET || {}).value ?? (airframe.px4_overrides || {}).NLF_TARGET ?? 25;
  }
  const nlHint = $('#nl-card .hint');""")
s=s.replace("'PX4 arms, raises the nose to 25 degrees, then climbs 1 metre and holds heading'", "'PX4 arms, raises the nose to the configured target, then climbs and holds heading'")
s=s.replace("It arms, lifts the nose to 25°, then takes off and holds heading.", "Set Takeoff pitch° in the Geometry tab before takeoff. Hover pitch and Landed pitch are configured in Geometry.")
s=s.replace("These simulator-only controls are inactive.", "Other controls in this card are simulator-only and inactive.")
old = "$$('#nl-card input').forEach(inp => inp.addEventListener('change', () => { airframe.design.nose_lift = noseLiftCfg(); pushAirframe(true); }));"
new = """$$('#nl-card input').forEach(inp => { inp.onchange = () => {
    if (status.mode === 'sitl' && +((params.NLF_ENABLE || {}).value ?? (airframe.px4_overrides || {}).NLF_ENABLE) === 1) return;
    airframe.design.nose_lift = noseLiftCfg(); pushAirframe(true);
  }; });"""
assert old in s
s=s.replace(old,new)
s += """
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
"""
# Recover a missed startup event without weakening arming gates.
s = "let lastPrearmReportRequest = 0;\nlet prearmReportRequestPending = false;\n" + s
needle = "  const nativeNoseLift = status.mode === 'sitl'"
assert needle in s
s = s.replace(needle, """  if (status.mode === 'sitl' && status.ctl_connected && !status.armed &&
      !status.arm_ready && !status.resetting &&
      !prearmReportRequestPending && Date.now() - lastPrearmReportRequest > 10000) {
    lastPrearmReportRequest = Date.now();
    prearmReportRequestPending = true;
    api('/api/px4/shell', { command: 'commander check', timeout: 0.5 })
      .catch(e => logLine('[ui] Could not refresh arming checks: ' + e.message))
      .finally(() => { prearmReportRequestPending = false; });
  }
""" + needle)
s=s.replace("nativeNoseLift ? 'Nose lift + Takeoff (PX4)' : 'Takeoff'", "'Takeoff'")
s=s.replace("nativeNoseLift ? 'Land + lower nose' : 'Land'", "'Land'")
s=s.replace("$('#mode-pills button[data-mode=\"land\"]')", "$('#btn-land')")
s=s.replace("$$('#tab-sim button[data-cmd]')", "$$('#tab-sim button[data-cmd], #btn-land')")
s=s.replace("landButton.disabled = nativeNoseLift && !status.armed;", "landButton.disabled = !status.ctl_connected || !status.armed;")
s=s.replace('Use Nose lift + Takeoff (PX4) below', 'Use Takeoff below').replace('Then use Land + lower nose above.', 'Then use Land below.')
s=s.replace("bindNumber('af-landed', v => airframe.landed_pitch_deg = v);", """$('#af-landed').addEventListener('change', async e => {
  const value = Number(e.target.value);
  if (status.armed || !Number.isFinite(value)) { e.target.value = airframe.landed_pitch_deg; return; }
  try {
    const result = await api(status.ctl_connected ? '/api/params/set' : '/api/airframe/override', { name: 'NLF_LAND_ANG', value });
    if (!result.ok) throw new Error(result.error || 'Landed pitch was not saved');
    airframe.landed_pitch_deg = value;
    airframe.px4_overrides = { ...(airframe.px4_overrides || {}), NLF_LAND_ANG: value };
    params.NLF_LAND_ANG = { ...(params.NLF_LAND_ANG || {}), value };
    scene.setAirframe(airframe); pushAirframe(true);
    logLine('[ui] Landed pitch saved: ' + value + '°');
  } catch(e) { logLine('[ui] ' + e.message); $('#af-landed').value = airframe.landed_pitch_deg; }
});""")
s=s.replace("  const targetInput = $('#af-takeoff');", """  $('#af-hover').disabled = !!status.armed;
  $('#af-landed').disabled = !!status.armed;
  const targetInput = $('#af-takeoff');""")
# Synchronize the explicit landed angle before starting the native sequence.
s=s.replace("      if (status.armed) return;", """      if (status.armed) return;
      const landing = await api('/api/params/set', { name: 'NLF_LAND_ANG', value: Number(airframe.landed_pitch_deg) });
      if (!landing.ok) throw new Error(landing.error || 'Could not apply landed pitch');""", 1)
# Landed pitch is derived by the server whenever the leg geometry changes.
start = s.index("$('#af-landed').addEventListener('change', async e => {")
end = s.index("\n});", start) + len("\n});")
s = s[:start] + s[end:]
s=s.replace("$('#af-landed').disabled = !!status.armed;", "$('#af-landed').readOnly = true;")
s=s.replace("      // the server resolves mass.from_items", """      if (res.airframe) {
        airframe.landed_pitch_deg = res.airframe.landed_pitch_deg;
        airframe.px4_overrides = { ...(airframe.px4_overrides || {}), NLF_LAND_ANG: res.airframe.landed_pitch_deg };
        $('#af-landed').value = res.airframe.landed_pitch_deg;
      }
      // the server resolves mass.from_items""")
p.write_text(s)
h = base/'ui/index.html'
html = h.read_text()
line = next(line for line in html.splitlines() if 'id="af-landed"' in line)
html = html.replace(line, line + '\n        <label title="Nose-up angle reached on the ground before takeoff. Editable while connected and disarmed; does not change hover pitch.">Takeoff pitch° <input id="af-takeoff" type="number" step="any" value="25" disabled></label>')
# Keep the three pitch fields together, separate from the inertia controls.
line = next(line for line in html.splitlines() if 'id="af-hover"' in line)
html = html.replace(line, '      </div>\n      <div class="row">\n' + line)
pitch_lines = [next(line for line in html.splitlines() if f'id="{field}"' in line) for field in ('af-hover', 'af-landed', 'af-takeoff')]
html = html.replace('\n'.join(pitch_lines), '\n'.join([pitch_lines[2], pitch_lines[0], pitch_lines[1]]))
land_line = next(line for line in html.splitlines() if 'data-mode="land"' in line)
html = html.replace(land_line + '\n', '')
# The bottom Takeoff control replaces the duplicate mode shortcut.
takeoff_mode_line = next(line for line in html.splitlines() if 'data-mode="takeoff"' in line)
html = html.replace(takeoff_mode_line + '\n', '')
takeoff_line = next(line for line in html.splitlines() if 'id="btn-takeoff"' in line)
html = html.replace(takeoff_line, takeoff_line + '\n      <button id="btn-land" class="pill" data-cmd="mode" data-mode="land" disabled>Land</button>')
html = html.replace('Nose-up pitch of the airframe when it stands on its legs; the simulator rests the vehicle at this attitude.', 'Calculated automatically from the enabled landing feet. Actual resting pitch can differ slightly as the legs compress.')
line = next(line for line in html.splitlines() if 'id="af-landed"' in line)
html = html.replace(line, line + '\n        <span class="hint">After changing hover pitch, use Update PX4, then Reset. Landed pitch is calculated from the landing legs.</span>')
html=html.replace('Landed pitch° <input', 'Landed pitch° (auto) <input')
h.write_text(html)
