// Editor indices remain stable across enable/disable; deletions explicitly remap them.
export function noseSelection(rotors, saved) {
  return saved ?? rotors.flatMap((r, i) => r.pos[0] > 0.2 && r.enabled !== false ? [i] : []);
}
export function removeNoseMotor(selection, deleted) {
  return selection.filter(i => i !== deleted).map(i => i > deleted ? i - 1 : i);
}
export function simulatorNoseSelected(mode, config) {
  return mode === 'sitl' && config?.enabled === true;
}
