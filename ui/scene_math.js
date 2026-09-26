// Pure coordinate helpers shared by the browser scene and its lightweight tests.
// The simulator pose is the CG in NED, while model geometry is stored relative
// to the structural origin in FRD. The structural origin must therefore be
// drawn at -CG inside the vehicle group.
export function frameOffsetForCg(cg = [0, 0, 0]) {
  const x = Number(cg[0] || 0), y = Number(cg[1] || 0), z = Number(cg[2] || 0);
  return [x ? -x : 0, z, y ? -y : 0];
}
