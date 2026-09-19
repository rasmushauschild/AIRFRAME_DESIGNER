"""Physical coefficients for the ten-motor, two-front-fan ground sequence."""
import numpy as np

def ground_sequence_params(af):
    out = {"NLF_CFG_OK": 0}
    rotors = af.active_rotors()
    legs = af.active_legs()
    if len(rotors) != 10 or len(legs) != 3:
        return out
    feet = sorted([np.asarray(l.foot(), float) for l in legs], key=lambda p: p[0])
    # Two rear feet must form a transverse hinge. General landing gear needs a different controller.
    if abs(feet[0][0]-feet[1][0]) > .01 or abs(feet[0][2]-feet[1][2]) > .01:
        return out
    pivot = (feet[0]+feet[1])/2
    cg = np.asarray(af.cg)
    if not (feet[0][1] < cg[1] < feet[1][1] or feet[1][1] < cg[1] < feet[0][1]):
        return out
    if cg[0] <= pivot[0] or feet[2][0] <= cg[0]:
        return out
    front = rotors[8:10]
    if any(abs(r.thrust_exponent-2) > 1e-6 for r in front):
        return out
    thrust = [np.asarray(r.axis, float)/np.linalg.norm(r.axis)*r.effective_max_thrust() for r in front]
    moments = [np.cross(np.asarray(r.pos)-pivot, force) - r.km*force for r, force in zip(front, thrust)]
    yaw = [np.cross(np.asarray(r.pos)-cg, force)[2] - r.km*force[2] for r, force in zip(front, thrust)]
    if yaw[0]*yaw[1] >= 0 or min(m[1] for m in moments) <= 0:
        return out
    w9 = -2*yaw[1]/(yaw[0]-yaw[1])
    authority = w9*moments[0][1] + (2-w9)*moments[1][1]
    values = [af.mass.mass, *(cg-pivot)[[0,2]], authority, w9]
    if not np.isfinite(values).all() or af.mass.mass <= 0 or authority <= 0:
        return out
    out.update(dict(zip(["NLF_MASS", "NLF_GX", "NLF_GZ", "NLF_MOM", "NLF_W9"], map(float,values))))
    out["NLF_CFG_OK"] = 1
    return out
