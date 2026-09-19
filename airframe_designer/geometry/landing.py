"""Estimate the unloaded ground posture from enabled landing-foot geometry."""
import math
import numpy as np


def apply_landed_pitch(airframe):
    legs = airframe.active_legs()
    if len(legs) < 3:
        raise ValueError('Automatic landed pitch requires at least three enabled landing legs')
    feet = np.asarray([leg.foot() for leg in legs], dtype=float)
    radii = np.asarray([leg.foot_radius for leg in legs], dtype=float)
    if not np.isfinite(feet).all() or not np.isfinite(radii).all():
        raise ValueError('Landing feet must have finite coordinates and radii')
    design = np.column_stack((feet[:, 0], feet[:, 1], np.ones(len(feet))))
    if np.linalg.matrix_rank(design) < 3:
        raise ValueError('Landing feet must span a support plane')
    nz = 1.0
    for _ in range(20):
        a, b, c = np.linalg.lstsq(design, feet[:, 2] + radii / nz, rcond=None)[0]
        nz = 1.0 / math.sqrt(1 + a*a + b*b)
    normal = np.array([-a, -b, 1.0]) * nz
    heights = feet @ normal + radii
    if np.ptp(heights) > .01:
        raise ValueError('Landing feet do not share a ground plane within 1 cm; check leg geometry')
    pitch = round(math.degrees(math.atan2(a, math.sqrt(1 + b*b))), 2)
    airframe.landed_pitch_deg = pitch
    airframe.px4_overrides['NLF_LAND_ANG'] = pitch
    return pitch
