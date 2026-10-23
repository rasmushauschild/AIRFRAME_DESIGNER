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


def set_park_pitch(airframe, pitch_deg, min_length=0.02):
    """Stand the airframe at ``pitch_deg`` nose-up by changing only the legs ahead of the CG.

    The feet behind the CG are what the aircraft pivots on when it rotates off the stand, so they stay where the
    design put them; every leg ahead of the CG keeps its attachment point and direction and only changes length,
    until its foot touches the ground plane through the rear feet at that pitch. A pitch the legs cannot reach - a
    front leg shorter than ``min_length``, a leg running along the ground, the CG outside the feet - is refused
    with the reason and nothing is changed. The angle is then read back from the feet as usual."""
    legs = airframe.active_legs()
    if len(legs) < 3:
        raise ValueError('Setting the park pitch requires at least three enabled landing legs')
    cg = np.asarray(airframe.mass.cg, dtype=float)
    feet = np.asarray([leg.foot() for leg in legs], dtype=float)
    front = feet[:, 0] > cg[0]
    if front.all() or not front.any():
        raise ValueError('Setting the park pitch needs feet both ahead of and behind the CG')
    phi = math.radians(float(pitch_deg))
    down = np.array([-math.sin(phi), 0.0, math.cos(phi)])          # world down seen in the structural frame
    ahead = np.array([math.cos(phi), 0.0, math.sin(phi)])          # world forward along the ground
    radii = np.asarray([leg.foot_radius for leg in legs], dtype=float)
    rear_heights = feet[~front] @ down + radii[~front]
    if np.ptp(rear_heights) > .01:
        raise ValueError(f'The rear feet are not level at {pitch_deg:g} deg; check leg geometry')
    ground = float(np.mean(rear_heights))
    lengths = {}
    for leg, is_front, r in zip(legs, front, radii):
        if not is_front:
            continue
        along = float(leg.direction() @ down)
        if along < 0.2:
            raise ValueError(f'Leg {leg.name} runs nearly along the ground at {pitch_deg:g} deg')
        length = (ground - r - float(np.asarray(leg.attach, dtype=float) @ down)) / along
        if length < min_length:
            raise ValueError(f'At {pitch_deg:g} deg leg {leg.name} would be {length:.3f} m long; '
                             f'the shortest allowed is {min_length:g} m')
        lengths[id(leg)] = length
    new_feet = np.asarray([np.asarray(leg.attach, dtype=float) + lengths[id(leg)] * leg.direction()
                           if id(leg) in lengths else f for leg, f in zip(legs, feet)])
    arm = (new_feet - cg) @ ahead
    if arm[front].min() <= 0 or arm[~front].max() >= 0:
        raise ValueError(f'At {pitch_deg:g} deg the CG would not sit between the front and rear feet; it would tip')
    for leg in legs:
        if id(leg) in lengths:
            leg.length = round(lengths[id(leg)], 4)
    return apply_landed_pitch(airframe)
