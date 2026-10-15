"""Flight dynamics: the attitude quaternion helpers, the landing-gear geometry, and JSBSim.

There is one physics engine, `jsbsim_backend.JSBSimBody`, and it is not selectable. It is imported where it
is used rather than here, so nothing pays for loading JSBSim unless it is actually going to fly something.
"""
from ..constants import G
from .contact import LegContacts
from .quaternion import q_deriv, q_from_euler, q_normalize, q_to_euler, q_to_rotmat

__all__ = ["G", "LegContacts", "q_normalize", "q_to_rotmat", "q_from_euler", "q_to_euler", "q_deriv"]
