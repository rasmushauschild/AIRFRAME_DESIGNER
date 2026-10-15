"""Airframe geometry segments. Everything is in the *structural* body frame, FRD (x forward, y right, z down),
metres, with the origin at the airframe's reference point. The centre of gravity is a property of the
``mass`` segment, so moving the CG never means re-entering every position."""
from .frames import unit, rot_x, rot_y, rot_z, rodrigues, tilt_cant_to_axis, axis_to_tilt_cant
from .mass import MassProperties, MassItem
from .propulsion import Rotor
from .wings import Wing, WingAeroCoefficients, wing_panels
from .gear import Leg, generate_legs
from .body import Body
from .airframe import Airframe, SCHEMA_VERSION, migrate
from .paths import get_path, set_path, list_paths, apply_variables

__all__ = ["unit", "rot_x", "rot_y", "rot_z", "rodrigues", "tilt_cant_to_axis", "axis_to_tilt_cant",
           "MassProperties", "MassItem", "Rotor", "Wing", "WingAeroCoefficients", "wing_panels",
           "Leg", "generate_legs", "Body", "Airframe", "SCHEMA_VERSION", "migrate",
           "get_path", "set_path", "list_paths", "apply_variables"]
