"""Physical constants, defined once.

Gravity used to be written out in five different files. They all agreed, but five copies of a number is five
chances for them to stop agreeing, and a simulator whose answer depends on which module you asked is not one
you can trust. There is one definition here and everything imports it.
"""
from __future__ import annotations

__all__ = ["G"]

#: Standard gravity, m/s^2 (CODATA / ISO 80000-3). The same value JSBSim and PX4 use.
G = 9.80665
