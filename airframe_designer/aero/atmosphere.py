"""Standard atmosphere constants (sea level values; the vehicles here fly low)."""
RHO = 1.225      # kg/m^3
from ..constants import G


def isa_temperature_c(alt_m: float) -> float:
    return 15.0 - 0.0065 * alt_m


def isa_pressure_hpa(alt_m: float) -> float:
    return 1013.25 * (1.0 - 2.25577e-5 * alt_m) ** 5.25588
