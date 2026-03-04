"""Atmospheric environment model (Python version).

This mirrors the ISA-like model implemented in src/core/Environment.hpp
so that analysis scripts (e.g. compare_models.py) can query density
and related quantities directly from Python.
"""

import math

# ============ PHYSICAL CONSTANTS ============
R = 287.05287       # Specific gas constant for air [J/(kg·K)]
GAMMA = 1.4         # Specific heat ratio
G0 = 9.80665        # Sea-level gravity [m/s²]
R_EARTH = 6_371_000.0  # Earth radius [m]

# Sea-level conditions
T0_SL = 288.15      # Temperature [K] (15°C)
P0_SL = 101_325.0   # Pressure [Pa]
RHO0_SL = 1.225     # Density [kg/m³]

# Atmospheric layers
TROPOPAUSE = 11_000.0   # [m]
STRATOPAUSE = 20_000.0  # [m]
LAPSE_RATE = 0.0065     # [K/m]


# ============ GRAVITY ============

def getGravity(altitude: float) -> float:
    """Gravity as a function of altitude [m]."""
    factor = R_EARTH / (R_EARTH + altitude)
    return G0 * factor * factor


# ============ ATMOSPHERE ============

def getTemperature(altitude: float) -> float:
    """Temperature [K] as a function of altitude [m] (layered ISA)."""
    if altitude <= TROPOPAUSE:
        # Troposphere: T = T0 - L * h
        return T0_SL - LAPSE_RATE * altitude
    elif altitude <= STRATOPAUSE:
        # Tropopause (isothermal)
        return T0_SL - LAPSE_RATE * TROPOPAUSE  # ~216.65 K
    else:
        # Upper stratosphere (kept constant for this model)
        return 216.65


def getPressure(altitude: float) -> float:
    """Pressure [Pa] as a function of altitude [m]."""
    if altitude <= TROPOPAUSE:
        # Troposphere: P = P0 * (T/T0)^(g/(L*R))
        T = getTemperature(altitude)
        exponent = G0 / (LAPSE_RATE * R)
        return P0_SL * (T / T0_SL) ** exponent
    else:
        # Tropopause / Stratosphere: exponential model
        T_tropo = getTemperature(TROPOPAUSE)
        P_tropo = getPressure(TROPOPAUSE)
        scale_height = R * T_tropo / G0
        return P_tropo * math.exp(-(altitude - TROPOPAUSE) / scale_height)


def getDensity(altitude: float) -> float:
    """Density [kg/m³] as a function of altitude [m] via ideal gas law."""
    P = getPressure(altitude)
    T = getTemperature(altitude)
    return P / (R * T)


def getDensitySimple(altitude: float) -> float:
    """Simple exponential density model: rho = rho0 * exp(-h/H)."""
    H = 8400.0  # Scale height [m]
    return RHO0_SL * math.exp(-altitude / H)


def getSpeedOfSound(altitude: float) -> float:
    """Speed of sound [m/s] as a function of altitude [m]."""
    T = getTemperature(altitude)
    return math.sqrt(GAMMA * R * T)
