# Threat Model

This document states what RTVLAS-Slim is designed to detect, what it has been measured against, and what is outside the current architecture.

## In Scope

RTVLAS-Slim is in scope for navigation-solution-level spoofing where the GPS position, velocity, or clock-bias behavior becomes inconsistent with an IMU-driven prediction.

The monitored inputs are processed telemetry, primarily:

- `HIGHRES_IMU`
- `GPS_RAW_INT`
- `GLOBAL_POSITION_INT`

The measured attack paths include:

| Path | Attack model | Evidence |
| --- | --- | --- |
| TEXBAT `ds2` processed replay | abrupt carry-off | `0.978 / 0.034` anomaly TPR/FPR |
| TEXBAT `ds3` processed replay | gradual low-magnitude drift | `0.953 / 0.032` anomaly TPR/FPR |
| TEXBAT `ds7` processed replay | subtle time-push / phase-aligned behavior | `0.999 / 0.000` anomaly TPR/FPR |
| PX4 SIH live proxy abrupt offset | software MAVLink man-in-the-middle on `GLOBAL_POSITION_INT` | `13 / 2 / 15` trusted/flagged/rejected |
| PX4 SIH gradual carry-off | `30 m` north ramp over `2.5 s` | `25 / 6 / 14` trusted/flagged/rejected |

## Out Of Scope

RTVLAS-Slim is not an RF-layer spoofing detector.

A receiver-level RF attack that remains internally consistent through the GPS receiver tracking loops can produce clean-looking processed navigation solutions. RTVLAS-Slim sees the processed output after the receiver, so this is an architectural boundary, not a parameter-tuning problem.

Other out-of-scope attacks:

- Attacks that compromise the IMU stream itself.
- Attacks that compromise the flight controller before RTVLAS-Slim receives telemetry.
- Attacks that compromise the host running the detector and the signing key.
- Jamming-only scenarios where no usable navigation solution is emitted.
- Multipath and weak-signal field behavior not represented in the simulator or processed TEXBAT paths.

## Platform Coverage

The code is written around common MAVLink telemetry messages used by PX4 and ArduPilot-style systems. It is receiver-agnostic in the sense that it does not require a specific GPS receiver vendor or raw receiver access.

The measured live and multi-mission results in this repository are PX4 SIH only. ArduPilot, hardware PX4, fixed-wing, VTOL, outdoor flight, and high-dynamics vehicle profiles are not measured in this repository.

## Reviewer Conclusion

A technical reviewer should conclude:

- RTVLAS-Slim is a plausible software-layer detector for processed telemetry spoofing where GPS motion diverges from inertial prediction.
- The current processed TEXBAT results show that persistence logic matters over naive one-shot thresholds.
- The current PX4 SIH results show clean nominal behavior across four simulator regimes and measurable detection of software-injected spoof profiles.
- RTVLAS-Slim is not a substitute for RF-layer monitoring against sophisticated electronic warfare systems.
- Hardware receiver tests and outdoor flight validation are required before making field-performance claims.
