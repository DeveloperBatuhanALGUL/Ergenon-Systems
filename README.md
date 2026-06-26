# Ergenon Systems

Open-source flight control and mission computer software for a conceptual
6th-generation aircraft platform, running on **TurkOS-Ergenon**, a
real-time operating system distribution built on the TurkOS kernel.

## Disclaimer

This is a research and educational software architecture project. It does
not contain, and will never contain, weapon targeting, guidance, fire
control logic, electronic warfare signal processing, or radar cross
section design data. Munitions-related modules are limited to passive
inventory and status telemetry only.

## License

This project is licensed under the Ergenon Systems Source-Visible
License. Commercial use is strictly and permanently prohibited under any
circumstances. See [LICENSE](./LICENSE).

## Architecture Overview

The system is organized into independently verifiable layers:

- **Flight Control Core** — deterministic flight control laws, sensor
  fusion, and redundancy management, written in Ada/SPARK for high
  assurance.
- **Sentinel Network** — a distributed mesh of lightweight watchdog
  agents, each bound to a single subsystem, continuously verifying its
  output against safe operating bounds and escalating anomalies upward.
- **Intervention Layer** — fail-safe actuation that isolates or
  reconfigures a subsystem when the Sentinel Network escalates a fault.
- **Command Link** — encrypted telemetry uplink to ground command,
  with region-scoped key authorities ensuring position data is only
  decipherable by the relevant responsible station.
- **Mission Computer** — navigation, fuel and power management, weather
  integration, and passive equipment status reporting for the pilot.
- **Anatolia Ergenon** — an advisory AI layer that summarizes system
  state and anomalies for the pilot. It does not make or execute weapon
  employment decisions.
- **Hardware Abstraction Layer** — direct hardware interfaces (UART,
  SPI, I2C, CAN, GPIO, ADC) for sensors and actuators.

## Languages

- **Ada/SPARK** — flight-critical control loops and redundancy logic
- **C** — RTOS kernel, HAL, drivers
- **Rust** — memory-safe non-real-time services
- **Ergenon Rule DSL** — declarative configuration language for Sentinel
  Network thresholds and rules

## Naming Convention

Module codenames throughout this codebase draw, non-uniformly, from
across Turkic history and culture — Gokturk, Seljuk, Ottoman, Crimean
Tatar, Karakhanid, Avar, Khazar, Kipchak, and others. All code, comments,
and documentation are written in English; only proper module names carry
this cultural reference.

## Status

Architecture in active design. No flight-critical module is complete or
verified. Do not use this software in any real aircraft or hardware.

## Maintainer

Batuss — https://batuss.com
