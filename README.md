<div align="center">

<img src="./docs/assets/batuss-seal.svg" width="140" alt="Batuss Seal" />

# ERGENON SYSTEMS

[![Typing SVG](https://readme-typing-svg.demolab.com?font=Fira+Code&size=20&pause=1000&color=6B2E2C&center=true&vCenter=true&width=600&lines=Flight+Control+%26+Mission+Computer;Running+on+TurkOS-Ergenon;Built+by+Batuss)](https://git.io/typing-svg)

[![License](https://img.shields.io/badge/license-Source--Visible--Non--Commercial-6B2E2C)](./LICENSE)
[![Status](https://img.shields.io/badge/status-architecture--design-2F6F52)](#status)
[![Platform](https://img.shields.io/badge/platform-TurkOS--Ergenon-E3D5C3)](#)
[![Made by](https://img.shields.io/badge/made%20by-Batuss-6B2E2C)](https://batuss.com)
[![Languages](https://img.shields.io/badge/languages-Ada%20%7C%20C%20%7C%20Rust-2F6F52)](#languages)
[![Commercial Use](https://img.shields.io/badge/commercial%20use-PROHIBITED-red)](./LICENSE)

</div>

---

## Disclaimer

This is a research and educational software architecture project. It does
not contain, and will never contain, weapon targeting, guidance, fire
control logic, electronic warfare signal processing, or radar cross
section design data. Munitions-related modules are limited to passive
inventory and status telemetry only.

## License

This project is licensed under the **Ergenon Systems Source-Visible
License**. Commercial use is strictly and permanently prohibited under
any circumstances, with no exceptions. See [LICENSE](./LICENSE).

## Architecture Overview

```mermaid
flowchart TB
    subgraph GROUND["Sivas Command Link"]
        HQ["Ground Command Station"]
    end

    subgraph AIRCRAFT["TurkOS-Ergenon Runtime"]
        FCC["Flight Control Core<br/>(Ada/SPARK)"]
        SENT["Sentinel Network<br/>(Watchdog Mesh)"]
        INT["Intervention Layer<br/>(Fail-Safe Actuation)"]
        MC["Mission Computer<br/>(Nav, Fuel, Power, Weather)"]
        AI["Anatolia Ergenon<br/>(Advisory AI Layer)"]
        HAL["Hardware Abstraction Layer"]
    end

    SENT -->|monitors| FCC
    SENT -->|monitors| MC
    SENT -->|escalates anomaly| INT
    INT -->|reconfigures| FCC
    INT -->|reconfigures| MC
    FCC <--> HAL
    MC <--> HAL
    AI -->|summarizes state| MC
    MC -->|encrypted telemetry| HQ
    HQ -.->|region-scoped key| MC
```

## Module Table

| Layer | Function | Language |
|---|---|---|
| Flight Control Core | Deterministic flight control laws, redundancy management | Ada/SPARK |
| Sentinel Network | Distributed watchdog mesh, per-subsystem bound checking | C |
| Intervention Layer | Fail-safe isolation and reconfiguration on fault escalation | C |
| Command Link | Encrypted uplink, region-scoped key authority | C / Rust |
| Mission Computer | Navigation, fuel, power, weather, equipment status | C |
| Anatolia Ergenon | Advisory AI summarization layer for pilot workload reduction | Rust |
| Hardware Abstraction Layer | UART, SPI, I2C, CAN, GPIO, ADC interfaces | C |
| Ergenon Rule DSL | Declarative Sentinel Network threshold configuration | Custom DSL |

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

<div align="center">

**Batuss** — [batuss.com](https://batuss.com)

</div>
