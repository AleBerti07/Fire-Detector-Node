# Fire Detector Architecture — TTN IoT Node
### Three-Stage Adaptive Sensing System

---

## Overview

This system implements a **cost-effective, power-aware fire detection node** on The Things Network (LoRaWAN EU868). It operates across three states of increasing alertness, balancing battery life and radio duty cycle in normal conditions while enabling rapid, high-resolution sensing when anomalies are detected.

The core principle: **sense cheaply until something looks wrong, then escalate aggressively.**

---

## State Machine

```
┌─────────────────┐         threshold crossed        ┌──────────────────────┐
│                 │  ─────────────────────────────►  │                      │
│   STAGE 1       │                                   │   STAGE 2            │
│   Power-Saving  │  ◄─────────────────────────────  │   Abnormal           │
│                 │         readings normalize        │                      │
└─────────────────┘                                   └──────────────────────┘
                                                               │
                                                               │  alert triggered + TTN uplink
                                                               ▼
                                                      ┌──────────────────────┐
                                                      │   STAGE 3            │
                                                      │   Alarmed            │
                                                      │   (manual reset only)│
                                                      └──────────────────────┘
```

> **Note:** Escalation requires N consecutive high readings (hysteresis). De-escalation from Stage 2 → Stage 1 requires M consecutive normal readings. Stage 3 → Stage 2 requires **manual reset** to prevent false all-clear during an active fire.

---

## Stage 1 — Power-Saving Mode

**Purpose:** Baseline monitoring. Conserve battery and stay within LoRaWAN duty cycle limits.

| Sensor         | Interval     | Rationale                                              |
|----------------|--------------|--------------------------------------------------------|
| CO2 / Particles | Every 30 min | Slow ambient drift; early chemical signature of fire   |
| Temperature    | Every 30 min | Detect gross ambient changes (optional, can be paired) |
| Humidity       | —            | Not sampled; stable in normal conditions               |

**TTN Uplink:** **Silent by default.** Two exceptions only:
1. **24-hour heartbeat** — one scheduled uplink per day containing rolling averages and a watchdog cycle count (see below)
2. **Threshold crossing** — an immediate unscheduled uplink if a transition to Stage 2 is triggered

**24-Hour Heartbeat Payload:**
- Average temperature, CO2, humidity over the last 24h
- Min/max observed values per sensor
- Battery voltage
- `cycleCount` — number of 30-min sensing cycles completed since last heartbeat (expected: 48). A low count signals a device fault or power issue.
- Current state enum


**Transition Trigger (→ Stage 2):**
- CO2 exceeds baseline threshold (e.g. > 600 ppm), **or**
- Temperature exceeds ambient threshold (e.g. > 30°C), **or**
- Composite score ≥ threshold for N consecutive readings (hysteresis)
- On trigger: immediately uplink with current readings **plus last 3 rolling averages** to give the backend pre-anomaly context

**LED Indicator:** Green LED steady — system nominal.

---

## Stage 2 — Abnormal Mode

**Purpose:** Elevated vigilance. Frequent sensing to track the developing situation and compute rate-of-change. Sends a TTN alert uplink on entry.

| Sensor         | Interval       | Rationale                                                     |
|----------------|----------------|---------------------------------------------------------------|
| Temperature    | Every 1–2 min  | High frequency enables derivative computation (dT/dt)         |
| CO2 / Particles | Every 5–10 min | Detect rising combustion byproducts                           |
| Humidity       | Every 5–10 min | Dropping humidity is an early fire precursor                  |

**TTN Uplink:** On state entry (alert message) + every sensing cycle.

**Derivative Monitoring:**  
Temperature samples are stored in a rolling buffer. The rate of change (°C/min) is computed across the sampling window:

```
dT/dt = (T_now - T_prev) / Δt
```

If `dT/dt > 2°C/min` → escalate to Stage 3 immediately, bypassing hysteresis.

**Transition Trigger (→ Stage 3):**  
- Temperature derivative exceeds fast-rise threshold, **or**  
- Composite score ≥ alarm threshold for N consecutive readings, **or**  
- Manual button press

**De-escalation (→ Stage 1):**  
- M consecutive readings with all values below Stage 1 thresholds  
- Hysteresis counter prevents oscillation

**LED Indicator:** Green + Red LED — elevated alert.

---

## Stage 3 — Alarmed Mode

**Purpose:** Active fire detection. Maximum sensing resolution. Detect both slow smoldering and fast-growing fires via absolute thresholds and rate-of-rise.

| Sensor         | Interval  | Rationale                                                      |
|----------------|-----------|----------------------------------------------------------------|
| Temperature    | Every 1 min | Absolute threshold + rate-of-rise detection                  |
| CO2 / Particles | Every 1 min | Confirm combustion byproducts                                |
| Humidity       | Every 1 min | Confirm low-humidity fire environment                        |

**TTN Uplink:** Every 1 minute minimum, or **immediately on fire detection**.

**Fire Detection Criteria (any one triggers FIRE alert):**

| Metric                    | Threshold                               | Detection Type      |
|---------------------------|-----------------------------------------|---------------------|
| Temperature absolute       | > 60°C                                  | Smoldering / radiant heat |
| Temperature rate-of-rise   | ≥ 0.2°C/sec (warn) / ≥ 0.6°C/sec (fire) | Fast-growing fire |
| CO2 rate-of-rise           | ≥ 5 ppm/sec                             | Rapid combustion    |
| Composite score            | ≥ 70                                    | Multi-sensor fusion |


**TTN Payload includes:**
- All sensor readings
- Current composite score
- Rate-of-rise values (dT/dt, dCO2/dt)
- Alarm flag
- Current state enum

**De-escalation:** **Manual reset only** (black button). Stage 3 does not auto-clear to prevent false all-clear during an active fire.

**LED Indicator:** Red LED blinking.

---

## Composite Scoring Model

A weighted score (0–100) fuses all sensor inputs. This score drives state transitions via hysteresis:

```
score = (tScore × 0.4) + (hScore × 0.3) + (cScore × 0.3)
```

| Component | Formula | Weight |
|-----------|---------|--------|
| Temperature score | `constrain((T - 30) / 70, 0, 1) × 100` | 40% |
| Humidity score (inverted) | `constrain((50 - H) / 50, 0, 1) × 100` | 30% |
| CO2 score | `constrain((CO2 - 400) / 600, 0, 1) × 100` | 30% |

| Score Range | State        |
|-------------|--------------|
| 0 – 24      | Stage 1 (Power-Saving) |
| 25 – 44     | Stage 2 (Abnormal)     |
| 45 – 69     | Stage 2 (Abnormal, elevated) |
| ≥ 70        | Stage 3 (Alarmed)      |

---

## Sensing Intervals Summary

| Stage         | Temperature | CO2 / Particles | Humidity   | TTN Uplink                          |
|---------------|-------------|-----------------|------------|-------------------------------------|
| Power-Saving  | 30 min      | 30 min          | —          | 24h heartbeat + on threshold breach |
| Abnormal      | 1–2 min     | 5–10 min        | 5–10 min   | On entry + every sensing cycle      |
| Alarmed       | 1 min       | 1 min           | 1 min      | Every 1 min + immediately on fire   |

---

## TTN Duty Cycle Compliance (EU868)

LoRaWAN EU868 enforces a ~1% duty cycle. Approximate airtime per uplink (SF7, 51 bytes payload): **~50ms**.

| Stage         | Uplinks/day | Uplinks/hour | Est. airtime/hour | Duty cycle used |
|---------------|-------------|--------------|-------------------|-----------------|
| Power-Saving  | 1 (heartbeat) + alerts | ~0.04 avg  | ~2ms avg    | < 0.0001%       |
| Abnormal      | —           | ~10–15       | ~600ms            | ~0.017%         |
| Alarmed       | —           | 60           | ~3000ms           | ~0.083%         |

Stage 1 duty cycle usage is now negligible. All stages remain **well within the 1% EU868 limit**.

---

## Hardware Mapping (Current)

| Sensor              | Object                  | Pin               |
|---------------------|-------------------------|-------------------|
| Temperature (sim)   | `iotShieldPotmeter`     | `PIN_POT_RED`     |
| Humidity (sim)      | `iotShieldPotmeter`     | `PIN_POT_WHITE`   |
| CO2                 | `iotShieldCO2Sensor`    | —                 |
| Manual alarm        | `iotShieldButton`       | `PIN_SWITCH_RED`  |
| Manual reset        | `iotShieldButton`       | `PIN_SWITCH_BLACK`|
| Status LEDs         | `iotShieldLED` ×4       | Pins 1–4          |

> Temperature and humidity are currently simulated via potentiometers. 
---

## Recommended Next Steps

1. **Implement per-stage sensing timers** — replace the single `TICK_INTERVAL` with stage-aware intervals for each sensor group.
2. **Add a rolling average buffer** — accumulate 30-min sensor readings into a circular buffer. Used both for the 24h heartbeat payload and for providing pre-anomaly context on Stage 1 → Stage 2 escalation.
3. **Add a `cycleCount` watchdog counter** — increment every 30-min sensing cycle, include in heartbeat, reset after each uplink.
4. **Add battery voltage sensing** — read via ADC and include in heartbeat. Allows remote battery health monitoring without extra uplinks.
5. **Add a rolling temperature buffer** — store the last N temperature readings with timestamps to compute `dT/dt` across the Abnormal and Alarmed stages.
6. **Tune rate-of-rise thresholds** against real sensor data before deployment — start conservative (0.2°C/sec) and tighten from there.
