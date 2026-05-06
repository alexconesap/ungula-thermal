# UngulaThermal

> **High-performance embedded C++ libraries for ESP32, STM32 and other MCUs** — PID thermal control, heater and fan manager, NTC temperature sensor.

PID-based thermal control library for ESP32. Handles heater channels with adaptive duty floors, fan control with tachometry, and NTC thermistor reading. Meant to be reusable across projects.

## Unit Convention

| Context | Unit |
| --- | --- |
| All internal temperatures | **Celsius (°C)** |
| Wire protocol / UI | Fahrenheit (°F) — converted at the application layer |
| Fan speed input | **Percentage (0-100%)** |
| PWM duty output | 0-255 (8-bit) |

> **Rule**: The thermal library never knows about Fahrenheit. Conversion is done at the application layer using `temperature::celsiusToFahrenheit()` from `lib/`.

## Quick Start

A complete heater + fan control loop:

```cpp
#include <ungula/thermal/heater_control.h>
#include <ungula/thermal/fan_control.h>

using namespace ungula::thermal;

HeaterChannel heater(0, 36, 25, -75.0);  // channel 0, ADC pin 36, PWM pin 25, calibration offset
FanController<2> fans;  // 2 fan channels for this project

void setup() {
    heater.setSetpointC(315.0);  // ~600°F
}

void loop() {
    uint32_t now = millis();

    // Read ADC and run the full PID pipeline
    int adcMv = analogReadMilliVolts(36);
    HeaterChannelState state = heater.update(adcMv, now, 0.020);  // 20ms dt

    // Apply heater PWM
    analogWrite(25, state.currentDuty);

    // Update fans at 80% speed
    fans.update(now, 80);
    uint16_t fanDuty = fans.calculateOutputDuty(80);
    analogWrite(fanPin, fanDuty);

    delay(20);  // 50 Hz control loop
}
```

## Examples

### Reading Temperature

The heater channel reads an NTC thermistor, applies a calibration offset, and makes the result available:

```cpp
int adcMv = analogReadMilliVolts(36);
heater.update(adcMv, millis(), 0.020);

double tempC = heater.getTemperatureC();
double tempF = heater.getTemperatureF();
bool connected = heater.isSensorConnected();  // false if NTC is disconnected

Serial.printf("Heater 0: %.1f C (%.1f F), sensor %s\n",
              tempC, tempF, connected ? "OK" : "FAULT");
```

The sensor returns NaN (and `isSensorConnected()` returns false) when something is wrong — voltage too low (disconnected), resistance out of range, or temperature outside [-40°C, 350°C].

### Dual-Mode PID

The PID controller switches between aggressive "approach" gains (when far from setpoint) and conservative "hold" gains (when close). You set both:

```cpp
PidGains approach = {2.0, 0.05, 0.5};   // aggressive — get there fast
PidGains hold     = {0.8, 0.02, 0.3};   // gentle — avoid overshoot

heater.setPidGains(approach, hold);
```

The switch happens automatically at `gainSwitchBandC` from the setpoint.

## How the PID Algorithm Works

This section explains the full PID control pipeline in plain language. You need this to understand what each config parameter does and how to adjust them when the temperature is not behaving as expected.

### The basic idea

A PID controller reads the current temperature, compares it to a target (the setpoint), and calculates how much power to send to the heater. It does this 50 times per second (every 20 ms). The output is a PWM duty value between 0 (heater off) and `outputMax` (heater full power, typically 255).

The "error" is the difference between where you want to be and where you are:

```text
error = setpoint - currentTemperature
```

Positive error means the heater is too cold. Negative means it overshot. The three PID terms react to this error differently:

- **P (Proportional)**: Reacts to the current error. Big error = big output. `pTerm = Kp * error`.
- **I (Integral)**: Reacts to accumulated error over time. If the temperature has been sitting 1 degree below setpoint for a while, the integral grows and pushes the output up. `iTerm = Ki * sum_of_errors_over_time`.
- **D (Derivative)**: Reacts to the speed of temperature change. If the temperature is rising fast toward the setpoint, the derivative term reduces output to prevent overshoot. `dTerm = Kd * (-rate_of_change)`. Note the negative sign: this is "derivative on measurement", meaning the D term opposes fast temperature changes regardless of whether the setpoint moved.

The final output is: `output = P + I + D`, clamped to `[0, outputMax]`.

### Dual-mode gains: approach vs hold

A single set of PID gains cannot do two jobs well. When the heater is cold and needs to reach 300 C, you want aggressive gains to get there fast. When it is sitting at 300 C and just needs to hold steady, those same aggressive gains would cause oscillation.

The solution: two sets of gains, switched automatically based on proximity to the setpoint.

```text
|<------------- approach mode ------------>|<-- hold mode -->|
                                           |                 |
  far from setpoint                 gainSwitchBandC    setpoint
```

- **Approach mode**: Uses `approachGains` (higher Kp, higher Ki). Active when `|error| > gainSwitchBandC`.
- **Hold mode**: Uses `holdGains` (lower Kp, lower Ki). Active when `|error| <= gainSwitchBandC`.

The `gainSwitchBandC` parameter controls where the switch happens. A typical value is 0.56 C (1 F).

### Setpoint approach conditioning

When the temperature is far below the setpoint and has not reached it yet, the PID temporarily aims slightly above the target. This "approach offset" helps the system arrive faster and reduces the time spent creeping up the last few degrees.

```text
effectiveSetpoint = setpoint + setpointApproachOffsetC   (when far below and first approach)
effectiveSetpoint = setpoint                             (after reaching the setpoint once)
```

The offset is active only when `currentTemp < setpoint - setpointApproachThresholdC` and the temperature has never reached the setpoint yet. Once it reaches the setpoint for the first time (the "reached setpoint latch" is set), the offset is permanently disabled until a reset.

### Hysteresis: when is "setpoint reached"?

The system uses hysteresis to decide when the temperature has "reached" the setpoint and when it has "lost" it. This prevents rapid toggling between states when the temperature is right at the boundary.

- Setpoint is considered **reached** when `currentTemp >= setpoint + hysteresisAboveSetpointC`
- Setpoint is considered **lost** when `currentTemp <= setpoint - hysteresisBelowSetpointC`

Between these two thresholds, the state does not change. Typical values: 0.44 C above, 0.67 C below.

### Ki reduction near setpoint

When the temperature is very close to the setpoint, the integral term can cause overshoot because it keeps pushing even though the error is small. The PID applies automatic Ki reduction in two bands:

- **Narrow band** (`|error| < kiReductionBandNarrowC`): Ki is multiplied by `kiReductionFactorNarrow` (e.g., 0.5 = half strength)
- **Wide band** (`|error| < kiReductionBandWideC`): Ki is multiplied by `kiReductionFactorWide` (e.g., 0.6 = 60% strength)

Outside both bands, full Ki applies.

### Anti-windup

When the heater is at maximum power (output saturated at 255), the integral term keeps growing because the error is still positive. When the temperature finally arrives, this bloated integral causes massive overshoot. Anti-windup prevents this.

When the output is saturated, the controller applies a back-calculation correction to the integral:

```text
correction = (saturated_output - unsaturated_output) * antiwindupGain * dt
```

The `antiwindupGain` controls how aggressively the integral is pulled back. A value of 0.2 is moderate. Higher values (0.5-1.0) pull back faster but can make the system sluggish during warmup.

### Derivative filtering

The raw temperature reading has noise. Taking the derivative of a noisy signal amplifies the noise. The controller uses an exponential moving average filter on the temperature before computing the derivative:

```text
filteredTemp = filteredTemp + alpha * (rawTemp - filteredTemp)
derivative = (filteredTemp - previousFilteredTemp) / dt
```

The `derivativeFilterAlpha` controls how much smoothing is applied:

- `alpha = 1.0`: No filtering. The derivative reacts instantly but is noisy.
- `alpha = 0.05`: Heavy filtering. The derivative is smooth but responds slowly.
- `alpha = 0.25` (typical): A reasonable compromise.

### Derivative enable/disable

The derivative term is not always active. When the temperature is far below the setpoint and rising, the derivative term would fight the heating (it opposes any fast change). This slows down the warmup for no benefit.

The derivative is enabled only when:

- The temperature is at or above the effective setpoint, OR
- The error is within `gainSwitchBandC + derivativeEnableMarginC` of the setpoint

In other words, the D term only kicks in when you are near the target or already overshooting.

## PID Tuning Guide

This section describes how to adjust the PID configuration when the temperature is not behaving as expected. All adjustments are made in the `PidConfig` struct that your project passes to `PidController` (or `HeaterChannelConfig` if using `HeaterChannel`).

### Reading the symptoms

Before changing anything, identify the problem. Run the system and monitor the temperature. Look at the PID diagnostic log or plot the temperature over time. Common patterns:

| Symptom | Likely cause | What to adjust |
| --- | --- | --- |
| Temperature oscillates ±5 F or more around setpoint | Kp or Ki too high in hold mode | Reduce `holdGains.kp` and/or `holdGains.ki` |
| Temperature sits 2-3 F below setpoint and never arrives | Ki too low, or Ki reduction too aggressive | Increase `holdGains.ki` or increase `kiReductionFactorNarrow` |
| Big overshoot on first warmup (10+ F) | Approach gains too aggressive, anti-windup too weak | Reduce `approachGains.kp`, increase `antiwindupGain` |
| Temperature creeps up very slowly near setpoint | Kp too low in approach mode, or gain switch band too wide | Increase `approachGains.kp`, or reduce `gainSwitchBandC` |
| Oscillation only when holding, not during warmup | Hold gains too aggressive | Reduce `holdGains.kp` by 20-30% |
| Slow recovery after a disturbance (door open, etc.) | Ki too low, or integral limit too tight | Increase `holdGains.ki` or increase `integralLimit` |
| Temperature "rings" (overshoots, undershoots, repeats) | Kd too low or derivative filter too heavy | Increase `holdGains.kd`, or increase `derivativeFilterAlpha` |

### Acceptable oscillation: the ±5 F example

Suppose your target is 520 F (271.1 C) and you want the temperature to stay within ±5 F (±2.78 C). That means the hold-mode controller must keep the error below 2.78 C.

**Step 1: Set `gainSwitchBandC` wide enough.** The gain switch band defines where hold mode activates. If it is 0.56 C (1 F), the system only enters hold mode when within 1 F. If oscillation peaks at ±5 F, the system is constantly bouncing between approach and hold modes, which makes things worse. Set `gainSwitchBandC` to at least 2.78 C (5 F) so hold mode covers the entire acceptable range:

```cpp
.gainSwitchBandC = 5.0 / 1.8   // 2.78 C = 5 F
```

**Step 2: Reduce hold-mode Kp.** If the proportional gain is too high, even a small error produces a large output swing, causing oscillation. Start by cutting it in half. If oscillation decreases, keep going. If the temperature becomes sluggish, you went too far.

```cpp
.holdGains = {4.0, 0.15, 2.0}   // halved Kp from 8 to 4
```

**Step 3: Reduce hold-mode Ki.** The integral term is often the primary cause of oscillation. It accumulates error over time and then overcorrects. Reduce Ki or tighten the Ki reduction bands:

```cpp
.holdGains = {4.0, 0.10, 2.0}           // reduced Ki from 0.15 to 0.10
.kiReductionBandNarrowC = 5.0 / 1.8     // apply reduction within 5 F
.kiReductionFactorNarrow = 0.3           // only 30% of Ki when very close
```

**Step 4: Increase derivative if oscillation persists.** The derivative term damps oscillation by opposing rapid temperature changes. If the system oscillates symmetrically around the setpoint, increasing Kd can help:

```cpp
.holdGains = {4.0, 0.10, 4.0}   // increased Kd from 2 to 4
```

Make sure `derivativeEnableMarginC` is wide enough that the derivative is actually active in the oscillation zone.

**Step 5: Verify warmup is still acceptable.** After tuning hold mode, run a full warmup from cold. The approach gains handle warmup, so reducing hold gains should not affect warmup speed. But if warmup overshoots the setpoint by more than your ±5 F band, reduce `approachGains.kp` or increase `antiwindupGain`.

### General tuning rules

1. **Change one parameter at a time.** PID parameters interact. If you change three things at once and it gets worse, you will not know which change caused it.
2. **Start with Kp, then Ki, then Kd.** Get the proportional response right first. Then add integral to eliminate steady-state offset. Then add derivative to damp oscillation.
3. **The integral term is the usual culprit for oscillation.** When in doubt, reduce Ki. A system that sits 1 F below setpoint is better than one that oscillates ±10 F.
4. **The approach gains affect warmup only.** Once the temperature is near the setpoint, hold gains take over. Tune them independently.
5. **All temperatures in the config are in Celsius.** If you think in Fahrenheit, divide by 1.8. For example, a ±5 F band is `5.0 / 1.8 = 2.78 C`.
6. **Test at the actual operating temperature.** PID behavior can change depending on the absolute temperature because the heater's thermal characteristics are not perfectly linear.

### Governor (Slew Rate Limiting)

The duty governor limits how fast the PWM output can change, preventing current spikes on heater outputs:

```cpp
// Default: 8% up / 3% down per 100ms (of 0-255 range)
// The governor is built into HeaterChannel — you don't need to manage it separately.
// But if you want to use it standalone:

DutyGovernor governor;
double smoothedDuty = governor.update(targetDuty, nowMs);
```

### Fan Tachometry

The fan controller tracks RPM from tachometer pulses. Call `recordPulse` from a pin-change ISR:

```cpp
// Template parameter N = number of fan channels in your project
static constexpr uint8_t NUM_FANS = 4;
FanController<NUM_FANS> fans;

// ISR for fan 0 tachometer
void IRAM_ATTR onFan0Pulse() {
    fans.recordPulse(0, micros());
}

void setup() {
    attachInterrupt(digitalPinToInterrupt(FAN0_TACH_PIN), onFan0Pulse, FALLING);
}

void loop() {
    if (fans.update(millis(), speedPercent)) {
        // RPM was recalculated (happens every FAN_SAMPLE_PERIOD_MS)
        for (uint8_t ch = 0; ch < NUM_FANS; ++ch) {
            auto state = fans.getChannelState(ch);
            Serial.printf("Fan %d: %d RPM, %s\n", ch, state.rpm,
                          state.connected ? "OK" : "DISCONNECTED");
        }
    }
}
```

A fan counts as "connected" if its RPM is above 50. Fan PWM output is inverted by default (0 = full speed, 255 = off).

### Using the PID Controller Standalone

If you need PID control for something other than heaters:

```cpp
PidConfig cfg = {
    .approachGains = {1.0, 0.1, 0.3},
    .holdGains = {0.5, 0.05, 0.2},
    .derivativeFilterAlpha = 0.25,
    .derivativeFilterAlphaMin = 0.05,
    .derivativeFilterAlphaMax = 1.0,
    .setpointApproachThresholdC = 2.8,
    .setpointApproachOffsetC = 1.1,
    .gainSwitchBandC = 0.56,
    .derivativeEnableMarginC = 0.28,
    .kiReductionBandNarrowC = 0.56,
    .kiReductionBandWideC = 1.11,
    .kiReductionFactorNarrow = 0.5,
    .kiReductionFactorWide = 0.6,
    .integralLimit = 500.0,
    .antiwindupGain = 0.2,
    .hysteresisAboveSetpointC = 0.44,
    .hysteresisBelowSetpointC = 0.67,
    .outputMax = 255.0,
};
PidController pid(cfg);

void loop() {
    double dt = 0.020;  // 20ms
    PidOutput out = pid.compute(currentTemp, setpoint, dt);
    analogWrite(heaterPin, static_cast<int>(out.rawOutput));
}
```

The `PidOutput` struct gives you the full breakdown: `rawOutput`, `pTerm`, `iTerm`, `dTerm`, `derivativeCps`, `effectiveSetpoint`, `inHoldMode`.

## Channel Counts

The number of heater and fan channels is **not defined by the library**. Each application provides its own count:

- `HeaterChannel` is a single-channel class — instantiate as many as you need (array, individual objects, etc.)
- `FanController<N>` and `FanTachometer<N>` are class templates — the channel count `N` is a compile-time parameter

This means you can use the same library for a 1-channel bench heater or an 8-channel industrial oven without any library changes.

```cpp
// A single-fan project
FanController<1> fan;

// A 4-zone reflow station
static constexpr uint8_t NUM_ZONES = 4;
HeaterChannel heaters[NUM_ZONES] = { ... };
FanController<NUM_ZONES> fans;

// An 8-channel oven
FanController<8> fans;
```

## Configuration

The library has **no hardcoded project-specific constants**. All tuning parameters are injected via config structs at construction time:

| Config Struct | Used By | Key Fields |
| --- | --- | --- |
| `PidConfig` | `PidController` | Approach/hold gains, filtering, hysteresis, anti-windup |
| `NtcConfig` | `TemperatureSensor`, `ntcMillivoltsToTempC()` | NTC circuit params, validation ranges |
| `DutyFloorConfig` | `DutyFloorCalculator` | Floor duty thresholds, setpoint range |
| `GovernorConfig` | `DutyGovernor` | Ramp fractions, update period, PWM resolution |
| `RateLimiterConfig` | `RateLimiter` | Rate threshold, max rate, reduction factor |
| `FanTachConfig` | `FanTachometer` | Pulses/rev, min RPM, sample period, debounce |
| `FanOutputConfig` | `FanPwmCalculator` | PWM resolution, inverted output |
| `HeaterChannelConfig` | `HeaterChannel` | Bundles all of the above |

## Testing

```shell
cd lib_thermal/tests
chmod +x *.sh
./1_build.sh
./2_run.sh
```

Or manually:

```shell
cd lib_thermal/tests
cmake -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

No external dependencies are needed for testing — the test suite is self-contained (GoogleTest is fetched automatically).

| Suite | What it covers |
| --- | --- |
| `PidController` | Proportional response, gain switching, integral clamping, derivative filter, bumpless transfer |
| `NtcConversion` | ADC-to-temperature, disconnected detection, monotonicity |
| `HeaterChannel` | Full pipeline: sensor → PID → governor → duty output |
| `FanController` | Tachometer RPM, inverted PWM, connection detection |
| `DutyGovernor` | Slew rate limiting, ramp-up/ramp-down |

## Dependencies

This library has **no runtime dependencies**. All thermal algorithms (PID, NTC conversion, fan tachometry) are pure math with no external library calls.

The Arduino build system discovers the library via `library.properties`. No sibling repos are required for building or testing.

## Acknowledgements

Thanks to Claude and ChatGPT for helping on generating this documentation.

## License

MIT License — see [LICENSE](license.txt) file.

---

## Arduino CLI symlink note (rarely relevant)

This library ships a flat forwarder header at `src/ungula_thermal.h` that
just `#include`s `ungula/thermal.h`. `library.properties` `includes=` points
at the forwarder.

It only exists to work around an Arduino CLI quirk: when the library is
consumed through a symlink, the CLI sometimes fails to discover headers
nested under `src/ungula/`. The flat forwarder fixes that scan.

**Host code keeps including the real header**:

```cpp
#include <ungula/thermal.h>
```

PlatformIO, ESP-IDF component builds, and plain CMake setups can ignore
the forwarder.
