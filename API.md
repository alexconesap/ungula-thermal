# UngulaThermal

Portable embedded C++17 thermal control. Provides a dual-mode PID controller, a single-channel `HeaterChannel` (sensor → PID → duty floor → governor → rate limiter pipeline), an N-channel `FanController` with tachometry, and an NTC `TemperatureSensor`. Pure C++, no Arduino, no logging, no heap after construction. The host wires ADC, PWM and ISRs.

All temperatures are Celsius. PWM duty is a `double` clamped to `[0, outputMax]` (typically 255). Fan speed commands are `int` percentages 0–100.

---

## Usage

All examples assume:

```cpp
#include <ungula_thermal.h>          // pulls all four headers
using namespace ungula::thermal;
```

The library declares pure types. The host supplies time (`nowMs`, `dtSeconds`), ADC reads, PWM writes and ISRs.

### Use case: single heater channel, full pipeline

```cpp
#include <ungula_thermal.h>
#include <Arduino.h>

using namespace ungula::thermal;

static const HeaterChannelConfig kHeaterCfg = {
    .pid = {
        .approachGains            = {2.0, 0.05, 0.5},
        .holdGains                = {0.8, 0.02, 0.3},
        .derivativeFilterAlpha    = 0.25,
        .derivativeFilterAlphaMin = 0.05,
        .derivativeFilterAlphaMax = 1.0,
        .setpointApproachThresholdC = 2.8,
        .setpointApproachOffsetC    = 1.1,
        .gainSwitchBandC            = 0.56,
        .derivativeEnableMarginC    = 0.28,
        .kiReductionBandNarrowC     = 0.56,
        .kiReductionBandWideC       = 1.11,
        .kiReductionFactorNarrow    = 0.5,
        .kiReductionFactorWide      = 0.6,
        .integralLimit              = 500.0,
        .antiwindupGain             = 0.2,
        .hysteresisAboveSetpointC   = 0.44,
        .hysteresisBelowSetpointC   = 0.67,
        .outputMax                  = 255.0,
    },
    .floor = {
        .atSetpointLow      = 30.0,  .atSetpointHigh     = 60.0,
        .belowSpCloseLow    = 40.0,  .belowSpCloseHigh   = 80.0,
        .belowSpFarLow      = 60.0,  .belowSpFarHigh     = 120.0,
        .aboveSpBaseOffset  = 0.0,   .aboveSpMinimum     = 0.0,
        .aboveSpAtMaxError  = 0.0,
        .errorThresholdFarC = 5.6,   .errorThresholdMidC = 2.8,
        .errorThresholdCloseC = 1.1, .holdBandC          = 0.56,
        .setpointMinF = 200.0,       .setpointMaxF       = 700.0,
        .pwmResolution = 255,
    },
    .governor = {
        .rampUpFraction = 0.08, .rampDownFraction = 0.03,
        .updatePeriodMs = 100,  .pwmResolution    = 255,
    },
    .rateLimiter = {
        .thresholdCps = 2.0, .maxRateCps = 6.0,
        .outputReductionFactor = 0.5, .pwmResolution = 255,
    },
    .ntc = {
        .seriesResistorOhms        = 10000.0,
        .nominalResistanceOhms     = 100000.0,
        .betaCoefficient           = 3950.0,
        .referenceTempC            = 25.0,
        .supplyVoltageMv           = 3300,
        .disconnectedMarginMv      = 50.0,
        .minReasonableResistanceOhms = 100.0,
        .maxReasonableResistanceOhms = 5'000'000.0,
        .maxValidTempC = 350.0, .minValidTempC = -40.0,
    },
};

HeaterChannel heater(/*index=*/0, /*thermistorPin=*/36, /*heaterPin=*/25,
                     /*calibrationOffsetC=*/-2.5, kHeaterCfg);

void setup() {
    heater.setSetpointC(260.0);   // Celsius only
}

void loop() {
    const uint32_t now   = millis();
    const int      adcMv = analogReadMilliVolts(36);
    const double   dt    = 0.020;             // 20 ms loop

    HeaterChannelState s = heater.update(adcMv, now, dt);

    analogWrite(25, static_cast<int>(s.currentDuty));

    if (!heater.isSensorConnected()) {
        analogWrite(25, 0);                   // SAFE state: heater off
    }

    delay(20);
}
```

When to use this: a single heated zone with NTC feedback and PWM driver. `HeaterChannel::update()` runs the whole sensor → PID → floor → governor → rate-limiter pipeline; the host only writes the returned `currentDuty` to PWM.

### Use case: standalone PID for a non-heater plant

```cpp
#include <thermal/pid_controller.h>
using namespace ungula::thermal;

static PidConfig kPid = { /* same fields as above */ };
PidController pid(kPid);

void controlStep(double processValue, double setpoint) {
    PidOutput out = pid.compute(processValue, setpoint, 0.020);
    // out.rawOutput is already clamped to [0, outputMax].
    // out.inHoldMode, out.effectiveSetpoint, out.pTerm/iTerm/dTerm
    // are exposed for diagnostics only.
}
```

When to use this: any SISO control problem that fits the dual-mode (approach/hold) shape. No hardware tie-in.

### Use case: N-channel fan controller with tachometry

```cpp
#include <ungula_thermal.h>
#include <Arduino.h>

using namespace ungula::thermal;

static constexpr uint8_t NUM_FANS = 4;

static const FanTachConfig kTach = {
    .pulsesPerRev      = 2,
    .minRpmThreshold   = 50,
    .samplePeriodMs    = 1000,
    .minEdgeIntervalUs = 200,
};
static const FanOutputConfig kOut = {
    .pwmResolution = 255,
    .invertedOutput = true,    // 0 = full speed, 255 = off
};

FanController<NUM_FANS> fans(kTach, kOut);

void IRAM_ATTR onFan0Pulse() { fans.recordPulse(0, micros()); }

void setup() {
    attachInterrupt(digitalPinToInterrupt(34), onFan0Pulse, FALLING);
}

void loop() {
    const int speedPct = 80;
    if (fans.update(millis(), speedPct)) {
        // sample period elapsed; RPM and connected flag refreshed
    }
    const uint16_t duty = fans.calculateOutputDuty(speedPct);
    analogWrite(/*fanPwmPin=*/26, duty);
}
```

When to use this: any project that drives 4-wire PC fans, needs RPM feedback, or needs to detect a stalled / disconnected fan. `N` is a compile-time channel count.

### Use case: NTC ADC reading without HeaterChannel

```cpp
#include <thermal/temp_sensor.h>
using namespace ungula::thermal;

static const NtcConfig kNtc = { /* see HeaterChannel example */ };

TemperatureSensor sensor(/*channelIndex=*/0, /*adcPin=*/36,
                         /*calibrationOffsetC=*/0.0, kNtc);

double readTemp(int adcMv) {
    return sensor.readTemperatureC(adcMv);   // NaN if invalid
}

bool sensorOk() { return sensor.isConnected(); }
```

When to use this: temperature monitoring without a control loop, or when the heater PWM is driven by something other than `HeaterChannel`.

### Use case: bumpless transfer when taking over PID control

```cpp
heater.initializeBumpless(/*currentDuty=*/120.0);
// or, on the raw PID:
pid.initializeForBumplessTransfer(/*currentOutput=*/120.0,
                                  /*currentTemp=*/250.0,
                                  /*setpoint=*/260.0);
```

When to use this: switching from manual / open-loop drive into closed-loop without a duty step.

---

## Public types

### `PidGains`
`{ double kp; double ki; double kd; }`. One set per mode.

### `PidConfig`
Full PID tuning. Fields:

- `approachGains`, `holdGains` — `PidGains`. Selected by `|error| > gainSwitchBandC`.
- `derivativeFilterAlpha` — EMA alpha on the temperature feeding D term. `1.0` = no filter, `0.05` = heavy.
- `derivativeFilterAlphaMin`, `derivativeFilterAlphaMax` — clamps for `setDerivativeFilterAlpha()`.
- `setpointApproachThresholdC` / `setpointApproachOffsetC` — when below setpoint by more than the threshold and the "reached" latch is unset, the PID aims at `setpoint + offset`.
- `gainSwitchBandC` — half-width of the hold band.
- `derivativeEnableMarginC` — D term active only when `temp ≥ effectiveSetpoint` or within `gainSwitchBandC + derivativeEnableMarginC`.
- `kiReductionBandNarrowC` / `kiReductionFactorNarrow`, `kiReductionBandWideC` / `kiReductionFactorWide` — Ki scaled down near setpoint.
- `integralLimit` — symmetric clamp on the I accumulator.
- `antiwindupGain` — back-calculation gain when output is saturated.
- `hysteresisAboveSetpointC`, `hysteresisBelowSetpointC` — latching of the "reached setpoint" flag.
- `outputMax` — upper clamp; lower is fixed at `0.0` unless `setOutputLimits()` is used.

### `PidOutput`
Per-step diagnostics: `rawOutput` (clamped), `pTerm`, `iTerm`, `dTerm`, `derivativeCps`, `effectiveSetpoint`, `inHoldMode`.

### `NtcConfig`
NTC divider parameters and validity gates. `disconnectedMarginMv` plus `[minReasonableResistanceOhms, maxReasonableResistanceOhms]` and `[minValidTempC, maxValidTempC]` define the connected window. Outside this window the sensor returns `NaN` and `isConnected()` returns false.

### `GovernorConfig`
`rampUpFraction`, `rampDownFraction` are fractions of `pwmResolution` allowed per `updatePeriodMs`.

### `RateLimiterConfig`
When `|derivativeCps| > thresholdCps`, target duty is multiplied by `outputReductionFactor`; saturating limit is `maxRateCps`.

### `DutyFloorConfig`
Lookup-table floor duty depending on signed error vs setpoint, plus a hold band and a setpoint range (`setpointMinF`, `setpointMaxF`, **Fahrenheit**, used only as normalization input — every other temperature in the library is Celsius).

### `HeaterChannelConfig`
Bundle: `pid`, `floor`, `governor`, `rateLimiter`, `ntc`. Pass once at construction.

### `HeaterChannelState`
Returned by `HeaterChannel::update()`:
`currentDuty`, `targetDuty`, `floorDuty`, `floorReason` (`FloorReason`), `temperatureC`, `setpointC`, `derivativeCps`, `rateLimitActive`, `lastGovernorUpdateMs`.

### `FloorReason`
`enum class : uint8_t { None, BelowSetpoint, AboveSetpoint, HoldBand, RateLimited }`.

### `FanTachConfig` / `FanOutputConfig` / `FanChannelState`
Tachometer setup, PWM mapping, and per-channel `{ pulseCount, lastEdgeUs, rpm, connected }`.

---

## Public functions / methods

### Free functions

```cpp
double ntcMillivoltsToTempC(int millivolts, const NtcConfig& cfg);
double applyCalibrationOffset(double rawTempC, double offsetC);
inline double celsiusToFahrenheit(double c);    // <thermal/conversions.h>
inline double fahrenheitToCelsius(double f);
```

`ntcMillivoltsToTempC()` returns `NaN` if the reading violates any validity gate.

### `class PidController`

- `explicit PidController(const PidConfig&)` — copies config, zeroes state.
- `void reset()` — clears integral, filtered temp, derivative, "reached setpoint" latch.
- `void setGains(const PidGains& approach, const PidGains& hold)`.
- `void setDerivativeFilterAlpha(double)` — clamped to `[alphaMin, alphaMax]`.
- `double getDerivativeFilterAlpha() const`.
- `void setOutputLimits(double minOutput, double maxOutput)` — overrides the default `[0, outputMax]`.
- `void initializeForBumplessTransfer(double currentOutput, double currentTemp, double setpoint)` — seeds integral so the next `compute()` returns ~`currentOutput`.
- `PidOutput compute(double processValue, double setpoint, double dtSeconds)` — main step. `dtSeconds` must be > 0. Output is clamped.
- Getters: `getIntegral()`, `getFilteredTemperature()`, `getLastDerivative()`, `hasReachedSetpoint()`.

Side effects: none beyond own state. No hardware. No allocation after construction.

### `class HeaterChannel`

- `HeaterChannel(uint8_t index, int thermistorPin, int heaterPin, double calibrationOffsetC, const HeaterChannelConfig& cfg)` — pins are stored verbatim; the class never touches them itself, the host owns ADC/PWM I/O.
- `void setSetpointC(double)`, `void setSetpointTrimC(double)`, `double getEffectiveSetpointC() const` (= `setpointC + setpointTrimC`).
- `void setDerivativeFilterAlpha(double)`.
- `void setApproachGains`, `setHoldGains`, `setPidGains(const PidGains&, const PidGains&)`.
- `HeaterChannelState update(int adcMillivolts, uint32_t nowMs, double dtSeconds)` — runs sensor → PID → floor → governor → rate limiter. Returns the new state. The host writes `state.currentDuty` to PWM.
- `void resetControl()` — resets PID and governor; sensor history kept.
- `void initializeBumpless(double currentDuty)`.
- Getters: `getTemperatureC`, `getTemperatureF`, `isSensorConnected`, `getDutyPwm`, `getState`, `getIndex`, `getHeaterPin`.

Failure behavior: when `isSensorConnected()` is false, `temperatureC` will be NaN. The library does **not** force the duty to zero — the host must check and apply its SAFE state (see Safety).

### `class DutyGovernor`

- `explicit DutyGovernor(const GovernorConfig&)`.
- `void setConfig(const GovernorConfig&)`.
- `void reset(double initialDuty = 0.0)`.
- `double update(double targetDuty, uint32_t nowMs)` — slew-rate-limited duty.
- `double getCurrentDuty() const`.

### `class DutyFloorCalculator`

- `explicit DutyFloorCalculator(const DutyFloorConfig&)`.
- `double computeFloor(double tempC, double setpointC, FloorReason& reasonOut) const`.

### `class RateLimiter`

- `explicit RateLimiter(const RateLimiterConfig&)`.
- `void setConfig(const RateLimiterConfig&)`.
- `double applyLimit(double targetDuty, double derivativeCps, bool& limitActiveOut) const`.

### `class TemperatureSensor`

- `TemperatureSensor(uint8_t channelIndex, int adcPin, double calibrationOffsetC, const NtcConfig&)`.
- `void setCalibrationOffset(double)`, `double getCalibrationOffset() const`.
- `void setNtcConfig(const NtcConfig&)`.
- `double readTemperatureC(int adcMillivolts)` — returns NaN if invalid; updates internal "last" cache and connected flag.
- `bool isConnected() const` — based on the last `readTemperatureC` call.
- Getters: `getLastRawTempC`, `getLastCalibratedTempC`, `getLastAdcMillivolts`, `getChannelIndex`, `getAdcPin`.

### `template<uint8_t N> class FanTachometer`

- `explicit FanTachometer(const FanTachConfig&)`.
- `bool recordPulse(uint8_t channel, uint32_t timestampUs)` — **ISR-safe** pulse counter; debounces with `minEdgeIntervalUs`. Returns false on invalid channel or debounce.
- `void update(bool commandedOff)` — recomputes RPM from accumulated pulses; clears pulse count. `commandedOff=true` forces `connected=false` regardless of RPM.
- `const FanChannelState& getChannelState(uint8_t) const`.
- `void getConnectedStatus(bool* out)`, `void getRpmValues(uint16_t* out)`.
- `void reset()`, `uint32_t getPulseCount(uint8_t) const`.
- Compile-time: `static constexpr uint8_t NUM_CHANNELS = N`.

Note: `update()`'s RPM math assumes its caller times it to `samplePeriodMs ≈ 1000 ms`. Outside of that the formula `(pulses * 60) / pulsesPerRev` is misleading. `FanController::update()` enforces this period.

### `class FanPwmCalculator`

- `explicit FanPwmCalculator(const FanOutputConfig&)`.
- `uint16_t calculateDuty(int speedPercent) const` — clamps `speedPercent` to `[0,100]`, scales to `pwmResolution`, optionally inverts.

### `template<uint8_t N> class FanController`

- `FanController(const FanTachConfig&, const FanOutputConfig&)`.
- `bool recordPulse(uint8_t, uint32_t)` — ISR-safe wrapper.
- `bool update(uint32_t nowMs, int speedCommandPercent)` — returns `true` only when the sample period elapsed and RPM was refreshed.
- `uint16_t calculateOutputDuty(int speedPercent) const`.
- `const FanChannelState& getChannelState(uint8_t) const`.
- `void getConnectedStatus(bool*)`.
- `bool isAnyFanConnected() const`, `bool areAllFansConnected() const`.
- `void reset()`.

### `namespace adc` (helpers)

- `void sortFiveElements(int[5])`.
- `int computeMedianOfFive(int[5])`.
- `template<class F> int readWithMedianFiltering(F readOnce, uint8_t totalSamples, uint8_t groupSize)`.
- `template<class F> int readWithAveraging(F readOnce, uint8_t sampleCount)`.
- `inline int rawToMillivolts(int raw, uint16_t supplyMv, uint16_t adcResolution)`.

---

## Lifecycle

1. **Construct** every object with its config struct at boot. No object has a default constructor.
2. **Configure**: optionally call `setSetpointC`, `setPidGains`, `setCalibrationOffset`, `setDerivativeFilterAlpha`.
3. **Operate** in the host's control loop:
   - Heater: `update(adcMv, nowMs, dt)` → write `state.currentDuty` to PWM.
   - Fans: call `recordPulse()` from the tach ISR; call `update(nowMs, speedPct)` once per loop; write `calculateOutputDuty(speedPct)` to PWM.
4. **Reset / takeover**: `resetControl()` zeroes PID and governor; `initializeBumpless()` seeds the integral so closed-loop output matches an existing manual duty.

Violating the order: calling `update()` with `dtSeconds <= 0` will divide by zero; calling `setSetpointC` mid-warmup is fine; reusing the same `HeaterChannelConfig` for several channels is fine (it is copied).

---

## Error handling

The library returns no exceptions and no error codes. Failure is communicated by:

- `TemperatureSensor::isConnected() == false` and `readTemperatureC()` / `getLastCalibratedTempC()` returning **NaN** when the NTC is open, shorted, or the temperature is outside `[minValidTempC, maxValidTempC]`.
- `HeaterChannel::isSensorConnected() == false` mirrors the above.
- `HeaterChannelState.floorReason == RateLimited` and `rateLimitActive == true` indicate the rate limiter intervened.
- `FanChannelState.connected == false` when measured RPM ≤ `minRpmThreshold` or speed command is 0.

The host is responsible for converting these signals into corrective action (see Safety).

---

## Threading / timing / hardware notes

- `FanTachometer::recordPulse()` and `FanController::recordPulse()` are designed to be called from a falling-edge / pin-change ISR. They do not allocate, do not block, and only mutate the target channel's `pulseCount` and `lastEdgeUs`. **No other method is ISR-safe.**
- `recordPulse()` is not atomic against `update()` running on the loop thread. On single-core ESP32 this is benign because the ISR preempts. On multi-core systems pin both to the same core or guard with a critical section.
- All other methods are non-thread-safe — call them from a single task.
- `HeaterChannel::update()` is non-blocking. Cost is roughly one PID step + a small lookup.
- `dtSeconds` for `PidController::compute()` and `HeaterChannel::update()` must be the actual elapsed time and > 0.
- The library never calls `millis()`, `analogRead*()`, `analogWrite()`, `attachInterrupt()`, or `Serial`. The host wires all of that.

---

## Safety (safety-critical usage)

This library is a control kernel, not a safety supervisor. There is no built-in over-temperature cutoff, no watchdog, and no heater enable line. The host project must enforce SAFE states.

Recommended SAFE pattern for a heater:

```cpp
HeaterChannelState s = heater.update(adcMv, now, dt);

bool fault = !heater.isSensorConnected()
          || std::isnan(s.temperatureC)
          || s.temperatureC > MAX_ALLOWED_C
          || (now - s.lastGovernorUpdateMs) > MAX_STALE_MS;

analogWrite(heaterPin, fault ? 0 : static_cast<int>(s.currentDuty));
```

Library-level safety surface:

- **Sensor validity**: `NtcConfig.minValidTempC`, `maxValidTempC`, `disconnectedMarginMv`, and the resistance window define the "trusted" envelope. Outside of it, `temperatureC = NaN`.
- **Output clamp**: `PidConfig.outputMax` is the absolute ceiling on duty. `PidController::setOutputLimits` may narrow it further at runtime.
- **Rate limit**: `RateLimiter` reduces the duty when the temperature derivative exceeds `thresholdCps`, and signals the limit via `state.rateLimitActive`. This is a stability guard, not a safety guard.
- **Slew limit**: `DutyGovernor` caps how fast the heater can ramp up or down per `updatePeriodMs`. Tune `rampUpFraction` to limit current-spike risk.
- **Anti-windup**: `antiwindupGain` prevents the integral from blowing up while saturated; this reduces overshoot, which in turn reduces over-temperature events.

There is no FSM in this library. Heater state is implicit (sensor connected/disconnected, in-hold-mode latch, rate-limit flag); the host must aggregate these flags into its own safety FSM.

---

## Internals not part of the public API

- `PidController::computeEffectiveSetpoint`, `selectGains`, `applyKiReduction` — private helpers; the dual-mode behavior is observable through `PidOutput.inHoldMode` and `effectiveSetpoint`.
- `DutyFloorCalculator::normalizeSetpoint`, `computeFloorBelowSetpoint`, `computeFloorAboveSetpoint`, `computeHoldFloor` — private; the public surface is `computeFloor()` and `FloorReason`.
- `HeaterChannel` private members (`sensor_`, `pid_`, `governor_`, `floorCalc_`, `rateLimiter_`, `state_`) — do not introspect; use `getState()` and `getTemperatureC()`.
- `FanChannelState::pulseCount` is a transient counter cleared every `update()`. Treat it as opaque.
- `library.properties` `setpointMinF` / `setpointMaxF` use Fahrenheit purely as the floor calculator's normalization range; do not interpret them as a project-wide unit choice. All other temperatures are Celsius.
- The README's Quick Start uses a 4-arg `HeaterChannel` constructor and a default-constructed `FanController<2>`. **The actual constructors require a `HeaterChannelConfig` and a `(FanTachConfig, FanOutputConfig)` pair respectively.** Trust the headers, not the README.

---

## Recommended improvements (proposed — not implemented)

The current public surface is broad: every sub-block (`DutyGovernor`, `DutyFloorCalculator`, `RateLimiter`, `TemperatureSensor`, `FanTachometer`, `FanPwmCalculator`) is exposed even though `HeaterChannel` and `FanController` already compose them. This invites accidental coupling.

Proposed deepening (none of this exists yet):

1. Make `DutyGovernor`, `DutyFloorCalculator`, `RateLimiter`, `FanPwmCalculator`, `FanTachometer` `internal` (move headers under `src/thermal/internal/`) and re-expose only `HeaterChannel`, `FanController`, `PidController`, `TemperatureSensor`, plus the config structs.
2. Add a first-class fault enum returned by `HeaterChannel::update()` (e.g. `HeaterFault::None | SensorOpen | OverTemp | StaleClock`) so the host SAFE check is one comparison, not four.
3. Add a unit-tagged temperature type (`Celsius{double}`) to remove the `setpointMinF` / `setpointMaxF` Fahrenheit leak in `DutyFloorConfig`.
4. Provide a host-side helper that wraps the recommended SAFE pattern (`heater.updateSafe(adcMv, now, dt, &dutyOut)` returning `HeaterFault`).
5. Add a static maximum-duty hard ceiling separate from `outputMax`, latched at construction, that no setter can raise.

---

## LLM usage rules

- Use only the symbols documented above. Do not include private headers or call private methods.
- Trust the headers, not `README.md`, when they disagree (the Quick Start in README has stale constructor signatures).
- Prefer `HeaterChannel` over assembling `PidController + DutyGovernor + DutyFloorCalculator + RateLimiter + TemperatureSensor` by hand.
- Never call `millis()`, `analogRead*()`, `analogWrite()`, or `attachInterrupt()` from inside this library — those are the host's job.
- Always handle `isSensorConnected() == false` by forcing the heater output to zero (or another host-defined SAFE value).
- Always check `dtSeconds > 0` before calling `PidController::compute()` or `HeaterChannel::update()`.
- For fans, `recordPulse()` is the only method safe to call from an ISR.
- All temperatures used with this API are Celsius. Convert at the host boundary using `celsiusToFahrenheit()` / `fahrenheitToCelsius()` from `thermal/conversions.h`.
- Do not invent a fault enum, a SAFE wrapper, or a unit-typed temperature — they do not exist yet (see "Recommended improvements").
