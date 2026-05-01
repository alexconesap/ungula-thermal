#include <gtest/gtest.h>
#include <thermal/heater_control.h>

#include <cmath>

using namespace ungula::thermal;

// ---- Test-local constants (values from the old thermal_config.h) ----

static constexpr double FAHRENHEIT_TO_CELSIUS_FACTOR = 1.8;
static constexpr uint16_t PWM_RESOLUTION = 255;
static constexpr double GOVERNOR_RAMP_UP_FRACTION = 0.08;
static constexpr double GOVERNOR_RAMP_DOWN_FRACTION = 0.03;
static constexpr uint32_t GOVERNOR_UPDATE_PERIOD_MS = 100;
static constexpr double FLOOR_ERROR_THRESHOLD_FAR_C = 10.0 / FAHRENHEIT_TO_CELSIUS_FACTOR;
static constexpr double RATE_LIMIT_THRESHOLD_CPS = 2.0;
static constexpr double RATE_LIMIT_MAX_CPS = 5.0;
static constexpr double RATE_LIMIT_OUTPUT_REDUCTION = 0.5;
static constexpr double DEFAULT_CALIBRATION_OFFSET_C = -75.0;

static inline GovernorConfig testGovernorConfig() {
    return GovernorConfig{
            .rampUpFraction = GOVERNOR_RAMP_UP_FRACTION,
            .rampDownFraction = GOVERNOR_RAMP_DOWN_FRACTION,
            .updatePeriodMs = GOVERNOR_UPDATE_PERIOD_MS,
            .pwmResolution = PWM_RESOLUTION,
    };
}

static inline RateLimiterConfig testRateLimiterConfig() {
    return RateLimiterConfig{
            .thresholdCps = RATE_LIMIT_THRESHOLD_CPS,
            .maxRateCps = RATE_LIMIT_MAX_CPS,
            .outputReductionFactor = RATE_LIMIT_OUTPUT_REDUCTION,
            .pwmResolution = PWM_RESOLUTION,
    };
}

static inline DutyFloorConfig testDutyFloorConfig() {
    return DutyFloorConfig{
            .atSetpointLow = 0.88,
            .atSetpointHigh = 0.90,
            .belowSpCloseLow = 0.86,
            .belowSpCloseHigh = 0.93,
            .belowSpFarLow = 0.91,
            .belowSpFarHigh = 0.97,
            .aboveSpBaseOffset = 0.22,
            .aboveSpMinimum = 0.50,
            .aboveSpAtMaxError = 0.15,
            .errorThresholdFarC = FLOOR_ERROR_THRESHOLD_FAR_C,
            .errorThresholdMidC = 2.0 / FAHRENHEIT_TO_CELSIUS_FACTOR,
            .errorThresholdCloseC = 0.5 / FAHRENHEIT_TO_CELSIUS_FACTOR,
            .holdBandC = 6.0 / FAHRENHEIT_TO_CELSIUS_FACTOR,
            .setpointMinF = 350.0,
            .setpointMaxF = 600.0,
            .pwmResolution = PWM_RESOLUTION,
    };
}

static inline NtcConfig testNtcConfig() {
    return NtcConfig{
            .seriesResistorOhms = 470.0,
            .nominalResistanceOhms = 100000.0,
            .betaCoefficient = 3950.0,
            .referenceTempC = 25.0,
            .supplyVoltageMv = 3300,
            .disconnectedMarginMv = 2.0,
            .minReasonableResistanceOhms = 1.0,
            .maxReasonableResistanceOhms = 5000000.0,
            .maxValidTempC = 350.0,
            .minValidTempC = -40.0,
    };
}

static inline PidConfig testPidConfig() {
    return PidConfig{
            .approachGains = {36.0 / FAHRENHEIT_TO_CELSIUS_FACTOR,
                              0.55 / FAHRENHEIT_TO_CELSIUS_FACTOR,
                              5.5 / FAHRENHEIT_TO_CELSIUS_FACTOR},
            .holdGains = {16.0 / FAHRENHEIT_TO_CELSIUS_FACTOR, 0.32 / FAHRENHEIT_TO_CELSIUS_FACTOR,
                          5.5 / FAHRENHEIT_TO_CELSIUS_FACTOR},
            .derivativeFilterAlpha = 0.25,
            .derivativeFilterAlphaMin = 0.05,
            .derivativeFilterAlphaMax = 1.0,
            .setpointApproachThresholdC = 5.0 / FAHRENHEIT_TO_CELSIUS_FACTOR,
            .setpointApproachOffsetC = 2.0 / FAHRENHEIT_TO_CELSIUS_FACTOR,
            .gainSwitchBandC = 1.0 / FAHRENHEIT_TO_CELSIUS_FACTOR,
            .derivativeEnableMarginC = 0.5 / FAHRENHEIT_TO_CELSIUS_FACTOR,
            .kiReductionBandNarrowC = 1.0 / FAHRENHEIT_TO_CELSIUS_FACTOR,
            .kiReductionBandWideC = 2.0 / FAHRENHEIT_TO_CELSIUS_FACTOR,
            .kiReductionFactorNarrow = 0.5,
            .kiReductionFactorWide = 0.6,
            .integralLimit = 500.0,
            .antiwindupGain = 0.2,
            .hysteresisAboveSetpointC = 0.8 / FAHRENHEIT_TO_CELSIUS_FACTOR,
            .hysteresisBelowSetpointC = 1.2 / FAHRENHEIT_TO_CELSIUS_FACTOR,
            .outputMax = 255.0,
    };
}

static inline HeaterChannelConfig testHeaterChannelConfig() {
    return HeaterChannelConfig{
            .pid = testPidConfig(),
            .floor = testDutyFloorConfig(),
            .governor = testGovernorConfig(),
            .rateLimiter = testRateLimiterConfig(),
            .ntc = testNtcConfig(),
    };
}

// --- DutyGovernor ---

TEST(DutyGovernor, FirstUpdateJumpsToTarget) {
    DutyGovernor gov(testGovernorConfig());
    double duty = gov.update(200.0, 1000);
    EXPECT_DOUBLE_EQ(duty, 200.0);
}

TEST(DutyGovernor, RateLimitsLargeStepUp) {
    DutyGovernor gov(testGovernorConfig());
    gov.update(0.0, 1000);  // Initialize at 0

    // After period elapses, step to 255
    double duty = gov.update(255.0, 1000 + GOVERNOR_UPDATE_PERIOD_MS);
    double maxStep = GOVERNOR_RAMP_UP_FRACTION * PWM_RESOLUTION;
    EXPECT_LE(duty, maxStep + 0.01);
}

TEST(DutyGovernor, RateLimitsLargeStepDown) {
    DutyGovernor gov(testGovernorConfig());
    gov.update(255.0, 1000);  // Initialize at 255

    // Step down to 0
    double duty = gov.update(0.0, 1000 + GOVERNOR_UPDATE_PERIOD_MS);
    double maxStepDown = GOVERNOR_RAMP_DOWN_FRACTION * PWM_RESOLUTION;
    EXPECT_GE(duty, 255.0 - maxStepDown - 0.01);
}

TEST(DutyGovernor, IgnoresUpdateWithinPeriod) {
    DutyGovernor gov(testGovernorConfig());
    gov.update(100.0, 1000);

    // Update within period window should not change
    double duty = gov.update(200.0, 1000 + GOVERNOR_UPDATE_PERIOD_MS / 2);
    EXPECT_DOUBLE_EQ(duty, 100.0);
}

TEST(DutyGovernor, ResetSetsInitialDuty) {
    DutyGovernor gov(testGovernorConfig());
    gov.update(200.0, 1000);
    gov.reset(50.0);

    EXPECT_DOUBLE_EQ(gov.getCurrentDuty(), 50.0);
}

// --- DutyFloorCalculator ---

TEST(DutyFloor, FarBelowSetpointReturnsFullFloor) {
    DutyFloorCalculator calc(testDutyFloorConfig());
    FloorReason reason;
    double setpoint = 290.0;
    double temp = setpoint - FLOOR_ERROR_THRESHOLD_FAR_C - 10.0;

    double floor = calc.computeFloor(temp, setpoint, reason);
    EXPECT_EQ(reason, FloorReason::BelowSetpoint);
    // Far below setpoint should return PWM_RESOLUTION (1.0 * 255)
    EXPECT_NEAR(floor, static_cast<double>(PWM_RESOLUTION), 1.0);
}

TEST(DutyFloor, AboveSetpointReducesFloor) {
    DutyFloorCalculator calc(testDutyFloorConfig());
    FloorReason reason;
    double setpoint = 290.0;
    double temp = setpoint + 3.0;

    double floor = calc.computeFloor(temp, setpoint, reason);
    EXPECT_EQ(reason, FloorReason::AboveSetpoint);
    EXPECT_LT(floor, static_cast<double>(PWM_RESOLUTION));
    EXPECT_GT(floor, 0.0);
}

TEST(DutyFloor, FarAboveSetpointReturnsMinFloor) {
    DutyFloorCalculator calc(testDutyFloorConfig());
    FloorReason reason;
    double setpoint = 290.0;
    double temp = setpoint + FLOOR_ERROR_THRESHOLD_FAR_C + 5.0;

    double floor = calc.computeFloor(temp, setpoint, reason);
    // Way above setpoint: no floor applies
    EXPECT_EQ(reason, FloorReason::None);
}

// --- RateLimiter ---

TEST(RateLimiter, NoLimitBelowThreshold) {
    RateLimiter limiter(testRateLimiterConfig());
    bool active = false;
    double duty = limiter.applyLimit(200.0, 1.0, active);
    EXPECT_FALSE(active);
    EXPECT_DOUBLE_EQ(duty, 200.0);
}

TEST(RateLimiter, ReducesOutputAboveThreshold) {
    RateLimiter limiter(testRateLimiterConfig());
    bool active = false;
    double duty = limiter.applyLimit(200.0, RATE_LIMIT_THRESHOLD_CPS + 1.0, active);
    EXPECT_TRUE(active);
    EXPECT_LT(duty, 200.0);
}

TEST(RateLimiter, MaxReductionAtMaxRate) {
    RateLimiter limiter(testRateLimiterConfig());
    bool active = false;
    double duty = limiter.applyLimit(200.0, RATE_LIMIT_MAX_CPS + 10.0, active);
    EXPECT_TRUE(active);
    double expected = 200.0 * (1.0 - RATE_LIMIT_OUTPUT_REDUCTION);
    EXPECT_NEAR(duty, expected, 1.0);
}

// --- HeaterChannel integration ---

TEST(HeaterChannel, UpdateWithValidAdcProducesDuty) {
    HeaterChannel ch(0, 34, 25, DEFAULT_CALIBRATION_OFFSET_C, testHeaterChannelConfig());
    ch.setSetpointC(290.0);

    auto state = ch.update(1500, 1000, 0.02);
    EXPECT_TRUE(std::isfinite(state.temperatureC));
    // With valid temperature, we should get some duty output
    EXPECT_GE(state.currentDuty, 0.0);
}

TEST(HeaterChannel, UpdateWithZeroAdcReturnsZeroDuty) {
    HeaterChannel ch(0, 34, 25, 0.0, testHeaterChannelConfig());
    ch.setSetpointC(290.0);

    auto state = ch.update(0, 1000, 0.02);
    // Disconnected sensor -> zero duty
    EXPECT_DOUBLE_EQ(state.currentDuty, 0.0);
    EXPECT_FALSE(ch.isSensorConnected());
}

TEST(HeaterChannel, ResetControlClearsDuty) {
    HeaterChannel ch(0, 34, 25, DEFAULT_CALIBRATION_OFFSET_C, testHeaterChannelConfig());
    ch.setSetpointC(290.0);

    ch.update(1500, 1000, 0.02);
    ch.resetControl();

    auto& state = ch.getState();
    EXPECT_DOUBLE_EQ(state.currentDuty, 0.0);
}

TEST(HeaterChannel, SetpointTrimAddedToSetpoint) {
    HeaterChannel ch(0, 0, 0, 0.0, testHeaterChannelConfig());
    ch.setSetpointC(290.0);
    ch.setSetpointTrimC(5.0);
    EXPECT_DOUBLE_EQ(ch.getEffectiveSetpointC(), 295.0);
}
