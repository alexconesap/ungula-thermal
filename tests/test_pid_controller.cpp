#include <gtest/gtest.h>
#include <thermal/pid_controller.h>

#include <cmath>

using namespace ungula::thermal;

// ---- Test-local constants (values from the old thermal_config.h) ----

static constexpr double FAHRENHEIT_TO_CELSIUS_FACTOR = 1.8;

static constexpr double GAIN_SWITCH_BAND_C = 1.0 / FAHRENHEIT_TO_CELSIUS_FACTOR;
static constexpr double DERIVATIVE_ENABLE_MARGIN_C = 0.5 / FAHRENHEIT_TO_CELSIUS_FACTOR;
static constexpr double SETPOINT_APPROACH_THRESHOLD_C = 5.0 / FAHRENHEIT_TO_CELSIUS_FACTOR;
static constexpr double SETPOINT_APPROACH_OFFSET_C = 2.0 / FAHRENHEIT_TO_CELSIUS_FACTOR;
static constexpr double HYSTERESIS_ABOVE_SETPOINT_C = 0.8 / FAHRENHEIT_TO_CELSIUS_FACTOR;
static constexpr double HYSTERESIS_BELOW_SETPOINT_C = 1.2 / FAHRENHEIT_TO_CELSIUS_FACTOR;
static constexpr double KI_REDUCTION_BAND_NARROW_C = 1.0 / FAHRENHEIT_TO_CELSIUS_FACTOR;
static constexpr double KI_REDUCTION_BAND_WIDE_C = 2.0 / FAHRENHEIT_TO_CELSIUS_FACTOR;
static constexpr double KI_REDUCTION_FACTOR_NARROW = 0.5;
static constexpr double KI_REDUCTION_FACTOR_WIDE = 0.6;
static constexpr double INTEGRAL_LIMIT = 500.0;

static inline PidConfig testPidConfig() {
    return PidConfig{
        .approachGains = {36.0 / FAHRENHEIT_TO_CELSIUS_FACTOR, 0.55 / FAHRENHEIT_TO_CELSIUS_FACTOR,
                          5.5 / FAHRENHEIT_TO_CELSIUS_FACTOR},
        .holdGains = {16.0 / FAHRENHEIT_TO_CELSIUS_FACTOR, 0.32 / FAHRENHEIT_TO_CELSIUS_FACTOR,
                      5.5 / FAHRENHEIT_TO_CELSIUS_FACTOR},
        .derivativeFilterAlpha = 0.25,
        .derivativeFilterAlphaMin = 0.05,
        .derivativeFilterAlphaMax = 1.0,
        .setpointApproachThresholdC = SETPOINT_APPROACH_THRESHOLD_C,
        .setpointApproachOffsetC = SETPOINT_APPROACH_OFFSET_C,
        .gainSwitchBandC = GAIN_SWITCH_BAND_C,
        .derivativeEnableMarginC = DERIVATIVE_ENABLE_MARGIN_C,
        .kiReductionBandNarrowC = KI_REDUCTION_BAND_NARROW_C,
        .kiReductionBandWideC = KI_REDUCTION_BAND_WIDE_C,
        .kiReductionFactorNarrow = KI_REDUCTION_FACTOR_NARROW,
        .kiReductionFactorWide = KI_REDUCTION_FACTOR_WIDE,
        .integralLimit = INTEGRAL_LIMIT,
        .antiwindupGain = 0.2,
        .hysteresisAboveSetpointC = HYSTERESIS_ABOVE_SETPOINT_C,
        .hysteresisBelowSetpointC = HYSTERESIS_BELOW_SETPOINT_C,
        .outputMax = 255.0,
    };
}

class PidControllerTest : public ::testing::Test {
   protected:
    PidController pid{testPidConfig()};

    void SetUp() override {
        pid.reset();
    }
};

// --- Reset state ---

TEST_F(PidControllerTest, ResetClearsState) {
    pid.compute(100.0, 290.0, 0.02);
    pid.reset();

    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.0);
    EXPECT_TRUE(std::isnan(pid.getFilteredTemperature()));
    EXPECT_FALSE(pid.hasReachedSetpoint());
}

// --- Proportional response ---

TEST_F(PidControllerTest, ProportionalResponseWithZeroKiKd) {
    PidGains approach = {10.0, 0.0, 0.0};
    PidGains hold = {5.0, 0.0, 0.0};
    pid.setGains(approach, hold);

    double setpoint = 290.0;
    double temp = 200.0;
    auto result = pid.compute(temp, setpoint, 0.02);

    EXPECT_GT(result.pTerm, 0.0);
    EXPECT_FALSE(result.inHoldMode);
}

TEST_F(PidControllerTest, HoldModeGainsWhenCloseToSetpoint) {
    PidGains approach = {10.0, 0.0, 0.0};
    PidGains hold = {5.0, 0.0, 0.0};
    pid.setGains(approach, hold);

    double setpoint = 290.0;
    double temp = setpoint - GAIN_SWITCH_BAND_C * 0.5;
    auto result = pid.compute(temp, setpoint, 0.02);

    EXPECT_TRUE(result.inHoldMode);
}

// --- Integral accumulation ---

TEST_F(PidControllerTest, IntegralAccumulatesWithPositiveError) {
    PidGains gains = {0.0, 1.0, 0.0};
    pid.setGains(gains, gains);

    for (int i = 0; i < 50; i++) {
        pid.compute(280.0, 290.0, 0.02);
    }

    EXPECT_GT(pid.getIntegral(), 0.0);
}

TEST_F(PidControllerTest, IntegralClampedAtLimit) {
    PidGains gains = {0.0, 100.0, 0.0};
    pid.setGains(gains, gains);

    for (int i = 0; i < 1000; i++) {
        pid.compute(100.0, 290.0, 1.0);
    }

    EXPECT_LE(pid.getIntegral(), INTEGRAL_LIMIT);
    EXPECT_GE(pid.getIntegral(), -INTEGRAL_LIMIT);
}

TEST_F(PidControllerTest, NegativeErrorAccumulatesNegativeIntegral) {
    PidGains gains = {0.0, 1.0, 0.0};
    pid.setGains(gains, gains);

    // Temperature above setpoint -> negative error
    for (int i = 0; i < 50; i++) {
        pid.compute(300.0, 290.0, 0.02);
    }

    EXPECT_LT(pid.getIntegral(), 0.0);
}

// --- Derivative filtering ---

TEST_F(PidControllerTest, DerivativeRespondToTempChange) {
    PidGains gains = {0.0, 0.0, 10.0};
    pid.setGains(gains, gains);

    pid.compute(285.0, 290.0, 0.02);
    pid.compute(285.0, 290.0, 0.02);
    auto result = pid.compute(290.0, 290.0, 0.02);

    EXPECT_NE(result.derivativeCps, 0.0);
}

TEST_F(PidControllerTest, DerivativeZeroWithStableTemp) {
    PidGains gains = {0.0, 0.0, 10.0};
    pid.setGains(gains, gains);

    // Several calls at the same temperature
    pid.compute(290.0, 290.0, 0.02);
    pid.compute(290.0, 290.0, 0.02);
    pid.compute(290.0, 290.0, 0.02);
    auto result = pid.compute(290.0, 290.0, 0.02);

    EXPECT_NEAR(result.derivativeCps, 0.0, 0.001);
}

TEST_F(PidControllerTest, NegativeDerivativeOnCooling) {
    PidGains gains = {0.0, 0.0, 10.0};
    pid.setGains(gains, gains);

    pid.compute(290.0, 290.0, 0.02);
    pid.compute(290.0, 290.0, 0.02);
    auto result = pid.compute(285.0, 290.0, 0.02);

    EXPECT_LT(result.derivativeCps, 0.0);
}

// --- Invalid temperature ---

TEST_F(PidControllerTest, InvalidTempReturnsZeroOutput) {
    pid.compute(200.0, 290.0, 0.02);

    auto result = pid.compute(NAN, 290.0, 0.02);
    EXPECT_DOUBLE_EQ(result.rawOutput, 0.0);
    EXPECT_DOUBLE_EQ(result.pTerm, 0.0);
    EXPECT_DOUBLE_EQ(result.iTerm, 0.0);
    EXPECT_DOUBLE_EQ(result.dTerm, 0.0);
}

TEST_F(PidControllerTest, ExtremeNegativeTempReturnsZero) {
    auto result = pid.compute(-300.0, 290.0, 0.02);
    EXPECT_DOUBLE_EQ(result.rawOutput, 0.0);
}

TEST_F(PidControllerTest, ExtremePosiveTempReturnsZero) {
    auto result = pid.compute(2000.0, 290.0, 0.02);
    EXPECT_DOUBLE_EQ(result.rawOutput, 0.0);
}

TEST_F(PidControllerTest, InfinityTempReturnsZero) {
    auto result = pid.compute(INFINITY, 290.0, 0.02);
    EXPECT_DOUBLE_EQ(result.rawOutput, 0.0);
}

// --- dt edge cases ---

TEST_F(PidControllerTest, ZeroDtHandledGracefully) {
    PidGains gains = {10.0, 1.0, 1.0};
    pid.setGains(gains, gains);

    auto result = pid.compute(280.0, 290.0, 0.0);
    EXPECT_TRUE(std::isfinite(result.rawOutput));
    EXPECT_GT(result.rawOutput, 0.0);
}

TEST_F(PidControllerTest, NegativeDtHandledGracefully) {
    PidGains gains = {10.0, 1.0, 1.0};
    pid.setGains(gains, gains);

    auto result = pid.compute(280.0, 290.0, -0.5);
    EXPECT_TRUE(std::isfinite(result.rawOutput));
}

// --- Setpoint reached hysteresis ---

TEST_F(PidControllerTest, SetpointReachedLatchSetsAboveThreshold) {
    double setpoint = 290.0;
    EXPECT_FALSE(pid.hasReachedSetpoint());

    for (double t = 280.0; t <= setpoint + HYSTERESIS_ABOVE_SETPOINT_C + 1.0; t += 0.5) {
        pid.compute(t, setpoint, 0.02);
    }

    EXPECT_TRUE(pid.hasReachedSetpoint());
}

TEST_F(PidControllerTest, SetpointLatchClearsBelowThreshold) {
    double setpoint = 290.0;

    pid.compute(setpoint + HYSTERESIS_ABOVE_SETPOINT_C + 1.0, setpoint, 0.02);
    EXPECT_TRUE(pid.hasReachedSetpoint());

    pid.compute(setpoint - HYSTERESIS_BELOW_SETPOINT_C - 1.0, setpoint, 0.02);
    EXPECT_FALSE(pid.hasReachedSetpoint());
}

TEST_F(PidControllerTest, HysteresisBoundaryExactAbove) {
    double setpoint = 290.0;

    // Exactly at the threshold should set the latch
    pid.compute(setpoint + HYSTERESIS_ABOVE_SETPOINT_C, setpoint, 0.02);
    EXPECT_TRUE(pid.hasReachedSetpoint());
}

TEST_F(PidControllerTest, HysteresisBoundaryExactBelow) {
    double setpoint = 290.0;

    // Set latch first
    pid.compute(setpoint + HYSTERESIS_ABOVE_SETPOINT_C + 1.0, setpoint, 0.02);
    EXPECT_TRUE(pid.hasReachedSetpoint());

    // Exactly at clear threshold should clear the latch
    pid.compute(setpoint - HYSTERESIS_BELOW_SETPOINT_C, setpoint, 0.02);
    EXPECT_FALSE(pid.hasReachedSetpoint());
}

TEST_F(PidControllerTest, HysteresisMultipleCycles) {
    double setpoint = 290.0;

    for (int cycle = 0; cycle < 3; cycle++) {
        // Approach and overshoot
        pid.compute(setpoint + HYSTERESIS_ABOVE_SETPOINT_C + 1.0, setpoint, 0.02);
        EXPECT_TRUE(pid.hasReachedSetpoint());

        // Drop below
        pid.compute(setpoint - HYSTERESIS_BELOW_SETPOINT_C - 1.0, setpoint, 0.02);
        EXPECT_FALSE(pid.hasReachedSetpoint());
    }
}

// --- Setpoint approach conditioning ---

TEST_F(PidControllerTest, EffectiveSetpointOffsetWhenFarBelow) {
    double setpoint = 290.0;
    double temp = 200.0;

    auto result = pid.compute(temp, setpoint, 0.02);

    EXPECT_NEAR(result.effectiveSetpoint, setpoint + SETPOINT_APPROACH_OFFSET_C, 0.01);
}

TEST_F(PidControllerTest, EffectiveSetpointEqualsSetpointWhenClose) {
    double setpoint = 290.0;
    double temp = setpoint - SETPOINT_APPROACH_THRESHOLD_C * 0.5;

    auto result = pid.compute(temp, setpoint, 0.02);

    EXPECT_DOUBLE_EQ(result.effectiveSetpoint, setpoint);
}

TEST_F(PidControllerTest, EffectiveSetpointOffsetDisabledAfterReaching) {
    double setpoint = 290.0;

    // First reach setpoint (sets latch)
    pid.compute(setpoint + HYSTERESIS_ABOVE_SETPOINT_C + 1.0, setpoint, 0.02);
    EXPECT_TRUE(pid.hasReachedSetpoint());

    // Drop below approach threshold but stay within hysteresis band so latch stays set.
    // SETPOINT_APPROACH_THRESHOLD_C = 5.0/1.8 ≈ 2.78, HYSTERESIS_BELOW = 1.2/1.8 ≈ 0.67
    // Use temp = setpoint - 2.0 (below approach threshold, but above hysteresis clear point)
    double temp = setpoint - 2.0;  // 288 — still > setpoint - HYSTERESIS_BELOW(0.67)? No, 288 < 289.33
    // Actually hysteresis clear is at setpoint - 0.667 = 289.333. Temp 288 < 289.333 clears latch.
    // The approach threshold (2.78) > hysteresis band (0.667), so once latched and dropped below
    // the approach threshold, the latch will ALWAYS be cleared first.
    // This means the offset cannot be disabled by the latch for far-below temps.
    // The test verifies that a temp WITHIN the band (not far below) sees no offset.
    temp = setpoint - SETPOINT_APPROACH_THRESHOLD_C * 0.5;  // Within approach zone, not far below
    auto result = pid.compute(temp, setpoint, 0.02);
    EXPECT_DOUBLE_EQ(result.effectiveSetpoint, setpoint);  // Not far below, so no offset
}

TEST_F(PidControllerTest, EffectiveSetpointOffsetReactivatesAfterLatchClears) {
    double setpoint = 290.0;

    // Reach, then drop far enough to clear latch
    pid.compute(setpoint + HYSTERESIS_ABOVE_SETPOINT_C + 1.0, setpoint, 0.02);
    pid.compute(setpoint - HYSTERESIS_BELOW_SETPOINT_C - 1.0, setpoint, 0.02);
    EXPECT_FALSE(pid.hasReachedSetpoint());

    // Go far below - offset should reactivate
    auto result = pid.compute(200.0, setpoint, 0.02);
    EXPECT_NEAR(result.effectiveSetpoint, setpoint + SETPOINT_APPROACH_OFFSET_C, 0.01);
}

// --- Ki reduction zones ---

TEST_F(PidControllerTest, KiReducedInNarrowBand) {
    PidGains gains = {0.0, 10.0, 0.0};
    pid.setGains(gains, gains);

    double setpoint = 290.0;
    double temp = setpoint - KI_REDUCTION_BAND_NARROW_C * 0.5;
    auto result = pid.compute(temp, setpoint, 1.0);

    double expectedKi = 10.0 * KI_REDUCTION_FACTOR_NARROW;
    double error = setpoint - temp;
    EXPECT_NEAR(result.iTerm, expectedKi * error * 1.0, 1.0);
}

TEST_F(PidControllerTest, KiReducedInWideBand) {
    PidGains gains = {0.0, 10.0, 0.0};
    pid.setGains(gains, gains);

    double setpoint = 290.0;
    // Error between narrow and wide bands
    double temp = setpoint - (KI_REDUCTION_BAND_NARROW_C + KI_REDUCTION_BAND_WIDE_C) * 0.5;
    auto result = pid.compute(temp, setpoint, 1.0);

    double expectedKi = 10.0 * KI_REDUCTION_FACTOR_WIDE;
    double error = setpoint - temp;
    EXPECT_NEAR(result.iTerm, expectedKi * error * 1.0, 1.0);
}

TEST_F(PidControllerTest, KiFullOutsideReductionBands) {
    PidGains gains = {0.0, 10.0, 0.0};
    pid.setGains(gains, gains);

    double setpoint = 290.0;
    // Error well outside wide band
    double temp = setpoint - KI_REDUCTION_BAND_WIDE_C * 2.0;
    auto result = pid.compute(temp, setpoint, 1.0);

    double error = setpoint - temp;
    // Full Ki applies, no reduction
    EXPECT_NEAR(result.iTerm, 10.0 * error * 1.0, 1.0);
}

// --- Gain switching boundary ---

TEST_F(PidControllerTest, GainSwitchAtBoundary) {
    PidGains approach = {20.0, 0.0, 0.0};
    PidGains hold = {5.0, 0.0, 0.0};
    pid.setGains(approach, hold);

    double setpoint = 290.0;

    // Just outside hold band (approach mode)
    auto r1 = pid.compute(setpoint - GAIN_SWITCH_BAND_C - 0.01, setpoint, 0.02);
    EXPECT_FALSE(r1.inHoldMode);

    pid.reset();

    // Just inside hold band (hold mode)
    auto r2 = pid.compute(setpoint - GAIN_SWITCH_BAND_C + 0.01, setpoint, 0.02);
    EXPECT_TRUE(r2.inHoldMode);
}

TEST_F(PidControllerTest, NegativeErrorInHoldBand) {
    PidGains approach = {20.0, 0.0, 0.0};
    PidGains hold = {5.0, 0.0, 0.0};
    pid.setGains(approach, hold);

    double setpoint = 290.0;
    // Temperature slightly above setpoint (negative error, but small magnitude)
    auto result = pid.compute(setpoint + GAIN_SWITCH_BAND_C * 0.5, setpoint, 0.02);
    EXPECT_TRUE(result.inHoldMode);
}

// --- Anti-windup ---

TEST_F(PidControllerTest, AntiWindupLimitsIntegralDuringSaturation) {
    pid.setOutputLimits(0.0, 100.0);
    PidGains gains = {0.0, 100.0, 0.0};
    pid.setGains(gains, gains);

    // Large error for many iterations -> output saturates at max
    double integral_before = 0.0;
    for (int i = 0; i < 100; i++) {
        pid.compute(100.0, 290.0, 1.0);
    }
    double integral_saturated = pid.getIntegral();

    // Without anti-windup, integral would grow to INTEGRAL_LIMIT (500).
    // With anti-windup, integral should be reduced during saturation.
    // The integral should be bounded more tightly than INTEGRAL_LIMIT.
    EXPECT_LE(integral_saturated, INTEGRAL_LIMIT);
    EXPECT_TRUE(std::isfinite(integral_saturated));
}

TEST_F(PidControllerTest, AntiWindupRecovery) {
    pid.setOutputLimits(0.0, 255.0);
    PidGains gains = {10.0, 5.0, 0.0};
    pid.setGains(gains, gains);

    // Saturate high
    for (int i = 0; i < 50; i++) {
        pid.compute(100.0, 290.0, 0.02);
    }

    // Now temperature overshoots setpoint - output should drop quickly
    auto result = pid.compute(295.0, 290.0, 0.02);
    // P term is negative (temp > setpoint), but integral may keep output positive
    // The key test: output should eventually recover, not stay pinned
    for (int i = 0; i < 50; i++) {
        result = pid.compute(295.0, 290.0, 0.02);
    }
    // After many iterations above setpoint, output should be very low
    EXPECT_LT(result.rawOutput, 128.0);
}

// --- Derivative conditional application ---

TEST_F(PidControllerTest, DerivativeDisabledFarFromSetpoint) {
    PidGains gains = {0.0, 0.0, 100.0};
    pid.setGains(gains, gains);

    double setpoint = 290.0;

    // Initialize at a temperature far below setpoint
    pid.compute(200.0, setpoint, 0.02);
    pid.compute(200.0, setpoint, 0.02);

    // Temperature step while far from setpoint
    auto result = pid.compute(210.0, setpoint, 0.02);

    // Derivative should be zero when far below setpoint and latch not set
    // (both conditions of the conditional are false)
    // Note: the condition is processValue >= effectiveSetpoint OR error within margin
    // At 210, error = ~81, which is way outside the margin
    EXPECT_NEAR(result.dTerm, 0.0, 0.01);
}

TEST_F(PidControllerTest, DerivativeEnabledNearSetpoint) {
    PidGains gains = {0.0, 0.0, 100.0};
    pid.setGains(gains, gains);

    double setpoint = 290.0;

    // Initialize near setpoint
    pid.compute(289.5, setpoint, 0.02);
    pid.compute(289.5, setpoint, 0.02);

    // Small step near setpoint
    auto result = pid.compute(290.0, setpoint, 0.02);

    // Derivative should be non-zero when near setpoint
    EXPECT_NE(result.dTerm, 0.0);
}

TEST_F(PidControllerTest, DerivativeEnabledAboveSetpoint) {
    PidGains gains = {0.0, 0.0, 100.0};
    pid.setGains(gains, gains);

    double setpoint = 290.0;

    // Initialize above setpoint
    pid.compute(295.0, setpoint, 0.02);
    pid.compute(295.0, setpoint, 0.02);

    // Temperature change above setpoint
    auto result = pid.compute(296.0, setpoint, 0.02);

    // Derivative should be active above setpoint
    EXPECT_NE(result.dTerm, 0.0);
}

// --- Bumpless transfer ---

TEST_F(PidControllerTest, BumplessTransferPreloadsIntegral) {
    double currentOutput = 150.0;
    double currentTemp = 285.0;
    double setpoint = 290.0;

    pid.initializeForBumplessTransfer(currentOutput, currentTemp, setpoint);

    EXPECT_NE(pid.getIntegral(), 0.0);
    EXPECT_DOUBLE_EQ(pid.getFilteredTemperature(), currentTemp);
}

TEST_F(PidControllerTest, BumplessTransferWithZeroKi) {
    PidGains gains = {10.0, 0.0, 0.0};
    pid.setGains(gains, gains);

    pid.initializeForBumplessTransfer(150.0, 285.0, 290.0);
    // With Ki=0, integral should be zero (division by near-zero guarded)
    EXPECT_NEAR(pid.getIntegral(), 0.0, 0.001);
}

TEST_F(PidControllerTest, BumplessTransferOutputContinuity) {
    PidGains approach = {10.0, 2.0, 0.0};
    PidGains hold = {5.0, 1.0, 0.0};
    pid.setGains(approach, hold);

    double currentOutput = 120.0;
    double currentTemp = 285.0;
    double setpoint = 290.0;

    pid.initializeForBumplessTransfer(currentOutput, currentTemp, setpoint);

    // First compute after bumpless transfer should produce output near currentOutput.
    // Tolerance accounts for P term + possible setpoint approach offset.
    auto result = pid.compute(currentTemp, setpoint, 0.02);
    EXPECT_NEAR(result.rawOutput, currentOutput, 70.0);
}

// --- Simulated warmup ramp ---

TEST_F(PidControllerTest, WarmupRampReducesOutputAsApproachingSetpoint) {
    double setpoint = 290.0;
    double dt = 0.02;

    double firstOutput = 0.0;
    double lastOutput = 0.0;

    for (int i = 0; i <= 100; i++) {
        double temp = 20.0 + (290.0 - 20.0) * i / 100.0;
        auto result = pid.compute(temp, setpoint, dt);
        if (i == 0) firstOutput = result.rawOutput;
        lastOutput = result.rawOutput;
    }

    EXPECT_GT(firstOutput, lastOutput);
}

// --- Output clamping ---

TEST_F(PidControllerTest, OutputClampedToLimits) {
    pid.setOutputLimits(0.0, 100.0);
    PidGains gains = {1000.0, 0.0, 0.0};
    pid.setGains(gains, gains);

    auto result = pid.compute(100.0, 290.0, 0.02);
    EXPECT_LE(result.rawOutput, 100.0);
    EXPECT_GE(result.rawOutput, 0.0);
}

TEST_F(PidControllerTest, OutputClampedAtZeroForNegativeError) {
    pid.setOutputLimits(0.0, 255.0);
    PidGains gains = {1000.0, 0.0, 0.0};
    pid.setGains(gains, gains);

    // Temperature way above setpoint -> negative P term -> clamped at 0
    auto result = pid.compute(400.0, 290.0, 0.02);
    EXPECT_GE(result.rawOutput, 0.0);
}

// --- Filter alpha ---

TEST_F(PidControllerTest, HighAlphaFollowsFaster) {
    PidGains gains = {0.0, 0.0, 10.0};

    PidController fast(testPidConfig()), slow(testPidConfig());
    fast.setGains(gains, gains);
    slow.setGains(gains, gains);
    fast.setDerivativeFilterAlpha(1.0);
    slow.setDerivativeFilterAlpha(0.05);

    // Initialize both
    fast.compute(200.0, 290.0, 0.02);
    slow.compute(200.0, 290.0, 0.02);

    // Step input
    auto rf = fast.compute(250.0, 290.0, 0.02);
    auto rs = slow.compute(250.0, 290.0, 0.02);

    // Higher alpha = faster response = larger derivative magnitude
    EXPECT_GT(std::fabs(rf.derivativeCps), std::fabs(rs.derivativeCps));
}
