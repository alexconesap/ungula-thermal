// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa

#include <gtest/gtest.h>
#include <ungula/thermal/pid_controller.h>

#include <cmath>
#include <vector>

using namespace ungula::thermal;

// ============================================================================
// Production config values (identical to rbb2_config.h)
// ============================================================================

static constexpr double F2C = 1.8;

static PidConfig productionConfig()
{
    return PidConfig{
        .approachGains = { 36.0 / F2C, 0.55 / F2C, 5.5 / F2C },
        .holdGains = { 16.0 / F2C, 0.32 / F2C, 5.5 / F2C },
        .derivativeFilterAlpha = 0.25,
        .derivativeFilterAlphaMin = 0.05,
        .derivativeFilterAlphaMax = 1.0,
        .setpointApproachThresholdC = 5.0 / F2C,
        .setpointApproachOffsetC = 2.0 / F2C,
        .gainSwitchBandC = 1.0 / F2C,
        .derivativeEnableMarginC = 0.5 / F2C,
        .kiReductionBandNarrowC = 1.0 / F2C,
        .kiReductionBandWideC = 2.0 / F2C,
        .kiReductionFactorNarrow = 0.5,
        .kiReductionFactorWide = 0.6,
        .integralLimit = 500.0,
        .antiwindupGain = 0.2,
        .hysteresisAboveSetpointC = 0.8 / F2C,
        .hysteresisBelowSetpointC = 1.2 / F2C,
        .outputMax = 255.0,
    };
}

// ============================================================================
// Section 1: Config injection verification
// ============================================================================

class PidConfigInjection : public ::testing::Test {
protected:
    PidController pid{ productionConfig() };
};

TEST_F(PidConfigInjection, ApproachGainsAreUsedFarFromSetpoint)
{
    // P-only: set Ki=Kd=0 to isolate Kp
    PidGains approach = { 20.0, 0.0, 0.0 };
    PidGains hold = { 5.0, 0.0, 0.0 };
    pid.setGains(approach, hold);

    double sp = 300.0;
    double temp = 200.0; // far from setpoint -> approach mode
    auto r = pid.compute(temp, sp, 0.02);

    // Error = effectiveSetpoint - temp. Far below = approach offset applied.
    double effectiveSp = sp + productionConfig().setpointApproachOffsetC;
    double expectedP = 20.0 * (effectiveSp - temp);
    double expectedOutput = std::min(expectedP, 255.0);

    EXPECT_FALSE(r.inHoldMode);
    EXPECT_NEAR(r.pTerm, expectedP, 0.01);
    EXPECT_NEAR(r.rawOutput, expectedOutput, 0.01);
}

TEST_F(PidConfigInjection, HoldGainsAreUsedCloseToSetpoint)
{
    PidGains approach = { 20.0, 0.0, 0.0 };
    PidGains hold = { 5.0, 0.0, 0.0 };
    pid.setGains(approach, hold);

    double sp = 300.0;
    double temp = sp - 0.2; // within gainSwitchBandC (~0.556)
    auto r = pid.compute(temp, sp, 0.02);

    EXPECT_TRUE(r.inHoldMode);
    EXPECT_NEAR(r.pTerm, 5.0 * (sp - temp), 0.01);
}

TEST_F(PidConfigInjection, OutputMaxFromConfigIsRespected)
{
    PidGains gains = { 1000.0, 0.0, 0.0 };
    pid.setGains(gains, gains);

    auto r = pid.compute(100.0, 300.0, 0.02);
    EXPECT_DOUBLE_EQ(r.rawOutput, 255.0);
}

TEST_F(PidConfigInjection, IntegralLimitFromConfigIsRespected)
{
    PidGains gains = { 0.0, 100.0, 0.0 };
    pid.setGains(gains, gains);

    for (int i = 0; i < 500; i++) {
        pid.compute(100.0, 300.0, 1.0);
    }

    EXPECT_LE(pid.getIntegral(), productionConfig().integralLimit);
    EXPECT_GE(pid.getIntegral(), -productionConfig().integralLimit);
}

TEST_F(PidConfigInjection, DerivativeAlphaClampedByConfig)
{
    pid.setDerivativeFilterAlpha(999.0);
    EXPECT_DOUBLE_EQ(pid.getDerivativeFilterAlpha(), productionConfig().derivativeFilterAlphaMax);

    pid.setDerivativeFilterAlpha(-1.0);
    EXPECT_DOUBLE_EQ(pid.getDerivativeFilterAlpha(), productionConfig().derivativeFilterAlphaMin);
}

TEST_F(PidConfigInjection, HysteresisThresholdsFromConfig)
{
    double sp = 300.0;
    auto cfg = productionConfig();

    // Just below upper hysteresis: latch not set
    pid.compute(sp + cfg.hysteresisAboveSetpointC - 0.01, sp, 0.02);
    EXPECT_FALSE(pid.hasReachedSetpoint());

    // At upper hysteresis: latch set
    pid.compute(sp + cfg.hysteresisAboveSetpointC, sp, 0.02);
    EXPECT_TRUE(pid.hasReachedSetpoint());

    // Inside band between hysteresis points: latch stays
    pid.compute(sp, sp, 0.02);
    EXPECT_TRUE(pid.hasReachedSetpoint());

    // At lower hysteresis: latch clears
    pid.compute(sp - cfg.hysteresisBelowSetpointC, sp, 0.02);
    EXPECT_FALSE(pid.hasReachedSetpoint());
}

TEST_F(PidConfigInjection, SetpointApproachOffsetFromConfig)
{
    auto cfg = productionConfig();
    double sp = 300.0;
    double farTemp = sp - cfg.setpointApproachThresholdC - 10.0;

    auto r = pid.compute(farTemp, sp, 0.02);
    EXPECT_NEAR(r.effectiveSetpoint, sp + cfg.setpointApproachOffsetC, 0.001);
}

TEST_F(PidConfigInjection, KiReductionBandsFromConfig)
{
    auto cfg = productionConfig();
    PidGains gains = { 0.0, 10.0, 0.0 };
    pid.setGains(gains, gains);

    double sp = 300.0;

    // Narrow band: error < kiReductionBandNarrowC
    double tempNarrow = sp - cfg.kiReductionBandNarrowC * 0.3;
    auto rn = pid.compute(tempNarrow, sp, 1.0);
    double errorN = sp - tempNarrow;
    double expectedNarrowKi = 10.0 * cfg.kiReductionFactorNarrow;
    EXPECT_NEAR(rn.iTerm, expectedNarrowKi * errorN, 0.5);

    pid.reset();

    // Wide band: narrow < error < wide
    double tempWide = sp - (cfg.kiReductionBandNarrowC + cfg.kiReductionBandWideC) * 0.5;
    auto rw = pid.compute(tempWide, sp, 1.0);
    double errorW = sp - tempWide;
    // setpointApproachOffset may kick in: check effective setpoint
    double expectedWideKi = 10.0 * cfg.kiReductionFactorWide;
    EXPECT_NEAR(rw.iTerm, expectedWideKi * (rw.effectiveSetpoint - tempWide), 0.5);
}

// ============================================================================
// Section 2: Deterministic P/I/D term verification
// ============================================================================

// Use a simple config where we can compute expected values by hand
static PidConfig simpleConfig()
{
    return PidConfig{
        .approachGains = { 10.0, 0.5, 2.0 },
        .holdGains = { 5.0, 0.25, 2.0 },
        .derivativeFilterAlpha = 1.0, // no filtering, instant response
        .derivativeFilterAlphaMin = 0.05,
        .derivativeFilterAlphaMax = 1.0,
        .setpointApproachThresholdC = 5.0,
        .setpointApproachOffsetC = 2.0,
        .gainSwitchBandC = 1.0,
        .derivativeEnableMarginC = 0.5,
        .kiReductionBandNarrowC = 1.0,
        .kiReductionBandWideC = 2.0,
        .kiReductionFactorNarrow = 0.5,
        .kiReductionFactorWide = 0.6,
        .integralLimit = 500.0,
        .antiwindupGain = 0.2,
        .hysteresisAboveSetpointC = 1.0,
        .hysteresisBelowSetpointC = 1.0,
        .outputMax = 255.0,
    };
}

TEST(PidDeterministic, PTermOnlyFirstCall)
{
    auto cfg = simpleConfig();
    cfg.approachGains = { 10.0, 0.0, 0.0 };
    cfg.holdGains = { 5.0, 0.0, 0.0 };
    PidController pid(cfg);

    // First call: temp far below setpoint -> approach mode
    // effectiveSetpoint = 100 + 2 = 102 (approach offset applied)
    // error = 102 - 50 = 52
    // P = 10 * 52 = 520 -> clamped to 255
    auto r = pid.compute(50.0, 100.0, 0.02);
    EXPECT_DOUBLE_EQ(r.effectiveSetpoint, 102.0);
    EXPECT_NEAR(r.pTerm, 520.0, 0.01);
    EXPECT_DOUBLE_EQ(r.rawOutput, 255.0);
}

TEST(PidDeterministic, PTermInHoldBand)
{
    auto cfg = simpleConfig();
    cfg.approachGains = { 10.0, 0.0, 0.0 };
    cfg.holdGains = { 5.0, 0.0, 0.0 };
    PidController pid(cfg);

    // temp close to setpoint: |error| < gainSwitchBandC (1.0)
    // effectiveSetpoint = 100 (within threshold, no offset)
    // error = 100 - 99.5 = 0.5
    // P = 5.0 * 0.5 = 2.5
    auto r = pid.compute(99.5, 100.0, 0.02);
    EXPECT_TRUE(r.inHoldMode);
    EXPECT_NEAR(r.pTerm, 2.5, 0.01);
    EXPECT_NEAR(r.rawOutput, 2.5, 0.01);
}

TEST(PidDeterministic, IntegralAccumulationExact)
{
    auto cfg = simpleConfig();
    cfg.approachGains = { 0.0, 1.0, 0.0 };
    cfg.holdGains = { 0.0, 1.0, 0.0 };
    cfg.setpointApproachThresholdC = 100.0; // push threshold far so no offset
    cfg.kiReductionBandNarrowC = 0.0; // disable Ki reduction
    cfg.kiReductionBandWideC = 0.0;
    PidController pid(cfg);

    double sp = 100.0;
    double temp = 90.0;
    double dt = 0.5;
    double error = sp - temp; // 10.0

    // After 1 call: integral = error * dt = 10 * 0.5 = 5
    // iTerm = Ki * integral = 1.0 * 5 = 5
    auto r1 = pid.compute(temp, sp, dt);
    EXPECT_NEAR(pid.getIntegral(), 5.0, 0.001);
    EXPECT_NEAR(r1.iTerm, 5.0, 0.01);

    // After 2nd call: integral = 5 + 10*0.5 = 10
    auto r2 = pid.compute(temp, sp, dt);
    EXPECT_NEAR(pid.getIntegral(), 10.0, 0.001);
    EXPECT_NEAR(r2.iTerm, 10.0, 0.01);
}

TEST(PidDeterministic, DerivativeTermExactWithAlpha1)
{
    // alpha=1.0 means filteredTemp = processValue (no filtering)
    auto cfg = simpleConfig();
    cfg.approachGains = { 0.0, 0.0, 10.0 };
    cfg.holdGains = { 0.0, 0.0, 10.0 };
    cfg.gainSwitchBandC = 100.0; // everything in hold band -> D always enabled
    cfg.derivativeEnableMarginC = 100.0;
    PidController pid(cfg);

    double sp = 100.0;
    double dt = 0.02;

    // First call: filteredTemp = 95, lastFilteredTemp = 95 -> dTemp = 0
    pid.compute(95.0, sp, dt);

    // Second call: filteredTemp = 96, dTemp = (96 - 95) / 0.02 = 50 C/s
    // D term = Kd * (-dTemp) = 10 * (-50) = -500 -> clamped to 0 (output min)
    auto r = pid.compute(96.0, sp, dt);
    EXPECT_NEAR(r.derivativeCps, 50.0, 0.01);
    EXPECT_NEAR(r.dTerm, -500.0, 0.1);
    EXPECT_DOUBLE_EQ(r.rawOutput, 0.0); // clamped at outputMin
}

TEST(PidDeterministic, DerivativeFilteringWithLowAlpha)
{
    auto cfg = simpleConfig();
    cfg.derivativeFilterAlpha = 0.1;
    cfg.approachGains = { 0.0, 0.0, 1.0 };
    cfg.holdGains = { 0.0, 0.0, 1.0 };
    cfg.gainSwitchBandC = 100.0;
    cfg.derivativeEnableMarginC = 100.0;
    PidController pid(cfg);

    double sp = 100.0;
    double dt = 0.02;

    // Initialize: filteredTemp = 95
    pid.compute(95.0, sp, dt);

    // Step to 100: filteredTemp = 95 + 0.1*(100 - 95) = 95.5
    // dTemp = (95.5 - 95) / 0.02 = 25
    auto r = pid.compute(100.0, sp, dt);
    EXPECT_NEAR(r.derivativeCps, 25.0, 0.01);

    // Compare: with alpha=1.0, dTemp would be (100-95)/0.02 = 250
    // So filtering reduced derivative by 10x as expected
}

// ============================================================================
// Section 3: Simulated temperature trajectories (array-driven)
// ============================================================================

class PidTrajectoryTest : public ::testing::Test {
protected:
    PidController pid{ productionConfig() };
    static constexpr double DT = 0.02; // 50Hz like production
    static constexpr double SP = 315.0; // ~600F, typical catheter reflow setpoint in Celsius

    struct TrajectoryPoint {
        double temperature;
        // Expected invariants checked per-point
    };
};

TEST_F(PidTrajectoryTest, HeatingRampFromColdToSetpoint)
{
    // Simulate a realistic heating ramp: 25C -> 315C over ~300 steps
    // (20ms per step = 6 seconds of simulated time)
    double temps[] = {
        25.0,  30.0,  38.0,  48.0,  60.0,  75.0,  92.0,  110.0, 130.0, 150.0, 168.0, 185.0, 200.0,
        215.0, 228.0, 240.0, 250.0, 258.0, 265.0, 272.0, 278.0, 283.0, 288.0, 292.0, 296.0, 299.0,
        301.5, 303.5, 305.0, 306.5, 308.0, 309.2, 310.3, 311.2, 312.0, 312.6, 313.1, 313.5, 313.9,
        314.2, 314.4, 314.6, 314.8, 315.0, 315.3, 315.5, 315.3, 315.0, 315.1, 315.0,
    };
    int n = sizeof(temps) / sizeof(temps[0]);

    double prevOutput = 0.0;
    bool sawApproachMode = false;
    bool sawHoldMode = false;
    bool latchSetAtSomePoint = false;

    for (int i = 0; i < n; i++) {
        auto r = pid.compute(temps[i], SP, DT);

        // Output must always be finite and within [0, outputMax]
        EXPECT_TRUE(std::isfinite(r.rawOutput)) << "Step " << i;
        EXPECT_GE(r.rawOutput, 0.0) << "Step " << i;
        EXPECT_LE(r.rawOutput, 255.0) << "Step " << i;

        // P term must be finite
        EXPECT_TRUE(std::isfinite(r.pTerm)) << "Step " << i;
        EXPECT_TRUE(std::isfinite(r.iTerm)) << "Step " << i;
        EXPECT_TRUE(std::isfinite(r.dTerm)) << "Step " << i;

        // Effective setpoint must be >= nominal setpoint
        EXPECT_GE(r.effectiveSetpoint, SP - 0.001) << "Step " << i;

        if (!r.inHoldMode)
            sawApproachMode = true;
        if (r.inHoldMode)
            sawHoldMode = true;
        if (pid.hasReachedSetpoint())
            latchSetAtSomePoint = true;

        prevOutput = r.rawOutput;
    }

    // Over this trajectory we must see both modes
    EXPECT_TRUE(sawApproachMode) << "Should see approach mode during early heating";
    EXPECT_TRUE(sawHoldMode) << "Should see hold mode near setpoint";

    // The latch should have been set once temp crossed setpoint + hysteresis
    EXPECT_TRUE(latchSetAtSomePoint) << "Should reach setpoint during ramp";
}

TEST_F(PidTrajectoryTest, OscillationAroundSetpoint)
{
    // Simulate temperature oscillating around setpoint with realistic thermal noise
    // This is the most important test: verifies PID behavior during steady-state
    double oscillation[] = {
        // First, approach setpoint
        300.0,
        305.0,
        310.0,
        313.0,
        314.5,
        315.0,
        // Now oscillate: +-0.3C (typical thermal noise)
        315.2,
        315.1,
        314.8,
        314.7,
        314.9,
        315.1,
        315.3,
        315.2,
        315.0,
        314.8,
        314.7,
        314.9,
        315.0,
        315.2,
        315.3,
        315.1,
        314.9,
        314.8,
        315.0,
        315.1,
        315.0,
        314.9,
        315.0,
        315.1,
        315.0,
        314.9,
        315.0,
        // Wider swing: +-0.5C
        315.5,
        315.3,
        315.0,
        314.6,
        314.5,
        314.7,
        315.0,
        315.4,
        315.5,
        315.2,
        314.9,
        314.6,
        314.8,
        315.1,
        315.3,
        315.0,
        314.8,
        315.0,
    };
    int n = sizeof(oscillation) / sizeof(oscillation[0]);

    std::vector<PidOutput> outputs;
    for (int i = 0; i < n; i++) {
        auto r = pid.compute(oscillation[i], SP, DT);
        outputs.push_back(r);

        EXPECT_TRUE(std::isfinite(r.rawOutput)) << "Step " << i;
        EXPECT_GE(r.rawOutput, 0.0) << "Step " << i;
        EXPECT_LE(r.rawOutput, 255.0) << "Step " << i;
    }

    // During steady-state oscillation, the output should remain moderate
    // (not pegged at 0 or 255). Check the oscillation phase (steps 6+).
    for (int i = 6; i < n; i++) {
        EXPECT_LT(outputs[i].rawOutput, 200.0)
            << "Step " << i << ": output too high during oscillation, PID not settling";
    }

    // The integral should be bounded and not wind up
    EXPECT_LE(std::fabs(pid.getIntegral()), productionConfig().integralLimit);
}

TEST_F(PidTrajectoryTest, CoolingAndReheating)
{
    // Start at setpoint, cool down (heater off or fan on), then reheat
    double trajectory[] = {
        // At setpoint
        315.0,
        315.0,
        315.0,
        // Cooling phase (gradual)
        314.5,
        313.8,
        312.5,
        311.0,
        309.0,
        306.0,
        303.0,
        300.0,
        // Reheating
        300.5,
        302.0,
        304.0,
        306.0,
        308.0,
        310.0,
        312.0,
        313.5,
        314.0,
        314.5,
        314.8,
        315.0,
        315.1,
        315.0,
    };
    int n = sizeof(trajectory) / sizeof(trajectory[0]);

    bool outputIncreasedDuringCooling = false;
    double prevOutput = 0.0;

    for (int i = 0; i < n; i++) {
        auto r = pid.compute(trajectory[i], SP, DT);

        EXPECT_TRUE(std::isfinite(r.rawOutput)) << "Step " << i;
        EXPECT_GE(r.rawOutput, 0.0) << "Step " << i;
        EXPECT_LE(r.rawOutput, 255.0) << "Step " << i;

        // During cooling (steps 3-10), output should generally increase
        // as the error grows
        if (i > 3 && i < 10 && r.rawOutput > prevOutput) {
            outputIncreasedDuringCooling = true;
        }

        prevOutput = r.rawOutput;
    }

    EXPECT_TRUE(outputIncreasedDuringCooling) << "PID should increase output when temperature drops below setpoint";
}

TEST_F(PidTrajectoryTest, SetpointChangeFromLowToHigh)
{
    // Start controlling at 200C, then change setpoint to 315C
    double sp1 = 200.0;
    double sp2 = 315.0;

    // Stabilize at sp1
    for (int i = 0; i < 50; i++) {
        pid.compute(200.0, sp1, DT);
    }
    auto stable = pid.compute(200.0, sp1, DT);
    EXPECT_TRUE(stable.inHoldMode);

    // Change setpoint - output should jump high
    auto r = pid.compute(200.0, sp2, DT);
    EXPECT_GT(r.rawOutput, stable.rawOutput);
    EXPECT_FALSE(r.inHoldMode); // now far from new setpoint
}

TEST_F(PidTrajectoryTest, LongRunStabilityWithRealisticNoise)
{
    // Run 5000 iterations (100s simulated) with small random-like noise
    // using a deterministic pattern
    double temp = 315.0;
    double noisePattern[] = { 0.1, -0.05, 0.15, -0.12, 0.08, -0.03, 0.2, -0.18, 0.05, -0.07 };
    int patternLen = sizeof(noisePattern) / sizeof(noisePattern[0]);

    for (int i = 0; i < 5000; i++) {
        temp = SP + noisePattern[i % patternLen];
        auto r = pid.compute(temp, SP, DT);

        EXPECT_TRUE(std::isfinite(r.rawOutput)) << "Iteration " << i;
        EXPECT_GE(r.rawOutput, 0.0) << "Iteration " << i;
        EXPECT_LE(r.rawOutput, 255.0) << "Iteration " << i;
    }

    // After 5000 iterations of small noise, integral should be bounded
    EXPECT_LE(std::fabs(pid.getIntegral()), productionConfig().integralLimit);
    EXPECT_TRUE(std::isfinite(pid.getFilteredTemperature()));
}

// ============================================================================
// Section 4: Error conditions and recovery
// ============================================================================

class PidErrorConditions : public ::testing::Test {
protected:
    PidController pid{ productionConfig() };
    static constexpr double DT = 0.02;
    static constexpr double SP = 315.0;
};

TEST_F(PidErrorConditions, NanInputResetsStateAndReturnsZero)
{
    // Build up some state (close to setpoint so anti-windup doesn't reverse sign)
    for (int i = 0; i < 20; i++) {
        pid.compute(314.0, SP, DT);
    }
    EXPECT_NE(pid.getIntegral(), 0.0);

    // NaN resets everything
    auto r = pid.compute(NAN, SP, DT);
    EXPECT_DOUBLE_EQ(r.rawOutput, 0.0);
    EXPECT_DOUBLE_EQ(r.pTerm, 0.0);
    EXPECT_DOUBLE_EQ(r.iTerm, 0.0);
    EXPECT_DOUBLE_EQ(r.dTerm, 0.0);
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.0);
    EXPECT_FALSE(pid.hasReachedSetpoint());
}

TEST_F(PidErrorConditions, InfinityInputResetsState)
{
    pid.compute(280.0, SP, DT);
    auto r = pid.compute(INFINITY, SP, DT);
    EXPECT_DOUBLE_EQ(r.rawOutput, 0.0);
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.0);
}

TEST_F(PidErrorConditions, NegativeInfinityInputResetsState)
{
    pid.compute(280.0, SP, DT);
    auto r = pid.compute(-INFINITY, SP, DT);
    EXPECT_DOUBLE_EQ(r.rawOutput, 0.0);
}

TEST_F(PidErrorConditions, ExtremeColdReadingResetsState)
{
    // isValidTemp rejects < -200
    pid.compute(280.0, SP, DT);
    auto r = pid.compute(-201.0, SP, DT);
    EXPECT_DOUBLE_EQ(r.rawOutput, 0.0);
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.0);
}

TEST_F(PidErrorConditions, ExtremeHotReadingResetsState)
{
    // isValidTemp rejects >= 1800
    pid.compute(280.0, SP, DT);
    auto r = pid.compute(1800.0, SP, DT);
    EXPECT_DOUBLE_EQ(r.rawOutput, 0.0);
}

TEST_F(PidErrorConditions, BoundaryValidTemperatures)
{
    // -200 is valid (just barely)
    auto r1 = pid.compute(-200.0, SP, DT);
    EXPECT_TRUE(std::isfinite(r1.rawOutput));

    pid.reset();

    // -200.01 is invalid
    auto r2 = pid.compute(-200.01, SP, DT);
    EXPECT_DOUBLE_EQ(r2.rawOutput, 0.0);

    pid.reset();

    // 1799.9 is valid
    auto r3 = pid.compute(1799.9, SP, DT);
    EXPECT_TRUE(std::isfinite(r3.rawOutput));
}

TEST_F(PidErrorConditions, SensorDropoutAndRecovery)
{
    // Simulate: good readings -> sensor fails (NaN) -> good readings resume
    // Verify PID recovers gracefully

    // Phase 1: establish state
    for (int i = 0; i < 50; i++) {
        pid.compute(310.0, SP, DT);
    }
    double integralBefore = pid.getIntegral();
    EXPECT_GT(integralBefore, 0.0);

    // Phase 2: sensor fails
    auto failResult = pid.compute(NAN, SP, DT);
    EXPECT_DOUBLE_EQ(failResult.rawOutput, 0.0);
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.0);

    // Phase 3: sensor recovers - PID should restart cleanly
    auto r1 = pid.compute(310.0, SP, DT);
    EXPECT_TRUE(std::isfinite(r1.rawOutput));
    EXPECT_GT(r1.rawOutput, 0.0);

    // After a few more steps it should be working normally again
    for (int i = 0; i < 10; i++) {
        auto r = pid.compute(310.0, SP, DT);
        EXPECT_TRUE(std::isfinite(r.rawOutput));
        EXPECT_GT(r.rawOutput, 0.0);
    }
}

TEST_F(PidErrorConditions, MultipleSensorDropouts)
{
    // Rapid fail/recover cycles
    for (int cycle = 0; cycle < 10; cycle++) {
        // Good reading
        auto rGood = pid.compute(300.0, SP, DT);
        EXPECT_TRUE(std::isfinite(rGood.rawOutput));

        // Bad reading
        auto rBad = pid.compute(NAN, SP, DT);
        EXPECT_DOUBLE_EQ(rBad.rawOutput, 0.0);
    }

    // Must still work after all the chaos
    auto rFinal = pid.compute(300.0, SP, DT);
    EXPECT_TRUE(std::isfinite(rFinal.rawOutput));
    EXPECT_GT(rFinal.rawOutput, 0.0);
}

TEST_F(PidErrorConditions, ZeroDtClampedToMinimum)
{
    // dt=0 is changed to 0.001 internally
    auto r = pid.compute(300.0, SP, 0.0);
    EXPECT_TRUE(std::isfinite(r.rawOutput));
    EXPECT_GT(r.rawOutput, 0.0);
}

TEST_F(PidErrorConditions, NegativeDtClampedToMinimum)
{
    auto r = pid.compute(300.0, SP, -5.0);
    EXPECT_TRUE(std::isfinite(r.rawOutput));
}

TEST_F(PidErrorConditions, VeryLargeDtDoesNotProduceInfinity)
{
    auto r = pid.compute(300.0, SP, 1000.0);
    EXPECT_TRUE(std::isfinite(r.rawOutput));
    EXPECT_GE(r.rawOutput, 0.0);
    EXPECT_LE(r.rawOutput, 255.0);
    EXPECT_LE(pid.getIntegral(), productionConfig().integralLimit);
}

TEST_F(PidErrorConditions, VerySmallDt)
{
    auto r = pid.compute(300.0, SP, 0.0000001);
    EXPECT_TRUE(std::isfinite(r.rawOutput));
    EXPECT_GE(r.rawOutput, 0.0);
    EXPECT_LE(r.rawOutput, 255.0);
}

TEST_F(PidErrorConditions, SuddenTemperatureSpike)
{
    // Simulate sudden spike: 315 -> 500 -> 315
    for (int i = 0; i < 10; i++) {
        pid.compute(315.0, SP, DT);
    }

    // Spike: should produce zero output (temp way above setpoint)
    auto rSpike = pid.compute(500.0, SP, DT);
    EXPECT_DOUBLE_EQ(rSpike.rawOutput, 0.0);

    // Return to normal: output should recover
    auto rReturn = pid.compute(315.0, SP, DT);
    EXPECT_TRUE(std::isfinite(rReturn.rawOutput));
}

TEST_F(PidErrorConditions, SuddenTemperatureDrop)
{
    // Simulate sensor reading dropping to near-zero
    for (int i = 0; i < 10; i++) {
        pid.compute(315.0, SP, DT);
    }

    // Drop to 0: should produce max output
    auto rDrop = pid.compute(0.0, SP, DT);
    EXPECT_DOUBLE_EQ(rDrop.rawOutput, 255.0);
}

TEST_F(PidErrorConditions, SetpointAtZero)
{
    auto r = pid.compute(100.0, 0.0, DT);
    EXPECT_TRUE(std::isfinite(r.rawOutput));
    // temp > sp -> output should be 0
    EXPECT_DOUBLE_EQ(r.rawOutput, 0.0);
}

TEST_F(PidErrorConditions, SetpointNegative)
{
    auto r = pid.compute(-100.0, -50.0, DT);
    EXPECT_TRUE(std::isfinite(r.rawOutput));
}

TEST_F(PidErrorConditions, ProcessValueEqualsSetpointExactly)
{
    auto r = pid.compute(SP, SP, DT);
    EXPECT_TRUE(std::isfinite(r.rawOutput));
    // Error = 0 -> P = 0, I barely accumulates, D = 0
    EXPECT_NEAR(r.pTerm, 0.0, 0.01);
}

// ============================================================================
// Section 5: Different config / correction values
// ============================================================================

TEST(PidDifferentConfigs, AggressiveVsConservativeGains)
{
    auto aggressive = productionConfig();
    aggressive.approachGains = { 40.0, 2.0, 8.0 };
    aggressive.holdGains = { 20.0, 1.0, 8.0 };

    auto conservative = productionConfig();
    conservative.approachGains = { 5.0, 0.1, 1.0 };
    conservative.holdGains = { 2.0, 0.05, 1.0 };

    PidController pidA(aggressive);
    PidController pidC(conservative);

    double dt = 0.02;
    double sp = 300.0;
    double temp = 280.0;

    auto rA = pidA.compute(temp, sp, dt);
    auto rC = pidC.compute(temp, sp, dt);

    // Aggressive should produce higher output
    EXPECT_GT(rA.rawOutput, rC.rawOutput);
    EXPECT_GT(std::fabs(rA.pTerm), std::fabs(rC.pTerm));
}

TEST(PidDifferentConfigs, HighVsLowOutputMax)
{
    auto cfg100 = simpleConfig();
    cfg100.outputMax = 100.0;
    cfg100.approachGains = { 100.0, 0.0, 0.0 };
    cfg100.holdGains = { 100.0, 0.0, 0.0 };

    auto cfg1000 = simpleConfig();
    cfg1000.outputMax = 1000.0;
    cfg1000.approachGains = { 100.0, 0.0, 0.0 };
    cfg1000.holdGains = { 100.0, 0.0, 0.0 };

    PidController pid100(cfg100);
    PidController pid1000(cfg1000);

    auto r100 = pid100.compute(200.0, 300.0, 0.02);
    auto r1000 = pid1000.compute(200.0, 300.0, 0.02);

    EXPECT_DOUBLE_EQ(r100.rawOutput, 100.0);
    EXPECT_GT(r1000.rawOutput, 100.0);
}

TEST(PidDifferentConfigs, NarrowVsWideGainSwitchBand)
{
    auto narrow = simpleConfig();
    narrow.gainSwitchBandC = 0.1;
    narrow.approachGains = { 20.0, 0.0, 0.0 };
    narrow.holdGains = { 2.0, 0.0, 0.0 };

    auto wide = simpleConfig();
    wide.gainSwitchBandC = 10.0;
    wide.approachGains = { 20.0, 0.0, 0.0 };
    wide.holdGains = { 2.0, 0.0, 0.0 };

    PidController pidN(narrow);
    PidController pidW(wide);

    double sp = 300.0;
    double temp = sp - 0.5; // 0.5C from setpoint

    auto rN = pidN.compute(temp, sp, 0.02);
    auto rW = pidW.compute(temp, sp, 0.02);

    // Narrow band: 0.5 > 0.1 -> approach mode -> Kp=20
    EXPECT_FALSE(rN.inHoldMode);
    // Wide band: 0.5 < 10 -> hold mode -> Kp=2
    EXPECT_TRUE(rW.inHoldMode);
    EXPECT_GT(rN.rawOutput, rW.rawOutput);
}

TEST(PidDifferentConfigs, NarrowVsWideHysteresis)
{
    auto narrow = simpleConfig();
    narrow.hysteresisAboveSetpointC = 0.1;
    narrow.hysteresisBelowSetpointC = 0.1;

    auto wide = simpleConfig();
    wide.hysteresisAboveSetpointC = 5.0;
    wide.hysteresisBelowSetpointC = 5.0;

    PidController pidN(narrow);
    PidController pidW(wide);

    double sp = 300.0;

    // Both should latch at sp + their respective threshold
    pidN.compute(sp + 0.1, sp, 0.02);
    pidW.compute(sp + 0.1, sp, 0.02);
    EXPECT_TRUE(pidN.hasReachedSetpoint()); // 0.1 >= 0.1
    EXPECT_FALSE(pidW.hasReachedSetpoint()); // 0.1 < 5.0

    pidW.compute(sp + 5.0, sp, 0.02);
    EXPECT_TRUE(pidW.hasReachedSetpoint());
}

TEST(PidDifferentConfigs, ZeroAntiwindupGain)
{
    auto cfg = simpleConfig();
    cfg.antiwindupGain = 0.0;
    cfg.approachGains = { 0.0, 10.0, 0.0 };
    cfg.holdGains = { 0.0, 10.0, 0.0 };
    PidController pid(cfg);

    pid.setOutputLimits(0.0, 50.0); // Low limit to saturate quickly

    // With no anti-windup, integral should hit the integralLimit
    for (int i = 0; i < 200; i++) {
        pid.compute(100.0, 300.0, 1.0);
    }

    // Integral should reach integralLimit (500) since no back-calculation
    EXPECT_NEAR(pid.getIntegral(), cfg.integralLimit, 1.0);
}

TEST(PidDifferentConfigs, StrongAntiwindupReducesIntegralMoreThanWeak)
{
    // Compare two PIDs: one with strong anti-windup, one with none.
    // Strong anti-windup should keep the integral smaller after the
    // output saturates and we then cross the setpoint.
    auto cfgStrong = simpleConfig();
    cfgStrong.antiwindupGain = 1.0;
    cfgStrong.approachGains = { 0.0, 5.0, 0.0 };
    cfgStrong.holdGains = { 0.0, 5.0, 0.0 };

    auto cfgNone = simpleConfig();
    cfgNone.antiwindupGain = 0.0;
    cfgNone.approachGains = { 0.0, 5.0, 0.0 };
    cfgNone.holdGains = { 0.0, 5.0, 0.0 };

    PidController pidStrong(cfgStrong);
    PidController pidNone(cfgNone);

    pidStrong.setOutputLimits(0.0, 50.0);
    pidNone.setOutputLimits(0.0, 50.0);

    // Saturate both for a while
    for (int i = 0; i < 50; i++) {
        pidStrong.compute(200.0, 300.0, 0.02);
        pidNone.compute(200.0, 300.0, 0.02);
    }

    // Strong anti-windup should have reduced integral relative to no-antiwindup
    EXPECT_LT(pidStrong.getIntegral(), pidNone.getIntegral());
}

TEST(PidDifferentConfigs, POnlyController)
{
    auto cfg = simpleConfig();
    cfg.approachGains = { 10.0, 0.0, 0.0 };
    cfg.holdGains = { 5.0, 0.0, 0.0 };
    PidController pid(cfg);

    for (int i = 0; i < 100; i++) {
        auto r = pid.compute(280.0, 300.0, 0.02);
        EXPECT_NEAR(r.iTerm, 0.0, 0.001);
        EXPECT_NEAR(r.dTerm, 0.0, 0.001);
        // Only P is active
        EXPECT_GT(r.pTerm, 0.0);
    }
}

TEST(PidDifferentConfigs, PIController)
{
    auto cfg = simpleConfig();
    cfg.approachGains = { 10.0, 1.0, 0.0 };
    cfg.holdGains = { 5.0, 0.5, 0.0 };
    PidController pid(cfg);

    PidOutput lastResult;
    for (int i = 0; i < 100; i++) {
        lastResult = pid.compute(280.0, 300.0, 0.02);
    }

    EXPECT_GT(lastResult.pTerm, 0.0);
    EXPECT_GT(lastResult.iTerm, 0.0);
    EXPECT_NEAR(lastResult.dTerm, 0.0, 0.001);
}

TEST(PidDifferentConfigs, LowIntegralLimit)
{
    auto cfg = simpleConfig();
    cfg.integralLimit = 1.0; // very tight
    cfg.approachGains = { 0.0, 100.0, 0.0 };
    cfg.holdGains = { 0.0, 100.0, 0.0 };
    PidController pid(cfg);

    for (int i = 0; i < 100; i++) {
        pid.compute(200.0, 300.0, 1.0);
    }

    EXPECT_LE(pid.getIntegral(), 1.0);
    EXPECT_GE(pid.getIntegral(), -1.0);
}

TEST(PidDifferentConfigs, NoSetpointApproachOffset)
{
    auto cfg = simpleConfig();
    cfg.setpointApproachOffsetC = 0.0;
    PidController pid(cfg);

    double sp = 300.0;
    auto r = pid.compute(100.0, sp, 0.02);
    EXPECT_DOUBLE_EQ(r.effectiveSetpoint, sp);
}

TEST(PidDifferentConfigs, LargeSetpointApproachOffset)
{
    auto cfg = simpleConfig();
    cfg.setpointApproachOffsetC = 10.0;
    PidController pid(cfg);

    double sp = 300.0;
    auto r = pid.compute(100.0, sp, 0.02);
    EXPECT_DOUBLE_EQ(r.effectiveSetpoint, sp + 10.0);
}

TEST(PidDifferentConfigs, NoKiReduction)
{
    auto cfg = simpleConfig();
    cfg.kiReductionBandNarrowC = 0.0;
    cfg.kiReductionBandWideC = 0.0;
    PidController pid(cfg);

    PidGains gains = { 0.0, 10.0, 0.0 };
    pid.setGains(gains, gains);

    double sp = 300.0;
    double temp = sp - 0.1;
    auto r = pid.compute(temp, sp, 1.0);

    // No reduction: full Ki applied
    double error = sp - temp;
    EXPECT_NEAR(r.iTerm, 10.0 * error, 0.1);
}

TEST(PidDifferentConfigs, FullKiReductionInNarrowBand)
{
    auto cfg = simpleConfig();
    cfg.kiReductionFactorNarrow = 0.0; // complete suppression
    cfg.kiReductionBandNarrowC = 5.0; // wide narrow band
    PidController pid(cfg);

    PidGains gains = { 0.0, 10.0, 0.0 };
    pid.setGains(gains, gains);

    double sp = 300.0;
    double temp = sp - 1.0; // within narrow band (error=1 < 5)
    auto r = pid.compute(temp, sp, 1.0);

    // Ki reduced to 0 -> iTerm should be 0
    EXPECT_NEAR(r.iTerm, 0.0, 0.001);
}

// ============================================================================
// Section 6: Cross-config comparison (production vs modified)
// ============================================================================

TEST(PidCrossConfig, ProductionConfigProducesSameResultsAsTwoIdenticalInstances)
{
    PidController pid1(productionConfig());
    PidController pid2(productionConfig());

    double sp = 315.0;
    double temps[] = { 100.0, 200.0, 280.0, 310.0, 314.0, 315.0, 316.0, 314.5, 315.0 };

    for (double t : temps) {
        auto r1 = pid1.compute(t, sp, 0.02);
        auto r2 = pid2.compute(t, sp, 0.02);

        EXPECT_DOUBLE_EQ(r1.rawOutput, r2.rawOutput) << "at temp=" << t;
        EXPECT_DOUBLE_EQ(r1.pTerm, r2.pTerm) << "at temp=" << t;
        EXPECT_DOUBLE_EQ(r1.iTerm, r2.iTerm) << "at temp=" << t;
        EXPECT_DOUBLE_EQ(r1.dTerm, r2.dTerm) << "at temp=" << t;
        EXPECT_DOUBLE_EQ(r1.effectiveSetpoint, r2.effectiveSetpoint) << "at temp=" << t;
        EXPECT_EQ(r1.inHoldMode, r2.inHoldMode) << "at temp=" << t;
    }
}

TEST(PidCrossConfig, ResetRestoresToInitialBehavior)
{
    PidController pid(productionConfig());

    double sp = 315.0;

    // First compute after construction
    auto rFirst = pid.compute(300.0, sp, 0.02);

    // Run many iterations to build up state
    for (int i = 0; i < 100; i++) {
        pid.compute(310.0 + (i % 10) * 0.5, sp, 0.02);
    }

    // Reset and compute at the same point as first call
    pid.reset();
    auto rAfterReset = pid.compute(300.0, sp, 0.02);

    // Should produce identical results
    EXPECT_DOUBLE_EQ(rFirst.rawOutput, rAfterReset.rawOutput);
    EXPECT_DOUBLE_EQ(rFirst.pTerm, rAfterReset.pTerm);
    EXPECT_DOUBLE_EQ(rFirst.iTerm, rAfterReset.iTerm);
    EXPECT_DOUBLE_EQ(rFirst.dTerm, rAfterReset.dTerm);
    EXPECT_DOUBLE_EQ(rFirst.effectiveSetpoint, rAfterReset.effectiveSetpoint);
}

TEST(PidCrossConfig, ModifiedGainsViaSetGainsMatchNewConfig)
{
    // Create one PID with default config, then modify gains
    PidController pidModified(productionConfig());
    PidGains newApproach = { 50.0, 1.0, 3.0 };
    PidGains newHold = { 25.0, 0.5, 3.0 };
    pidModified.setGains(newApproach, newHold);

    // Create another PID with the gains set in config
    auto cfg = productionConfig();
    cfg.approachGains = newApproach;
    cfg.holdGains = newHold;
    PidController pidConfig(cfg);

    double sp = 315.0;
    auto r1 = pidModified.compute(300.0, sp, 0.02);
    auto r2 = pidConfig.compute(300.0, sp, 0.02);

    EXPECT_DOUBLE_EQ(r1.rawOutput, r2.rawOutput);
    EXPECT_DOUBLE_EQ(r1.pTerm, r2.pTerm);
}

// ============================================================================
// Section 7: Bumpless transfer with different configs
// ============================================================================

TEST(PidBumplessTransfer, TransferPreservesOutputContinuity)
{
    PidController pid(productionConfig());

    double currentOutput = 180.0;
    double currentTemp = 310.0;
    double sp = 315.0;

    pid.initializeForBumplessTransfer(currentOutput, currentTemp, sp);

    // Immediate compute should produce output close to currentOutput
    auto r = pid.compute(currentTemp, sp, 0.02);
    // P term = Kp * error = ~20 * 5 = ~100
    // I term should be pre-loaded to make total ≈ currentOutput
    // Allow reasonable tolerance
    EXPECT_NEAR(r.rawOutput, currentOutput, 100.0);
    EXPECT_TRUE(std::isfinite(r.rawOutput));
}

TEST(PidBumplessTransfer, TransferWithZeroOutput)
{
    PidController pid(productionConfig());
    pid.initializeForBumplessTransfer(0.0, 315.0, 315.0);

    auto r = pid.compute(315.0, 315.0, 0.02);
    EXPECT_TRUE(std::isfinite(r.rawOutput));
    EXPECT_GE(r.rawOutput, 0.0);
}

TEST(PidBumplessTransfer, TransferWithMaxOutput)
{
    PidController pid(productionConfig());
    pid.initializeForBumplessTransfer(255.0, 200.0, 315.0);

    auto r = pid.compute(200.0, 315.0, 0.02);
    EXPECT_TRUE(std::isfinite(r.rawOutput));
    EXPECT_EQ(r.rawOutput, 255.0); // should still saturate
}

TEST(PidBumplessTransfer, IntegralPreloadClampedToLimit)
{
    PidController pid(productionConfig());

    // Very high output would require huge integral -> should be clamped
    pid.initializeForBumplessTransfer(10000.0, 315.0, 315.0);
    EXPECT_LE(pid.getIntegral(), productionConfig().integralLimit);
    EXPECT_GE(pid.getIntegral(), -productionConfig().integralLimit);
}

// ============================================================================
// Section 8: Derivative enable/disable logic in detail
// ============================================================================

TEST(PidDerivativeLogic, DisabledFarBelowSetpoint)
{
    auto cfg = simpleConfig();
    cfg.approachGains = { 0.0, 0.0, 100.0 };
    cfg.holdGains = { 0.0, 0.0, 100.0 };
    cfg.gainSwitchBandC = 1.0;
    cfg.derivativeEnableMarginC = 0.5;
    PidController pid(cfg);

    // Far below setpoint: derivative disabled
    pid.compute(200.0, 300.0, 0.02);
    auto r = pid.compute(210.0, 300.0, 0.02);

    // Error ~90, which is > gainSwitchBandC + derivativeEnableMarginC (1.5)
    // And processValue (210) < effectiveSetpoint (302)
    // Both conditions false -> dTerm = 0
    EXPECT_NEAR(r.dTerm, 0.0, 0.01);
    EXPECT_NE(r.derivativeCps, 0.0); // derivative IS computed, just not applied
}

TEST(PidDerivativeLogic, EnabledWithinMargin)
{
    auto cfg = simpleConfig();
    cfg.approachGains = { 0.0, 0.0, 10.0 };
    cfg.holdGains = { 0.0, 0.0, 10.0 };
    cfg.gainSwitchBandC = 1.0;
    cfg.derivativeEnableMarginC = 0.5;
    PidController pid(cfg);

    double sp = 300.0;
    // Error = 1.2, which is < gainSwitchBandC + margin (1.5)
    pid.compute(298.8, sp, 0.02);
    auto r = pid.compute(299.0, sp, 0.02);
    EXPECT_NE(r.dTerm, 0.0);
}

TEST(PidDerivativeLogic, EnabledAboveSetpoint)
{
    auto cfg = simpleConfig();
    cfg.approachGains = { 0.0, 0.0, 10.0 };
    cfg.holdGains = { 0.0, 0.0, 10.0 };
    PidController pid(cfg);

    double sp = 300.0;
    pid.compute(301.0, sp, 0.02);
    auto r = pid.compute(302.0, sp, 0.02);

    // processValue (302) > effectiveSetpoint (300) -> D enabled
    EXPECT_NE(r.dTerm, 0.0);
}

TEST(PidDerivativeLogic, DerivativeIsOnProcessNotError)
{
    // PID uses derivative-on-measurement (negative of dTemp/dt)
    auto cfg = simpleConfig();
    cfg.derivativeFilterAlpha = 1.0;
    cfg.approachGains = { 0.0, 0.0, 10.0 };
    cfg.holdGains = { 0.0, 0.0, 10.0 };
    cfg.gainSwitchBandC = 100.0;
    cfg.derivativeEnableMarginC = 100.0;
    PidController pid(cfg);

    pid.compute(300.0, 300.0, 0.02);
    // Temp rises by 1C -> dTerm should be negative (opposing the rise)
    auto r = pid.compute(301.0, 300.0, 0.02);
    EXPECT_LT(r.dTerm, 0.0);

    pid.reset();
    pid.compute(300.0, 300.0, 0.02);
    // Temp drops by 1C -> dTerm should be positive (opposing the fall)
    auto r2 = pid.compute(299.0, 300.0, 0.02);
    EXPECT_GT(r2.dTerm, 0.0);
}
