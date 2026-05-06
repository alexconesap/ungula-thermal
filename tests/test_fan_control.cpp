#include <gtest/gtest.h>
#include <ungula/thermal/fan_control.h>

using namespace ungula::thermal;

// ---- Test-local constants (values from the old thermal_config.h) ----

static constexpr uint8_t FAN_PULSES_PER_REV = 2;
static constexpr uint16_t FAN_MIN_RPM_THRESHOLD = 50;
static constexpr uint32_t FAN_SAMPLE_PERIOD_MS = 1000;
static constexpr uint32_t FAN_MIN_EDGE_INTERVAL_US = 1000;
static constexpr uint16_t FAN_PWM_RESOLUTION = 255;
static constexpr bool FAN_INVERTED_OUTPUT = true;

static inline FanTachConfig testFanTachConfig() {
    return FanTachConfig{
            .pulsesPerRev = FAN_PULSES_PER_REV,
            .minRpmThreshold = FAN_MIN_RPM_THRESHOLD,
            .samplePeriodMs = FAN_SAMPLE_PERIOD_MS,
            .minEdgeIntervalUs = FAN_MIN_EDGE_INTERVAL_US,
    };
}

static inline FanOutputConfig testFanOutputConfig() {
    return FanOutputConfig{
            .pwmResolution = FAN_PWM_RESOLUTION,
            .invertedOutput = FAN_INVERTED_OUTPUT,
    };
}

// Test with 4 channels (typical configuration)
static constexpr uint8_t TEST_NUM_CHANNELS = 4;

// --- FanTachometer ---

TEST(FanTachometer, InitialStateZeroRpm) {
    FanTachometer<TEST_NUM_CHANNELS> tach(testFanTachConfig());
    auto& state = tach.getChannelState(0);
    EXPECT_EQ(state.rpm, 0);
    EXPECT_FALSE(state.connected);
}

TEST(FanTachometer, RecordPulseAndUpdateRpm) {
    FanTachometer<TEST_NUM_CHANNELS> tach(testFanTachConfig());

    // Record enough pulses for 3000 RPM (with 2 pulses per rev):
    // 3000 RPM = 100 pulses/sec over FAN_SAMPLE_PERIOD_MS (1s)
    uint32_t ts = 10000;
    for (int i = 0; i < 100; i++) {
        tach.recordPulse(0, ts);
        ts += 10000;  // 10ms interval
    }

    tach.update(false);
    auto& state = tach.getChannelState(0);
    EXPECT_EQ(state.rpm, (100 * 60) / FAN_PULSES_PER_REV);
    EXPECT_TRUE(state.connected);
}

TEST(FanTachometer, DebounceRejectsFastPulses) {
    FanTachometer<TEST_NUM_CHANNELS> tach(testFanTachConfig());

    uint32_t ts = 10000;
    tach.recordPulse(0, ts);

    // Second pulse too fast (within FAN_MIN_EDGE_INTERVAL_US)
    bool accepted = tach.recordPulse(0, ts + FAN_MIN_EDGE_INTERVAL_US / 2);
    EXPECT_FALSE(accepted);

    // After debounce period
    accepted = tach.recordPulse(0, ts + FAN_MIN_EDGE_INTERVAL_US + 1);
    EXPECT_TRUE(accepted);
}

TEST(FanTachometer, NotConnectedWhenCommandedOff) {
    FanTachometer<TEST_NUM_CHANNELS> tach(testFanTachConfig());

    uint32_t ts = 10000;
    for (int i = 0; i < 100; i++) {
        tach.recordPulse(0, ts);
        ts += 10000;
    }

    tach.update(true);  // commanded off
    auto& state = tach.getChannelState(0);
    EXPECT_FALSE(state.connected);
}

TEST(FanTachometer, InvalidChannelReturnsSafeDefaults) {
    FanTachometer<TEST_NUM_CHANNELS> tach(testFanTachConfig());
    auto& state = tach.getChannelState(99);
    EXPECT_EQ(state.rpm, 0);
    EXPECT_FALSE(state.connected);
}

TEST(FanTachometer, ResetClearsAllChannels) {
    FanTachometer<TEST_NUM_CHANNELS> tach(testFanTachConfig());
    tach.recordPulse(0, 100000);
    tach.recordPulse(1, 100000);
    tach.reset();

    EXPECT_EQ(tach.getPulseCount(0), 0u);
    EXPECT_EQ(tach.getPulseCount(1), 0u);
}

TEST(FanTachometer, SingleChannelWorks) {
    FanTachometer<1> tach(testFanTachConfig());

    uint32_t ts = 10000;
    for (int i = 0; i < 50; i++) {
        tach.recordPulse(0, ts);
        ts += 10000;
    }

    tach.update(false);
    EXPECT_GT(tach.getChannelState(0).rpm, 0);

    // Channel 1 does not exist
    EXPECT_FALSE(tach.recordPulse(1, ts));
    EXPECT_EQ(tach.getChannelState(1).rpm, 0);
}

TEST(FanTachometer, EightChannelsWork) {
    FanTachometer<8> tach(testFanTachConfig());

    uint32_t ts = 10000;
    for (uint8_t ch = 0; ch < 8; ++ch) {
        for (int i = 0; i < 20; i++) {
            tach.recordPulse(ch, ts);
            ts += 10000;
        }
    }

    tach.update(false);
    for (uint8_t ch = 0; ch < 8; ++ch) {
        EXPECT_GT(tach.getChannelState(ch).rpm, 0);
        EXPECT_TRUE(tach.getChannelState(ch).connected);
    }
}

// --- FanPwmCalculator ---

TEST(FanPwmCalculator, ZeroPercentInvertedReturnsMax) {
    FanOutputConfig cfg{.pwmResolution = FAN_PWM_RESOLUTION, .invertedOutput = true};
    FanPwmCalculator calc(cfg);

    uint16_t duty = calc.calculateDuty(0);
    EXPECT_EQ(duty, FAN_PWM_RESOLUTION);
}

TEST(FanPwmCalculator, HundredPercentInvertedReturnsZero) {
    FanOutputConfig cfg{.pwmResolution = FAN_PWM_RESOLUTION, .invertedOutput = true};
    FanPwmCalculator calc(cfg);

    uint16_t duty = calc.calculateDuty(100);
    EXPECT_EQ(duty, 0);
}

TEST(FanPwmCalculator, FiftyPercentNonInverted) {
    FanOutputConfig cfg{.pwmResolution = FAN_PWM_RESOLUTION, .invertedOutput = false};
    FanPwmCalculator calc(cfg);

    uint16_t duty = calc.calculateDuty(50);
    EXPECT_EQ(duty, FAN_PWM_RESOLUTION / 2);
}

TEST(FanPwmCalculator, NegativePercentClampedToZero) {
    FanOutputConfig cfg{.pwmResolution = FAN_PWM_RESOLUTION, .invertedOutput = false};
    FanPwmCalculator calc(cfg);

    uint16_t duty = calc.calculateDuty(-10);
    EXPECT_EQ(duty, 0);
}

TEST(FanPwmCalculator, OverHundredPercentClamped) {
    FanOutputConfig cfg{.pwmResolution = FAN_PWM_RESOLUTION, .invertedOutput = false};
    FanPwmCalculator calc(cfg);

    uint16_t duty = calc.calculateDuty(150);
    EXPECT_EQ(duty, FAN_PWM_RESOLUTION);
}

// --- FanController ---

TEST(FanController, UpdateRespectsSamplePeriod) {
    FanController<TEST_NUM_CHANNELS> fc(testFanTachConfig(), testFanOutputConfig());

    bool updated = fc.update(1000, 50);
    EXPECT_TRUE(updated);  // first call always updates

    updated = fc.update(1500, 50);  // within 1000ms period
    EXPECT_FALSE(updated);

    updated = fc.update(2001, 50);
    EXPECT_TRUE(updated);
}

TEST(FanController, NoFansConnectedInitially) {
    FanController<TEST_NUM_CHANNELS> fc(testFanTachConfig(), testFanOutputConfig());
    EXPECT_FALSE(fc.isAnyFanConnected());
    EXPECT_FALSE(fc.areAllFansConnected());
}

TEST(FanController, NumChannelsExposed) {
    FanController<4> fc4(testFanTachConfig(), testFanOutputConfig());
    EXPECT_EQ(fc4.NUM_CHANNELS, 4);

    FanController<1> fc1(testFanTachConfig(), testFanOutputConfig());
    EXPECT_EQ(fc1.NUM_CHANNELS, 1);

    FanController<8> fc8(testFanTachConfig(), testFanOutputConfig());
    EXPECT_EQ(fc8.NUM_CHANNELS, 8);
}
