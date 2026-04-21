#include <gtest/gtest.h>
#include <thermal/temp_sensor.h>

#include <cmath>

using namespace ungula::thermal;

// ---- Test-local constants and config builder (values from the old thermal_config.h) ----

static constexpr double MAX_VALID_TEMP_C = 350.0;
static constexpr double MIN_VALID_TEMP_C = -40.0;

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
        .maxValidTempC = MAX_VALID_TEMP_C,
        .minValidTempC = MIN_VALID_TEMP_C,
    };
}

// --- NTC conversion tests ---

TEST(NtcConversion, ValidMillivoltsReturnsTemperature) {
    NtcConfig cfg = testNtcConfig();
    // Mid-range millivolts should produce a valid temperature
    double temp = ntcMillivoltsToTempC(1500, cfg);
    EXPECT_TRUE(std::isfinite(temp));
    EXPECT_GT(temp, MIN_VALID_TEMP_C);
    EXPECT_LT(temp, MAX_VALID_TEMP_C);
}

TEST(NtcConversion, DisconnectedDetection) {
    NtcConfig cfg = testNtcConfig();
    // Very low voltage = disconnected sensor
    double temp = ntcMillivoltsToTempC(1, cfg);
    EXPECT_TRUE(std::isnan(temp));
}

TEST(NtcConversion, ZeroMillivoltsReturnsNan) {
    NtcConfig cfg = testNtcConfig();
    double temp = ntcMillivoltsToTempC(0, cfg);
    EXPECT_TRUE(std::isnan(temp));
}

TEST(NtcConversion, SupplyVoltageReturnsNan) {
    NtcConfig cfg = testNtcConfig();
    // At or above supply voltage the denominator goes to zero
    double temp = ntcMillivoltsToTempC(cfg.supplyVoltageMv, cfg);
    EXPECT_TRUE(std::isnan(temp));
}

TEST(NtcConversion, AboveSupplyVoltageReturnsNan) {
    NtcConfig cfg = testNtcConfig();
    double temp = ntcMillivoltsToTempC(cfg.supplyVoltageMv + 100, cfg);
    EXPECT_TRUE(std::isnan(temp));
}

TEST(NtcConversion, HigherMillivoltsProduceLowerTemperature) {
    // NTC with series resistor: higher voltage = higher resistance = lower temp
    // Use mid-range values that produce valid temperatures
    NtcConfig cfg = testNtcConfig();
    double temp1 = ntcMillivoltsToTempC(1200, cfg);
    double temp2 = ntcMillivoltsToTempC(2800, cfg);

    ASSERT_TRUE(std::isfinite(temp1));
    ASSERT_TRUE(std::isfinite(temp2));
    EXPECT_GT(temp1, temp2);
}

// --- Calibration offset ---

TEST(CalibrationOffset, AppliesOffset) {
    double raw = 100.0;
    double offset = -5.0;
    double result = applyCalibrationOffset(raw, offset);
    EXPECT_DOUBLE_EQ(result, 95.0);
}

TEST(CalibrationOffset, NanInputStaysNan) {
    double result = applyCalibrationOffset(NAN, -5.0);
    EXPECT_TRUE(std::isnan(result));
}

// --- TemperatureSensor class ---

TEST(TemperatureSensor, ReadValidTemperature) {
    TemperatureSensor sensor(0, 34, 0.0, testNtcConfig());
    double temp = sensor.readTemperatureC(1500);
    EXPECT_TRUE(std::isfinite(temp));
    EXPECT_TRUE(sensor.isConnected());
}

TEST(TemperatureSensor, ReadDisconnectedSensor) {
    TemperatureSensor sensor(0, 34, 0.0, testNtcConfig());
    sensor.readTemperatureC(0);
    EXPECT_FALSE(sensor.isConnected());
}

TEST(TemperatureSensor, CalibrationOffsetApplied) {
    TemperatureSensor sensor1(0, 34, 0.0, testNtcConfig());
    TemperatureSensor sensor2(0, 34, -10.0, testNtcConfig());

    double temp1 = sensor1.readTemperatureC(1500);
    double temp2 = sensor2.readTemperatureC(1500);

    ASSERT_TRUE(std::isfinite(temp1));
    ASSERT_TRUE(std::isfinite(temp2));
    EXPECT_NEAR(temp2, temp1 - 10.0, 0.01);
}

TEST(TemperatureSensor, GettersReturnCorrectValues) {
    TemperatureSensor sensor(2, 36, -5.0, testNtcConfig());
    EXPECT_EQ(sensor.getChannelIndex(), 2);
    EXPECT_EQ(sensor.getAdcPin(), 36);
    EXPECT_NEAR(sensor.getCalibrationOffset(), -5.0, 0.001);
}

// --- ADC median filtering ---

TEST(AdcMedian, SortFiveElements) {
    int arr[5] = {5, 3, 1, 4, 2};
    adc::sortFiveElements(arr);
    EXPECT_EQ(arr[0], 1);
    EXPECT_EQ(arr[1], 2);
    EXPECT_EQ(arr[2], 3);
    EXPECT_EQ(arr[3], 4);
    EXPECT_EQ(arr[4], 5);
}

TEST(AdcMedian, MedianOfFiveReturnsMidValue) {
    int arr[5] = {10, 30, 20, 50, 40};
    int median = adc::computeMedianOfFive(arr);
    EXPECT_EQ(median, 30);
}

TEST(AdcMedian, MedianOfIdenticalValues) {
    int arr[5] = {42, 42, 42, 42, 42};
    int median = adc::computeMedianOfFive(arr);
    EXPECT_EQ(median, 42);
}

TEST(AdcMedian, AlreadySortedArray) {
    int arr[5] = {1, 2, 3, 4, 5};
    int median = adc::computeMedianOfFive(arr);
    EXPECT_EQ(median, 3);
}

TEST(AdcMedian, ReverseSortedArray) {
    int arr[5] = {5, 4, 3, 2, 1};
    int median = adc::computeMedianOfFive(arr);
    EXPECT_EQ(median, 3);
}

// --- ADC raw to millivolts ---

TEST(AdcRawToMv, KnownConversion) {
    // Half of ADC range should be half of supply voltage
    int mv = adc::rawToMillivolts(2048, 3300, 4096);
    EXPECT_EQ(mv, 1650);  // 2048/4096 * 3300
}

TEST(AdcRawToMv, ZeroRawReturnsZeroMv) {
    int mv = adc::rawToMillivolts(0, 3300, 4096);
    EXPECT_EQ(mv, 0);
}

TEST(AdcRawToMv, MaxRawReturnsSupplyVoltage) {
    int mv = adc::rawToMillivolts(4095, 3300, 4096);
    // Should be very close to 3300 (3300 * 4095 / 4096 = 3299)
    EXPECT_GE(mv, 3298);
    EXPECT_LE(mv, 3300);
}
