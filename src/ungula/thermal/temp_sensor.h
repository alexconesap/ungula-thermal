// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

namespace ungula::thermal
{

    struct NtcConfig {
        double seriesResistorOhms;
        double nominalResistanceOhms;
        double betaCoefficient;
        double referenceTempC;
        uint16_t supplyVoltageMv;
        double disconnectedMarginMv;
        double minReasonableResistanceOhms;
        double maxReasonableResistanceOhms;
        double maxValidTempC;
        double minValidTempC;
    };

    double ntcMillivoltsToTempC(int millivolts, const NtcConfig &cfg);
    double applyCalibrationOffset(double rawTempC, double offsetC);

    class TemperatureSensor {
    public:
        TemperatureSensor(uint8_t channelIndex, int adcPin, double calibrationOffsetC, const NtcConfig &ntcCfg);

        void setCalibrationOffset(double offsetC)
        {
            calibrationOffset_ = offsetC;
        }
        double getCalibrationOffset() const
        {
            return calibrationOffset_;
        }
        void setNtcConfig(const NtcConfig &cfg)
        {
            ntcConfig_ = cfg;
        }

        double readTemperatureC(int adcMillivolts);
        bool isConnected() const;

        double getLastRawTempC() const
        {
            return lastRawTempC_;
        }
        double getLastCalibratedTempC() const
        {
            return lastCalibratedTempC_;
        }
        int getLastAdcMillivolts() const
        {
            return lastAdcMv_;
        }
        uint8_t getChannelIndex() const
        {
            return channelIndex_;
        }
        int getAdcPin() const
        {
            return adcPin_;
        }

    private:
        uint8_t channelIndex_;
        int adcPin_;
        double calibrationOffset_;
        NtcConfig ntcConfig_;
        double lastRawTempC_;
        double lastCalibratedTempC_;
        int lastAdcMv_;
    };

} // namespace ungula::thermal

namespace ungula::thermal::adc
{

    void sortFiveElements(int arr[5]);
    int computeMedianOfFive(int arr[5]);

    template <typename ReadFunc> int readWithMedianFiltering(ReadFunc readOnce, uint8_t totalSamples, uint8_t groupSize)
    {
        int groups = (totalSamples > groupSize) ? (totalSamples / groupSize) : 1;
        long sum = 0;

        for (int g = 0; g < groups; ++g) {
            int buffer[5] = { 0, 0, 0, 0, 0 };
            for (int i = 0; i < groupSize && i < 5; ++i) {
                buffer[i] = readOnce();
            }
            sortFiveElements(buffer);
            sum += buffer[2];
        }

        return static_cast<int>(sum / groups);
    }

    template <typename ReadFunc> int readWithAveraging(ReadFunc readOnce, uint8_t sampleCount)
    {
        long sum = 0;
        for (int i = 0; i < sampleCount; ++i) {
            sum += readOnce();
        }
        return static_cast<int>(sum / sampleCount);
    }

    inline int rawToMillivolts(int rawValue, uint16_t supplyMv, uint16_t adcResolution)
    {
        return (rawValue * supplyMv) / adcResolution;
    }

} // namespace ungula::thermal::adc
