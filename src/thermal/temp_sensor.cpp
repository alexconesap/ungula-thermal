// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "temp_sensor.h"

#include <cmath>

namespace ungula {
    namespace thermal {

        double ntcMillivoltsToTempC(int millivolts, const NtcConfig& cfg) {
            double vm = static_cast<double>(millivolts);
            double vcc = static_cast<double>(cfg.supplyVoltageMv);

            if (vm < cfg.disconnectedMarginMv) {
                return NAN;
            }

            double denominator = vcc - vm;
            if (denominator <= 0.0) {
                return NAN;
            }

            double rNtc = cfg.seriesResistorOhms * (vm / denominator);

            if (!std::isfinite(rNtc) || rNtc < cfg.minReasonableResistanceOhms || rNtc > cfg.maxReasonableResistanceOhms) {
                return NAN;
            }

            constexpr double KELVIN_OFFSET = 273.15;
            double t0Kelvin = cfg.referenceTempC + KELVIN_OFFSET;

            double lnRatio = std::log(rNtc / cfg.nominalResistanceOhms);
            double inverseT = (1.0 / t0Kelvin) + (1.0 / cfg.betaCoefficient) * lnRatio;

            if (!std::isfinite(inverseT) || inverseT <= 0.0) {
                return NAN;
            }

            double tempKelvin = 1.0 / inverseT;
            double tempC = tempKelvin - KELVIN_OFFSET;

            // Reject physically impossible readings (floating/disconnected pin noise)
            if (tempC > cfg.maxValidTempC || tempC < cfg.minValidTempC) {
                return NAN;
            }

            return tempC;
        }

        double applyCalibrationOffset(double rawTempC, double offsetC) {
            if (!std::isfinite(rawTempC)) {
                return NAN;
            }
            return rawTempC + offsetC;
        }

        TemperatureSensor::TemperatureSensor(uint8_t channelIndex, int adcPin, double calibrationOffsetC, const NtcConfig& ntcCfg)
            : channelIndex_(channelIndex),
              adcPin_(adcPin),
              calibrationOffset_(calibrationOffsetC),
              ntcConfig_(ntcCfg),
              lastRawTempC_(NAN),
              lastCalibratedTempC_(NAN),
              lastAdcMv_(0) {
        }

        double TemperatureSensor::readTemperatureC(int adcMillivolts) {
            lastAdcMv_ = adcMillivolts;
            lastRawTempC_ = ntcMillivoltsToTempC(adcMillivolts, ntcConfig_);
            lastCalibratedTempC_ = applyCalibrationOffset(lastRawTempC_, calibrationOffset_);
            return lastCalibratedTempC_;
        }

        bool TemperatureSensor::isConnected() const {
            return std::isfinite(lastCalibratedTempC_);
        }

        namespace adc {

            void sortFiveElements(int arr[5]) {
                for (int i = 0; i < 5; ++i) {
                    for (int j = i + 1; j < 5; ++j) {
                        if (arr[j] < arr[i]) {
                            int temp = arr[i];
                            arr[i] = arr[j];
                            arr[j] = temp;
                        }
                    }
                }
            }

            int computeMedianOfFive(int arr[5]) {
                sortFiveElements(arr);
                return arr[2];
            }

        }  // namespace adc

    }  // namespace thermal
}  // namespace ungula
