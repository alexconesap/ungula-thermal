// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

namespace ungula::thermal {

    struct PidGains {
            double kp;
            double ki;
            double kd;
    };

    struct PidConfig {
            PidGains approachGains;
            PidGains holdGains;
            double derivativeFilterAlpha;
            double derivativeFilterAlphaMin;
            double derivativeFilterAlphaMax;
            double setpointApproachThresholdC;
            double setpointApproachOffsetC;
            double gainSwitchBandC;
            double derivativeEnableMarginC;
            double kiReductionBandNarrowC;
            double kiReductionBandWideC;
            double kiReductionFactorNarrow;
            double kiReductionFactorWide;
            double integralLimit;
            double antiwindupGain;
            double hysteresisAboveSetpointC;
            double hysteresisBelowSetpointC;
            double outputMax;
    };

    struct PidOutput {
            double rawOutput;
            double pTerm;
            double iTerm;
            double dTerm;
            double derivativeCps;
            double effectiveSetpoint;
            bool inHoldMode;
    };

    class PidController {
        public:
            explicit PidController(const PidConfig& config);

            void reset();
            void setGains(const PidGains& approach, const PidGains& hold);
            void setDerivativeFilterAlpha(double alpha);
            double getDerivativeFilterAlpha() const {
                return derivativeAlpha_;
            }
            void setOutputLimits(double minOutput, double maxOutput);
            void initializeForBumplessTransfer(double currentOutput, double currentTemp,
                                               double setpoint);
            PidOutput compute(double processValue, double setpoint, double dtSeconds);

            double getIntegral() const {
                return integral_;
            }
            double getFilteredTemperature() const {
                return filteredTemp_;
            }
            double getLastDerivative() const {
                return lastDerivativeCps_;
            }
            bool hasReachedSetpoint() const {
                return reachedSetpointLatch_;
            }

        private:
            double computeEffectiveSetpoint(double processValue, double setpoint) const;
            PidGains selectGains(double error) const;
            double applyKiReduction(double ki, double error) const;

            PidConfig config_;

            PidGains approachGains_;
            PidGains holdGains_;

            double integral_;
            double filteredTemp_;
            double lastFilteredTemp_;
            double lastDerivativeCps_;

            double derivativeAlpha_;
            double outputMin_;
            double outputMax_;

            bool initialized_;
            bool reachedSetpointLatch_;
    };

}  // namespace ungula::thermal
