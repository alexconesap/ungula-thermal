// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "pid_controller.h"

#include <cmath>

namespace ungula::thermal
{

static inline double clamp(double v, double lo, double hi)
{
        return v < lo ? lo : (v > hi ? hi : v);
}

static inline bool isValidTemp(double t)
{
        return std::isfinite(t) && t > -200.0 && t < 1800.0;
}

PidController::PidController(const PidConfig &config)
        : config_(config)
        , integral_(0.0)
        , filteredTemp_(NAN)
        , lastFilteredTemp_(NAN)
        , lastDerivativeCps_(0.0)
        , derivativeAlpha_(config.derivativeFilterAlpha)
        , outputMin_(0.0)
        , outputMax_(config.outputMax)
        , initialized_(false)
        , reachedSetpointLatch_(false)
{
        approachGains_ = config.approachGains;
        holdGains_ = config.holdGains;
}

void PidController::reset()
{
        integral_ = 0.0;
        filteredTemp_ = NAN;
        lastFilteredTemp_ = NAN;
        lastDerivativeCps_ = 0.0;
        initialized_ = false;
        reachedSetpointLatch_ = false;
}

void PidController::setGains(const PidGains &approach, const PidGains &hold)
{
        approachGains_ = approach;
        holdGains_ = hold;
}

void PidController::setDerivativeFilterAlpha(double alpha)
{
        derivativeAlpha_ =
            clamp(alpha, config_.derivativeFilterAlphaMin, config_.derivativeFilterAlphaMax);
}

void PidController::setOutputLimits(double minOutput, double maxOutput)
{
        outputMin_ = minOutput;
        outputMax_ = maxOutput;
}

void PidController::initializeForBumplessTransfer(double currentOutput, double currentTemp,
                                                  double setpoint)
{
        filteredTemp_ = currentTemp;
        lastFilteredTemp_ = currentTemp;
        lastDerivativeCps_ = 0.0;

        double error = setpoint - currentTemp;
        PidGains gains = selectGains(error);
        double kiEffective = applyKiReduction(gains.ki, error);

        if (std::fabs(kiEffective) > 1e-9) {
                integral_ = currentOutput / kiEffective;
                integral_ = clamp(integral_, -config_.integralLimit, config_.integralLimit);
        } else {
                integral_ = 0.0;
        }

        initialized_ = true;
}

double PidController::computeEffectiveSetpoint(double processValue, double setpoint) const
{
        bool farBelowSetpoint = (processValue < setpoint - config_.setpointApproachThresholdC);
        bool notYetReached = !reachedSetpointLatch_;

        if (farBelowSetpoint && notYetReached) {
                return setpoint + config_.setpointApproachOffsetC;
        }
        return setpoint;
}

PidGains PidController::selectGains(double error) const
{
        bool inHoldBand = std::fabs(error) <= config_.gainSwitchBandC;
        return inHoldBand ? holdGains_ : approachGains_;
}

double PidController::applyKiReduction(double ki, double error) const
{
        double absError = std::fabs(error);

        if (absError < config_.kiReductionBandNarrowC) {
                return ki * config_.kiReductionFactorNarrow;
        } else if (absError < config_.kiReductionBandWideC) {
                return ki * config_.kiReductionFactorWide;
        }
        return ki;
}

PidOutput PidController::compute(double processValue, double setpoint, double dtSeconds)
{
        PidOutput result = {};

        if (!isValidTemp(processValue)) {
                reset();
                return result;
        }

        if (dtSeconds <= 0.0) {
                dtSeconds = 0.001;
        }

        if (processValue >= setpoint + config_.hysteresisAboveSetpointC) {
                reachedSetpointLatch_ = true;
        }
        if (processValue <= setpoint - config_.hysteresisBelowSetpointC) {
                reachedSetpointLatch_ = false;
        }

        result.effectiveSetpoint = computeEffectiveSetpoint(processValue, setpoint);
        double error = result.effectiveSetpoint - processValue;

        if (std::isnan(filteredTemp_)) {
                filteredTemp_ = processValue;
        }
        filteredTemp_ = filteredTemp_ + derivativeAlpha_ * (processValue - filteredTemp_);

        if (std::isnan(lastFilteredTemp_)) {
                lastFilteredTemp_ = filteredTemp_;
        }
        double dTemp = (filteredTemp_ - lastFilteredTemp_) / dtSeconds;
        lastFilteredTemp_ = filteredTemp_;
        lastDerivativeCps_ = dTemp;
        result.derivativeCps = dTemp;

        PidGains gains = selectGains(error);
        result.inHoldMode = (std::fabs(error) <= config_.gainSwitchBandC);

        result.pTerm = gains.kp * error;

        double kiEffective = applyKiReduction(gains.ki, error);
        integral_ += error * dtSeconds;
        integral_ = clamp(integral_, -config_.integralLimit, config_.integralLimit);
        result.iTerm = kiEffective * integral_;

        bool shouldApplyDerivative =
            (processValue >= result.effectiveSetpoint) ||
            (std::fabs(error) <= config_.gainSwitchBandC + config_.derivativeEnableMarginC);
        result.dTerm = shouldApplyDerivative ? (gains.kd * (-dTemp)) : 0.0;

        double output = result.pTerm + result.iTerm + result.dTerm;

        double outputSaturated = clamp(output, outputMin_, outputMax_);

        if (output != outputSaturated) {
                double antiwindupCorrection =
                    (outputSaturated - output) * config_.antiwindupGain * dtSeconds;
                integral_ += antiwindupCorrection;
                integral_ = clamp(integral_, -config_.integralLimit, config_.integralLimit);
        }

        result.rawOutput = outputSaturated;
        initialized_ = true;

        return result;
}

} // namespace ungula::thermal
