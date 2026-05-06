// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "heater_control.h"

#include <ungula/core/util/types.h>
#include <cmath>

using namespace ungula::core::util::math;

namespace ungula::thermal {


// Safe mid-range default so the PID has a sane reference before the first
// real setpoint arrives, avoiding full-power output at startup.
static constexpr double kDefaultSafeSetpointC = 290.0;

// ---- DutyGovernor ----

DutyGovernor::DutyGovernor(const GovernorConfig& cfg)
    : config_(cfg), currentDuty_(0.0), lastUpdateMs_(0), initialized_(false) {}

void DutyGovernor::reset(double initialDuty) {
    currentDuty_ = clamp(initialDuty, 0.0, static_cast<double>(config_.pwmResolution));
    lastUpdateMs_ = 0;
    initialized_ = false;
}

double DutyGovernor::update(double targetDuty, uint32_t nowMs) {
    targetDuty = clamp(targetDuty, 0.0, static_cast<double>(config_.pwmResolution));

    if (!initialized_) {
        currentDuty_ = targetDuty;
        lastUpdateMs_ = nowMs;
        initialized_ = true;
        return currentDuty_;
    }

    if (nowMs - lastUpdateMs_ < config_.updatePeriodMs) {
        return currentDuty_;
    }
    lastUpdateMs_ = nowMs;

    double delta = targetDuty - currentDuty_;
    double maxStepUp = config_.rampUpFraction * config_.pwmResolution;
    double maxStepDown = config_.rampDownFraction * config_.pwmResolution;

    if (delta > maxStepUp) {
        delta = maxStepUp;
    } else if (delta < -maxStepDown) {
        delta = -maxStepDown;
    }

    currentDuty_ += delta;
    currentDuty_ = clamp(currentDuty_, 0.0, static_cast<double>(config_.pwmResolution));

    return currentDuty_;
}

// ---- DutyFloorCalculator ----

DutyFloorCalculator::DutyFloorCalculator(const DutyFloorConfig& cfg) : config_(cfg) {}

double DutyFloorCalculator::normalizeSetpoint(double setpointC) const {
    double range = config_.setpointMaxC - config_.setpointMinC;
    if (range <= 0.0) {
        return 0.5;
    }
    double t = (setpointC - config_.setpointMinC) / range;
    return t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
}

double DutyFloorCalculator::computeFloor(double tempC, double setpointC,
                                         FloorReason& reasonOut) const {
    reasonOut = FloorReason::None;
    double maxFloor = 0.0;

    bool belowSetpoint = (tempC < setpointC);
    double error = setpointC - tempC;

    if (belowSetpoint) {
        double floorBelow = computeFloorBelowSetpoint(error, setpointC);
        if (floorBelow > maxFloor) {
            maxFloor = floorBelow;
            reasonOut = FloorReason::BelowSetpoint;
        }
    } else {
        double errorAbove = tempC - setpointC;
        if (errorAbove <= config_.errorThresholdFarC) {
            double floorAbove = computeFloorAboveSetpoint(errorAbove, setpointC);
            if (floorAbove > maxFloor) {
                maxFloor = floorAbove;
                reasonOut = FloorReason::AboveSetpoint;
            }
        }
    }

    if (belowSetpoint && std::fabs(error) <= config_.holdBandC) {
        double holdFloor = computeHoldFloor(setpointC);
        if (holdFloor > maxFloor) {
            maxFloor = holdFloor;
            reasonOut = FloorReason::HoldBand;
        }
    }

    return maxFloor * config_.pwmResolution;
}

double DutyFloorCalculator::computeFloorBelowSetpoint(double errorC,
                                                      double setpointC) const {
    double spNorm = normalizeSetpoint(setpointC);

    double floorFar = lerp(config_.belowSpFarLow, config_.belowSpFarHigh, spNorm);
    double floorClose = lerp(config_.belowSpCloseLow, config_.belowSpCloseHigh, spNorm);

    if (errorC >= config_.errorThresholdFarC) {
        return 1.0;
    }

    if (errorC >= config_.errorThresholdMidC) {
        double denom = config_.errorThresholdFarC - config_.errorThresholdMidC;
        if (denom <= 0.0) {
            return floorFar;
        }
        double t = (errorC - config_.errorThresholdMidC) / denom;
        return lerp(floorFar, 1.0, t);
    }

    double effectiveError = errorC;
    if (effectiveError < config_.errorThresholdCloseC) {
        effectiveError = config_.errorThresholdCloseC;
    }
    double denom = config_.errorThresholdMidC - config_.errorThresholdCloseC;
    if (denom <= 0.0) {
        return floorClose;
    }
    double t = (effectiveError - config_.errorThresholdCloseC) / denom;
    return lerp(floorClose, floorFar, t);
}

double DutyFloorCalculator::computeFloorAboveSetpoint(double errorAboveC,
                                                      double setpointC) const {
    double spNorm = normalizeSetpoint(setpointC);
    double holdFloor = lerp(config_.atSetpointLow, config_.atSetpointHigh, spNorm);

    double baseFloor = holdFloor - config_.aboveSpBaseOffset;
    if (baseFloor < config_.aboveSpMinimum) {
        baseFloor = config_.aboveSpMinimum;
    }

    if (errorAboveC >= config_.errorThresholdFarC || config_.errorThresholdFarC <= 0.0) {
        return config_.aboveSpAtMaxError;
    }

    double t = errorAboveC / config_.errorThresholdFarC;
    return lerp(baseFloor, config_.aboveSpAtMaxError, t);
}

double DutyFloorCalculator::computeHoldFloor(double setpointC) const {
    double spNorm = normalizeSetpoint(setpointC);
    return lerp(config_.atSetpointLow, config_.atSetpointHigh, spNorm);
}

// ---- RateLimiter ----

RateLimiter::RateLimiter(const RateLimiterConfig& cfg) : config_(cfg) {}

double RateLimiter::applyLimit(double targetDuty, double derivativeCps,
                               bool& limitActiveOut) const {
    limitActiveOut = false;

    if (derivativeCps <= config_.thresholdCps) {
        return targetDuty;
    }

    limitActiveOut = true;

    double excessRate = derivativeCps - config_.thresholdCps;
    double maxExcess = config_.maxRateCps - config_.thresholdCps;
    if (maxExcess <= 0.0) {
        maxExcess = 1.0;
    }

    double reductionFactor = excessRate / maxExcess;
    reductionFactor = clamp01(reductionFactor);

    double reduction = config_.outputReductionFactor * reductionFactor;
    double limitedDuty = targetDuty * (1.0 - reduction);

    return clamp(limitedDuty, 0.0, static_cast<double>(config_.pwmResolution));
}

// ---- HeaterChannel ----

HeaterChannel::HeaterChannel(uint8_t index, int thermistorPin, int heaterPin,
                             double calibrationOffsetC, const HeaterChannelConfig& cfg)
    : index_(index),
      heaterPin_(heaterPin),
      setpointC_(kDefaultSafeSetpointC),
      setpointTrimC_(0.0),
      approachGains_(cfg.pid.approachGains),
      holdGains_(cfg.pid.holdGains),
      sensor_(index, thermistorPin, calibrationOffsetC, cfg.ntc),
      pid_(cfg.pid),
      governor_(cfg.governor),
      floorCalc_(cfg.floor),
      rateLimiter_(cfg.rateLimiter),
      state_{} {}

void HeaterChannel::resetControl() {
    pid_.reset();
    governor_.reset(0.0);
    state_ = {};
}

void HeaterChannel::initializeBumpless(double currentDuty) {
    double tempC = sensor_.getLastCalibratedTempC();
    if (ungula::core::util::temp::isValidTemperature(tempC)) {
        pid_.initializeForBumplessTransfer(currentDuty, tempC, getEffectiveSetpointC());
        governor_.reset(currentDuty);
    }
}

double HeaterChannel::getTemperatureF() const {
    return ungula::core::util::temp::celsiusToFahrenheit(getTemperatureC());
}

HeaterChannelState HeaterChannel::update(int adcMillivolts, uint32_t nowMs,
                                         double dtSeconds) {
    double tempC = sensor_.readTemperatureC(adcMillivolts);
    double setpoint = getEffectiveSetpointC();

    state_.temperatureC = tempC;
    state_.setpointC = setpoint;

    if (!ungula::core::util::temp::isValidTemperature(tempC)) {
        resetControl();
        state_.currentDuty = 0.0;
        state_.targetDuty = 0.0;
        state_.floorDuty = 0.0;
        state_.floorReason = FloorReason::None;
        state_.derivativeCps = 0.0;
        state_.rateLimitActive = false;
        return state_;
    }

    PidOutput pidOut = pid_.compute(tempC, setpoint, dtSeconds);
    state_.derivativeCps = pidOut.derivativeCps;

    bool rateLimited = false;
    double rateLimitedOutput =
            rateLimiter_.applyLimit(pidOut.rawOutput, pidOut.derivativeCps, rateLimited);
    state_.rateLimitActive = rateLimited;

    FloorReason floorReason;
    double floorDuty = floorCalc_.computeFloor(tempC, setpoint, floorReason);
    state_.floorDuty = floorDuty;
    state_.floorReason = floorReason;

    double targetDuty = rateLimitedOutput;
    if (targetDuty < floorDuty) {
        targetDuty = floorDuty;
    }
    state_.targetDuty = targetDuty;

    double governedDuty = governor_.update(targetDuty, nowMs);
    state_.currentDuty = governedDuty;
    state_.lastGovernorUpdateMs = nowMs;

    return state_;
}

    }  // namespace ungula::thermal
