// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

#include "pid_controller.h"
#include "temp_sensor.h"
#include "ungula/core/util/types.h"

namespace ungula::thermal
{

struct GovernorConfig {
        double rampUpFraction;
        double rampDownFraction;
        uint32_t updatePeriodMs;
        uint16_t pwmResolution;
};

struct RateLimiterConfig {
        double thresholdCps;
        double maxRateCps;
        double outputReductionFactor;
        uint16_t pwmResolution;
};

struct DutyFloorConfig {
        double atSetpointLow;
        double atSetpointHigh;
        double belowSpCloseLow;
        double belowSpCloseHigh;
        double belowSpFarLow;
        double belowSpFarHigh;
        double aboveSpBaseOffset;
        double aboveSpMinimum;
        double aboveSpAtMaxError;
        double errorThresholdFarC;
        double errorThresholdMidC;
        double errorThresholdCloseC;
        double holdBandC;
        double setpointMinC; // minimum expected setpoint in °C (used to normalise the
        // setpoint curve)
        double setpointMaxC; // maximum expected setpoint in °C
        uint16_t pwmResolution;
};

struct HeaterChannelConfig {
        PidConfig pid;
        DutyFloorConfig floor;
        GovernorConfig governor;
        RateLimiterConfig rateLimiter;
        NtcConfig ntc;
};

enum class FloorReason : uint8_t {
        None = 0,
        BelowSetpoint = 1,
        AboveSetpoint = 2,
        HoldBand = 3,
        RateLimited = 4
};

struct HeaterChannelState {
        double currentDuty;
        double targetDuty;
        double floorDuty;
        FloorReason floorReason;
        double temperatureC;
        double setpointC;
        double derivativeCps;
        bool rateLimitActive;
        uint32_t lastGovernorUpdateMs;
};

class DutyGovernor {
    public:
        explicit DutyGovernor(const GovernorConfig &cfg);
        void setConfig(const GovernorConfig &cfg)
        {
                config_ = cfg;
        }
        void reset(double initialDuty = 0.0);
        double update(double targetDuty, uint32_t nowMs);
        double getCurrentDuty() const
        {
                return currentDuty_;
        }

    private:
        GovernorConfig config_;
        double currentDuty_;
        uint32_t lastUpdateMs_;
        bool initialized_;
};

class DutyFloorCalculator {
    public:
        explicit DutyFloorCalculator(const DutyFloorConfig &cfg);
        double computeFloor(double tempC, double setpointC, FloorReason &reasonOut) const;

    private:
        double normalizeSetpoint(double setpointC) const;
        double computeFloorBelowSetpoint(double errorC, double setpointC) const;
        double computeFloorAboveSetpoint(double errorAboveC, double setpointC) const;
        double computeHoldFloor(double setpointC) const;

        DutyFloorConfig config_;
};

class RateLimiter {
    public:
        explicit RateLimiter(const RateLimiterConfig &cfg);
        void setConfig(const RateLimiterConfig &cfg)
        {
                config_ = cfg;
        }
        double applyLimit(double targetDuty, double derivativeCps, bool &limitActiveOut) const;

    private:
        RateLimiterConfig config_;
};

class HeaterChannel {
    public:
        HeaterChannel(uint8_t index, int thermistorPin, int heaterPin, double calibrationOffsetC,
                      const HeaterChannelConfig &cfg);

        void setSetpointC(double setpointC)
        {
                setpointC_ = setpointC;
        }
        void setSetpointTrimC(double trimC)
        {
                setpointTrimC_ = trimC;
        }
        double getEffectiveSetpointC() const
        {
                return setpointC_ + setpointTrimC_;
        }
        void setDerivativeFilterAlpha(double alpha)
        {
                pid_.setDerivativeFilterAlpha(alpha);
        }
        void setApproachGains(const PidGains &g)
        {
                pid_.setGains(g, holdGains_);
                approachGains_ = g;
        }
        void setHoldGains(const PidGains &g)
        {
                pid_.setGains(approachGains_, g);
                holdGains_ = g;
        }
        void setPidGains(const PidGains &approach, const PidGains &hold)
        {
                approachGains_ = approach;
                holdGains_ = hold;
                pid_.setGains(approach, hold);
        }

        HeaterChannelState update(int adcMillivolts, uint32_t nowMs, double dtSeconds);
        void resetControl();
        void initializeBumpless(double currentDuty);

        double getTemperatureC() const
        {
                return sensor_.getLastCalibratedTempC();
        }
        double getTemperatureF() const;
        bool isSensorConnected() const
        {
                return sensor_.isConnected();
        }
        uint16_t getDutyPwm() const
        {
                return static_cast<uint16_t>(governor_.getCurrentDuty());
        }
        const HeaterChannelState &getState() const
        {
                return state_;
        }
        uint8_t getIndex() const
        {
                return index_;
        }
        int getHeaterPin() const
        {
                return heaterPin_;
        }

    private:
        uint8_t index_;
        int heaterPin_;
        double setpointC_;
        double setpointTrimC_;

        PidGains approachGains_;
        PidGains holdGains_;

        TemperatureSensor sensor_;
        PidController pid_;
        DutyGovernor governor_;
        DutyFloorCalculator floorCalc_;
        RateLimiter rateLimiter_;
        HeaterChannelState state_;
};

} // namespace ungula::thermal
