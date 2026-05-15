// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "fan_control.h"

namespace ungula::thermal
{

FanPwmCalculator::FanPwmCalculator(const FanOutputConfig &config)
        : config_(config)
{
}

uint16_t FanPwmCalculator::calculateDuty(int speedPercent) const
{
        if (speedPercent < 0)
                speedPercent = 0;
        if (speedPercent > 100)
                speedPercent = 100;

        uint16_t duty;
        if (config_.invertedOutput) {
                duty = static_cast<uint16_t>(config_.pwmResolution -
                                             (speedPercent * config_.pwmResolution / 100));
        } else {
                duty = static_cast<uint16_t>(speedPercent * config_.pwmResolution / 100);
        }
        return duty;
}

} // namespace ungula::thermal
