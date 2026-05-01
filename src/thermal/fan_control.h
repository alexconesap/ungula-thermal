// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>

namespace ungula {
    namespace thermal {

        struct FanTachConfig {
                uint8_t pulsesPerRev;
                uint16_t minRpmThreshold;
                uint32_t samplePeriodMs;
                uint32_t minEdgeIntervalUs;
        };

        struct FanChannelState {
                uint32_t pulseCount;
                uint32_t lastEdgeUs;
                uint16_t rpm;
                bool connected;

                FanChannelState() : pulseCount(0), lastEdgeUs(0), rpm(0), connected(false) {}
        };

        struct FanOutputConfig {
                uint16_t pwmResolution;
                bool invertedOutput;
        };

        // ---- FanTachometer ----
        // Tracks RPM and connectivity for N fan channels via ISR-driven pulse counting.

        template <uint8_t N>
        class FanTachometer {
            public:
                static constexpr uint8_t NUM_CHANNELS = N;

                explicit FanTachometer(const FanTachConfig& cfg) : config_(cfg) {
                    reset();
                }

                bool recordPulse(uint8_t channel, uint32_t timestampUs) {
                    if (channel >= N)
                        return false;

                    FanChannelState& ch = channels_[channel];
                    uint32_t elapsed = timestampUs - ch.lastEdgeUs;
                    if (elapsed < config_.minEdgeIntervalUs)
                        return false;

                    ch.pulseCount++;
                    ch.lastEdgeUs = timestampUs;
                    return true;
                }

                void update(bool commandedOff) {
                    for (uint8_t i = 0; i < N; ++i) {
                        FanChannelState& ch = channels_[i];
                        uint8_t ppr = (config_.pulsesPerRev > 0) ? config_.pulsesPerRev : 1;
                        ch.rpm = static_cast<uint16_t>((ch.pulseCount * 60) / ppr);
                        ch.connected = (!commandedOff) && (ch.rpm > config_.minRpmThreshold);
                        ch.pulseCount = 0;
                    }
                }

                const FanChannelState& getChannelState(uint8_t channel) const {
                    static const FanChannelState emptyState;
                    if (channel >= N)
                        return emptyState;
                    return channels_[channel];
                }

                void getConnectedStatus(bool* outConnected) const {
                    for (uint8_t i = 0; i < N; ++i) {
                        outConnected[i] = channels_[i].connected;
                    }
                }

                void getRpmValues(uint16_t* outRpm) const {
                    for (uint8_t i = 0; i < N; ++i) {
                        outRpm[i] = channels_[i].rpm;
                    }
                }

                void reset() {
                    for (uint8_t i = 0; i < N; ++i) {
                        channels_[i] = FanChannelState();
                    }
                }

                uint32_t getPulseCount(uint8_t channel) const {
                    if (channel >= N)
                        return 0;
                    return channels_[channel].pulseCount;
                }

            private:
                FanTachConfig config_;
                FanChannelState channels_[N];
        };

        // ---- FanPwmCalculator ----
        // Converts speed percentage (0-100) to PWM duty value.

        class FanPwmCalculator {
            public:
                explicit FanPwmCalculator(const FanOutputConfig& config);
                uint16_t calculateDuty(int speedPercent) const;
                const FanOutputConfig& getConfig() const {
                    return config_;
                }

            private:
                FanOutputConfig config_;
        };

        // ---- FanController ----
        // Combines tachometer + PWM calculator into a single fan management unit for N channels.

        template <uint8_t N>
        class FanController {
            public:
                static constexpr uint8_t NUM_CHANNELS = N;

                FanController(const FanTachConfig& tachCfg, const FanOutputConfig& outCfg)
                    : tachometer_(tachCfg),
                      pwmCalculator_(outCfg),
                      samplePeriodMs_(tachCfg.samplePeriodMs),
                      lastUpdateMs_(0) {}

                bool recordPulse(uint8_t channel, uint32_t timestampUs) {
                    return tachometer_.recordPulse(channel, timestampUs);
                }

                bool update(uint32_t nowMs, int speedCommandPercent) {
                    if (nowMs - lastUpdateMs_ < samplePeriodMs_)
                        return false;
                    lastUpdateMs_ = nowMs;
                    bool commandedOff = (speedCommandPercent <= 0);
                    tachometer_.update(commandedOff);
                    return true;
                }

                uint16_t calculateOutputDuty(int speedPercent) const {
                    return pwmCalculator_.calculateDuty(speedPercent);
                }

                const FanChannelState& getChannelState(uint8_t channel) const {
                    return tachometer_.getChannelState(channel);
                }

                void getConnectedStatus(bool* outConnected) const {
                    tachometer_.getConnectedStatus(outConnected);
                }

                bool isAnyFanConnected() const {
                    for (uint8_t i = 0; i < N; ++i) {
                        if (tachometer_.getChannelState(i).connected)
                            return true;
                    }
                    return false;
                }

                bool areAllFansConnected() const {
                    for (uint8_t i = 0; i < N; ++i) {
                        if (!tachometer_.getChannelState(i).connected)
                            return false;
                    }
                    return true;
                }

                void reset() {
                    tachometer_.reset();
                    lastUpdateMs_ = 0;
                }

            private:
                FanTachometer<N> tachometer_;
                FanPwmCalculator pwmCalculator_;
                uint32_t samplePeriodMs_;
                uint32_t lastUpdateMs_;
        };

    }  // namespace thermal
}  // namespace ungula
