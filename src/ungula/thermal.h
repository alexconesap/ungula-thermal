// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once
#ifndef __cplusplus
#error UngulaThermal requires a C++ compiler
#endif

// Ungula Thermal Library - thermal control for embedded projects
// Include this header to activate the library in Arduino

// (No dependency on UngulaCore — thermal code uses only its own
// types and the C++ standard library. Libraries do not log directly.)

// Thermal control
#include "ungula/thermal/fan_control.h"
#include "ungula/thermal/heater_control.h"
#include "ungula/thermal/pid_controller.h"
#include "ungula/thermal/temp_sensor.h"
