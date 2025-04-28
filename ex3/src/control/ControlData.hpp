#pragma once
#include "ControllerSm.hpp"
#include <stdint.h>

// fields in this structure are set by control system, cleared by UI.
struct ControlEvents {
    bool armed = false;
    bool alarm = false;
    bool disarmed = false;

    // example: arm/disarm attempt while already in that state
    bool event_ignored = false;
};

struct ControlData {
    ControllerSm::StateId state_id = ControllerSm::StateId::DISARMED;
    bool motion_detected = false;
    int32_t countdown_ms = 0; // larger than int16 so that it can extend past ~30 seconds
};
