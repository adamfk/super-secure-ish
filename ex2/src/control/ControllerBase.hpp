#pragma once

class ControllerBase {
public:
    ControllerBase() = default;
    virtual ~ControllerBase() = default;

    virtual void clear_countdown() = 0;
    virtual void init_arming_countdown() = 0;
    virtual void init_alarm_countdown() = 0;

    virtual bool is_countdown_complete() const = 0;
    virtual bool is_motion_detected() const = 0;

    virtual void notify_disarmed() = 0;
    virtual void notify_armed() = 0;
    virtual void notify_alarm() = 0;
    virtual void notify_event_ignored() = 0;

    virtual void set_red_led(bool on) = 0;
};
