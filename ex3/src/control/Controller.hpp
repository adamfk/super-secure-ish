#pragma once
#include "../io_config.hpp"
#include "ControlData.hpp"
#include "ControllerSm.hpp"
#include <Arduino.h>
#include <stdint.h>

class Controller : protected ControllerSm {
public:
    Controller()
    {
    }

    void setup()
    {
        pinMode(IO_PIN_RED_LED, OUTPUT);
        pinMode(IO_PIN_MOTION_SENSOR, INPUT);

        start(); // start the state machine
    }

    void update(const uint32_t loop_delta_ms)
    {
        m_control_data.motion_detected = digitalRead(IO_PIN_MOTION_SENSOR) == HIGH;

        if (m_countdown_ms > 0) {
            m_countdown_ms -= loop_delta_ms;
        }

        dispatch_fsm_event(ControllerSm::EventId::DO);
    }

    void panic()
    {
        dispatch_fsm_event(ControllerSm::EventId::ALARM_NOW);
    }

    void arm()
    {
        dispatch_fsm_event(ControllerSm::EventId::ARM);
    }

    void disarm()
    {
        dispatch_fsm_event(ControllerSm::EventId::DISARM);
    }

    const ControlData& get_control_data()
    {
        return m_control_data;
    }

    ControlEvents& get_published_events()
    {
        return m_published_events;
    }

protected:
    int16_t m_countdown_ms = 0;
    ControlData m_control_data {};
    ControlEvents m_published_events {}; // set by control system, cleared by UI.

    void dispatch_fsm_event(ControllerSm::EventId eventId)
    {
        // print event name to console. Not the periodic DO event though. Too noisy.
        if (eventId != ControllerSm::EventId::DO) {
            Serial.print(F("CONTROL dispatching event: "));
            Serial.println(ControllerSm::eventIdToString(eventId));
        }

        const ControllerSm::StateId prev_state = stateId;
        dispatchEvent(eventId); // this method is inherited from ControllerSm
        const ControllerSm::StateId current_state = stateId;

        m_handle_state_change(prev_state, current_state);

        // publish data
        m_control_data.countdown_ms = m_countdown_ms;
        m_control_data.state_id = current_state;
    }

    void clear_countdown() override
    {
        m_countdown_ms = 0;
    }

    void init_arming_countdown() override
    {
        m_countdown_ms = 5 * 1000;
    }

    void init_alarm_countdown() override
    {
        m_countdown_ms = 5 * 1000;
    }

    bool is_countdown_complete() const override
    {
        return m_countdown_ms <= 0;
    }

    bool is_motion_detected() const override
    {
        return m_control_data.motion_detected;
    }

    void set_red_led(bool on) override
    {
        digitalWrite(IO_PIN_RED_LED, on ? HIGH : LOW);
    }

    void notify_event_ignored() override
    {
        m_published_events.event_ignored = true;
    }
    void notify_armed() override
    {
        m_published_events.armed = true;
    }
    void notify_disarmed() override
    {
        m_published_events.disarmed = true;
    }
    void notify_alarm() override
    {
        m_published_events.alarm = true;
    }

    void m_handle_state_change(const ControllerSm::StateId prev_state, const ControllerSm::StateId current_state)
    {
        if (prev_state == current_state) {
            return; // no state change
        }

        // state change detected

        // log console message
        Serial.print(F("CONTROL state changed from "));
        Serial.print(ControllerSm::stateIdToString(prev_state));
        Serial.print(F(" to "));
        Serial.println(ControllerSm::stateIdToString(current_state));
    }
};
