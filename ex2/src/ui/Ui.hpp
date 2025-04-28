#pragma once
#include "../audio/Audio.hpp"
#include "Display.hpp"
#include "UiSm.hpp"
#include <Arduino.h>

// forward declare classes to avoid circular dependencies
class Controller;

class Ui {
public:
    Ui(Controller& controller)
        : m_display(controller.get_control_data())
    {
        // setup state machine variables
        m_fsm.vars.ctrl = &controller;
        m_fsm.vars.lcd = &m_display;
        m_fsm.vars.audio = &m_audio;
    }

    void setup()
    {
        // start the state machine!
        m_fsm.start();
    }

    void update(const uint32_t now_ms, const uint32_t loop_delta_ms)
    {
        m_process_control_events();
        m_process_fsm_timeout(loop_delta_ms);
        m_audio.update(now_ms);
    }

    void dispatch_event(UiSm::EventId event_id)
    {
        const UiSm::StateId prev_state = m_fsm.stateId;
        m_fsm.dispatchEvent(event_id);
        const UiSm::StateId current_state = m_fsm.stateId;

        m_handle_ui_state_change(prev_state, current_state);
    }

private:
    UiSm m_fsm {};
    Audio m_audio {};
    Display m_display;

    void m_process_fsm_timeout(const uint32_t loop_delta_ms)
    {
        if (m_fsm.vars.timeout_ms <= 0) {
            return; // already timed out
        }

        m_fsm.vars.timeout_ms -= loop_delta_ms;
        if (m_fsm.vars.timeout_ms <= 0) {
            dispatch_event(UiSm::EventId::TIMEOUT);
        }
    }

    void m_process_control_events()
    {
        ControlEvents& events = m_get_control_events();

        if (events.armed) {
            events.armed = false; // clear the event
            m_audio.play_armed();
        } else if (events.disarmed) {
            events.disarmed = false; // clear the event
            m_audio.play_disarmed();
        } else if (events.event_ignored) {
            events.event_ignored = false; // clear the event
            m_audio.play_ignored();
        } else if (events.alarm) {
            events.alarm = false; // clear the event
            m_audio.play_alarm();
        }
    }

    void m_handle_ui_state_change(const UiSm::StateId prev_state, const UiSm::StateId current_state)
    {
        if (prev_state == current_state) {
            return; // no state change
        }

        // log console message
        Serial.print(F("UI state changed from "));
        Serial.print(UiSm::stateIdToString(prev_state));
        Serial.print(F(" to "));
        Serial.println(UiSm::stateIdToString(current_state));
    }

    const ControlData& m_get_control_data()
    {
        // State machine already stores a pointer to the controller.
        return m_fsm.vars.ctrl->get_control_data();
    }

    ControlEvents& m_get_control_events()
    {
        // State machine already stores a pointer to the controller.
        return m_fsm.vars.ctrl->get_published_events();
    }
};
