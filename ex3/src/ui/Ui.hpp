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
        m_process_audio(now_ms);
        m_process_control_events();
        m_process_lcd_update(loop_delta_ms);
        m_process_fsm_timeout(loop_delta_ms);
    }

    void dispatch_event(UiSm::EventId event_id)
    {
        const UiSm::StateId prev_state = m_fsm.stateId;
        m_fsm.vars.event_id = event_id;
        m_fsm.dispatchEvent(event_id);
        const UiSm::StateId current_state = m_fsm.stateId;

        m_handle_control_state_change();
        m_handle_ui_state_change(prev_state, current_state);
    }

private:
    int16_t m_ms_to_screen_update = 0;
    UiSm m_fsm {};
    Audio m_audio {};
    Display m_display;
    ControllerSm::StateId m_last_control_state_id {};

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

    const ControlData& m_get_control_data()
    {
        return m_fsm.vars.ctrl->get_control_data();
    }

    // Every X milliseconds, we send an LCD_UPDATE event to the state machine.
    void m_process_lcd_update(const uint32_t loop_delta_ms)
    {
        m_ms_to_screen_update -= loop_delta_ms;

        const bool is_time_to_update = m_ms_to_screen_update <= 0;
        if (is_time_to_update) {
            m_reset_lcd_update_timer();
            dispatch_event(UiSm::EventId::LCD_UPDATE);
        }
    }

    void m_process_audio(const uint32_t now_ms)
    {
        const ControlData& control_data = m_get_control_data();

        const bool is_counting_down = control_data.countdown_ms > 0;

        if (!is_counting_down) {
            // Run audio system as normal.
            m_audio.update(now_ms);
        } else {
            // We are counting down. Ignore any songs and beep with countdown.
            const int MS_PER_SECOND = 1000;
            const int16_t remainder_ms = control_data.countdown_ms % MS_PER_SECOND;
            if (remainder_ms > 900) {
                m_audio.start_beep();
            } else {
                m_audio.end_beep();
            }
        }
    }

    void m_reset_lcd_update_timer()
    {
        m_ms_to_screen_update = 100;
    }

    void m_handle_control_state_change()
    {
        const ControlData& control_data = m_get_control_data();
        if (control_data.state_id == m_last_control_state_id) {
            return; // no control state change
        }
        m_last_control_state_id = control_data.state_id;

        m_reset_lcd_update_timer(); // reset the LCD update timer so that it aligns with display update
        dispatch_event(UiSm::EventId::REFRESH);
    }

    void m_process_control_events()
    {
        // State machine stores a pointer to the controller. Might be cleaner to store a reference here as well.
        Controller& controller = *m_fsm.vars.ctrl;
        ControlEvents& events = controller.get_published_events();

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
};
