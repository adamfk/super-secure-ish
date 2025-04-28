#pragma once
#include "ButtonSm.h"
#include <Arduino.h>

/**
 * A thin wrapper around the ButtonSm state machine to make it easier to use
 * with Arduino.
 */
class MyArduinoButton {
public:
    ButtonSm fsm {};

    /**
     * Setup the button state machine and pin.
     */
    void setup(uint8_t pin)
    {
        m_pin = pin;
        pinMode(m_pin, INPUT_PULLUP);

        ButtonSm_ctor(&fsm); // construct button state machine
        ButtonSm_start(&fsm);
    }

    /**
     * Reads button pin, updates button state machine and timer.
     */
    void update(uint32_t loop_delta_ms)
    {
        // read pin status and set input to state machine
        fsm.vars.input_active = (digitalRead(m_pin) == LOW);

        // update button timer
        fsm.vars.timer_ms += loop_delta_ms;
        if (fsm.vars.timer_ms < loop_delta_ms) {
            // overflow
            fsm.vars.timer_ms = UINT16_MAX;
        }

        // run state machine
        ButtonSm_dispatch_event(&fsm, ButtonSm_EventId_DO);
    }

    /**
     * Checks if a button event occurred, clears the event and prints it to serial.
     */
    void print_button_events(const char* button_name)
    {
        if (fsm.vars.press_event) {
            m_print_button_event(button_name, F("press"));
        }

        if (fsm.vars.long_event) {
            m_print_button_event(button_name, F("long"));
        }

        if (fsm.vars.release_event) {
            m_print_button_event(button_name, F("release"));
            Serial.println();
        }
    }

    void clear_button_events()
    {
        fsm.vars.press_event = false;
        fsm.vars.long_event = false;
        fsm.vars.release_event = false;
    }

private:
    uint8_t m_pin;

    /**
     * Simply prints button and event to serial.
     */
    void m_print_button_event(const char* button_name, const __FlashStringHelper* event)
    {
        Serial.print(button_name);
        Serial.print(": ");
        Serial.print(event);
        Serial.println("");
    }
};
