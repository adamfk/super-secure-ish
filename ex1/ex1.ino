#include "ButtonSm.h"

////////////////////////////////////////////////////////////////
// CONSTANTS
////////////////////////////////////////////////////////////////
static const int IO_PIN_BUTTON_ENTER = A2;


////////////////////////////////////////////////////////////////
// VARIABLES
////////////////////////////////////////////////////////////////
ButtonSm button_enter;
// static_assert(sizeof(button_enter) == 4, "ButtonSm struct size is 4 if we tell GCC to pack the enumeration");

uint32_t last_loop_ms = 0;


////////////////////////////////////////////////////////////////
// SETUP - called once at startup by Arduino
////////////////////////////////////////////////////////////////
void setup()
{
    // setup input pins
    pinMode(IO_PIN_BUTTON_ENTER, INPUT_PULLUP);

    Serial.begin(115200);
    Serial.println("Press and hold enter button to test state machine.");

    // setup and start the button state machines
    ButtonSm_ctor(&button_enter);
    ButtonSm_start(&button_enter);
}

////////////////////////////////////////////////////////////////
// LOOP - called repeatedly by Arduino
////////////////////////////////////////////////////////////////
void loop()
{
    const uint32_t now_ms = millis();
    const uint32_t loop_delta_ms = now_ms - last_loop_ms;
    last_loop_ms = now_ms;

    // update state machine inputs
    button_enter.vars.input_active = (digitalRead(IO_PIN_BUTTON_ENTER) == LOW);
    button_enter.vars.timer_ms += loop_delta_ms; // ex3 saturates on overflow

    // run state machine
    ButtonSm_dispatch_event(&button_enter, ButtonSm_EventId_DO);

    // ------------ check for events and print them -------------

    if (button_enter.vars.press_event) {
        button_enter.vars.press_event = false; // clear event
        Serial.println("ENTER_PRESS");
    }

    if (button_enter.vars.long_event) {
        button_enter.vars.long_event = false; // clear event
        Serial.println("ENTER_HELD");
    }

    if (button_enter.vars.release_event) {
        button_enter.vars.release_event = false; // clear event
        Serial.println("ENTER_RELEASE");
    }

    delay(5); // lower wokwi simulation CPU usage
}
