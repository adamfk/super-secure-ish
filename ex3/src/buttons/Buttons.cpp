#include "Buttons.hpp"

// for AVR, better to use PROGMEM
const char* const Buttons::m_button_names[ButtonId_COUNT] = {
    "UP",
    "DOWN",
    "BACK",
    "ENTER"
};

Buttons::Buttons(Ui& ui)
    : m_ui(ui)
{
    // Setup buttons
    m_buttons[ButtonId_UP].setup(IO_PIN_BUTTON_UP);
    m_buttons[ButtonId_DOWN].setup(IO_PIN_BUTTON_DOWN);
    m_buttons[ButtonId_BACK].setup(IO_PIN_BUTTON_BACK);
    m_buttons[ButtonId_ENTER].setup(IO_PIN_BUTTON_ENTER);

    // Static assertions to ensure the m_button_names array is in sync with the ButtonId enum.
    // Done here because the ButtonId enum is private to this class.
    static_assert(Buttons::ButtonId_UP == 0, "m_button_names array out of sync with ButtonId enum");
    static_assert(Buttons::ButtonId_DOWN == 1, "m_button_names array out of sync with ButtonId enum");
    static_assert(Buttons::ButtonId_BACK == 2, "m_button_names array out of sync with ButtonId enum");
    static_assert(Buttons::ButtonId_ENTER == 3, "m_button_names array out of sync with ButtonId enum");
    static_assert(Buttons::ButtonId_COUNT == 4, "m_button_names array out of sync with ButtonId enum");
}

void Buttons::update(const uint32_t loop_delta_ms)
{
    // run the button state machines
    for (int i = 0; i < ButtonId_COUNT; i++) {
        m_buttons[i].update(loop_delta_ms);
        // m_buttons[i].print_button_events(m_button_names[i]);
    }

    // Detect and send ANY_KEY press event
    for (int i = 0; i < ButtonId_COUNT; i++) {
        const auto& button = m_buttons[i];
        if (button.fsm.vars.press_event) {
            m_ui.dispatch_event(UiSm::EventId::ANY_KEY);
            break;
        }
    }

    auto& up_button = m_buttons[ButtonId_UP];
    auto& down_button = m_buttons[ButtonId_DOWN];
    auto& back_button = m_buttons[ButtonId_BACK];
    auto& enter_button = m_buttons[ButtonId_ENTER];

    if (up_button.fsm.vars.press_event) {
        m_ui.dispatch_event(UiSm::EventId::UP_PRESS);
    }

    if (down_button.fsm.vars.press_event) {
        m_ui.dispatch_event(UiSm::EventId::DOWN_PRESS);
    }

    if (back_button.fsm.vars.press_event) {
        m_ui.dispatch_event(UiSm::EventId::BACK_PRESS);
    }

    if (back_button.fsm.vars.long_event) {
        m_ui.dispatch_event(UiSm::EventId::BACK_HELD);
    }

    if (enter_button.fsm.vars.press_event) {
        m_ui.dispatch_event(UiSm::EventId::ENTER_PRESS);
    }

    // detect secret button combination (UP + DOWN held for 2 seconds)
    if (is_held_2_seconds(up_button) && is_held_2_seconds(down_button)) {
        // reset the timers to avoid repeat event
        up_button.fsm.vars.timer_ms = 0;
        down_button.fsm.vars.timer_ms = 0;

        m_ui.dispatch_event(UiSm::EventId::UP_DOWN_HELD);
    }

    // detect PANIC button combination (BACK + ENTER held for 2 seconds)
    if (is_held_2_seconds(back_button) && is_held_2_seconds(enter_button)) {
        // reset the timers to avoid repeat event
        back_button.fsm.vars.timer_ms = 0;
        enter_button.fsm.vars.timer_ms = 0;

        m_ui.dispatch_event(UiSm::EventId::PANIC_BUTTON);
    }

    for (int i = 0; i < ButtonId_COUNT; i++) {
        m_buttons[i].clear_button_events();
    }
}

bool Buttons::is_held_2_seconds(const MyArduinoButton& button)
{
    return (button.fsm.vars.press_status && button.fsm.vars.timer_ms > 2000);
}
