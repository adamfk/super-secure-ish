#include "Buttons.hpp"

Buttons::Buttons(Ui& ui)
    : m_ui(ui)
{
    // Setup buttons
    m_buttons[ButtonId_UP].setup(IO_PIN_BUTTON_UP);
    m_buttons[ButtonId_DOWN].setup(IO_PIN_BUTTON_DOWN);
    m_buttons[ButtonId_BACK].setup(IO_PIN_BUTTON_BACK);
    m_buttons[ButtonId_ENTER].setup(IO_PIN_BUTTON_ENTER);
}

void Buttons::update(const uint32_t loop_delta_ms)
{
    // run the button state machines
    for (int i = 0; i < ButtonId_COUNT; i++) {
        m_buttons[i].update(loop_delta_ms);
    }

    // Detect and send ANY_KEY press event
    for (int i = 0; i < ButtonId_COUNT; i++) {
        const auto& button = m_buttons[i];
        if (button.fsm.vars.press_event) {
            m_ui.dispatch_event(UiSm::EventId::ANY_KEY);
            break;
        }
    }

    MyArduinoButton& up_button = m_buttons[ButtonId_UP];
    MyArduinoButton& down_button = m_buttons[ButtonId_DOWN];
    MyArduinoButton& back_button = m_buttons[ButtonId_BACK];
    MyArduinoButton& enter_button = m_buttons[ButtonId_ENTER];

    if (up_button.fsm.vars.press_event) {
        m_ui.dispatch_event(UiSm::EventId::UP_PRESS);
    }

    if (down_button.fsm.vars.press_event) {
        m_ui.dispatch_event(UiSm::EventId::DOWN_PRESS);
    }

    if (back_button.fsm.vars.press_event) {
        m_ui.dispatch_event(UiSm::EventId::BACK_PRESS);
    }

    if (enter_button.fsm.vars.press_event) {
        m_ui.dispatch_event(UiSm::EventId::ENTER_PRESS);
    }

    for (int i = 0; i < ButtonId_COUNT; i++) {
        m_buttons[i].clear_button_events();
    }
}
