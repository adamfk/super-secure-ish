#pragma once
#include "../io_config.hpp"
#include "../ui/Ui.hpp"
#include "MyArduinoButton.hpp"
#include <stdint.h>

class Buttons {
public:
    Buttons(Ui& ui);
    void update(const uint32_t loop_delta_ms);

private:
    enum ButtonId {
        ButtonId_UP,
        ButtonId_DOWN,
        ButtonId_BACK,
        ButtonId_ENTER,
        ButtonId_COUNT
    };

    Ui& m_ui;
    bool m_up_down_event = false;
    MyArduinoButton m_buttons[ButtonId_COUNT];
    static const char* const m_button_names[ButtonId_COUNT];

    static bool is_held_2_seconds(const MyArduinoButton& button);
};
