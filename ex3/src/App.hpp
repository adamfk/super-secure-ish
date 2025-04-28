#pragma once
#include "buttons/Buttons.hpp"
#include "control/ControlData.hpp"
#include "control/Controller.hpp"
#include "serial_ui/SerialUi.hpp"
#include "ui/Ui.hpp"
#include <Arduino.h>

class App {
public:
    void setup()
    {
        Serial.begin(115200);
        Serial.println("Starting up...");
        m_ui.setup();
        m_controller.setup();
    }

    void update(const uint32_t now_ms)
    {
        const uint32_t loop_delta_ms = now_ms - m_last_loop_ms;
        m_last_loop_ms = now_ms;

        m_buttons.update(loop_delta_ms);
        m_ui.update(now_ms, loop_delta_ms);
        m_serial_ui.update();
        m_controller.update(loop_delta_ms);
    }

private:
    Controller m_controller {};
    SerialUi m_serial_ui { m_controller };
    Ui m_ui { m_controller };
    Buttons m_buttons { m_ui };
    uint32_t m_last_loop_ms = 0;
};
