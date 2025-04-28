#pragma once
#include "../control/Controller.hpp"
#include <Arduino.h>

/**
 * This serial UI simply reads commands from the serial port and sends them to the controller.
 * It could be extended to also show the state of the controller.
 */
class SerialUi {
public:
    SerialUi(Controller& controller)
        : m_controller(controller)
    {
    }

    void update()
    {
        if (Serial.available()) {
            char command = Serial.read();
            m_handle_command(command);
        }
    }

private:
    Controller& m_controller;

    void m_handle_command(char command)
    {
        if (command == '\r' || command == '\n') {
            return; // Ignore newlines and carriage returns
        }

        Serial.print(F("SerialUi: processing command '"));
        Serial.print(command);
        Serial.println(F("'."));

        switch (command) {
        case 'a':
            m_controller.arm();
            break;
        case 'd':
            m_controller.disarm();
            break;
        case 'p':
            m_controller.panic();
            break;
        default:
            Serial.println(F("SerialUi: ignoring unknown command."));
            break;
        }
    }
};
