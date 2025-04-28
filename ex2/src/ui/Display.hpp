#pragma once
#include "../control/ControlData.hpp"
#include "../io_config.hpp"
#include <Arduino.h>
#include <LiquidCrystal.h>

/**
 * This class is implemented rather simply for demonstration purposes.
 */
class Display {
public:
    Display(const ControlData& control_data)
        : m_lcd(IO_PIN_LCD1, IO_PIN_LCD2, IO_PIN_LCD3, IO_PIN_LCD4, IO_PIN_LCD5, IO_PIN_LCD6)
        , m_control_data(control_data)
    {
        m_lcd.begin(LCD_COLUMNS, LCD_ROWS);
    }

    void clear()
    {
        m_lcd.clear();
    }

    void set_line(uint8_t line, const __FlashStringHelper* str)
    {
        const uint8_t column = 0;
        m_lcd.setCursor(column, line);
        m_lcd.print(str);
    }

    void clear_remaining_on_line()
    {
        for (size_t i = 0; i < LCD_COLUMNS; i++) {
            m_lcd.write(' ');
        }
    }

    void SPLASH_SCREEN()
    {
        clear();
        // lcd guide:  ||||||||||||||||
        set_line(0, F("SUPER SECURE-ish"));
        set_line(1, F(" SYSTEM v0.2.3"));
    }

    void HOME()
    {
        clear();

        switch (m_control_data.state_id) {
        case ControllerSm::StateId::ROOT:
            set_line(0, F("?")); // this shouldn't happen
            break;

        case ControllerSm::StateId::DISARMED:
            // lcd guide:  ||||||||||||||||
            set_line(0, F(">>> DISARMED <<<"));
            HOME_show_press_enter();
            break;

        case ControllerSm::StateId::ARMING:
            // lcd guide:  ||||||||||||||||
            set_line(0, F(">>>  ARMING  <<<"));
            set_line(1, F("countdown: ")); // rest of line is written in HOME_update()
            break;

        case ControllerSm::StateId::ARMED:
            // lcd guide:  ||||||||||||||||
            set_line(0, F(">>>  ARMED  <<<"));
            HOME_show_press_enter();
            break;

        case ControllerSm::StateId::ALARM_COUNTDOWN:
            // lcd guide:  ||||||||||||||||
            set_line(0, F(">>> TRIGGER <<<"));
            set_line(1, F("countdown: ")); // rest of line is written in HOME_update()
            break;

        case ControllerSm::StateId::ALARM_ACTIVE:
            // lcd guide:  ||||||||||||||||
            set_line(0, F("!!!  ALARM  !!!"));
            HOME_show_press_enter();
            break;
        }

        HOME_update();
    }

    void HOME_show_press_enter()
    {
        set_line(1, F(" press enter"));
    }

    void HOME_update()
    {
        switch (m_control_data.state_id) {
        case ControllerSm::StateId::ALARM_ACTIVE:
        case ControllerSm::StateId::ROOT:
        case ControllerSm::StateId::DISARMED:
        case ControllerSm::StateId::ARMED:
            // nothing here needs to be dynamically updated
            break;

        case ControllerSm::StateId::ALARM_COUNTDOWN:
        case ControllerSm::StateId::ARMING:
            m_lcd.setCursor(10, 1);
            m_lcd.print(millis_to_display_seconds(m_control_data.countdown_ms));
            clear_remaining_on_line();
            break;
        }
    }

    void MENU__DISARM_SYSTEM()
    {
        clear();
        // lcd guide:  ||||||||||||||||
        set_line(0, F("--- MENU 1/4 ---"));
        set_line(1, F("> disarm system"));
    }

    void DISARM_SYSTEM()
    {
        clear();
        // lcd guide:  ||||||||||||||||
        set_line(0, F("---- DISARM ----"));
        ask_confirm_line2();
    }

    void ask_confirm_line2()
    {
        set_line(1, F("> confirm?"));
    }

    void MENU__ARM_SYSTEM()
    {
        clear();
        // lcd guide:  ||||||||||||||||
        set_line(0, F("--- MENU 2/4 ---"));
        set_line(1, F("> arm system"));
    }

    void ARM_SYSTEM()
    {
        clear();
        // lcd guide:  ||||||||||||||||
        set_line(0, F("-- ARM SYSTEM --"));
        ask_confirm_line2();
    }

    void MENU__CONFIG()
    {
        clear();
        // lcd guide:  ||||||||||||||||
        set_line(0, F("--- MENU 3/4 ---"));
        set_line(1, F("> configure"));
    }

    void CONFIG__CHANGE_CODE()
    {
        clear();
        // lcd guide:  ||||||||||||||||
        set_line(0, F("-- CONFIG 1/3 --"));
        set_line(1, F("> change code"));
    }

    void CHANGE_CODE()
    {
        clear();
        // lcd guide:  ||||||||||||||||
        set_line(0, F("-- CHANGE CODE -"));
        set_line(1, F("NA. try secret.."));
    }

    void CONFIG__ARMING_DELAY()
    {
        clear();
        // lcd guide:  ||||||||||||||||
        set_line(0, F("-- CONFIG 2/3 --"));
        set_line(1, F("> arming delay"));
    }

    void ARMING_DELAY()
    {
        clear();
        // lcd guide:  ||||||||||||||||
        set_line(0, F("--- ARM DELAY --"));
        set_line(1, F("not implemented"));
    }

    void CONFIG__ALARM_DELAY()
    {
        clear();
        // lcd guide:  ||||||||||||||||
        set_line(0, F("-- CONFIG 3/3 --"));
        set_line(1, F("> alarm delay"));
    }

    void ALARM_DELAY()
    {
        clear();
        // lcd guide:  ||||||||||||||||
        set_line(0, F("-- ALARM DELAY -"));
        set_line(1, F("not implemented"));
    }

    void MENU__DATA()
    {
        clear();
        // lcd guide:  ||||||||||||||||
        set_line(0, F("--- MENU 4/4 ---"));
        set_line(1, F("> live data"));
    }

    void UPTIME()
    {
        clear();
        // lcd guide:  ||||||||||||||||
        set_line(0, F("-- DATA:UPTIME -"));
        UPTIME_update();
    }

    void UPTIME_update()
    {
        m_lcd.setCursor(0, 1);
        m_lcd.print(millis_to_display_seconds(millis()));
        clear_remaining_on_line();
    }

    void SENSOR()
    {
        clear();
        // lcd guide:  ||||||||||||||||
        set_line(0, F("-- DATA:SENSOR -"));
        SENSOR_update();
    }

    void SENSOR_update()
    {
        m_lcd.setCursor(0, 1);
        const auto str = m_control_data.motion_detected ? F("motion detected") : F("no motion");
        m_lcd.print(str);
        clear_remaining_on_line();
    }

    void PANIC()
    {
        clear();
        // lcd guide:  ||||||||||||||||
        set_line(0, F("! PANIC BUTTON !"));
        set_line(1, F("! ALARM ACTIVE !"));
    }

    // if countdown timer is 5 seconds, people want to see it go from 5 to 1, rather than 4 to 0.
    uint32_t millis_to_display_seconds(uint32_t ms)
    {
        const int MS_PER_SECOND = 1000;
        return (ms + MS_PER_SECOND - 1) / MS_PER_SECOND; // this a simple ceiling function
    }

private:
    static const char SPECIAL_CHAR_CODE = '\0';
    static const uint8_t LCD_ROWS = 2;
    static const uint8_t LCD_COLUMNS = 16;
    static const uint8_t PIXEL_ROWS = 8;

    LiquidCrystal m_lcd;
    const ControlData& m_control_data;
    uint8_t m_custom_char = 0;
};
