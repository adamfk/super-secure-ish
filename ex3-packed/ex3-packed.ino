//////////////////////////////////////////////////////////////////////////////////
// FILE: src/io_config.hpp
//////////////////////////////////////////////////////////////////////////////////
// #pragma once
#include <pins_arduino.h> // for A0 defines

static const int IO_PIN_BUZZER = 6;
static const int IO_PIN_RED_LED = 2;
static const int IO_PIN_MOTION_SENSOR = A0;
static const int IO_PIN_LCD1 = 12;
static const int IO_PIN_LCD2 = 11;
static const int IO_PIN_LCD3 = 10;
static const int IO_PIN_LCD4 = 9;
static const int IO_PIN_LCD5 = 8;
static const int IO_PIN_LCD6 = 7;
static const int IO_PIN_BUTTON_ENTER = A2;
static const int IO_PIN_BUTTON_DOWN = A3;
static const int IO_PIN_BUTTON_BACK = A4;
static const int IO_PIN_BUTTON_UP = A5;


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/audio/Note.hpp
//////////////////////////////////////////////////////////////////////////////////
// #pragma once
#include <stdint.h>

struct Note
{
    uint16_t tone_hz;
    uint16_t duration_ms;
};


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/audio/Song.hpp
//////////////////////////////////////////////////////////////////////////////////
// #pragma once
// #include "Note.hpp"
#include <stdint.h>

struct Song
{
    explicit Song(const Note* notes, uint16_t notes_length, uint8_t note_interior_end_gap_ms = 0)
        : notes(notes)
        , notes_length(notes_length)
        , note_interior_end_gap_ms(note_interior_end_gap_ms) {}

    const Note* notes;
    const uint16_t notes_length;

    // This doesn't affect the note duration, but it does affect the time between note tones.
    // The longer this is, the shorter the note's tone duration will be.
    const uint8_t note_interior_end_gap_ms;
};


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/audio/SongPlayer.hpp
//////////////////////////////////////////////////////////////////////////////////
/**
 * Inspired from https://wokwi.com/projects/390785670905308161
 */
// #pragma once
// #include "../io_config.hpp"
// #include "Song.hpp"
#include <Arduino.h> // for tone() and noTone()

/**
 * NOTE: this class is susceptible to uint32_t milliseconds overflow every 50 days or so.
 * The way we handle time here is best when we have 64 bit timestamps.
 * It could also be converted to use loop_delta_ms instead of current_time_ms.
 */
class SongPlayer {
public:
    void set_song(const Song& song, uint8_t frequency_scaler = 1)
    {
        m_song = &song;
        m_frequency_scaler = frequency_scaler;
    }

    void stop()
    {
        noTone(IO_PIN_BUZZER);
        m_note_index = UINT16_MAX;
    }

    void play_from_start(const uint32_t current_time_ms)
    {
        if (m_song == nullptr) {
            return;
        }

        const Song& song = *m_song;

        m_note_index = 0;
        m_next_event_at_ms = current_time_ms;
        m_play_note(song, song.notes[m_note_index], current_time_ms);
    }

    void update(const uint32_t current_time_ms)
    {
        if (m_song == nullptr) {
            return;
        }

        const Song& song = *m_song;

        if (m_note_index > song.notes_length) {
            return; // no more notes to play
        }

        if (current_time_ms < m_next_event_at_ms) {
            return; // not yet time to play the next note
        }

        const bool need_to_stop_last_note = (m_note_index == song.notes_length);
        if (need_to_stop_last_note) {
            noTone(IO_PIN_BUZZER); // stop playing the last note
            m_note_index++; // to prevent this from being called again
            return;
        }

        const Note& note = song.notes[m_note_index];
        m_play_note(song, note, current_time_ms);
    }

    static uint16_t calc_tone_duration_ms(uint16_t note_duration_ms, uint8_t note_interior_end_gap_ms)
    {
        // UNSIGNED SUBTRACTION! Watch for underflow.
        // clamp if underflowed
        uint16_t tone_duration_ms = note_duration_ms - note_interior_end_gap_ms;
        if (tone_duration_ms > note_duration_ms) {
            // underflowed, clamp it
            tone_duration_ms = 0;
        }

        return tone_duration_ms;
    }

private:
    uint32_t m_next_event_at_ms = 0;
    uint16_t m_note_index = 0;
    uint8_t m_frequency_scaler = 1;
    const Song* m_song = nullptr;

    void m_play_note(const Song& song, const Note& note, const uint32_t current_time_ms)
    {
        if (note.tone_hz == 0) {
            noTone(IO_PIN_BUZZER);
        } else {
            const uint16_t tone_duration_ms = calc_tone_duration_ms(note.duration_ms, song.note_interior_end_gap_ms);
            const uint16_t scaled_frequency = note.tone_hz * m_frequency_scaler;
            tone(IO_PIN_BUZZER, scaled_frequency, tone_duration_ms);
        }

        // For timing the next note, we have two options:
        //     - option 1. prioritize song timing over note timing
        //     - option 2. prioritize note timing over song timing

        // option 1: prioritize song timing over note timing
        // m_next_event_at_ms += note.duration_ms;
        // (void)current_time_ms; // to avoid unused variable warning

        // option 2: prioritize note timing over song timing
        m_next_event_at_ms = current_time_ms + note.duration_ms;

        m_note_index++;
    }
};


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/audio/songs.hpp
//////////////////////////////////////////////////////////////////////////////////
// #pragma once
// #include "Song.hpp"

class Ff7FanFare {
public:
    static const Song song;

private:
    static const Note notes[];
};

class ArmSong {
public:
    static const Song song;

private:
    static const Note notes[];
};

class DisarmSong {
public:
    static const Song song;

private:
    static const Note notes[];
};

class DoubleBeepLowSong {
public:
    static const Song song;

private:
    static const Note notes[];
};

class AlarmSong {
public:
    static const Song song;

private:
    static const Note notes[];
};


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/audio/songs.cpp
//////////////////////////////////////////////////////////////////////////////////
// #include "songs.hpp"
#include <stddef.h>

#define COUNT_OF(x) ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))

// These frequencies will need to be scaled up to be audible with a piezo speaker,
// but we are simulating on a PC and I find the lower frequencies less irritating :)
// Tones: https://github.com/bhagman/Tone
#define Ab1 52
#define Bb1 58
#define C_2 65

#define G4 392
#define B4 494

#define C3 131
#define E3 165
#define G3 196

#define C5 523
#define E5 659
#define G5 783

// minimum duration for a note to be audible with wokwi
#define MINIMUM_DURATION_MS 50

// quarter note duration
#define WHOLE 520
#define HALF (WHOLE / 2)
#define QUARTER (HALF / 2)
#define EIGHTH (QUARTER / 2)

// https://www.hooktheory.com/theorytab/view/nobuo-uematsu/final-fantasy-vii---victory-fanfare
// RAM: AVR based boards should consider storing in PROGMEM to save RAM
const Note Ff7FanFare::notes[] = {
    { .tone_hz = C_2, .duration_ms = QUARTER },
    { .tone_hz = C_2, .duration_ms = QUARTER },
    { .tone_hz = C_2, .duration_ms = QUARTER },
    { .tone_hz = C_2, .duration_ms = QUARTER * 3 },
    { .tone_hz = Ab1, .duration_ms = QUARTER * 3 },
    { .tone_hz = Bb1, .duration_ms = QUARTER * 3 },
    { .tone_hz = C_2, .duration_ms = HALF },
    { .tone_hz = Bb1, .duration_ms = QUARTER },
    { .tone_hz = C_2, .duration_ms = WHOLE * 2 }
};

// RAM: AVR based boards should consider storing in PROGMEM to save RAM
const Song Ff7FanFare::song {
    .notes = &notes[0],
    .notes_length = COUNT_OF(notes),
    .note_interior_end_gap_ms = MINIMUM_DURATION_MS // so we can hear the end of each note
};

////////////////////////////////////////////////////////////////////////////////////////////////////

// RAM: AVR based boards should consider storing in PROGMEM to save RAM
const Note ArmSong::notes[] = {
    { .tone_hz = C5, .duration_ms = QUARTER },
    { .tone_hz = E5, .duration_ms = QUARTER },
    { .tone_hz = G5, .duration_ms = QUARTER },
};

// RAM: AVR based boards should consider storing in PROGMEM to save RAM
const Song ArmSong::song {
    .notes = &notes[0],
    .notes_length = COUNT_OF(notes),
    .note_interior_end_gap_ms = QUARTER / 2 // we want lots of space between notes
};

////////////////////////////////////////////////////////////////////////////////////////////////////

// RAM: AVR based boards should consider storing in PROGMEM to save RAM
const Note DisarmSong::notes[] = {
    { .tone_hz = G3, .duration_ms = QUARTER },
    { .tone_hz = E3, .duration_ms = QUARTER },
    { .tone_hz = C3, .duration_ms = QUARTER },
};

// RAM: AVR based boards should consider storing in PROGMEM to save RAM
const Song DisarmSong::song {
    .notes = &notes[0],
    .notes_length = COUNT_OF(notes),
    .note_interior_end_gap_ms = QUARTER / 2 // we want lots of space between notes
};

////////////////////////////////////////////////////////////////////////////////////////////////////

// RAM: AVR based boards should consider storing in PROGMEM to save RAM
const Note DoubleBeepLowSong::notes[] = {
    { .tone_hz = C_2, .duration_ms = QUARTER },
    { .tone_hz = C_2, .duration_ms = QUARTER },
};

// RAM: AVR based boards should consider storing in PROGMEM to save RAM
const Song DoubleBeepLowSong::song {
    .notes = &notes[0],
    .notes_length = COUNT_OF(notes),
    .note_interior_end_gap_ms = MINIMUM_DURATION_MS
};

////////////////////////////////////////////////////////////////////////////////////////////////////

// RAM: AVR based boards should consider storing in PROGMEM to save RAM
const Note AlarmSong::notes[] = {
    { .tone_hz = B4, .duration_ms = WHOLE },
    { .tone_hz = G4, .duration_ms = WHOLE },
    { .tone_hz = B4, .duration_ms = WHOLE },
    { .tone_hz = G4, .duration_ms = WHOLE },
};

// RAM: AVR based boards should consider storing in PROGMEM to save RAM
const Song AlarmSong::song {
    .notes = &notes[0],
    .notes_length = COUNT_OF(notes),
    .note_interior_end_gap_ms = 0 // no gap between notes
};


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/audio/Audio.hpp
//////////////////////////////////////////////////////////////////////////////////
// #pragma once
// #include "../io_config.hpp"
// #include "SongPlayer.hpp"
// #include "songs.hpp"
#include <Arduino.h> // for tone() and noTone()

class Audio {
public:
    Audio()
    {
    }

    void update(const uint32_t now_ms)
    {
        m_player.update(now_ms);
    }

    void start_beep()
    {
        tone(IO_PIN_BUZZER, 400, 100);
    }

    void end_beep()
    {
        noTone(IO_PIN_BUZZER);
    }

    void play_armed()
    {
        m_player.set_song(ArmSong::song);
        m_player.play_from_start(millis());
    }

    void play_disarmed()
    {
        m_player.set_song(DisarmSong::song);
        m_player.play_from_start(millis());
    }

    void play_secret()
    {
        const uint8_t frequency_scaler = 3;
        m_player.set_song(Ff7FanFare::song, frequency_scaler);
        m_player.play_from_start(millis());
    }

    void play_alarm()
    {
        m_player.set_song(AlarmSong::song);
        m_player.play_from_start(millis());
    }

    void play_ignored()
    {
        m_player.set_song(DoubleBeepLowSong::song);
        m_player.play_from_start(millis());
    }

    void stop()
    {
        m_player.stop();
    }

private:
    SongPlayer m_player {};
};


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/control/ControllerBase.hpp
//////////////////////////////////////////////////////////////////////////////////
// #pragma once

class ControllerBase {
public:
    ControllerBase() = default;
    virtual ~ControllerBase() = default;

    virtual void clear_countdown() = 0;
    virtual void init_arming_countdown() = 0;
    virtual void init_alarm_countdown() = 0;

    virtual bool is_countdown_complete() const = 0;
    virtual bool is_motion_detected() const = 0;

    virtual void notify_disarmed() = 0;
    virtual void notify_armed() = 0;
    virtual void notify_alarm() = 0;
    virtual void notify_event_ignored() = 0;

    virtual void set_red_led(bool on) = 0;
};


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/control/ControllerSm.hpp
//////////////////////////////////////////////////////////////////////////////////
// Autogenerated with StateSmith 0.18.2+6062baabb038910ff3841c3cd5938115c0ed0a03.
// Algorithm: Balanced2. See https://github.com/StateSmith/StateSmith/wiki/Algorithms

// Whatever you put in this `FileTop` section will end up 
// being printed at the top of every generated code file.

// #pragma once  // You can also specify normal include guard. See https://github.com/StateSmith/StateSmith/blob/main/docs/settings.md
#include <stdint.h>
// #include "ControllerBase.hpp"


// Generated state machine
class ControllerSm : public ControllerBase
{
public:
    enum class EventId: uint8_t
    {
        DO = 0, // The `do` event is special. State event handlers do not consume this event (ancestors all get it too) unless a transition occurs.
        ALARM_NOW = 1,
        ARM = 2,
        DISARM = 3,
    };
    
    enum
    {
        EventIdCount = 4
    };
    
    enum class StateId: uint8_t
    {
        ROOT = 0,
        ALARM_ACTIVE = 1,
        ALARM_COUNTDOWN = 2,
        ARMED = 3,
        ARMING = 4,
        DISARMED = 5,
    };
    
    enum
    {
        StateIdCount = 6
    };
    
    // Used internally by state machine. Feel free to inspect, but don't modify.
    StateId stateId;
    
    // State machine constructor. Must be called before start or dispatch event functions. Not thread safe.
    ControllerSm()
    {
    }
    
    // Starts the state machine. Must be called before dispatching events. Not thread safe.
    void start();
    
    // Dispatches an event to the state machine. Not thread safe.
    // Note! This function assumes that the `eventId` parameter is valid.
    void dispatchEvent(EventId eventId);
    
    // Thread safe.
    static char const * stateIdToString(StateId id);
    
    // Thread safe.
    static char const * eventIdToString(EventId id);


// ################################### PRIVATE MEMBERS ###################################
private:
    
    // This function is used when StateSmith doesn't know what the active leaf state is at
    // compile time due to sub states or when multiple states need to be exited.
    void exitUpToStateHandler(StateId desiredState);
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state ROOT
    ////////////////////////////////////////////////////////////////////////////////
    
    void ROOT_enter();
    
    void ROOT_alarm_now();
    
    void ROOT_arm();
    
    void ROOT_disarm();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state ALARM_ACTIVE
    ////////////////////////////////////////////////////////////////////////////////
    
    void ALARM_ACTIVE_enter();
    
    void ALARM_ACTIVE_exit();
    
    void ALARM_ACTIVE_disarm();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state ALARM_COUNTDOWN
    ////////////////////////////////////////////////////////////////////////////////
    
    void ALARM_COUNTDOWN_enter();
    
    void ALARM_COUNTDOWN_exit();
    
    void ALARM_COUNTDOWN_disarm();
    
    void ALARM_COUNTDOWN_do();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state ARMED
    ////////////////////////////////////////////////////////////////////////////////
    
    void ARMED_enter();
    
    void ARMED_exit();
    
    void ARMED_disarm();
    
    void ARMED_do();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state ARMING
    ////////////////////////////////////////////////////////////////////////////////
    
    void ARMING_enter();
    
    void ARMING_exit();
    
    void ARMING_disarm();
    
    void ARMING_do();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state DISARMED
    ////////////////////////////////////////////////////////////////////////////////
    
    void DISARMED_enter();
    
    void DISARMED_exit();
    
    void DISARMED_arm();
};


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/control/ControlData.hpp
//////////////////////////////////////////////////////////////////////////////////
// #pragma once
// #include "ControllerSm.hpp"
#include <stdint.h>

// fields in this structure are set by control system, cleared by UI.
struct ControlEvents {
    bool armed = false;
    bool alarm = false;
    bool disarmed = false;

    // example: arm/disarm attempt while already in that state
    bool event_ignored = false;
};

struct ControlData {
    ControllerSm::StateId state_id = ControllerSm::StateId::DISARMED;
    bool motion_detected = false;
    int32_t countdown_ms = 0; // larger than int16 so that it can extend past ~30 seconds
};


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/control/Controller.hpp
//////////////////////////////////////////////////////////////////////////////////
// #pragma once
// #include "../io_config.hpp"
// #include "ControlData.hpp"
// #include "ControllerSm.hpp"
#include <Arduino.h>
#include <stdint.h>

class Controller : protected ControllerSm {
public:
    Controller()
    {
    }

    void setup()
    {
        pinMode(IO_PIN_RED_LED, OUTPUT);
        pinMode(IO_PIN_MOTION_SENSOR, INPUT);

        start(); // start the state machine
    }

    void update(const uint32_t loop_delta_ms)
    {
        m_control_data.motion_detected = digitalRead(IO_PIN_MOTION_SENSOR) == HIGH;

        if (m_countdown_ms > 0) {
            m_countdown_ms -= loop_delta_ms;
        }

        dispatch_fsm_event(ControllerSm::EventId::DO);
    }

    void panic()
    {
        dispatch_fsm_event(ControllerSm::EventId::ALARM_NOW);
    }

    void arm()
    {
        dispatch_fsm_event(ControllerSm::EventId::ARM);
    }

    void disarm()
    {
        dispatch_fsm_event(ControllerSm::EventId::DISARM);
    }

    const ControlData& get_control_data()
    {
        return m_control_data;
    }

    ControlEvents& get_published_events()
    {
        return m_published_events;
    }

protected:
    int16_t m_countdown_ms = 0;
    ControlData m_control_data {};
    ControlEvents m_published_events {}; // set by control system, cleared by UI.

    void dispatch_fsm_event(ControllerSm::EventId eventId)
    {
        // print event name to console. Not the periodic DO event though. Too noisy.
        if (eventId != ControllerSm::EventId::DO) {
            Serial.print(F("CONTROL dispatching event: "));
            Serial.println(ControllerSm::eventIdToString(eventId));
        }

        const ControllerSm::StateId prev_state = stateId;
        dispatchEvent(eventId); // this method is inherited from ControllerSm
        const ControllerSm::StateId current_state = stateId;

        m_handle_state_change(prev_state, current_state);

        // publish data
        m_control_data.countdown_ms = m_countdown_ms;
        m_control_data.state_id = current_state;
    }

    void clear_countdown() override
    {
        m_countdown_ms = 0;
    }

    void init_arming_countdown() override
    {
        m_countdown_ms = 5 * 1000;
    }

    void init_alarm_countdown() override
    {
        m_countdown_ms = 5 * 1000;
    }

    bool is_countdown_complete() const override
    {
        return m_countdown_ms <= 0;
    }

    bool is_motion_detected() const override
    {
        return m_control_data.motion_detected;
    }

    void set_red_led(bool on) override
    {
        digitalWrite(IO_PIN_RED_LED, on ? HIGH : LOW);
    }

    void notify_event_ignored() override
    {
        m_published_events.event_ignored = true;
    }
    void notify_armed() override
    {
        m_published_events.armed = true;
    }
    void notify_disarmed() override
    {
        m_published_events.disarmed = true;
    }
    void notify_alarm() override
    {
        m_published_events.alarm = true;
    }

    void m_handle_state_change(const ControllerSm::StateId prev_state, const ControllerSm::StateId current_state)
    {
        if (prev_state == current_state) {
            return; // no state change
        }

        // state change detected

        // log console message
        Serial.print(F("CONTROL state changed from "));
        Serial.print(ControllerSm::stateIdToString(prev_state));
        Serial.print(F(" to "));
        Serial.println(ControllerSm::stateIdToString(current_state));
    }
};


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/serial_ui/SerialUi.hpp
//////////////////////////////////////////////////////////////////////////////////
// #pragma once
// #include "../control/Controller.hpp"
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


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/ui/UiSm.hpp
//////////////////////////////////////////////////////////////////////////////////
// Autogenerated with StateSmith 0.18.2+6062baabb038910ff3841c3cd5938115c0ed0a03.
// Algorithm: Balanced2. See https://github.com/StateSmith/StateSmith/wiki/Algorithms

// #pragma once  // You can also specify normal include guard. See https://github.com/StateSmith/StateSmith/blob/main/docs/settings.md
#include <stdint.h>
// #include "../audio/Audio.hpp"
// #include "../control/Controller.hpp"
#include <stdint.h>

// forward declare Display because of circular references. We store a pointer to it in the FSM variables section.
class Display;


// Generated state machine
class UiSm
{
public:
    enum class EventId: uint8_t
    {
        ANY_KEY = 0,
        BACK_HELD = 1,
        BACK_PRESS = 2,
        DOWN_PRESS = 3,
        ENTER_PRESS = 4,
        LCD_UPDATE = 5,
        PANIC_BUTTON = 6,
        REFRESH = 7,
        TIMEOUT = 8,
        UP_DOWN_HELD = 9,
        UP_PRESS = 10,
    };
    
    enum
    {
        EventIdCount = 11
    };
    
    enum class StateId: uint8_t
    {
        ROOT = 0,
        HOME = 1,
        MENU = 2,
        ARM_SYSTEM = 3,
        CONFIG = 4,
        ALARM_DELAY = 5,
        ARMING_DELAY = 6,
        CHANGE_CODE = 7,
        CONFIG__ALARM_DELAY = 8,
        CONFIG__ARMING_DELAY = 9,
        CONFIG__CHANGE_CODE = 10,
        SECRET_MENU = 11,
        DATA = 12,
        SENSOR = 13,
        UPTIME = 14,
        DISARM_SYSTEM = 15,
        MENU__ARM_SYSTEM = 16,
        MENU__CONFIG = 17,
        MENU__DATA = 18,
        MENU__DISARM_SYSTEM = 19,
        PANIC = 20,
        SPLASH_SCREEN = 21,
    };
    
    enum
    {
        StateIdCount = 22
    };
    
    // Used internally by state machine. Feel free to inspect, but don't modify.
    StateId stateId;
    
    // State machine variables. Can be used for inputs, outputs, user variables...
    class Vars
    {
    public:
        Display* lcd;      // pointer because StateSmith doesn't support references (yet!)
        Controller* ctrl;  // pointer because StateSmith doesn't support references (yet!)
        Audio* audio;      // pointer because StateSmith doesn't support references (yet!)
        int32_t timeout_ms; // Must be > 16 bits to allow for 60+ seconds.
        EventId event_id; // Optional. Manually set before dispatching event.
    };
    
    // Variables. Can be used for inputs, outputs, user variables...
    Vars vars {};
    
    // State machine constructor. Must be called before start or dispatch event functions. Not thread safe.
    UiSm()
    {
    }
    
    // Starts the state machine. Must be called before dispatching events. Not thread safe.
    void start();
    
    // Dispatches an event to the state machine. Not thread safe.
    // Note! This function assumes that the `eventId` parameter is valid.
    void dispatchEvent(EventId eventId);
    
    // Thread safe.
    static char const * stateIdToString(StateId id);
    
    // Thread safe.
    static char const * eventIdToString(EventId id);

private:
    static const int32_t DATA_TIMEOUT_MS = 60 * 1000L; // C99 transpiler doesn't support 'ClassCode' section.



// ################################### PRIVATE MEMBERS ###################################
private:
    
    // This function is used when StateSmith doesn't know what the active leaf state is at
    // compile time due to sub states or when multiple states need to be exited.
    void exitUpToStateHandler(StateId desiredState);
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state ROOT
    ////////////////////////////////////////////////////////////////////////////////
    
    void ROOT_enter();
    
    void ROOT_panic_button();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state HOME
    ////////////////////////////////////////////////////////////////////////////////
    
    void HOME_enter();
    
    void HOME_exit();
    
    void HOME_enter_press();
    
    void HOME_lcd_update();
    
    void HOME_refresh();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state MENU
    ////////////////////////////////////////////////////////////////////////////////
    
    void MENU_enter();
    
    void MENU_exit();
    
    void MENU_any_key();
    
    void MENU_back_held();
    
    void MENU_back_press();
    
    void MENU_timeout();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state ARM_SYSTEM
    ////////////////////////////////////////////////////////////////////////////////
    
    void ARM_SYSTEM_enter();
    
    void ARM_SYSTEM_exit();
    
    void ARM_SYSTEM_back_press();
    
    void ARM_SYSTEM_enter_press();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state CONFIG
    ////////////////////////////////////////////////////////////////////////////////
    
    void CONFIG_enter();
    
    void CONFIG_exit();
    
    void CONFIG_back_press();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state ALARM_DELAY
    ////////////////////////////////////////////////////////////////////////////////
    
    void ALARM_DELAY_enter();
    
    void ALARM_DELAY_exit();
    
    void ALARM_DELAY_back_press();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state ARMING_DELAY
    ////////////////////////////////////////////////////////////////////////////////
    
    void ARMING_DELAY_enter();
    
    void ARMING_DELAY_exit();
    
    void ARMING_DELAY_back_press();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state CHANGE_CODE
    ////////////////////////////////////////////////////////////////////////////////
    
    void CHANGE_CODE_enter();
    
    void CHANGE_CODE_exit();
    
    void CHANGE_CODE_back_press();
    
    void CHANGE_CODE_up_down_held();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state CONFIG__ALARM_DELAY
    ////////////////////////////////////////////////////////////////////////////////
    
    void CONFIG__ALARM_DELAY_enter();
    
    void CONFIG__ALARM_DELAY_exit();
    
    void CONFIG__ALARM_DELAY_enter_press();
    
    void CONFIG__ALARM_DELAY_up_press();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state CONFIG__ARMING_DELAY
    ////////////////////////////////////////////////////////////////////////////////
    
    void CONFIG__ARMING_DELAY_enter();
    
    void CONFIG__ARMING_DELAY_exit();
    
    void CONFIG__ARMING_DELAY_down_press();
    
    void CONFIG__ARMING_DELAY_enter_press();
    
    void CONFIG__ARMING_DELAY_up_press();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state CONFIG__CHANGE_CODE
    ////////////////////////////////////////////////////////////////////////////////
    
    void CONFIG__CHANGE_CODE_enter();
    
    void CONFIG__CHANGE_CODE_exit();
    
    void CONFIG__CHANGE_CODE_down_press();
    
    void CONFIG__CHANGE_CODE_enter_press();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state SECRET_MENU
    ////////////////////////////////////////////////////////////////////////////////
    
    void SECRET_MENU_enter();
    
    void SECRET_MENU_exit();
    
    void SECRET_MENU_back_press();
    
    void SECRET_MENU_down_press();
    
    void SECRET_MENU_enter_press();
    
    void SECRET_MENU_lcd_update();
    
    void SECRET_MENU_up_press();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state DATA
    ////////////////////////////////////////////////////////////////////////////////
    
    void DATA_enter();
    
    void DATA_exit();
    
    void DATA_any_key();
    
    void DATA_back_press();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state SENSOR
    ////////////////////////////////////////////////////////////////////////////////
    
    void SENSOR_enter();
    
    void SENSOR_exit();
    
    void SENSOR_lcd_update();
    
    void SENSOR_up_press();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state UPTIME
    ////////////////////////////////////////////////////////////////////////////////
    
    void UPTIME_enter();
    
    void UPTIME_exit();
    
    void UPTIME_down_press();
    
    void UPTIME_lcd_update();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state DISARM_SYSTEM
    ////////////////////////////////////////////////////////////////////////////////
    
    void DISARM_SYSTEM_enter();
    
    void DISARM_SYSTEM_exit();
    
    void DISARM_SYSTEM_back_press();
    
    void DISARM_SYSTEM_enter_press();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state MENU__ARM_SYSTEM
    ////////////////////////////////////////////////////////////////////////////////
    
    void MENU__ARM_SYSTEM_enter();
    
    void MENU__ARM_SYSTEM_exit();
    
    void MENU__ARM_SYSTEM_down_press();
    
    void MENU__ARM_SYSTEM_enter_press();
    
    void MENU__ARM_SYSTEM_up_press();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state MENU__CONFIG
    ////////////////////////////////////////////////////////////////////////////////
    
    void MENU__CONFIG_enter();
    
    void MENU__CONFIG_exit();
    
    void MENU__CONFIG_down_press();
    
    void MENU__CONFIG_enter_press();
    
    void MENU__CONFIG_up_press();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state MENU__DATA
    ////////////////////////////////////////////////////////////////////////////////
    
    void MENU__DATA_enter();
    
    void MENU__DATA_exit();
    
    void MENU__DATA_enter_press();
    
    void MENU__DATA_up_press();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state MENU__DISARM_SYSTEM
    ////////////////////////////////////////////////////////////////////////////////
    
    void MENU__DISARM_SYSTEM_enter();
    
    void MENU__DISARM_SYSTEM_exit();
    
    void MENU__DISARM_SYSTEM_down_press();
    
    void MENU__DISARM_SYSTEM_enter_press();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state PANIC
    ////////////////////////////////////////////////////////////////////////////////
    
    void PANIC_enter();
    
    void PANIC_exit();
    
    void PANIC_timeout();
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // event handlers for state SPLASH_SCREEN
    ////////////////////////////////////////////////////////////////////////////////
    
    void SPLASH_SCREEN_enter();
    
    void SPLASH_SCREEN_exit();
    
    void SPLASH_SCREEN_timeout();
};


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/ui/Display.hpp
//////////////////////////////////////////////////////////////////////////////////
// #pragma once
// #include "../control/ControlData.hpp"
// #include "../io_config.hpp"
// #include "UiSm.hpp"
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

    void SECRET_MENU()
    {
        update_special_char();
        fill_screen(SPECIAL_CHAR_CODE);
    }

    void fill_screen(char c)
    {
        for (uint8_t row = 0; row < LCD_ROWS; row++) {
            m_lcd.setCursor(0, row);
            for (uint8_t col = 0; col < LCD_COLUMNS; col++) {
                m_lcd.write(c);
            }
        }
    }

    void SECRET_MENU_update()
    {
        update_special_char();
    }

    void SECRET_MENU_button(UiSm::EventId button_event_id)
    {
        switch (button_event_id) {
        case UiSm::EventId::BACK_PRESS:
            m_custom_char = 0;
            break;

        case UiSm::EventId::UP_PRESS:
            m_custom_char++;
            break;

        case UiSm::EventId::DOWN_PRESS:
            m_custom_char--;
            break;

        default:
        case UiSm::EventId::ENTER_PRESS:
            m_custom_char ^= 0b01010101;
            break;
        }
    }

    void update_special_char()
    {
        uint8_t pixels[PIXEL_ROWS] = {};

        const uint8_t last_bit = (m_custom_char >> 7);
        m_custom_char <<= 1;
        m_custom_char |= (last_bit ^ 0b1);

        for (uint8_t row = 0; row < PIXEL_ROWS; row++) {
            pixels[row] = m_custom_char;
        }

        m_lcd.createChar(SPECIAL_CHAR_CODE, pixels);
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


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/ui/Ui.hpp
//////////////////////////////////////////////////////////////////////////////////
// #pragma once
// #include "../audio/Audio.hpp"
// #include "Display.hpp"
// #include "UiSm.hpp"
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


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/buttons/ButtonSm.h
//////////////////////////////////////////////////////////////////////////////////
// Autogenerated with StateSmith 0.18.2+6062baabb038910ff3841c3cd5938115c0ed0a03.
// Algorithm: Balanced2. See https://github.com/StateSmith/StateSmith/wiki/Algorithms

// #pragma once  // You can also specify normal include guard. See https://github.com/StateSmith/StateSmith/blob/main/docs/settings.md
#include <stdint.h>
#include <stdint.h> // for fixed width integer state machine variables below



#ifdef __cplusplus
extern "C" {
#endif

typedef enum __attribute__((packed)) ButtonSm_EventId
{
    ButtonSm_EventId_DO = 0 // The `do` event is special. State event handlers do not consume this event (ancestors all get it too) unless a transition occurs.
} ButtonSm_EventId;

enum
{
    ButtonSm_EventIdCount = 1
};

typedef enum __attribute__((packed)) ButtonSm_StateId
{
    ButtonSm_StateId_ROOT = 0,
    ButtonSm_StateId_PRESSED_DEBOUNCE = 1,
    ButtonSm_StateId_PRESSED_STABLE = 2,
    ButtonSm_StateId_CONFIRM_LONG = 3,
    ButtonSm_StateId_PRESSED_LONG = 4,
    ButtonSm_StateId_RELEASED_DEBOUNCE = 5,
    ButtonSm_StateId_RELEASED_STABLE = 6
} ButtonSm_StateId;

enum
{
    ButtonSm_StateIdCount = 7
};


// Generated state machine
// forward declaration
typedef struct ButtonSm ButtonSm;

// State machine variables. Can be used for inputs, outputs, user variables...
typedef struct ButtonSm_Vars
{
    // ----- INPUT/OUTPUT VARIABLES -----
    // You **ADD** your loop delta time (ms since last loop) to this variable so it can be used for timing.
    uint16_t timer_ms;
    
    // ----- INPUT VARIABLES -----
    // You **SET** this to 1 when button is detected as active, and 0 when inactive.
    uint8_t input_active: 1;
    
    // ----- OUTPUT EVENT VARIABLES -----
    // You **READ** and **CLEAR** event flags in using code.
    uint8_t release_event: 1;
    uint8_t press_event: 1;
    uint8_t long_event: 1;
    
    // ----- OUTPUT STATUS VARIABLES -----
    // You **READ** these flags in using code, but don't clear them.
    uint8_t press_status: 1;
    uint8_t long_status: 1;
} ButtonSm_Vars;


// State machine constructor. Must be called before start or dispatch event functions. Not thread safe.
void ButtonSm_ctor(ButtonSm* sm);

// Starts the state machine. Must be called before dispatching events. Not thread safe.
void ButtonSm_start(ButtonSm* sm);

// Dispatches an event to the state machine. Not thread safe.
// Note! This function assumes that the `event_id` parameter is valid.
void ButtonSm_dispatch_event(ButtonSm* sm, ButtonSm_EventId event_id);

// Thread safe.
char const * ButtonSm_state_id_to_string(ButtonSm_StateId id);

// Thread safe.
char const * ButtonSm_event_id_to_string(ButtonSm_EventId id);

// Generated state machine
struct ButtonSm
{
    // Used internally by state machine. Feel free to inspect, but don't modify.
    ButtonSm_StateId state_id;
    
    // Variables. Can be used for inputs, outputs, user variables...
    ButtonSm_Vars vars;
};


#ifdef __cplusplus
}  // extern "C"
#endif



//////////////////////////////////////////////////////////////////////////////////
// FILE: src/buttons/ButtonSm.c
//////////////////////////////////////////////////////////////////////////////////
// Autogenerated with StateSmith 0.18.2+6062baabb038910ff3841c3cd5938115c0ed0a03.
// Algorithm: Balanced2. See https://github.com/StateSmith/StateSmith/wiki/Algorithms

// #include "ButtonSm.h"
#include <stdbool.h> // required for `consume_event` flag
#include <string.h> // for memset

// This function is used when StateSmith doesn't know what the active leaf state is at
// compile time due to sub states or when multiple states need to be exited.
static void exit_up_to_state_handler(ButtonSm* sm, ButtonSm_StateId desired_state);

static void ROOT_enter(ButtonSm* sm);

static void PRESSED_DEBOUNCE_enter(ButtonSm* sm);

static void PRESSED_DEBOUNCE_exit(ButtonSm* sm);

static void PRESSED_DEBOUNCE_do(ButtonSm* sm);

static void PRESSED_STABLE_enter(ButtonSm* sm);

static void PRESSED_STABLE_exit(ButtonSm* sm);

static void PRESSED_STABLE_do(ButtonSm* sm);

static void CONFIRM_LONG_enter(ButtonSm* sm);

static void CONFIRM_LONG_exit(ButtonSm* sm);

static void CONFIRM_LONG_do(ButtonSm* sm);

static void PRESSED_LONG_enter(ButtonSm* sm);

static void PRESSED_LONG_exit(ButtonSm* sm);

static void RELEASED_DEBOUNCE_enter(ButtonSm* sm);

static void RELEASED_DEBOUNCE_exit(ButtonSm* sm);

static void RELEASED_DEBOUNCE_do(ButtonSm* sm);

static void RELEASED_STABLE_enter(ButtonSm* sm);

static void RELEASED_STABLE_exit(ButtonSm* sm);

static void RELEASED_STABLE_do(ButtonSm* sm);


// State machine constructor. Must be called before start or dispatch event functions. Not thread safe.
void ButtonSm_ctor(ButtonSm* sm)
{
    memset(sm, 0, sizeof(*sm));
}

// Starts the state machine. Must be called before dispatching events. Not thread safe.
void ButtonSm_start(ButtonSm* sm)
{
    ROOT_enter(sm);
    // ROOT behavior
    // uml: TransitionTo(ROOT.<InitialState>)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition). Already at LCA, no exiting required.
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `ROOT.<InitialState>`.
        // ROOT.<InitialState> is a pseudo state and cannot have an `enter` trigger.
        
        // ROOT.<InitialState> behavior
        // uml: TransitionTo(RELEASED_DEBOUNCE)
        {
            // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition). Already at LCA, no exiting required.
            
            // Step 2: Transition action: ``.
            
            // Step 3: Enter/move towards transition target `RELEASED_DEBOUNCE`.
            RELEASED_DEBOUNCE_enter(sm);
            
            // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
            return;
        } // end of behavior for ROOT.<InitialState>
    } // end of behavior for ROOT
}

// Dispatches an event to the state machine. Not thread safe.
// Note! This function assumes that the `event_id` parameter is valid.
void ButtonSm_dispatch_event(ButtonSm* sm, ButtonSm_EventId event_id)
{
    // This state machine design only has a single event type so we can safely assume
    // that the dispatched event is `do` without checking the `event_id` parameter.
    (void)event_id; // This line prevents an 'unused variable' compiler warning
    
    switch (sm->state_id)
    {
        // STATE: ButtonSm
        case ButtonSm_StateId_ROOT:
            // state and ancestors have no handler for `do` event.
            break;
        
        // STATE: PRESSED_DEBOUNCE
        case ButtonSm_StateId_PRESSED_DEBOUNCE:
            PRESSED_DEBOUNCE_do(sm); 
            break;
        
        // STATE: PRESSED_STABLE
        case ButtonSm_StateId_PRESSED_STABLE:
            PRESSED_STABLE_do(sm); 
            break;
        
        // STATE: CONFIRM_LONG
        case ButtonSm_StateId_CONFIRM_LONG:
            CONFIRM_LONG_do(sm); 
            break;
        
        // STATE: PRESSED_LONG
        case ButtonSm_StateId_PRESSED_LONG:
            PRESSED_STABLE_do(sm);  // First ancestor handler for this event

            break;
        
        // STATE: RELEASED_DEBOUNCE
        case ButtonSm_StateId_RELEASED_DEBOUNCE:
            RELEASED_DEBOUNCE_do(sm); 
            break;
        
        // STATE: RELEASED_STABLE
        case ButtonSm_StateId_RELEASED_STABLE:
            RELEASED_STABLE_do(sm); 
            break;
    }
    
}

// This function is used when StateSmith doesn't know what the active leaf state is at
// compile time due to sub states or when multiple states need to be exited.
static void exit_up_to_state_handler(ButtonSm* sm, ButtonSm_StateId desired_state)
{
    while (sm->state_id != desired_state)
    {
        switch (sm->state_id)
        {
            case ButtonSm_StateId_PRESSED_DEBOUNCE: PRESSED_DEBOUNCE_exit(sm); break;
            
            case ButtonSm_StateId_PRESSED_STABLE: PRESSED_STABLE_exit(sm); break;
            
            case ButtonSm_StateId_CONFIRM_LONG: CONFIRM_LONG_exit(sm); break;
            
            case ButtonSm_StateId_PRESSED_LONG: PRESSED_LONG_exit(sm); break;
            
            case ButtonSm_StateId_RELEASED_DEBOUNCE: RELEASED_DEBOUNCE_exit(sm); break;
            
            case ButtonSm_StateId_RELEASED_STABLE: RELEASED_STABLE_exit(sm); break;
            
            default: return;  // Just to be safe. Prevents infinite loop if state ID memory is somehow corrupted.
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state ROOT
////////////////////////////////////////////////////////////////////////////////

static void ROOT_enter(ButtonSm* sm)
{
    sm->state_id = ButtonSm_StateId_ROOT;
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state PRESSED_DEBOUNCE
////////////////////////////////////////////////////////////////////////////////

static void PRESSED_DEBOUNCE_enter(ButtonSm* sm)
{
    sm->state_id = ButtonSm_StateId_PRESSED_DEBOUNCE;
    
    // PRESSED_DEBOUNCE behavior
    // uml: enter / { timer_ms = 0; }
    {
        // Step 1: execute action `timer_ms = 0;`
        sm->vars.timer_ms = 0;
    } // end of behavior for PRESSED_DEBOUNCE
    
    // PRESSED_DEBOUNCE behavior
    // uml: enter / { press_status = true; }
    {
        // Step 1: execute action `press_status = true;`
        sm->vars.press_status = true;
    } // end of behavior for PRESSED_DEBOUNCE
    
    // PRESSED_DEBOUNCE behavior
    // uml: enter / { press_event = true; }
    {
        // Step 1: execute action `press_event = true;`
        sm->vars.press_event = true;
    } // end of behavior for PRESSED_DEBOUNCE
}

static void PRESSED_DEBOUNCE_exit(ButtonSm* sm)
{
    sm->state_id = ButtonSm_StateId_ROOT;
}

static void PRESSED_DEBOUNCE_do(ButtonSm* sm)
{
    // PRESSED_DEBOUNCE behavior
    // uml: do [timer_ms > 50] TransitionTo(PRESSED_STABLE)
    if (sm->vars.timer_ms > 50)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        PRESSED_DEBOUNCE_exit(sm);
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `PRESSED_STABLE`.
        PRESSED_STABLE_enter(sm);
        
        // PRESSED_STABLE.<InitialState> behavior
        // uml: TransitionTo(CONFIRM_LONG)
        {
            // Step 1: Exit states until we reach `PRESSED_STABLE` state (Least Common Ancestor for transition). Already at LCA, no exiting required.
            
            // Step 2: Transition action: ``.
            
            // Step 3: Enter/move towards transition target `CONFIRM_LONG`.
            CONFIRM_LONG_enter(sm);
            
            // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
            return;
        } // end of behavior for PRESSED_STABLE.<InitialState>
    } // end of behavior for PRESSED_DEBOUNCE
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state PRESSED_STABLE
////////////////////////////////////////////////////////////////////////////////

static void PRESSED_STABLE_enter(ButtonSm* sm)
{
    sm->state_id = ButtonSm_StateId_PRESSED_STABLE;
}

static void PRESSED_STABLE_exit(ButtonSm* sm)
{
    sm->state_id = ButtonSm_StateId_ROOT;
}

static void PRESSED_STABLE_do(ButtonSm* sm)
{
    // PRESSED_STABLE behavior
    // uml: do [! input_active] / { release_event = true; } TransitionTo(RELEASED_DEBOUNCE)
    if (! sm->vars.input_active)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        exit_up_to_state_handler(sm, ButtonSm_StateId_ROOT);
        
        // Step 2: Transition action: `release_event = true;`.
        sm->vars.release_event = true;
        
        // Step 3: Enter/move towards transition target `RELEASED_DEBOUNCE`.
        RELEASED_DEBOUNCE_enter(sm);
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for PRESSED_STABLE
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state CONFIRM_LONG
////////////////////////////////////////////////////////////////////////////////

static void CONFIRM_LONG_enter(ButtonSm* sm)
{
    sm->state_id = ButtonSm_StateId_CONFIRM_LONG;
}

static void CONFIRM_LONG_exit(ButtonSm* sm)
{
    sm->state_id = ButtonSm_StateId_PRESSED_STABLE;
}

static void CONFIRM_LONG_do(ButtonSm* sm)
{
    bool consume_event = false;
    
    // CONFIRM_LONG behavior
    // uml: do [timer_ms > 1000] TransitionTo(PRESSED_LONG)
    if (sm->vars.timer_ms > 1000)
    {
        // Step 1: Exit states until we reach `PRESSED_STABLE` state (Least Common Ancestor for transition).
        CONFIRM_LONG_exit(sm);
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `PRESSED_LONG`.
        PRESSED_LONG_enter(sm);
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for CONFIRM_LONG
    
    // Check if event has been consumed before calling ancestor handler.
    if (!consume_event)
    {
        PRESSED_STABLE_do(sm);
    }
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state PRESSED_LONG
////////////////////////////////////////////////////////////////////////////////

static void PRESSED_LONG_enter(ButtonSm* sm)
{
    sm->state_id = ButtonSm_StateId_PRESSED_LONG;
    
    // PRESSED_LONG behavior
    // uml: enter / { long_event = true; }
    {
        // Step 1: execute action `long_event = true;`
        sm->vars.long_event = true;
    } // end of behavior for PRESSED_LONG
    
    // PRESSED_LONG behavior
    // uml: enter / { long_status = true; }
    {
        // Step 1: execute action `long_status = true;`
        sm->vars.long_status = true;
    } // end of behavior for PRESSED_LONG
}

static void PRESSED_LONG_exit(ButtonSm* sm)
{
    // PRESSED_LONG behavior
    // uml: exit / { long_status = false; }
    {
        // Step 1: execute action `long_status = false;`
        sm->vars.long_status = false;
    } // end of behavior for PRESSED_LONG
    
    sm->state_id = ButtonSm_StateId_PRESSED_STABLE;
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state RELEASED_DEBOUNCE
////////////////////////////////////////////////////////////////////////////////

static void RELEASED_DEBOUNCE_enter(ButtonSm* sm)
{
    sm->state_id = ButtonSm_StateId_RELEASED_DEBOUNCE;
    
    // RELEASED_DEBOUNCE behavior
    // uml: enter / { timer_ms = 0; }
    {
        // Step 1: execute action `timer_ms = 0;`
        sm->vars.timer_ms = 0;
    } // end of behavior for RELEASED_DEBOUNCE
    
    // RELEASED_DEBOUNCE behavior
    // uml: enter / { press_status = false; }
    {
        // Step 1: execute action `press_status = false;`
        sm->vars.press_status = false;
    } // end of behavior for RELEASED_DEBOUNCE
}

static void RELEASED_DEBOUNCE_exit(ButtonSm* sm)
{
    sm->state_id = ButtonSm_StateId_ROOT;
}

static void RELEASED_DEBOUNCE_do(ButtonSm* sm)
{
    // RELEASED_DEBOUNCE behavior
    // uml: do [timer_ms > 50] TransitionTo(RELEASED_STABLE)
    if (sm->vars.timer_ms > 50)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        RELEASED_DEBOUNCE_exit(sm);
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `RELEASED_STABLE`.
        RELEASED_STABLE_enter(sm);
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for RELEASED_DEBOUNCE
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state RELEASED_STABLE
////////////////////////////////////////////////////////////////////////////////

static void RELEASED_STABLE_enter(ButtonSm* sm)
{
    sm->state_id = ButtonSm_StateId_RELEASED_STABLE;
}

static void RELEASED_STABLE_exit(ButtonSm* sm)
{
    sm->state_id = ButtonSm_StateId_ROOT;
}

static void RELEASED_STABLE_do(ButtonSm* sm)
{
    // RELEASED_STABLE behavior
    // uml: do [input_active] TransitionTo(PRESSED_DEBOUNCE)
    if (sm->vars.input_active)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        RELEASED_STABLE_exit(sm);
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `PRESSED_DEBOUNCE`.
        PRESSED_DEBOUNCE_enter(sm);
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for RELEASED_STABLE
    
    // No ancestor handles this event.
}

// Thread safe.
char const * ButtonSm_state_id_to_string(ButtonSm_StateId id)
{
    switch (id)
    {
        case ButtonSm_StateId_ROOT: return "ROOT";
        case ButtonSm_StateId_PRESSED_DEBOUNCE: return "PRESSED_DEBOUNCE";
        case ButtonSm_StateId_PRESSED_STABLE: return "PRESSED_STABLE";
        case ButtonSm_StateId_CONFIRM_LONG: return "CONFIRM_LONG";
        case ButtonSm_StateId_PRESSED_LONG: return "PRESSED_LONG";
        case ButtonSm_StateId_RELEASED_DEBOUNCE: return "RELEASED_DEBOUNCE";
        case ButtonSm_StateId_RELEASED_STABLE: return "RELEASED_STABLE";
        default: return "?";
    }
}

// Thread safe.
char const * ButtonSm_event_id_to_string(ButtonSm_EventId id)
{
    switch (id)
    {
        case ButtonSm_EventId_DO: return "DO";
        default: return "?";
    }
}


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/buttons/MyArduinoButton.hpp
//////////////////////////////////////////////////////////////////////////////////
// #pragma once
// #include "ButtonSm.h"
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


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/buttons/Buttons.hpp
//////////////////////////////////////////////////////////////////////////////////
// #pragma once
// #include "../io_config.hpp"
// #include "../ui/Ui.hpp"
// #include "MyArduinoButton.hpp"
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


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/buttons/Buttons.cpp
//////////////////////////////////////////////////////////////////////////////////
// #include "Buttons.hpp"

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


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/control/ControllerSm.cpp
//////////////////////////////////////////////////////////////////////////////////
// Autogenerated with StateSmith 0.18.2+6062baabb038910ff3841c3cd5938115c0ed0a03.
// Algorithm: Balanced2. See https://github.com/StateSmith/StateSmith/wiki/Algorithms

// Whatever you put in this `FileTop` section will end up 
// being printed at the top of every generated code file.

// #include "ControllerSm.hpp"
#include <stdbool.h> // required for `consume_event` flag
#include <string.h> // for memset
// // #include "your_header_here.h"



// Starts the state machine. Must be called before dispatching events. Not thread safe.
void ControllerSm::start()
{
    ROOT_enter();
    // ROOT behavior
    // uml: TransitionTo(ROOT.<InitialState>)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition). Already at LCA, no exiting required.
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `ROOT.<InitialState>`.
        // ROOT.<InitialState> is a pseudo state and cannot have an `enter` trigger.
        
        // ROOT.<InitialState> behavior
        // uml: TransitionTo(DISARMED)
        {
            // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition). Already at LCA, no exiting required.
            
            // Step 2: Transition action: ``.
            
            // Step 3: Enter/move towards transition target `DISARMED`.
            DISARMED_enter();
            
            // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
            return;
        } // end of behavior for ROOT.<InitialState>
    } // end of behavior for ROOT
}

// Dispatches an event to the state machine. Not thread safe.
// Note! This function assumes that the `eventId` parameter is valid.
void ControllerSm::dispatchEvent(EventId eventId)
{
    switch (this->stateId)
    {
        // STATE: ControllerSm
        case StateId::ROOT:
            switch (eventId)
            {
                case EventId::ALARM_NOW: ROOT_alarm_now(); break;
                case EventId::ARM: ROOT_arm(); break;
                case EventId::DISARM: ROOT_disarm(); break;
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: ALARM_ACTIVE
        case StateId::ALARM_ACTIVE:
            switch (eventId)
            {
                case EventId::DISARM: ALARM_ACTIVE_disarm(); break;
                case EventId::ALARM_NOW: ROOT_alarm_now(); break; // First ancestor handler for this event
                case EventId::ARM: ROOT_arm(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: ALARM_COUNTDOWN
        case StateId::ALARM_COUNTDOWN:
            switch (eventId)
            {
                case EventId::DO: ALARM_COUNTDOWN_do(); break;
                case EventId::DISARM: ALARM_COUNTDOWN_disarm(); break;
                case EventId::ALARM_NOW: ROOT_alarm_now(); break; // First ancestor handler for this event
                case EventId::ARM: ROOT_arm(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: ARMED
        case StateId::ARMED:
            switch (eventId)
            {
                case EventId::DO: ARMED_do(); break;
                case EventId::DISARM: ARMED_disarm(); break;
                case EventId::ALARM_NOW: ROOT_alarm_now(); break; // First ancestor handler for this event
                case EventId::ARM: ROOT_arm(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: ARMING
        case StateId::ARMING:
            switch (eventId)
            {
                case EventId::DO: ARMING_do(); break;
                case EventId::DISARM: ARMING_disarm(); break;
                case EventId::ALARM_NOW: ROOT_alarm_now(); break; // First ancestor handler for this event
                case EventId::ARM: ROOT_arm(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: DISARMED
        case StateId::DISARMED:
            switch (eventId)
            {
                case EventId::ARM: DISARMED_arm(); break;
                case EventId::ALARM_NOW: ROOT_alarm_now(); break; // First ancestor handler for this event
                case EventId::DISARM: ROOT_disarm(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
    }
    
}

// This function is used when StateSmith doesn't know what the active leaf state is at
// compile time due to sub states or when multiple states need to be exited.
void ControllerSm::exitUpToStateHandler(StateId desiredState)
{
    while (this->stateId != desiredState)
    {
        switch (this->stateId)
        {
            case StateId::ALARM_ACTIVE: ALARM_ACTIVE_exit(); break;
            
            case StateId::ALARM_COUNTDOWN: ALARM_COUNTDOWN_exit(); break;
            
            case StateId::ARMED: ARMED_exit(); break;
            
            case StateId::ARMING: ARMING_exit(); break;
            
            case StateId::DISARMED: DISARMED_exit(); break;
            
            default: return;  // Just to be safe. Prevents infinite loop if state ID memory is somehow corrupted.
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state ROOT
////////////////////////////////////////////////////////////////////////////////

void ControllerSm::ROOT_enter()
{
    this->stateId = StateId::ROOT;
}

void ControllerSm::ROOT_alarm_now()
{
    // ROOT behavior
    // uml: (ALARM_NOW, ARM, DISARM) / { notify_event_ignored(); }
    {
        // Step 1: execute action `notify_event_ignored();`
        notify_event_ignored();
    } // end of behavior for ROOT
    
    // ROOT behavior
    // uml: ALARM_NOW TransitionTo(ALARM_ACTIVE)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        exitUpToStateHandler(StateId::ROOT);
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `ALARM_ACTIVE`.
        ALARM_ACTIVE_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for ROOT
    
    // No ancestor handles this event.
}

void ControllerSm::ROOT_arm()
{
    // ROOT behavior
    // uml: (ALARM_NOW, ARM, DISARM) / { notify_event_ignored(); }
    {
        // Step 1: execute action `notify_event_ignored();`
        notify_event_ignored();
    } // end of behavior for ROOT
    
    // No ancestor handles this event.
}

void ControllerSm::ROOT_disarm()
{
    // ROOT behavior
    // uml: (ALARM_NOW, ARM, DISARM) / { notify_event_ignored(); }
    {
        // Step 1: execute action `notify_event_ignored();`
        notify_event_ignored();
    } // end of behavior for ROOT
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state ALARM_ACTIVE
////////////////////////////////////////////////////////////////////////////////

void ControllerSm::ALARM_ACTIVE_enter()
{
    this->stateId = StateId::ALARM_ACTIVE;
    
    // ALARM_ACTIVE behavior
    // uml: enter / { set_red_led(true); }
    {
        // Step 1: execute action `set_red_led(true);`
        set_red_led(true);
    } // end of behavior for ALARM_ACTIVE
    
    // ALARM_ACTIVE behavior
    // uml: enter / { notify_alarm(); }
    {
        // Step 1: execute action `notify_alarm();`
        notify_alarm();
    } // end of behavior for ALARM_ACTIVE
}

void ControllerSm::ALARM_ACTIVE_exit()
{
    // ALARM_ACTIVE behavior
    // uml: exit / { set_red_led(false); }
    {
        // Step 1: execute action `set_red_led(false);`
        set_red_led(false);
    } // end of behavior for ALARM_ACTIVE
    
    this->stateId = StateId::ROOT;
}

void ControllerSm::ALARM_ACTIVE_disarm()
{
    bool consume_event = false;
    
    // ALARM_ACTIVE behavior
    // uml: DISARM TransitionTo(DISARMED)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        ALARM_ACTIVE_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `DISARMED`.
        DISARMED_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for ALARM_ACTIVE
    
    // Check if event has been consumed before calling ancestor handler.
    if (!consume_event)
    {
        ROOT_disarm();
    }
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state ALARM_COUNTDOWN
////////////////////////////////////////////////////////////////////////////////

void ControllerSm::ALARM_COUNTDOWN_enter()
{
    this->stateId = StateId::ALARM_COUNTDOWN;
    
    // ALARM_COUNTDOWN behavior
    // uml: enter / { init_alarm_countdown(); }
    {
        // Step 1: execute action `init_alarm_countdown();`
        init_alarm_countdown();
    } // end of behavior for ALARM_COUNTDOWN
}

void ControllerSm::ALARM_COUNTDOWN_exit()
{
    // ALARM_COUNTDOWN behavior
    // uml: exit / { clear_countdown(); }
    {
        // Step 1: execute action `clear_countdown();`
        clear_countdown();
    } // end of behavior for ALARM_COUNTDOWN
    
    this->stateId = StateId::ROOT;
}

void ControllerSm::ALARM_COUNTDOWN_disarm()
{
    bool consume_event = false;
    
    // ALARM_COUNTDOWN behavior
    // uml: DISARM TransitionTo(DISARMED)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        ALARM_COUNTDOWN_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `DISARMED`.
        DISARMED_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for ALARM_COUNTDOWN
    
    // Check if event has been consumed before calling ancestor handler.
    if (!consume_event)
    {
        ROOT_disarm();
    }
}

void ControllerSm::ALARM_COUNTDOWN_do()
{
    // ALARM_COUNTDOWN behavior
    // uml: do [is_countdown_complete()] TransitionTo(ALARM_ACTIVE)
    if (is_countdown_complete())
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        ALARM_COUNTDOWN_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `ALARM_ACTIVE`.
        ALARM_ACTIVE_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for ALARM_COUNTDOWN
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state ARMED
////////////////////////////////////////////////////////////////////////////////

void ControllerSm::ARMED_enter()
{
    this->stateId = StateId::ARMED;
    
    // ARMED behavior
    // uml: enter / { notify_armed(); }
    {
        // Step 1: execute action `notify_armed();`
        notify_armed();
    } // end of behavior for ARMED
}

void ControllerSm::ARMED_exit()
{
    this->stateId = StateId::ROOT;
}

void ControllerSm::ARMED_disarm()
{
    bool consume_event = false;
    
    // ARMED behavior
    // uml: DISARM TransitionTo(DISARMED)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        ARMED_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `DISARMED`.
        DISARMED_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for ARMED
    
    // Check if event has been consumed before calling ancestor handler.
    if (!consume_event)
    {
        ROOT_disarm();
    }
}

void ControllerSm::ARMED_do()
{
    // ARMED behavior
    // uml: do [is_motion_detected()] TransitionTo(ALARM_COUNTDOWN)
    if (is_motion_detected())
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        ARMED_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `ALARM_COUNTDOWN`.
        ALARM_COUNTDOWN_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for ARMED
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state ARMING
////////////////////////////////////////////////////////////////////////////////

void ControllerSm::ARMING_enter()
{
    this->stateId = StateId::ARMING;
    
    // ARMING behavior
    // uml: enter / { init_arming_countdown(); }
    {
        // Step 1: execute action `init_arming_countdown();`
        init_arming_countdown();
    } // end of behavior for ARMING
}

void ControllerSm::ARMING_exit()
{
    // ARMING behavior
    // uml: exit / { clear_countdown(); }
    {
        // Step 1: execute action `clear_countdown();`
        clear_countdown();
    } // end of behavior for ARMING
    
    this->stateId = StateId::ROOT;
}

void ControllerSm::ARMING_disarm()
{
    bool consume_event = false;
    
    // ARMING behavior
    // uml: DISARM TransitionTo(DISARMED)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        ARMING_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `DISARMED`.
        DISARMED_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for ARMING
    
    // Check if event has been consumed before calling ancestor handler.
    if (!consume_event)
    {
        ROOT_disarm();
    }
}

void ControllerSm::ARMING_do()
{
    // ARMING behavior
    // uml: do [is_countdown_complete()] TransitionTo(ARMED)
    if (is_countdown_complete())
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        ARMING_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `ARMED`.
        ARMED_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for ARMING
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state DISARMED
////////////////////////////////////////////////////////////////////////////////

void ControllerSm::DISARMED_enter()
{
    this->stateId = StateId::DISARMED;
    
    // DISARMED behavior
    // uml: enter / { notify_disarmed(); }
    {
        // Step 1: execute action `notify_disarmed();`
        notify_disarmed();
    } // end of behavior for DISARMED
}

void ControllerSm::DISARMED_exit()
{
    this->stateId = StateId::ROOT;
}

void ControllerSm::DISARMED_arm()
{
    bool consume_event = false;
    
    // DISARMED behavior
    // uml: ARM TransitionTo(ARMING)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        DISARMED_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `ARMING`.
        ARMING_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for DISARMED
    
    // Check if event has been consumed before calling ancestor handler.
    if (!consume_event)
    {
        ROOT_arm();
    }
}

// Thread safe.
char const * ControllerSm::stateIdToString(StateId id)
{
    switch (id)
    {
        case StateId::ROOT: return "ROOT";
        case StateId::ALARM_ACTIVE: return "ALARM_ACTIVE";
        case StateId::ALARM_COUNTDOWN: return "ALARM_COUNTDOWN";
        case StateId::ARMED: return "ARMED";
        case StateId::ARMING: return "ARMING";
        case StateId::DISARMED: return "DISARMED";
        default: return "?";
    }
}

// Thread safe.
char const * ControllerSm::eventIdToString(EventId id)
{
    switch (id)
    {
        case EventId::ALARM_NOW: return "ALARM_NOW";
        case EventId::ARM: return "ARM";
        case EventId::DISARM: return "DISARM";
        case EventId::DO: return "DO";
        default: return "?";
    }
}


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/ui/UiSm.cpp
//////////////////////////////////////////////////////////////////////////////////
// Autogenerated with StateSmith 0.18.2+6062baabb038910ff3841c3cd5938115c0ed0a03.
// Algorithm: Balanced2. See https://github.com/StateSmith/StateSmith/wiki/Algorithms

// #include "UiSm.hpp"
#include <stdbool.h> // required for `consume_event` flag
#include <string.h> // for memset
// #include "Display.hpp"
#define IDLE_TIMEOUT_MS (10 * 1000) // this works for C99 as well as C++11



// Starts the state machine. Must be called before dispatching events. Not thread safe.
void UiSm::start()
{
    ROOT_enter();
    // ROOT behavior
    // uml: TransitionTo(ROOT.<InitialState>)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition). Already at LCA, no exiting required.
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `ROOT.<InitialState>`.
        // ROOT.<InitialState> is a pseudo state and cannot have an `enter` trigger.
        
        // ROOT.<InitialState> behavior
        // uml: TransitionTo(SPLASH_SCREEN)
        {
            // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition). Already at LCA, no exiting required.
            
            // Step 2: Transition action: ``.
            
            // Step 3: Enter/move towards transition target `SPLASH_SCREEN`.
            SPLASH_SCREEN_enter();
            
            // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
            return;
        } // end of behavior for ROOT.<InitialState>
    } // end of behavior for ROOT
}

// Dispatches an event to the state machine. Not thread safe.
// Note! This function assumes that the `eventId` parameter is valid.
void UiSm::dispatchEvent(EventId eventId)
{
    switch (this->stateId)
    {
        // STATE: UiSm
        case StateId::ROOT:
            switch (eventId)
            {
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break;
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: HOME
        case StateId::HOME:
            switch (eventId)
            {
                case EventId::REFRESH: HOME_refresh(); break;
                case EventId::LCD_UPDATE: HOME_lcd_update(); break;
                case EventId::ENTER_PRESS: HOME_enter_press(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: MENU
        case StateId::MENU:
            switch (eventId)
            {
                case EventId::ANY_KEY: MENU_any_key(); break;
                case EventId::BACK_PRESS: MENU_back_press(); break;
                case EventId::BACK_HELD: MENU_back_held(); break;
                case EventId::TIMEOUT: MENU_timeout(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: ARM_SYSTEM
        case StateId::ARM_SYSTEM:
            switch (eventId)
            {
                case EventId::BACK_PRESS: ARM_SYSTEM_back_press(); break;
                case EventId::ENTER_PRESS: ARM_SYSTEM_enter_press(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                case EventId::ANY_KEY: MENU_any_key(); break; // First ancestor handler for this event
                case EventId::BACK_HELD: MENU_back_held(); break; // First ancestor handler for this event
                case EventId::TIMEOUT: MENU_timeout(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: CONFIG
        case StateId::CONFIG:
            switch (eventId)
            {
                case EventId::BACK_PRESS: CONFIG_back_press(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                case EventId::ANY_KEY: MENU_any_key(); break; // First ancestor handler for this event
                case EventId::BACK_HELD: MENU_back_held(); break; // First ancestor handler for this event
                case EventId::TIMEOUT: MENU_timeout(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: ALARM_DELAY
        case StateId::ALARM_DELAY:
            switch (eventId)
            {
                case EventId::BACK_PRESS: ALARM_DELAY_back_press(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                case EventId::ANY_KEY: MENU_any_key(); break; // First ancestor handler for this event
                case EventId::BACK_HELD: MENU_back_held(); break; // First ancestor handler for this event
                case EventId::TIMEOUT: MENU_timeout(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: ARMING_DELAY
        case StateId::ARMING_DELAY:
            switch (eventId)
            {
                case EventId::BACK_PRESS: ARMING_DELAY_back_press(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                case EventId::ANY_KEY: MENU_any_key(); break; // First ancestor handler for this event
                case EventId::BACK_HELD: MENU_back_held(); break; // First ancestor handler for this event
                case EventId::TIMEOUT: MENU_timeout(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: CHANGE_CODE
        case StateId::CHANGE_CODE:
            switch (eventId)
            {
                case EventId::BACK_PRESS: CHANGE_CODE_back_press(); break;
                case EventId::UP_DOWN_HELD: CHANGE_CODE_up_down_held(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                case EventId::ANY_KEY: MENU_any_key(); break; // First ancestor handler for this event
                case EventId::BACK_HELD: MENU_back_held(); break; // First ancestor handler for this event
                case EventId::TIMEOUT: MENU_timeout(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: CONFIG__ALARM_DELAY
        case StateId::CONFIG__ALARM_DELAY:
            switch (eventId)
            {
                case EventId::UP_PRESS: CONFIG__ALARM_DELAY_up_press(); break;
                case EventId::ENTER_PRESS: CONFIG__ALARM_DELAY_enter_press(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                case EventId::ANY_KEY: MENU_any_key(); break; // First ancestor handler for this event
                case EventId::BACK_PRESS: CONFIG_back_press(); break; // First ancestor handler for this event
                case EventId::BACK_HELD: MENU_back_held(); break; // First ancestor handler for this event
                case EventId::TIMEOUT: MENU_timeout(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: CONFIG__ARMING_DELAY
        case StateId::CONFIG__ARMING_DELAY:
            switch (eventId)
            {
                case EventId::DOWN_PRESS: CONFIG__ARMING_DELAY_down_press(); break;
                case EventId::UP_PRESS: CONFIG__ARMING_DELAY_up_press(); break;
                case EventId::ENTER_PRESS: CONFIG__ARMING_DELAY_enter_press(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                case EventId::ANY_KEY: MENU_any_key(); break; // First ancestor handler for this event
                case EventId::BACK_PRESS: CONFIG_back_press(); break; // First ancestor handler for this event
                case EventId::BACK_HELD: MENU_back_held(); break; // First ancestor handler for this event
                case EventId::TIMEOUT: MENU_timeout(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: CONFIG__CHANGE_CODE
        case StateId::CONFIG__CHANGE_CODE:
            switch (eventId)
            {
                case EventId::DOWN_PRESS: CONFIG__CHANGE_CODE_down_press(); break;
                case EventId::ENTER_PRESS: CONFIG__CHANGE_CODE_enter_press(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                case EventId::ANY_KEY: MENU_any_key(); break; // First ancestor handler for this event
                case EventId::BACK_PRESS: CONFIG_back_press(); break; // First ancestor handler for this event
                case EventId::BACK_HELD: MENU_back_held(); break; // First ancestor handler for this event
                case EventId::TIMEOUT: MENU_timeout(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: SECRET_MENU
        case StateId::SECRET_MENU:
            switch (eventId)
            {
                case EventId::LCD_UPDATE: SECRET_MENU_lcd_update(); break;
                case EventId::UP_PRESS: SECRET_MENU_up_press(); break;
                case EventId::DOWN_PRESS: SECRET_MENU_down_press(); break;
                case EventId::BACK_PRESS: SECRET_MENU_back_press(); break;
                case EventId::ENTER_PRESS: SECRET_MENU_enter_press(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                case EventId::ANY_KEY: MENU_any_key(); break; // First ancestor handler for this event
                case EventId::BACK_HELD: MENU_back_held(); break; // First ancestor handler for this event
                case EventId::TIMEOUT: MENU_timeout(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: DATA
        case StateId::DATA:
            switch (eventId)
            {
                case EventId::ANY_KEY: DATA_any_key(); break;
                case EventId::BACK_PRESS: DATA_back_press(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                case EventId::BACK_HELD: MENU_back_held(); break; // First ancestor handler for this event
                case EventId::TIMEOUT: MENU_timeout(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: SENSOR
        case StateId::SENSOR:
            switch (eventId)
            {
                case EventId::LCD_UPDATE: SENSOR_lcd_update(); break;
                case EventId::UP_PRESS: SENSOR_up_press(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                case EventId::ANY_KEY: DATA_any_key(); break; // First ancestor handler for this event
                case EventId::BACK_PRESS: DATA_back_press(); break; // First ancestor handler for this event
                case EventId::BACK_HELD: MENU_back_held(); break; // First ancestor handler for this event
                case EventId::TIMEOUT: MENU_timeout(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: UPTIME
        case StateId::UPTIME:
            switch (eventId)
            {
                case EventId::LCD_UPDATE: UPTIME_lcd_update(); break;
                case EventId::DOWN_PRESS: UPTIME_down_press(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                case EventId::ANY_KEY: DATA_any_key(); break; // First ancestor handler for this event
                case EventId::BACK_PRESS: DATA_back_press(); break; // First ancestor handler for this event
                case EventId::BACK_HELD: MENU_back_held(); break; // First ancestor handler for this event
                case EventId::TIMEOUT: MENU_timeout(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: DISARM_SYSTEM
        case StateId::DISARM_SYSTEM:
            switch (eventId)
            {
                case EventId::BACK_PRESS: DISARM_SYSTEM_back_press(); break;
                case EventId::ENTER_PRESS: DISARM_SYSTEM_enter_press(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                case EventId::ANY_KEY: MENU_any_key(); break; // First ancestor handler for this event
                case EventId::BACK_HELD: MENU_back_held(); break; // First ancestor handler for this event
                case EventId::TIMEOUT: MENU_timeout(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: MENU__ARM_SYSTEM
        case StateId::MENU__ARM_SYSTEM:
            switch (eventId)
            {
                case EventId::DOWN_PRESS: MENU__ARM_SYSTEM_down_press(); break;
                case EventId::UP_PRESS: MENU__ARM_SYSTEM_up_press(); break;
                case EventId::ENTER_PRESS: MENU__ARM_SYSTEM_enter_press(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                case EventId::ANY_KEY: MENU_any_key(); break; // First ancestor handler for this event
                case EventId::BACK_PRESS: MENU_back_press(); break; // First ancestor handler for this event
                case EventId::BACK_HELD: MENU_back_held(); break; // First ancestor handler for this event
                case EventId::TIMEOUT: MENU_timeout(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: MENU__CONFIG
        case StateId::MENU__CONFIG:
            switch (eventId)
            {
                case EventId::DOWN_PRESS: MENU__CONFIG_down_press(); break;
                case EventId::UP_PRESS: MENU__CONFIG_up_press(); break;
                case EventId::ENTER_PRESS: MENU__CONFIG_enter_press(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                case EventId::ANY_KEY: MENU_any_key(); break; // First ancestor handler for this event
                case EventId::BACK_PRESS: MENU_back_press(); break; // First ancestor handler for this event
                case EventId::BACK_HELD: MENU_back_held(); break; // First ancestor handler for this event
                case EventId::TIMEOUT: MENU_timeout(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: MENU__DATA
        case StateId::MENU__DATA:
            switch (eventId)
            {
                case EventId::UP_PRESS: MENU__DATA_up_press(); break;
                case EventId::ENTER_PRESS: MENU__DATA_enter_press(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                case EventId::ANY_KEY: MENU_any_key(); break; // First ancestor handler for this event
                case EventId::BACK_PRESS: MENU_back_press(); break; // First ancestor handler for this event
                case EventId::BACK_HELD: MENU_back_held(); break; // First ancestor handler for this event
                case EventId::TIMEOUT: MENU_timeout(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: MENU__DISARM_SYSTEM
        case StateId::MENU__DISARM_SYSTEM:
            switch (eventId)
            {
                case EventId::DOWN_PRESS: MENU__DISARM_SYSTEM_down_press(); break;
                case EventId::ENTER_PRESS: MENU__DISARM_SYSTEM_enter_press(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                case EventId::ANY_KEY: MENU_any_key(); break; // First ancestor handler for this event
                case EventId::BACK_PRESS: MENU_back_press(); break; // First ancestor handler for this event
                case EventId::BACK_HELD: MENU_back_held(); break; // First ancestor handler for this event
                case EventId::TIMEOUT: MENU_timeout(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: PANIC
        case StateId::PANIC:
            switch (eventId)
            {
                case EventId::TIMEOUT: PANIC_timeout(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
        
        // STATE: SPLASH_SCREEN
        case StateId::SPLASH_SCREEN:
            switch (eventId)
            {
                case EventId::TIMEOUT: SPLASH_SCREEN_timeout(); break;
                case EventId::PANIC_BUTTON: ROOT_panic_button(); break; // First ancestor handler for this event
                
                default: break; // to avoid "unused enumeration value in switch" warning
            }
            break;
    }
    
}

// This function is used when StateSmith doesn't know what the active leaf state is at
// compile time due to sub states or when multiple states need to be exited.
void UiSm::exitUpToStateHandler(StateId desiredState)
{
    while (this->stateId != desiredState)
    {
        switch (this->stateId)
        {
            case StateId::HOME: HOME_exit(); break;
            
            case StateId::MENU: MENU_exit(); break;
            
            case StateId::ARM_SYSTEM: ARM_SYSTEM_exit(); break;
            
            case StateId::CONFIG: CONFIG_exit(); break;
            
            case StateId::ALARM_DELAY: ALARM_DELAY_exit(); break;
            
            case StateId::ARMING_DELAY: ARMING_DELAY_exit(); break;
            
            case StateId::CHANGE_CODE: CHANGE_CODE_exit(); break;
            
            case StateId::CONFIG__ALARM_DELAY: CONFIG__ALARM_DELAY_exit(); break;
            
            case StateId::CONFIG__ARMING_DELAY: CONFIG__ARMING_DELAY_exit(); break;
            
            case StateId::CONFIG__CHANGE_CODE: CONFIG__CHANGE_CODE_exit(); break;
            
            case StateId::SECRET_MENU: SECRET_MENU_exit(); break;
            
            case StateId::DATA: DATA_exit(); break;
            
            case StateId::SENSOR: SENSOR_exit(); break;
            
            case StateId::UPTIME: UPTIME_exit(); break;
            
            case StateId::DISARM_SYSTEM: DISARM_SYSTEM_exit(); break;
            
            case StateId::MENU__ARM_SYSTEM: MENU__ARM_SYSTEM_exit(); break;
            
            case StateId::MENU__CONFIG: MENU__CONFIG_exit(); break;
            
            case StateId::MENU__DATA: MENU__DATA_exit(); break;
            
            case StateId::MENU__DISARM_SYSTEM: MENU__DISARM_SYSTEM_exit(); break;
            
            case StateId::PANIC: PANIC_exit(); break;
            
            case StateId::SPLASH_SCREEN: SPLASH_SCREEN_exit(); break;
            
            default: return;  // Just to be safe. Prevents infinite loop if state ID memory is somehow corrupted.
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state ROOT
////////////////////////////////////////////////////////////////////////////////

void UiSm::ROOT_enter()
{
    this->stateId = StateId::ROOT;
}

void UiSm::ROOT_panic_button()
{
    // ROOT behavior
    // uml: PANIC_BUTTON TransitionTo(PANIC)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        exitUpToStateHandler(StateId::ROOT);
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `PANIC`.
        PANIC_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for ROOT
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state HOME
////////////////////////////////////////////////////////////////////////////////

void UiSm::HOME_enter()
{
    this->stateId = StateId::HOME;
    
    // HOME behavior
    // uml: (enter, REFRESH) / { lcd->HOME(); }
    {
        // Step 1: execute action `lcd->HOME();`
        this->vars.lcd->HOME();
    } // end of behavior for HOME
}

void UiSm::HOME_exit()
{
    this->stateId = StateId::ROOT;
}

void UiSm::HOME_enter_press()
{
    // HOME behavior
    // uml: ENTER_PRESS TransitionTo(MENU)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        HOME_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `MENU`.
        MENU_enter();
        
        // MENU.<InitialState> behavior
        // uml: TransitionTo(MENU__DISARM_SYSTEM)
        {
            // Step 1: Exit states until we reach `MENU` state (Least Common Ancestor for transition). Already at LCA, no exiting required.
            
            // Step 2: Transition action: ``.
            
            // Step 3: Enter/move towards transition target `MENU__DISARM_SYSTEM`.
            MENU__DISARM_SYSTEM_enter();
            
            // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
            return;
        } // end of behavior for MENU.<InitialState>
    } // end of behavior for HOME
    
    // No ancestor handles this event.
}

void UiSm::HOME_lcd_update()
{
    // HOME behavior
    // uml: LCD_UPDATE / { lcd->HOME_update(); }
    {
        // Step 1: execute action `lcd->HOME_update();`
        this->vars.lcd->HOME_update();
    } // end of behavior for HOME
    
    // No ancestor handles this event.
}

void UiSm::HOME_refresh()
{
    // HOME behavior
    // uml: (enter, REFRESH) / { lcd->HOME(); }
    {
        // Step 1: execute action `lcd->HOME();`
        this->vars.lcd->HOME();
    } // end of behavior for HOME
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state MENU
////////////////////////////////////////////////////////////////////////////////

void UiSm::MENU_enter()
{
    this->stateId = StateId::MENU;
    
    // MENU behavior
    // uml: (enter, ANY_KEY) / { timeout_ms = IDLE_TIMEOUT_MS; }
    {
        // Step 1: execute action `timeout_ms = IDLE_TIMEOUT_MS;`
        this->vars.timeout_ms = IDLE_TIMEOUT_MS;
    } // end of behavior for MENU
}

void UiSm::MENU_exit()
{
    this->stateId = StateId::ROOT;
}

void UiSm::MENU_any_key()
{
    // MENU behavior
    // uml: (enter, ANY_KEY) / { timeout_ms = IDLE_TIMEOUT_MS; }
    {
        // Step 1: execute action `timeout_ms = IDLE_TIMEOUT_MS;`
        this->vars.timeout_ms = IDLE_TIMEOUT_MS;
    } // end of behavior for MENU
    
    // No ancestor handles this event.
}

void UiSm::MENU_back_held()
{
    // MENU behavior
    // uml: (BACK_PRESS, BACK_HELD, TIMEOUT) TransitionTo(HOME)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        exitUpToStateHandler(StateId::ROOT);
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `HOME`.
        HOME_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for MENU
    
    // No ancestor handles this event.
}

void UiSm::MENU_back_press()
{
    // MENU behavior
    // uml: (BACK_PRESS, BACK_HELD, TIMEOUT) TransitionTo(HOME)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        exitUpToStateHandler(StateId::ROOT);
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `HOME`.
        HOME_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for MENU
    
    // No ancestor handles this event.
}

void UiSm::MENU_timeout()
{
    // MENU behavior
    // uml: (BACK_PRESS, BACK_HELD, TIMEOUT) TransitionTo(HOME)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        exitUpToStateHandler(StateId::ROOT);
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `HOME`.
        HOME_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for MENU
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state ARM_SYSTEM
////////////////////////////////////////////////////////////////////////////////

void UiSm::ARM_SYSTEM_enter()
{
    this->stateId = StateId::ARM_SYSTEM;
    
    // ARM_SYSTEM behavior
    // uml: enter / { lcd->ARM_SYSTEM(); }
    {
        // Step 1: execute action `lcd->ARM_SYSTEM();`
        this->vars.lcd->ARM_SYSTEM();
    } // end of behavior for ARM_SYSTEM
}

void UiSm::ARM_SYSTEM_exit()
{
    this->stateId = StateId::MENU;
}

void UiSm::ARM_SYSTEM_back_press()
{
    bool consume_event = false;
    
    // ARM_SYSTEM behavior
    // uml: BACK_PRESS TransitionTo(MENU__ARM_SYSTEM)
    {
        // Step 1: Exit states until we reach `MENU` state (Least Common Ancestor for transition).
        ARM_SYSTEM_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `MENU__ARM_SYSTEM`.
        MENU__ARM_SYSTEM_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for ARM_SYSTEM
    
    // Check if event has been consumed before calling ancestor handler.
    if (!consume_event)
    {
        MENU_back_press();
    }
}

void UiSm::ARM_SYSTEM_enter_press()
{
    // ARM_SYSTEM behavior
    // uml: ENTER_PRESS / { ctrl->arm(); } TransitionTo(HOME)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        exitUpToStateHandler(StateId::ROOT);
        
        // Step 2: Transition action: `ctrl->arm();`.
        this->vars.ctrl->arm();
        
        // Step 3: Enter/move towards transition target `HOME`.
        HOME_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for ARM_SYSTEM
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state CONFIG
////////////////////////////////////////////////////////////////////////////////

void UiSm::CONFIG_enter()
{
    this->stateId = StateId::CONFIG;
}

void UiSm::CONFIG_exit()
{
    this->stateId = StateId::MENU;
}

void UiSm::CONFIG_back_press()
{
    bool consume_event = false;
    
    // CONFIG behavior
    // uml: BACK_PRESS TransitionTo(MENU__CONFIG)
    {
        // Step 1: Exit states until we reach `MENU` state (Least Common Ancestor for transition).
        exitUpToStateHandler(StateId::MENU);
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `MENU__CONFIG`.
        MENU__CONFIG_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for CONFIG
    
    // Check if event has been consumed before calling ancestor handler.
    if (!consume_event)
    {
        MENU_back_press();
    }
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state ALARM_DELAY
////////////////////////////////////////////////////////////////////////////////

void UiSm::ALARM_DELAY_enter()
{
    this->stateId = StateId::ALARM_DELAY;
    
    // ALARM_DELAY behavior
    // uml: enter / { lcd->ALARM_DELAY(); }
    {
        // Step 1: execute action `lcd->ALARM_DELAY();`
        this->vars.lcd->ALARM_DELAY();
    } // end of behavior for ALARM_DELAY
}

void UiSm::ALARM_DELAY_exit()
{
    this->stateId = StateId::CONFIG;
}

void UiSm::ALARM_DELAY_back_press()
{
    bool consume_event = false;
    
    // ALARM_DELAY behavior
    // uml: BACK_PRESS TransitionTo(CONFIG__ALARM_DELAY)
    {
        // Step 1: Exit states until we reach `CONFIG` state (Least Common Ancestor for transition).
        ALARM_DELAY_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `CONFIG__ALARM_DELAY`.
        CONFIG__ALARM_DELAY_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for ALARM_DELAY
    
    // Check if event has been consumed before calling ancestor handler.
    if (!consume_event)
    {
        CONFIG_back_press();
    }
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state ARMING_DELAY
////////////////////////////////////////////////////////////////////////////////

void UiSm::ARMING_DELAY_enter()
{
    this->stateId = StateId::ARMING_DELAY;
    
    // ARMING_DELAY behavior
    // uml: enter / { lcd->ARMING_DELAY(); }
    {
        // Step 1: execute action `lcd->ARMING_DELAY();`
        this->vars.lcd->ARMING_DELAY();
    } // end of behavior for ARMING_DELAY
}

void UiSm::ARMING_DELAY_exit()
{
    this->stateId = StateId::CONFIG;
}

void UiSm::ARMING_DELAY_back_press()
{
    bool consume_event = false;
    
    // ARMING_DELAY behavior
    // uml: BACK_PRESS TransitionTo(CONFIG__ARMING_DELAY)
    {
        // Step 1: Exit states until we reach `CONFIG` state (Least Common Ancestor for transition).
        ARMING_DELAY_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `CONFIG__ARMING_DELAY`.
        CONFIG__ARMING_DELAY_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for ARMING_DELAY
    
    // Check if event has been consumed before calling ancestor handler.
    if (!consume_event)
    {
        CONFIG_back_press();
    }
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state CHANGE_CODE
////////////////////////////////////////////////////////////////////////////////

void UiSm::CHANGE_CODE_enter()
{
    this->stateId = StateId::CHANGE_CODE;
    
    // CHANGE_CODE behavior
    // uml: enter / { lcd->CHANGE_CODE(); }
    {
        // Step 1: execute action `lcd->CHANGE_CODE();`
        this->vars.lcd->CHANGE_CODE();
    } // end of behavior for CHANGE_CODE
}

void UiSm::CHANGE_CODE_exit()
{
    this->stateId = StateId::CONFIG;
}

void UiSm::CHANGE_CODE_back_press()
{
    bool consume_event = false;
    
    // CHANGE_CODE behavior
    // uml: BACK_PRESS TransitionTo(CONFIG__CHANGE_CODE)
    {
        // Step 1: Exit states until we reach `CONFIG` state (Least Common Ancestor for transition).
        CHANGE_CODE_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `CONFIG__CHANGE_CODE`.
        CONFIG__CHANGE_CODE_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for CHANGE_CODE
    
    // Check if event has been consumed before calling ancestor handler.
    if (!consume_event)
    {
        CONFIG_back_press();
    }
}

void UiSm::CHANGE_CODE_up_down_held()
{
    // CHANGE_CODE behavior
    // uml: UP_DOWN_HELD TransitionTo(SECRET_MENU)
    {
        // Step 1: Exit states until we reach `CONFIG` state (Least Common Ancestor for transition).
        CHANGE_CODE_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `SECRET_MENU`.
        SECRET_MENU_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for CHANGE_CODE
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state CONFIG__ALARM_DELAY
////////////////////////////////////////////////////////////////////////////////

void UiSm::CONFIG__ALARM_DELAY_enter()
{
    this->stateId = StateId::CONFIG__ALARM_DELAY;
    
    // CONFIG__ALARM_DELAY behavior
    // uml: enter / { lcd->CONFIG__ALARM_DELAY(); }
    {
        // Step 1: execute action `lcd->CONFIG__ALARM_DELAY();`
        this->vars.lcd->CONFIG__ALARM_DELAY();
    } // end of behavior for CONFIG__ALARM_DELAY
}

void UiSm::CONFIG__ALARM_DELAY_exit()
{
    this->stateId = StateId::CONFIG;
}

void UiSm::CONFIG__ALARM_DELAY_enter_press()
{
    // CONFIG__ALARM_DELAY behavior
    // uml: ENTER_PRESS TransitionTo(ALARM_DELAY)
    {
        // Step 1: Exit states until we reach `CONFIG` state (Least Common Ancestor for transition).
        CONFIG__ALARM_DELAY_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `ALARM_DELAY`.
        ALARM_DELAY_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for CONFIG__ALARM_DELAY
    
    // No ancestor handles this event.
}

void UiSm::CONFIG__ALARM_DELAY_up_press()
{
    // CONFIG__ALARM_DELAY behavior
    // uml: UP_PRESS TransitionTo(CONFIG__ARMING_DELAY)
    {
        // Step 1: Exit states until we reach `CONFIG` state (Least Common Ancestor for transition).
        CONFIG__ALARM_DELAY_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `CONFIG__ARMING_DELAY`.
        CONFIG__ARMING_DELAY_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for CONFIG__ALARM_DELAY
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state CONFIG__ARMING_DELAY
////////////////////////////////////////////////////////////////////////////////

void UiSm::CONFIG__ARMING_DELAY_enter()
{
    this->stateId = StateId::CONFIG__ARMING_DELAY;
    
    // CONFIG__ARMING_DELAY behavior
    // uml: enter / { lcd->CONFIG__ARMING_DELAY(); }
    {
        // Step 1: execute action `lcd->CONFIG__ARMING_DELAY();`
        this->vars.lcd->CONFIG__ARMING_DELAY();
    } // end of behavior for CONFIG__ARMING_DELAY
}

void UiSm::CONFIG__ARMING_DELAY_exit()
{
    this->stateId = StateId::CONFIG;
}

void UiSm::CONFIG__ARMING_DELAY_down_press()
{
    // CONFIG__ARMING_DELAY behavior
    // uml: DOWN_PRESS TransitionTo(CONFIG__ALARM_DELAY)
    {
        // Step 1: Exit states until we reach `CONFIG` state (Least Common Ancestor for transition).
        CONFIG__ARMING_DELAY_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `CONFIG__ALARM_DELAY`.
        CONFIG__ALARM_DELAY_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for CONFIG__ARMING_DELAY
    
    // No ancestor handles this event.
}

void UiSm::CONFIG__ARMING_DELAY_enter_press()
{
    // CONFIG__ARMING_DELAY behavior
    // uml: ENTER_PRESS TransitionTo(ARMING_DELAY)
    {
        // Step 1: Exit states until we reach `CONFIG` state (Least Common Ancestor for transition).
        CONFIG__ARMING_DELAY_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `ARMING_DELAY`.
        ARMING_DELAY_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for CONFIG__ARMING_DELAY
    
    // No ancestor handles this event.
}

void UiSm::CONFIG__ARMING_DELAY_up_press()
{
    // CONFIG__ARMING_DELAY behavior
    // uml: UP_PRESS TransitionTo(CONFIG__CHANGE_CODE)
    {
        // Step 1: Exit states until we reach `CONFIG` state (Least Common Ancestor for transition).
        CONFIG__ARMING_DELAY_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `CONFIG__CHANGE_CODE`.
        CONFIG__CHANGE_CODE_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for CONFIG__ARMING_DELAY
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state CONFIG__CHANGE_CODE
////////////////////////////////////////////////////////////////////////////////

void UiSm::CONFIG__CHANGE_CODE_enter()
{
    this->stateId = StateId::CONFIG__CHANGE_CODE;
    
    // CONFIG__CHANGE_CODE behavior
    // uml: enter / { lcd->CONFIG__CHANGE_CODE(); }
    {
        // Step 1: execute action `lcd->CONFIG__CHANGE_CODE();`
        this->vars.lcd->CONFIG__CHANGE_CODE();
    } // end of behavior for CONFIG__CHANGE_CODE
}

void UiSm::CONFIG__CHANGE_CODE_exit()
{
    this->stateId = StateId::CONFIG;
}

void UiSm::CONFIG__CHANGE_CODE_down_press()
{
    // CONFIG__CHANGE_CODE behavior
    // uml: DOWN_PRESS TransitionTo(CONFIG__ARMING_DELAY)
    {
        // Step 1: Exit states until we reach `CONFIG` state (Least Common Ancestor for transition).
        CONFIG__CHANGE_CODE_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `CONFIG__ARMING_DELAY`.
        CONFIG__ARMING_DELAY_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for CONFIG__CHANGE_CODE
    
    // No ancestor handles this event.
}

void UiSm::CONFIG__CHANGE_CODE_enter_press()
{
    // CONFIG__CHANGE_CODE behavior
    // uml: ENTER_PRESS TransitionTo(CHANGE_CODE)
    {
        // Step 1: Exit states until we reach `CONFIG` state (Least Common Ancestor for transition).
        CONFIG__CHANGE_CODE_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `CHANGE_CODE`.
        CHANGE_CODE_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for CONFIG__CHANGE_CODE
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state SECRET_MENU
////////////////////////////////////////////////////////////////////////////////

void UiSm::SECRET_MENU_enter()
{
    this->stateId = StateId::SECRET_MENU;
    
    // SECRET_MENU behavior
    // uml: enter / { lcd->SECRET_MENU(); }
    {
        // Step 1: execute action `lcd->SECRET_MENU();`
        this->vars.lcd->SECRET_MENU();
    } // end of behavior for SECRET_MENU
    
    // SECRET_MENU behavior
    // uml: enter / { audio->play_secret(); }
    {
        // Step 1: execute action `audio->play_secret();`
        this->vars.audio->play_secret();
    } // end of behavior for SECRET_MENU
}

void UiSm::SECRET_MENU_exit()
{
    this->stateId = StateId::CONFIG;
}

void UiSm::SECRET_MENU_back_press()
{
    bool consume_event = false;
    
    // SECRET_MENU behavior
    // uml: (UP_PRESS, DOWN_PRESS, BACK_PRESS, ENTER_PRESS) / { lcd->SECRET_MENU_button(event_id); }
    {
        // Consume event `back_press`.
        consume_event = true;
        // Step 1: execute action `lcd->SECRET_MENU_button(event_id);`
        this->vars.lcd->SECRET_MENU_button(this->vars.event_id);
    } // end of behavior for SECRET_MENU
    
    // Check if event has been consumed before calling ancestor handler.
    if (!consume_event)
    {
        CONFIG_back_press();
    }
}

void UiSm::SECRET_MENU_down_press()
{
    // SECRET_MENU behavior
    // uml: (UP_PRESS, DOWN_PRESS, BACK_PRESS, ENTER_PRESS) / { lcd->SECRET_MENU_button(event_id); }
    {
        // Step 1: execute action `lcd->SECRET_MENU_button(event_id);`
        this->vars.lcd->SECRET_MENU_button(this->vars.event_id);
    } // end of behavior for SECRET_MENU
    
    // No ancestor handles this event.
}

void UiSm::SECRET_MENU_enter_press()
{
    // SECRET_MENU behavior
    // uml: (UP_PRESS, DOWN_PRESS, BACK_PRESS, ENTER_PRESS) / { lcd->SECRET_MENU_button(event_id); }
    {
        // Step 1: execute action `lcd->SECRET_MENU_button(event_id);`
        this->vars.lcd->SECRET_MENU_button(this->vars.event_id);
    } // end of behavior for SECRET_MENU
    
    // No ancestor handles this event.
}

void UiSm::SECRET_MENU_lcd_update()
{
    // SECRET_MENU behavior
    // uml: LCD_UPDATE / { lcd->SECRET_MENU_update(); }
    {
        // Step 1: execute action `lcd->SECRET_MENU_update();`
        this->vars.lcd->SECRET_MENU_update();
    } // end of behavior for SECRET_MENU
    
    // SECRET_MENU behavior
    // uml: LCD_UPDATE / { timeout_ms = IDLE_TIMEOUT_MS; /* disable timeout */ }
    {
        // Step 1: execute action `timeout_ms = IDLE_TIMEOUT_MS; /* disable timeout */`
        this->vars.timeout_ms = IDLE_TIMEOUT_MS; /* disable timeout */
    } // end of behavior for SECRET_MENU
    
    // No ancestor handles this event.
}

void UiSm::SECRET_MENU_up_press()
{
    // SECRET_MENU behavior
    // uml: (UP_PRESS, DOWN_PRESS, BACK_PRESS, ENTER_PRESS) / { lcd->SECRET_MENU_button(event_id); }
    {
        // Step 1: execute action `lcd->SECRET_MENU_button(event_id);`
        this->vars.lcd->SECRET_MENU_button(this->vars.event_id);
    } // end of behavior for SECRET_MENU
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state DATA
////////////////////////////////////////////////////////////////////////////////

void UiSm::DATA_enter()
{
    this->stateId = StateId::DATA;
    
    // DATA behavior
    // uml: (enter, ANY_KEY) / { timeout_ms = DATA_TIMEOUT_MS; }
    {
        // Step 1: execute action `timeout_ms = DATA_TIMEOUT_MS;`
        this->vars.timeout_ms = DATA_TIMEOUT_MS;
    } // end of behavior for DATA
}

void UiSm::DATA_exit()
{
    // DATA behavior
    // uml: exit / { timeout_ms = IDLE_TIMEOUT_MS; }
    {
        // Step 1: execute action `timeout_ms = IDLE_TIMEOUT_MS;`
        this->vars.timeout_ms = IDLE_TIMEOUT_MS;
    } // end of behavior for DATA
    
    this->stateId = StateId::MENU;
}

void UiSm::DATA_any_key()
{
    bool consume_event = false;
    
    // DATA behavior
    // uml: (enter, ANY_KEY) / { timeout_ms = DATA_TIMEOUT_MS; }
    {
        // Consume event `any_key`.
        consume_event = true;
        // Step 1: execute action `timeout_ms = DATA_TIMEOUT_MS;`
        this->vars.timeout_ms = DATA_TIMEOUT_MS;
    } // end of behavior for DATA
    
    // Check if event has been consumed before calling ancestor handler.
    if (!consume_event)
    {
        MENU_any_key();
    }
}

void UiSm::DATA_back_press()
{
    bool consume_event = false;
    
    // DATA behavior
    // uml: BACK_PRESS TransitionTo(MENU__DATA)
    {
        // Step 1: Exit states until we reach `MENU` state (Least Common Ancestor for transition).
        exitUpToStateHandler(StateId::MENU);
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `MENU__DATA`.
        MENU__DATA_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for DATA
    
    // Check if event has been consumed before calling ancestor handler.
    if (!consume_event)
    {
        MENU_back_press();
    }
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state SENSOR
////////////////////////////////////////////////////////////////////////////////

void UiSm::SENSOR_enter()
{
    this->stateId = StateId::SENSOR;
    
    // SENSOR behavior
    // uml: enter / { lcd->SENSOR(); }
    {
        // Step 1: execute action `lcd->SENSOR();`
        this->vars.lcd->SENSOR();
    } // end of behavior for SENSOR
}

void UiSm::SENSOR_exit()
{
    this->stateId = StateId::DATA;
}

void UiSm::SENSOR_lcd_update()
{
    // SENSOR behavior
    // uml: LCD_UPDATE / { lcd->SENSOR_update(); }
    {
        // Step 1: execute action `lcd->SENSOR_update();`
        this->vars.lcd->SENSOR_update();
    } // end of behavior for SENSOR
    
    // No ancestor handles this event.
}

void UiSm::SENSOR_up_press()
{
    // SENSOR behavior
    // uml: UP_PRESS TransitionTo(UPTIME)
    {
        // Step 1: Exit states until we reach `DATA` state (Least Common Ancestor for transition).
        SENSOR_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `UPTIME`.
        UPTIME_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for SENSOR
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state UPTIME
////////////////////////////////////////////////////////////////////////////////

void UiSm::UPTIME_enter()
{
    this->stateId = StateId::UPTIME;
    
    // UPTIME behavior
    // uml: enter / { lcd->UPTIME(); }
    {
        // Step 1: execute action `lcd->UPTIME();`
        this->vars.lcd->UPTIME();
    } // end of behavior for UPTIME
}

void UiSm::UPTIME_exit()
{
    this->stateId = StateId::DATA;
}

void UiSm::UPTIME_down_press()
{
    // UPTIME behavior
    // uml: DOWN_PRESS TransitionTo(SENSOR)
    {
        // Step 1: Exit states until we reach `DATA` state (Least Common Ancestor for transition).
        UPTIME_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `SENSOR`.
        SENSOR_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for UPTIME
    
    // No ancestor handles this event.
}

void UiSm::UPTIME_lcd_update()
{
    // UPTIME behavior
    // uml: LCD_UPDATE / { lcd->UPTIME_update(); }
    {
        // Step 1: execute action `lcd->UPTIME_update();`
        this->vars.lcd->UPTIME_update();
    } // end of behavior for UPTIME
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state DISARM_SYSTEM
////////////////////////////////////////////////////////////////////////////////

void UiSm::DISARM_SYSTEM_enter()
{
    this->stateId = StateId::DISARM_SYSTEM;
    
    // DISARM_SYSTEM behavior
    // uml: enter / { lcd->DISARM_SYSTEM(); }
    {
        // Step 1: execute action `lcd->DISARM_SYSTEM();`
        this->vars.lcd->DISARM_SYSTEM();
    } // end of behavior for DISARM_SYSTEM
}

void UiSm::DISARM_SYSTEM_exit()
{
    this->stateId = StateId::MENU;
}

void UiSm::DISARM_SYSTEM_back_press()
{
    bool consume_event = false;
    
    // DISARM_SYSTEM behavior
    // uml: BACK_PRESS TransitionTo(MENU__DISARM_SYSTEM)
    {
        // Step 1: Exit states until we reach `MENU` state (Least Common Ancestor for transition).
        DISARM_SYSTEM_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `MENU__DISARM_SYSTEM`.
        MENU__DISARM_SYSTEM_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for DISARM_SYSTEM
    
    // Check if event has been consumed before calling ancestor handler.
    if (!consume_event)
    {
        MENU_back_press();
    }
}

void UiSm::DISARM_SYSTEM_enter_press()
{
    // DISARM_SYSTEM behavior
    // uml: ENTER_PRESS / { ctrl->disarm(); } TransitionTo(HOME)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        exitUpToStateHandler(StateId::ROOT);
        
        // Step 2: Transition action: `ctrl->disarm();`.
        this->vars.ctrl->disarm();
        
        // Step 3: Enter/move towards transition target `HOME`.
        HOME_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for DISARM_SYSTEM
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state MENU__ARM_SYSTEM
////////////////////////////////////////////////////////////////////////////////

void UiSm::MENU__ARM_SYSTEM_enter()
{
    this->stateId = StateId::MENU__ARM_SYSTEM;
    
    // MENU__ARM_SYSTEM behavior
    // uml: enter / { lcd->MENU__ARM_SYSTEM(); }
    {
        // Step 1: execute action `lcd->MENU__ARM_SYSTEM();`
        this->vars.lcd->MENU__ARM_SYSTEM();
    } // end of behavior for MENU__ARM_SYSTEM
}

void UiSm::MENU__ARM_SYSTEM_exit()
{
    this->stateId = StateId::MENU;
}

void UiSm::MENU__ARM_SYSTEM_down_press()
{
    // MENU__ARM_SYSTEM behavior
    // uml: DOWN_PRESS TransitionTo(MENU__CONFIG)
    {
        // Step 1: Exit states until we reach `MENU` state (Least Common Ancestor for transition).
        MENU__ARM_SYSTEM_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `MENU__CONFIG`.
        MENU__CONFIG_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for MENU__ARM_SYSTEM
    
    // No ancestor handles this event.
}

void UiSm::MENU__ARM_SYSTEM_enter_press()
{
    // MENU__ARM_SYSTEM behavior
    // uml: ENTER_PRESS TransitionTo(ARM_SYSTEM)
    {
        // Step 1: Exit states until we reach `MENU` state (Least Common Ancestor for transition).
        MENU__ARM_SYSTEM_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `ARM_SYSTEM`.
        ARM_SYSTEM_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for MENU__ARM_SYSTEM
    
    // No ancestor handles this event.
}

void UiSm::MENU__ARM_SYSTEM_up_press()
{
    // MENU__ARM_SYSTEM behavior
    // uml: UP_PRESS TransitionTo(MENU__DISARM_SYSTEM)
    {
        // Step 1: Exit states until we reach `MENU` state (Least Common Ancestor for transition).
        MENU__ARM_SYSTEM_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `MENU__DISARM_SYSTEM`.
        MENU__DISARM_SYSTEM_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for MENU__ARM_SYSTEM
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state MENU__CONFIG
////////////////////////////////////////////////////////////////////////////////

void UiSm::MENU__CONFIG_enter()
{
    this->stateId = StateId::MENU__CONFIG;
    
    // MENU__CONFIG behavior
    // uml: enter / { lcd->MENU__CONFIG(); }
    {
        // Step 1: execute action `lcd->MENU__CONFIG();`
        this->vars.lcd->MENU__CONFIG();
    } // end of behavior for MENU__CONFIG
}

void UiSm::MENU__CONFIG_exit()
{
    this->stateId = StateId::MENU;
}

void UiSm::MENU__CONFIG_down_press()
{
    // MENU__CONFIG behavior
    // uml: DOWN_PRESS TransitionTo(MENU__DATA)
    {
        // Step 1: Exit states until we reach `MENU` state (Least Common Ancestor for transition).
        MENU__CONFIG_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `MENU__DATA`.
        MENU__DATA_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for MENU__CONFIG
    
    // No ancestor handles this event.
}

void UiSm::MENU__CONFIG_enter_press()
{
    // MENU__CONFIG behavior
    // uml: ENTER_PRESS TransitionTo(CONFIG)
    {
        // Step 1: Exit states until we reach `MENU` state (Least Common Ancestor for transition).
        MENU__CONFIG_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `CONFIG`.
        CONFIG_enter();
        
        // CONFIG.<InitialState> behavior
        // uml: TransitionTo(CONFIG__CHANGE_CODE)
        {
            // Step 1: Exit states until we reach `CONFIG` state (Least Common Ancestor for transition). Already at LCA, no exiting required.
            
            // Step 2: Transition action: ``.
            
            // Step 3: Enter/move towards transition target `CONFIG__CHANGE_CODE`.
            CONFIG__CHANGE_CODE_enter();
            
            // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
            return;
        } // end of behavior for CONFIG.<InitialState>
    } // end of behavior for MENU__CONFIG
    
    // No ancestor handles this event.
}

void UiSm::MENU__CONFIG_up_press()
{
    // MENU__CONFIG behavior
    // uml: UP_PRESS TransitionTo(MENU__ARM_SYSTEM)
    {
        // Step 1: Exit states until we reach `MENU` state (Least Common Ancestor for transition).
        MENU__CONFIG_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `MENU__ARM_SYSTEM`.
        MENU__ARM_SYSTEM_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for MENU__CONFIG
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state MENU__DATA
////////////////////////////////////////////////////////////////////////////////

void UiSm::MENU__DATA_enter()
{
    this->stateId = StateId::MENU__DATA;
    
    // MENU__DATA behavior
    // uml: enter / { lcd->MENU__DATA(); }
    {
        // Step 1: execute action `lcd->MENU__DATA();`
        this->vars.lcd->MENU__DATA();
    } // end of behavior for MENU__DATA
}

void UiSm::MENU__DATA_exit()
{
    this->stateId = StateId::MENU;
}

void UiSm::MENU__DATA_enter_press()
{
    // MENU__DATA behavior
    // uml: ENTER_PRESS TransitionTo(DATA)
    {
        // Step 1: Exit states until we reach `MENU` state (Least Common Ancestor for transition).
        MENU__DATA_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `DATA`.
        DATA_enter();
        
        // DATA.<InitialState> behavior
        // uml: TransitionTo(UPTIME)
        {
            // Step 1: Exit states until we reach `DATA` state (Least Common Ancestor for transition). Already at LCA, no exiting required.
            
            // Step 2: Transition action: ``.
            
            // Step 3: Enter/move towards transition target `UPTIME`.
            UPTIME_enter();
            
            // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
            return;
        } // end of behavior for DATA.<InitialState>
    } // end of behavior for MENU__DATA
    
    // No ancestor handles this event.
}

void UiSm::MENU__DATA_up_press()
{
    // MENU__DATA behavior
    // uml: UP_PRESS TransitionTo(MENU__CONFIG)
    {
        // Step 1: Exit states until we reach `MENU` state (Least Common Ancestor for transition).
        MENU__DATA_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `MENU__CONFIG`.
        MENU__CONFIG_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for MENU__DATA
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state MENU__DISARM_SYSTEM
////////////////////////////////////////////////////////////////////////////////

void UiSm::MENU__DISARM_SYSTEM_enter()
{
    this->stateId = StateId::MENU__DISARM_SYSTEM;
    
    // MENU__DISARM_SYSTEM behavior
    // uml: enter / { lcd->MENU__DISARM_SYSTEM(); }
    {
        // Step 1: execute action `lcd->MENU__DISARM_SYSTEM();`
        this->vars.lcd->MENU__DISARM_SYSTEM();
    } // end of behavior for MENU__DISARM_SYSTEM
}

void UiSm::MENU__DISARM_SYSTEM_exit()
{
    this->stateId = StateId::MENU;
}

void UiSm::MENU__DISARM_SYSTEM_down_press()
{
    // MENU__DISARM_SYSTEM behavior
    // uml: DOWN_PRESS TransitionTo(MENU__ARM_SYSTEM)
    {
        // Step 1: Exit states until we reach `MENU` state (Least Common Ancestor for transition).
        MENU__DISARM_SYSTEM_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `MENU__ARM_SYSTEM`.
        MENU__ARM_SYSTEM_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for MENU__DISARM_SYSTEM
    
    // No ancestor handles this event.
}

void UiSm::MENU__DISARM_SYSTEM_enter_press()
{
    // MENU__DISARM_SYSTEM behavior
    // uml: ENTER_PRESS TransitionTo(DISARM_SYSTEM)
    {
        // Step 1: Exit states until we reach `MENU` state (Least Common Ancestor for transition).
        MENU__DISARM_SYSTEM_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `DISARM_SYSTEM`.
        DISARM_SYSTEM_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for MENU__DISARM_SYSTEM
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state PANIC
////////////////////////////////////////////////////////////////////////////////

void UiSm::PANIC_enter()
{
    this->stateId = StateId::PANIC;
    
    // PANIC behavior
    // uml: enter / { lcd->PANIC(); }
    {
        // Step 1: execute action `lcd->PANIC();`
        this->vars.lcd->PANIC();
    } // end of behavior for PANIC
    
    // PANIC behavior
    // uml: enter / { ctrl->panic(); }
    {
        // Step 1: execute action `ctrl->panic();`
        this->vars.ctrl->panic();
    } // end of behavior for PANIC
    
    // PANIC behavior
    // uml: enter / { timeout_ms = 3*1000; }
    {
        // Step 1: execute action `timeout_ms = 3*1000;`
        this->vars.timeout_ms = 3*1000;
    } // end of behavior for PANIC
}

void UiSm::PANIC_exit()
{
    this->stateId = StateId::ROOT;
}

void UiSm::PANIC_timeout()
{
    // PANIC behavior
    // uml: TIMEOUT TransitionTo(HOME)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        PANIC_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `HOME`.
        HOME_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for PANIC
    
    // No ancestor handles this event.
}


////////////////////////////////////////////////////////////////////////////////
// event handlers for state SPLASH_SCREEN
////////////////////////////////////////////////////////////////////////////////

void UiSm::SPLASH_SCREEN_enter()
{
    this->stateId = StateId::SPLASH_SCREEN;
    
    // SPLASH_SCREEN behavior
    // uml: enter / { lcd->SPLASH_SCREEN(); }
    {
        // Step 1: execute action `lcd->SPLASH_SCREEN();`
        this->vars.lcd->SPLASH_SCREEN();
    } // end of behavior for SPLASH_SCREEN
    
    // SPLASH_SCREEN behavior
    // uml: enter / { timeout_ms = 3*1000; }
    {
        // Step 1: execute action `timeout_ms = 3*1000;`
        this->vars.timeout_ms = 3*1000;
    } // end of behavior for SPLASH_SCREEN
}

void UiSm::SPLASH_SCREEN_exit()
{
    this->stateId = StateId::ROOT;
}

void UiSm::SPLASH_SCREEN_timeout()
{
    // SPLASH_SCREEN behavior
    // uml: TIMEOUT TransitionTo(HOME)
    {
        // Step 1: Exit states until we reach `ROOT` state (Least Common Ancestor for transition).
        SPLASH_SCREEN_exit();
        
        // Step 2: Transition action: ``.
        
        // Step 3: Enter/move towards transition target `HOME`.
        HOME_enter();
        
        // Step 4: complete transition. Ends event dispatch. No other behaviors are checked.
        return;
    } // end of behavior for SPLASH_SCREEN
    
    // No ancestor handles this event.
}

// Thread safe.
char const * UiSm::stateIdToString(StateId id)
{
    switch (id)
    {
        case StateId::ROOT: return "ROOT";
        case StateId::HOME: return "HOME";
        case StateId::MENU: return "MENU";
        case StateId::ARM_SYSTEM: return "ARM_SYSTEM";
        case StateId::CONFIG: return "CONFIG";
        case StateId::ALARM_DELAY: return "ALARM_DELAY";
        case StateId::ARMING_DELAY: return "ARMING_DELAY";
        case StateId::CHANGE_CODE: return "CHANGE_CODE";
        case StateId::CONFIG__ALARM_DELAY: return "CONFIG__ALARM_DELAY";
        case StateId::CONFIG__ARMING_DELAY: return "CONFIG__ARMING_DELAY";
        case StateId::CONFIG__CHANGE_CODE: return "CONFIG__CHANGE_CODE";
        case StateId::SECRET_MENU: return "SECRET_MENU";
        case StateId::DATA: return "DATA";
        case StateId::SENSOR: return "SENSOR";
        case StateId::UPTIME: return "UPTIME";
        case StateId::DISARM_SYSTEM: return "DISARM_SYSTEM";
        case StateId::MENU__ARM_SYSTEM: return "MENU__ARM_SYSTEM";
        case StateId::MENU__CONFIG: return "MENU__CONFIG";
        case StateId::MENU__DATA: return "MENU__DATA";
        case StateId::MENU__DISARM_SYSTEM: return "MENU__DISARM_SYSTEM";
        case StateId::PANIC: return "PANIC";
        case StateId::SPLASH_SCREEN: return "SPLASH_SCREEN";
        default: return "?";
    }
}

// Thread safe.
char const * UiSm::eventIdToString(EventId id)
{
    switch (id)
    {
        case EventId::ANY_KEY: return "ANY_KEY";
        case EventId::BACK_HELD: return "BACK_HELD";
        case EventId::BACK_PRESS: return "BACK_PRESS";
        case EventId::DOWN_PRESS: return "DOWN_PRESS";
        case EventId::ENTER_PRESS: return "ENTER_PRESS";
        case EventId::LCD_UPDATE: return "LCD_UPDATE";
        case EventId::PANIC_BUTTON: return "PANIC_BUTTON";
        case EventId::REFRESH: return "REFRESH";
        case EventId::TIMEOUT: return "TIMEOUT";
        case EventId::UP_DOWN_HELD: return "UP_DOWN_HELD";
        case EventId::UP_PRESS: return "UP_PRESS";
        default: return "?";
    }
}


//////////////////////////////////////////////////////////////////////////////////
// FILE: src/App.hpp
//////////////////////////////////////////////////////////////////////////////////
// #pragma once
// #include "buttons/Buttons.hpp"
// #include "control/ControlData.hpp"
// #include "control/Controller.hpp"
// #include "serial_ui/SerialUi.hpp"
// #include "ui/Ui.hpp"
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


//////////////////////////////////////////////////////////////////////////////////
// FILE: ex3.ino
//////////////////////////////////////////////////////////////////////////////////
// #include "src/App.hpp"

App app;

void setup()
{
    delay(500); // seems required for wokwi to not miss the first note
    app.setup();
}

void loop()
{
    uint32_t now_ms = millis();
    app.update(now_ms);
    delay(5); // lower wokwi simulation CPU usage
}


