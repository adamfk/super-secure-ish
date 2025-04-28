/**
 * Inspired from https://wokwi.com/projects/390785670905308161
 */
#pragma once
#include "../io_config.hpp"
#include "Song.hpp"
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
