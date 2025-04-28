#pragma once
#include "Note.hpp"
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
