#include "songs.hpp"
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
