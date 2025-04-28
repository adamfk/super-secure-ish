#pragma once
#include "../io_config.hpp"
#include "SongPlayer.hpp"
#include "songs.hpp"
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
