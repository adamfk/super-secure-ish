// Autogenerated with StateSmith 0.18.2+6062baabb038910ff3841c3cd5938115c0ed0a03.
// Algorithm: Balanced2. See https://github.com/StateSmith/StateSmith/wiki/Algorithms

#pragma once  // You can also specify normal include guard. See https://github.com/StateSmith/StateSmith/blob/main/docs/settings.md
#include <stdint.h>
#include "ControllerBase.hpp"


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
