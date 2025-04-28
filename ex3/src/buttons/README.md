TODO put picture here

States:
```
RELEASED_DEBOUNCE
RELEASED_STABLE
PRESSED_DEBOUNCE
PRESSED_STABLE
    CONFIRM_LONG
    PRESSED_LONG
```

Transitions:
```
' when debouncing timer is done, we are stable (and released)
RELEASED_DEBOUNCE --> RELEASED_STABLE

' when button is pressed, we need to debounce it
RELEASED_STABLE --> PRESSED_DEBOUNCE

' when debouncing timer is done, we are stable (and pressed)
PRESSED_DEBOUNCE --> PRESSED_STABLE

' when button is released, we need to debounce it
PRESSED_STABLE --> RELEASED_DEBOUNCE
```


State machine variables:
```c
// INPUTs
uint16_t timer_ms;
uint8_t input_active: 1;

// OUTPUTs
uint8_t release_event: 1;
uint8_t press_event: 1;
uint8_t long_event: 1;
uint8_t press_status: 1;
uint8_t long_status: 1;
```
