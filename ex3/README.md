And then finally in `ex3`, we have the full blown design which adds stuff like:
- special button combination detection
    - hold `UP` + `DOWN` for 2 seconds to send `UP_DOWN_HELD` to UI. Required for entering secret menu from `CHANGE_CODE` screen.
    - hold `LEFT` + `RIGHT` for 2 seconds on any screen to send Panic input event.
- Panic feature (AKA "alarm now")
- many more menu pages
    - secret menu that plays Final Fantasy 7 melody and reacts to button presses
- code generating PlantUML menu
    - nested menu PlantUML is easy enough to write by hand, but I wanted to show some interesting potential.
    - more below
- detecting FSM events that are ignored
    - search for `notify_event_ignored()`
- storing FSM events in a variable for advanced usage
    - used to send button events to secret menu display
    - search for `lcd->SECRET_MENU_button(event_id)`
- trigger maps
    - search for `TriggerMap`

## Code generating <u>some</u> PlantUML menu text
Generating menu code is not a new idea, but that's not what we are doing here.

"Normal" menu code generation has a bunch of downsides. Mainly that it isn't very flexible for supporting custom requirements. I've had clients that want secret menus, short cuts & connections between menus, time based rotating between menu pages... When you have requirements like this, it's often too much work to add that functionality to your menu library or code generator. Instead, you just implement the <u>entire</u> FSM manually. That's what I've mostly done in the past (depending on time pressures).

However, in this project, we get the **best of both worlds**:
1. python generates 90% of the menu PlantUML
2. we manually add the last 10% of functionality by hand
3. the two work together seamlessly!!!

Instead of implementing all of the menu wiring by hand (easy but time consuming), we can define our desired menu layout with just a bit of text:
```
MENU
    DISARM_SYSTEM
        **EDIT**
    ARM_SYSTEM
        **EDIT**
    CONFIG
        CHANGE_CODE
            **EDIT**
        ARMING_DELAY
            **EDIT**
        ALARM_DELAY
            **EDIT**
    DATA
        UPTIME
        SENSOR
```

Then run [./src/ui/menu-gen.py](./src/ui/menu-gen.py) and it will update [./src/ui/UiSm.plantuml](./src/ui/UiSm.plantuml) with all the necessary states, transitions and behaviors.

That's not so special on its own. But.... what is really cool is that we can then easily customize the generated content by hand **AND** those customizations don't get overwritten by `menu-gen.py`. This is super cool! Being able to seamlessly mix generated and hand written PlantUML is something I've just started exploring.


## Run code generation
From the `ex3` directory run one of the below commands (they are all equivalent) to have StateSmith watch all 3 PlantUML FSM designs.
* `ss.cli run **/*.plantuml --watch`
* `ss.cli run --here --recursive --watch`
* `ss.cli run -hrw`

## Online Hardware Simulation
https://wokwi.com/projects/425266535976055809

If you want to make changes and upload to wokwi, you can try using the `pack-for-wokwi.py` script.

