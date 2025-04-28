In `ex2`, we add the following:
- audio system
- 4 button FSM instances
- some button helper classes
- sending events from buttons to simple UI FSM
- sending commands from UI to Control system FSM

## Run code generation
From the `ex2` directory run one of the below commands (they are all equivalent) to have StateSmith watch all 3 PlantUML FSM designs.
* `ss.cli run **/*.plantuml --watch`
* `ss.cli run --here --recursive --watch`
* `ss.cli run -hrw`

## Online Hardware Simulation
https://wokwi.com/projects/428919177893749761

If you want to make changes and upload to wokwi, you can try using the `pack-for-wokwi.py` script.
