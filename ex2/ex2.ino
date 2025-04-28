#include "src/App.hpp"

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
