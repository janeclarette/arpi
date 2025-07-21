#pragma once
#include <stdint.h>

class LEDHandler {
public:
    static const int NUM_LEDS = 28;
    static const int LED_PIN = 21;

    void setup();   // Initialize LED strip
    void run();     // Run LED animation
    void reset();   // Clear LED strip

private:
    void generateRandomColors();     // Populate ledColors with random colors
    void chooseSpeedCategory();      // Set stepDelay to random speed

    uint32_t ledColors[NUM_LEDS];    // Buffer for animation colors
    unsigned long stepDelay = 100;   // Delay between LED steps
};
