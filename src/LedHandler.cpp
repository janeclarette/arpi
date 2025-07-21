#include "LEDHandler.h"
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

// Static NeoPixel strip object
Adafruit_NeoPixel strip(LEDHandler::NUM_LEDS, LEDHandler::LED_PIN, NEO_GRB + NEO_KHZ800);

void LEDHandler::setup() {
    strip.begin();
    strip.setBrightness(50);
    strip.show();  // Clear strip on startup
    randomSeed(analogRead(0));
}

void LEDHandler::reset() {
    strip.clear();
    strip.show();
}

void LEDHandler::run() {
    Serial.println("🎆 Running LED animation...");
    unsigned long animationStart = millis();

    while (millis() - animationStart < 5000) {
        generateRandomColors();
        chooseSpeedCategory();

        for (int i = 0; i < NUM_LEDS; i += 2) {
            strip.clear();

            if (i < NUM_LEDS) {
                strip.setPixelColor(i, ledColors[i]);
            }
            if (i + 1 < NUM_LEDS) {
                strip.setPixelColor(i + 1, ledColors[i + 1]);
            }

            strip.show();
            delay(stepDelay);
        }

        strip.clear();
        strip.show();
        delay(100); // optional pause between animation cycles
    }

    reset(); // clear LEDs at the end
    Serial.println("✅ LED animation complete.");
}

// Private helper: generate random colors for animation
void LEDHandler::generateRandomColors() {
    for (int i = 0; i < NUM_LEDS; i++) {
        ledColors[i] = strip.Color(random(0, 256), random(0, 256), random(0, 256));
    }
}

// Private helper: choose speed randomly and print the speed tier
void LEDHandler::chooseSpeedCategory() {
    int category = random(0, 3);
    switch (category) {
        case 0:
            stepDelay = random(50, 101);
            Serial.println("Speed: SUPER FAST");
            break;
        case 1:
            stepDelay = random(100, 201);
            Serial.println("Speed: FAST");
            break;
        case 2:
            stepDelay = random(200, 401);
            Serial.println("Speed: MEDIUM FAST");
            break;
    }
}
