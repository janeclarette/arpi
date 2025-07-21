#include <Arduino.h>
#include "neon.h"

// Neon relay pins
#define RELAY_NEON_PIN1 33
#define RELAY_NEON_PIN2 32
#define RELAY_NEON_PIN3 22
#define RELAY_NEON_PIN4 23
#define RELAY_NEON_PIN5 2

const uint8_t neonPins[] = {
  RELAY_NEON_PIN1, RELAY_NEON_PIN2, RELAY_NEON_PIN3,
  RELAY_NEON_PIN4, RELAY_NEON_PIN5
};
const size_t neonCount = sizeof(neonPins) / sizeof(neonPins[0]);

void setupNeon() {
  for (size_t i = 0; i < neonCount; i++) {
    pinMode(neonPins[i], OUTPUT);
    digitalWrite(neonPins[i], HIGH); // OFF
  }
}

void neonAllOn() {
  for (size_t i = 0; i < neonCount; i++) digitalWrite(neonPins[i], LOW);
  Serial.println("All Neon Lights ON.");
}

void neonAllOff() {
  for (size_t i = 0; i < neonCount; i++) digitalWrite(neonPins[i], HIGH);
  Serial.println("All Neon Lights OFF.");
}

void neonPattern1() {
  Serial.println("Neon Pattern 1: Sequential Blink");
  for (size_t i = 0; i < neonCount; i++) {
    digitalWrite(neonPins[i], LOW);
    delay(200);
    digitalWrite(neonPins[i], HIGH);
  }
}

void neonPattern2() {
  Serial.println("Neon Pattern 2: Chase Forward and Back");
  for (size_t i = 0; i < neonCount; i++) {
    digitalWrite(neonPins[i], LOW);
    delay(150);
    digitalWrite(neonPins[i], HIGH);
  }
  for (int i = neonCount - 1; i >= 0; i--) {
    digitalWrite(neonPins[i], LOW);
    delay(150);
    digitalWrite(neonPins[i], HIGH);
  }
}

void neonPattern() {
  Serial.println("Neon Pattern: Complex 5-second display");
  unsigned long startTime = millis();
  
  // Run pattern for at least 5 seconds
  while (millis() - startTime < 5000) {
    // Wave pattern - lights turn on in sequence and stay on
    for (size_t i = 0; i < neonCount; i++) {
      digitalWrite(neonPins[i], LOW);
      delay(100);
    }
    delay(200);
    
    // Turn all off briefly
    for (size_t i = 0; i < neonCount; i++) {
      digitalWrite(neonPins[i], HIGH);
    }
    delay(100);
    
    // Reverse wave - lights turn on from end to start
    for (int i = neonCount - 1; i >= 0; i--) {
      digitalWrite(neonPins[i], LOW);
      delay(100);
    }
    delay(200);
    
    // Blink all together
    for (size_t i = 0; i < neonCount; i++) {
      digitalWrite(neonPins[i], LOW);
    }
    delay(300);
    for (size_t i = 0; i < neonCount; i++) {
      digitalWrite(neonPins[i], HIGH);
    }
    delay(300);
  }
  
  // Ensure all lights are off at the end
  neonAllOff();
  Serial.println("Neon pattern complete");
}

// Compatibility aliases
void turnOnAllNeon() {
  neonAllOn();
}

void turnOffAllNeon() {
  neonAllOff();
}
