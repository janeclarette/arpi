#include <Arduino.h>
#include "solonoid.h"

#define RELAY_SOLONOID_PIN 13

void setupSolonoid() {
  pinMode(RELAY_SOLONOID_PIN, OUTPUT);
  digitalWrite(RELAY_SOLONOID_PIN, HIGH); // OFF
}

void SolonoidOn() {
  digitalWrite(RELAY_SOLONOID_PIN, LOW);
  Serial.println("Solonoid ON.");
}

void SolonoidOff() {
  digitalWrite(RELAY_SOLONOID_PIN, HIGH);
  Serial.println("Solonoid OFF.");
}
