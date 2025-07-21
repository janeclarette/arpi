#include <Arduino.h>
#include "beacon.h"

#define RELAY_BEACON_PIN 15

void setupBeacon() {
  pinMode(RELAY_BEACON_PIN, OUTPUT);
  digitalWrite(RELAY_BEACON_PIN, HIGH); // OFF
}

void beaconOn() {
  digitalWrite(RELAY_BEACON_PIN, LOW);
  Serial.println("Beacon ON.");
}

void beaconOff() {
  digitalWrite(RELAY_BEACON_PIN, HIGH);
  Serial.println("Beacon OFF.");
}
