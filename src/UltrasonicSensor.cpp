#include "UltrasonicSensor.h"

// Static constants
const float UltrasonicSensor::MIN_DISTANCE = 2.0;    // 2 cm minimum
const float UltrasonicSensor::MAX_DISTANCE = 400.0;  // 400 cm maximum for HC-SR04
const float UltrasonicSensor::SOUND_SPEED = 0.034;   // cm/microsecond

UltrasonicSensor::UltrasonicSensor(uint8_t trigPin, uint8_t echoPin, unsigned long timeoutUs)
    : _trigPin(trigPin), _echoPin(echoPin), _timeoutUs(timeoutUs) {
}

void UltrasonicSensor::begin() {
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
    digitalWrite(_trigPin, LOW);
}

float UltrasonicSensor::getDistance() {
    // Trigger pulse
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);
    
    // Read echo
    long duration = pulseIn(_echoPin, HIGH, _timeoutUs);
    
    if (duration > 0) {
        // Convert to distance in cm
        float distance = (duration * SOUND_SPEED) / 2;
        return distance;
    }
    
    return -1; // Timeout or no echo
}

bool UltrasonicSensor::isValidReading(float distance) {
    return (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE);
}

float UltrasonicSensor::getAverageDistance(int samples, int delayMs) {
    float total = 0;
    int validCount = 0;
    
    for (int i = 0; i < samples; i++) {
        float distance = getDistance();
        if (isValidReading(distance)) {
            total += distance;
            validCount++;
        }
        
        if (i < samples - 1) { // Don't delay after last reading
            delay(delayMs);
        }
    }
    
    return (validCount > 0) ? (total / validCount) : -1;
}

void UltrasonicSensor::getDistanceStats(int samples, float& avgDistance, 
                                       float& minDistance, float& maxDistance, 
                                       int& validReadings) {
    float total = 0;
    validReadings = 0;
    minDistance = MAX_DISTANCE + 1;  // Initialize to impossibly high value
    maxDistance = 0;
    
    for (int i = 0; i < samples; i++) {
        float distance = getDistance();
        
        if (isValidReading(distance)) {
            total += distance;
            validReadings++;
            
            if (distance < minDistance) minDistance = distance;
            if (distance > maxDistance) maxDistance = distance;
        }
        
        if (i < samples - 1) { // Don't delay after last reading
            delay(200);
        }
    }
    
    avgDistance = (validReadings > 0) ? (total / validReadings) : -1;
    
    // If no valid readings, reset min/max
    if (validReadings == 0) {
        minDistance = -1;
        maxDistance = -1;
    }
}
