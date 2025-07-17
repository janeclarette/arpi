#pragma once
#include <Arduino.h>

class UltrasonicSensor {
public:
    UltrasonicSensor(uint8_t trigPin, uint8_t echoPin, 
                     unsigned long timeoutUs = 30000);
    
    void begin();
    float getDistance();              // Get distance in cm
    bool isValidReading(float distance); // Check if reading is within valid range
    
    // Multiple reading functions for better accuracy
    float getAverageDistance(int samples = 5, int delayMs = 100);
    void getDistanceStats(int samples, float& avgDistance, float& minDistance, 
                         float& maxDistance, int& validReadings);

private:
    uint8_t _trigPin, _echoPin;
    unsigned long _timeoutUs;
    
    static const float MIN_DISTANCE;  // Minimum valid distance (cm)
    static const float MAX_DISTANCE;  // Maximum valid distance (cm)
    static const float SOUND_SPEED;   // Speed of sound factor
};
