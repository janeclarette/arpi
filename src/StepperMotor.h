#pragma once
#include <Arduino.h>

class StepperMotor {
public:
    StepperMotor(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4,
                 int delayUs = 2000);
    void begin();
    void rotate(int steps, bool cw = true);   // cw=true → forward

private:
    void setCoils(uint8_t a, uint8_t b, uint8_t c, uint8_t d);

    uint8_t _pins[4];
    int     _delayUs;
    static const uint8_t _seq[8][4];
};
