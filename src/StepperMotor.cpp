#include "StepperMotor.h"

const uint8_t StepperMotor::_seq[8][4] = {
    {1,0,0,1}, {1,0,0,0}, {1,1,0,0}, {0,1,0,0},
    {0,1,1,0}, {0,0,1,0}, {0,0,1,1}, {0,0,0,1}
};

StepperMotor::StepperMotor(uint8_t in1, uint8_t in2,
                           uint8_t in3, uint8_t in4, int delayUs) :
    _pins{in1,in2,in3,in4}, _delayUs(delayUs) {}

void StepperMotor::begin() {
    for (uint8_t p : _pins) pinMode(p, OUTPUT);
    setCoils(0,0,0,0);
}

void StepperMotor::rotate(int steps, bool cw) {
    for (int s=0; s<steps; ++s) {
        for (int i=0; i<8; ++i) {
            int idx = cw ? i : (7-i);
            setCoils(_seq[idx][0],_seq[idx][1],_seq[idx][2],_seq[idx][3]);
            delayMicroseconds(_delayUs);
        }
    }
    setCoils(0,0,0,0);
}

void StepperMotor::setCoils(uint8_t a,uint8_t b,uint8_t c,uint8_t d){
    for (int i=0;i<4;++i) digitalWrite(_pins[i], (i==0?a:i==1?b:i==2?c:d));
}
