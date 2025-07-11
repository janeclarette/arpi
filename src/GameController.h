#pragma once
#include "CoinAcceptor.h"
#include "StepperMotor.h"
#include "RFIDReader.h"

class GameController {
public:
    GameController(CoinAcceptor& coin, StepperMotor& stepper, RFIDReader& rfid,
                   uint8_t trigPin, uint8_t echoPin);

    void begin();     // initialize pins and serial messages
    void update();    // call this in loop()

private:
    enum class State { WAIT_COIN, RUNNING, WAIT_RFID, IDLE };
    State state = State::WAIT_COIN;

    CoinAcceptor& coin;
    StepperMotor& stepper;
    RFIDReader&   rfid;

    uint8_t trigPin, echoPin;

    int ballCount = 0;
    bool b1 = false, b2 = false, b3 = false;

    void checkUltrasonic();
    void resetBallState();
};
