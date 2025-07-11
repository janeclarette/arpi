#include "GameController.h"

GameController::GameController(CoinAcceptor& coinAcc, StepperMotor& step, RFIDReader& rfidReader,
                               uint8_t trig, uint8_t echo)
    : coin(coinAcc), stepper(step), rfid(rfidReader),
      trigPin(trig), echoPin(echo) {}

void GameController::begin() {
    coin.begin();
    stepper.begin();
    rfid.begin();

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    resetBallState();

    Serial.println("ESP32 Ready. Insert ₱5 coin to start...");
}

void GameController::update() {
    switch (state) {
        case State::WAIT_COIN:
            if (coin.poll()) {
                Serial.println("₱5 coin inserted — starting game...");
                state = State::RUNNING;
            }
            break;

        case State::RUNNING:
            checkUltrasonic();
            if (ballCount == 3) {
                Serial.println("All balls detected. Returning...");
                stepper.rotate(256, false);  // 180° CCW
                coin.disable();
                state = State::WAIT_RFID;
            }
            break;

        case State::WAIT_RFID:
            if (rfid.poll()) {
                Serial.print("RFID UID: "); Serial.println(rfid.uid());
                Serial.println("Rewards claimed!");
                state = State::IDLE;
            }
            break;

        case State::IDLE:
            // Waiting for RESET_GAME
            break;
    }

    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n'); cmd.trim();
        if (cmd == "RESET_GAME") {
            Serial.println("Restarting ESP32…");
            delay(300);
            ESP.restart();
        }
    }
}

void GameController::resetBallState() {
    ballCount = 0;
    b1 = b2 = b3 = false;
}

void GameController::checkUltrasonic() {
    digitalWrite(trigPin, LOW);  delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration  = pulseIn(echoPin, HIGH);
    float distance = (duration * 0.0343f) / 2.0f;
    Serial.print("DISTANCE: "); Serial.println(distance);

    if (distance <= 20 && distance > 13 && !b1) {
        b1 = true; ++ballCount;
        Serial.println("BALL 1");
        delay(400);
    } else if (distance <= 13 && distance > 5 && b1 && !b2) {
        b2 = true; ++ballCount;
        Serial.println("BALL 2");
        delay(400);
    } else if (distance <= 5 && b2 && !b3) {
        b3 = true; ++ballCount;
        Serial.println("BALL 3");
        delay(400);
    }
}
