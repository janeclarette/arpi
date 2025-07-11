#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>

class RFIDReader {
public:
    RFIDReader(uint8_t ssPin, uint8_t rstPin);
    void begin();
    bool poll();          // true on fresh card
    String uid() const;   // last UID as hex

private:
    MFRC522 _rfid;
    String  _uid;
};
