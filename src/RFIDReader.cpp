#include "RFIDReader.h"

RFIDReader::RFIDReader(uint8_t ssPin, uint8_t rstPin) : _rfid(ssPin, rstPin) {}

void RFIDReader::begin() {
    SPI.begin();
    _rfid.PCD_Init();
}

bool RFIDReader::poll() {
    if (!_rfid.PICC_IsNewCardPresent() || !_rfid.PICC_ReadCardSerial())
        return false;

    _uid.reserve(12);
    _uid = "";
    for (byte i=0;i<_rfid.uid.size;i++) {
        if (_rfid.uid.uidByte[i] < 0x10) _uid += "0";
        _uid += String(_rfid.uid.uidByte[i], HEX);
    }
    _uid.toUpperCase();

    _rfid.PICC_HaltA();
    return true;
}

String RFIDReader::uid() const { return _uid; }
