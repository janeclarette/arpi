#include "CoinAcceptor.h"

CoinAcceptor* CoinAcceptor::_instance = nullptr;

CoinAcceptor::CoinAcceptor(uint8_t coinPin, uint8_t relayPin,
                           unsigned long ignoreMs, unsigned long debounceMs)
    : _pinCoin(coinPin), _pinRelay(relayPin),
      _ignore(ignoreMs), _debounce(debounceMs) {}

void CoinAcceptor::begin() {
    pinMode(_pinCoin,  INPUT_PULLUP);
    pinMode(_pinRelay, OUTPUT);
    enable();

    _bootTime  = millis();
    _instance  = this;
    attachInterrupt(digitalPinToInterrupt(_pinCoin), CoinAcceptor::isr, FALLING);
}

void CoinAcceptor::enable()  { digitalWrite(_pinRelay, LOW);  }
void CoinAcceptor::disable() { digitalWrite(_pinRelay, HIGH); }

void CoinAcceptor::reset()   { _coinSeen = false; }

bool CoinAcceptor::poll() {
    bool tmp = _coinSeen;
    _coinSeen = false;
    return tmp;
}

void IRAM_ATTR CoinAcceptor::isr() { if (_instance) _instance->isrBody(); }

void CoinAcceptor::isrBody() {
    unsigned long now = millis();
    if (now > _bootTime + _ignore && (now - _lastPulse) > _debounce) {
        _lastPulse = now;
        _coinSeen  = true;
    }
}
