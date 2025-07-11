#pragma once
#include <Arduino.h>

class CoinAcceptor {
public:
    CoinAcceptor(uint8_t coinPin,
                 uint8_t relayPin,
                 unsigned long ignoreMs  = 1000,
                 unsigned long debounceMs = 1000);

    void begin();
    bool poll();                // true once per new ₱5 pulse
    void enable();              // turn coin mech ON
    void disable();             // turn coin mech OFF
    void reset();               // forget pulse history

private:
    static void IRAM_ATTR isr();          // interrupt trampoline
    void isrBody();                       // real handler

    uint8_t _pinCoin, _pinRelay;
    unsigned long _ignore, _debounce;
    volatile unsigned long _lastPulse = 0;
    volatile bool _coinSeen          = false;
    unsigned long _bootTime          = 0;

    static CoinAcceptor* _instance;      // singleton pointer for ISR
};
