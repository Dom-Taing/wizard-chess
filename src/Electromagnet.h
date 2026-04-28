#pragma once
#include <Arduino.h>

class Electromagnet {
public:
    explicit Electromagnet(int pin) : _pin(pin) {}

    void begin() {
        pinMode(_pin, OUTPUT);
        off();
    }

    void on()  { digitalWrite(_pin, HIGH); _state = true; }
    void off() { digitalWrite(_pin, LOW);  _state = false; }
    bool isOn() const { return _state; }

private:
    int  _pin;
    bool _state = false;
};
