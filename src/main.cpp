#include <Arduino.h>
#include "StepperMotor.h"
#include "CoreXY.h"

StepperMotor motorA(D4, D5, D6);
StepperMotor motorB(D9, D8, D7);
CoreXY xy(motorA, motorB);

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("[CoreXY] boot");

    xy.begin();
    xy.setSpeed(1500);
    xy.setHome();
    xy.setMaxBounds(16450, 16450);

    Serial.println("[CoreXY] running square demo");
    xy.moveTo(16450, 0);
    Serial.printf("  at (%ld, %ld)\n", xy.getX(), xy.getY());
    xy.moveTo(16450, 16450);
    Serial.printf("  at (%ld, %ld)\n", xy.getX(), xy.getY());
    xy.moveTo(0, 16450);
    Serial.printf("  at (%ld, %ld)\n", xy.getX(), xy.getY());
    xy.moveTo(0, 0);
    Serial.printf("  at (%ld, %ld)\n", xy.getX(), xy.getY());

    Serial.println("[CoreXY] bounds test: moveTo(20000, 0) should be refused");
    xy.moveTo(20000, 0);
    Serial.printf("  after refused move, at (%ld, %ld)\n", xy.getX(), xy.getY());

    Serial.println("[CoreXY] demo complete");
    xy.disable();
}

void loop() {}
