#include <Arduino.h>
#include "StepperMotor.h"
#include "CoreXY.h"
#include "Electromagnet.h"

StepperMotor motorA(D4, D5, D6);
StepperMotor motorB(D9, D8, D7);
CoreXY xy(motorA, motorB);
Electromagnet magnet(D2);

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("[CoreXY] boot");

    xy.begin();
    xy.setSpeed(1500);
    xy.setHome();
    xy.setMaxBounds(16400, 16400);
    magnet.begin();

    Serial.println("[CoreXY] Ready. Enter a square (e.g. A1, H8), ON, or OFF");
}

// A1 = (3.8, 5.0); each letter = +2.0 cm X, each number = +5.0 cm Y
static constexpr float SQUARE_ORIGIN_X = 3.8f;
static constexpr float SQUARE_ORIGIN_Y = 5.5f;
static constexpr float SQUARE_STEP_X   = 5.0f;
static constexpr float SQUARE_STEP_Y   = 5.0f;

void loop() {
    if (!Serial.available()) return;

    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();

    if (input == "ON") {
        magnet.on();
        Serial.println("  Electromagnet ON");
        return;
    }
    if (input == "OFF") {
        magnet.off();
        Serial.println("  Electromagnet OFF");
        return;
    }

    if (input.length() < 2) {
        Serial.println("  Invalid. Use a square like A1 or H8, or ON/OFF");
        return;
    }

    char col = input.charAt(0);
    int  row = input.substring(1).toInt();

    if (col < 'A' || col > 'H' || row < 1 || row > 8) {
        Serial.println("  Out of range. Columns A-H, rows 1-8");
        return;
    }

    float x = SQUARE_ORIGIN_X + (col - 'A') * SQUARE_STEP_X;
    float y = SQUARE_ORIGIN_Y + (row - 1)   * SQUARE_STEP_Y;

    Serial.printf("  %c%d -> (%.2f cm, %.2f cm)\n", col, row, x, y);
    xy.moveToCm(x, y);
    Serial.printf("  arrived at (%.2f cm, %.2f cm)\n",
                  xy.getX() / CoreXY::STEPS_PER_CM,
                  xy.getY() / CoreXY::STEPS_PER_CM);
}
