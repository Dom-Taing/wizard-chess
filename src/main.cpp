#include <Arduino.h>
#include "StepperMotor.h"
#include "CoreXY.h"
#include "Electromagnet.h"
#include "ChessGame.h"
#include "MovePlanner.h"

StepperMotor motorA(2, 3, 4);
StepperMotor motorB(18, 19, 20);
CoreXY xy(motorA, motorB);
Electromagnet magnet(6);

// A1 = (3.8, 5.5); each letter = +5.0 cm X, each number = +5.0 cm Y
static constexpr float SQUARE_ORIGIN_X = 2.5f;
static constexpr float SQUARE_ORIGIN_Y = 2.5f;
static constexpr float SQUARE_STEP_X   = 5.0f;
static constexpr float SQUARE_STEP_Y   = 5.0f;

ChessGame   game;
MovePlanner planner(game, {SQUARE_ORIGIN_X, SQUARE_ORIGIN_Y, SQUARE_STEP_X, SQUARE_STEP_Y});

static bool parseSquare(const String& s, Position& out) {
    if (s.length() < 2) return false;
    char col = s.charAt(0);
    int  row = s.substring(1).toInt();
    if (col < 'A' || col > 'H' || row < 1 || row > 8) return false;
    out = {col, row};
    return true;
}

static const char* turnStr(PieceColor c) {
    return c == WHITE ? "WHITE" : c == BLACK ? "BLACK" : "?";
}

static const char* pieceTypeStr(PieceType t) {
    switch (t) {
        case PAWN:   return "Pawn";
        case ROOK:   return "Rook";
        case KNIGHT: return "Knight";
        case BISHOP: return "Bishop";
        case QUEEN:  return "Queen";
        case KING:   return "King";
        default:     return "(empty)";
    }
}

static void printSquare(const Position& p) {
    if (p == kNoSquare) Serial.print("off-board");
    else                Serial.printf("%c%d", p.col, p.row);
}

static void executeMove(Position from, Position to) {
    Piece moving   = game.getPieceAt(from);
    Piece captured = game.getPieceAt(to);

    if (!planner.startMove(from, to)) {
        Serial.println("  Illegal move (or planner rejected it)");
        return;
    }

    Serial.printf("  Moving %s %s: %c%d -> %c%d",
                  turnStr(moving.color), pieceTypeStr(moving.type),
                  from.col, from.row, to.col, to.row);
    if (captured.type != NONE) {
        Serial.printf("  (captures %s %s)",
                      turnStr(captured.color), pieceTypeStr(captured.type));
    }
    Serial.println();

    while (!planner.isMoveDone()) {
        Step s = planner.peekNextStep();
        switch (s.type) {
            case MOVE_TO:
                Serial.print("  MOVE_TO ");
                printSquare(s.target);
                Serial.println();
                xy.moveToCm(s.x, s.y);
                break;
            case MAGNET_ON:
                Serial.print("  MAGNET ON  @ ");
                printSquare(s.target);
                Serial.println();
                magnet.on();
                break;
            case MAGNET_OFF:
                Serial.print("  MAGNET OFF @ ");
                printSquare(s.target);
                Serial.println();
                delay(150);
                magnet.off();
                break;
        }
        planner.nextStep();
    }

    Serial.printf("  Move done. Next turn: %s\n", turnStr(game.getTurn()));
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("[ChessBot] boot");

    xy.begin();
    xy.setSpeed(1500);
    xy.setHome();
    xy.setMaxBounds(16400, 16400);
    magnet.begin();

    Serial.println("[ChessBot] Ready. Enter a move like 'E2 E4' (or 'E2-E4').");
    Serial.printf("  Turn: %s\n", turnStr(game.getTurn()));
}

void loop() {
    if (!Serial.available()) return;

    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();
    if (input.length() == 0) return;

    if (input == "RESET") {
        game.reset();
        Serial.println("  Game reset.");
        Serial.printf("  Turn: %s\n", turnStr(game.getTurn()));
        return;
    }

    // Accept "E2 E4", "E2-E4", "E2,E4"
    input.replace('-', ' ');
    input.replace(',', ' ');
    int sp = input.indexOf(' ');
    if (sp < 0) {
        Serial.println("  Invalid. Format: <from> <to>  e.g. E2 E4");
        return;
    }

    String fromStr = input.substring(0, sp);  fromStr.trim();
    String toStr   = input.substring(sp + 1); toStr.trim();

    Position from{}, to{};
    if (!parseSquare(fromStr, from) || !parseSquare(toStr, to)) {
        Serial.println("  Bad square. Use A1..H8.");
        return;
    }

    Serial.printf("  %c%d -> %c%d\n", from.col, from.row, to.col, to.row);
    executeMove(from, to);
}
