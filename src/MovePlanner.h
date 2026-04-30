#pragma once
#include "ChessTypes.h"
#include "ChessGame.h"
#include <queue>

class MovePlanner {
public:
    MovePlanner(ChessGame& game, const PhysicalConfig& config);

    bool startMove(Position from, Position to);
    // Call peekNextStep() first to read the current step, then nextStep() to advance.
    // Returns true if more steps remain after advancing.
    bool nextStep();
    bool isMoveDone() const;
    Step peekNextStep() const;

    // Public for testing — converts a chess Position to physical cm coordinates
    void physicalCoords(Position pos, float& x, float& y) const;

private:
    ChessGame&     _game;
    PhysicalConfig _cfg;
    std::queue<Step> _steps;
};
