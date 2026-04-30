#include "MovePlanner.h"

MovePlanner::MovePlanner(ChessGame& game, const PhysicalConfig& config)
    : _game(game), _cfg(config) {}

void MovePlanner::physicalCoords(Position pos, float& x, float& y) const {
    x = _cfg.originX + (pos.col - 'A') * _cfg.stepX;
    y = _cfg.originY + (pos.row - 1)  * _cfg.stepY;
}

bool MovePlanner::startMove(Position from, Position to) {
    if (!_game.isLegalMove(from, to)) return false;
    _steps = std::queue<Step>();  // flush any leftover steps from a prior move
    // Full path planning implemented in Tasks 8–10
    return true;
}

bool MovePlanner::nextStep() {
    if (_steps.empty()) return false;
    _steps.pop();
    return !_steps.empty();
}

bool MovePlanner::isMoveDone() const {
    return _steps.empty();
}

Step MovePlanner::peekNextStep() const {
    if (_steps.empty()) return {};
    return _steps.front();
}
