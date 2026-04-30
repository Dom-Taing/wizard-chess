#include "MovePlanner.h"

MovePlanner::MovePlanner(ChessGame& game, const PhysicalConfig& config)
    : _game(game), _cfg(config) {}

void MovePlanner::physicalCoords(Position pos, float& x, float& y) const {
    x = _cfg.originX + (pos.col - 'A') * _cfg.stepX;
    y = _cfg.originY + (pos.row - 1)  * _cfg.stepY;
}

bool MovePlanner::startMove(Position from, Position to) {
    if (!_game.isLegalMove(from, to)) return false;
    _steps = std::queue<Step>();

    float fx, fy, tx, ty;
    physicalCoords(from, fx, fy);
    physicalCoords(to,   tx, ty);

    int dc = (int)(to.col - from.col);
    int dr = (int)(to.row - from.row);

    // MAGNET_ON at source
    _steps.push({MAGNET_ON, from, fx, fy});

    // Horizontal leg (if needed)
    if (dc != 0) _steps.push({MOVE_TO, from, tx, fy});

    // Vertical leg (if needed)
    if (dr != 0) _steps.push({MOVE_TO, to, tx, ty});

    // MAGNET_OFF at destination
    _steps.push({MAGNET_OFF, to, tx, ty});

    _game.applyMove(from, to);
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
