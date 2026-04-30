#include "MovePlanner.h"

MovePlanner::MovePlanner(ChessGame& game, const PhysicalConfig& config)
    : _game(game), _cfg(config) {}

void MovePlanner::physicalCoords(Position pos, float& x, float& y) const {
    x = _cfg.originX + (pos.col - 'A') * _cfg.stepX;
    y = _cfg.originY + (pos.row - 1)  * _cfg.stepY;
}

bool MovePlanner::startMove(Position from, Position to) {
    if (!_game.isLegalMove(from, to)) return false;
    if (!_steps.empty()) return false;  // reject if a move is already in progress

    float fx, fy, tx, ty;
    physicalCoords(from, fx, fy);
    physicalCoords(to,   tx, ty);

    int dc = (int)(to.col - from.col);
    int dr = (int)(to.row - from.row);

    // x/y are 0 for MAGNET steps per Step struct contract; position is in target field
    _steps.push({MAGNET_ON,  from, 0.0f, 0.0f});

    // Horizontal leg (if needed)
    if (dc != 0) _steps.push({MOVE_TO, from, tx, fy});

    // Vertical leg (if needed)
    if (dr != 0) _steps.push({MOVE_TO, to, tx, ty});

    _steps.push({MAGNET_OFF, to, 0.0f, 0.0f});

    bool ok = _game.applyMove(from, to);
    (void)ok;  // guaranteed by isLegalMove check above
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
