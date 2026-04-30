#include "MovePlanner.h"
#include <vector>
#include <algorithm>

MovePlanner::MovePlanner(ChessGame& game, const PhysicalConfig& config)
    : _game(game), _cfg(config) {
    initBorderSlots();
}

void MovePlanner::initBorderSlots() {
    _borderSlots.clear();
    _borderOccupied.clear();
    // 8 slots above rank 8
    float yAbove = _cfg.originY + 8 * _cfg.stepY;
    for (int c = 0; c < 8; c++) {
        float x = _cfg.originX + c * _cfg.stepX;
        _borderSlots.push_back({x, yAbove});
        _borderOccupied.push_back(false);
    }
    // 8 slots below rank 1
    float yBelow = _cfg.originY - _cfg.stepY;
    for (int c = 0; c < 8; c++) {
        float x = _cfg.originX + c * _cfg.stepX;
        _borderSlots.push_back({x, yBelow});
        _borderOccupied.push_back(false);
    }
}

int MovePlanner::nextFreeBorderSlot() const {
    for (int i = 0; i < (int)_borderSlots.size(); i++) {
        if (!_borderOccupied[i]) return i;
    }
    return -1;
}

void MovePlanner::physicalCoords(Position pos, float& x, float& y) const {
    x = _cfg.originX + (pos.col - 'A') * _cfg.stepX;
    y = _cfg.originY + (pos.row - 1)  * _cfg.stepY;
}

bool MovePlanner::findParkSquare(Position blocker, Position mainFrom, Position mainTo, Position& park) const {
    // Candidate directions: up, down, left, right
    int drs[] = {1, -1, 0, 0};
    int dcs[] = {0, 0, -1, 1};
    for (int i = 0; i < 4; i++) {
        Position candidate = {(char)(blocker.col + dcs[i]), blocker.row + drs[i]};
        if (candidate.col < 'A' || candidate.col > 'H') continue;
        if (candidate.row < 1  || candidate.row > 8)   continue;
        if (!_game.isEmpty(candidate))                  continue;
        bool onHorizLeg = (candidate.row == mainFrom.row &&
                           candidate.col > std::min(mainFrom.col, mainTo.col) &&
                           candidate.col < std::max(mainFrom.col, mainTo.col));
        bool onVertLeg  = (candidate.col == mainTo.col &&
                           candidate.row > std::min(mainFrom.row, mainTo.row) &&
                           candidate.row < std::max(mainFrom.row, mainTo.row));
        // Also exclude the L-corner pivot square (intersection of both legs)
        bool isPivot = (candidate.col == mainTo.col && candidate.row == mainFrom.row);
        if (onHorizLeg || onVertLeg || isPivot) continue;
        park = candidate;
        return true;
    }
    return false;
}

bool MovePlanner::startMove(Position from, Position to) {
    if (!_game.isLegalMove(from, to)) return false;
    if (!_steps.empty()) return false;

    float fx, fy, tx, ty;
    physicalCoords(from, fx, fy);
    physicalCoords(to,   tx, ty);

    int dc = (int)(to.col - from.col);
    int dr = (int)(to.row - from.row);
    int stepC = (dc == 0) ? 0 : (dc > 0 ? 1 : -1);
    int stepR = (dr == 0) ? 0 : (dr > 0 ? 1 : -1);

    // Handle capture: park the enemy piece at a border slot before anything else
    bool isCapture = !_game.isEmpty(to);
    int borderIdx = -1;
    if (isCapture) {
        borderIdx = nextFreeBorderSlot();
        if (borderIdx < 0) {
            return false;  // no border slots free (shouldn't happen in a normal game)
        }
        auto [bx, by] = _borderSlots[borderIdx];
        _steps.push({MAGNET_ON,  to,       0.0f, 0.0f});
        _steps.push({MOVE_TO,    to,       bx,   by});
        _steps.push({MAGNET_OFF, {'Z', 0}, 0.0f, 0.0f});
    }

    // Collect blockers with their park squares
    std::vector<std::pair<Position, Position>> displacements;

    // Horizontal leg blockers
    if (dc != 0) {
        for (int c = from.col + stepC; c != to.col; c += stepC) {
            Position sq = {(char)c, from.row};
            if (!_game.isEmpty(sq)) {
                Position park;
                if (!findParkSquare(sq, from, to, park)) {
                    _steps = std::queue<Step>();
                    return false;  // no room to park blocker — cannot plan move
                }
                displacements.push_back({sq, park});
            }
        }
    }

    // Vertical leg blockers
    if (dr != 0) {
        for (int r = from.row + stepR; r != to.row; r += stepR) {
            Position sq = {to.col, r};
            if (!_game.isEmpty(sq)) {
                Position park;
                if (!findParkSquare(sq, from, to, park)) {
                    _steps = std::queue<Step>();
                    return false;  // no room to park blocker — cannot plan move
                }
                displacements.push_back({sq, park});
            }
        }
    }

    // Push park sub-sequences
    for (auto& [blocker, park] : displacements) {
        float px, py;
        physicalCoords(park, px, py);
        _steps.push({MAGNET_ON,  blocker, 0.0f, 0.0f});
        _steps.push({MOVE_TO,    park,    px,   py});
        _steps.push({MAGNET_OFF, park,    0.0f, 0.0f});
    }

    // Main move
    _steps.push({MAGNET_ON, from, 0.0f, 0.0f});
    if (dc != 0) _steps.push({MOVE_TO, from, tx, fy});
    if (dr != 0) _steps.push({MOVE_TO, to,   tx, ty});
    _steps.push({MAGNET_OFF, to, 0.0f, 0.0f});

    // Restore sub-sequences (reverse order)
    for (int i = (int)displacements.size() - 1; i >= 0; i--) {
        auto& [blocker, park] = displacements[i];
        float bx, by;
        physicalCoords(blocker, bx, by);
        _steps.push({MAGNET_ON,  park,    0.0f, 0.0f});
        _steps.push({MOVE_TO,    blocker, bx,   by});
        _steps.push({MAGNET_OFF, blocker, 0.0f, 0.0f});
    }

    bool ok = _game.applyMove(from, to);
    (void)ok;  // guaranteed by isLegalMove check above
    if (isCapture) _borderOccupied[borderIdx] = true;
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
