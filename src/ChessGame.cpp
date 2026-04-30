#include "ChessGame.h"
#include <cstring>
#include <type_traits>

static_assert(std::is_trivially_copyable<Piece>::value, "Piece must be trivially copyable");

ChessGame::ChessGame() { initStandard(); }

ChessGame::ChessGame(const Piece board[8][8]) : _turn(WHITE) {
    memcpy(_board, board, sizeof(_board));
}

void ChessGame::reset() {
    _turn = WHITE;
    _captured.clear();
    initStandard();
}

void ChessGame::initStandard() {
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            _board[r][c] = {NONE, NO_COLOR};

    PieceType backRank[] = {ROOK, KNIGHT, BISHOP, QUEEN, KING, BISHOP, KNIGHT, ROOK};
    for (int c = 0; c < 8; c++) {
        _board[0][c] = {backRank[c], WHITE};
        _board[1][c] = {PAWN, WHITE};
        _board[6][c] = {PAWN, BLACK};
        _board[7][c] = {backRank[c], BLACK};
    }
}

PieceColor ChessGame::getTurn() const { return _turn; }

Piece ChessGame::getPieceAt(Position p) const {
    if (ci(p) < 0 || ci(p) > 7 || ri(p) < 0 || ri(p) > 7) return {NONE, NO_COLOR};
    return at(p);
}
bool ChessGame::isEmpty(Position p) const {
    if (ci(p) < 0 || ci(p) > 7 || ri(p) < 0 || ri(p) > 7) return true;
    return at(p).type == NONE;
}
const std::vector<Piece>& ChessGame::getCaptured() const { return _captured; }

// Stubs — will be filled in Tasks 3–6
bool ChessGame::isLegalMove(Position, Position) const { return false; }
bool ChessGame::applyMove(Position, Position)         { return false; }
bool ChessGame::isPawnMove  (Position, Position) const { return false; }
bool ChessGame::isRookMove  (Position, Position) const { return false; }
bool ChessGame::isKnightMove(Position, Position) const { return false; }
bool ChessGame::isBishopMove(Position, Position) const { return false; }
bool ChessGame::isQueenMove (Position, Position) const { return false; }
bool ChessGame::isKingMove  (Position, Position) const { return false; }
bool ChessGame::isPathClear (Position, Position) const { return false; }
