#pragma once
#include <cstdint>

enum PieceType  : uint8_t { NONE, PAWN, ROOK, KNIGHT, BISHOP, QUEEN, KING };
enum PieceColor : uint8_t { NO_COLOR, WHITE, BLACK };

struct Piece {
    PieceType  type  = NONE;
    PieceColor color = NO_COLOR;
};

struct Position {
    char col = 'A';  // 'A'–'H'
    int  row = 1;    // 1–8
    bool operator==(const Position& o) const { return col == o.col && row == o.row; }
    bool operator!=(const Position& o) const { return !(*this == o); }
};

struct PhysicalConfig {
    float originX = 3.8f;   // physical X of A-file in cm
    float originY = 5.5f;   // physical Y of rank 1 in cm
    float stepX   = 5.0f;   // cm per file
    float stepY   = 5.0f;   // cm per rank
};

enum StepType : uint8_t { MOVE_TO, MAGNET_ON, MAGNET_OFF };

struct Step {
    StepType type   = MOVE_TO;
    Position target = {};    // chess square (meaningful for on-board MOVE_TO)
    float    x      = 0.0f; // physical X in cm (authoritative for all MOVE_TO)
    float    y      = 0.0f; // physical Y in cm (authoritative for all MOVE_TO)
    // x and y are 0 for MAGNET_ON / MAGNET_OFF steps
};
