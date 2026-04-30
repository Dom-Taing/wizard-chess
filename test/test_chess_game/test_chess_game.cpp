#include <unity.h>
#include "../../src/ChessGame.h"

void setUp()    {}
void tearDown() {}

// ── Standard init ──────────────────────────────────────────────
void test_standard_white_pawn_row2() {
    ChessGame g;
    Piece p = g.getPieceAt({'E', 2});
    TEST_ASSERT_EQUAL(PAWN,  p.type);
    TEST_ASSERT_EQUAL(WHITE, p.color);
}

void test_standard_black_pawn_row7() {
    ChessGame g;
    Piece p = g.getPieceAt({'D', 7});
    TEST_ASSERT_EQUAL(PAWN,  p.type);
    TEST_ASSERT_EQUAL(BLACK, p.color);
}

void test_standard_white_king_e1() {
    ChessGame g;
    Piece p = g.getPieceAt({'E', 1});
    TEST_ASSERT_EQUAL(KING,  p.type);
    TEST_ASSERT_EQUAL(WHITE, p.color);
}

void test_standard_empty_e4() {
    ChessGame g;
    TEST_ASSERT_TRUE(g.isEmpty({'E', 4}));
}

// ── Custom init ────────────────────────────────────────────────
void test_custom_board() {
    Piece board[8][8] = {};
    board[0][4] = {KING, WHITE};  // E1
    board[7][4] = {KING, BLACK};  // E8
    ChessGame g(board);
    TEST_ASSERT_EQUAL(KING, g.getPieceAt({'E', 1}).type);
    TEST_ASSERT_EQUAL(KING, g.getPieceAt({'E', 8}).type);
    TEST_ASSERT_TRUE(g.isEmpty({'E', 4}));
}

// ── Reset ──────────────────────────────────────────────────────
void test_reset_restores_standard() {
    Piece board[8][8] = {};
    board[0][4] = {KING, WHITE};
    ChessGame g(board);
    g.reset();
    Piece p = g.getPieceAt({'A', 1});
    TEST_ASSERT_EQUAL(ROOK,  p.type);
    TEST_ASSERT_EQUAL(WHITE, p.color);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_standard_white_pawn_row2);
    RUN_TEST(test_standard_black_pawn_row7);
    RUN_TEST(test_standard_white_king_e1);
    RUN_TEST(test_standard_empty_e4);
    RUN_TEST(test_custom_board);
    RUN_TEST(test_reset_restores_standard);
    return UNITY_END();
}
