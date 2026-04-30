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

// ── Pawn moves ─────────────────────────────────────────────────
void test_pawn_forward_one() {
    ChessGame g;
    TEST_ASSERT_TRUE(g.isLegalMove({'E', 2}, {'E', 3}));
}

void test_pawn_forward_two_from_start() {
    ChessGame g;
    TEST_ASSERT_TRUE(g.isLegalMove({'E', 2}, {'E', 4}));
}

void test_pawn_forward_two_blocked_by_piece_at_e3() {
    Piece board[8][8] = {};
    board[1][4] = {PAWN, WHITE};   // E2
    board[2][4] = {PAWN, BLACK};   // E3 — blocker
    ChessGame g(board);
    TEST_ASSERT_FALSE(g.isLegalMove({'E', 2}, {'E', 4}));
}

void test_pawn_cannot_go_backward() {
    Piece board[8][8] = {};
    board[2][4] = {PAWN, WHITE};   // E3
    ChessGame g(board);
    TEST_ASSERT_FALSE(g.isLegalMove({'E', 3}, {'E', 2}));
}

void test_pawn_diagonal_capture_legal() {
    Piece board[8][8] = {};
    board[3][4] = {PAWN, WHITE};   // E4
    board[4][3] = {PAWN, BLACK};   // D5 — enemy
    ChessGame g(board);
    TEST_ASSERT_TRUE(g.isLegalMove({'E', 4}, {'D', 5}));
}

void test_pawn_diagonal_no_capture_illegal() {
    ChessGame g;   // D3 is empty
    TEST_ASSERT_FALSE(g.isLegalMove({'E', 2}, {'D', 3}));
}

void test_pawn_cannot_capture_forward() {
    Piece board[8][8] = {};
    board[1][4] = {PAWN, WHITE};   // E2
    board[2][4] = {PAWN, BLACK};   // E3 — blocked
    ChessGame g(board);
    TEST_ASSERT_FALSE(g.isLegalMove({'E', 2}, {'E', 3}));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_standard_white_pawn_row2);
    RUN_TEST(test_standard_black_pawn_row7);
    RUN_TEST(test_standard_white_king_e1);
    RUN_TEST(test_standard_empty_e4);
    RUN_TEST(test_custom_board);
    RUN_TEST(test_reset_restores_standard);
    RUN_TEST(test_pawn_forward_one);
    RUN_TEST(test_pawn_forward_two_from_start);
    RUN_TEST(test_pawn_forward_two_blocked_by_piece_at_e3);
    RUN_TEST(test_pawn_cannot_go_backward);
    RUN_TEST(test_pawn_diagonal_capture_legal);
    RUN_TEST(test_pawn_diagonal_no_capture_illegal);
    RUN_TEST(test_pawn_cannot_capture_forward);
    return UNITY_END();
}
