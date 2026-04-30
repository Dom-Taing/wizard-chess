#include <unity.h>
#include "../../src/MovePlanner.h"

void setUp()    {}
void tearDown() {}

// ── Coordinate mapping ─────────────────────────────────────────

void test_coord_a1() {
    ChessGame g;
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    float x, y;
    mp.physicalCoords({'A', 1}, x, y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.8f, x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.5f, y);
}

void test_coord_h8() {
    ChessGame g;
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    float x, y;
    mp.physicalCoords({'H', 8}, x, y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.8f + 7*5.0f, x);  // 38.8
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.5f + 7*5.0f, y);  // 40.5
}

void test_coord_e4() {
    ChessGame g;
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    float x, y;
    mp.physicalCoords({'E', 4}, x, y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.8f + 4*5.0f, x);  // 23.8
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.5f + 3*5.0f, y);  // 20.5
}

// ── Step queue state ───────────────────────────────────────────

void test_is_move_done_initially() {
    ChessGame g;
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    TEST_ASSERT_TRUE(mp.isMoveDone());
}

// ── Simple move step sequence ──────────────────────────────────

void test_simple_move_e2e4_step_count() {
    ChessGame g;
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    TEST_ASSERT_TRUE(mp.startMove({'E', 2}, {'E', 4}));
    // E2->E4: dc=0, dr=2 → MAGNET_ON + MOVE_TO(vertical) + MAGNET_OFF = 3 steps
    int count = 0;
    while (!mp.isMoveDone()) { mp.nextStep(); count++; }
    TEST_ASSERT_EQUAL(3, count);
}

void test_simple_move_sequence_e2e4() {
    ChessGame g;
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    mp.startMove({'E', 2}, {'E', 4});

    float expX, expY;
    float srcX = 3.8f + 4*5.0f;  // E column
    float srcY = 5.5f + 1*5.0f;  // rank 2
    float dstX = 3.8f + 4*5.0f;  // E column
    float dstY = 5.5f + 3*5.0f;  // rank 4

    // Step 1: MAGNET_ON at E2 — x/y must be source physical coords
    Step s = mp.peekNextStep();
    TEST_ASSERT_EQUAL(MAGNET_ON, s.type);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, srcX, s.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, srcY, s.y);
    mp.nextStep();

    // Step 2: MOVE_TO E4 (vertical only — dc==0, same column)
    s = mp.peekNextStep();
    TEST_ASSERT_EQUAL(MOVE_TO, s.type);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, dstX, s.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, dstY, s.y);
    mp.nextStep();

    // Step 3: MAGNET_OFF at E4 — x/y must be destination physical coords
    s = mp.peekNextStep();
    TEST_ASSERT_EQUAL(MAGNET_OFF, s.type);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, dstX, s.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, dstY, s.y);
    mp.nextStep();

    TEST_ASSERT_TRUE(mp.isMoveDone());
}

void test_simple_move_diagonal_two_legs() {
    // Use a rook on A1 moving to A5 (purely vertical, 1 leg)
    // then test a bishop move for two legs: C1 -> F4
    Piece board[8][8] = {};
    board[0][2] = {BISHOP, WHITE};  // C1
    ChessGame g(board);
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    TEST_ASSERT_TRUE(mp.startMove({'C', 1}, {'F', 4}));
    // dc=3, dr=3 → MAGNET_ON + MOVE_TO(horiz) + MOVE_TO(vert) + MAGNET_OFF = 4 steps
    int count = 0;
    while (!mp.isMoveDone()) { mp.nextStep(); count++; }
    TEST_ASSERT_EQUAL(4, count);
}

void test_simple_move_updates_game_state() {
    ChessGame g;
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    mp.startMove({'E', 2}, {'E', 4});
    // drain all steps
    while (!mp.isMoveDone()) mp.nextStep();
    // game board must reflect the move and turn must have flipped
    TEST_ASSERT_TRUE(g.isEmpty({'E', 2}));
    Piece p = g.getPieceAt({'E', 4});
    TEST_ASSERT_EQUAL(PAWN, p.type);
    TEST_ASSERT_EQUAL(WHITE, p.color);
    TEST_ASSERT_EQUAL(BLACK, g.getTurn());
}

void test_start_move_illegal_returns_false() {
    ChessGame g;
    PhysicalConfig cfg = {3.8f, 5.5f, 5.0f, 5.0f};
    MovePlanner mp(g, cfg);
    TEST_ASSERT_FALSE(mp.startMove({'E', 2}, {'E', 5}));  // pawn can't jump 3
    TEST_ASSERT_TRUE(mp.isMoveDone());                    // queue stays empty
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_coord_a1);
    RUN_TEST(test_coord_h8);
    RUN_TEST(test_coord_e4);
    RUN_TEST(test_is_move_done_initially);
    RUN_TEST(test_simple_move_e2e4_step_count);
    RUN_TEST(test_simple_move_sequence_e2e4);
    RUN_TEST(test_simple_move_diagonal_two_legs);
    RUN_TEST(test_simple_move_updates_game_state);
    RUN_TEST(test_start_move_illegal_returns_false);
    return UNITY_END();
}
